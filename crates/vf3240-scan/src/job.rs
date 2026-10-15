//! One scan from an open device to a streamed image file.
//!
//! By default the vendor's per-scan calibration runs first and hardware
//! shading and gamma 1.6 are applied; `raw` skips all three (milestone 1
//! output). Rows are streamed to the output as they arrive, so host memory
//! stays at a few lines regardless of resolution. Compression and writing run
//! on a second thread fed through a bounded channel, so the USB reader never
//! waits on the disk; the scanner's SDRAM only buffers ~170 lines (0.2 s at
//! 300 dpi). TIFF is uncompressed unless `compression` says otherwise; PNG is
//! always Sub-filtered Deflate, as the format requires.
//!
//! The writer thread runs the corrections on 16-bit samples (8-bit scans are
//! widened, gray rows travel as three identical channels): the motor
//! staircase (`stairs`, 2400 and 1200 dpi) or the constant colour
//! registration (`register`), then the chip-seam fill (`seams`), then the
//! rows are narrowed or byte-ordered for the file format.

use std::fs::File;
use std::io::{self, BufWriter};
use std::sync::mpsc;
use std::sync::{Arc, Mutex};
use std::time::Instant;
use vf3240_core::{asic, lamp, motor, power, scan, setup, unpack, Device};
use vf3240_image::{png, ppm, register, seamcal, seams, stairs, tiff};

/// What to scan and how to store it.
#[derive(Debug, Clone, Copy)]
pub struct Job {
    pub mode: setup::Mode,
    /// Output lines; `mode.full_lines` for the whole bed.
    pub lines: u32,
    /// Scan area origin and width in pixels at the mode's dpi, in the image's
    /// own coordinates (column 0 = sensor pixel 0 side); `width` 0 = to the edge.
    pub x_pos: usize,
    pub y_pos: u32,
    pub width: usize,
    /// Skip calibration, shading and gamma.
    pub raw: bool,
    pub compression: tiff::Compression,
    /// Vertical R and B shifts in lines for the constant registration
    /// (`register`); `(0.0, 0.0)` disables it. Also the staircase stage's fallback.
    pub register: (f64, f64),
    /// Motor current level 0–3 for scanning (`ScanSetup::motor_vref`); vendor default 2.
    pub motor_vref: u8,
    /// Correct the motor staircase at 2400/1200 dpi (`stairs`); falls back to `register`.
    pub destair: bool,
    /// Fill the chip-junction gaps (`seams`); widens the image by their sum.
    pub deseam: bool,
    /// Seam gaps and chip stagger of this unit (`seams::SeamTable`).
    pub seams: seams::SeamTable,
    /// Take the chips' vertical stagger out (with the staircase stage).
    pub destagger: bool,
    /// 8-bit gray (`10-gray-mode.md`) instead of 24-bit colour.
    pub gray: bool,
    /// 16-bit samples (`07-readout.md` §7) instead of 8-bit.
    pub depth16: bool,
}

impl Job {
    pub fn full_bed(mode: setup::Mode) -> Self {
        Job {
            mode,
            lines: mode.full_lines,
            x_pos: 0,
            y_pos: 0,
            width: 0,
            raw: false,
            compression: tiff::Compression::None,
            register: register::default_shifts(mode.dpi),
            motor_vref: 2,
            destair: true,
            deseam: true,
            seams: seams::SeamTable::DEFAULT,
            destagger: true,
            gray: false,
            depth16: false,
        }
    }
}

/// One output file, chosen by extension (`.png`, `.tif`/`.tiff`, else PPM).
enum Sink {
    Ppm(ppm::Writer<BufWriter<File>>),
    Png(png::Writer<BufWriter<File>>),
    Tiff(tiff::Writer<BufWriter<File>>),
    /// Rows handed to a callback instead of a file (8-bit samples).
    Callback(RowCallback),
}

/// Receives each finished output row as bytes (channels interleaved, 8-bit).
pub type RowCallback = Box<dyn FnMut(&[u8]) -> io::Result<()> + Send>;

/// How one row of samples reaches a sink: narrowed to bytes, or 16-bit in
/// the format's byte order (TIFF little-endian, PNG and PPM big-endian).
struct RowCodec {
    bits: usize,
    big_endian: bool,
    buf: Vec<u8>,
}

impl RowCodec {
    fn encode(&mut self, samples: &[u16]) -> &[u8] {
        self.buf.clear();
        if self.bits == 8 {
            self.buf.extend(samples.iter().map(|&v| v as u8));
        } else if self.big_endian {
            self.buf
                .extend(samples.iter().flat_map(|v| v.to_be_bytes()));
        } else {
            self.buf
                .extend(samples.iter().flat_map(|v| v.to_le_bytes()));
        }
        &self.buf
    }
}

impl Sink {
    fn create(
        path: &str,
        width: usize,
        rows: usize,
        dpi: u32,
        compression: tiff::Compression,
        channels: usize,
        bits: usize,
    ) -> io::Result<(Sink, RowCodec)> {
        let file = BufWriter::with_capacity(4 << 20, File::create(path)?);
        let lower = path.to_ascii_lowercase();
        let (sink, big_endian) = if lower.ends_with(".png") {
            (
                Sink::Png(png::Writer::new(file, width, rows, dpi, channels, bits)?),
                true,
            )
        } else if lower.ends_with(".tif") || lower.ends_with(".tiff") {
            (
                Sink::Tiff(tiff::Writer::new(
                    file,
                    width,
                    rows,
                    dpi,
                    compression,
                    channels,
                    bits,
                )?),
                false,
            )
        } else {
            (
                Sink::Ppm(ppm::Writer::new(file, width, rows, channels, bits)?),
                true,
            )
        };
        let codec = RowCodec {
            bits,
            big_endian,
            buf: Vec::with_capacity(width * channels * bits / 8),
        };
        Ok((sink, codec))
    }
    fn write_row(&mut self, row: &[u8]) -> io::Result<()> {
        match self {
            Sink::Ppm(w) => w.write_row(row),
            Sink::Png(w) => w.write_row(row),
            Sink::Tiff(w) => w.write_row(row),
            Sink::Callback(f) => f(row),
        }
    }
    fn finish(self) -> io::Result<()> {
        match self {
            Sink::Ppm(w) => w.finish().map(drop),
            Sink::Png(w) => w.finish().map(drop),
            Sink::Tiff(w) => w.finish().map(drop),
            Sink::Callback(_) => Ok(()),
        }
    }
}

/// Rows in flight between the USB reader and the writer thread (10 MB at 2400 dpi, 16-bit samples).
const CHANNEL_ROWS: usize = 64;
/// Lines between progress reports.
const PROGRESS_EVERY: u32 = 1000;

fn mmss(d: std::time::Duration) -> String {
    let s = d.as_secs();
    format!("{}:{:02}", s / 60, s % 60)
}

/// One progress line: lines done, elapsed since the job started, and the
/// remaining/total estimate from the line rate since data became ready.
fn progress(
    done: u32,
    total: u32,
    elapsed: std::time::Duration,
    scanning: std::time::Duration,
) -> String {
    let per_line = scanning.as_secs_f64() / f64::from(done);
    let remaining = std::time::Duration::from_secs_f64(per_line * f64::from(total - done));
    format!(
        "line {done}/{total} {:3.0}%  elapsed {}  remaining {}  total ~{}",
        100.0 * f64::from(done) / f64::from(total),
        mmss(elapsed),
        mmss(remaining),
        mmss(elapsed + remaining)
    )
}

/// Run `job` on `dev`, writing `out`. Progress goes to `log`. Returns the
/// number of rows written. The carriage is homed afterwards even on failure.
pub fn scan(
    dev: &mut Device,
    out: &str,
    job: &Job,
    log: &mut dyn FnMut(&str),
) -> Result<usize, Box<dyn std::error::Error>> {
    run(dev, job, log, Some(out), None)
}

/// Like `scan`, but every finished row (8-bit, corrections applied) goes to
/// `rows` instead of a file. `job.depth16` is ignored.
pub fn scan_rows(
    dev: &mut Device,
    job: &Job,
    log: &mut dyn FnMut(&str),
    rows: RowCallback,
) -> Result<usize, Box<dyn std::error::Error>> {
    let job = Job {
        depth16: false,
        ..*job
    };
    run(dev, &job, log, None, Some(rows))
}

fn run(
    dev: &mut Device,
    job: &Job,
    log: &mut dyn FnMut(&str),
    out: Option<&str>,
    callback: Option<RowCallback>,
) -> Result<usize, Box<dyn std::error::Error>> {
    let mode = job.mode;
    let max_value: u16 = if job.depth16 { 65535 } else { 255 };
    let bits = if job.depth16 { 16 } else { 8 };
    let x_pos = job.x_pos.min(mode.full_width_px - 1);
    let width = if job.width == 0 {
        mode.full_width_px - x_pos
    } else {
        job.width.min(mode.full_width_px - x_pos)
    };
    let lines = job
        .lines
        .min(mode.full_lines.saturating_sub(job.y_pos).max(1));
    let deseam = if job.deseam {
        seams::Deseam::new(mode.dpi, width, &job.seams.gaps, x_pos, max_value)
    } else {
        None
    };
    let out_width = deseam.as_ref().map_or(width, |d| d.out_width());
    // Open the output first so a format limit fails before the scanner moves.
    let (mut sink, mut codec) = match (out, callback) {
        (Some(path), _) => Sink::create(
            path,
            out_width,
            lines as usize,
            mode.dpi,
            job.compression,
            if job.gray { 1 } else { 3 },
            bits,
        )?,
        (None, Some(f)) => (
            Sink::Callback(f),
            RowCodec {
                bits: 8,
                big_endian: false,
                buf: Vec::with_capacity(out_width * 3),
            },
        ),
        (None, None) => unreachable!("scan needs a file or a callback"),
    };
    let (tx, rx) = mpsc::sync_channel::<Vec<u16>>(CHANNEL_ROWS);
    let t_writer = Instant::now();
    let job_gray = job.gray;
    let (shift_r, shift_b) = job.register;
    // Lines per full motor step (600 full steps/inch): 4 at 2400 dpi, 2 at 1200.
    let stair_period = match mode.dpi {
        2400 => 4,
        1200 => 2,
        _ => 1,
    };
    let use_stairs = job.destair && stair_period > 1;
    // Vertical stagger of the chips, corrected with the staircase.
    let chip_offsets = if job.destagger {
        seams::chip_offsets(mode.dpi, width, x_pos, &job.seams.dy)
    } else {
        None
    };
    let writer = std::thread::spawn(move || -> io::Result<String> {
        let mut busy = std::time::Duration::ZERO;
        let mut n = 0u32;
        // Vertical stage (staircase or constant registration), then the
        // horizontal seam fill, then the file. Gray rows travel through the
        // stages as three identical channels and are reduced here.
        let gray = job_gray;
        let mut wide = Vec::new();
        let mut mono = Vec::new();
        let mut emit = |r: &[u16]| -> io::Result<()> {
            let full: &[u16] = match &deseam {
                Some(d) => {
                    d.apply(r, &mut wide);
                    &wide
                }
                None => r,
            };
            if gray {
                mono.clear();
                mono.extend(full.iter().step_by(3));
                sink.write_row(codec.encode(&mono))
            } else {
                sink.write_row(codec.encode(full))
            }
        };
        let mut st = if use_stairs {
            Some(
                stairs::Destair::new(width, stair_period, (shift_r, shift_b), max_value)
                    .with_chip_offsets(chip_offsets.clone().unwrap_or_default()),
            )
        } else {
            None
        };
        let mut reg = register::Registration::new(width * 3, shift_r, shift_b, max_value);
        for row in rx {
            let t = Instant::now();
            match st.as_mut() {
                Some(st) => st.push(row, &mut emit)?,
                None => reg.push(row, &mut emit)?,
            }
            busy += t.elapsed();
            n += 1;
        }
        let last_row = t_writer.elapsed();
        let t = Instant::now();
        let phase = match st.as_mut() {
            Some(st) => {
                st.finish(&mut emit)?;
                st.summary()
            }
            None => {
                reg.finish(&mut emit)?;
                "constant registration".to_string()
            }
        };
        sink.finish()?;
        Ok(format!(
            "writer: {n} rows, {:.1?} busy ({:.0} µs/row), last row at {last_row:.1?}, finish took {:.1?}; {phase}; output width {out_width}",
            busy,
            busy.as_secs_f64() * 1e6 / f64::from(n.max(1)),
            t.elapsed()
        ))
    });

    power::set_power_saving(dev.regs(), false)?;
    let regs = dev.regs();
    let t0 = Instant::now();

    // Vendor sequence: home (re-inits), init for the resolution, LED on,
    // lamp on, program, start.
    asic::initiate(regs, 300, 300)?;
    regs.write_bit(0x3a, 2, false)?;
    lamp::switch(regs, false)?;
    motor::home(regs, scan::VENDOR_TIMEOUT)?;
    log(&format!("homed at {:.1?}", t0.elapsed()));

    let banks = asic::initiate(regs, mode.dpi, mode.dpi)?;
    regs.write_bit(0x3a, 2, true)?;
    let mut s = setup::ScanSetup::new(mode, lines);
    s.y_pos = job.y_pos;
    s.depth16 = job.depth16;
    s.motor_vref = job.motor_vref.min(3);
    s.gray = job.gray;
    if !job.raw {
        let cal_mode = mode.calibration_mode();
        let (afe_offset, shutter_ms) = if job.gray {
            let cal = vf3240_core::calibration::run_gray(regs, cal_mode, banks, &mut |m| {
                log(&format!("  {m}"))
            })?;
            (cal.afe_offset, cal.shutter_ms)
        } else {
            let cal = vf3240_core::calibration::run(regs, cal_mode, banks, &mut |m| {
                log(&format!("  {m}"))
            })?;
            (cal.afe_offset, cal.shutter_ms)
        };
        log(&format!(
            "calibrated at {:.1?}: offsets {:03x?} shutters {:.3?} ms",
            t0.elapsed(),
            afe_offset,
            shutter_ms
        ));
        s.afe_offset = afe_offset;
        s.shutter_ms = shutter_ms;
        s.shading = true;
    }
    lamp::switch(regs, true)?;
    if !job.raw {
        vf3240_core::image::upload_gamma(regs, &vf3240_core::image::gamma_lut(1.6))?;
    }
    s.gamma = !job.raw;
    setup::program(regs, &s)?;
    scan::start(regs)?;
    scan::wait_data_ready(regs, scan::VENDOR_TIMEOUT)?;
    log(&format!("data ready at {:.1?}", t0.elapsed()));

    let t_ready = Instant::now();
    let geom = unpack::Geometry::for_dpi(mode.dpi).unwrap();
    let mut raw = vec![0u8; s.raw_line_bytes()];
    let mut got = 0usize;
    let result = (|| -> vf3240_core::Result<()> {
        for i in 0..lines {
            scan::read_line(regs, &mut raw)?;
            let mut rgb: Vec<u16> = Vec::with_capacity(width * 3);
            let x0 = geom.start_pixel_offset + x_pos;
            match (job.gray, job.depth16) {
                (true, false) => {
                    let mut g = Vec::with_capacity(width);
                    unpack::unpack_gray8(&geom, &raw, x0, width, &mut g)
                        .expect("crop fits in the line");
                    rgb.extend(g.iter().flat_map(|&v| [u16::from(v); 3]));
                }
                (true, true) => {
                    let mut g = Vec::with_capacity(width);
                    unpack::unpack_gray16(&geom, &raw, x0, width, &mut g)
                        .expect("crop fits in the line");
                    rgb.extend(g.iter().flat_map(|&v| [v; 3]));
                }
                (false, false) => {
                    let mut c = Vec::with_capacity(width * 3);
                    unpack::unpack_color8(&geom, &raw, x0, width, &mut c)
                        .expect("crop fits in the line");
                    rgb.extend(c.iter().map(|&v| u16::from(v)));
                }
                (false, true) => {
                    unpack::unpack_color16(&geom, &raw, x0, width, &mut rgb)
                        .expect("crop fits in the line");
                }
            }
            if tx.send(rgb).is_err() {
                break; // the writer failed; its error surfaces from join() below
            }
            got += 1;
            if i % PROGRESS_EVERY == 0 && i > 0 {
                log(&progress(i, lines, t0.elapsed(), t_ready.elapsed()));
            }
        }
        Ok(())
    })();
    drop(tx);
    // Always stop and home, even after a read failure; the writer drains meanwhile.
    let stop = scan::stop(regs, scan::VENDOR_TIMEOUT);
    log(&format!("stopped at {:.1?}: {stop:?}", t0.elapsed()));
    regs.write_bit(0x3a, 2, false)?;
    let home = motor::home(regs, scan::VENDOR_TIMEOUT);
    log(&format!("homed at {:.1?}: {home:?}", t0.elapsed()));
    // The ASIC init table sets 0x32 bit 5 (power saver on) every time, so the
    // disable at the start of the job is undone by the scan itself; re-disable
    // now or the scanner powers itself down a few minutes after the last scan.
    power::set_power_saving(regs, false)?;
    let finished = match writer.join() {
        Ok(r) => r,
        Err(e) => {
            let msg = e
                .downcast_ref::<&str>()
                .map(|s| s.to_string())
                .or_else(|| e.downcast_ref::<String>().cloned())
                .unwrap_or_default();
            Err(io::Error::other(format!("writer thread panicked: {msg}")))
        }
    };
    log(&format!("file closed at {:.1?}", t0.elapsed()));
    result?;
    stop?;
    home?;
    log(&finished?);
    log(&format!(
        "{} {out_width}x{got} at {:.1?}",
        out.map_or("scanned".to_string(), |p| format!("wrote {p}:")),
        t0.elapsed()
    ));
    Ok(got)
}

/// Scan the seam-calibration target (top 8.5 in of the bed at 2400 dpi,
/// seam fill and stagger correction off, nothing written to disk) `passes`
/// times, calling `between` before each pass after the first so the sheet
/// can be moved, then measure the 16 seams and merge the median into
/// `base` (`vf3240_image::seamcal`).
pub fn calibrate_seams(
    dev: &mut Device,
    base: &seams::SeamTable,
    passes: usize,
    verbose: bool,
    log: &mut dyn FnMut(&str),
    between: &mut dyn FnMut(usize),
) -> Result<(seams::SeamTable, Vec<String>), Box<dyn std::error::Error>> {
    let mode = setup::Mode::for_dpi(2400).ok_or("2400 dpi")?;
    let job = Job {
        lines: (8.5 * 2400.0) as u32,
        deseam: false,
        destagger: false,
        seams: *base,
        ..Job::full_bed(mode)
    };
    let cols = seamcal::nominal_cols(mode.full_width_px)?;
    let mut results = Vec::new();
    for pass in 0..passes.max(1) {
        if pass > 0 {
            between(pass + 1);
        }
        let sc = Arc::new(Mutex::new(seamcal::SeamColumns::new(
            &cols,
            mode.full_width_px,
        )));
        let sink = Arc::clone(&sc);
        scan_rows(
            dev,
            &job,
            log,
            Box::new(move |row: &[u8]| {
                sink.lock().expect("seam columns").push_row(row);
                Ok(())
            }),
        )?;
        let sc = sc.lock().expect("seam columns");
        results.push(sc.measure(seamcal::BAND, seamcal::STEP, verbose, log));
    }
    Ok(seamcal::merge(base, &cols, &results))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    #[test]
    fn progress_extrapolates_from_the_line_rate() {
        // 1000 of 4000 lines in 10 s of scanning, 14 s since the job began.
        assert_eq!(
            progress(1000, 4000, Duration::from_secs(14), Duration::from_secs(10)),
            "line 1000/4000  25%  elapsed 0:14  remaining 0:30  total ~0:44"
        );
        assert_eq!(mmss(Duration::from_secs(417)), "6:57");
    }

    #[test]
    fn row_codec_narrows_or_orders_bytes() {
        let mut c8 = RowCodec {
            bits: 8,
            big_endian: false,
            buf: Vec::new(),
        };
        assert_eq!(c8.encode(&[1, 255, 0]), &[1, 255, 0]);
        let mut be = RowCodec {
            bits: 16,
            big_endian: true,
            buf: Vec::new(),
        };
        assert_eq!(be.encode(&[0x1234]), &[0x12, 0x34]);
        let mut le = RowCodec {
            bits: 16,
            big_endian: false,
            buf: Vec::new(),
        };
        assert_eq!(le.encode(&[0x1234]), &[0x34, 0x12]);
    }
}
