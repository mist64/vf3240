//! `vf3240`: scan with the VF3240 from the command line.
//!
//! ```text
//! vf3240 <out.ppm|out.png|out.tif> [lines] [options]   one scan, then exit
//! vf3240 --multiple <dir> [--prefix scan] [options]    scan on every button press
//! vf3240 --seam-target <target.pdf>                    print this, lay it on the bed
//! vf3240 --calibrate-seams [--passes N] [--seams …]    scan the printed target N times (sheet
//!                                                      moved between passes), measure the seams
//! vf3240 --measure-seams <scan.tif>… [--seams …]       measure existing such scans
//! ```
//!
//! `--seams` gives this unit's seam gaps and chip stagger in the compact form
//! `gap/step,…` (16 items at 2400 dpi, `seams::SeamTable::parse`), or
//! `@file` holding that text; the `VF3240_SEAMS` environment variable is
//! the fallback. Without either, a default of 4 px per seam and no stagger
//! is used and every scan start says so. `--no-destagger` keeps
//! the chips' vertical stagger.
//!
//! Options common to both forms: `--dpi 200|300|600|1200|2400` (default 300),
//! `--gray`, `--depth 8|16`, `--area x,y,w,h` (inches, image coordinates,
//! 0 = full width / height), `--uncompressed` (TIFF is Deflate + predictor by
//! default), `--raw` (no calibration), `--no-register | --register R,B`,
//! `--no-destair`, `--no-deseam`, `--vref 0-3`. See `vf3240_scan::job`.
//!
//! `--multiple` waits for a scanner button, scans into `<dir>/<prefix>-NNNN.tif`
//! and loops. Numbers continue from the highest existing file of that prefix,
//! so the tool can be restarted without overwriting anything.
//!
//! Button detection: the key status byte (control request `0x8E`,
//! `wIndex = 0x21`, `MyScanner_GetKeyStatus`) is polled every 50 ms. It is
//! `0x00` until a button is pressed, then holds one bit per button (`0x04`,
//! `0x08`, `0x20`, `0x40`) until a read clears it (`02-opcode-map.md`,
//! verified on hardware). Any non-zero event starts a scan and is logged;
//! presses made during a scan are discarded.

use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::time::{Duration, Instant};
use vf3240_core::transport::Transport;
use vf3240_core::{power, setup, Device};
use vf3240_image::{register, seamcal, seams, target, tiff};
use vf3240_scan::job;

const OP_READ: u16 = 0x008e;
const IDX_KEY: u16 = 0x0021;
const POLL: Duration = Duration::from_millis(50);

fn usage() -> ExitCode {
    eprintln!(
        "usage: vf3240 <out.ppm|out.png|out.tif> [lines] [options]\n       \
         vf3240 --multiple <dir> [--prefix scan] [options]\n       \
         vf3240 --seam-target <target.pdf>\n       \
         vf3240 --calibrate-seams [--passes N] [--seams ...] [--verbose]   (scans the printed target N times, default 3, sheet moved between)\n       \
         vf3240 --measure-seams <scan.tif>... [--seams ...] [--verbose]\n\
         options: [--dpi 200|300|600|1200|2400] [--gray] [--depth 8|16] [--area x,y,w,h inches] \
         [--uncompressed] [--raw] [--no-register | --register R,B] [--no-destair] [--no-deseam] \
         [--no-destagger] [--seams gap/step,...x16 | @file] [--vref 0-3]"
    );
    ExitCode::from(2)
}

/// Take `--flag` out of `args`; true if it was there.
fn flag(args: &mut Vec<String>, name: &str) -> bool {
    let found = args.iter().any(|a| a == name);
    args.retain(|a| a != name);
    found
}

/// Take `--flag value` out of `args`: `Ok(None)` if absent, `Err` if the value is missing.
fn value(args: &mut Vec<String>, name: &str) -> Result<Option<String>, ()> {
    let Some(i) = args.iter().position(|a| a == name) else {
        return Ok(None);
    };
    let v = args.get(i + 1).cloned().ok_or(())?;
    args.drain(i..i + 2);
    Ok(Some(v))
}

fn parse_area(v: &str) -> Option<[f64; 4]> {
    let f: Vec<f64> = v
        .split(',')
        .map(|x| x.parse().ok())
        .collect::<Option<_>>()?;
    if f.len() == 4 && f.iter().all(|x| *x >= 0.0) {
        Some([f[0], f[1], f[2], f[3]])
    } else {
        None
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Action {
    Scan,
    Multiple,
    SeamTarget,
    CalibrateSeams,
    MeasureSeams,
}

/// Everything the user asked for, parsed; `None` on a usage error.
struct Options {
    job: job::Job,
    /// A seam table was given (`--seams` or `VF3240_SEAMS`).
    seams_given: bool,
    action: Action,
    prefix: String,
    verbose: bool,
    /// `--calibrate-seams`: scans of the target in different placements.
    passes: usize,
    /// Output file, directory (`--multiple`) or PDF.
    target: String,
    /// `--measure-seams`: the scans to measure.
    files: Vec<String>,
}

/// `--seams` text, `@file`, or `VF3240_SEAMS`; `None` when neither is given
/// (the caller falls back to `SeamTable::DEFAULT` and warns).
fn seam_table(arg: Option<String>) -> Result<Option<seams::SeamTable>, String> {
    let text = match arg.or_else(|| std::env::var("VF3240_SEAMS").ok()) {
        None => return Ok(None),
        Some(t) => t,
    };
    let text = match text.strip_prefix('@') {
        Some(path) => std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))?,
        None => text,
    };
    seams::SeamTable::parse(text.trim()).map(Some)
}

fn parse(mut args: Vec<String>) -> Option<Options> {
    let raw = flag(&mut args, "--raw");
    let no_register = flag(&mut args, "--no-register");
    let gray = flag(&mut args, "--gray");
    let no_destair = flag(&mut args, "--no-destair");
    let no_deseam = flag(&mut args, "--no-deseam");
    let uncompressed = flag(&mut args, "--uncompressed");
    let no_destagger = flag(&mut args, "--no-destagger");
    let verbose = flag(&mut args, "--verbose");
    let mut action = Action::Scan;
    for (name, a) in [
        ("--multiple", Action::Multiple),
        ("--seam-target", Action::SeamTarget),
        ("--calibrate-seams", Action::CalibrateSeams),
        ("--measure-seams", Action::MeasureSeams),
    ] {
        if flag(&mut args, name) {
            if action != Action::Scan {
                return None;
            }
            action = a;
        }
    }
    let seam_table = match seam_table(value(&mut args, "--seams").ok()?) {
        Ok(t) => t,
        Err(e) => {
            eprintln!("vf3240: --seams: {e}");
            return None;
        }
    };
    let passes = match value(&mut args, "--passes")
        .ok()?
        .map(|v| v.parse::<usize>())
    {
        None => 3,
        Some(Ok(n)) if (1..=9).contains(&n) => n,
        _ => return None,
    };
    let seams_given = seam_table.is_some();
    let seam_table = seam_table.unwrap_or(seams::SeamTable::DEFAULT);
    let area = match value(&mut args, "--area").ok()? {
        None => None,
        Some(v) => Some(parse_area(&v)?),
    };
    let depth16 = match value(&mut args, "--depth").ok()?.as_deref() {
        None | Some("8") => false,
        Some("16") => true,
        _ => return None,
    };
    let register_override = match value(&mut args, "--register").ok()? {
        None => None,
        Some(v) => {
            let (r, b) = v.split_once(',')?;
            Some((r.parse().ok()?, b.parse().ok()?))
        }
    };
    let motor_vref = match value(&mut args, "--vref").ok()?.map(|v| v.parse::<u8>()) {
        None => 2,
        Some(Ok(v)) if v <= 3 => v,
        _ => return None,
    };
    let dpi = match value(&mut args, "--dpi").ok()?.map(|v| v.parse::<u32>()) {
        None => 300,
        Some(Ok(d)) => d,
        _ => return None,
    };
    let prefix = value(&mut args, "--prefix")
        .ok()?
        .unwrap_or_else(|| "scan".into());
    let Some(mode) = setup::Mode::for_dpi(dpi) else {
        eprintln!("vf3240: {dpi} dpi is not implemented (200, 300, 600, 1200 or 2400)");
        return None;
    };
    let mut files = Vec::new();
    let (target, lines) = match (action, args.as_slice()) {
        (Action::CalibrateSeams, []) => (String::new(), mode.full_lines),
        (Action::CalibrateSeams, _) => return None,
        (Action::MeasureSeams, [first, ..]) => {
            files = args.clone();
            (first.clone(), mode.full_lines)
        }
        (_, [target]) => (target.clone(), mode.full_lines),
        (Action::Scan, [target, n]) => match n.parse::<u32>() {
            Ok(n) if n > 0 => (target.clone(), n),
            _ => return None,
        },
        _ => return None,
    };
    let job = job::Job {
        mode,
        lines: area.map_or(lines, |a| {
            if a[3] > 0.0 {
                (a[3] * f64::from(dpi)) as u32
            } else {
                lines
            }
        }),
        raw,
        compression: if uncompressed {
            tiff::Compression::None
        } else {
            tiff::Compression::Deflate
        },
        register: if no_register {
            (0.0, 0.0)
        } else {
            register_override.unwrap_or(register::default_shifts(dpi))
        },
        motor_vref,
        destair: !no_destair,
        deseam: !no_deseam,
        seams: seam_table,
        destagger: !no_destagger,
        gray,
        depth16,
        x_pos: area.map_or(0, |a| (a[0] * f64::from(dpi)) as usize),
        y_pos: area.map_or(0, |a| (a[1] * f64::from(dpi)) as u32),
        width: area.map_or(0, |a| (a[2] * f64::from(dpi)) as usize),
    };
    Some(Options {
        job,
        seams_given,
        action,
        prefix,
        verbose,
        passes,
        target,
        files,
    })
}

fn main() -> ExitCode {
    let Some(opts) = parse(std::env::args().skip(1).collect()) else {
        return usage();
    };
    if !opts.seams_given && matches!(opts.action, Action::Scan | Action::Multiple) {
        eprintln!(
            "vf3240: no --seams given and VF3240_SEAMS unset: using the default seam table \
             (4 px per junction, no stagger), which is only roughly right for any scanner. \
             Run `vf3240 --calibrate-seams` once for this unit."
        );
    }
    let result = match opts.action {
        Action::Multiple => run_multiple(Path::new(&opts.target), &opts.prefix, &opts.job),
        Action::SeamTarget => target::seam_target_pdf()
            .and_then(|pdf| std::fs::write(&opts.target, pdf))
            .map(|()| eprintln!("wrote {}: A4 landscape, print at 100%", opts.target))
            .map_err(Into::into),
        Action::CalibrateSeams => calibrate_seams(&opts, true),
        Action::MeasureSeams => calibrate_seams(&opts, false),
        Action::Scan => Device::open()
            .map_err(Into::into)
            .and_then(|mut dev| {
                job::scan(&mut dev, &opts.target, &opts.job, &mut |m| eprintln!("{m}"))
            })
            .map(|_| ()),
    };
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("vf3240: {e}");
            ExitCode::FAILURE
        }
    }
}

/// `--calibrate-seams`: scan the printed target and measure; or
/// `--measure-seams scan.tif` on an existing full-width 2400 dpi scan made
/// with `--no-deseam --no-destagger`. Prints the table in `--seams` form.
fn calibrate_seams(opts: &Options, scan: bool) -> Result<(), Box<dyn std::error::Error>> {
    let mut log = |m: &str| eprintln!("{m}");
    let (table, report) = if scan {
        let mut dev = Device::open()?;
        let mut between = |pass: usize| {
            eprintln!(
                "pass {pass} of {}: move or turn the sheet a little, then press Enter",
                opts.passes
            );
            let mut line = String::new();
            let _ = std::io::stdin().read_line(&mut line);
        };
        job::calibrate_seams(
            &mut dev,
            &opts.job.seams,
            opts.passes,
            opts.verbose,
            &mut log,
            &mut between,
        )?
    } else {
        seamcal::calibrate_files(&opts.files, &opts.job.seams, opts.verbose, &mut log)?
    };
    for line in report {
        println!("{line}");
    }
    println!("\n--seams {}", table.to_arg());
    Ok(())
}

/// `<dir>/<prefix>-NNNN.tif` with NNNN one above the highest number already there.
fn next_path(dir: &Path, prefix: &str) -> std::io::Result<PathBuf> {
    let mut highest = 0u32;
    for entry in std::fs::read_dir(dir)? {
        let name = entry?.file_name();
        let name = name.to_string_lossy();
        if let Some(rest) = name.strip_prefix(prefix).and_then(|r| r.strip_prefix('-')) {
            let digits: String = rest.chars().take_while(|c| c.is_ascii_digit()).collect();
            if let Ok(n) = digits.parse::<u32>() {
                highest = highest.max(n);
            }
        }
    }
    Ok(dir.join(format!("{prefix}-{:04}.tif", highest + 1)))
}

fn key_status(dev: &mut Device) -> vf3240_core::Result<u8> {
    let mut key = [0u8; 1];
    dev.regs()
        .transport()
        .control_in(OP_READ, IDX_KEY, &mut key)?;
    Ok(key[0])
}

/// Block until a button event is latched; returns the event byte.
fn wait_for_press(dev: &mut Device) -> vf3240_core::Result<u8> {
    loop {
        let k = key_status(dev)?;
        if k != 0 {
            return Ok(k);
        }
        std::thread::sleep(POLL);
    }
}

/// Button number 1–4 for an event byte (bits 2, 3, 5, 6), as the probe saw them.
fn button_number(key: u8) -> Option<u8> {
    match key {
        0x04 => Some(1),
        0x08 => Some(2),
        0x20 => Some(3),
        0x40 => Some(4),
        _ => None,
    }
}

fn run_multiple(
    dir: &Path,
    prefix: &str,
    job: &job::Job,
) -> Result<(), Box<dyn std::error::Error>> {
    std::fs::create_dir_all(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    let mut dev = Device::open()?;
    power::set_power_saving(dev.regs(), false)?;
    // Discard an event latched before we started.
    let stale = key_status(&mut dev)?;
    eprintln!(
        "ready: {} dpi into {}/{prefix}-NNNN.tif{}; waiting for a button",
        job.mode.dpi,
        dir.display(),
        if stale != 0 {
            format!(" (discarded stale key event 0x{stale:02x})")
        } else {
            String::new()
        }
    );
    loop {
        let key = wait_for_press(&mut dev)?;
        let out = next_path(dir, prefix)?;
        let t0 = Instant::now();
        match button_number(key) {
            Some(n) => eprintln!("button {n} (0x{key:02x}): scanning {}", out.display()),
            None => eprintln!("unknown key event 0x{key:02x}: scanning {}", out.display()),
        }
        match job::scan(&mut dev, &out.to_string_lossy(), job, &mut |m| {
            eprintln!("  {m}")
        }) {
            Ok(rows) => eprintln!(
                "done {}: {rows} rows in {:.1?}",
                out.display(),
                t0.elapsed()
            ),
            Err(e) => eprintln!("scan failed: {e}"),
        }
        // A press during the scan is latched; read it away rather than scan again.
        let during = key_status(&mut dev)?;
        if during != 0 {
            eprintln!("ignoring key event 0x{during:02x} made during the scan");
        }
        eprintln!("waiting for a button");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn opts(s: &str) -> Option<Options> {
        parse(s.split_whitespace().map(String::from).collect())
    }

    #[test]
    fn single_scan_is_the_default_and_multiple_takes_a_directory() {
        let o = opts("out.tif").unwrap();
        assert_eq!(o.action, Action::Scan);
        assert_eq!(o.job.seams, seams::SeamTable::DEFAULT);
        assert!(!o.seams_given);
        assert_eq!(o.target, "out.tif");
        assert_eq!(o.job.mode.dpi, 300);
        assert_eq!(o.job.lines, o.job.mode.full_lines);
        assert_eq!(o.job.compression, tiff::Compression::Deflate);
        let o = opts("--multiple . --dpi 2400 --prefix page").unwrap();
        assert_eq!(o.action, Action::Multiple);
        assert_eq!(o.target, ".");
        assert_eq!(o.prefix, "page");
        assert_eq!(o.job.mode.dpi, 2400);
    }

    #[test]
    fn seam_table_and_the_other_actions_parse() {
        let o = opts("x.tif --seams 4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4/0.5 --no-destagger").unwrap();
        assert_eq!(o.job.seams.gaps, [4.0; 16]);
        assert_eq!(o.job.seams.dy[15], 0.5);
        assert!(o.seams_given);
        assert!(!o.job.destagger);
        assert!(opts("x.tif --seams 4,4").is_none());
        assert_eq!(
            opts("--seam-target t.pdf").unwrap().action,
            Action::SeamTarget
        );
        assert_eq!(
            opts("--calibrate-seams --verbose").unwrap().action,
            Action::CalibrateSeams
        );
        assert!(opts("--calibrate-seams cal.tif").is_none());
        assert_eq!(
            opts("--measure-seams cal.tif").unwrap().action,
            Action::MeasureSeams
        );
        assert!(opts("--measure-seams --multiple x").is_none());
    }

    #[test]
    fn lines_only_apply_to_a_single_scan() {
        assert_eq!(opts("out.tif 500").unwrap().job.lines, 500);
        assert!(opts("--multiple dir 500").is_none());
        assert!(opts("out.tif 0").is_none());
        assert!(opts("").is_none());
    }

    #[test]
    fn options_are_parsed_and_bad_values_are_usage_errors() {
        let o = opts(
            "x.png --gray --depth 16 --uncompressed --area 1,2,3,4 --dpi 600 --vref 1 --no-destair --no-deseam --raw",
        )
        .unwrap();
        assert!(o.job.gray && o.job.depth16 && o.job.raw && !o.job.destair && !o.job.deseam);
        assert_eq!(o.job.compression, tiff::Compression::None);
        assert_eq!(
            (o.job.x_pos, o.job.y_pos, o.job.width, o.job.lines),
            (600, 1200, 1800, 2400)
        );
        assert_eq!(o.job.motor_vref, 1);
        assert_eq!(
            opts("x.tif --register 0.1,0.2").unwrap().job.register,
            (0.1, 0.2)
        );
        assert_eq!(
            opts("x.tif --no-register").unwrap().job.register,
            (0.0, 0.0)
        );
        for bad in [
            "x.tif --depth 12",
            "x.tif --dpi",
            "x.tif --dpi 150",
            "x.tif --vref 4",
            "x.tif --area 1,2,3",
            "x.tif --register 1",
        ] {
            assert!(opts(bad).is_none(), "{bad}");
        }
    }

    #[test]
    fn next_path_continues_above_the_highest_existing_number() {
        let dir = std::env::temp_dir().join(format!("vf3240-test-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        assert_eq!(next_path(&dir, "scan").unwrap(), dir.join("scan-0001.tif"));
        for name in [
            "scan-0001.tif",
            "scan-0007.tif",
            "scan-0003.tif",
            "other-0099.tif",
            "scan-x.tif",
        ] {
            std::fs::write(dir.join(name), b"").unwrap();
        }
        assert_eq!(next_path(&dir, "scan").unwrap(), dir.join("scan-0008.tif"));
        assert_eq!(
            next_path(&dir, "other").unwrap(),
            dir.join("other-0100.tif")
        );
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn button_numbers_follow_the_probe_log() {
        assert_eq!(button_number(0x04), Some(1));
        assert_eq!(button_number(0x08), Some(2));
        assert_eq!(button_number(0x20), Some(3));
        assert_eq!(button_number(0x40), Some(4));
        assert_eq!(button_number(0x00), None);
    }
}
