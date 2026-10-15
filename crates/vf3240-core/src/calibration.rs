//! Per-scan calibration: AFE offset, exposure, white and dark shading, and
//! the upload of the per-sample correction table into the ASIC.
//! From `docs/protocol/09-calibration.md`.

use crate::bulk;
use crate::motor::{self, Direction};
use crate::registers::Registers;
use crate::scan;
use crate::setup::{Mode, ScanSetup};
use crate::transport::Transport;
use crate::{lamp, Result};
use std::time::Duration;

/// Speed of the positioning move.
const STRIP_PPS: f32 = 800.0;
/// Bisection passes for offset and exposure (`dwAdjMaxNum`).
const MAX_PASSES: usize = 15;
/// Dark-level target window on the 8-bit average (`nLowBound..=nUpBound`).
const OFFSET_TARGET: (u32, u32) = (6, 7);
/// White-level target window on the 8-bit statistic.
const EXPOSURE_TARGET: (u32, u32) = (0xaa, 0xb4);
/// Lower exposure bound in ms (`fMin`).
const EXPOSURE_MIN_MS: f32 = 0.01;
/// White-shading frame: motor resolution, fixed speed, scan unit.
const WHITE_MOTOR_RES: u32 = 600;
const WHITE_PPS: f32 = 464.9;
const CAL_SCAN_UNIT: u32 = 8;
/// Dark-shading frame lines.
const DARK_LINES: u32 = 10;
/// `CtlImage_ShadingBase × 0xF000`: numerator of the gain.
const GAIN_NUMERATOR: u32 = 0x3c00_0000;
/// `dwOffset = 6` pairs: the table data starts at word 12.
const TABLE_OFFSET_PAIRS: usize = 6;

/// `AFEOffsetValue[idx]`: a 9-bit sign-magnitude sweep (`09-calibration.md` §4.4).
pub fn afe_offset_value(idx: u16) -> u16 {
    if idx < 256 {
        0xff - idx
    } else {
        idx
    }
}

/// Everything calibration produces for the real scan.
#[derive(Debug, Clone)]
pub struct Calibration {
    pub afe_offset: [u16; 3],
    pub shutter_ms: [f32; 3],
    /// Per raw plane (B, R, G), per raw sample σ.
    pub white: [Vec<u16>; 3],
    pub dark: [Vec<u16>; 3],
}

/// Program a frame, start it, read all its lines, stop. Returns the raw bytes.
pub fn capture<T: Transport>(regs: &mut Registers<T>, s: &ScanSetup) -> Result<Vec<u8>> {
    crate::setup::program(regs, s)?;
    scan::start(regs)?;
    scan::wait_data_ready(regs, scan::VENDOR_TIMEOUT)?;
    let line = s.raw_line_bytes();
    let mut raw = vec![0u8; line * s.lines as usize];
    let result = raw
        .chunks_mut(line)
        .try_for_each(|l| scan::read_line(regs, l));
    let stop = scan::stop(regs, scan::VENDOR_TIMEOUT);
    result?;
    stop?;
    Ok(raw)
}

fn still_frame(
    mode: Mode,
    lines: u32,
    depth16: bool,
    afe_offset: [u16; 3],
    shutter_ms: [f32; 3],
) -> ScanSetup {
    let mut s = ScanSetup::new(mode, lines);
    s.still = true;
    s.depth16 = depth16;
    s.afe_offset = afe_offset;
    s.shutter_ms = shutter_ms;
    s.begin_line_zero = true;
    s.scan_unit = Some(CAL_SCAN_UNIT);
    s.write_vref = false;
    s
}

/// `CalcAveragePerBigSegment2`: dark average of the middle 40 % of each CIS
/// segment, from the G plane of one 8-bit raw line. Returns the averages in
/// the order they drive AFE 5, 6, 7 (segments 0, 2, 1).
pub fn dark_averages(raw: &[u8], mode: &Mode) -> [u32; 3] {
    let w = 3 * mode.seg_pixels as usize;
    let s = mode.seg_pixels as usize;
    let g = &raw[2 * w..3 * w];
    let (lo, hi) = ((s as f32 * 0.3) as usize, (s as f32 * 0.7) as usize);
    let n = (hi - lo) as u32;
    let mut sums = [0u32; 3];
    for i in lo..hi {
        sums[0] += u32::from(g[3 * i + 2]);
        sums[1] += u32::from(g[3 * i]);
        sums[2] += u32::from(g[3 * i + 1]);
    }
    [
        (sums[0] / n) & 0xff,
        (sums[1] / n) & 0xff,
        (sums[2] / n) & 0xff,
    ]
}

/// White level per channel from one 8-bit raw line: the maximum over 351
/// ten-pixel section means of packed pixels `[267, 3863)` (the intent of
/// `CalcMaxRGBData`; the vendor's code only keeps the last section, see
/// `09-calibration.md` §5.3).
pub fn white_levels(raw: &[u8], mode: &Mode) -> [u32; 3] {
    let w = 3 * mode.seg_pixels as usize;
    let s = mode.seg_pixels as usize;
    let x0 = mode.cal_start_pixel;
    let extent = mode.dpi as usize * 0xe0c / 300;
    let sections = 0x15f;
    let width = extent / sections;
    let sample = |plane: usize, px: usize| -> u32 {
        let (seg, k) = (px / s, px % s);
        let sigma = w - 1 - (3 * k + seg);
        u32::from(raw[plane * w + sigma])
    };
    let mut best = [0u32; 3];
    for j in 0..sections {
        let mut acc = [0u32; 3];
        for k in 0..width {
            let px = x0 + j * width + k;
            acc[0] += sample(1, px);
            acc[1] += sample(2, px);
            acc[2] += sample(0, px);
        }
        for c in 0..3 {
            best[c] = best[c].max((acc[c] / width as u32) & 0xff);
        }
    }
    best
}

fn u16_at(raw: &[u8], i: usize) -> u16 {
    u16::from_le_bytes([raw[2 * i], raw[2 * i + 1]])
}

/// `CalcGetShadingDataColor3`: per plane and raw sample, sort the `h` line
/// values and average sorted ranks `lo..lo+v` with `v = h/2`,
/// `lo = ((h − v) × 3) >> 2`.
pub fn white_shading(raw: &[u8], w: usize, h: usize) -> [Vec<u16>; 3] {
    let v = h >> 1;
    let lo = ((h - v) * 3) >> 2;
    let mut out = [vec![0u16; w], vec![0u16; w], vec![0u16; w]];
    let mut col = vec![0u16; h];
    for (plane, o) in out.iter_mut().enumerate() {
        for (sigma, slot) in o.iter_mut().enumerate() {
            for (j, c) in col.iter_mut().enumerate() {
                *c = u16_at(raw, sigma + j * 3 * w + plane * w);
            }
            col.sort_unstable();
            let sum: u32 = col[lo..lo + v].iter().map(|&x| u32::from(x)).sum();
            *slot = (sum / v as u32) as u16;
        }
    }
    out
}

/// `CalcGetShadingDataColor2`: per plane and raw sample, plain mean of `h` lines.
pub fn dark_shading(raw: &[u8], w: usize, h: usize) -> [Vec<u16>; 3] {
    let mut out = [vec![0u16; w], vec![0u16; w], vec![0u16; w]];
    for (plane, o) in out.iter_mut().enumerate() {
        for (sigma, slot) in o.iter_mut().enumerate() {
            let sum: u32 = (0..h)
                .map(|j| u32::from(u16_at(raw, sigma + j * 3 * w + plane * w)))
                .sum();
            *slot = (sum / h as u32) as u16;
        }
    }
    out
}

/// `CtlImage_WriteShadingColor`'s per-bank pair table: `[dark, gain]` words,
/// data from word 12, four words skipped at 252 of every 256-word block.
pub fn shading_table(white: &[u16], dark: &[u16]) -> Vec<u8> {
    let mut words: Vec<u16> = Vec::with_capacity(white.len() * 2 + 256);
    words.resize(TABLE_OFFSET_PAIRS * 2, 0);
    for (&wh, &dk) in white.iter().zip(dark) {
        let diff = if dk < wh { u32::from(wh - dk) } else { 1 };
        let gain = GAIN_NUMERATOR / diff;
        words.push(dk);
        words.push(gain.min(0xffff) as u16);
        if words.len() % 0x100 == 0xfc {
            words.extend_from_slice(&[0; 4]);
        }
    }
    words.iter().flat_map(|w| w.to_le_bytes()).collect()
}

/// Upload the three per-plane tables to the shading banks, then clear `0x01` bit 1.
pub fn upload_shading<T: Transport>(
    regs: &mut Registers<T>,
    banks: [u32; 3],
    cal: &Calibration,
) -> Result<()> {
    for (c, &bank) in banks.iter().enumerate() {
        let table = shading_table(&cal.white[c], &cal.dark[c]);
        bulk::write_block(regs.transport(), bank, &table)?;
    }
    regs.write_bit(0x01, 1, false)
}

fn bisect_offset<T: Transport>(
    regs: &mut Registers<T>,
    mode: Mode,
    log: &mut dyn FnMut(&str),
) -> Result<[u16; 3]> {
    let mut idx = [0x7fu16; 3];
    let mut val = [0x80u16; 3];
    let (mut lo_idx, mut hi_idx) = ([0x1ffu16; 3], [0u16; 3]); // vendor's wMin / wMax
    let mut found = [false; 3];
    for pass in 0..MAX_PASSES {
        lamp::switch(regs, false)?;
        let raw = capture(regs, &still_frame(mode, 1, false, val, mode.shutter_ms))?;
        let d = dark_averages(&raw, &mode);
        log(&format!(
            "offset pass {pass}: values {val:03x?} → dark {d:?}"
        ));
        for c in 0..3 {
            if found[c] {
                continue;
            }
            if d[c] > OFFSET_TARGET.1 {
                hi_idx[c] = idx[c];
            } else if d[c] < OFFSET_TARGET.0 {
                lo_idx[c] = idx[c];
            } else {
                found[c] = true;
                continue;
            }
            idx[c] = (lo_idx[c] + hi_idx[c]) / 2;
            val[c] = afe_offset_value(idx[c]);
        }
        if found.iter().all(|&f| f) {
            break;
        }
    }
    Ok(val)
}

fn bisect_exposure<T: Transport>(
    regs: &mut Registers<T>,
    mode: Mode,
    offsets: [u16; 3],
    log: &mut dyn FnMut(&str),
) -> Result<[f32; 3]> {
    let mut cur = mode.shutter_ms;
    let mut hi = [mode.tg_ms; 3];
    let mut lo = [EXPOSURE_MIN_MS; 3];
    let mut found = [false; 3];
    for pass in 0..MAX_PASSES {
        lamp::switch(regs, true)?;
        let raw = capture(regs, &still_frame(mode, 1, false, offsets, cur))?;
        let m = white_levels(&raw, &mode);
        log(&format!(
            "exposure pass {pass}: shutters {cur:?} → white {m:?}"
        ));
        for c in 0..3 {
            if found[c] {
                continue;
            }
            if m[c] > EXPOSURE_TARGET.1 {
                hi[c] = cur[c];
            } else if m[c] < EXPOSURE_TARGET.0 {
                lo[c] = cur[c];
            } else {
                found[c] = true;
                continue;
            }
            cur[c] = ((f64::from(hi[c]) + f64::from(lo[c])) / 2.0) as f32;
        }
        if found.iter().all(|&f| f) {
            break;
        }
    }
    Ok(cur)
}

/// `Scanner_Calibration2` for a colour mode. Leaves the carriage on the
/// strip (plus the white-shading travel), as the vendor does, and uploads
/// the shading tables to `banks` (from `asic::initiate`).
pub fn run<T: Transport>(
    regs: &mut Registers<T>,
    mode: Mode,
    banks: [u32; 3],
    log: &mut dyn FnMut(&str),
) -> Result<Calibration> {
    lamp::switch(regs, true)?;
    std::thread::sleep(Duration::from_millis(100));
    lamp::switch(regs, false)?;
    // Must be the plain move: `move_motor2` sets 0x37[4]/0x38[4], which put
    // the ASIC in a standalone-move mode and leave the carriage standing
    // still during the scan that follows (observed on hardware).
    motor::move_motor(regs, STRIP_PPS, mode.cal_strip_steps, Direction::Away)?;
    scan::wait_motor_stop(regs, scan::VENDOR_TIMEOUT)?;

    let afe_offset = bisect_offset(regs, mode, log)?;
    let shutter_ms = bisect_exposure(regs, mode, afe_offset, log)?;

    let w = 3 * mode.seg_pixels as usize;
    lamp::switch(regs, true)?;
    let mut white_frame = ScanSetup::new(mode, mode.cal_white_lines);
    white_frame.depth16 = true;
    white_frame.afe_offset = afe_offset;
    white_frame.shutter_ms = shutter_ms;
    white_frame.motor_resolution = WHITE_MOTOR_RES;
    white_frame.fixed_pps = Some(WHITE_PPS);
    white_frame.scan_unit = Some(CAL_SCAN_UNIT);
    white_frame.begin_line_zero = true;
    let raw = capture(regs, &white_frame)?;
    let white = white_shading(&raw, w, mode.cal_white_lines as usize);

    lamp::switch(regs, false)?;
    let raw = capture(
        regs,
        &still_frame(mode, DARK_LINES, true, afe_offset, shutter_ms),
    )?;
    let dark = dark_shading(&raw, w, DARK_LINES as usize);
    log(&format!(
        "white mid-plane means B/R/G {:?}, dark {:?}",
        white
            .iter()
            .map(|p| p.iter().map(|&x| u32::from(x)).sum::<u32>() / w as u32)
            .collect::<Vec<_>>(),
        dark.iter()
            .map(|p| p.iter().map(|&x| u32::from(x)).sum::<u32>() / w as u32)
            .collect::<Vec<_>>()
    ));

    let cal = Calibration {
        afe_offset,
        shutter_ms,
        white,
        dark,
    };
    upload_shading(regs, banks, &cal)?;
    Ok(cal)
}

/// Gray calibration result (`10-gray-mode.md` §4): colour offsets and
/// shutters, one white and one dark table over the raw sample index.
#[derive(Debug, Clone)]
pub struct CalibrationGray {
    pub afe_offset: [u16; 3],
    pub shutter_ms: [f32; 3],
    pub white: Vec<u16>,
    pub dark: Vec<u16>,
}

/// `CalcGetShadingDataGray3`: single plane, same rank window as colour.
pub fn white_shading_gray(raw: &[u8], w: usize, h: usize) -> Vec<u16> {
    let v = h >> 1;
    let lo = ((h - v) * 3) >> 2;
    let mut out = vec![0u16; w];
    let mut col = vec![0u16; h];
    for (sigma, slot) in out.iter_mut().enumerate() {
        for (j, c) in col.iter_mut().enumerate() {
            *c = u16_at(raw, sigma + j * w);
        }
        col.sort_unstable();
        let sum: u32 = col[lo..lo + v].iter().map(|&x| u32::from(x)).sum();
        *slot = (sum / v as u32) as u16;
    }
    out
}

/// `CalcGetShadingDataGray2`: single plane, plain mean of `h` lines.
pub fn dark_shading_gray(raw: &[u8], w: usize, h: usize) -> Vec<u16> {
    (0..w)
        .map(|sigma| {
            let sum: u32 = (0..h).map(|j| u32::from(u16_at(raw, sigma + j * w))).sum();
            (sum / h as u32) as u16
        })
        .collect()
}

/// `CtlImage_WriteShadingGray`: the pair table to bank 0 only, then clear `0x01` bit 1.
pub fn upload_shading_gray<T: Transport>(
    regs: &mut Registers<T>,
    bank0: u32,
    cal: &CalibrationGray,
) -> Result<()> {
    let table = shading_table(&cal.white, &cal.dark);
    bulk::write_block(regs.transport(), bank0, &table)?;
    regs.write_bit(0x01, 1, false)
}

/// `Scanner_Calibration2` for a gray mode: the offset and exposure
/// bisections run in colour exactly as [`run`]; the white and dark frames
/// are taken in 16-bit gray and uploaded to bank 0 (`10-gray-mode.md` §4).
pub fn run_gray<T: Transport>(
    regs: &mut Registers<T>,
    mode: Mode,
    banks: [u32; 3],
    log: &mut dyn FnMut(&str),
) -> Result<CalibrationGray> {
    lamp::switch(regs, true)?;
    std::thread::sleep(Duration::from_millis(100));
    lamp::switch(regs, false)?;
    // Must be the plain move: `move_motor2` sets 0x37[4]/0x38[4], which put
    // the ASIC in a standalone-move mode and leave the carriage standing
    // still during the scan that follows (observed on hardware).
    motor::move_motor(regs, STRIP_PPS, mode.cal_strip_steps, Direction::Away)?;
    scan::wait_motor_stop(regs, scan::VENDOR_TIMEOUT)?;

    let afe_offset = bisect_offset(regs, mode, log)?;
    let shutter_ms = bisect_exposure(regs, mode, afe_offset, log)?;

    let w = 3 * mode.seg_pixels as usize;
    lamp::switch(regs, true)?;
    let mut white_frame = ScanSetup::new(mode, mode.cal_white_lines);
    white_frame.gray = true;
    white_frame.depth16 = true;
    white_frame.afe_offset = afe_offset;
    white_frame.shutter_ms = shutter_ms;
    white_frame.motor_resolution = WHITE_MOTOR_RES;
    white_frame.fixed_pps = Some(WHITE_PPS);
    white_frame.scan_unit = Some(CAL_SCAN_UNIT);
    white_frame.begin_line_zero = true;
    let raw = capture(regs, &white_frame)?;
    let white = white_shading_gray(&raw, w, mode.cal_white_lines as usize);

    lamp::switch(regs, false)?;
    let mut dark_frame = still_frame(mode, DARK_LINES, true, afe_offset, shutter_ms);
    dark_frame.gray = true;
    let raw = capture(regs, &dark_frame)?;
    let dark = dark_shading_gray(&raw, w, DARK_LINES as usize);
    log(&format!(
        "gray white mean {}, dark mean {}",
        white.iter().map(|&x| u32::from(x)).sum::<u32>() / w as u32,
        dark.iter().map(|&x| u32::from(x)).sum::<u32>() / w as u32
    ));
    let cal = CalibrationGray {
        afe_offset,
        shutter_ms,
        white,
        dark,
    };
    upload_shading_gray(regs, banks[0], &cal)?;
    Ok(cal)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn afe_offset_table_is_a_sign_magnitude_sweep() {
        assert_eq!(afe_offset_value(0), 0xff);
        assert_eq!(afe_offset_value(0x7f), 0x80);
        assert_eq!(afe_offset_value(0x3f), 0xc0);
        assert_eq!(afe_offset_value(0x13f), 0x13f);
        assert_eq!(afe_offset_value(0x1ff), 0x1ff);
    }

    #[test]
    fn shading_table_places_samples_at_the_documented_words() {
        let white = vec![0xf000u16; 3888];
        let mut dark = vec![0u16; 3888];
        dark[120] = 7; // first sample of block 1
        dark[3887] = 9;
        let t = shading_table(&white, &dark);
        assert_eq!(t.len(), 15816);
        let word = |i: usize| u16::from_le_bytes([t[2 * i], t[2 * i + 1]]);
        assert_eq!(word(12), 0); // σ = 0 dark
        assert_eq!(word(13), 0x4000); // unity gain
        assert_eq!(word(250), 0); // σ = 119 dark
        assert_eq!(word(252), 0); // skipped
        assert_eq!(word(256), 7); // σ = 120
        assert_eq!(word(7906), 9); // σ = 3887
        assert_eq!(u32::from(word(7907)), 0x3c00_0000 / (0xf000 - 9));
    }

    #[test]
    fn gain_saturates_and_handles_white_not_above_dark() {
        let t = shading_table(&[0x100, 5], &[0, 5]);
        let word = |i: usize| u16::from_le_bytes([t[2 * i], t[2 * i + 1]]);
        assert_eq!(word(13), 0xffff);
        assert_eq!(word(15), 0xffff);
    }

    #[test]
    fn dark_averages_read_the_g_plane_middle_of_each_segment() {
        let m = Mode::COLOR_300;
        let w = 3888;
        let mut raw = vec![0u8; 3 * w];
        for i in 0..w {
            raw[2 * w + i] = match i % 3 {
                2 => 10, // segment 0
                0 => 20, // segment 2
                _ => 30, // segment 1
            };
        }
        assert_eq!(dark_averages(&raw, &m), [10, 20, 30]);
    }

    #[test]
    fn white_levels_take_the_brightest_section() {
        let m = Mode::COLOR_300;
        let w = 3888;
        let mut raw = vec![100u8; 3 * w];
        // Brighten packed pixels 2000..2010 in the R plane (plane 1).
        for px in 2000..2010 {
            let (seg, k) = (px / 1296, px % 1296);
            raw[w + (w - 1 - (3 * k + seg))] = 200;
        }
        let lv = white_levels(&raw, &m);
        assert_eq!(lv[1], 100);
        assert!(lv[0] >= 150, "{lv:?}");
    }

    #[test]
    fn white_shading_averages_ranks_26_to_59_of_69() {
        let w = 2;
        let h = 69;
        let mut raw = vec![0u8; 2 * 3 * w * h];
        // plane 0, σ = 0: values 0..69 by line; everything else 0.
        for j in 0..h {
            let v = j as u16;
            raw[2 * (j * 3 * w)..2 * (j * 3 * w) + 2].copy_from_slice(&v.to_le_bytes());
        }
        let out = white_shading(&raw, w, h);
        let expected: u32 = (26..60u32).sum::<u32>() / 34;
        assert_eq!(out[0][0], expected as u16);
        assert_eq!(out[1][0], 0);
    }

    #[test]
    fn dark_shading_is_a_plain_mean() {
        let w = 1;
        let h = 10;
        let mut raw = vec![0u8; 2 * 3 * w * h];
        for j in 0..h {
            raw[2 * (j * 3 * w + 2)..2 * (j * 3 * w + 2) + 2]
                .copy_from_slice(&(j as u16 * 10).to_le_bytes());
        }
        assert_eq!(dark_shading(&raw, w, h)[2][0], 45);
    }

    #[test]
    fn gray_white_shading_uses_the_colour_rank_window_on_one_plane() {
        // w = 2 samples, h = 69 lines: sample 0 is j (sorted ranks 26..59 average 42),
        // sample 1 is 1000 + j.
        let (w, h) = (2usize, 69usize);
        let mut raw = Vec::new();
        for j in 0..h as u16 {
            raw.extend_from_slice(&j.to_le_bytes());
            raw.extend_from_slice(&(1000 + j).to_le_bytes());
        }
        let out = white_shading_gray(&raw, w, h);
        assert_eq!(out, vec![42, 1042]);
        assert_eq!(dark_shading_gray(&raw, w, h), vec![34, 1034]);
    }
}
