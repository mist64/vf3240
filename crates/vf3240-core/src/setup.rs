//! Scan parameter programming: everything between ASIC init and `scan::start`.
//!
//! From `docs/protocol/05-scan-parameters.md` (§3.3 and the §4.2 step table)
//! with the motor steps from `06-motor.md` §5. 24-bit colour at 300, 600,
//! 1200 and 2400 dpi; the per-dpi constants live in [`Mode`].

use crate::afe::write_afe_register;
use crate::bulk;
use crate::motor::{self, Direction, SlopeTable};
use crate::registers::Registers;
use crate::tables::{
    CCD_COLOR_AFE_TIMING_IDX1, CCD_COLOR_AFE_TIMING_IDX2, CCD_COLOR_AFE_TIMING_IDX4,
    CCD_COLOR_ASIC_TIMING, CURVE_DATA1, CURVE_DATA2,
};
use crate::transport::Transport;
use crate::{Error, Result};

/// Pixel clock period in µs (`CtlDevice_PixelTime`).
const PIXEL_TIME: f32 = 0.25;
/// `MotorGear`: motor steps per inch at the scan's reference. From `.data`.
const MOTOR_GEAR: f32 = 600.0;
/// `FastTBunit`, `FastTBPPS`: fast-feed step unit and speed. From `.data`.
const FAST_UNIT: u32 = 2;
const FAST_PPS: f32 = 1500.0;
/// `ShadingBase` is 0x4000 (`ShadingGain = 4`), so `0x06` bit 3 is set when enabling shading.
const SHADING_BASE_IS_4000: bool = true;
/// Vendor default AFE gain (`MyScanner_SetParameter2`); the offset default is 0x80.
const AFE_GAIN: u16 = 0x29;
/// Register `0xa6:0xa7` init value used by Z1 (never written by the driver).
const REG_A6_INIT: u32 = 240;

/// Per-resolution constants for a colour scan. From `05-scan-parameters.md` §2–§4.
#[derive(Debug, Clone, Copy)]
pub struct Mode {
    pub dpi: u32,
    /// `CCD_DPIHW` / `CtlSensor_iOptRes`.
    pub opt_res: u32,
    /// `ScanTBunit` (from `MyScanner_StartScan`'s ladder; 300 → 4).
    pub scan_unit: u32,
    /// `spTimingIdx`.
    pub timing_idx: u32,
    /// `dwCISPixelsPerSeg`.
    pub seg_pixels: u32,
    /// `fTGTime` in ms (colour).
    pub tg_ms: f32,
    /// `fTGTime` in ms for gray (`10-gray-mode.md` §2): one line carries all three LED pulses.
    pub tg_gray_ms: f32,
    /// `fExposureTimeR/G/B` in ms.
    pub shutter_ms: [f32; 3],
    /// Constant term of `spScanBeginLine` (0x32a at 300 dpi).
    pub begin_line_const: u32,
    /// Y-offset scale of `spScanBeginLine` (2400 or 4800 per inch).
    pub begin_line_scale: f32,
    /// Calibration crop origin (`dwStartPixelOffset` of the calibration copy,
    /// `09-calibration.md` §2.1) and strip distance (§2.2), white-shading lines (§2.3).
    pub cal_start_pixel: usize,
    pub cal_strip_steps: u32,
    pub cal_white_lines: u32,
    /// Full-bed output width and lines from the SANE shim's defaults
    /// (`05-scan-parameters.md` §1.2).
    pub full_width_px: usize,
    pub full_lines: u32,
}

impl Mode {
    /// 300 dpi, 24-bit colour.
    pub const COLOR_300: Mode = Mode {
        dpi: 300,
        opt_res: 1200,
        scan_unit: 4,
        timing_idx: 4,
        seg_pixels: 1296,
        tg_ms: 0.387,
        tg_gray_ms: 1.161,
        shutter_ms: [0.35, 0.33, 0.16],
        begin_line_const: 0x32a,
        begin_line_scale: 2400.0,
        cal_start_pixel: 0x10b,
        cal_strip_steps: 0xe1,
        cal_white_lines: 0x45,
        full_width_px: 3593,
        full_lines: 5040,
    };

    /// 600 dpi, 24-bit colour. Constants from `05-scan-parameters.md` §2.3–2.4,
    /// §3.1–3.2, `09-calibration.md` §2.
    pub const COLOR_600: Mode = Mode {
        dpi: 600,
        opt_res: 2400,
        scan_unit: 8,
        timing_idx: 4,
        seg_pixels: 2592,
        tg_ms: 0.717,
        tg_gray_ms: 2.151,
        shutter_ms: [0.67, 0.67, 0.35],
        begin_line_const: 0x582,
        begin_line_scale: 4800.0,
        cal_start_pixel: 0x216,
        cal_strip_steps: 0xfa,
        cal_white_lines: 0x51,
        full_width_px: 7187,
        full_lines: 10080,
    };

    /// 1200 dpi, 24-bit colour. Constants from `05-scan-parameters.md` §2.3–2.4,
    /// §3.1–3.2, §4.4, `09-calibration.md` §2 (40 white lines because
    /// `dwYResolution == 1200`).
    pub const COLOR_1200: Mode = Mode {
        dpi: 1200,
        opt_res: 2400,
        scan_unit: 2,
        timing_idx: 2,
        seg_pixels: 5184,
        tg_ms: 1.683,
        tg_gray_ms: 5.049,
        shutter_ms: [1.315, 1.315, 0.65],
        begin_line_const: 0x51e,
        begin_line_scale: 4800.0,
        cal_start_pixel: 0x42c,
        cal_strip_steps: 0xfa,
        cal_white_lines: 0x28,
        full_width_px: 14375,
        full_lines: 20160,
    };

    /// 2400 dpi, 24-bit colour. Constants from `05-scan-parameters.md` §2.3–2.4,
    /// §3.1–3.2, §4.4, `09-calibration.md` §2. The line period is long enough
    /// that `Final_TG` exceeds 16 bits and the slope tables use time base 4.
    pub const COLOR_2400: Mode = Mode {
        dpi: 2400,
        opt_res: 2400,
        scan_unit: 8,
        timing_idx: 1,
        seg_pixels: 10368,
        tg_ms: 3.367,
        tg_gray_ms: 10.101,
        shutter_ms: [2.62, 2.62, 2.62],
        begin_line_const: 0x4a6,
        begin_line_scale: 4800.0,
        cal_start_pixel: 0x858,
        cal_strip_steps: 0xfa,
        cal_white_lines: 0x17,
        full_width_px: 28751,
        full_lines: 40320,
    };

    /// 200 dpi, 24-bit colour: the 300 dpi window with `DPISET = 800` (the
    /// ASIC decimates 1200 → 800; 864 output pixels per segment, 05 §4.4) and
    /// the fast motor curve (2584 pps > 2095, 06 §5.4). Calibration runs in
    /// the 300 dpi bucket (09 §1), so the `cal_*` fields are 300's and the
    /// caller must calibrate with [`Mode::calibration_mode`].
    pub const COLOR_200: Mode = Mode {
        dpi: 200,
        opt_res: 1200,
        scan_unit: 4,
        timing_idx: 4,
        seg_pixels: 864,
        tg_ms: 0.387,
        tg_gray_ms: 1.161,
        shutter_ms: [0.35, 0.33, 0.16],
        begin_line_const: 0x35c,
        begin_line_scale: 2400.0,
        cal_start_pixel: 0x10b,
        cal_strip_steps: 0xe1,
        cal_white_lines: 0x45,
        full_width_px: 2396,
        full_lines: 3359,
    };

    /// The mode calibration frames use: 200 dpi calibrates as 300 (09 §1).
    pub fn calibration_mode(&self) -> Mode {
        if self.dpi == 200 {
            Mode::COLOR_300
        } else {
            *self
        }
    }

    /// The mode for a resolution, if transcribed.
    pub fn for_dpi(dpi: u32) -> Option<Mode> {
        match dpi {
            200 => Some(Mode::COLOR_200),
            300 => Some(Mode::COLOR_300),
            600 => Some(Mode::COLOR_600),
            1200 => Some(Mode::COLOR_1200),
            2400 => Some(Mode::COLOR_2400),
            _ => None,
        }
    }

    /// `CCD_ColorAFETiming` row for this mode's timing index (§4.3).
    pub fn afe_timing(&self) -> &'static [(u8, u16); 8] {
        match self.timing_idx {
            1 => &CCD_COLOR_AFE_TIMING_IDX1,
            2 => &CCD_COLOR_AFE_TIMING_IDX2,
            _ => &CCD_COLOR_AFE_TIMING_IDX4,
        }
    }

    /// Bytes per raw colour line: `(24 × seg_pixels / 8) × 3`.
    pub fn raw_line_bytes(&self) -> usize {
        (3 * self.seg_pixels as usize) * 3
    }

    /// Bytes per raw 8-bit gray line: one plane of `W = 3 × seg_pixels` samples.
    pub fn raw_line_bytes_gray(&self) -> usize {
        3 * self.seg_pixels as usize
    }

    /// Gray TG period in 0.25 µs ticks (same arithmetic as [`tg_ticks`](Self::tg_ticks)).
    pub fn tg_ticks_gray(&self) -> u32 {
        ((f64::from(self.tg_gray_ms) * 1000.0) / f64::from(PIXEL_TIME)) as u32
    }

    /// Gray shutter encoding (`10-gray-mode.md` §2): cumulative end times of
    /// the three LED pulses weighted 3/10, 5/10, 2/10 of the colour tick counts.
    pub fn shutter_ticks_gray(ms: [f32; 3]) -> [u32; 3] {
        let [r, g, b] = Mode::shutter_ticks(ms);
        let a = r * 3 / 10;
        let bb = a + g * 5 / 10;
        [a, bb, bb + b * 2 / 10]
    }

    /// TG period in 0.25 µs ticks: `trunc(tg_ms × 1000 / 0.25)` in the
    /// vendor's float→double arithmetic (0.387 → 1547, not 1548).
    pub fn tg_ticks(&self) -> u32 {
        ((f64::from(self.tg_ms) * 1000.0) / f64::from(PIXEL_TIME)) as u32
    }

    /// Shutter times in 0.25 µs ticks, same arithmetic as [`tg_ticks`](Self::tg_ticks).
    pub fn shutter_ticks(ms: [f32; 3]) -> [u32; 3] {
        let t = |ms: f32| ((f64::from(ms) * 1000.0) / f64::from(PIXEL_TIME)) as u32;
        [t(ms[0]), t(ms[1]), t(ms[2])]
    }

    /// `spScanBeginLine` for a Y origin in pixels.
    pub fn begin_line(&self, y_pos: u32) -> u32 {
        (y_pos as f32 / self.dpi as f32 * self.begin_line_scale) as u32 + self.begin_line_const
    }
}

/// What to scan and how.
#[derive(Debug, Clone, Copy)]
pub struct ScanSetup {
    pub mode: Mode,
    /// Output lines (`spTotalScanLines`).
    pub lines: u32,
    /// Y origin in pixels (`dwYPos`); 0 for the top of the bed.
    pub y_pos: u32,
    /// `spStillMode`: no carriage motion (calibration frames).
    pub still: bool,
    /// `spShading`: hardware shading correction.
    pub shading: bool,
    /// `spGammaMapping`: hardware gamma LUT.
    pub gamma: bool,
    /// `spPixelDepth == 16`: 16-bit samples (calibration frames). Doubles the raw line.
    pub depth16: bool,
    /// `spImageType == 1`: gray (one sensor line per output line, 3 LED pulses in it).
    pub gray: bool,
    /// AFE offsets written to AFE registers 5/6/7 (`wOffsetR/G/B`).
    pub afe_offset: [u16; 3],
    /// Shutter (LED on) times in ms for R/G/B (`spShutterTime`).
    pub shutter_ms: [f32; 3],
    /// `spMotorResolution`: 300 for the real scan, 600 for the white-shading frame.
    pub motor_resolution: u32,
    /// `spMotorFixedSpeed`: override the TG-derived speed (464.9 for white shading).
    pub fixed_pps: Option<f32>,
    /// `ScanTBunit` override (8 during the vendor's calibration, else from the mode).
    pub scan_unit: Option<u32>,
    /// Calibration frames use `spScanBeginLine = 0` instead of the mode's lead-in.
    pub begin_line_zero: bool,
    /// Whether to end with the `Vref(0,0,2,2)` writes (real scan and white frame: yes;
    /// still calibration frames: no, see `09-calibration.md` §3.1).
    pub write_vref: bool,
    /// Motor current level (0–3) for the scan and backtrack phases of the
    /// `Vref` registers; the vendor uses 2. Exposed for experiments.
    pub motor_vref: u8,
}

impl ScanSetup {
    /// The vendor's real-scan setup for `mode`: moving, defaults, no correction.
    pub fn new(mode: Mode, lines: u32) -> Self {
        ScanSetup {
            mode,
            lines,
            y_pos: 0,
            still: false,
            shading: false,
            gamma: false,
            depth16: false,
            gray: false,
            afe_offset: [0x80; 3],
            shutter_ms: mode.shutter_ms,
            motor_resolution: mode.dpi,
            fixed_pps: None,
            scan_unit: None,
            begin_line_zero: false,
            write_vref: true,
            motor_vref: 2,
        }
    }

    /// Bytes in one raw line for this setup.
    pub fn raw_line_bytes(&self) -> usize {
        let base = if self.gray {
            self.mode.raw_line_bytes_gray()
        } else {
            self.mode.raw_line_bytes()
        };
        base * if self.depth16 { 2 } else { 1 }
    }
}

/// Host-side values the programming produced that later steps need.
#[derive(Debug, Clone)]
pub struct Programmed {
    pub scan_table: Option<SlopeTable>,
    pub feed_steps: u32,
    pub fast_move: bool,
}

fn ticks_to_pps(final_tg: u32) -> f32 {
    (1.0e6 / PIXEL_TIME) / final_tg as f32
}

/// `ScanCtrl_SetScanParameter` (§4.2) followed by the register writes of
/// `Scanner_SetScanParameter` (§3.3). Colour, flatbed, `spRepeatParams = 0`.
pub fn program<T: Transport>(regs: &mut Registers<T>, s: &ScanSetup) -> Result<Programmed> {
    let m = &s.mode;
    if Mode::for_dpi(m.dpi).is_none() {
        return Err(Error::Unsupported(
            "only 200, 300, 600, 1200 and 2400 dpi are implemented",
        ));
    }
    // Colour: three sub-lines per output line; gray: one (`10-gray-mode.md` §2).
    let ccdlmt: u8 = if s.gray { 0 } else { 2 };

    // Steps 1–2: park CCDLMT and the TG period.
    regs.write_field(0x0c, 0, 4, 0)?;
    regs.write_u24(0x7d, 1)?;
    // Step 4: dummy lines = 0 → one step per line.
    regs.write_field(0x1d, 0, 5, 0)?;
    let step_per_line: u32 = 1;
    // Step 5: back-tracking enabled.
    motor::set_backtrack(regs, true)?;
    // Step 6: WriteColorTiming(timing_idx): 37 ASIC pairs + 8 AFE registers.
    regs.write_multi(&CCD_COLOR_ASIC_TIMING)?;
    for &(a, d) in m.afe_timing().iter() {
        write_afe_register(regs, a, d)?;
    }
    // Step 7: CtlImage_SetColorDepth: `Filter | LineArt<<3 | BitSet<<2`; colour → 0,
    // gray channel 1 → Filter 2, 16-bit → BitSet.
    regs.write_field(
        0x04,
        4,
        4,
        (if s.gray { 2 } else { 0 }) | (if s.depth16 { 4 } else { 0 }),
    )?;
    // Step 8: dummy pixel from the shadow of 0x21:0x22 and 0x1f:0x20.
    let r = |regs: &Registers<T>, a: u16| u32::from(regs.shadow(a).unwrap_or(0));
    let dummy = (r(regs, 0x21) << 8 | r(regs, 0x22)) * 2 + (r(regs, 0x1f) << 8 | r(regs, 0x20));
    regs.write_u16(0x80, dummy as u16)?;
    // Steps 9–10: scan margin window in units of timing_idx optical pixels.
    let xrate = m.opt_res / m.dpi;
    let end = (m.seg_pixels * xrate) & 0xffff;
    let begin_reg = dummy;
    let end_reg = begin_reg + end / m.timing_idx;
    regs.write_u24(0x82, begin_reg)?;
    regs.write_u24(0x85, end_reg)?;
    // Step 11: DPIHW code for opt_res.
    let dpihw_code = match m.opt_res {
        300 | 600 => 0,
        1200 => 1,
        1600 | 2400 => 2,
        _ => 3,
    };
    regs.write_field(0x05, 6, 2, dpihw_code)?;
    // Step 12: DPISET = timing_idx × (opt_res / xrate).
    regs.write_u16(0x2c, (m.timing_idx * (m.opt_res / xrate)) as u16)?;
    // Step 14: scan length counts sub-lines.
    regs.write_u24(0x25, u32::from(ccdlmt + 1) * s.lines)?;
    // Steps 15–16: TG period.
    let tg = if s.gray {
        m.tg_ticks_gray()
    } else {
        m.tg_ticks()
    };
    regs.write_u24(0x7d, tg)?;
    // Steps 17–18: shutter times; gray selects the single-line mode (0x60 bit 2)
    // and encodes cumulative pulse end times.
    regs.write_bit(0x60, 2, s.gray)?;
    let sh = if s.gray {
        Mode::shutter_ticks_gray(s.shutter_ms)
    } else {
        Mode::shutter_ticks(s.shutter_ms)
    };
    regs.write_u24(0x8a, sh[0])?;
    regs.write_u24(0x8d, sh[1])?;
    regs.write_u24(0x90, sh[2])?;

    let (scan_table, feed_steps, fast_move);
    if s.still {
        // §5.2: flat 4-entry slope table at the TG period, no feed, motor unpowered.
        let flat = [tg as u16; 4];
        let bytes: Vec<u8> = flat.iter().flat_map(|w| w.to_le_bytes()).collect();
        for (addr, len_reg) in [
            (0x1000_0000u32, 0xa4u16),
            (0x1000_4000, 0xaa),
            (0x1000_8000, 0xac),
        ] {
            bulk::write_block(regs.transport(), addr, &bytes)?;
            regs.write_u16(len_reg, 4)?;
        }
        motor::set_feed_steps(regs, 1)?;
        motor::enable_fast_move(regs, false)?;
        motor::set_motor_power(regs, false)?;
        motor::write_remainders(regs, 0, 0)?;
        scan_table = None;
        feed_steps = 1;
        fast_move = false;
    } else {
        // Step 19: Vref(1,1,1,1) — dead, overwritten below, but on the wire.
        regs.write(0x3b, 0x55)?;
        // Steps 20–21 happen inside the table loads. Step 22–24: speed from TG.
        let scan_unit = s.scan_unit.unwrap_or(m.scan_unit);
        let gear_per_res = MOTOR_GEAR / s.motor_resolution as f32;
        let final_tg =
            ((step_per_line as f32 * tg as f32) / gear_per_res) as u32 * u32::from(ccdlmt + 1);
        let tb: u32 = {
            let b = (final_tg >> 16) + 1;
            [1u32, 2, 4, 8, 16, 32]
                .into_iter()
                .find(|&t| t >= b)
                .unwrap_or(32)
        };
        let pps = s.fixed_pps.unwrap_or_else(|| ticks_to_pps(final_tg));
        // Step 20/25: scan table (unit set inside), then step 21/26: fast table.
        // The vendor sets both units first (steps 20–21), then TBTIME, then loads;
        // the loads re-set the units, so the wire order is unit,unit,tb,unit...,unit...
        motor_set_units(regs, scan_unit, FAST_UNIT)?;
        regs.write_field(0x1c, 0, 3, motor::tb_code(tb))?;
        // `crID = (fpps <= 2095.0) ? CurveData2 : CurveData1` (06 §4.1 step 5).
        let curve: &[f32] = if pps <= 2095.0 {
            &CURVE_DATA2
        } else {
            &CURVE_DATA1
        };
        let st = motor::load_scan_table(regs, curve, pps, tb, scan_unit)?;
        let ft = motor::load_fast_table(regs, &CURVE_DATA2, FAST_PPS, tb, FAST_UNIT, 1)?;
        // Step 27: CalculateFeedReg(begin_line / (opt_res / MotorGear)).
        let begin_line = if s.begin_line_zero {
            0
        } else {
            m.begin_line(s.y_pos)
        };
        let begin = (begin_line as f32 / (m.opt_res as f32 / MOTOR_GEAR)) as i64;
        motor::write_a2(regs, scan_unit)?;
        let scan_len = st.words.len() as i64 / scan_unit as i64;
        let fast_len = ft.words.len() as i64 / FAST_UNIT as i64;
        let a2: i64 = 1; // read16(0xa2) / scan_unit, and 0xa2 was just written with scan_unit
        let tmpline = begin - 2 * fast_len - scan_len - a2;
        let (feed, fast) = if tmpline >= 1 {
            (FAST_UNIT as i64 * tmpline, true)
        } else {
            (scan_unit as i64 * (begin - scan_len).max(0), false)
        };
        motor::set_feed_steps(regs, feed as u32)?;
        motor::enable_fast_move(regs, fast)?;
        // Step 28: motor power.
        motor::set_motor_power(regs, true)?;
        // Step 29: Z1/Z2.
        let l = u32::from(ccdlmt + 1);
        let b_value = if fast { scan_unit } else { feed as u32 };
        let period = l * tg;
        let z1 = (st.sum + (REG_A6_INIT - 1) * st.high) % period;
        let z2 = st
            .sum
            .wrapping_add(b_value.wrapping_sub(1).wrapping_mul(st.high))
            % period;
        motor::write_remainders(regs, z1, z2)?;
        scan_table = Some(st);
        feed_steps = feed as u32;
        fast_move = fast;
    }

    // Step 30: SDRAM image buffer (ImageType 1 layout, 1 chip, 0x4000 pages).
    let shd_full_range: u32 = if m.opt_res >= 2400 { 192 } else { 96 };
    let range = (0x4000 - shd_full_range - 40) >> 1;
    let b0 = shd_full_range + 40;
    let banks = [b0, b0 + range - 1, b0 + range, b0 + 2 * range - 1];
    for i in 0..12u16 {
        regs.write_u16(0xe0 + 2 * i, banks[(i % 4) as usize] as u16)?;
    }
    regs.write(0xf8, 0x01)?;
    // Step 31: MAXWD.
    let base = ((range * 1024) >> 8).saturating_sub(1) * 4;
    let maxwd = ((base as f64 + (s.raw_line_bytes() as f64 / 2.0 / 2.0) * 2.0) / 4.0) as u32;
    regs.write_u24(0x28, maxwd)?;
    // Steps 32–35: enables and direction.
    motor::enable_auto_go_home(regs, false)?;
    regs.write_bit(0x05, 3, s.gamma)?;
    regs.write_bit(0x01, 5, s.shading)?;
    if s.shading {
        regs.write_bit(0x06, 3, SHADING_BASE_IS_4000)?;
    }
    motor::set_direction(regs, Direction::Away)?;
    // Steps 36–47: the "assorted bits".
    regs.write_bit(0x19, 0, true)?;
    regs.write_field(0x0c, 0, 4, ccdlmt)?;
    regs.write_bit(0x01, 7, true)?;
    regs.write_bit(0x08, 4, true)?;
    regs.write_bit(0x01, 7, true)?;
    regs.write_bit(0x1e, 0, true)?;
    regs.write_bit(0x19, 0, true)?;
    regs.write_bit(0x1e, 2, true)?;
    regs.write_bit(0x08, 4, false)?;
    regs.write_bit(0x1b, 7, true)?;
    regs.write_bit(0x19, 2, true)?;
    // Steps 49–51: no test pattern, clear counters, settle.
    regs.write_field(0x06, 0, 3, 0)?;
    regs.write(0x0d, 0x05)?;
    std::thread::sleep(std::time::Duration::from_millis(u64::from(ccdlmt + 1)));

    // §3.3: Scanner_SetScanParameter's own writes; the per-dpi ladder has a
    // 2400/300/200 flavour and a 1200/600 flavour.
    regs.write_bit(0x35, 0, true)?;
    regs.write_bit(0x36, 4, true)?;
    regs.write_bit(0x33, 4, false)?;
    match m.dpi {
        1200 | 600 => {
            regs.write_bit(0x4f, 3, false)?;
            regs.write_bit(0x36, 1, true)?;
            regs.write_bit(0x33, 1, m.dpi == 1200)?;
            regs.write_bit(0x32, 0, m.dpi == 1200)?;
        }
        _ => {
            regs.write_bit(0x4f, 3, true)?;
            regs.write(0x68, 0x00)?;
            regs.write(0x69, 0x6d)?;
            regs.write_field(0x13, 5, 3, 1)?;
            regs.write_field(0x12, 5, 3, 7)?;
            regs.write_bit(0x1e, 3, m.dpi != 2400)?;
            if m.dpi != 2400 {
                regs.write_bit(0x19, 7, true)?;
            }
        }
    }
    for a in 2..5u8 {
        write_afe_register(regs, a, AFE_GAIN)?;
    }
    for (i, a) in (5..8u8).enumerate() {
        write_afe_register(regs, a, s.afe_offset[i])?;
    }
    if s.write_vref {
        motor::set_vref(regs, 0, 0, s.motor_vref, s.motor_vref)?;
    }

    Ok(Programmed {
        scan_table,
        feed_steps,
        fast_move,
    })
}

/// Steps 20–21: `CtlMotor_SetScanTableUnit` / `SetFastTableUnit` as called
/// directly by `ScanCtrl_SetScanParameter` before the loads repeat them.
fn motor_set_units<T: Transport>(
    regs: &mut Registers<T>,
    scan_unit: u32,
    fast_unit: u32,
) -> Result<()> {
    motor::set_scan_table_unit(regs, scan_unit)?;
    motor::set_fast_table_unit(regs, fast_unit)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::asic;
    use crate::transport::MockTransport;

    fn programmed(
        still: bool,
        shading: bool,
        gamma: bool,
    ) -> (Registers<MockTransport>, Programmed) {
        let mut regs = Registers::new(MockTransport::new());
        asic::initiate(&mut regs, 300, 300).unwrap();
        regs.write_bit(0x3a, 2, true).unwrap();
        let mut s = ScanSetup::new(Mode::COLOR_300, 5040);
        s.still = still;
        s.shading = shading;
        s.gamma = gamma;
        let p = program(&mut regs, &s).unwrap();
        (regs, p)
    }

    #[test]
    fn derived_constants_match_the_doc() {
        let m = Mode::COLOR_300;
        assert_eq!(m.tg_ticks(), 1547);
        assert_eq!(Mode::shutter_ticks(m.shutter_ms), [1399, 1320, 639]);
        assert_eq!(m.raw_line_bytes(), 11664);
        assert_eq!(m.begin_line(0), 810);
    }

    #[test]
    fn register_snapshot_after_programming_matches_section_4_7() {
        let (regs, p) = programmed(false, true, true);
        let expect: &[(u16, u8)] = &[
            (0x01, 0xa0),
            (0x02, 0x18),
            (0x04, 0x02),
            (0x05, 0x48),
            (0x06, 0x18),
            (0x08, 0x40),
            (0x0c, 0x02),
            (0x0d, 0x05),
            (0x12, 0xe0),
            (0x13, 0x3f),
            (0x19, 0x85),
            (0x1b, 0x80),
            (0x1c, 0x20),
            (0x1d, 0x20),
            (0x1e, 0x1d),
            (0x3b, 0x0a),
            (0x3c, 0x0a),
            (0x4f, 0x08),
            (0x60, 0x00),
            (0x68, 0x00),
            (0x69, 0x6d),
            (0x32, 0x69),
            (0x33, 0x0e),
            (0x35, 0x7f),
            (0x36, 0x1f),
            (0x25, 0x00),
            (0x26, 0x3b),
            (0x27, 0x10),
            (0x28, 0x00),
            (0x29, 0x84),
            (0x2a, 0xa1),
            (0x2c, 0x04),
            (0x2d, 0xb0),
            (0x7d, 0x00),
            (0x7e, 0x06),
            (0x7f, 0x0b),
            (0x80, 0x00),
            (0x81, 0xaf),
            (0x82, 0x00),
            (0x83, 0x00),
            (0x84, 0xaf),
            (0x85, 0x00),
            (0x86, 0x05),
            (0x87, 0xbf),
            (0x8a, 0x00),
            (0x8b, 0x05),
            (0x8c, 0x77),
            (0x8d, 0x00),
            (0x8e, 0x05),
            (0x8f, 0x28),
            (0x90, 0x00),
            (0x91, 0x02),
            (0x92, 0x7f),
            (0xe0, 0x00),
            (0xe1, 0x88),
            (0xe2, 0x20),
            (0xe3, 0x43),
            (0xe4, 0x20),
            (0xe5, 0x44),
            (0xe6, 0x3f),
            (0xe7, 0xff),
            (0xf8, 0x01),
            (0xa2, 0x00),
            (0xa3, 0x04),
        ];
        for &(a, v) in expect {
            assert_eq!(regs.shadow(a), Some(v), "reg 0x{a:02x}");
        }
        assert!(p.fast_move);
        assert_eq!(p.feed_steps, 306);
        let st = p.scan_table.unwrap();
        assert_eq!(st.words.len() % 4, 0);
        assert_eq!(regs.shadow(0x3d), Some(0x00));
        assert_eq!(regs.shadow(0x3e), Some(0x01));
        assert_eq!(regs.shadow(0x3f), Some(0x32));
    }

    #[test]
    fn gray_300_programming_matches_10_gray_mode() {
        let m = Mode::COLOR_300;
        assert_eq!(m.tg_ticks_gray(), 4644);
        assert_eq!(Mode::shutter_ticks_gray(m.shutter_ms), [419, 1079, 1206]);
        assert_eq!(m.raw_line_bytes_gray(), 3888);
        let mut regs = Registers::new(MockTransport::new());
        asic::initiate(&mut regs, 300, 300).unwrap();
        let mut s = ScanSetup::new(m, 5040);
        s.gray = true;
        assert_eq!(s.raw_line_bytes(), 3888);
        program(&mut regs, &s).unwrap();
        assert_eq!(
            (regs.shadow(0x7e), regs.shadow(0x7f)),
            (Some(0x12), Some(0x24))
        ); // TG 4644
        assert_eq!(
            (regs.shadow(0x8b), regs.shadow(0x8c)),
            (Some(0x01), Some(0xa3))
        ); // 419
        assert_eq!(
            (regs.shadow(0x8e), regs.shadow(0x8f)),
            (Some(0x04), Some(0x37))
        ); // 1079
        assert_eq!(
            (regs.shadow(0x91), regs.shadow(0x92)),
            (Some(0x04), Some(0xb6))
        ); // 1206
        assert_eq!(regs.shadow(0x60).map(|v| v & 4), Some(4));
        assert_eq!(regs.shadow(0x04).map(|v| v >> 4), Some(2)); // Filter 2
        assert_eq!(regs.shadow(0x0c).map(|v| v & 0xf), Some(0)); // CCDLMT 0
        assert_eq!(
            (regs.shadow(0x25), regs.shadow(0x26), regs.shadow(0x27)),
            (Some(0), Some(0x13), Some(0xb0))
        ); // 5040 lines
    }

    #[test]
    fn color_200_uses_the_300_window_decimated_and_the_fast_curve() {
        let m = Mode::COLOR_200;
        assert_eq!(m.raw_line_bytes(), 7776);
        assert_eq!(m.calibration_mode().dpi, 300);
        let mut regs = Registers::new(MockTransport::new());
        asic::initiate(&mut regs, 200, 200).unwrap();
        let p = program(&mut regs, &ScanSetup::new(m, 3359)).unwrap();
        assert_eq!(
            (regs.shadow(0x2c), regs.shadow(0x2d)),
            (Some(0x03), Some(0x20))
        ); // DPISET 800
        assert_eq!(
            (regs.shadow(0x86), regs.shadow(0x87)),
            (Some(0x05), Some(0xbf))
        ); // margin end 1471
        assert_eq!(
            (regs.shadow(0x25), regs.shadow(0x26), regs.shadow(0x27)),
            (Some(0), Some(0x27), Some(0x5d))
        ); // 10077
        assert_eq!(
            (regs.shadow(0x28), regs.shadow(0x29), regs.shadow(0x2a)),
            (Some(0), Some(0x82), Some(0xbb))
        ); // MAXWD 33467
        let st = p.scan_table.unwrap();
        // 2584 pps from CurveData1: every entry below the target plus the target, padded to 4.
        let below = CURVE_DATA1.iter().filter(|&&v| v < 2583.98).count();
        assert_eq!(st.words.len(), (below + 1).div_ceil(4) * 4);
        assert!(*st.words.last().unwrap() <= 388); // 4e6 / (2584 × 1 × 4) ≈ 387
    }

    #[test]
    fn color_600_derived_values_match_05_section_4_4() {
        let m = Mode::COLOR_600;
        assert_eq!(m.tg_ticks(), 2868);
        assert_eq!(Mode::shutter_ticks(m.shutter_ms), [2680, 2680, 1399]);
        assert_eq!(m.raw_line_bytes(), 23328);
        let mut regs = Registers::new(MockTransport::new());
        asic::initiate(&mut regs, 600, 600).unwrap();
        let p = program(&mut regs, &ScanSetup::new(m, 10080)).unwrap();
        assert_eq!(
            (regs.shadow(0x2c), regs.shadow(0x2d)),
            (Some(0x09), Some(0x60))
        ); // DPISET 2400
        assert_eq!(
            (regs.shadow(0x86), regs.shadow(0x87)),
            (Some(0x0a), Some(0xcf))
        ); // margin end 2767
        assert_eq!(
            (regs.shadow(0x28), regs.shadow(0x29), regs.shadow(0x2a)),
            (Some(0), Some(0x89), Some(0x93))
        ); // MAXWD 35219
        assert_eq!(
            (regs.shadow(0x25), regs.shadow(0x26), regs.shadow(0x27)),
            (Some(0), Some(0x76), Some(0x20))
        ); // 30240
        assert_eq!(regs.shadow(0x05).map(|v| v >> 6), Some(2)); // DPIHW 2400
        assert_eq!(
            (regs.shadow(0xe0), regs.shadow(0xe1)),
            (Some(0x00), Some(0xe8))
        );
        assert_eq!(p.scan_table.unwrap().words.len(), 28); // 464.9 pps at unit 8
        assert_eq!(regs.shadow(0x4f), Some(0x00));
        assert_eq!(regs.shadow(0x36), Some(0x1f | 0x02));
    }

    #[test]
    fn color_1200_derived_values_match_05_section_4_4() {
        let m = Mode::COLOR_1200;
        assert_eq!(m.tg_ticks(), 6731);
        assert_eq!(Mode::shutter_ticks(m.shutter_ms), [5260, 5260, 2599]);
        assert_eq!(m.raw_line_bytes(), 46656);
        assert_eq!(m.afe_timing()[1], (0x01, 0x000a));
        let mut regs = Registers::new(MockTransport::new());
        asic::initiate(&mut regs, 1200, 1200).unwrap();
        let p = program(&mut regs, &ScanSetup::new(m, 20160)).unwrap();
        assert_eq!(
            (regs.shadow(0x2c), regs.shadow(0x2d)),
            (Some(0x09), Some(0x60))
        ); // DPISET 2400
        assert_eq!(
            (regs.shadow(0x86), regs.shadow(0x87)),
            (Some(0x14), Some(0xef))
        ); // margin end 5359
        assert_eq!(
            (regs.shadow(0x28), regs.shadow(0x29), regs.shadow(0x2a)),
            (Some(0), Some(0x94), Some(0xf7))
        ); // MAXWD 38135
        assert_eq!(
            (regs.shadow(0x25), regs.shadow(0x26), regs.shadow(0x27)),
            (Some(0), Some(0xec), Some(0x40))
        ); // 60480
        assert_eq!(
            (regs.shadow(0x7e), regs.shadow(0x7f)),
            (Some(0x1a), Some(0x4b))
        ); // TG 6731
        assert_eq!(regs.shadow(0x1c).map(|v| v & 7), Some(0)); // Final_TG 40386 → tb 1
        assert_eq!(regs.shadow(0xa0).map(|v| v & 7), Some(1)); // scan unit 2 → code 1
        assert_eq!(regs.shadow(0x4f), Some(0x00));
        assert_eq!(regs.shadow(0x33).map(|v| v & 2), Some(2)); // 1200: 0x33 bit1 = 1
        assert_eq!(regs.shadow(0x32).map(|v| v & 1), Some(1)); // 1200: 0x32 bit0 = 1
        assert!(p.scan_table.unwrap().words.len() >= 4);
    }

    #[test]
    fn color_2400_derived_values_match_05_section_4_4() {
        let m = Mode::COLOR_2400;
        assert_eq!(m.tg_ticks(), 13468);
        assert_eq!(Mode::shutter_ticks(m.shutter_ms), [10479, 10479, 10479]);
        assert_eq!(m.raw_line_bytes(), 93312);
        assert_eq!(m.afe_timing()[1], (0x01, 0x000b));
        let mut regs = Registers::new(MockTransport::new());
        asic::initiate(&mut regs, 2400, 2400).unwrap();
        let p = program(&mut regs, &ScanSetup::new(m, 40320)).unwrap();
        assert_eq!(
            (regs.shadow(0x2c), regs.shadow(0x2d)),
            (Some(0x09), Some(0x60))
        ); // DPISET 2400
        assert_eq!(
            (regs.shadow(0x86), regs.shadow(0x87)),
            (Some(0x29), Some(0x2f))
        ); // margin end 10543
        assert_eq!(
            (regs.shadow(0x28), regs.shadow(0x29), regs.shadow(0x2a)),
            (Some(0), Some(0xab), Some(0xbf))
        ); // MAXWD 43967
        assert_eq!(
            (regs.shadow(0x25), regs.shadow(0x26), regs.shadow(0x27)),
            (Some(0x01), Some(0xd8), Some(0x80))
        ); // 120960
        assert_eq!(
            (regs.shadow(0x7e), regs.shadow(0x7f)),
            (Some(0x34), Some(0x9c))
        ); // TG 13468
        assert_eq!(regs.shadow(0x1c).map(|v| v & 7), Some(2)); // Final_TG 161616 → tb 4 → code 2
        assert_eq!(regs.shadow(0xa0).map(|v| v & 7), Some(3)); // scan unit 8 → code 3
        assert_eq!(regs.shadow(0x4f), Some(0x08));
        assert_eq!(regs.shadow(0x1e).map(|v| v & 8), Some(0)); // 2400: 0x1e bit3 = 0
        assert_eq!(regs.shadow(0x19).map(|v| v & 0x80), Some(0)); // no 0x19 bit7 at 2400
        let st = p.scan_table.unwrap();
        // 24.75 pps at tb 4, unit 8: the first two curve entries (5, 23) then the target.
        assert_eq!(st.words.len(), 4);
        assert_eq!(st.words[0], 25000);
        assert_eq!(*st.words.last().unwrap(), 5050);
    }

    #[test]
    fn uncorrected_mode_clears_shading_and_gamma_bits_only() {
        let (regs, _) = programmed(false, false, false);
        assert_eq!(regs.shadow(0x01), Some(0x80));
        assert_eq!(regs.shadow(0x05), Some(0x40));
        assert_eq!(regs.shadow(0x06), Some(0x10));
    }

    #[test]
    fn white_shading_frame_matches_09_section_3_2() {
        let mut regs = Registers::new(MockTransport::new());
        asic::initiate(&mut regs, 300, 300).unwrap();
        let mut s = ScanSetup::new(Mode::COLOR_300, 69);
        s.depth16 = true;
        s.motor_resolution = 600;
        s.fixed_pps = Some(464.9);
        s.scan_unit = Some(8);
        s.begin_line_zero = true;
        let p = program(&mut regs, &s).unwrap();
        assert_eq!(regs.shadow(0x04), Some(0x42));
        assert_eq!(
            (regs.shadow(0x26), regs.shadow(0x27)),
            (Some(0x00), Some(0xcf))
        );
        assert_eq!(
            (regs.shadow(0x29), regs.shadow(0x2a)),
            (Some(0x8a), Some(0x53))
        );
        let st = p.scan_table.unwrap();
        assert_eq!(st.words.len(), 28);
        assert_eq!(&st.words[..4], &[65535, 21739, 12195, 8333]);
        assert_eq!(*st.words.last().unwrap(), 1075);
        assert_eq!((st.sum, st.high), (162063, 1075));
        assert_eq!(p.feed_steps, 0);
        assert!(!p.fast_move);
        assert_eq!(regs.shadow(0x02), Some(0x10));
        assert_eq!(
            (regs.shadow(0xb3), regs.shadow(0xb4)),
            (Some(0x05), Some(0x12))
        ); // Z1 1298
        assert_eq!(
            (regs.shadow(0xb6), regs.shadow(0xb7)),
            (Some(0x0c), Some(0x7a))
        ); // Z2 3194
        assert_eq!(regs.shadow(0xa0).map(|v| v & 7), Some(3)); // scan unit 8 → code 3
    }

    #[test]
    fn still_mode_leaves_the_motor_unpowered_with_a_flat_table() {
        let (regs, p) = programmed(true, false, false);
        assert_eq!(regs.shadow(0x02), Some(0x00));
        assert_eq!(regs.shadow(0x3d), Some(0));
        assert_eq!(regs.shadow(0x3f), Some(1));
        assert_eq!(regs.shadow(0xa4), Some(0));
        assert_eq!(regs.shadow(0xa5), Some(4));
        assert!(p.scan_table.is_none());
        assert_eq!(regs.shadow(0xb2), Some(0));
    }
}
