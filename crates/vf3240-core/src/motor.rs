//! Carriage motor: step units, slope tables, standalone moves and homing.
//! From `docs/protocol/06-motor.md` §3–§5.

use crate::bulk;
use crate::registers::Registers;
use crate::scan::{wait_motor_stop, REG_STATUS, STATUS_AT_HOME, STATUS_MOTOR_MOVING};
use crate::tables::CURVE_DATA2;
use crate::transport::Transport;
use crate::{Error, Result};
use std::time::Duration;

/// Pixel clock period in µs: `Clock_Pixel / Clock_ASIC = 12 / 48`. From `CtlDevice_SetScannerClock`.
pub const PIXEL_TIME_US: f32 = 0.25;
/// Slope tables are padded to a multiple of this many entries (`StepTim`).
const STEP_TIM: usize = 4;
/// Maximum slope table length (`maxMotorStep`).
const MAX_MOTOR_STEP: usize = 8192;

/// Device addresses and length registers of the three scan slope-table slots.
/// From `CtlMotor_WriteScanSlopeTable`.
const SCAN_SLOTS: [(u32, u16); 3] = [
    (0x1000_0000, 0xa4),
    (0x1000_4000, 0xaa),
    (0x1000_8000, 0xac),
];
/// Same for the two fast slots. From `CtlMotor_WriteFastSlopeTable`.
const FAST_SLOTS: [(u32, u16); 2] = [(0x1000_c000, 0xae), (0x1001_0000, 0xb0)];

/// Motor control register `0x02` bits. From the register map in `06-motor.md` §2.
const REG_MOTOR_CTRL: u16 = 0x02;
/// bit 2: 1 = toward home.
const BIT_DIR_HOME: u8 = 2;
/// bit 3: use the fast table for the feed phase.
const BIT_FAST_MOVE: u8 = 3;
/// bit 4: motor power.
const BIT_MOTOR_POWER: u8 = 4;
/// bit 5: auto go home after scan.
const BIT_AUTO_GO_HOME: u8 = 5;
/// bit 6: written as `!backtrack`.
const BIT_NO_BACKTRACK: u8 = 6;
/// Motor start trigger. From `CtlMotor_SetMotorStartMove`.
const REG_MOTOR_START: u16 = 0x0f;
/// Time-base code, bits 0..2. From `CtlSensor_SetTBTime`.
const REG_TBTIME: u16 = 0x1c;
/// Feed step count, 20-bit big-endian over `0x3d..0x3f`. From `CtlMotor_SetFeedSteps`.
const REG_FEED_STEPS: u16 = 0x3d;
/// Vref selectors. From `CtlMotor_SetMotorVref` / `_2`.
const REG_VREF: u16 = 0x3b;
const REG_VREF2: u16 = 0x3c;
/// Step-unit codes: bits 0..2 scan, bits 3..5 fast. From `CtlMotor_Set{Scan,Fast}TableUnit`.
const REG_UNIT_CODE: u16 = 0xa0;
/// Written with the scan unit by `CtlMotor_CalculateFeedReg`; name unknown.
const REG_A2: u16 = 0xa2;
/// Z1 / Z2 remainders. From `CtlMotor_WriteMotorRemainder`.
const REG_Z1: u16 = 0xb2;
const REG_Z2: u16 = 0xb5;

/// Direction of a move.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    /// `iDir = 0`: toward the home sensor.
    Home,
    /// `iDir = 1`: away from home, the scan direction.
    Away,
}

/// `CtlSensor_SetTBTime(t)`: code for a time-base divider in {1,2,4,8,16,32}.
pub fn tb_code(tb: u32) -> u8 {
    match tb {
        1 => 0,
        2 => 1,
        4 => 2,
        8 => 3,
        16 => 4,
        _ => 5,
    }
}

/// Unit code and `0x37[5]` extension bit for the scan table. From §4.2.
fn scan_unit_code(unit: u32) -> (u8, Option<bool>) {
    match unit {
        1 => (0, Some(false)),
        2 => (1, Some(false)),
        4 => (2, Some(false)),
        8 => (3, Some(false)),
        16 => (3, Some(true)),
        _ => (1, None),
    }
}

/// Unit code for the fast table: as the scan code except that unit 2 falls
/// through to code 2 in the vendor (§4.2). Reproduced deliberately, since
/// every vendor scan exercises it.
fn fast_unit_code(unit: u32) -> (u8, Option<bool>) {
    match unit {
        2 => (2, Some(false)),
        _ => scan_unit_code(unit),
    }
}

pub fn set_scan_table_unit<T: Transport>(regs: &mut Registers<T>, unit: u32) -> Result<()> {
    let (code, ext) = scan_unit_code(unit);
    regs.write_bit(0x38, 5, true)?;
    if let Some(e) = ext {
        regs.write_bit(0x37, 5, e)?;
    }
    regs.write_field(REG_UNIT_CODE, 0, 3, code)
}

pub fn set_fast_table_unit<T: Transport>(regs: &mut Registers<T>, unit: u32) -> Result<()> {
    let (code, ext) = fast_unit_code(unit);
    regs.write_bit(0x38, 5, true)?;
    if let Some(e) = ext {
        regs.write_bit(0x37, 5, e)?;
    }
    regs.write_field(REG_UNIT_CODE, 3, 3, code)
}

/// Period word for one unit-step at `pps`: `trunc(((1e6/pps)/PixelTime)/tb/unit)`.
fn period_word(pps: f32, tb: u32, unit: u32) -> u32 {
    ((((1.0e6_f32 / pps) / PIXEL_TIME_US) / tb as f32) / unit as f32) as u32
}

/// A built slope table plus the sums the Z1/Z2 computation needs.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SlopeTable {
    pub words: Vec<u16>,
    /// `tb × Σ words` (`m_iTableSum`).
    pub sum: u32,
    /// `tb × last word` (`m_iTableHigh`).
    pub high: u32,
}

/// Build a slope table from an ascending pps curve (§4.3): every curve entry
/// below `target_pps` (clamped to 16 bits), then the target word, padded with
/// copies to a multiple of 4, capped at 8192 entries.
pub fn build_slope_table(curve: &[f32], target_pps: f32, tb: u32, unit: u32) -> SlopeTable {
    let mut words: Vec<u16> = curve
        .iter()
        .take_while(|&&p| p < target_pps)
        .map(|&p| period_word(p, tb, unit).min(0xffff) as u16)
        .collect();
    words.push(period_word(target_pps, tb, unit) as u16);
    while words.len() % STEP_TIM != 0 {
        words.push(*words.last().unwrap());
    }
    words.truncate(MAX_MOTOR_STEP);
    let sum = tb * words.iter().map(|&w| u32::from(w)).sum::<u32>();
    let high = tb * u32::from(*words.last().unwrap());
    SlopeTable { words, sum, high }
}

fn upload_table<T: Transport>(
    regs: &mut Registers<T>,
    slots: &[(u32, u16)],
    words: &[u16],
) -> Result<()> {
    let bytes: Vec<u8> = words.iter().flat_map(|w| w.to_le_bytes()).collect();
    let len = u16::try_from(words.len()).expect("table capped at 8192");
    for &(addr, len_reg) in slots {
        bulk::write_block(regs.transport(), addr, &bytes)?;
        regs.write_u16(len_reg, len)?;
    }
    Ok(())
}

/// `CtlMotor_LoadPPSCurveToScanTable`: unit, Vref zeroed, table to all three scan slots.
pub fn load_scan_table<T: Transport>(
    regs: &mut Registers<T>,
    curve: &[f32],
    target_pps: f32,
    tb: u32,
    unit: u32,
) -> Result<SlopeTable> {
    if target_pps > *curve.last().unwrap() {
        return Err(Error::MotorSpeedOutOfRange(target_pps));
    }
    set_scan_table_unit(regs, unit)?;
    regs.write(REG_VREF, 0)?;
    regs.write(REG_VREF2, 0)?;
    let table = build_slope_table(curve, target_pps, tb, unit);
    upload_table(regs, &SCAN_SLOTS, &table.words)?;
    Ok(table)
}

/// `CtlMotor_LoadPPSCurveToFastTable`: unit, Vref written then zeroed, table to both fast slots.
pub fn load_fast_table<T: Transport>(
    regs: &mut Registers<T>,
    curve: &[f32],
    target_pps: f32,
    tb: u32,
    unit: u32,
    vref: u8,
) -> Result<SlopeTable> {
    set_fast_table_unit(regs, unit)?;
    let old = regs.shadow(REG_VREF).unwrap_or(0);
    regs.write(REG_VREF, (old & 0x0f) | (vref << 6) | (vref << 4))?;
    regs.write(REG_VREF, 0)?;
    regs.write(REG_VREF2, 0)?;
    let table = build_slope_table(curve, target_pps, tb, unit);
    upload_table(regs, &FAST_SLOTS, &table.words)?;
    Ok(table)
}

/// `CtlMotor_SetFeedSteps(n)`: 20-bit big-endian over `0x3d..0x3f`.
pub fn set_feed_steps<T: Transport>(regs: &mut Registers<T>, steps: u32) -> Result<()> {
    regs.write_u24(REG_FEED_STEPS, steps & 0xf_ffff)
}

pub fn set_direction<T: Transport>(regs: &mut Registers<T>, dir: Direction) -> Result<()> {
    regs.write_bit(REG_MOTOR_CTRL, BIT_DIR_HOME, dir == Direction::Home)
}

pub fn enable_fast_move<T: Transport>(regs: &mut Registers<T>, on: bool) -> Result<()> {
    regs.write_bit(REG_MOTOR_CTRL, BIT_FAST_MOVE, on)
}

pub fn set_motor_power<T: Transport>(regs: &mut Registers<T>, on: bool) -> Result<()> {
    regs.write_bit(REG_MOTOR_CTRL, BIT_MOTOR_POWER, on)
}

pub fn enable_auto_go_home<T: Transport>(regs: &mut Registers<T>, on: bool) -> Result<()> {
    regs.write_bit(REG_MOTOR_CTRL, BIT_AUTO_GO_HOME, on)
}

/// `CtlMotor_SetStartStop(b)`: bit 6 := `!b`; the scan path passes `b = 1`.
pub fn set_backtrack<T: Transport>(regs: &mut Registers<T>, backtrack: bool) -> Result<()> {
    regs.write_bit(REG_MOTOR_CTRL, BIT_NO_BACKTRACK, !backtrack)
}

/// `CtlMotor_SetMotorVref(gohome, ffast, ssback, fscan)` as a full write of
/// four 2-bit fields (no `-1` fields).
pub fn set_vref<T: Transport>(
    regs: &mut Registers<T>,
    gohome: u8,
    ffast: u8,
    ssback: u8,
    fscan: u8,
) -> Result<()> {
    let v = (gohome << 6) | (ffast << 4) | (ssback << 2) | fscan;
    regs.write(REG_VREF, v)?;
    regs.write(REG_VREF2, v)
}

/// `CtlMotor_SetMotorStartMove`: write `0xFF` to `0x0f`.
pub fn start_move<T: Transport>(regs: &mut Registers<T>) -> Result<()> {
    regs.write(REG_MOTOR_START, 0xff)
}

/// `CtlMotor_WriteMotorRemainder(z1, z2)`.
pub fn write_remainders<T: Transport>(regs: &mut Registers<T>, z1: u32, z2: u32) -> Result<()> {
    regs.write_u24(REG_Z1, z1)?;
    regs.write_u24(REG_Z2, z2)
}

/// Write `0xa2:0xa3` with the scan unit (`CtlMotor_CalculateFeedReg`'s first act).
pub fn write_a2<T: Transport>(regs: &mut Registers<T>, scan_unit: u32) -> Result<()> {
    regs.write_u16(REG_A2, (scan_unit & 0xff) as u16)
}

/// Current motor status bits from `0x101`.
pub fn is_moving<T: Transport>(regs: &Registers<T>) -> Result<bool> {
    Ok(regs.read(REG_STATUS)? & STATUS_MOTOR_MOVING != 0)
}

pub fn is_home<T: Transport>(regs: &Registers<T>) -> Result<bool> {
    Ok(regs.read(REG_STATUS)? & STATUS_AT_HOME != 0)
}

/// `ScanCtrl_MoveMotor2(fpps, steps, dir, unit)`: start a standalone move.
/// Does not wait; follow with [`wait_motor_stop`].
///
/// Returns the fast table that was loaded. The curve is `CurveData2`, so
/// `pps` must be ≤ 2096; higher speeds need `CurveData1` (not transcribed).
pub fn move_motor2<T: Transport>(
    regs: &mut Registers<T>,
    pps: f32,
    steps: u32,
    dir: Direction,
    unit: u32,
) -> Result<SlopeTable> {
    if pps > *CURVE_DATA2.last().unwrap() {
        return Err(Error::MotorSpeedOutOfRange(pps));
    }
    regs.write_bit(0x38, 4, true)?;
    regs.write_bit(0x37, 4, true)?;
    regs.write_field(REG_TBTIME, 0, 3, tb_code(1))?;
    set_direction(regs, dir)?;
    let mut table = load_fast_table(regs, &CURVE_DATA2, pps, 1, unit, 2)?;
    let mut feed = (unit * steps) as i64 - 2 * table.words.len() as i64;
    if feed < 1 {
        // §4.1 step 6a: shrink the ramps so they fit inside the move.
        let idx = ((unit * steps.saturating_sub(1)) >> 1) as usize;
        let lower = CURVE_DATA2[idx.min(CURVE_DATA2.len() - 1)];
        table = load_fast_table(regs, &CURVE_DATA2, lower, 1, unit, 2)?;
        feed = ((unit * steps) as i64 - 2 * table.words.len() as i64).max(1);
    }
    enable_fast_move(regs, true)?;
    set_feed_steps(regs, feed as u32)?;
    set_motor_power(regs, true)?;
    start_move(regs)?;
    Ok(table)
}

/// `ScanCtrl_MoveMotor(fpps, steps, dir)`: the plain standalone move at unit 2,
/// used by calibration to reach the white strip (`09-calibration.md` §2.2).
/// Unlike [`move_motor2`] it does **not** set `0x38[4]`/`0x37[4]` and has no
/// short-move fix-up. Does not wait.
pub fn move_motor<T: Transport>(
    regs: &mut Registers<T>,
    pps: f32,
    steps: u32,
    dir: Direction,
) -> Result<SlopeTable> {
    const UNIT: u32 = 2;
    if pps > *CURVE_DATA2.last().unwrap() {
        return Err(Error::MotorSpeedOutOfRange(pps));
    }
    regs.write_field(REG_TBTIME, 0, 3, tb_code(1))?;
    set_direction(regs, dir)?;
    let table = load_fast_table(regs, &CURVE_DATA2, pps, 1, UNIT, 1)?;
    let feed = (UNIT * steps) as i64 - 2 * table.words.len() as i64;
    enable_fast_move(regs, true)?;
    set_feed_steps(regs, feed.max(1) as u32)?;
    set_motor_power(regs, true)?;
    start_move(regs)?;
    Ok(table)
}

/// The vendor's home move: `Api_MoveMotor2(1900.0, 12000, Home, 4)`, then wait.
///
/// The move is far longer than the bed; stopping relies on the ASIC
/// honouring the home sensor (`06-motor.md` §3.2, inferred). Callers must
/// have verified that on hardware with a short move first.
pub fn home<T: Transport>(regs: &mut Registers<T>, timeout: Duration) -> Result<()> {
    if is_home(regs)? {
        return Ok(());
    }
    if !is_moving(regs)? {
        move_motor2(regs, 1900.0, 12000, Direction::Home, 4)?;
    }
    wait_motor_stop(regs, timeout)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transport::{Call, MockTransport};

    #[test]
    fn home_move_fast_table_matches_the_doc() {
        let t = build_slope_table(&CURVE_DATA2, 1900.0, 1, 4);
        assert_eq!(t.words.len(), 360);
        assert_eq!(&t.words[..4], &[0xffff, 43478, 24390, 16666]);
        assert_eq!(*t.words.last().unwrap(), 526);
    }

    #[test]
    fn scan_table_at_300dpi_matches_the_doc() {
        let t = build_slope_table(&CURVE_DATA2, 1722.65, 1, 4);
        assert_eq!(t.words.len(), 268);
        assert_eq!(&t.words[..6], &[0xffff, 43478, 24390, 16666, 12820, 10416]);
        assert_eq!(&t.words[264..], &[580, 580, 580, 580]);
        assert_eq!(t.sum, 452918);
        assert_eq!(t.high, 580);
    }

    #[test]
    fn fast_table_at_1500pps_unit2_matches_the_doc() {
        let t = build_slope_table(&CURVE_DATA2, 1500.0, 1, 2);
        assert_eq!(t.words.len(), 184);
        assert_eq!(&t.words[..5], &[0xffff, 0xffff, 48780, 33333, 25641]);
        assert_eq!(&t.words[179..], &[1336, 1333, 1333, 1333, 1333]);
    }

    #[test]
    fn unit_codes_including_the_fast_unit2_fallthrough() {
        assert_eq!(scan_unit_code(2), (1, Some(false)));
        assert_eq!(scan_unit_code(4), (2, Some(false)));
        assert_eq!(scan_unit_code(16), (3, Some(true)));
        assert_eq!(fast_unit_code(2), (2, Some(false)));
        assert_eq!(fast_unit_code(8), (3, Some(false)));
    }

    #[test]
    fn home_move_wire_sequence_matches_section_3_2() {
        let mut regs = Registers::new(MockTransport::new());
        // Shadow as after init for the registers the sequence touches.
        regs.write(0x38, 0xe0).unwrap();
        regs.write(0x37, 0x00).unwrap();
        regs.write(0x1c, 0x00).unwrap();
        regs.write(0x02, 0x78).unwrap();
        let n0 = regs.transport().calls().len();
        let table = move_motor2(&mut regs, 1900.0, 12000, Direction::Home, 4).unwrap();
        let calls = &regs.transport().calls()[n0..];
        let reg = |a: u8, v: u8| Call::ControlOut {
            value: 0x0083,
            index: 0,
            data: vec![a, v],
        };
        let bytes: Vec<u8> = table.words.iter().flat_map(|w| w.to_le_bytes()).collect();
        let expected = vec![
            reg(0x38, 0xf0), // 0x38[4]=1
            reg(0x37, 0x10), // 0x37[4]=1
            reg(0x1c, 0x00), // TB=1
            reg(0x02, 0x7c), // dir home: bit2=1
            reg(0x38, 0xf0), // 0x38[5]=1 (already)
            reg(0x37, 0x10), // 0x37[5]=0
            reg(0xa0, 0x10), // fast unit code 2 in bits 3..5
            reg(0x3b, 0xa0), // Vref(2,2,-1,-1) on shadow 0
            reg(0x3b, 0x00),
            reg(0x3c, 0x00),
            Call::ControlOut {
                value: 0x0082,
                index: 1,
                data: vec![0x00, 0xc0, 0x00, 0x10, 0xd0, 0x02, 0, 0],
            },
            Call::BulkOut {
                data: bytes.clone(),
            },
            Call::ControlOut {
                value: 0x0083,
                index: 0,
                data: vec![0xae, 0x01, 0xaf, 0x68],
            },
            Call::ControlOut {
                value: 0x0082,
                index: 1,
                data: vec![0x00, 0x00, 0x01, 0x10, 0xd0, 0x02, 0, 0],
            },
            Call::BulkOut { data: bytes },
            Call::ControlOut {
                value: 0x0083,
                index: 0,
                data: vec![0xb0, 0x01, 0xb1, 0x68],
            },
            reg(0x02, 0x7c), // fast move bit3 (already set)
            Call::ControlOut {
                value: 0x0083,
                index: 0,
                data: vec![0x3d, 0x00, 0x3e, 0xb8, 0x3f, 0xb0],
            }, // 47280
            reg(0x02, 0x7c), // power bit4 (already set)
            reg(0x0f, 0xff),
        ];
        assert_eq!(calls.len(), expected.len());
        for (i, (g, w)) in calls.iter().zip(&expected).enumerate() {
            assert_eq!(g, w, "step {}", i + 1);
        }
    }

    #[test]
    fn calibration_positioning_move_matches_09_section_2_2() {
        let mut regs = Registers::new(MockTransport::new());
        regs.write(0x02, 0x78).unwrap();
        let n0 = regs.transport().calls().len();
        let t = move_motor(&mut regs, 800.0, 225, Direction::Away).unwrap();
        assert_eq!(t.words.len(), 48);
        assert_eq!(&t.words[..6], &[65535, 65535, 48780, 33333, 25641, 20833]);
        assert_eq!(&t.words[46..], &[2528, 2500]);
        let calls = &regs.transport().calls()[n0..];
        assert_eq!(
            calls[0],
            Call::ControlOut {
                value: 0x0083,
                index: 0,
                data: vec![0x1c, 0x00]
            }
        );
        assert!(calls.iter().all(|c| !matches!(c, Call::ControlOut { data, .. } if (data[0] == 0x37 || data[0] == 0x38) && data[1] & 0x10 != 0)));
        assert!(calls.contains(&Call::ControlOut {
            value: 0x0083,
            index: 0,
            data: vec![0x3d, 0x00, 0x3e, 0x01, 0x3f, 0x62]
        }));
    }

    #[test]
    fn home_is_a_no_op_when_already_at_home() {
        let t = MockTransport::new();
        t.push_control_in(&[STATUS_AT_HOME, 0x55]);
        let mut regs = Registers::new(t);
        home(&mut regs, Duration::from_secs(1)).unwrap();
        assert_eq!(regs.transport().calls().len(), 1);
    }

    #[test]
    fn speeds_above_the_curve_are_rejected() {
        let mut regs = Registers::new(MockTransport::new());
        assert!(matches!(
            move_motor2(&mut regs, 5000.0, 10, Direction::Away, 2),
            Err(Error::MotorSpeedOutOfRange(_))
        ));
        assert!(regs.transport().calls().is_empty());
    }
}
