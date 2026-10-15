//! Scan start, data-ready poll, line readout and stop.
//! From `docs/protocol/07-readout.md` §1, §2 and §6. Parameter programming
//! (what happens before `start`) lives in `docs/protocol/05-scan-parameters.md`.

use crate::bulk;
use crate::registers::Registers;
use crate::transport::Transport;
use crate::{Error, Result};
use std::time::{Duration, Instant};

/// Control register whose bit 0 is "scan enable". From `CtlImage_EnableScan`.
const REG_SCAN_CONTROL: u16 = 0x01;
const SCAN_ENABLE_BIT: u8 = 0;
/// Writing `0xFF` here starts the motor. From `CtlMotor_SetMotorStartMove`.
const REG_MOTOR_START: u16 = 0x0f;
const MOTOR_START_VALUE: u8 = 0xff;
/// Status register. From `ScanCtrl_ChkScanDataReady`, `CtlMotor_WaitMotorStop`.
pub const REG_STATUS: u16 = 0x101;
/// `0x101` bit 0: motor moving.
pub const STATUS_MOTOR_MOVING: u8 = 0x01;
/// `0x101` bit 3: carriage at home sensor.
pub const STATUS_AT_HOME: u8 = 0x08;
/// `0x101` bit 4 (inferred): scan finished.
pub const STATUS_SCAN_FINISHED: u8 = 0x10;
/// `0x101` bit 6 (inferred): no image data yet.
pub const STATUS_NO_DATA_YET: u8 = 0x40;
/// Four-byte available-data counter (inferred); non-zero means readable.
pub const REG_DATA_COUNT: u16 = 0x102;

/// The vendor's poll budget (intended as 60 s, measured by it in CPU time).
pub const VENDOR_TIMEOUT: Duration = Duration::from_secs(60);
/// Pause between status polls. The vendor spins; we don't.
const POLL_INTERVAL: Duration = Duration::from_millis(2);

/// `ScanCtrl_StartScan` (flatbed): scan enable, then motor start.
pub fn start<T: Transport>(regs: &mut Registers<T>) -> Result<()> {
    regs.write_bit(REG_SCAN_CONTROL, SCAN_ENABLE_BIT, true)?;
    regs.write(REG_MOTOR_START, MOTOR_START_VALUE)
}

/// `ScanCtrl_ChkScanDataReady`: wait until the ASIC reports data.
///
/// Phase A polls `0x101` until "scan finished" (return early) or "no data
/// yet" clears; phase B polls the 4-byte counter at `0x102` until non-zero.
/// Each phase has its own `timeout`.
pub fn wait_data_ready<T: Transport>(regs: &Registers<T>, timeout: Duration) -> Result<()> {
    let t0 = Instant::now();
    loop {
        let st = regs.read(REG_STATUS)?;
        if st & STATUS_SCAN_FINISHED != 0 {
            return Ok(());
        }
        if st & STATUS_NO_DATA_YET == 0 {
            break;
        }
        if t0.elapsed() >= timeout {
            return Err(Error::Timeout("waiting for scan data (status phase)"));
        }
        std::thread::sleep(POLL_INTERVAL);
    }
    let t0 = Instant::now();
    loop {
        if regs.read_u32(REG_DATA_COUNT)? != 0 {
            return Ok(());
        }
        if t0.elapsed() >= timeout {
            return Err(Error::Timeout("waiting for scan data (counter phase)"));
        }
        std::thread::sleep(POLL_INTERVAL);
    }
}

/// Read one raw line of `buf.len()` bytes (`CMDASIC_ReadRawImage`).
pub fn read_line<T: Transport>(regs: &Registers<T>, buf: &mut [u8]) -> Result<()> {
    bulk::read_block(regs.transport(), bulk::IMAGE_ADDR, buf)
}

/// `CtlMotor_WaitMotorStop`: poll `0x101` bit 0 until clear.
pub fn wait_motor_stop<T: Transport>(regs: &Registers<T>, timeout: Duration) -> Result<()> {
    let t0 = Instant::now();
    loop {
        if regs.read(REG_STATUS)? & STATUS_MOTOR_MOVING == 0 {
            return Ok(());
        }
        if t0.elapsed() >= timeout {
            return Err(Error::Timeout("waiting for motor to stop"));
        }
        std::thread::sleep(POLL_INTERVAL);
    }
}

/// `ScanCtrl_StopScan` + `Api_WaitMotorStop`: clear scan enable, wait for the motor.
pub fn stop<T: Transport>(regs: &mut Registers<T>, timeout: Duration) -> Result<()> {
    regs.write_bit(REG_SCAN_CONTROL, SCAN_ENABLE_BIT, false)?;
    wait_motor_stop(regs, timeout)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transport::{Call, MockTransport};

    fn status(v: u8) -> [u8; 2] {
        [v, 0x55]
    }

    #[test]
    fn start_sets_scan_enable_then_kicks_the_motor() {
        let mut regs = Registers::new(MockTransport::new());
        regs.write(0x01, 0x20).unwrap(); // pretend shading-enable bit was programmed
        start(&mut regs).unwrap();
        let calls = regs.transport().calls();
        assert_eq!(
            calls[1],
            Call::ControlOut {
                value: 0x0083,
                index: 0,
                data: vec![0x01, 0x21]
            }
        );
        assert_eq!(
            calls[2],
            Call::ControlOut {
                value: 0x0083,
                index: 0,
                data: vec![0x0f, 0xff]
            }
        );
    }

    #[test]
    fn data_ready_returns_at_once_when_scan_finished_bit_is_set() {
        let t = MockTransport::new();
        t.push_control_in(&status(STATUS_SCAN_FINISHED));
        let regs = Registers::new(t);
        wait_data_ready(&regs, VENDOR_TIMEOUT).unwrap();
        assert_eq!(regs.transport().calls().len(), 1);
    }

    #[test]
    fn data_ready_polls_status_then_the_counter() {
        let t = MockTransport::new();
        t.push_control_in(&status(STATUS_NO_DATA_YET)); // still waiting
        t.push_control_in(&status(0x00)); // go to phase B
        t.push_control_in(&[0, 0, 0, 0, 0x55]); // counter zero
        t.push_control_in(&[0, 0, 0, 1, 0x55]); // counter non-zero
        let regs = Registers::new(t);
        wait_data_ready(&regs, VENDOR_TIMEOUT).unwrap();
        let calls = regs.transport().calls();
        assert_eq!(calls.len(), 4);
        assert_eq!(
            calls[2],
            Call::ControlIn {
                value: 0x018e,
                index: 0x0222,
                len: 5
            }
        );
    }

    #[test]
    fn data_ready_times_out_in_the_status_phase() {
        let t = MockTransport::new();
        for _ in 0..50 {
            t.push_control_in(&status(STATUS_NO_DATA_YET));
        }
        let regs = Registers::new(t);
        let err = wait_data_ready(&regs, Duration::from_millis(5)).unwrap_err();
        assert!(matches!(err, Error::Timeout(_)));
    }

    #[test]
    fn read_line_reads_from_the_image_address() {
        let t = MockTransport::new();
        t.push_bulk_in(&[7; 6]);
        let regs = Registers::new(t);
        let mut buf = [0u8; 6];
        read_line(&regs, &mut buf).unwrap();
        assert_eq!(
            regs.transport().calls()[0],
            Call::ControlOut {
                value: 0x0082,
                index: 0,
                data: vec![0, 0, 0, 0x10, 6, 0, 0, 0]
            }
        );
    }

    #[test]
    fn stop_clears_scan_enable_and_waits_for_the_motor() {
        let t = MockTransport::new();
        t.push_control_in(&status(STATUS_MOTOR_MOVING));
        t.push_control_in(&status(0x00));
        let mut regs = Registers::new(t);
        regs.write(0x01, 0x21).unwrap();
        stop(&mut regs, VENDOR_TIMEOUT).unwrap();
        let calls = regs.transport().calls();
        assert_eq!(
            calls[1],
            Call::ControlOut {
                value: 0x0083,
                index: 0,
                data: vec![0x01, 0x20]
            }
        );
        assert_eq!(calls.len(), 4);
    }
}
