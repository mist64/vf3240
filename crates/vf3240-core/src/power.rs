//! Idle power-saving control. See `docs/protocol/08-power-saving.md`.

use crate::registers::Registers;
use crate::transport::Transport;
use crate::Result;

/// Register whose bit 5 is always set by the vendor's power-saver call.
/// From `MyScanner_SetPowerSaverStatus` in `MyScanner.cpp`.
const REG_POWER_ARM: u16 = 0x35;
/// Register whose bit 5 selects power saving on (1) or off (0). Same source.
const REG_POWER_SELECT: u16 = 0x32;
const POWER_BIT: u8 = 5;

/// Enable or disable the scanner's idle power-down.
///
/// Mirrors `MyScanner_SetPowerSaverStatus`: `0x35` bit 5 := 1, then `0x32`
/// bit 5 := enable. Both registers are refreshed from the device first so the
/// shadow-based bit writes do not clobber their power-on values (both are
/// non-zero at power-on, see `docs/protocol/08-power-saving.md`).
pub fn set_power_saving<T: Transport>(regs: &mut Registers<T>, enable: bool) -> Result<()> {
    regs.refresh(REG_POWER_ARM)?;
    regs.refresh(REG_POWER_SELECT)?;
    regs.write_bit(REG_POWER_ARM, POWER_BIT, true)?;
    regs.write_bit(REG_POWER_SELECT, POWER_BIT, enable)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transport::{Call, MockTransport};

    #[test]
    fn disabling_refreshes_both_registers_then_clears_bit_5_of_0x32_only() {
        let t = MockTransport::new();
        t.push_control_in(&[0x7f, 0x55]); // 0x35 power-on value
        t.push_control_in(&[0xe0, 0x55]); // 0x32 power-on value
        let mut regs = Registers::new(t);
        set_power_saving(&mut regs, false).unwrap();
        assert_eq!(
            regs.transport().calls(),
            vec![
                Call::ControlIn {
                    value: 0x008e,
                    index: 0x3522,
                    len: 2
                },
                Call::ControlIn {
                    value: 0x008e,
                    index: 0x3222,
                    len: 2
                },
                Call::ControlOut {
                    value: 0x0083,
                    index: 0,
                    data: vec![0x35, 0x7f]
                },
                Call::ControlOut {
                    value: 0x0083,
                    index: 0,
                    data: vec![0x32, 0xc0]
                },
            ]
        );
    }

    #[test]
    fn enabling_sets_bit_5_of_0x32() {
        let t = MockTransport::new();
        t.push_control_in(&[0x5f, 0x55]);
        t.push_control_in(&[0xc0, 0x55]);
        let mut regs = Registers::new(t);
        set_power_saving(&mut regs, true).unwrap();
        let calls = regs.transport().calls();
        assert_eq!(
            calls[2],
            Call::ControlOut {
                value: 0x0083,
                index: 0,
                data: vec![0x35, 0x7f]
            }
        );
        assert_eq!(
            calls[3],
            Call::ControlOut {
                value: 0x0083,
                index: 0,
                data: vec![0x32, 0xe0]
            }
        );
    }
}
