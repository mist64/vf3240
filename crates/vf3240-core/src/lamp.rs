//! CIS lamp (LED bar) control. From `docs/protocol/06-motor.md` §7.

use crate::registers::Registers;
use crate::scan::REG_STATUS;
use crate::transport::Transport;
use crate::Result;

/// Lamp register: bit 4 on/off, bit 5 selects the "XP" lamp. From `CtlLamp_SetLampOnOff` / `CtlLamp_SetXPLampOnOff`.
const REG_LAMP: u16 = 0x03;
const LAMP_ON_BIT: u8 = 4;
const LAMP_XP_BIT: u8 = 5;
/// `0x101` bit 2: lamp currently on. From `CtlLamp_GetLampState`.
const STATUS_LAMP_ON: u8 = 0x04;

/// `Api_GetLampStatus(0)`: the flatbed lamp is reported on iff the status bit
/// is set and the XP lamp is not selected.
pub fn is_on<T: Transport>(regs: &Registers<T>) -> Result<bool> {
    let xp = regs.read(REG_LAMP)? & (1 << LAMP_XP_BIT) != 0;
    let lit = regs.read(REG_STATUS)? & STATUS_LAMP_ON != 0;
    Ok(lit && !xp)
}

/// `Scanner_SwitchLampPower(on)`: switch only if the state differs.
///
/// On: select the flatbed lamp, then pulse bit 4 off and on
/// (`Api_SwitchLampPower(0, 1)`). Off: clear bit 4.
pub fn switch<T: Transport>(regs: &mut Registers<T>, on: bool) -> Result<()> {
    let lit = is_on(regs)?;
    if on && !lit {
        regs.write_bit(REG_LAMP, LAMP_XP_BIT, false)?;
        regs.write_bit(REG_LAMP, LAMP_ON_BIT, false)?;
        regs.write_bit(REG_LAMP, LAMP_ON_BIT, true)?;
    } else if !on && lit {
        regs.write_bit(REG_LAMP, LAMP_ON_BIT, false)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transport::{Call, MockTransport};

    fn reg(a: u8, v: u8) -> Call {
        Call::ControlOut {
            value: 0x0083,
            index: 0,
            data: vec![a, v],
        }
    }

    #[test]
    fn switching_on_from_off_pulses_bit_4() {
        let t = MockTransport::new();
        t.push_control_in(&[0x9f, 0x55]); // 0x03: xp clear
        t.push_control_in(&[0x00, 0x55]); // 0x101: lamp not lit
        let mut regs = Registers::new(t);
        regs.write(0x03, 0x9f).unwrap(); // shadow as after init
        switch(&mut regs, true).unwrap();
        let calls = regs.transport().calls();
        assert_eq!(
            &calls[3..],
            &[reg(0x03, 0x9f), reg(0x03, 0x8f), reg(0x03, 0x9f)]
        );
    }

    #[test]
    fn switching_off_when_lit_clears_bit_4_once() {
        let t = MockTransport::new();
        t.push_control_in(&[0x9f, 0x55]);
        t.push_control_in(&[0x04, 0x55]);
        let mut regs = Registers::new(t);
        regs.write(0x03, 0x9f).unwrap();
        switch(&mut regs, false).unwrap();
        let calls = regs.transport().calls();
        assert_eq!(&calls[3..], &[reg(0x03, 0x8f)]);
    }

    #[test]
    fn no_write_when_already_in_the_requested_state() {
        let t = MockTransport::new();
        t.push_control_in(&[0x9f, 0x55]);
        t.push_control_in(&[0x04, 0x55]);
        let mut regs = Registers::new(t);
        switch(&mut regs, true).unwrap();
        assert_eq!(regs.transport().calls().len(), 2);
    }
}
