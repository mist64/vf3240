//! Analog front end (AFE) register access through the ASIC's AFE port.
//! From `docs/protocol/04-asic-init.md` §3.2.

use crate::registers::Registers;
use crate::transport::Transport;
use crate::Result;

/// ASIC register holding the AFE address for a write. From `CtlAfe_WriteAFERegister`.
const REG_AFE_WRITE_ADDR: u16 = 0x51;
/// ASIC register holding the AFE data high byte. Same source.
const REG_AFE_DATA_HI: u16 = 0x5d;
/// ASIC register holding the AFE data low byte; the write to it presumably
/// triggers the serial transaction, as it is always last. Same source.
const REG_AFE_DATA_LO: u16 = 0x5e;

/// Write one 16-bit AFE register, as a single `0x83` transfer of three pairs.
pub fn write_afe_register<T: Transport>(
    regs: &mut Registers<T>,
    afe_addr: u8,
    data: u16,
) -> Result<()> {
    regs.write_multi(&[
        (REG_AFE_WRITE_ADDR, afe_addr),
        (REG_AFE_DATA_HI, (data >> 8) as u8),
        (REG_AFE_DATA_LO, (data & 0xff) as u8),
    ])
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transport::{Call, MockTransport};

    #[test]
    fn afe_write_is_one_six_byte_transfer_low_byte_last() {
        let mut regs = Registers::new(MockTransport::new());
        write_afe_register(&mut regs, 0x02, 0x012f).unwrap();
        assert_eq!(
            regs.transport().calls(),
            vec![Call::ControlOut {
                value: 0x0083,
                index: 0,
                data: vec![0x51, 0x02, 0x5d, 0x01, 0x5e, 0x2f]
            }]
        );
    }
}
