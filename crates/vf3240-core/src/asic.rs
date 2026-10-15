//! ASIC initialisation: the vendor's `Api_InitiateASIC3` sequence.
//! From `docs/protocol/04-asic-init.md`, in particular the ordered wire
//! sequence in §4.10.

use crate::afe::write_afe_register;
use crate::bulk::OP_BULK_SETUP;
use crate::image::{identity_gamma, set_shading_banks, upload_gamma};
use crate::registers::Registers;
use crate::tables::{
    AFE_INIT_REG, ASIC_INIT_GPIO000_VALUE, ASIC_INIT_GPIO111_VALUE, ASIC_INIT_GPIO_INDEX,
    ASIC_INIT_REG,
};
use crate::transport::Transport;
use crate::Result;

/// Opcode of a USB-controller register write. From `CMDASIC_WriteUsbRegister`:
/// `wValue = 0x8C`, `wIndex = usb reg`, one data byte, no shadow.
pub const OP_USB_REG_WRITE: u16 = 0x008c;
const _: () = assert!(OP_USB_REG_WRITE != OP_BULK_SETUP);

/// Clock code for 48 MHz on USB registers `0x10` and `0x13`.
/// From `CtlDevice_SetScannerClock` / `CtlDevice_SetAMBAClock` (`Clock_ASIC = Clock_AMBA = 48`).
const CLOCK_CODE_48: u8 = 0x0d;

/// Which init table a resolution pair selects. From `CtlDevice_LoadInitRegData3`.
pub fn init_table(xres: u32, yres: u32) -> [(u16, u8); 117] {
    let mut t = ASIC_INIT_REG;
    let gpio = match (xres, yres) {
        (2400, 2400) | (1200, 2400) => Some(ASIC_INIT_GPIO111_VALUE),
        (1200, 1200) => Some(ASIC_INIT_GPIO000_VALUE),
        _ => None,
    };
    if let Some(v) = gpio {
        t[ASIC_INIT_GPIO_INDEX].1 = v;
    }
    t
}

/// `CCD_DPIHW`: the sensor's hardware resolution the vendor selects for a
/// scan resolution (1200 for ≤ 300 dpi, else 2400). From `MyScanner_StartScan`.
pub fn ccd_dpihw(resolution: u32) -> u32 {
    if resolution <= 300 {
        1200
    } else {
        2400
    }
}

/// Write one USB-controller register (opcode `0x8C`).
pub fn write_usb_register<T: Transport>(
    regs: &Registers<T>,
    usb_reg: u16,
    value: u8,
) -> Result<()> {
    regs.transport()
        .control_out(OP_USB_REG_WRITE, usb_reg, &[value])
}

/// Run the full `Api_InitiateASIC3(xres, yres)` sequence (§4.10 steps 1–40).
///
/// Returns the three shading bank bulk addresses for later uploads.
pub fn initiate<T: Transport>(regs: &mut Registers<T>, xres: u32, yres: u32) -> Result<[u32; 3]> {
    // Steps 1–4: the init table, four batched transfers.
    regs.write_multi(&init_table(xres, yres))?;
    // Steps 5–12: AFE init.
    for &(a, d) in AFE_INIT_REG.iter() {
        write_afe_register(regs, a, d)?;
    }
    // Steps 13–15: CtlDevice_SetSDRAMType(DRAM_SEL=4, ChipN=1).
    regs.write_field(0x0b, 0, 3, 4)?;
    regs.write_bit(0x08, 7, false)?;
    regs.write_bit(0x08, 1, false)?;
    // Steps 16–17: CtlDevice_SetScannerClock(48): 0x0b = (shadow & 0x1f) | 0x60.
    let b = regs.shadow(0x0b).unwrap_or(0) & 0x1f;
    regs.write(0x0b, b | 0x60)?;
    write_usb_register(regs, 0x10, CLOCK_CODE_48)?;
    // Step 18: CtlDevice_SetAMBAClock(48).
    write_usb_register(regs, 0x13, CLOCK_CODE_48)?;
    // Steps 19–20: CtlDevice_SetScanMOD(12).
    regs.write_bit(0x14, 0, false)?;
    regs.write_field(0x06, 5, 3, 0)?;
    // Steps 21–22: CtlADF_ADFInit.
    regs.write(0x0d, 0x02)?;
    regs.write_bit(0x08, 6, true)?;
    // Steps 23–25: shading banks.
    let banks = set_shading_banks(regs, ccd_dpihw(xres.max(yres)))?;
    // Steps 26–36: default gamma.
    upload_gamma(regs, &identity_gamma())?;
    // Steps 37–40: post-init bits.
    regs.write_field(0x34, 2, 2, 0)?;
    regs.write_field(0x34, 5, 2, 0)?;
    regs.write_bit(0x35, 0, false)?;
    regs.write_bit(0x35, 1, true)?;
    Ok(banks)
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
    fn init_table_selection_patches_only_register_0x32() {
        assert_eq!(init_table(300, 300)[42], (0x32, 0x69));
        assert_eq!(init_table(1200, 1200)[42], (0x32, 0x61));
        assert_eq!(init_table(2400, 2400)[42], (0x32, 0x7d));
        assert_eq!(init_table(1200, 2400)[42], (0x32, 0x7d));
        assert_eq!(init_table(600, 600)[42], (0x32, 0x69));
    }

    #[test]
    fn initiate_300dpi_matches_the_documented_wire_sequence() {
        let mut regs = Registers::new(MockTransport::new());
        let banks = initiate(&mut regs, 300, 300).unwrap();
        assert_eq!(banks, [0x1001_4000, 0x1002_4000, 0x1003_4000]);

        let calls = regs.transport().calls();
        // Steps 1–4: the table in 64/64/64/42-byte chunks.
        let mut expected: Vec<Call> = Vec::new();
        let flat: Vec<u8> = ASIC_INIT_REG
            .iter()
            .flat_map(|&(a, v)| [a as u8, v])
            .collect();
        for chunk in flat.chunks(64) {
            expected.push(Call::ControlOut {
                value: 0x0083,
                index: 0,
                data: chunk.to_vec(),
            });
        }
        // Steps 5–12.
        for &(a, d) in AFE_INIT_REG.iter() {
            expected.push(Call::ControlOut {
                value: 0x0083,
                index: 0,
                data: vec![0x51, a, 0x5d, 0, 0x5e, d as u8],
            });
        }
        // Steps 13–25.
        expected.extend([
            reg(0x0b, 0x8c),
            reg(0x08, 0x00),
            reg(0x08, 0x00),
            reg(0x0b, 0x6c),
            Call::ControlOut {
                value: 0x008c,
                index: 0x10,
                data: vec![0x0d],
            },
            Call::ControlOut {
                value: 0x008c,
                index: 0x13,
                data: vec![0x0d],
            },
            reg(0x14, 0x30),
            reg(0x06, 0x10),
            reg(0x0d, 0x02),
            reg(0x08, 0x40),
            reg(0xd0, 0x0a),
            reg(0xd1, 0x12),
            reg(0xd2, 0x1a),
        ]);
        // Steps 26–36.
        expected.extend([
            Call::ControlOut {
                value: 0x0083,
                index: 0,
                data: vec![0xc5, 0, 0xc6, 0],
            },
            Call::ControlOut {
                value: 0x0083,
                index: 0,
                data: vec![0xc7, 0, 0xc8, 0],
            },
            Call::ControlOut {
                value: 0x0083,
                index: 0,
                data: vec![0xc9, 0, 0xca, 0],
            },
            reg(0xbd, 0x00),
            reg(0xbe, 0x07),
        ]);
        let ramp = identity_gamma().to_vec();
        for (hi, mid) in [(0x00u8, 0x00u8), (0x00, 0x02), (0x00, 0x04)] {
            expected.push(Call::ControlOut {
                value: 0x0082,
                index: 1,
                data: vec![0x00, mid, hi, 0x01, 0x00, 0x02, 0, 0],
            });
            expected.push(Call::BulkOut { data: ramp.clone() });
        }
        // Steps 37–40.
        expected.extend([
            reg(0x34, 0x00),
            reg(0x34, 0x00),
            reg(0x35, 0x7e),
            reg(0x35, 0x7e),
        ]);

        assert_eq!(calls.len(), expected.len(), "step count");
        for (i, (got, want)) in calls.iter().zip(expected.iter()).enumerate() {
            assert_eq!(got, want, "step {}", i + 1);
        }
    }
}
