//! Image-path helpers from the vendor's `giimage.c`: gamma and shading banks.
//! From `docs/protocol/04-asic-init.md` §4.7–4.8.

use crate::bulk;
use crate::registers::Registers;
use crate::transport::Transport;
use crate::Result;

/// Device memory addresses of the three 256-entry 16-bit gamma tables.
/// From `CtlImage_SetDefaultGamma` / `CtlImage_SetGamma`.
pub const GAMMA_ADDRS: [u32; 3] = [0x0100_0000, 0x0100_0200, 0x0100_0400];

/// First shading bank register; `0xD0..=0xD2` hold the three channel banks
/// in 8 KiB units with an implicit `0x10000000` base. From `CtlImage_SDRamSet_ShadingBank`.
const REG_SHADING_BANK0: u16 = 0xd0;

/// Shading bank register values for `CCD_DPIHW` 1200 (scan ≤ 300 dpi) and
/// 2400 (> 300 dpi). From `04-asic-init.md` §4.7 (float32 emulation of the
/// vendor's formula; `n` = 8 or 16).
pub fn shading_bank_regs(ccd_dpihw: u32) -> [u8; 3] {
    let n: u8 = if ccd_dpihw >= 2400 { 16 } else { 8 };
    let r0 = 0x0a;
    [r0, r0 + n, r0 + 2 * n]
}

/// Host-side bulk address of a shading bank, from its register value.
/// `(reg + 0x8000) * 0x2000` per `CtlImage_SDRamSet_ShadingBank`.
pub fn shading_bank_addr(reg: u8) -> u32 {
    (u32::from(reg) + 0x8000) * 0x2000
}

/// Program the three shading bank registers; returns the bank addresses.
pub fn set_shading_banks<T: Transport>(
    regs: &mut Registers<T>,
    ccd_dpihw: u32,
) -> Result<[u32; 3]> {
    let banks = shading_bank_regs(ccd_dpihw);
    for (i, &b) in banks.iter().enumerate() {
        regs.write(REG_SHADING_BANK0 + i as u16, b)?;
    }
    Ok([
        shading_bank_addr(banks[0]),
        shading_bank_addr(banks[1]),
        shading_bank_addr(banks[2]),
    ])
}

/// The identity gamma ramp: entry `i` is `(i + 1) << 8`, the last is `0xFFFF`,
/// 16-bit little-endian. From `CtlImage_SetDefaultGamma(1)`.
pub fn identity_gamma() -> [u8; 512] {
    let mut out = [0u8; 512];
    for i in 0..256u32 {
        let v: u16 = if i == 255 {
            0xffff
        } else {
            ((i + 1) << 8) as u16
        };
        out[2 * i as usize..2 * i as usize + 2].copy_from_slice(&v.to_le_bytes());
    }
    out
}

/// The vendor's scan-time gamma LUT (`Scanner_SetGamma(16)` → `CtlImage_SetGamma`,
/// `docs/protocol/05-scan-parameters.md` §4.6): `gamma = 16 / 10`,
/// `t[i] = (u16)(int)(pow((i + 1) × 256 / 65536, 1 / gamma) × 65536)` for
/// `i < 255`, `t[255] = 0xFFFF`; 16-bit little-endian.
pub fn gamma_lut(gamma: f64) -> [u8; 512] {
    let mut out = [0u8; 512];
    for i in 0..256usize {
        let v: u16 = if i == 255 {
            0xffff
        } else {
            let x = ((i + 1) * 256) as f64 / 65536.0;
            (x.powf(1.0 / gamma) * 65536.0) as i32 as u16
        };
        out[2 * i..2 * i + 2].copy_from_slice(&v.to_le_bytes());
    }
    out
}

/// Upload the same 512-byte table to all three channel slots, after the
/// vendor's five preparatory register writes. From `04-asic-init.md` §4.8.
pub fn upload_gamma<T: Transport>(regs: &mut Registers<T>, table: &[u8; 512]) -> Result<()> {
    regs.write_u16(0xc5, 0x0000)?;
    regs.write_u16(0xc7, 0x0000)?;
    regs.write_u16(0xc9, 0x0000)?;
    regs.write(0xbd, 0x00)?;
    regs.write(0xbe, 0x07)?;
    for addr in GAMMA_ADDRS {
        bulk::write_block(regs.transport(), addr, table)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identity_gamma_is_a_ramp_ending_in_ffff() {
        let g = identity_gamma();
        assert_eq!(&g[..6], &[0x00, 0x01, 0x00, 0x02, 0x00, 0x03]);
        assert_eq!(&g[508..], &[0x00, 0xff, 0xff, 0xff]);
    }

    #[test]
    fn gamma_lut_is_monotonic_and_ends_at_ffff() {
        let g = gamma_lut(1.6);
        let words: Vec<u16> = g
            .chunks(2)
            .map(|c| u16::from_le_bytes([c[0], c[1]]))
            .collect();
        assert!(words.windows(2).all(|w| w[0] <= w[1]));
        assert_eq!(words[255], 0xffff);
        // (1/256)^(1/1.6) * 65536 = 2048 exactly-ish: 256^(-0.625) = 2^-5 → 65536/32 = 2048
        assert_eq!(words[0], 2048);
        assert_eq!(words[127], ((0.5f64).powf(1.0 / 1.6) * 65536.0) as u16);
    }

    #[test]
    fn shading_banks_match_the_documented_values() {
        assert_eq!(shading_bank_regs(1200), [0x0a, 0x12, 0x1a]);
        assert_eq!(shading_bank_regs(2400), [0x0a, 0x1a, 0x2a]);
        assert_eq!(shading_bank_addr(0x0a), 0x1001_4000);
        assert_eq!(shading_bank_addr(0x1a), 0x1003_4000);
    }
}
