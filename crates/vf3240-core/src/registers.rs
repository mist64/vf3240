//! ASIC register access.
//!
//! Registers are 9-bit addressed. Bit 8 of the address is folded into the high
//! byte of `wValue`; bits 7..0 travel in the payload (writes) or `wIndex`
//! (reads). See `docs/protocol/01-usb-transport.md`.

use crate::transport::Transport;
use crate::{Error, RegisterOp, Result};

/// Highest valid register address (9-bit space). From `CMDASIC_ReadRegister` in `asiccmd.c`.
pub const REG_ADDR_MAX: u16 = 0x01ff;

/// Register-write opcode. From `CMDASIC_WriteRegister` in `asiccmd.c`.
const OP_REG_WRITE: u16 = 0x0083;

/// Register-read opcode. From `CMDASIC_ReadRegister` in `asiccmd.c`.
const OP_REG_READ: u16 = 0x008e;

/// Low byte of `wIndex` on a register read. From `CMDASIC_ReadRegister`.
const REG_READ_INDEX_MAGIC: u16 = 0x0022;

/// Validity magic in byte 1 of a register-read reply. From `CMDASIC_ReadRegister` in `asiccmd.c`.
const REG_READ_REPLY_MAGIC: u8 = 0x55;

/// Pairs per batched write before a flush (64-byte payload). From `CMDASIC_WriteMultiRegister`.
const MULTI_WRITE_MAX_PAIRS: usize = 32;

/// Register file access with a host-side shadow copy.
pub struct Registers<T: Transport> {
    transport: T,
    shadow: [Option<u8>; (REG_ADDR_MAX as usize) + 1],
}

impl<T: Transport> Registers<T> {
    pub fn new(transport: T) -> Self {
        Self {
            transport,
            shadow: [None; (REG_ADDR_MAX as usize) + 1],
        }
    }

    /// Borrow the underlying transport.
    pub fn transport(&self) -> &T {
        &self.transport
    }

    /// Mutably borrow the underlying transport.
    ///
    /// Exists so callers above `Registers` (e.g. `Device::set_timeout`) can
    /// reach transport-level configuration such as the per-transfer timeout,
    /// which has no equivalent on the [`Transport`] trait itself.
    pub fn transport_mut(&mut self) -> &mut T {
        &mut self.transport
    }

    /// Last value written to `addr` by this session, if any.
    ///
    /// The vendor driver updates its shadow *before* issuing the transfer, so a
    /// failed write leaves a lie behind. We update only on success.
    pub fn shadow(&self, addr: u16) -> Option<u8> {
        self.shadow.get(addr as usize).copied().flatten()
    }

    fn check(addr: u16) -> Result<()> {
        if addr > REG_ADDR_MAX {
            return Err(Error::RegisterAddrOutOfRange(addr));
        }
        Ok(())
    }

    fn opcode(base: u16, addr: u16) -> u16 {
        base | ((addr >> 8) & 1) << 8
    }

    /// Write one register.
    pub fn write(&mut self, addr: u16, data: u8) -> Result<()> {
        Self::check(addr)?;
        self.transport
            .control_out(
                Self::opcode(OP_REG_WRITE, addr),
                0x0000,
                &[(addr & 0xff) as u8, data],
            )
            .map_err(|e| Error::RegisterAccess {
                addr,
                op: RegisterOp::Write,
                source: Box::new(e),
            })?;
        self.shadow[addr as usize] = Some(data);
        Ok(())
    }

    /// Read one register.
    pub fn read(&self, addr: u16) -> Result<u8> {
        Self::check(addr)?;
        let mut buf = [0u8; 2];
        self.transport
            .control_in(
                Self::opcode(OP_REG_READ, addr),
                ((addr & 0xff) << 8) | REG_READ_INDEX_MAGIC,
                &mut buf,
            )
            .map_err(|e| Error::RegisterAccess {
                addr,
                op: RegisterOp::Read,
                source: Box::new(e),
            })?;
        if buf[1] != REG_READ_REPLY_MAGIC {
            return Err(Error::BadRegisterMagic {
                addr,
                magic: buf[1],
            });
        }
        Ok(buf[0])
    }

    /// Read a 32-bit value spanning the four registers starting at `addr`.
    ///
    /// From `CMDASIC_ReadFourRegister` in `asiccmd.c`: same framing as
    /// [`read`](Self::read) but a 5-byte reply whose byte 4 must be `0x55`;
    /// the value is bytes 0..4 big-endian. Used on `0x102` by the data-ready
    /// poll (`docs/protocol/01-usb-transport.md`, "Wider accesses").
    pub fn read_u32(&self, addr: u16) -> Result<u32> {
        Self::check(addr)?;
        let mut buf = [0u8; 5];
        self.transport
            .control_in(
                Self::opcode(OP_REG_READ, addr),
                ((addr & 0xff) << 8) | REG_READ_INDEX_MAGIC,
                &mut buf,
            )
            .map_err(|e| Error::RegisterAccess {
                addr,
                op: RegisterOp::Read,
                source: Box::new(e),
            })?;
        if buf[4] != REG_READ_REPLY_MAGIC {
            return Err(Error::BadRegisterMagic {
                addr,
                magic: buf[4],
            });
        }
        Ok(u32::from_be_bytes([buf[0], buf[1], buf[2], buf[3]]))
    }

    /// Read `addr` from the device and seed the shadow with the result.
    ///
    /// Use before the first [`write_field`](Self::write_field) to a register
    /// that powered up non-zero and has not been written by this session
    /// (`docs/protocol/08-power-saving.md`, "Caution").
    pub fn refresh(&mut self, addr: u16) -> Result<u8> {
        let v = self.read(addr)?;
        self.shadow[addr as usize] = Some(v);
        Ok(v)
    }

    /// Write a 16-bit value as high byte at `addr`, low byte at `addr + 1`,
    /// in one transfer. From `CMDASIC_WriteTwiRegister` in `asiccmd.c`
    /// (`docs/protocol/04-asic-init.md` §4.8). The vendor masks the address
    /// to 8 bits, so only the low bank is addressable.
    pub fn write_u16(&mut self, addr: u16, value: u16) -> Result<()> {
        let a = addr & 0xff;
        self.write_multi(&[(a, (value >> 8) as u8), (a + 1, (value & 0xff) as u8)])
    }

    /// Write a 24-bit value big-endian across `addr`, `addr + 1`, `addr + 2`
    /// in one transfer. From `CMDASIC_WriteTriRegister` in `asiccmd.c`
    /// (`docs/protocol/06-motor.md` §2: feed steps `0x3d..0x3f`, TG period
    /// `0x7d..0x7f`, Z1/Z2 `0xb2..0xb7`). Low-bank addresses only.
    pub fn write_u24(&mut self, addr: u16, value: u32) -> Result<()> {
        let a = addr & 0xff;
        self.write_multi(&[
            (a, (value >> 16) as u8),
            (a + 1, (value >> 8) as u8),
            (a + 2, value as u8),
        ])
    }

    /// Write many registers, batched up to 32 pairs per control transfer.
    ///
    /// From `CMDASIC_WriteMultiRegister` in `asiccmd.c`
    /// (`docs/protocol/01-usb-transport.md`, "Batched register writes"):
    /// pairs are split by address bit 8 into an `0x83` stream and an `0x183`
    /// stream, each flushed as one `[addr, value]*` payload whenever it
    /// reaches 64 bytes, then leftovers flushed low stream first. Relative
    /// order within a stream is preserved; order between streams is not.
    ///
    /// Unlike the vendor, which updates its shadow before sending, all
    /// addresses are validated first and the shadow is updated per pair only
    /// after its transfer succeeds.
    pub fn write_multi(&mut self, pairs: &[(u16, u8)]) -> Result<()> {
        for &(addr, _) in pairs {
            Self::check(addr)?;
        }
        let mut low: Vec<(u16, u8)> = Vec::new();
        let mut high: Vec<(u16, u8)> = Vec::new();
        for &(addr, value) in pairs {
            let stream = if addr & 0x100 == 0 {
                &mut low
            } else {
                &mut high
            };
            stream.push((addr, value));
            if low.len() == MULTI_WRITE_MAX_PAIRS {
                self.flush_multi(OP_REG_WRITE, &low)?;
                low.clear();
            }
            if high.len() == MULTI_WRITE_MAX_PAIRS {
                self.flush_multi(OP_REG_WRITE | 0x100, &high)?;
                high.clear();
            }
        }
        if !low.is_empty() {
            self.flush_multi(OP_REG_WRITE, &low)?;
        }
        if !high.is_empty() {
            self.flush_multi(OP_REG_WRITE | 0x100, &high)?;
        }
        Ok(())
    }

    fn flush_multi(&mut self, opcode: u16, pairs: &[(u16, u8)]) -> Result<()> {
        let mut payload = Vec::with_capacity(pairs.len() * 2);
        for &(addr, value) in pairs {
            payload.push((addr & 0xff) as u8);
            payload.push(value);
        }
        self.transport
            .control_out(opcode, 0x0000, &payload)
            .map_err(|e| Error::RegisterAccess {
                addr: pairs[0].0,
                op: RegisterOp::Write,
                source: Box::new(e),
            })?;
        for &(addr, value) in pairs {
            self.shadow[addr as usize] = Some(value);
        }
        Ok(())
    }

    /// Write a multi-bit field within one register, in a single transfer.
    ///
    /// Mirrors `CMDASIC_WriteRegisterBit(nAddr, StartBit, Bitn, iData)` in
    /// `asiccmd.c`: it builds the masked value from the shadow in host memory
    /// and issues exactly ONE register write. That single-transfer property is
    /// the point of this method — composing a field from repeated
    /// [`write_bit`](Self::write_bit) calls would put intermediate field values
    /// on the wire, and on registers like the motor Vref at `0x3b` those
    /// intermediates are physically meaningful.
    ///
    /// `value` occupies `width` bits starting at `start_bit`, counted from the
    /// least significant bit. Bits outside the field keep their shadow value.
    ///
    /// Unlike the vendor, which validates neither, this rejects a field that
    /// does not fit in 8 bits and a value too wide for its field. It also
    /// rejects `width == 0`, where the vendor's mask loop degenerates to
    /// `mask = 0` and ORs the value in without clearing anything.
    ///
    /// **Caller obligation:** if the device powered up with a non-zero value at
    /// `addr`, call [`write`](Self::write) with the register's true value before
    /// the first field write to it. Bits never written are taken as 0, so they
    /// would otherwise be silently zeroed, clobbering real device state.
    pub fn write_field(&mut self, addr: u16, start_bit: u8, width: u8, value: u8) -> Result<()> {
        Self::check(addr)?;
        if width == 0 || u16::from(start_bit) + u16::from(width) > 8 {
            return Err(Error::InvalidBitField { start_bit, width });
        }
        if width < 8 && value >= (1u8 << width) {
            return Err(Error::FieldValueTooWide { value, width });
        }
        let mask: u8 = if width == 8 {
            0xff
        } else {
            ((1u8 << width) - 1) << start_bit
        };
        let base = self.shadow(addr).unwrap_or(0);
        let next = (base & !mask) | (value << start_bit);
        self.write(addr, next)
    }

    /// Set or clear a single bit, using the shadow as the base value.
    ///
    /// A one-bit [`write_field`](Self::write_field); see it for the shadow
    /// semantics and the caller obligation around power-on state. `bit` must be
    /// less than 8, otherwise this returns [`Error::InvalidBitField`].
    pub fn write_bit(&mut self, addr: u16, bit: u8, set: bool) -> Result<()> {
        self.write_field(addr, bit, 1, u8::from(set))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transport::{Call, MockTransport};

    #[test]
    fn write_field_issues_exactly_one_transfer() {
        // The whole point of mirroring CMDASIC_WriteRegisterBit: a 3-bit field
        // is ONE write, not three, so no intermediate field value reaches the
        // device. On registers like the motor Vref at 0x3b those intermediates
        // are physically meaningful.
        let mut r = Registers::new(MockTransport::new());
        r.write_field(0x003b, 2, 3, 0b101).unwrap();
        assert_eq!(
            r.transport().calls(),
            vec![Call::ControlOut {
                value: 0x0083,
                index: 0x0000,
                data: vec![0x3b, 0b0001_0100]
            }]
        );
    }

    #[test]
    fn write_field_preserves_bits_outside_the_field() {
        let mut r = Registers::new(MockTransport::new());
        r.write(0x0010, 0b1100_0011).unwrap();
        r.write_field(0x0010, 2, 3, 0b010).unwrap();
        // bits 7..5 and 1..0 keep their prior values; bits 4..2 become 010.
        assert_eq!(r.shadow(0x0010), Some(0b1100_1011));
        assert_eq!(r.transport().calls().len(), 2, "one write, one field write");
    }

    #[test]
    fn write_field_can_write_a_whole_register() {
        let mut r = Registers::new(MockTransport::new());
        r.write_field(0x0010, 0, 8, 0xa5).unwrap();
        assert_eq!(r.shadow(0x0010), Some(0xa5));
        assert_eq!(
            r.transport().calls().len(),
            1,
            "a full-width field is still one transfer"
        );
    }

    #[test]
    fn write_field_rejects_a_field_that_does_not_fit() {
        let mut r = Registers::new(MockTransport::new());
        assert!(matches!(
            r.write_field(0x0010, 6, 3, 0).unwrap_err(),
            Error::InvalidBitField {
                start_bit: 6,
                width: 3
            }
        ));
        assert!(matches!(
            r.write_field(0x0010, 0, 0, 0).unwrap_err(),
            Error::InvalidBitField {
                start_bit: 0,
                width: 0
            }
        ));
        assert!(
            r.transport().calls().is_empty(),
            "no transfer should be attempted"
        );
    }

    #[test]
    fn write_field_rejects_a_value_wider_than_its_field() {
        // The vendor shifts iData without masking, so an oversized value
        // silently corrupts neighbouring bits. We reject it.
        let mut r = Registers::new(MockTransport::new());
        assert!(matches!(
            r.write_field(0x0010, 1, 2, 0b100).unwrap_err(),
            Error::FieldValueTooWide {
                value: 0b100,
                width: 2
            }
        ));
        assert!(
            r.transport().calls().is_empty(),
            "no transfer should be attempted"
        );
    }

    #[test]
    fn write_bit_beyond_the_register_width_is_now_rejected_not_a_panic() {
        let mut r = Registers::new(MockTransport::new());
        assert!(matches!(
            r.write_bit(0x0010, 8, true).unwrap_err(),
            Error::InvalidBitField {
                start_bit: 8,
                width: 1
            }
        ));
    }

    #[test]
    fn write_encodes_low_address_in_the_payload() {
        let mut r = Registers::new(MockTransport::new());
        r.write(0x00a3, 0x5c).unwrap();
        assert_eq!(
            r.transport().calls(),
            vec![Call::ControlOut {
                value: 0x0083,
                index: 0x0000,
                data: vec![0xa3, 0x5c]
            }]
        );
    }

    #[test]
    fn write_folds_address_bit_8_into_the_opcode_high_byte() {
        let mut r = Registers::new(MockTransport::new());
        r.write(0x01a3, 0x5c).unwrap();
        assert_eq!(
            r.transport().calls(),
            vec![Call::ControlOut {
                value: 0x0183,
                index: 0x0000,
                data: vec![0xa3, 0x5c]
            }]
        );
    }

    #[test]
    fn read_encodes_address_in_windex_and_returns_byte_zero() {
        let t = MockTransport::new();
        t.push_control_in(&[0x5c, 0x55]);
        let r = Registers::new(t);
        assert_eq!(r.read(0x01a3).unwrap(), 0x5c);
        assert_eq!(
            r.transport().calls(),
            vec![Call::ControlIn {
                value: 0x018e,
                index: 0xa322,
                len: 2
            }]
        );
    }

    #[test]
    fn read_rejects_a_reply_without_the_0x55_magic() {
        let t = MockTransport::new();
        t.push_control_in(&[0x5c, 0x00]);
        let r = Registers::new(t);
        let err = r.read(0x00a3).unwrap_err();
        assert!(matches!(
            err,
            Error::BadRegisterMagic {
                addr: 0x00a3,
                magic: 0x00
            }
        ));
    }

    #[test]
    fn addresses_above_the_9_bit_space_are_rejected() {
        let mut r = Registers::new(MockTransport::new());
        assert!(matches!(
            r.write(0x0200, 0).unwrap_err(),
            Error::RegisterAddrOutOfRange(0x0200)
        ));
        assert!(matches!(
            r.read(0x0200).unwrap_err(),
            Error::RegisterAddrOutOfRange(0x0200)
        ));
        assert!(
            r.transport().calls().is_empty(),
            "no transfer should be attempted"
        );
    }

    #[test]
    fn shadow_is_only_updated_after_a_successful_write() {
        let mut r = Registers::new(MockTransport::new());
        assert_eq!(r.shadow(0x00a3), None);
        r.write(0x00a3, 0x5c).unwrap();
        assert_eq!(r.shadow(0x00a3), Some(0x5c));
    }

    #[test]
    fn transport_mut_grants_mutable_access_to_the_underlying_transport() {
        // This is what makes the timeout reachable from Device::set_timeout:
        // Registers previously only exposed an immutable transport() borrow.
        let mut r = Registers::new(MockTransport::new());
        r.transport_mut()
            .control_out(0x0083, 0x0000, &[0x00])
            .unwrap();
        assert_eq!(r.transport().calls().len(), 1);
    }

    #[test]
    fn a_failed_read_names_the_address_and_direction() {
        // No response queued -> MockTransport's control_in reports a short
        // transfer. Registers::read must wrap it with the address and "read".
        let r = Registers::new(MockTransport::new());
        let err = r.read(0x01a3).unwrap_err();
        match &err {
            Error::RegisterAccess { addr, op, source } => {
                assert_eq!(*addr, 0x01a3);
                assert_eq!(*op, crate::error::RegisterOp::Read);
                assert!(matches!(**source, Error::ShortTransfer { .. }));
            }
            other => panic!("expected RegisterAccess, got {other:?}"),
        }
        let s = err.to_string();
        assert!(s.contains("0x1a3") && s.contains("read"), "got: {s}");
    }

    #[test]
    fn write_bit_uses_the_shadow_rather_than_reading_back() {
        let mut r = Registers::new(MockTransport::new());
        r.write(0x0010, 0b0000_0001).unwrap();
        r.write_bit(0x0010, 3, true).unwrap();

        assert_eq!(r.shadow(0x0010), Some(0b0000_1001));
        assert_eq!(
            r.transport().calls(),
            vec![
                Call::ControlOut {
                    value: 0x0083,
                    index: 0x0000,
                    data: vec![0x10, 0b0000_0001]
                },
                Call::ControlOut {
                    value: 0x0083,
                    index: 0x0000,
                    data: vec![0x10, 0b0000_1001]
                },
            ],
            "write_bit must not issue a control_in"
        );
    }
    #[test]
    fn read_u32_uses_a_five_byte_reply_and_assembles_big_endian() {
        let t = MockTransport::new();
        t.push_control_in(&[0x01, 0x02, 0x03, 0x04, 0x55]);
        let regs = Registers::new(t);
        let v = regs.read_u32(0x102).unwrap();
        assert_eq!(v, 0x0102_0304);
        assert_eq!(
            regs.transport().calls(),
            vec![Call::ControlIn {
                value: 0x018e,
                index: 0x0222,
                len: 5
            }]
        );
    }

    #[test]
    fn read_u32_rejects_a_reply_without_the_magic_in_byte_four() {
        let t = MockTransport::new();
        t.push_control_in(&[0, 0, 0, 0, 0x00]);
        let regs = Registers::new(t);
        let err = regs.read_u32(0x102).unwrap_err();
        assert!(matches!(
            err,
            Error::BadRegisterMagic {
                addr: 0x102,
                magic: 0x00
            }
        ));
    }

    #[test]
    fn write_multi_batches_pairs_by_address_class_in_one_transfer_each() {
        let mut regs = Registers::new(MockTransport::new());
        regs.write_multi(&[(0x0010, 0xaa), (0x0105, 0xbb), (0x0011, 0xcc)])
            .unwrap();
        assert_eq!(
            regs.transport().calls(),
            vec![
                Call::ControlOut {
                    value: 0x0083,
                    index: 0,
                    data: vec![0x10, 0xaa, 0x11, 0xcc]
                },
                Call::ControlOut {
                    value: 0x0183,
                    index: 0,
                    data: vec![0x05, 0xbb]
                },
            ]
        );
        assert_eq!(regs.shadow(0x0010), Some(0xaa));
        assert_eq!(regs.shadow(0x0105), Some(0xbb));
        assert_eq!(regs.shadow(0x0011), Some(0xcc));
    }

    #[test]
    fn write_multi_flushes_every_32_pairs() {
        let mut regs = Registers::new(MockTransport::new());
        let pairs: Vec<(u16, u8)> = (0..33u16).map(|i| (i, i as u8)).collect();
        regs.write_multi(&pairs).unwrap();
        let calls = regs.transport().calls();
        assert_eq!(calls.len(), 2);
        match (&calls[0], &calls[1]) {
            (Call::ControlOut { data: a, .. }, Call::ControlOut { data: b, .. }) => {
                assert_eq!(a.len(), 64);
                assert_eq!(b, &vec![32, 32]);
            }
            other => panic!("unexpected calls {other:?}"),
        }
    }

    #[test]
    fn write_multi_rejects_out_of_range_addresses_before_sending_anything() {
        let mut regs = Registers::new(MockTransport::new());
        let err = regs.write_multi(&[(0x0010, 1), (0x0200, 2)]).unwrap_err();
        assert!(matches!(err, Error::RegisterAddrOutOfRange(0x200)));
        assert!(regs.transport().calls().is_empty());
        assert_eq!(regs.shadow(0x0010), None);
    }

    #[test]
    fn refresh_reads_the_device_and_seeds_the_shadow() {
        let t = MockTransport::new();
        t.push_control_in(&[0xe0, 0x55]);
        let mut regs = Registers::new(t);
        assert_eq!(regs.shadow(0x32), None);
        assert_eq!(regs.refresh(0x32).unwrap(), 0xe0);
        assert_eq!(regs.shadow(0x32), Some(0xe0));
    }
}
