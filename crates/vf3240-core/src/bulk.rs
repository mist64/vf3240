//! Bulk memory transfers: a control-channel setup followed by one bulk transfer.
//!
//! See `docs/protocol/01-usb-transport.md`, "Bulk memory path".

use crate::transport::Transport;
use crate::Result;

/// Opcode of the bulk length/address setup. From `CMDASIC_SetupBulkLength` in `asiccmd.c`.
pub const OP_BULK_SETUP: u16 = 0x0082;

/// `wIndex` for a device-to-host bulk transfer (`bmode = 0`). From `CMDASIC_ReadRawImage`.
const MODE_READ: u16 = 0;
/// `wIndex` for a host-to-device bulk transfer (`bmode = 1`). From `CMDASIC_WriteBulkData`.
const MODE_WRITE: u16 = 1;

/// Address image lines are read from. From `CMDASIC_ReadRawImage`.
pub const IMAGE_ADDR: u32 = 0x1000_0000;

fn setup<T: Transport>(t: &T, addr: u32, len: usize, mode: u16) -> Result<()> {
    let len = u32::try_from(len).map_err(|_| crate::Error::BulkTooLong(len))?;
    let mut payload = [0u8; 8];
    payload[..4].copy_from_slice(&addr.to_le_bytes());
    payload[4..].copy_from_slice(&len.to_le_bytes());
    t.control_out(OP_BULK_SETUP, mode, &payload)
}

/// Read `buf.len()` bytes from device memory at `addr`.
///
/// Setup with `bmode = 0` then one bulk IN. A short read is an error; the
/// vendor ignores the transferred count and we deliberately do not.
pub fn read_block<T: Transport>(t: &T, addr: u32, buf: &mut [u8]) -> Result<()> {
    setup(t, addr, buf.len(), MODE_READ)?;
    let n = t.bulk_in(buf)?;
    if n != buf.len() {
        return Err(crate::Error::ShortTransfer {
            expected: buf.len(),
            actual: n,
        });
    }
    Ok(())
}

/// Write `data` to device memory at `addr`.
///
/// Setup with `bmode = 1` then one bulk OUT.
pub fn write_block<T: Transport>(t: &T, addr: u32, data: &[u8]) -> Result<()> {
    setup(t, addr, data.len(), MODE_WRITE)?;
    t.bulk_out(data)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transport::{Call, MockTransport};
    use crate::Error;

    #[test]
    fn read_block_sets_up_then_reads_exactly_len_bytes() {
        let t = MockTransport::new();
        t.push_bulk_in(&[1, 2, 3, 4]);
        let mut buf = [0u8; 4];
        read_block(&t, 0x1000_0000, &mut buf).unwrap();
        assert_eq!(buf, [1, 2, 3, 4]);
        assert_eq!(
            t.calls(),
            vec![
                Call::ControlOut {
                    value: 0x0082,
                    index: 0,
                    data: vec![0, 0, 0, 0x10, 4, 0, 0, 0]
                },
                Call::BulkIn { len: 4 },
            ]
        );
    }

    #[test]
    fn read_block_treats_a_short_bulk_read_as_an_error() {
        let t = MockTransport::new();
        t.push_bulk_in(&[1, 2]);
        let mut buf = [0u8; 4];
        let err = read_block(&t, 0x1000_0000, &mut buf).unwrap_err();
        assert!(matches!(
            err,
            Error::ShortTransfer {
                expected: 4,
                actual: 2
            }
        ));
    }

    #[test]
    fn write_block_sets_up_with_mode_one_then_writes() {
        let t = MockTransport::new();
        write_block(&t, 0x0100_0200, &[9, 8, 7]).unwrap();
        assert_eq!(
            t.calls(),
            vec![
                Call::ControlOut {
                    value: 0x0082,
                    index: 1,
                    data: vec![0, 2, 0, 1, 3, 0, 0, 0]
                },
                Call::BulkOut {
                    data: vec![9, 8, 7]
                },
            ]
        );
    }
}
