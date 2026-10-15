//! The four USB primitives the VF3240 protocol is built from.
//!
//! Everything above this layer is generic over [`Transport`], so the register
//! and device layers are unit-testable without a scanner attached.

use crate::{Error, Result};
use std::cell::RefCell;
use std::collections::VecDeque;

/// Bulk IN endpoint. From `CMDIO_BulkRead` in `usbio.c` — hardcoded, not
/// discovered from the descriptors.
pub const EP_BULK_IN: u8 = 0x81;

/// Bulk OUT endpoint. From `CMDIO_BulkWrite` in `usbio.c`.
pub const EP_BULK_OUT: u8 = 0x02;

/// The single vendor request all control traffic uses.
/// From `CMDIO_WriteCommand` / `CMDIO_ReadCommand` in `usbio.c`.
pub const REQUEST_VENDOR: u8 = 0x0c;

/// USB transport primitives.
pub trait Transport {
    /// Vendor control transfer, host to device (`bmRequestType` 0x40).
    fn control_out(&self, value: u16, index: u16, data: &[u8]) -> Result<()>;

    /// Vendor control transfer, device to host (`bmRequestType` 0xC0).
    ///
    /// Must fill `buf` completely or return [`Error::ShortTransfer`].
    fn control_in(&self, value: u16, index: u16, buf: &mut [u8]) -> Result<()>;

    /// Read from the bulk IN endpoint. Returns the byte count actually moved.
    fn bulk_in(&self, buf: &mut [u8]) -> Result<usize>;

    /// Write to the bulk OUT endpoint. Short writes are an error.
    fn bulk_out(&self, data: &[u8]) -> Result<()>;
}

/// One recorded transport operation.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Call {
    ControlOut {
        value: u16,
        index: u16,
        data: Vec<u8>,
    },
    ControlIn {
        value: u16,
        index: u16,
        len: usize,
    },
    BulkIn {
        len: usize,
    },
    BulkOut {
        data: Vec<u8>,
    },
}

/// A [`Transport`] that records calls and replays queued responses.
#[derive(Debug, Default)]
pub struct MockTransport {
    calls: RefCell<Vec<Call>>,
    control_in_queue: RefCell<VecDeque<Vec<u8>>>,
    bulk_in_queue: RefCell<VecDeque<Vec<u8>>>,
}

impl MockTransport {
    pub fn new() -> Self {
        Self::default()
    }

    /// Queue the bytes a subsequent `control_in` will return.
    pub fn push_control_in(&self, bytes: &[u8]) {
        self.control_in_queue.borrow_mut().push_back(bytes.to_vec());
    }

    /// Queue the bytes a subsequent `bulk_in` will return.
    pub fn push_bulk_in(&self, bytes: &[u8]) {
        self.bulk_in_queue.borrow_mut().push_back(bytes.to_vec());
    }

    /// Every call recorded so far, in order.
    pub fn calls(&self) -> Vec<Call> {
        self.calls.borrow().clone()
    }
}

impl Transport for MockTransport {
    fn control_out(&self, value: u16, index: u16, data: &[u8]) -> Result<()> {
        self.calls.borrow_mut().push(Call::ControlOut {
            value,
            index,
            data: data.to_vec(),
        });
        Ok(())
    }

    fn control_in(&self, value: u16, index: u16, buf: &mut [u8]) -> Result<()> {
        self.calls.borrow_mut().push(Call::ControlIn {
            value,
            index,
            len: buf.len(),
        });
        let response = self
            .control_in_queue
            .borrow_mut()
            .pop_front()
            .unwrap_or_default();
        if response.len() != buf.len() {
            return Err(Error::ShortTransfer {
                expected: buf.len(),
                actual: response.len(),
            });
        }
        buf.copy_from_slice(&response);
        Ok(())
    }

    fn bulk_in(&self, buf: &mut [u8]) -> Result<usize> {
        self.calls
            .borrow_mut()
            .push(Call::BulkIn { len: buf.len() });
        let response = self
            .bulk_in_queue
            .borrow_mut()
            .pop_front()
            .unwrap_or_default();
        let n = response.len().min(buf.len());
        buf[..n].copy_from_slice(&response[..n]);
        Ok(n)
    }

    fn bulk_out(&self, data: &[u8]) -> Result<()> {
        self.calls.borrow_mut().push(Call::BulkOut {
            data: data.to_vec(),
        });
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mock_records_control_out_verbatim() {
        let t = MockTransport::new();
        t.control_out(0x0083, 0x0000, &[0xa3, 0x5c]).unwrap();
        assert_eq!(
            t.calls(),
            vec![Call::ControlOut {
                value: 0x0083,
                index: 0x0000,
                data: vec![0xa3, 0x5c]
            }]
        );
    }

    #[test]
    fn mock_replays_queued_control_in_responses_in_order() {
        let t = MockTransport::new();
        t.push_control_in(&[0x11, 0x55]);
        t.push_control_in(&[0x22, 0x55]);

        let mut buf = [0u8; 2];
        t.control_in(0x008e, 0x0000, &mut buf).unwrap();
        assert_eq!(buf, [0x11, 0x55]);
        t.control_in(0x008e, 0x0000, &mut buf).unwrap();
        assert_eq!(buf, [0x22, 0x55]);
    }

    #[test]
    fn mock_control_in_without_a_queued_response_is_a_short_transfer() {
        let t = MockTransport::new();
        let mut buf = [0u8; 2];
        let err = t.control_in(0x008e, 0x0000, &mut buf).unwrap_err();
        assert!(matches!(
            err,
            Error::ShortTransfer {
                expected: 2,
                actual: 0
            }
        ));
    }
}
