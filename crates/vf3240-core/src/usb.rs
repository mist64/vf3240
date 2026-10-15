//! [`Transport`] over a real USB device via `rusb`.

use crate::transport::{Transport, EP_BULK_IN, EP_BULK_OUT, REQUEST_VENDOR};
use crate::{Error, Result};
use rusb::{DeviceHandle, GlobalContext};
use std::time::Duration;

/// `bmRequestType` for host-to-device vendor requests.
/// From `CMDIO_WriteCommand` in `usbio.c`.
pub const RT_VENDOR_OUT: u8 = 0x40;

/// `bmRequestType` for device-to-host vendor requests.
/// From `CMDIO_ReadCommand` in `usbio.c`.
pub const RT_VENDOR_IN: u8 = 0xc0;

/// Default per-transfer timeout.
///
/// Matches the vendor's global `libusb_timeout`, found in `.data` at vaddr
/// `0x22b2a8` of `libviisanvf3240dr.so`; it is applied to both control and
/// bulk transfers there, so we do the same here.
pub const DEFAULT_TIMEOUT: Duration = Duration::from_secs(30);

/// A [`Transport`] backed by an open `rusb` device handle.
pub struct UsbTransport {
    handle: DeviceHandle<GlobalContext>,
    timeout: Duration,
}

impl UsbTransport {
    /// Wrap an already-opened, already-claimed device handle.
    pub fn from_handle(handle: DeviceHandle<GlobalContext>) -> Self {
        Self {
            handle,
            timeout: DEFAULT_TIMEOUT,
        }
    }

    /// Override the per-transfer timeout.
    pub fn set_timeout(&mut self, timeout: Duration) {
        self.timeout = timeout;
    }

    /// Borrow the underlying handle.
    pub fn handle(&self) -> &DeviceHandle<GlobalContext> {
        &self.handle
    }
}

impl Transport for UsbTransport {
    fn control_out(&self, value: u16, index: u16, data: &[u8]) -> Result<()> {
        let n = self.handle.write_control(
            RT_VENDOR_OUT,
            REQUEST_VENDOR,
            value,
            index,
            data,
            self.timeout,
        )?;
        if n != data.len() {
            return Err(Error::ShortTransfer {
                expected: data.len(),
                actual: n,
            });
        }
        Ok(())
    }

    fn control_in(&self, value: u16, index: u16, buf: &mut [u8]) -> Result<()> {
        let want = buf.len();
        let n = self.handle.read_control(
            RT_VENDOR_IN,
            REQUEST_VENDOR,
            value,
            index,
            buf,
            self.timeout,
        )?;
        if n != want {
            return Err(Error::ShortTransfer {
                expected: want,
                actual: n,
            });
        }
        Ok(())
    }

    // Note the deliberate asymmetry between bulk_in and bulk_out, which differs
    // from the vendor driver (usbio.c):
    //
    // - bulk_out returns Error::ShortTransfer if the write is partial. The vendor's
    //   CMDIO_BulkWrite discards its trans_bytes count and silently reports success
    //   on partial writes; we treat that as an error.
    //
    // - bulk_in returns the byte count rather than erroring. A short read is
    //   legitimate at the end of a scan line, so partial reads are expected.

    fn bulk_in(&self, buf: &mut [u8]) -> Result<usize> {
        Ok(self.handle.read_bulk(EP_BULK_IN, buf, self.timeout)?)
    }

    fn bulk_out(&self, data: &[u8]) -> Result<()> {
        let n = self.handle.write_bulk(EP_BULK_OUT, data, self.timeout)?;
        if n != data.len() {
            return Err(Error::ShortTransfer {
                expected: data.len(),
                actual: n,
            });
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn request_types_match_the_vendor_driver() {
        // From CMDIO_WriteCommand / CMDIO_ReadCommand in usbio.c.
        assert_eq!(RT_VENDOR_OUT, 0x40);
        assert_eq!(RT_VENDOR_IN, 0xc0);
    }

    #[test]
    fn default_timeout_is_not_unbounded() {
        assert!(DEFAULT_TIMEOUT > Duration::ZERO);
    }

    #[test]
    fn default_timeout_matches_the_vendors_libusb_timeout_global() {
        // 30000 ms, from .data at vaddr 0x22b2a8 of libviisanvf3240dr.so.
        assert_eq!(DEFAULT_TIMEOUT, Duration::from_secs(30));
    }
}
