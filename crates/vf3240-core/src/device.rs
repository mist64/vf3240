//! Finding, opening and claiming a VF3240.
//!
//! The sequence mirrors `CMDIO_OpenDevice` in `usbio.c`; see
//! `docs/protocol/03-device-open.md`.

use crate::registers::Registers;
use crate::transport::Transport;
use crate::usb::UsbTransport;
use crate::{Error, Result};
use std::time::Duration;

/// From `etc/udev/rules.d/viisan_sane_vf3240.rules` and `CMDIO_OpenDevice`.
pub const VENDOR_ID: u16 = 0x2e5a;
/// From `etc/udev/rules.d/viisan_sane_vf3240.rules` and `CMDIO_OpenDevice`.
pub const PRODUCT_ID: u16 = 0xd011;

/// Opcode for the connection-type probe. From `CMDIO_GetConnectionType`.
const OP_CONNECTION_TYPE: u16 = 0x008e;

/// Opaque link flags reported by the device.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConnectionFlags {
    /// Raw byte as returned by the device.
    pub raw: u8,
}

impl ConnectionFlags {
    /// Whether control writes should be retried and status-polled.
    ///
    /// The physical meaning of bit 3 is unknown — plausibly a link-speed or
    /// bus-type indicator. We model only the behaviour it selects.
    ///
    /// **This flag is currently informational only.** Per
    /// `docs/protocol/03-device-open.md`, when the vendor driver sees this
    /// condition it retries a dropped control write up to 3 times, polling
    /// `CMDIO_GetCtrlWriteStatus` (opcode `0x8E`, `wIndex = 0x20`, success
    /// iff the returned byte is `0x55`) between attempts. This driver does
    /// **not** implement that retry/verify loop: `Registers::write` issues a
    /// single control write and trusts the libusb return code, with no
    /// read-back verification. So when `needs_write_retry()` is `true` and
    /// the device silently drops a write, `Registers::write` still returns
    /// `Ok(())`. The retry path itself is deliberately out of scope here.
    pub fn needs_write_retry(&self) -> bool {
        self.raw & 0x08 == 0
    }
}

/// An open, claimed VF3240.
pub struct Device {
    regs: Registers<UsbTransport>,
    interface: u8,
    connection_flags: ConnectionFlags,
}

impl Device {
    /// Find, open and claim the first attached VF3240.
    ///
    /// Mirrors `CMDIO_OpenDevice`'s sequence (`docs/protocol/03-device-open.md`):
    /// enumerate devices looking for a VID/PID match, open/configure/claim
    /// it, and finish with the connection-type probe that
    /// `CMDIO_GetDeviceFeatures` runs as the last step of the open path.
    ///
    /// Devices are enumerated manually (rather than via
    /// `rusb::open_device_with_vid_pid`) so that a real open failure — most
    /// notably `LIBUSB_ERROR_ACCESS`, the case a held-open scanner (macOS
    /// Image Capture) or a missing udev rule on Linux produces — is
    /// propagated to the caller instead of being collapsed into
    /// `Error::DeviceNotFound`. `DeviceNotFound` is returned only when no
    /// descriptor with VID `0x2E5A`/PID `0xD011` was seen at all.
    pub fn open() -> Result<Self> {
        let mut handle = None;
        for device in rusb::devices()?.iter() {
            let desc = device.device_descriptor()?;
            if desc.vendor_id() == VENDOR_ID && desc.product_id() == PRODUCT_ID {
                handle = Some(device.open()?);
                break;
            }
        }
        let handle = handle.ok_or(Error::DeviceNotFound)?;

        // Linux only; harmless elsewhere. Prevents the kernel scanner driver
        // from holding the interface, which is the -6 (BUSY) case the vendor
        // driver merely warns about.
        let _ = handle.set_auto_detach_kernel_driver(true);

        let config = handle.device().config_descriptor(0)?;
        let config_number = config.number();
        let interface = config
            .interfaces()
            .next()
            .ok_or(Error::NoInterface)?
            .number();
        drop(config);

        // SET_CONFIGURATION resets the device's endpoint state, which would
        // break a scan another process is running, so only send it when the
        // device is not already configured. The vendor always sends it and
        // tolerates BUSY; so do we for the remaining case.
        if handle.active_configuration().ok() != Some(config_number) {
            match handle.set_active_configuration(config_number) {
                Ok(()) | Err(rusb::Error::Busy) => {}
                Err(e) => return Err(e.into()),
            }
        }

        handle.claim_interface(interface).map_err(|e| match e {
            rusb::Error::Busy | rusb::Error::Access => Error::DeviceBusy(e),
            other => Error::Usb(other),
        })?;

        let regs = Registers::new(UsbTransport::from_handle(handle));
        let connection_flags = Self::probe_connection_flags(&regs)?;

        Ok(Self {
            regs,
            interface,
            connection_flags,
        })
    }

    /// The claimed interface number.
    pub fn interface(&self) -> u8 {
        self.interface
    }

    /// Mutable register file access.
    pub fn regs(&mut self) -> &mut Registers<UsbTransport> {
        &mut self.regs
    }

    /// Read-only register file access, for callers that only need `read`
    /// (which already only requires `&self`) without taking `&mut Device`.
    pub fn registers(&self) -> &Registers<UsbTransport> {
        &self.regs
    }

    /// Override the per-transfer USB timeout for subsequent register and
    /// bulk operations, e.g. to raise it for a long-running scan phase.
    pub fn set_timeout(&mut self, timeout: Duration) {
        self.regs.transport_mut().set_timeout(timeout);
    }

    /// The device's link flags, probed once during [`Device::open`] (the
    /// connection-type probe that closes the open sequence, mirroring
    /// `CMDIO_GetDeviceFeatures` -> `CMDIO_GetConnectionType`).
    /// Enable or disable the scanner's idle power-down
    /// (`docs/protocol/08-power-saving.md`).
    pub fn set_power_saving(&mut self, enable: bool) -> Result<()> {
        crate::power::set_power_saving(&mut self.regs, enable)
    }

    pub fn connection_flags(&self) -> Result<ConnectionFlags> {
        Ok(self.connection_flags)
    }

    /// Issue the connection-type probe (opcode `0x8E`, `wIndex = 0`) used
    /// both at open time and by [`Device::connection_flags`]'s cache fill.
    fn probe_connection_flags(regs: &Registers<UsbTransport>) -> Result<ConnectionFlags> {
        let mut buf = [0u8; 1];
        regs.transport()
            .control_in(OP_CONNECTION_TYPE, 0x0000, &mut buf)?;
        Ok(ConnectionFlags { raw: buf[0] })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ids_match_the_udev_rule_shipped_by_the_vendor() {
        assert_eq!(VENDOR_ID, 0x2e5a);
        assert_eq!(PRODUCT_ID, 0xd011);
    }

    #[test]
    fn connection_flags_decode_bit_3_as_the_retry_selector() {
        // From CMDIO_GetConnectionType: type = (byte & 8) == 0, and
        // CMDIO_WriteCommand retries when that type is 1.
        assert!(ConnectionFlags { raw: 0b0000_0000 }.needs_write_retry());
        assert!(!ConnectionFlags { raw: 0b0000_1000 }.needs_write_retry());
    }

    #[test]
    fn connection_flags_is_copy_so_device_open_can_cache_it_by_value() {
        // Device::open probes this once and stores it on the struct;
        // connection_flags() then just returns the cached copy. That only
        // works cheaply if ConnectionFlags is Copy — pin that down here
        // since Device itself can't be constructed without hardware.
        let flags = ConnectionFlags { raw: 0x08 };
        let cached = flags;
        assert_eq!(flags, cached);
        assert_eq!(flags.raw, cached.raw);
    }

    // Device::open, Device::registers, and Device::connection_flags' caching
    // all require a live rusb::DeviceHandle and so cannot be exercised
    // without hardware — see tests/hardware.rs, which is gated behind the
    // `hardware` feature and covers those paths when a VF3240 is attached.
}
