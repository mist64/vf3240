//! Error type for the VF3240 core.

/// Which direction a register access was, for [`Error::RegisterAccess`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RegisterOp {
    Read,
    Write,
}

impl std::fmt::Display for RegisterOp {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RegisterOp::Read => write!(f, "read"),
            RegisterOp::Write => write!(f, "write"),
        }
    }
}

/// Result alias used throughout the crate.
pub type Result<T> = std::result::Result<T, Error>;

/// Everything that can go wrong talking to a VF3240.
#[derive(Debug, thiserror::Error)]
pub enum Error {
    /// The underlying libusb call failed.
    #[error("usb error: {0}")]
    Usb(#[from] rusb::Error),

    /// A register read or write failed at the transport layer.
    ///
    /// `Registers::read`/`write` attach this context so a stall or short
    /// transfer names the register address and direction it happened on,
    /// rather than surfacing as a bare "usb error: Pipe error" with no way
    /// to tell which register operation the ASIC rejected. The wrapped
    /// `source` is the underlying error as-is (e.g. `Error::Usb`,
    /// including `rusb::Error::Timeout` — timeouts are not conflated with
    /// protocol errors here, just given the same address/direction context).
    #[error("register {op} at 0x{addr:03x} failed: {source}")]
    RegisterAccess {
        addr: u16,
        op: RegisterOp,
        #[source]
        source: Box<Error>,
    },

    /// No device with VID:PID 2e5a:d011 is attached.
    #[error("no VF3240 found (expected USB 2e5a:d011)")]
    DeviceNotFound,

    /// The device's configuration descriptor exposes no interface to claim.
    #[error("device exposes no USB interface")]
    NoInterface,
    /// Another driver holds the scanner. On macOS that is the vendor's ICA
    /// driver, which claims the device as soon as it is plugged in; it has
    /// to be moved out of `/Library/Image Capture/Devices`. On Linux it is
    /// usually a permissions problem (udev rule or root); on Windows the
    /// device must be bound to WinUSB.
    #[error("cannot claim the scanner ({0}): another driver has it (on macOS move the vendor driver out of /Library/Image Capture/Devices), or access is denied (Linux: udev rule or root; Windows: bind WinUSB)")]
    DeviceBusy(rusb::Error),

    /// A transfer moved fewer bytes than requested.
    ///
    /// The vendor driver ignores this condition; we treat it as an error.
    #[error("short transfer: expected {expected} bytes, moved {actual}")]
    ShortTransfer { expected: usize, actual: usize },

    /// A register read came back without the expected validity magic.
    ///
    /// From `CMDASIC_ReadRegister` in `asiccmd.c`: byte 1 of the 2-byte reply
    /// must be `0x55`, otherwise the read is rejected.
    #[error("register read at 0x{addr:03x} returned magic 0x{magic:02x}, expected 0x55")]
    BadRegisterMagic { addr: u16, magic: u8 },

    /// A register address exceeded the 9-bit address space.
    #[error("register address 0x{0:x} is outside the 9-bit range (max 0x1ff)")]
    RegisterAddrOutOfRange(u16),

    /// A bit-field specification does not fit inside an 8-bit register.
    ///
    /// From `CMDASIC_WriteRegisterBit` in `asiccmd.c`, which takes a start bit
    /// and a width. The vendor validates neither; we do.
    #[error("bit field start={start_bit} width={width} does not fit in an 8-bit register")]
    InvalidBitField { start_bit: u8, width: u8 },

    /// A status poll exceeded its time budget.
    ///
    /// The vendor's equivalent loops use a 60 s budget measured in CPU time
    /// (`docs/protocol/07-readout.md` §1.2); ours is wall-clock.
    #[error("timed out {0}")]
    Timeout(&'static str),

    /// A requested motor speed exceeds the acceleration curve's top entry.
    ///
    /// `CtlMotor_LoadPPSCurveToScanTable` refuses the same condition.
    #[error("motor speed {0} pps exceeds the acceleration curve")]
    MotorSpeedOutOfRange(f32),

    /// A requested configuration is outside what has been transcribed from the vendor.
    #[error("unsupported: {0}")]
    Unsupported(&'static str),

    /// A bulk transfer length does not fit the 32-bit length field.
    ///
    /// From `CMDASIC_SetupBulkLength` in `asiccmd.c`, whose length is a `uint`.
    #[error("bulk transfer of {0} bytes exceeds the 32-bit length field")]
    BulkTooLong(usize),

    /// A field value has more significant bits than the field is wide.
    ///
    /// The vendor's `CMDASIC_WriteRegisterBit` shifts `iData` without masking
    /// it, so an oversized value silently corrupts neighbouring bits. We reject
    /// it instead.
    #[error("value 0x{value:02x} does not fit in a {width}-bit field")]
    FieldValueTooWide { value: u8, width: u8 },
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bad_magic_error_names_the_address_and_the_magic() {
        let e = Error::BadRegisterMagic {
            addr: 0x1a3,
            magic: 0x00,
        };
        let s = e.to_string();
        assert!(s.contains("0x1a3"), "message should name the address: {s}");
        assert!(s.contains("0x00"), "message should name the bad magic: {s}");
        assert!(
            s.contains("0x55"),
            "message should name the expected magic: {s}"
        );
    }

    #[test]
    fn short_transfer_error_reports_both_counts() {
        let e = Error::ShortTransfer {
            expected: 512,
            actual: 130,
        };
        let s = e.to_string();
        assert!(s.contains("512") && s.contains("130"), "got: {s}");
    }

    #[test]
    fn register_access_error_names_the_address_direction_and_source() {
        let e = Error::RegisterAccess {
            addr: 0x1a3,
            op: RegisterOp::Write,
            source: Box::new(Error::ShortTransfer {
                expected: 2,
                actual: 0,
            }),
        };
        let s = e.to_string();
        assert!(s.contains("0x1a3"), "should name the address: {s}");
        assert!(s.contains("write"), "should name the direction: {s}");
        assert!(
            s.contains("expected 2"),
            "should include the underlying error: {s}"
        );
    }

    #[test]
    fn register_access_error_distinguishes_read_from_write() {
        let read = Error::RegisterAccess {
            addr: 0x001,
            op: RegisterOp::Read,
            source: Box::new(Error::ShortTransfer {
                expected: 2,
                actual: 1,
            }),
        };
        assert!(read.to_string().contains("read"));
    }
}
