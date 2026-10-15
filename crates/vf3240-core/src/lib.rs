//! Open driver core for the VIISAN VF3240 A3 flatbed scanner (USB 2e5a:d011).
//!
//! Written from the protocol specification in `docs/protocol/`, which was
//! recovered by reverse engineering the vendor's Linux SANE driver. No vendor
//! code is linked or reproduced here.
#![forbid(unsafe_code)]

pub mod afe;
pub mod asic;
pub mod bulk;
pub mod calibration;
pub mod device;
pub mod error;
pub mod image;
pub mod lamp;
pub mod motor;
pub mod power;
pub mod registers;
pub mod scan;
pub mod setup;
pub mod tables;
pub mod transport;
pub mod unpack;
pub mod usb;

pub use device::{ConnectionFlags, Device, PRODUCT_ID, VENDOR_ID};
pub use error::{Error, RegisterOp, Result};
pub use registers::{Registers, REG_ADDR_MAX};
pub use transport::{Transport, EP_BULK_IN, EP_BULK_OUT, REQUEST_VENDOR};
pub use usb::UsbTransport;
