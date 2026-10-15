//! Dump the VF3240 register file. Run with: cargo run -p vf3240-core --example probe
use vf3240_core::{Device, REG_ADDR_MAX};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut dev = Device::open()?;
    let flags = dev.connection_flags()?;
    println!(
        "connection flags 0x{:02x}  needs_write_retry={}",
        flags.raw,
        flags.needs_write_retry()
    );

    for addr in 0..=REG_ADDR_MAX {
        match dev.regs().read(addr) {
            Ok(v) => println!("reg 0x{addr:03x} = 0x{v:02x}"),
            Err(e) => println!("reg 0x{addr:03x} : {e}"),
        }
    }
    Ok(())
}
