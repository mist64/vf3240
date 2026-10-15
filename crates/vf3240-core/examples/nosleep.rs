//! Keep-alive: open the scanner, disable power saving, exit. Run periodically
//! between scans (every ASIC init re-arms the idle timer; see 08-power-saving.md).

use vf3240_core::Device;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut dev = Device::open()?;
    dev.set_power_saving(false)?;
    let v = dev.regs().read(0x32)?;
    println!(
        "power saving disabled; 0x32 = {v:#04x} (bit 5 clear = {})",
        v & 0x20 == 0
    );
    Ok(())
}
