//! Register watch: `regwatch [seconds]`. Reads the whole 9-bit register
//! file repeatedly and prints every register whose value changes between
//! sweeps, plus the key status byte (`0x8E`/`0x21`) and any interrupt
//! packet on `0x83`. Used to find where the buttons show up.

use std::time::{Duration, Instant};
use vf3240_core::transport::Transport;
use vf3240_core::Device;

const REG_ADDR_MAX: u16 = 0x1ff;
const OP_READ: u16 = 0x008e;
const IDX_KEY: u16 = 0x0021;
const EP_INTERRUPT: u8 = 0x83;

fn sweep(dev: &mut Device) -> Vec<Option<u8>> {
    (0..=REG_ADDR_MAX)
        .map(|a| dev.regs().read(a).ok())
        .collect()
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let seconds: u64 = std::env::args()
        .nth(1)
        .and_then(|a| a.parse().ok())
        .unwrap_or(120);
    let mut dev = Device::open()?;
    dev.set_power_saving(false)?;
    let t0 = Instant::now();
    let mut last = sweep(&mut dev);
    println!(
        "baseline sweep took {:.2?}; watching for {seconds} s; press the buttons",
        t0.elapsed()
    );
    let mut last_key = None;
    let mut sweeps = 0u32;
    while t0.elapsed() < Duration::from_secs(seconds) {
        let now = sweep(&mut dev);
        sweeps += 1;
        for (addr, (a, b)) in last.iter().zip(&now).enumerate() {
            if a != b {
                println!(
                    "{:8.2?} reg {:#05x}: {:02x?} -> {:02x?}",
                    t0.elapsed(),
                    addr,
                    a,
                    b
                );
            }
        }
        last = now;
        let mut key = [0u8; 1];
        dev.regs()
            .transport()
            .control_in(OP_READ, IDX_KEY, &mut key)?;
        if last_key != Some(key[0]) {
            println!("{:8.2?} key status 0x{:02x}", t0.elapsed(), key[0]);
            last_key = Some(key[0]);
        }
        let mut pkt = [0u8; 8];
        if let Ok(n) = dev.regs().transport().handle().read_interrupt(
            EP_INTERRUPT,
            &mut pkt,
            Duration::from_millis(20),
        ) {
            println!("{:8.2?} interrupt packet {:02x?}", t0.elapsed(), &pkt[..n]);
        }
    }
    println!("done: {sweeps} sweeps");
    Ok(())
}
