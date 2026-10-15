//! Motor experiment: one standalone move, then report timing and status.
//! Run with: cargo run -p vf3240-core --example motor_probe -- <away|home> <steps> <pps> [unit]
use std::time::{Duration, Instant};
use vf3240_core::motor::{self, Direction};
use vf3240_core::{asic, lamp, power, Device};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let dir = match a.first().map(String::as_str) {
        Some("away") => Direction::Away,
        Some("home") => Direction::Home,
        _ => return Err("usage: motor_probe <away|home> <steps> <pps> [unit]".into()),
    };
    let steps: u32 = a[1].parse()?;
    let pps: f32 = a[2].parse()?;
    let unit: u32 = a.get(3).map(|s| s.parse()).transpose()?.unwrap_or(2);
    let mut dev = Device::open()?;
    power::set_power_saving(dev.regs(), false)?;
    let regs = dev.regs();
    let st = regs.read(0x101)?;
    eprintln!(
        "before: status 0x{st:02x} moving={} home={}",
        st & 1 != 0,
        st & 8 != 0
    );
    asic::initiate(regs, 300, 300)?;
    lamp::switch(regs, false)?;
    let t = Instant::now();
    motor::move_motor2(regs, pps, steps, dir, unit)?;
    let mut last = 0xff;
    loop {
        let s = regs.read(0x101)?;
        if s != last {
            eprintln!("  t={:?} status 0x{s:02x}", t.elapsed());
            last = s;
        }
        if s & 1 == 0 && t.elapsed() > Duration::from_millis(50) {
            break;
        }
        if t.elapsed() > Duration::from_secs(30) {
            eprintln!("still moving after 30 s, giving up");
            break;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    let st = regs.read(0x101)?;
    eprintln!(
        "after: {:?} status 0x{st:02x} moving={} home={}",
        t.elapsed(),
        st & 1 != 0,
        st & 8 != 0
    );
    Ok(())
}
