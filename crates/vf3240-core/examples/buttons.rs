//! Button probe: `buttons [seconds]`. Polls the key status byte (control
//! request `0x8E`, `wIndex = 0x21`, `MyScanner_GetKeyStatus`) every 50 ms and
//! reads the interrupt endpoint `0x83` between polls; prints every change and
//! every interrupt packet with a timestamp. Press the buttons while it runs.

use std::time::{Duration, Instant};
use vf3240_core::transport::Transport;
use vf3240_core::Device;

const OP_READ: u16 = 0x008e;
const IDX_KEY: u16 = 0x0021;
const EP_INTERRUPT: u8 = 0x83;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let seconds: u64 = std::env::args()
        .nth(1)
        .and_then(|a| a.parse().ok())
        .unwrap_or(60);
    let mut dev = Device::open()?;
    dev.set_power_saving(false)?;
    let t0 = Instant::now();
    let mut last: Option<u8> = None;
    let mut polls = 0u32;
    println!("polling for {seconds} s; press the buttons");
    while t0.elapsed() < Duration::from_secs(seconds) {
        let mut key = [0u8; 1];
        dev.regs()
            .transport()
            .control_in(OP_READ, IDX_KEY, &mut key)?;
        polls += 1;
        if last != Some(key[0]) {
            println!(
                "{:8.3?} key status 0x{:02x} (poll {polls})",
                t0.elapsed(),
                key[0]
            );
            last = Some(key[0]);
        }
        let mut pkt = [0u8; 8];
        match dev.regs().transport().handle().read_interrupt(
            EP_INTERRUPT,
            &mut pkt,
            Duration::from_millis(50),
        ) {
            Ok(n) => println!("{:8.3?} interrupt packet {:02x?}", t0.elapsed(), &pkt[..n]),
            Err(rusb::Error::Timeout) => {}
            Err(e) => println!("{:8.3?} interrupt error {e}", t0.elapsed()),
        }
    }
    println!("done: {polls} polls, final status {last:?}");
    Ok(())
}
