//! Still-mode experiment: capture N lines with the carriage motor unpowered.
//! Run with: cargo run -p vf3240-core --example still -- <lines> <out-prefix>
//! Writes <prefix>.raw (raw lines) and <prefix>.ppm (unpacked, full width).
use std::time::Duration;
use vf3240_core::{asic, lamp, power, scan, setup, unpack, Device};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let lines: u32 = args.next().as_deref().unwrap_or("16").parse()?;
    let prefix = args.next().unwrap_or_else(|| "still".into());
    let depth16 = args.next().as_deref() == Some("16");
    let mut dev = Device::open()?;
    power::set_power_saving(dev.regs(), false)?;
    let regs = dev.regs();
    eprintln!("status before: 0x{:02x}", regs.read(0x101)?);
    asic::initiate(regs, 300, 300)?;
    regs.write_bit(0x3a, 2, true)?;
    regs.write_field(0x1c, 0, 3, 0)?; // SetTBTime(1), normally done by homing
    lamp::switch(regs, true)?;
    let mut s = setup::ScanSetup::new(setup::Mode::COLOR_300, lines);
    s.still = true;
    s.depth16 = depth16;
    s.write_vref = false;
    setup::program(regs, &s)?;
    eprintln!("programmed; status 0x{:02x}", regs.read(0x101)?);
    scan::start(regs)?;
    let t = std::time::Instant::now();
    let ready = scan::wait_data_ready(regs, Duration::from_secs(10));
    eprintln!(
        "data ready: {ready:?} after {:?}; status 0x{:02x} count {}",
        t.elapsed(),
        regs.read(0x101)?,
        regs.read_u32(0x102)?
    );
    let line_bytes = s.raw_line_bytes();
    let mut raw = vec![0u8; line_bytes * lines as usize];
    let mut got = 0usize;
    for i in 0..lines as usize {
        match scan::read_line(regs, &mut raw[i * line_bytes..(i + 1) * line_bytes]) {
            Ok(()) => got += 1,
            Err(e) => {
                eprintln!("line {i}: {e}");
                break;
            }
        }
    }
    eprintln!(
        "read {got} lines in {:?}; status 0x{:02x} count {}",
        t.elapsed(),
        regs.read(0x101)?,
        regs.read_u32(0x102)?
    );
    let stop = scan::stop(regs, Duration::from_secs(5));
    eprintln!("stop: {stop:?}");
    std::fs::write(format!("{prefix}.raw"), &raw[..got * line_bytes])?;
    let geom = unpack::Geometry::for_dpi(300).unwrap();
    let w = geom.line_pixels();
    let mut rgb = Vec::new();
    for i in 0..got {
        let line = &raw[i * line_bytes..(i + 1) * line_bytes];
        if depth16 {
            // Preview the high bytes only (u16 LE → take byte 1 of each sample).
            let hi: Vec<u8> = line.chunks(2).map(|c| c[1]).collect();
            unpack::unpack_color8(&geom, &hi, 0, w, &mut rgb).unwrap();
        } else {
            unpack::unpack_color8(&geom, line, 0, w, &mut rgb).unwrap();
        }
    }
    let mut out = std::fs::File::create(format!("{prefix}.ppm"))?;
    use std::io::Write;
    write!(out, "P6\n{w} {got}\n255\n")?;
    out.write_all(&rgb)?;
    Ok(())
}
