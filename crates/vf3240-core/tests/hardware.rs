//! Tests requiring a physically attached VF3240.
//! Run with: cargo test -p vf3240-core --features hardware -- --test-threads=1
#![cfg(feature = "hardware")]

use std::time::Duration;
use vf3240_core::Device;

#[test]
fn device_can_be_opened_and_claimed() {
    let dev = Device::open().expect("VF3240 should be attached and claimable");
    assert_eq!(dev.interface(), 0, "expected the first interface");
}

#[test]
fn connection_flags_are_readable_and_cached_across_calls() {
    let dev = Device::open().unwrap();
    let flags = dev
        .connection_flags()
        .expect("connection probe should succeed");
    println!(
        "connection flags: 0x{:02x} (retry={})",
        flags.raw,
        flags.needs_write_retry()
    );

    // connection_flags() is now a cache read filled once by open() (finding
    // 3), not a fresh control transfer each call — it should be stable.
    let flags_again = dev.connection_flags().unwrap();
    assert_eq!(
        flags, flags_again,
        "connection_flags() should return the cached value"
    );
}

#[test]
fn registers_accessor_allows_reads_without_a_mutable_device() {
    let dev = Device::open().unwrap();
    // registers() takes &self, unlike regs()'s &mut self — reads shouldn't
    // require exclusive access (finding 5).
    let _ = dev.registers().read(0x0000);
}

#[test]
fn set_timeout_is_reachable_from_device() {
    let mut dev = Device::open().unwrap();
    // Reachability is the point here (finding 2): this used to be dead API
    // because nothing exposed &mut UsbTransport once a Device existed.
    dev.set_timeout(Duration::from_secs(5));
    let _ = dev.regs().read(0x0000);
}

#[test]
fn every_register_read_returns_the_0x55_magic() {
    let mut dev = Device::open().unwrap();
    let mut ok = 0usize;
    let mut bad = Vec::new();
    for addr in 0..=vf3240_core::REG_ADDR_MAX {
        match dev.regs().read(addr) {
            Ok(_) => ok += 1,
            Err(e) => bad.push((addr, e.to_string())),
        }
    }
    println!("{ok} registers read cleanly, {} failed", bad.len());
    for (addr, e) in bad.iter().take(20) {
        println!("  0x{addr:03x}: {e}");
    }
    assert!(
        ok > 0,
        "no register was readable — the read encoding is wrong"
    );
}

#[test]
fn power_saving_can_be_disabled_and_reads_back() {
    let mut dev = Device::open().unwrap();
    dev.set_power_saving(false)
        .expect("power-saver bit writes should succeed");
    let r32 = dev.regs().read(0x32).unwrap();
    let r35 = dev.regs().read(0x35).unwrap();
    println!("after disable: 0x32=0x{r32:02x} 0x35=0x{r35:02x}");
    assert_eq!(
        r32 & 0x20,
        0,
        "0x32 bit 5 should be clear after disabling power saving"
    );
    assert_ne!(r35 & 0x20, 0, "0x35 bit 5 should be set");
}

#[test]
fn asic_init_sequence_is_accepted_by_the_device() {
    let mut dev = Device::open().unwrap();
    let banks = vf3240_core::asic::initiate(dev.regs(), 300, 300)
        .expect("init sequence should be accepted");
    println!("shading banks {banks:08x?}");
    // Spot-check that the table landed: 0x02 is written as 0x78 by the init table.
    let r02 = dev.regs().read(0x02).unwrap();
    let r0b = dev.regs().read(0x0b).unwrap();
    let r35 = dev.regs().read(0x35).unwrap();
    println!("after init: 0x02=0x{r02:02x} 0x0b=0x{r0b:02x} 0x35=0x{r35:02x}");
    assert_eq!(r02, 0x78, "init table value for 0x02 should read back");
}
