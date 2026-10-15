//! Print the VF3240's configuration, interfaces and endpoints (no claim needed).

fn main() -> rusb::Result<()> {
    for dev in rusb::devices()?.iter() {
        let d = dev.device_descriptor()?;
        if (d.vendor_id(), d.product_id()) != (0x2e5a, 0xd011) {
            continue;
        }
        println!(
            "device {:04x}:{:04x} usb {} class {:#x} configs {}",
            d.vendor_id(),
            d.product_id(),
            d.usb_version(),
            d.class_code(),
            d.num_configurations()
        );
        for c in 0..d.num_configurations() {
            let cfg = dev.config_descriptor(c)?;
            println!("config {} max power {} mA", cfg.number(), cfg.max_power());
            for intf in cfg.interfaces() {
                for alt in intf.descriptors() {
                    println!(
                        "  interface {} alt {} class {:#x}/{:#x}/{:#x}",
                        alt.interface_number(),
                        alt.setting_number(),
                        alt.class_code(),
                        alt.sub_class_code(),
                        alt.protocol_code()
                    );
                    for ep in alt.endpoint_descriptors() {
                        println!(
                            "    endpoint {:#04x} {:?} {:?} max packet {} interval {}",
                            ep.address(),
                            ep.direction(),
                            ep.transfer_type(),
                            ep.max_packet_size(),
                            ep.interval()
                        );
                    }
                }
            }
        }
    }
    Ok(())
}
