//! Reads single words of the EEC address map (map 1) of a TC27x ED through the TAS bus and
//! prints the result of each: the value, or the TAS error code (0x8D for a refused access).
//! Read only; the cores are not touched. It shows which parts of the MCDS and EMEM address
//! space the adapter lets through.
//!
//! ```text
//! cargo run -p probe-rs-infineon --example eec_probe
//! ```
//!
//! `PROBE_RS_PROBE` selects another probe.

use probe_rs::{
    Permissions,
    architecture::tricore::bus::AddressMap,
    probe::{DebugProbeSelector, list::Lister},
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    probe_rs_infineon::register_plugin();
    let selector: DebugProbeSelector = std::env::var("PROBE_RS_PROBE")
        .unwrap_or_else(|_| "058b:0000".to_string())
        .parse()?;
    let mut session = Lister::new().open(selector)?.attach(
        probe_rs::config::TargetSelector::Auto,
        Permissions::default(),
    )?;
    let bus = session.get_tricore_bus()?;
    let probes: [(&str, AddressMap, u32); 10] = [
        (
            "SCU_CHIPID, map 0 (control)",
            AddressMap::Target,
            0xF003_6140,
        ),
        ("SCU_CHIPID, map 1", AddressMap::Eec, 0xF003_6140),
        ("MCDS OCS 0xF00E6004, map 1", AddressMap::Eec, 0xF00E_6004),
        ("MCDS ctrl 0xF00E6000, map 1", AddressMap::Eec, 0xF00E_6000),
        ("ED control 0xF00E0080, map 1", AddressMap::Eec, 0xF00E_0080),
        ("EMEM 0xAF000000, map 1", AddressMap::Eec, 0xAF00_0000),
        ("EMEM 0xAF00FFE0, map 1", AddressMap::Eec, 0xAF00_FFE0),
        ("EMEM 0xAF010000, map 1", AddressMap::Eec, 0xAF01_0000),
        ("EMEM 0xAF0FFFE0, map 1", AddressMap::Eec, 0xAF0F_FFE0),
        ("EMEM 0xAF00FFE0, map 0", AddressMap::Target, 0xAF00_FFE0),
    ];
    for (name, map, address) in probes {
        match bus.read_u32(map, address) {
            Ok(value) => println!("{name:<30} {address:#010x}: {value:#010x}"),
            Err(error) => println!("{name:<30} {address:#010x}: {error}"),
        }
    }
    Ok(())
}
