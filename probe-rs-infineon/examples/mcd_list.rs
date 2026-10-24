//! Loads an MCD library and prints the systems, devices and cores its server offers, with the
//! memory spaces of every core and the configuration keys that pick them. It opens each core
//! for a moment but does not halt or write anything.
//!
//! ```text
//! cargo run -p probe-rs-infineon --features mcd --example mcd_list -- "C:\Program Files\DAS64\clients\mcdxdas.dll[|<config>]"
//! ```
//!
//! Without an argument it takes the library from `PROBE_RS_MCD_DLL`.

use probe_rs_infineon::mcd::{CORE_KEY, DEVICE_KEY, McdEndpoint, McdProbe, SYSTEM_KEY};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt()
        .with_env_filter(tracing_subscriber::EnvFilter::from_default_env())
        .init();
    let serial = std::env::args().nth(1).unwrap_or_default();
    let endpoint = McdEndpoint::parse(&serial, |name| std::env::var(name).ok())?;
    println!("library {}", endpoint.library.display());
    let mut probe = McdProbe::load(endpoint)?;
    let cores = probe.inventory()?;
    if cores.is_empty() {
        println!("the server offers no core");
    }
    for entry in cores {
        let (system, device, core) = entry.indices;
        println!(
            "{SYSTEM_KEY}={system} {DEVICE_KEY}={device} {CORE_KEY}={core}: '{}' of {} (JTAG ID {:#010x}) on {}, system '{}'",
            entry.core, entry.device, entry.jtag_id, entry.access_hardware, entry.system
        );
        match entry.memory_spaces {
            Ok(spaces) => {
                for space in spaces {
                    println!(
                        "    memory space {} '{}', type {:#x}{}",
                        space.id,
                        space.name,
                        space.mem_type,
                        if space.device_memory {
                            " (device memory)"
                        } else {
                            ""
                        }
                    );
                }
            }
            Err(error) => println!("    memory spaces: {error}"),
        }
    }
    Ok(())
}
