//! Shows which PFLASH sectors hold data (read only): for each logical sector, the number of
//! written (readable) pages. Runs of erased sectors are printed as one line.
//!
//! ```text
//! cargo run -p probe-rs-infineon --example pflash_usage
//! ```
//!
//! The chip is detected; `PROBE_RS_PROBE` selects another probe.

use probe_rs::{Permissions, probe::list::Lister};
use probe_rs_infineon::flash::{FlashDevice, PAGE, read_pages};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    probe_rs_infineon::register_plugin();
    let selector = std::env::var("PROBE_RS_PROBE").unwrap_or_else(|_| "058b:0000".to_string());
    let probe = Lister::new().open(selector.parse::<probe_rs::probe::DebugProbeSelector>()?)?;
    let mut session = probe.attach(
        probe_rs::config::TargetSelector::Auto,
        Permissions::default(),
    )?;
    let chip = session.target().name.clone();
    let device = FlashDevice::for_chip(&chip).ok_or("no PFLASH map for this chip")?;
    println!("{chip}: {} logical sectors", device.sectors().len());

    let mut core = session.core(0)?;
    let mut erased_run: Option<(u32, u32)> = None;
    let flush = |run: &mut Option<(u32, u32)>| {
        if let Some((start, end)) = run.take() {
            println!("{start:#010x}..{end:#010x}  erased");
        }
    };
    for sector in device.sectors() {
        let pages = read_pages(&mut core, sector.base, sector.size as usize)?;
        let written = pages.iter().filter(|p| p.is_some()).count();
        if written == 0 {
            match &mut erased_run {
                Some((_, end)) if *end == sector.base => *end += sector.size,
                _ => {
                    flush(&mut erased_run);
                    erased_run = Some((sector.base, sector.base + sector.size));
                }
            }
            continue;
        }
        flush(&mut erased_run);
        println!(
            "{:#010x}  {written:4}/{} pages written",
            sector.base,
            sector.size as usize / PAGE
        );
    }
    flush(&mut erased_run);
    Ok(())
}
