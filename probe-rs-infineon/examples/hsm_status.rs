//! Reads what the debugger can reach of the HSM, without writing anything.
//!
//! Attaches to a target variant with the HSM as a core (`TC36x_HSM`) and prints `HSM_ID` and
//! `DBGBASE` of the HSM bridge ([`HsmStatus`], through CPU0). Then it asks the session for the
//! HSM core: while its debug window is closed the session reports it disabled; when the HSM
//! firmware enabled HSM debug it prints the core status, the PC if the core is halted and the
//! breakpoint units. That sets `DBGBASE`, the only write, and only then; the core is neither
//! halted nor run.
//!
//! ```text
//! cargo run -p probe-rs-infineon --example hsm_status -- [TC36x_HSM]
//! cargo run -p probe-rs-infineon --example hsm_status -- --read-only [TC36x]
//! ```
//!
//! `--read-only` is for a board that must not be written (an ECU): it reads `HSM_ID` and
//! `DBGBASE` through the bus and takes no core, so neither `DBGBASE` nor anything a core's
//! status check could put back is written.

use probe_rs::{
    Error, Permissions,
    architecture::tricore::bus::AddressMap,
    probe::{DebugProbeSelector, list::Lister},
};
use probe_rs_infineon::hsm::{self, HsmStatus};

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

fn word(value: Option<u32>) -> String {
    value.map_or_else(|| "bus error".to_string(), |value| format!("{value:#010x}"))
}

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(tracing_subscriber::EnvFilter::from_default_env())
        .init();
    let read_only = std::env::args().any(|a| a == "--read-only");
    let chip = std::env::args()
        .skip(1)
        .find(|a| a != "--read-only")
        .unwrap_or_else(|| if read_only { "TC36x" } else { "TC36x_HSM" }.to_string());
    probe_rs_infineon::register_plugin();
    let selector = std::env::var("PROBE_RS_PROBE").unwrap_or_else(|_| "058b:0000".to_string());
    let probe = Lister::new().open(selector.parse::<DebugProbeSelector>()?)?;
    let mut session = probe.attach(chip.as_str(), Permissions::default())?;

    if read_only {
        let bus = session.get_tricore_bus()?;
        let mut read = |address: u64| bus.read_u32(AddressMap::Target, address as u32).ok();
        let (id, dbgbase) = (read(hsm::HSM_ID), read(hsm::DBGBASE));
        let chipid = read(0xF003_6140);
        println!("CHIPID  {}", word(chipid));
        println!("HSM_ID  {}", word(id));
        println!("DBGBASE {}", word(dbgbase));
        return Ok(());
    }

    let status = HsmStatus::read(&mut session.core(0)?);
    println!("HSM_ID  {}", word(status.id));
    println!("DBGBASE {}", word(status.dbgbase));

    let Some(hsm) = session
        .target()
        .cores
        .iter()
        .position(|core| core.name == "hsm")
    else {
        println!("{chip} has no HSM core");
        return Ok(());
    };
    match session.core(hsm) {
        Err(Error::CoreDisabled(_)) => {
            println!("core {hsm} (hsm): disabled, the HSM debug window is closed");
        }
        Err(error) => println!("core {hsm} (hsm): {error}"),
        Ok(mut core) => {
            let status = core.status()?;
            println!("core {hsm} (hsm): {status:?}");
            if status.is_halted() {
                let pc: u32 = core.read_core_reg(core.program_counter().id())?;
                println!("PC      {pc:#010x}");
            }
            println!("breakpoint units: {}", core.available_breakpoint_units()?);
        }
    }
    // The TriCore cores are not affected by the HSM core.
    println!("cpu0: {:?}", session.core(0)?.status()?);
    Ok(())
}
