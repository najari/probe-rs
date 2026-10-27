//! Reads what the debugger can reach of the HSM, without writing anything.
//!
//! Prints `HSM_ID` and `DBGBASE` of the HSM bridge ([`HsmStatus`]). When the HSM firmware
//! enabled HSM debug (the window is open), it also reads `FP_CTRL` and `DHCSR` through the
//! debug memory window; that sets `DBGBASE`, the only write, and only then.
//!
//! ```text
//! cargo run -p probe-rs-infineon --example hsm_status -- [TC36x|TC275]
//! ```

use probe_rs::{
    MemoryInterface, Permissions,
    probe::{DebugProbeSelector, list::Lister},
};
use probe_rs_infineon::hsm::{HsmFpb, HsmStatus, HsmWindow};

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

/// `DHCSR` of the HSM Cortex-M3.
const DHCSR: u64 = 0xE000_EDF0;

fn word(value: Option<u32>) -> String {
    value.map_or_else(|| "bus error".to_string(), |value| format!("{value:#010x}"))
}

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(tracing_subscriber::EnvFilter::from_default_env())
        .init();
    let chip = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "TC36x".to_string());
    probe_rs_infineon::register_plugin();
    let selector = std::env::var("PROBE_RS_PROBE").unwrap_or_else(|_| "058b:0000".to_string());
    let probe = Lister::new().open(selector.parse::<DebugProbeSelector>()?)?;
    let mut session = probe.attach(chip.as_str(), Permissions::default())?;
    let mut core = session.core(0)?;

    let status = HsmStatus::read(&mut core);
    println!("HSM_ID  {}", word(status.id));
    println!("DBGBASE {}", word(status.dbgbase));
    if !status.debug_open() {
        println!("the HSM debug window is closed (HSM debug not enabled by the HSM firmware)");
        return Ok(());
    }

    let mut window = HsmWindow::new(&mut core);
    let ctrl = window.read_word_32(HsmFpb::FP_CTRL)?;
    println!("FP_CTRL {ctrl:#010x}");
    match HsmFpb::from_ctrl(ctrl) {
        Ok(fpb) => println!("FPB: {} code comparators", fpb.comparators()),
        Err(error) => println!("FPB: {error}"),
    }
    println!("DHCSR   {:#010x}", window.read_word_32(DHCSR)?);
    Ok(())
}
