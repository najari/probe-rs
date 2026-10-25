//! Does the GTM of a TC27x stop with CPU0? With the GTM in soft suspend sensitivity
//! (`GTM_OCS.SUS = 2`, TC27x User's Manual 25, OCS), the GTM halts while the OCDS trigger
//! switch's suspend signal is active. This example halts CPU0 two ways and looks whether the
//! GTM halts too: whether `GTM_OCS.SUSSTA` is set and whether an MCS channel's register stops
//! changing.
//!
//! 1. a halt request (`DBGSR.HALT`), which does not set the core's suspend-out (`DBGSR.SUSP`);
//! 2. a data watchpoint, whose trigger event sets `SUSP`.
//!
//! It needs the GTM test program running (`zed/aurix_tcf_demo/build_gtm.ps1` of aurix_debugger,
//! started with `start_in_flash`, RAM only): MCS1 channel 0 counts in R0, and CPU0 writes
//! `g_ticks` (`0x7000A00C`) in its loop. Only `GTM_OCS` is written, and put back to 0 at the
//! end; CPU0 runs at the end.
//!
//! ```text
//! cargo run --release -p probe-rs-infineon --example gtm_suspend
//! ```

use std::time::Duration;

use probe_rs::{
    Core, CoreStatus, MemoryInterface, Permissions, WatchpointAccess,
    probe::{DebugProbeSelector, list::Lister},
};

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

const GTM_OCS: u64 = 0xF019_FDE8;
const OCS_SUS_P: u32 = 1 << 28;
const OCS_SUSSTA: u32 = 1 << 29;
const OCS_SOFT_SUSPEND: u32 = 2 << 24;
/// R0 of MCS1 channel 0.
const MCS1_CH0_R0: u64 = 0xF013_1000;
/// CPU0's `DBGSR`.
const CPU0_DBGSR: u64 = 0xF881_FD00;
const G_TICKS: u64 = 0x7000_A00C;

/// Whether MCS1 channel 0 runs: its R0 read twice, 20 ms apart.
fn mcs_runs(core: &mut Core) -> Result<(bool, u32, u32)> {
    let a = core.read_word_32(MCS1_CH0_R0)?;
    std::thread::sleep(Duration::from_millis(20));
    let b = core.read_word_32(MCS1_CH0_R0)?;
    Ok((a != b, a, b))
}

fn report(core: &mut Core, what: &str) -> Result<()> {
    let ocs = core.read_word_32(GTM_OCS)?;
    let dbgsr = core.read_word_32(CPU0_DBGSR)?;
    let (runs, a, b) = mcs_runs(core)?;
    println!(
        "{what:<44} CPU0 {:?}, DBGSR {dbgsr:#010x} (SUSP {}), GTM_OCS {ocs:#010x} (SUSSTA {}), MCS1 ch0 R0 {a:#08x} -> {b:#08x}: {}",
        core.status()?,
        (dbgsr >> 4) & 1,
        (ocs & OCS_SUSSTA != 0) as u8,
        if runs { "GTM runs" } else { "GTM STOPPED" }
    );
    Ok(())
}

fn experiment(core: &mut Core) -> Result<()> {
    core.write_word_32(GTM_OCS, OCS_SUS_P | OCS_SOFT_SUSPEND)?;
    report(core, "SUS = 2, CPU0 running")?;

    // 1. A halt request.
    core.halt(Duration::from_millis(500))?;
    std::thread::sleep(Duration::from_millis(20));
    report(core, "1. halted by a request")?;
    core.run()?;
    std::thread::sleep(Duration::from_millis(20));
    report(core, "   running again")?;

    // 2. A data watchpoint on g_ticks: its trigger event sets SUSP.
    core.set_watchpoint(G_TICKS, 4, WatchpointAccess::Write)?;
    core.wait_for_core_halted(Duration::from_millis(1000))?;
    std::thread::sleep(Duration::from_millis(20));
    report(core, "2. halted at a data watchpoint (SUSP)")?;
    core.clear_watchpoint(G_TICKS)?;
    core.run()?;
    std::thread::sleep(Duration::from_millis(20));
    report(core, "   running again")?;
    Ok(())
}

fn main() -> Result<()> {
    probe_rs_infineon::register_plugin();
    let selector: DebugProbeSelector = std::env::var("PROBE_RS_PROBE")
        .unwrap_or_else(|_| "058b:0000".to_string())
        .parse()?;
    let mut probe = Lister::new().open(selector)?;
    if let Ok(speed) = std::env::var("PROBE_RS_SPEED") {
        probe.set_speed(speed.parse()?)?;
    }
    let mut session = probe.attach("TC275", Permissions::default())?;
    let mut core = session.core(0)?;
    if core.status()? != CoreStatus::Running {
        core.run()?;
    }
    let before = core.read_word_32(GTM_OCS)?;
    report(&mut core, "at the start")?;
    let outcome = experiment(&mut core);
    // Put the sensitivity back whatever happened, and let CPU0 run.
    let restored = core.write_word_32(GTM_OCS, OCS_SUS_P | (before & (0xF << 24)));
    if core.status()?.is_halted() {
        core.clear_watchpoint(G_TICKS).ok();
        core.run()?;
    }
    if let Err(error) = &outcome {
        println!("stopped: {error}");
    }
    restored?;
    report(&mut core, "at the end (GTM_OCS put back)")?;
    outcome
}
