//! Does the GTM of a TC27x or TC36x stop with CPU0? With the GTM in soft suspend sensitivity
//! (`GTM_OCS.SUS = 2`, TC27x User's Manual 25, OCS), the GTM halts while the OCDS trigger
//! switch's suspend signal is active. This example halts CPU0 two ways and looks whether the
//! GTM halts too: whether `GTM_OCS.SUSSTA` is set and whether an MCS channel's register stops
//! changing. `PROBE_RS_CHIP` names the chip (`TC275`, the default, or `TC36x`: its GTM_OCS,
//! MCS registers and STM0 are elsewhere, [`Chip`]).
//!
//! 1. a halt request (`DBGSR.HALT`), which does not set the core's suspend-out (`DBGSR.SUSP`);
//! 2. a data watchpoint, whose trigger event sets `SUSP`.
//!
//! It needs the GTM test program running (`zed/aurix_tcf_demo/build_gtm.ps1` of aurix_debugger,
//! `-Board tc36x` for the TC36x, started with `start_in_flash`, RAM only): MCS1 channel 0 counts
//! in R0, and CPU0 writes
//! `g_ticks` (`0x7000A00C`) in its loop. Only `GTM_OCS` is written, and put back to 0 at the
//! end; CPU0 runs at the end.
//!
//! ```text
//! cargo run --release -p probe-rs-infineon --example gtm_suspend
//! ```
//!
//! `TRC_HALT=<n>` also routes CPU0's halt state to trigger line n (`CBS_TRC0.HALT`) for a third
//! case, with STM0 in soft suspend too; `GTM_SUS=<n>` sets `GTM_OCS.SUS` to n instead of 2.
//!
//! On the TC36x test board (gtm_demo -Board tc36x) with `TRC_HALT=1`, trigger line 1 being the
//! system suspend signal of the TC3xx: halting CPU0 suspends STM0 (`SUSSTA` 1, the timer
//! stops), which the TC275 did not, but not the GTM, in soft (`GTM_SUS=2`) or hard
//! (`GTM_SUS=1`) suspend: `GTM_OCS.SUSSTA` stays 0 and MCS1 runs on. STM0 stops because line 1
//! suspends CPU0 (`CBS_TL1ST.C0`) and STMx follows `CPUxSUSOUT` (TC3xx User's Manual, STM OCS).
//! The GTM does not take the cores' suspend: "Suspend is globally controlled in OTGS and GTM
//! locally with the OCS register" (TC3xx User's Manual, GTM 28.25.19.1.1), and the OTGS's global
//! suspend is not in the manual (its OCDS chapter is a separate specification). Tried on the
//! board without effect on the GTM: CPU0's halt state on each of lines 1-6, the OCDS control bits
//! `CBS_OSTATE.OC0`-`OC2` (set through `CBS_OCNTRL`), and `M0`-`M2`, `DMA`, `DMA1` as targets of
//! line 1.
//!
//! For more attempts in case 3, `EXTRA="addr=value;..."` writes words for the case and puts them
//! back after it, and `OCNTRL=<hex>` sets `CBS_OCNTRL` control bits and clears them after it.
//! Tried on the TC36x (2026-10-28, bit fields from the TC36x SFR headers) without effect on the
//! GTM, with STM0 stopping each time: line 1 capture and hold (`CBS_TLCHE.TL1`, it then holds the
//! line and CPU0 does not run until it is cleared), the OCDS control bits `OC0`-`OC5` together and
//! `OC3`-`OC5` (they show as `CBS_OSTATE` bits 4-6), and every line 1 signal processing
//! `CBS_TLC.TLSP1` = 1-15. `CBS_TL1ST` has no GTM target (CPU0-5, M0-M2, HSSL, DMA, HSM).

use std::time::Duration;

use probe_rs::{
    Core, CoreStatus, MemoryInterface, Permissions, WatchpointAccess,
    probe::{DebugProbeSelector, list::Lister},
};

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

const OCS_SUS_P: u32 = 1 << 28;
const OCS_SUSSTA: u32 = 1 << 29;
const OCS_SOFT_SUSPEND: u32 = 2 << 24;
/// CPU0's `DBGSR`.
const CPU0_DBGSR: u64 = 0xF881_FD00;
const G_TICKS: u64 = 0x7000_A00C;
/// The trigger switch routing of CPU0 (TC27x and TC3xx).
const CBS_TRC0: u64 = 0xF000_0420;
/// The OCDS control bits (write only: `OCx_P` in the even bits enables the write of `OCx`).
const CBS_OCNTRL: u64 = 0xF000_047C;

/// The words of `EXTRA` (`addr=value;addr=value`, hexadecimal or decimal).
fn extra_writes() -> Result<Vec<(u64, u32)>> {
    let Ok(text) = std::env::var("EXTRA") else {
        return Ok(Vec::new());
    };
    let number = |s: &str| -> Result<u64> {
        let s = s.trim();
        Ok(
            match s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
                Some(hex) => u64::from_str_radix(&hex.replace('_', ""), 16)?,
                None => s.parse()?,
            },
        )
    };
    text.split(';')
        .filter(|w| !w.trim().is_empty())
        .map(|w| {
            let (address, value) = w.split_once('=').ok_or("EXTRA: addr=value")?;
            Ok((number(address)?, number(value)? as u32))
        })
        .collect()
}

/// Where a chip has what the experiment reads and writes.
struct Chip {
    name: &'static str,
    gtm_ocs: u64,
    /// R0 of MCS1 channel 0.
    mcs1_ch0_r0: u64,
    /// STM0's OCDS control and status, and its timer.
    stm0_ocs: u64,
    stm0_tim0: u64,
    /// `DBGSR` of the other cores.
    others: &'static [(&'static str, u64)],
}

const TC275: Chip = Chip {
    name: "TC275",
    gtm_ocs: 0xF019_FDE8,
    mcs1_ch0_r0: 0xF013_1000,
    stm0_ocs: 0xF000_00E8,
    stm0_tim0: 0xF000_0010,
    others: &[("CPU1", 0xF883_FD00), ("CPU2", 0xF885_FD00)],
};

/// The TC36x (its SFR headers): GTM_OCS at GTM + 0x9FD38, the MCS registers from 0xF01F0000,
/// STM0 at 0xF0001000.
const TC36X: Chip = Chip {
    name: "TC36x",
    gtm_ocs: 0xF019_FD38,
    mcs1_ch0_r0: 0xF01F_1000,
    stm0_ocs: 0xF000_10E8,
    stm0_tim0: 0xF000_1010,
    others: &[("CPU1", 0xF883_FD00)],
};

/// Whether MCS1 channel 0 runs: its R0 read twice, 20 ms apart.
fn mcs_runs(chip: &Chip, core: &mut Core) -> Result<(bool, u32, u32)> {
    let a = core.read_word_32(chip.mcs1_ch0_r0)?;
    std::thread::sleep(Duration::from_millis(20));
    let b = core.read_word_32(chip.mcs1_ch0_r0)?;
    Ok((a != b, a, b))
}

fn report(chip: &Chip, core: &mut Core, what: &str) -> Result<()> {
    let ocs = core.read_word_32(chip.gtm_ocs)?;
    let dbgsr = core.read_word_32(CPU0_DBGSR)?;
    let (runs, a, b) = mcs_runs(chip, core)?;
    println!(
        "{what:<44} CPU0 {:?}, DBGSR {dbgsr:#010x} (SUSP {}), GTM_OCS {ocs:#010x} (SUSSTA {}), MCS1 ch0 R0 {a:#08x} -> {b:#08x}: {}",
        core.status()?,
        (dbgsr >> 4) & 1,
        (ocs & OCS_SUSSTA != 0) as u8,
        if runs { "GTM runs" } else { "GTM STOPPED" }
    );
    Ok(())
}

fn experiment(chip: &Chip, core: &mut Core) -> Result<()> {
    // GTM_SUS=<n> picks the GTM's sensitivity (2 soft suspend, the default; 1 hard suspend).
    let sus = std::env::var("GTM_SUS")
        .ok()
        .and_then(|v| v.parse::<u32>().ok())
        .map_or(OCS_SOFT_SUSPEND, |v| (v & 0xF) << 24);
    core.write_word_32(chip.gtm_ocs, OCS_SUS_P | sus)?;
    report(chip, core, "SUS = 2, CPU0 running")?;

    // 1. A halt request.
    core.halt(Duration::from_millis(500))?;
    std::thread::sleep(Duration::from_millis(20));
    report(chip, core, "1. halted by a request")?;
    core.run()?;
    std::thread::sleep(Duration::from_millis(20));
    report(chip, core, "   running again")?;

    // 2. A data watchpoint on g_ticks: its trigger event sets SUSP.
    core.set_watchpoint(G_TICKS, 4, WatchpointAccess::Write)?;
    core.wait_for_core_halted(Duration::from_millis(1000))?;
    std::thread::sleep(Duration::from_millis(20));
    report(chip, core, "2. halted at a data watchpoint (SUSP)")?;
    core.clear_watchpoint(G_TICKS)?;
    core.run()?;
    std::thread::sleep(Duration::from_millis(20));
    report(chip, core, "   running again")?;

    // 3. CPU0's halt state on a trigger line (CBS_TRC0.HALT): the system suspend signal is
    // trigger line TL1 (TC3xx User's Manual 9.4.5.4, CBS_TLS.TL1).
    if let Some(value) = std::env::var("TRC_HALT")
        .ok()
        .and_then(|v| v.parse::<u32>().ok())
    {
        let trc = core.read_word_32(CBS_TRC0)?;
        // EXTRA: words written for the case and put back after it; OCNTRL: OCDS control bits
        // set for the case and cleared after it.
        let extra = extra_writes()?;
        let mut saved = Vec::new();
        for &(address, _) in &extra {
            saved.push((address, core.read_word_32(address)?));
        }
        let ocntrl = std::env::var("OCNTRL")
            .ok()
            .and_then(|v| u32::from_str_radix(v.trim_start_matches("0x"), 16).ok());
        let outcome = (|| -> Result<()> {
            core.write_word_32(CBS_TRC0, (trc & !0xF) | (value & 0xF))?;
            let now = core.read_word_32(CBS_TRC0)?;
            println!("CBS_TRC0 {trc:#010x} -> {now:#010x} (HALT = {value})");
            for (&(address, value), &(_, old)) in extra.iter().zip(&saved) {
                core.write_word_32(address, value)?;
                let now = core.read_word_32(address)?;
                println!("{address:#010x} {old:#010x} -> {now:#010x} (wrote {value:#010x})");
            }
            if let Some(value) = ocntrl {
                core.write_word_32(CBS_OCNTRL, value)?;
                println!(
                    "CBS_OCNTRL <- {value:#010x}, CBS_OSTATE {:#010x}",
                    core.read_word_32(0xF000_0480)?
                );
            }
            // STM0 too: is any peripheral suspended?
            core.write_word_32(chip.stm0_ocs, OCS_SUS_P | OCS_SOFT_SUSPEND)?;
            report(chip, core, "3. HALT routed, CPU0 running")?;
            stm(chip, core)?;
            core.halt(Duration::from_millis(500))?;
            std::thread::sleep(Duration::from_millis(20));
            report(chip, core, "3. halted by a request, HALT routed")?;
            stm(chip, core)?;
            others(chip, core)?;
            core.run()?;
            std::thread::sleep(Duration::from_millis(20));
            report(chip, core, "   running again")?;
            others(chip, core)?;
            Ok(())
        })();
        // Put the routing, STM0 and the EXTRA words back whatever happened, and clear the
        // OCNTRL bits (the same write enables, the bits 0).
        let mut restored = core
            .write_word_32(CBS_TRC0, trc)
            .and_then(|()| core.write_word_32(chip.stm0_ocs, OCS_SUS_P));
        for &(address, old) in saved.iter().rev() {
            restored = restored.and(core.write_word_32(address, old));
        }
        if let Some(value) = ocntrl {
            restored = restored.and(core.write_word_32(CBS_OCNTRL, value & 0x5555_5555));
        }
        println!("CBS_TRC0 put back: {:#010x}", core.read_word_32(CBS_TRC0)?);
        outcome?;
        restored?;
    }
    Ok(())
}

/// Whether STM0 counts, and its suspend state.
fn stm(chip: &Chip, core: &mut Core) -> Result<()> {
    let ocs = core.read_word_32(chip.stm0_ocs)?;
    let a = core.read_word_32(chip.stm0_tim0)?;
    std::thread::sleep(Duration::from_millis(5));
    let b = core.read_word_32(chip.stm0_tim0)?;
    println!(
        "   STM0 OCS {ocs:#010x} (SUSSTA {}), TIM0 {a:#010x} -> {b:#010x}: {}",
        (ocs & OCS_SUSSTA != 0) as u8,
        if a != b { "counts" } else { "STOPPED" }
    );
    Ok(())
}

/// The halt state of the other cores (a suspend line may hold them too).
fn others(chip: &Chip, core: &mut Core) -> Result<()> {
    for &(name, dbgsr) in chip.others {
        let value = core.read_word_32(dbgsr)?;
        println!(
            "   {name} DBGSR {value:#010x}: {}, SIH {}",
            if value & 0b110 == 0b010 {
                "halted"
            } else {
                "running"
            },
            (value >> 3) & 1
        );
    }
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
    let chip = match std::env::var("PROBE_RS_CHIP") {
        Ok(name) if name.to_ascii_uppercase().starts_with("TC36") => &TC36X,
        _ => &TC275,
    };
    let mut session = probe.attach(chip.name, Permissions::default())?;
    let mut core = session.core(0)?;
    if core.status()? != CoreStatus::Running {
        core.run()?;
    }
    let before = core.read_word_32(chip.gtm_ocs)?;
    report(chip, &mut core, "at the start")?;
    let outcome = experiment(chip, &mut core);
    // Put the sensitivity back whatever happened, and let CPU0 run.
    let restored = core.write_word_32(chip.gtm_ocs, OCS_SUS_P | (before & (0xF << 24)));
    if core.status()?.is_halted() {
        core.clear_watchpoint(G_TICKS).ok();
        core.run()?;
    }
    if let Err(error) = &outcome {
        println!("stopped: {error}");
    }
    restored?;
    report(chip, &mut core, "at the end (GTM_OCS put back)")?;
    outcome
}
