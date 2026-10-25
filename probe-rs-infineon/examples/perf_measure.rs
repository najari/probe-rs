//! Measures what the debug link costs on a TC275 (PRD 11, "performance targets"): the latency
//! of a halt, a single step, a register read and a status check, and the time and throughput of
//! memory reads and writes of 256 B, 4 KiB and 32 KiB. Every figure is a wall clock time; each
//! item is measured at least 20 times and shown as median and 95th percentile.
//!
//! The applications on the three cores keep running. CPU0 is halted for the measurements and
//! run again at the end. Memory is read from DSPR0 and PFLASH; the write test saves 16 KiB of
//! the CPU0 PSPR (a halted core, so nobody executes it; 32 KiB does not fit in it), writes a
//! pattern, checks it and puts the saved bytes back. Nothing is written to flash and the device
//! is not reset.
//!
//! ```text
//! cargo run --release -p probe-rs-infineon --example perf_measure
//! ```
//!
//! `PROBE_RS_PROBE` selects another probe (an MCD probe with `--features mcd`),
//! `PROBE_RS_SPEED` the interface speed in kHz (the TAS probe sets the adapter clock),
//! `PROBE_RS_CHIP` names the chip.

use std::time::{Duration, Instant};

use probe_rs::{
    Core, CoreStatus, MemoryInterface, Permissions, RegisterId,
    architecture::tricore::{bus::AddressMap, registers::CSFR_PC},
    probe::{DebugProbeSelector, list::Lister},
};

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

/// The rounds of each item.
const ROUNDS: usize = 20;

fn ms(d: Duration) -> f64 {
    d.as_secs_f64() * 1000.0
}

/// Median and 95th percentile of `samples`.
fn median_p95(samples: &mut [Duration]) -> (Duration, Duration) {
    samples.sort_unstable();
    let n = samples.len();
    (samples[n / 2], samples[(n * 95 / 100).min(n - 1)])
}

/// Prints min, median, 95th percentile and maximum of `samples` in milliseconds.
fn report(what: &str, samples: &mut [Duration]) {
    let (median, p95) = median_p95(samples);
    let n = samples.len();
    println!(
        "{what:<34} n={n:<4} min {:>7.2}  median {:>7.2}  p95 {:>7.2}  max {:>7.2} ms",
        ms(samples[0]),
        ms(median),
        ms(p95),
        ms(samples[n - 1]),
    );
}

/// Prints the median and 95th percentile of `samples` for `length` bytes each, and the
/// throughput at the median.
fn report_transfer(what: &str, length: usize, samples: &mut [Duration]) {
    let (median, p95) = median_p95(samples);
    println!(
        "{what:<34} n={:<4} {length:>6} B  median {:>8.2}  p95 {:>8.2} ms  {:>7.1} KiB/s",
        samples.len(),
        ms(median),
        ms(p95),
        length as f64 / 1024.0 / median.as_secs_f64(),
    );
}

fn time<T>(samples: &mut Vec<Duration>, f: impl FnOnce() -> T) -> T {
    let start = Instant::now();
    let value = f();
    samples.push(start.elapsed());
    value
}

/// Reads `length` bytes at `address` [`ROUNDS`] times.
fn read_item(core: &mut Core, what: &str, address: u64, length: usize) -> Result<()> {
    let mut buffer = vec![0; length];
    // One warm up read.
    core.read(address, &mut buffer)?;
    let mut samples = Vec::new();
    for _ in 0..ROUNDS {
        time(&mut samples, || core.read(address, &mut buffer))?;
    }
    report_transfer(what, length, &mut samples);
    Ok(())
}

/// Writes a pattern of each length to the CPU0 PSPR [`ROUNDS`] times, checks it, and puts the
/// saved bytes back whatever happened.
fn write_items(core: &mut Core, area: u64, lengths: &[usize]) -> Result<()> {
    let saved_length = lengths.iter().copied().max().unwrap_or(0);
    let mut saved = vec![0; saved_length];
    core.read(area, &mut saved)?;
    let outcome = (|| -> Result<bool> {
        let mut all_ok = true;
        for &length in lengths {
            let pattern: Vec<u8> = (0..length).map(|i| (i * 7 + length) as u8).collect();
            let mut samples = Vec::new();
            for _ in 0..ROUNDS {
                time(&mut samples, || core.write(area, &pattern))?;
            }
            let mut check = vec![0; length];
            core.read(area, &mut check)?;
            all_ok &= check == pattern;
            report_transfer(
                &format!(
                    "write CPU0 PSPR{}",
                    if check == pattern { "" } else { " (DIFFERS)" }
                ),
                length,
                &mut samples,
            );
        }
        Ok(all_ok)
    })();
    let restored = core.write(area, &saved);
    let all_ok = outcome?;
    restored?;
    let mut after = vec![0; saved_length];
    core.read(area, &mut after)?;
    println!(
        "patterns read back: {}, PSPR restored: {}",
        if all_ok { "ok" } else { "DIFFER" },
        if after == saved { "ok" } else { "DIFFERS" }
    );
    if !all_ok || after != saved {
        return Err("the PSPR writes did not read back".into());
    }
    Ok(())
}

fn main() -> Result<()> {
    probe_rs_infineon::register_plugin();
    let selector: DebugProbeSelector = std::env::var("PROBE_RS_PROBE")
        .unwrap_or_else(|_| "058b:0000".to_string())
        .parse()?;
    let chip = std::env::var("PROBE_RS_CHIP")
        .map(probe_rs::config::TargetSelector::from)
        .unwrap_or(probe_rs::config::TargetSelector::Auto);
    let mut probe = Lister::new().open(selector)?;
    if let Ok(speed) = std::env::var("PROBE_RS_SPEED") {
        probe.set_speed(speed.parse()?)?;
    }
    let name = probe.get_name();
    let mut session = probe.attach(chip, Permissions::default())?;
    // The TAS server's access hardware clock (map 15); an MCD probe may not reach it.
    let clock = session
        .get_tricore_bus()?
        .read_u32(AddressMap::Server, 0x000C_0100)
        .map(|hz| format!("{} kHz", hz / 1000))
        .unwrap_or_else(|_| "not reported".to_string());
    println!(
        "{} through {name}, access hardware clock {clock}",
        session.target().name
    );
    let mut core = session.core(0)?;

    // Halt and run.
    let (mut halts, mut runs) = (Vec::new(), Vec::new());
    for _ in 0..ROUNDS {
        time(&mut halts, || core.halt(Duration::from_millis(500)))?;
        time(&mut runs, || core.run())?;
        std::thread::sleep(Duration::from_millis(20));
    }
    core.halt(Duration::from_millis(500))?;
    report("halt (request to halted)", &mut halts);
    report("run", &mut runs);

    // Status check: what a debugger polls while the core runs, and while it is halted.
    let mut status_halted = Vec::new();
    for _ in 0..100 {
        time(&mut status_halted, || core.status())?;
    }
    report("status, core halted", &mut status_halted);

    // Register reads.
    let mut registers = Vec::new();
    for _ in 0..100 {
        time(&mut registers, || {
            core.read_core_reg::<u32>(RegisterId(CSFR_PC))
        })?;
    }
    report("read one register (PC)", &mut registers);

    // Single steps in the running application.
    let mut steps = Vec::new();
    for _ in 0..200 {
        time(&mut steps, || core.step())?;
    }
    report("single step", &mut steps);

    // Memory. DSPR0 and PFLASH are read, the CPU0 PSPR is written and restored.
    read_item(&mut core, "read DSPR0, 4 B", 0x7000_0000, 4)?;
    read_item(&mut core, "read DSPR0, 256 B", 0x7000_0000, 256)?;
    read_item(&mut core, "read DSPR0, 4 KiB", 0x7000_0000, 4 * 1024)?;
    read_item(&mut core, "read DSPR0, 32 KiB", 0x7000_0000, 32 * 1024)?;
    read_item(
        &mut core,
        "read PFLASH (cached), 32 KiB",
        0x8003_0000,
        32 * 1024,
    )?;
    write_items(&mut core, 0x7010_0000, &[256, 4 * 1024, 16 * 1024])?;

    // Status while the core runs.
    core.run()?;
    std::thread::sleep(Duration::from_millis(100));
    let mut status_running = Vec::new();
    for _ in 0..100 {
        time(&mut status_running, || core.status())?;
    }
    report("status, core running", &mut status_running);
    println!("core status: {:?}", core.status()?);
    if core.status()? != CoreStatus::Running {
        return Err("the core must run at the end".into());
    }
    Ok(())
}
