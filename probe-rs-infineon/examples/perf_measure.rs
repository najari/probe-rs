//! Measures what the debug link costs on a TC275 (PRD 11, "performance targets"): the latency
//! of a halt, a single step, a register read and a status check, and the throughput of memory
//! reads and writes. Every figure is a wall clock time over the TAS TCP link.
//!
//! The applications on the three cores keep running. CPU0 is halted for the measurements and
//! run again at the end. Memory is read from DSPR0 and PFLASH; the write test saves 16 KiB of
//! the CPU0 PSPR (a halted core, so nobody executes it), writes a pattern, checks it and puts
//! the saved bytes back. Nothing is written to flash and the device is not reset.
//!
//! ```text
//! cargo run --release -p probe-rs-infineon --example perf_measure
//! ```
//!
//! `PROBE_RS_PROBE` selects another probe, `PROBE_RS_CHIP` names the chip.

use std::time::{Duration, Instant};

use probe_rs::{
    Core, CoreStatus, MemoryInterface, Permissions, RegisterId,
    architecture::tricore::registers::CSFR_PC,
    probe::{DebugProbeSelector, list::Lister},
};

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

/// Prints min, median, 95th percentile and maximum of `samples` in milliseconds.
fn report(what: &str, samples: &mut [Duration]) {
    samples.sort_unstable();
    let ms = |d: Duration| d.as_secs_f64() * 1000.0;
    let n = samples.len();
    println!(
        "{what:<34} n={n:<4} min {:>7.2}  median {:>7.2}  p95 {:>7.2}  max {:>7.2} ms",
        ms(samples[0]),
        ms(samples[n / 2]),
        ms(samples[(n * 95 / 100).min(n - 1)]),
        ms(samples[n - 1]),
    );
}

fn time<T>(samples: &mut Vec<Duration>, f: impl FnOnce() -> T) -> T {
    let start = Instant::now();
    let value = f();
    samples.push(start.elapsed());
    value
}

/// Reads `length` bytes at `address` `rounds` times and prints the throughput.
fn read_throughput(
    core: &mut Core,
    what: &str,
    address: u64,
    length: usize,
    rounds: usize,
) -> Result<()> {
    let mut buffer = vec![0; length];
    // One warm up read.
    core.read(address, &mut buffer)?;
    let mut samples = Vec::new();
    for _ in 0..rounds {
        time(&mut samples, || core.read(address, &mut buffer))?;
    }
    let total: Duration = samples.iter().sum();
    let kib = (length * rounds) as f64 / 1024.0;
    println!(
        "{what:<34} {length:>6} B x {rounds:<3} {:>8.1} KiB/s, {:>7.2} ms per read",
        kib / total.as_secs_f64(),
        total.as_secs_f64() * 1000.0 / rounds as f64
    );
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
    let mut session = Lister::new()
        .open(selector)?
        .attach(chip, Permissions::default())?;
    println!("{}", session.target().name);
    let mut core = session.core(0)?;

    // Halt and run.
    let (mut halts, mut runs) = (Vec::new(), Vec::new());
    for _ in 0..20 {
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

    // Memory throughput. DSPR0 and PFLASH are read, the CPU0 PSPR is written and restored.
    read_throughput(
        &mut core,
        "read DSPR0 (global alias)",
        0x7000_0000,
        32 * 1024,
        5,
    )?;
    read_throughput(&mut core, "read DSPR0, 4 KiB", 0x7000_0000, 4 * 1024, 20)?;
    read_throughput(&mut core, "read DSPR0, 256 B", 0x7000_0000, 256, 50)?;
    read_throughput(
        &mut core,
        "read PFLASH (cached alias)",
        0x8003_0000,
        32 * 1024,
        5,
    )?;
    read_throughput(&mut core, "read DSPR0, 4 B", 0x7000_0000, 4, 100)?;

    let (area, length) = (0x7010_0000u64, 16 * 1024usize);
    let mut saved = vec![0; length];
    core.read(area, &mut saved)?;
    let pattern: Vec<u8> = (0..length).map(|i| (i * 7 + 3) as u8).collect();
    let mut write_samples = Vec::new();
    let mut check = vec![0; length];
    let outcome = (|| -> Result<()> {
        for _ in 0..5 {
            time(&mut write_samples, || core.write(area, &pattern))?;
        }
        core.read(area, &mut check)?;
        Ok(())
    })();
    // Put the saved bytes back, whatever happened.
    let restored = core.write(area, &saved);
    outcome?;
    restored?;
    let total: Duration = write_samples.iter().sum();
    println!(
        "{:<34} {length:>6} B x 5   {:>8.1} KiB/s, {:>7.2} ms per write; pattern read back {}",
        "write CPU0 PSPR",
        (length * 5) as f64 / 1024.0 / total.as_secs_f64(),
        total.as_secs_f64() * 1000.0 / 5.0,
        if check == pattern { "ok" } else { "DIFFERS" }
    );
    let mut after = vec![0; length];
    core.read(area, &mut after)?;
    println!(
        "PSPR restored: {}",
        if after == saved { "ok" } else { "DIFFERS" }
    );

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
