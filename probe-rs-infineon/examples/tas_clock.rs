//! Tries other clock frequencies of the TAS access hardware (the JTAG clock of the DAS adapter)
//! and measures the memory read throughput at each, to see whether the clock limits it.
//!
//! The clock is the TAS server's register at 0x000C0100 of address map 15, in Hz (Infineon's
//! `tas_perfmeter` changes it under "TAS Access HW Control"). The example reads PFLASH and
//! DSPR0 at the clock it finds, then sets each frequency given, reads the same bytes again and
//! compares them. It stops at the first error or difference, and puts the clock back at the end
//! in any case. Only memory reads; the cores keep running.
//!
//! ```text
//! cargo run --release -p probe-rs-infineon --example tas_clock -- 1000000 2000000 5000000
//! ```
//!
//! `tas_clock set <Hz>` only sets the clock and leaves it, for clients that do not set it
//! themselves (an MCD library on the same TAS server); `tas_clock set 400000` puts the
//! adapter's start clock back.
//!
//! `PROBE_RS_PROBE` selects another probe.

use std::time::{Duration, Instant};

use probe_rs::{
    Permissions,
    architecture::tricore::bus::{AddressMap, TriCoreBusAccess},
    probe::{DebugProbeSelector, list::Lister},
};

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

/// The access hardware clock in Hz, address map 15.
const ACC_HW_FREQUENCY: u32 = 0x000C_0100;

/// What is read: PFLASH (cached alias) and DSPR0, which changes while the cores run.
const PFLASH: (u32, usize) = (0x8003_0000, 16 * 1024);
const DSPR0: (u32, usize) = (0x7000_0000, 4 * 1024);

/// Reads `length` bytes at `address` `rounds` times; returns the last bytes and the median
/// time of one read.
fn timed_reads(
    bus: &mut dyn TriCoreBusAccess,
    (address, length): (u32, usize),
    rounds: usize,
) -> Result<(Vec<u8>, Duration)> {
    let mut buffer = vec![0; length];
    let mut samples = Vec::new();
    for _ in 0..rounds {
        let start = Instant::now();
        bus.read_block(AddressMap::Target, address, &mut buffer)?;
        samples.push(start.elapsed());
    }
    samples.sort_unstable();
    Ok((buffer, samples[samples.len() / 2]))
}

fn kib_per_s(length: usize, time: Duration) -> f64 {
    length as f64 / 1024.0 / time.as_secs_f64()
}

/// Sets the clock and reads it back.
fn set_clock(bus: &mut dyn TriCoreBusAccess, hz: u32) -> Result<u32> {
    bus.write_u32(AddressMap::Server, ACC_HW_FREQUENCY, hz)?;
    Ok(bus.read_u32(AddressMap::Server, ACC_HW_FREQUENCY)?)
}

/// Reads at each frequency in turn; stops at the first error or difference.
fn try_clocks(bus: &mut dyn TriCoreBusAccess, frequencies: &[u32], reference: &[u8]) -> Result<()> {
    for &hz in frequencies {
        let now = set_clock(bus, hz)?;
        let (data, flash_time) = timed_reads(bus, PFLASH, 5)?;
        if data != reference {
            let first = data
                .iter()
                .zip(reference)
                .position(|(a, b)| a != b)
                .unwrap_or(0);
            return Err(format!(
                "at {hz} Hz (reads back {now}) PFLASH differs from the reference at offset {first:#x}"
            )
            .into());
        }
        let (_, dspr_time) = timed_reads(bus, DSPR0, 10)?;
        println!(
            "{hz:>9} Hz (reads back {now:>9}): PFLASH 16 KiB {:>7.1} ms {:>7.1} KiB/s, DSPR0 4 KiB {:>6.1} ms {:>7.1} KiB/s, PFLASH same as at the start",
            flash_time.as_secs_f64() * 1000.0,
            kib_per_s(PFLASH.1, flash_time),
            dspr_time.as_secs_f64() * 1000.0,
            kib_per_s(DSPR0.1, dspr_time),
        );
    }
    Ok(())
}

fn main() -> Result<()> {
    probe_rs_infineon::register_plugin();
    let mut args = std::env::args().skip(1).peekable();
    let set_only = args.next_if(|a| a == "set").is_some();
    let frequencies = args
        .map(|a| a.parse::<u32>())
        .collect::<std::result::Result<Vec<_>, _>>()?;
    if set_only && frequencies.len() != 1 {
        return Err("tas_clock set <Hz>".into());
    }
    let selector: DebugProbeSelector = std::env::var("PROBE_RS_PROBE")
        .unwrap_or_else(|_| "058b:0000".to_string())
        .parse()?;
    let mut session = Lister::new().open(selector)?.attach(
        probe_rs::config::TargetSelector::Auto,
        Permissions::default(),
    )?;
    let bus = session.get_tricore_bus()?;

    let original = bus.read_u32(AddressMap::Server, ACC_HW_FREQUENCY)?;
    if set_only {
        let now = set_clock(bus, frequencies[0])?;
        println!("access hardware clock {original} Hz -> {now} Hz (left so)");
        return Ok(());
    }
    let (reference, flash_time) = timed_reads(bus, PFLASH, 5)?;
    let (_, dspr_time) = timed_reads(bus, DSPR0, 10)?;
    println!(
        "{original:>9} Hz (at the start)       : PFLASH 16 KiB {:>7.1} ms {:>7.1} KiB/s, DSPR0 4 KiB {:>6.1} ms {:>7.1} KiB/s",
        flash_time.as_secs_f64() * 1000.0,
        kib_per_s(PFLASH.1, flash_time),
        dspr_time.as_secs_f64() * 1000.0,
        kib_per_s(DSPR0.1, dspr_time),
    );

    let outcome = try_clocks(bus, &frequencies, &reference);
    // Put the clock back whatever happened, then check that reads are as at the start.
    let restored = set_clock(bus, original);
    if let Err(error) = &outcome {
        println!("stopped: {error}");
    }
    let restored = restored?;
    let (again, _) = timed_reads(bus, PFLASH, 1)?;
    println!(
        "clock put back: {restored} Hz, PFLASH {}",
        if again == reference {
            "same as at the start"
        } else {
            "DIFFERS"
        }
    );
    outcome
}
