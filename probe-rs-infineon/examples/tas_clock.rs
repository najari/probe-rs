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
//! cargo run --release -p probe-rs-infineon --example tas_clock -- each 3000000 4000000 6000000
//! ```
//!
//! `each` does not stop: at each frequency it reads three regions of different access times one
//! by one, DSPR0 (local RAM on the SRI), the SCU's CHIPID (on the SPB) and PFLASH (wait
//! states), each in a try of its own with the clock set again before it (after a failed access
//! the server is back at its start clock), and prints a line per frequency: which ones failed
//! with what. So it tells a clock the link cannot carry (every region fails) from accesses that
//! take too long for the clock (only the slow ones fail).
//!
//! `tas_clock set <Hz>` only sets the clock and leaves it, for clients that do not set it
//! themselves (an MCD library on the same TAS server); `tas_clock set 400000` puts the
//! adapter's start clock back.
//!
//! `PROBE_RS_PROBE` selects another probe, `PROBE_RS_PFLASH` the PFLASH address (hex).

use std::time::{Duration, Instant};

use probe_rs::{
    Permissions,
    architecture::tricore::bus::{AddressMap, TriCoreBusAccess},
    probe::{DebugProbeSelector, list::Lister},
};

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

/// The access hardware clock in Hz, address map 15.
const ACC_HW_FREQUENCY: u32 = 0x000C_0100;

/// What is read: 16 KiB of PFLASH and DSPR0, which changes while the cores run.
const DSPR0: (u32, usize) = (0x7000_0000, 4 * 1024);

/// The SCU's CHIPID register, on the SPB (some of the SCU's other words refuse a debugger read).
const SCU: (u32, usize) = (0xF003_6140, 4);

/// The PFLASH address read: `PROBE_RS_PFLASH` (hex), else 8003 0000H. An erased PFLASH word
/// reads with an ECC error, so it has to be one the board has written.
fn pflash_address() -> u32 {
    std::env::var("PROBE_RS_PFLASH")
        .ok()
        .and_then(|a| {
            u32::from_str_radix(a.trim_start_matches("0x").replace('_', "").as_str(), 16).ok()
        })
        .unwrap_or(0x8003_0000)
}

fn pflash() -> (u32, usize) {
    (pflash_address(), 16 * 1024)
}

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
        let (data, flash_time) = timed_reads(bus, pflash(), 5)?;
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
            kib_per_s(pflash().1, flash_time),
            dspr_time.as_secs_f64() * 1000.0,
            kib_per_s(DSPR0.1, dspr_time),
        );
    }
    Ok(())
}

/// One region read at `hz`, the clock set just before: the time, or what failed.
fn region_at(
    bus: &mut dyn TriCoreBusAccess,
    hz: u32,
    region: (u32, usize),
    rounds: usize,
) -> std::result::Result<Duration, String> {
    set_clock(bus, hz).map_err(|e| format!("setting the clock: {e}"))?;
    timed_reads(bus, region, rounds)
        .map(|(_, time)| time)
        .map_err(|e| e.to_string())
}

/// At each frequency every region, whatever failed before; a line per frequency.
fn each_region(bus: &mut dyn TriCoreBusAccess, frequencies: &[u32]) {
    for &hz in frequencies {
        let mut line = format!("{hz:>9} Hz:");
        for (name, region, rounds) in [
            ("DSPR0 4 KiB", DSPR0, 10),
            ("SCU CHIPID", SCU, 50),
            ("PFLASH 16 KiB", pflash(), 5),
        ] {
            match region_at(bus, hz, region, rounds) {
                Ok(time) => line.push_str(&format!(
                    "  {name} {:>7.1} KiB/s",
                    kib_per_s(region.1, time)
                )),
                Err(error) => line.push_str(&format!("  {name} FAILED ({error})")),
            }
        }
        let now = bus
            .read_u32(AddressMap::Server, ACC_HW_FREQUENCY)
            .map_or("?".to_string(), |v| v.to_string());
        println!("{line}  (clock now {now} Hz)");
    }
}

fn main() -> Result<()> {
    probe_rs_infineon::register_plugin();
    let mut args = std::env::args().skip(1).peekable();
    let set_only = args.next_if(|a| a == "set").is_some();
    let each = !set_only && args.next_if(|a| a == "each").is_some();
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
    let (reference, flash_time) = timed_reads(bus, pflash(), 5)?;
    let (_, dspr_time) = timed_reads(bus, DSPR0, 10)?;
    println!(
        "{original:>9} Hz (at the start)       : PFLASH 16 KiB {:>7.1} ms {:>7.1} KiB/s, DSPR0 4 KiB {:>6.1} ms {:>7.1} KiB/s",
        flash_time.as_secs_f64() * 1000.0,
        kib_per_s(pflash().1, flash_time),
        dspr_time.as_secs_f64() * 1000.0,
        kib_per_s(DSPR0.1, dspr_time),
    );

    let outcome = if each {
        each_region(bus, &frequencies);
        Ok(())
    } else {
        try_clocks(bus, &frequencies, &reference)
    };
    // Put the clock back whatever happened, then check that reads are as at the start.
    let restored = set_clock(bus, original);
    if let Err(error) = &outcome {
        println!("stopped: {error}");
    }
    let restored = restored?;
    let (again, _) = timed_reads(bus, pflash(), 1)?;
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
