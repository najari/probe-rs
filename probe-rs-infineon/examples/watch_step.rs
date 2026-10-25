//! A data watchpoint after the core ran on from an instruction breakpoint (which steps over the
//! breakpoint first): does the watchpoint still halt the core?
//!
//! The flash demo (`demo_flash.elf` in PF1, started with `start_in_flash`): `compute` starts at
//! `0x8021C080` and writes `g_result` (`0x7000A004`) with the store before `0x8021C090`.
//! Each halt prints its PC, reason and the trigger units of CPU0. Nothing is written to flash;
//! the breakpoint and watchpoint are removed and CPU0 runs at the end.
//!
//! ```text
//! cargo run -p probe-rs-infineon --example watch_step
//! ```

use std::time::{Duration, Instant};

use probe_rs::{
    Core, CoreStatus, MemoryInterface, Permissions, WatchpointAccess,
    architecture::tricore::bus::{AddressMap, TriCoreBusAccess},
    probe::{DebugProbeSelector, list::Lister},
};

const COMPUTE: u64 = 0x8021_C080;
const G_RESULT: u64 = 0x7000_A004;
const AFTER_STORE: u64 = 0x8021_C090;
/// `main`'s call of `compute`, after the store.
const MAIN_CALL: u64 = 0x8021_C0AC;
/// The first line of `square`, called before the store.
const SQUARE: u64 = 0x8021_C008;
const TRIG: u32 = 0xF881_F000;

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

fn wait_halt(core: &mut Core) -> Result<(u64, CoreStatus)> {
    let start = Instant::now();
    loop {
        let status = core.status()?;
        if status.is_halted() {
            let pc: u64 = core
                .read_core_reg::<u32>(core.program_counter().id())?
                .into();
            return Ok((pc, status));
        }
        if start.elapsed() > Duration::from_secs(2) {
            return Err("no halt within 2 s".into());
        }
        std::thread::sleep(Duration::from_millis(5));
    }
}

fn dump(bus: &mut dyn TriCoreBusAccess, label: &str) -> Result<()> {
    let mut line = format!("  {label:<14}");
    for unit in 0..8 {
        let evt = bus.read_u32(AddressMap::Target, TRIG + 8 * unit)?;
        let adr = bus
            .read_u32(AddressMap::Target, TRIG + 8 * unit + 4)
            .unwrap_or(0xDEAD_DEAD);
        if evt != 0 || unit >= 6 {
            line.push_str(&format!(" TR{unit} {evt:#x}@{adr:#010x}"));
        }
    }
    for (name, offset) in [
        ("DBGSR", 0xD00),
        ("EXEVT", 0xD08),
        ("CREVT", 0xD0C),
        ("SWEVT", 0xD10),
    ] {
        let value = bus.read_u32(AddressMap::Target, TRIG + offset)?;
        line.push_str(&format!(" {name} {value:#x}"));
    }
    println!("{line}");
    Ok(())
}

fn main() -> Result<()> {
    probe_rs_infineon::register_plugin();
    let selector = std::env::var("PROBE_RS_PROBE").unwrap_or_else(|_| "058b:0000".to_string());
    let mut session = Lister::new()
        .open(selector.parse::<DebugProbeSelector>()?)?
        .attach("TC275", Permissions::default())?;

    let failed = std::cell::Cell::new(0usize);
    let check = |what: &str, (pc, status): (u64, CoreStatus), want: u64| {
        let ok = pc == want;
        println!(
            "[{}] {what}: halted at {pc:#010x} ({status:?}), wanted {want:#010x}",
            if ok { "PASS" } else { "FAIL" }
        );
        if !ok {
            failed.set(failed.get() + 1);
        }
    };

    {
        let mut core = session.core(0)?;
        core.halt(Duration::from_millis(200))?;
        core.set_watchpoint(G_RESULT, 4, WatchpointAccess::Write)?;
    }
    dump(session.get_tricore_bus()?, "watchpoint")?;
    for round in 0..2 {
        let mut core = session.core(0)?;
        core.run()?;
        let halt = wait_halt(&mut core)?;
        drop(core);
        check(
            &format!("watchpoint alone, run {}", round + 1),
            halt,
            AFTER_STORE,
        );
        dump(session.get_tricore_bus()?, "halted")?;
    }

    // A single step from the watchpoint halt, no breakpoint set.
    {
        let mut core = session.core(0)?;
        core.step()?;
        core.run()?;
        let halt = wait_halt(&mut core)?;
        drop(core);
        check("watchpoint after a step", halt, AFTER_STORE);
        dump(session.get_tricore_bus()?, "halted")?;
    }

    {
        let mut core = session.core(0)?;
        core.set_hw_breakpoint(COMPUTE)?;
    }
    dump(session.get_tricore_bus()?, "breakpoint")?;
    {
        let mut core = session.core(0)?;
        core.run()?;
        let halt = wait_halt(&mut core)?;
        drop(core);
        check("breakpoint", halt, COMPUTE);
        dump(session.get_tricore_bus()?, "halted")?;
    }
    for round in 0..2 {
        // From the breakpoint: the run steps over it first.
        let mut core = session.core(0)?;
        core.run()?;
        let halt = wait_halt(&mut core)?;
        drop(core);
        check(
            &format!("watchpoint after the breakpoint, round {}", round + 1),
            halt,
            AFTER_STORE,
        );
        dump(session.get_tricore_bus()?, "halted")?;
        let mut core = session.core(0)?;
        core.run()?;
        let halt = wait_halt(&mut core)?;
        drop(core);
        check(&format!("breakpoint, round {}", round + 2), halt, COMPUTE);
    }

    // V2: halted by the breakpoint, the breakpoint cleared, then run (no step).
    {
        let mut core = session.core(0)?;
        core.clear_hw_breakpoint(COMPUTE)?;
        core.run()?;
        let halt = wait_halt(&mut core)?;
        drop(core);
        check("V2 breakpoint cleared at its halt, run", halt, AFTER_STORE);
    }
    // V3: halted by the breakpoint, a step by hand (the breakpoint set), then run.
    {
        let mut core = session.core(0)?;
        core.set_hw_breakpoint(COMPUTE)?;
        core.run()?;
        let halt = wait_halt(&mut core)?;
        check("V3 breakpoint", halt, COMPUTE);
        core.step()?;
        core.clear_hw_breakpoint(COMPUTE)?;
        core.run()?;
        let halt = wait_halt(&mut core)?;
        drop(core);
        check("V3 step at the breakpoint, cleared, run", halt, AFTER_STORE);
    }

    // V4: a breakpoint set after the store (main's call of compute), the core halted at the
    // watchpoint (not at a breakpoint), then run: the watchpoint should come before it.
    {
        let mut core = session.core(0)?;
        core.set_hw_breakpoint(MAIN_CALL)?;
        core.run()?;
        let halt = wait_halt(&mut core)?;
        check("V4 breakpoint later, from the watchpoint", halt, MAIN_CALL);
        core.run()?;
        let halt = wait_halt(&mut core)?;
        drop(core);
        check("V4 then from that breakpoint", halt, AFTER_STORE);
    }

    // V5: halted before the store (in square, its breakpoint then cleared), a breakpoint armed
    // after the store, run from a place without a breakpoint: is the watchpoint first?
    {
        let mut core = session.core(0)?;
        core.clear_hw_breakpoint(MAIN_CALL)?;
        core.set_hw_breakpoint(SQUARE)?;
        core.run()?;
        let halt = wait_halt(&mut core)?;
        check("V5 square", halt, SQUARE);
        core.clear_hw_breakpoint(SQUARE)?;
        core.set_hw_breakpoint(MAIN_CALL)?;
        core.run()?;
        let halt = wait_halt(&mut core)?;
        drop(core);
        check(
            "V5 armed breakpoint later, run from no breakpoint",
            halt,
            AFTER_STORE,
        );
    }

    // V6: the same as V5 with a one-byte watchpoint (one unit, no range).
    {
        let mut core = session.core(0)?;
        core.clear_watchpoint(G_RESULT)?;
        core.set_watchpoint(G_RESULT, 1, WatchpointAccess::Write)?;
        core.clear_hw_breakpoint(MAIN_CALL)?;
        core.set_hw_breakpoint(SQUARE)?;
        core.run()?;
        let halt = wait_halt(&mut core)?;
        check("V6 square", halt, SQUARE);
        core.clear_hw_breakpoint(SQUARE)?;
        core.set_hw_breakpoint(MAIN_CALL)?;
        drop(core);
        dump(session.get_tricore_bus()?, "V6 armed")?;
        let mut core = session.core(0)?;
        core.run()?;
        let halt = wait_halt(&mut core)?;
        drop(core);
        check(
            "V6 one-byte watchpoint, breakpoint armed",
            halt,
            AFTER_STORE,
        );
        let value = session.core(0)?.read_word_32(G_RESULT)?;
        check_value(
            "g_result at the watchpoint halt (written)",
            value,
            46,
            &failed,
        );
    }

    // The halt group: CPU0 and CPU1. CPU0 halts at its watchpoint (CPU1 with it); running CPU1
    // must leave CPU0 halted at its own watchpoint.
    {
        let mut core = session.core(0)?;
        let _ = core.clear_hw_breakpoint(MAIN_CALL);
        core.clear_watchpoint(G_RESULT)?;
        core.set_watchpoint(G_RESULT, 4, WatchpointAccess::Write)?;
    }
    for core in 0..2 {
        session.set_tricore_halt_group(core, true)?;
    }
    {
        // CPU1 runs, CPU0 from where it is.
        let mut core1 = session.core(1)?;
        if core1.core_halted()? {
            core1.run()?;
        }
        drop(core1);
        let mut core = session.core(0)?;
        core.run()?;
        let halt = wait_halt(&mut core)?;
        drop(core);
        check("group: CPU0 at its watchpoint", halt, AFTER_STORE);
        let bus = session.get_tricore_bus()?;
        let dbgsr0 = bus.read_u32(AddressMap::Target, TRIG + 0xD00)?;
        let dbgsr1 = bus.read_u32(AddressMap::Target, TRIG + 0xD00 + 0x2_0000)?;
        println!("  DBGSR CPU0 {dbgsr0:#x}, CPU1 {dbgsr1:#x}");
        let mut core1 = session.core(1)?;
        let halted1 = core1.core_halted()?;
        println!("  CPU1 halted with it: {halted1}");
        core1.run()?;
        drop(core1);
        std::thread::sleep(Duration::from_millis(200));
        let mut core = session.core(0)?;
        let still = core.core_halted()?;
        let pc: u32 = core.read_core_reg(core.program_counter().id())?;
        check_value(
            "group: CPU0 still halted at its watchpoint after CPU1 ran",
            u32::from(still && pc as u64 == AFTER_STORE),
            1,
            &failed,
        );
        println!("  CPU0 halted {still} at {pc:#010x}");
    }
    for core in 0..2 {
        session.set_tricore_halt_group(core, false)?;
    }

    let mut core = session.core(0)?;
    let _ = core.clear_hw_breakpoint(COMPUTE);
    let _ = core.clear_hw_breakpoint(MAIN_CALL);
    core.clear_watchpoint(G_RESULT)?;
    if core.core_halted()? {
        core.run()?;
    }
    println!("{} failed", failed.get());
    Ok(())
}

fn check_value(what: &str, value: u32, want: u32, failed: &std::cell::Cell<usize>) {
    let ok = value == want;
    println!(
        "[{}] {what}: {value}, wanted {want}",
        if ok { "PASS" } else { "FAIL" }
    );
    if !ok {
        failed.set(failed.get() + 1);
    }
}
