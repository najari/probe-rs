//! Checks the start and stop of the TC27x flash loader on the board, without programming: the
//! loader is started with `prepare_flash` and stopped with `finish_flash`, and no flash command
//! is sent in between, so the flash does not change.
//!
//! ```text
//! cargo run -p probe-rs-infineon --example loader_check -- wdt
//! cargo run -p probe-rs-infineon --example loader_check -- start-failure
//! ```
//!
//! `wdt` reads both watchdogs before the loader, while it runs and after it: the loader sets the
//! disable request (`CON1.DR`) while it runs and puts it back. Start from a reset and halt (the
//! `mcd_core` example's `reset`), where both `CON1` are 0; a running application has usually
//! disabled the watchdogs already.
//!
//! `start-failure` makes the start fail: a hardware breakpoint on a loader instruction before it
//! writes its magic (PSPR + 0x28, after it cleared the mailbox) keeps it from starting, so
//! `prepare_flash` fails with "the flash loader did not start" and puts back what it changed.
//! CPU0's registers, the loader's RAM, the watchdogs and the state of every core are compared
//! before and after. CPU0 must be halted before (`start_in_flash` or a reset and halt): the RAM
//! of a running CPU0 changes by itself.

use probe_rs::{
    CoreStatus, MemoryInterface, Permissions, Session,
    architecture::tricore::registers::TRICORE_CORE_REGISTERS, flashing::DebugFlashSequence,
    probe::list::Lister,
};
use probe_rs_infineon::flash::{AurixFlash, FlashDevice};

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

/// The RAM the loader uses (`LOADER_RAM` of the flasher): DSPR, then PSPR.
const LOADER_RAM: [(u64, usize); 2] = [(0x7000_A000, 0x6000), (0x7010_0000, 0x2000)];
/// The TC27x watchdog registers: CPU0 `CON0`, `CON1`, safety `CON0`, `CON1`.
const WATCHDOGS: [(&str, u64); 4] = [
    ("WDTCPU0_CON0", 0xF003_6100),
    ("WDTCPU0_CON1", 0xF003_6104),
    ("WDTS_CON0", 0xF003_60F0),
    ("WDTS_CON1", 0xF003_60F4),
];
/// A loader instruction after it cleared the mailbox and before it writes its magic
/// (`movh %d2, 0x4d32` of `m25_flash_loader_tc27x.bin`).
const BEFORE_MAGIC: u64 = 0x7010_0028;

/// What the loader start changes and `stop_loader` puts back.
struct State {
    registers: Vec<(&'static str, u32)>,
    ram: Vec<(u64, Vec<u8>)>,
    watchdogs: Vec<(&'static str, u32)>,
    cores: Vec<String>,
}

fn watchdogs(session: &mut Session) -> Result<Vec<(&'static str, u32)>> {
    let mut core = session.core(0)?;
    WATCHDOGS
        .iter()
        .map(|&(name, address)| Ok((name, core.read_word_32(address)?)))
        .collect()
}

fn state(session: &mut Session) -> Result<State> {
    let mut cores = Vec::new();
    for id in 0..session.list_cores().len() {
        let status = session.core(id)?.status()?;
        cores.push(match status {
            CoreStatus::Halted(_) => "halted".to_string(),
            other => format!("{other:?}").to_lowercase(),
        });
    }
    let watchdogs = watchdogs(session)?;
    let mut core = session.core(0)?;
    let mut registers = Vec::new();
    for register in TRICORE_CORE_REGISTERS.all_registers() {
        registers.push((register.name(), core.read_core_reg::<u32>(register.id)?));
    }
    let mut ram = Vec::new();
    for (address, size) in LOADER_RAM {
        let mut data = vec![0; size];
        core.read(address, &mut data)?;
        ram.push((address, data));
    }
    Ok(State {
        registers,
        ram,
        watchdogs,
        cores,
    })
}

/// Prints what differs; returns the number of differences.
fn compare(before: &State, after: &State) -> usize {
    let mut differences = 0;
    for ((name, a), (_, b)) in before.registers.iter().zip(&after.registers) {
        if a != b {
            println!("  register {name}: {a:#010x} -> {b:#010x}");
            differences += 1;
        }
    }
    for ((address, a), (_, b)) in before.ram.iter().zip(&after.ram) {
        let changed = a
            .chunks(4)
            .zip(b.chunks(4))
            .enumerate()
            .filter(|(_, (x, y))| x != y)
            .collect::<Vec<_>>();
        for (index, (x, y)) in changed.iter().take(6) {
            println!(
                "  RAM {:#010x}: {:02x?} -> {:02x?}",
                address + 4 * *index as u64,
                x,
                y
            );
        }
        if !changed.is_empty() {
            println!("  RAM at {address:#010x}: {} word(s) differ", changed.len());
        }
        differences += changed.len();
    }
    for ((name, a), (_, b)) in before.watchdogs.iter().zip(&after.watchdogs) {
        if a != b {
            println!("  {name}: {a:#010x} -> {b:#010x}");
            differences += 1;
        }
    }
    for (id, (a, b)) in before.cores.iter().zip(&after.cores).enumerate() {
        if a != b {
            println!("  core {id}: {a} -> {b}");
            differences += 1;
        }
    }
    differences
}

fn print_watchdogs(when: &str, values: &[(&str, u32)]) {
    let text = values
        .iter()
        .map(|(name, value)| format!("{name} {value:#010x}"))
        .collect::<Vec<_>>()
        .join(", ");
    println!("{when:<7} {text}");
}

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(tracing_subscriber::EnvFilter::from_default_env())
        .init();
    let mode = std::env::args().nth(1).unwrap_or_default();
    probe_rs_infineon::register_plugin();
    let selector = std::env::var("PROBE_RS_PROBE").unwrap_or_else(|_| "058b:0000".to_string());
    let probe = Lister::new().open(selector.parse::<probe_rs::probe::DebugProbeSelector>()?)?;
    let mut session = probe.attach("TC275", Permissions::default())?;
    let flash = AurixFlash::new(FlashDevice::Tc27x);

    match mode.as_str() {
        "wdt" => {
            print_watchdogs("before", &watchdogs(&mut session)?);
            flash.prepare_flash(&mut session)?;
            print_watchdogs("running", &watchdogs(&mut session)?);
            flash.finish_flash(&mut session)?;
            print_watchdogs("after", &watchdogs(&mut session)?);
        }
        "start-failure" => {
            let before = state(&mut session)?;
            println!("cores before: {}", before.cores.join(", "));
            if before.cores.first().map(String::as_str) != Some("halted") {
                return Err("CPU0 must be halted (start_in_flash, or a reset and halt)".into());
            }
            session.core(0)?.set_hw_breakpoint(BEFORE_MAGIC)?;
            match flash.prepare_flash(&mut session) {
                Ok(()) => {
                    // Not expected: stop the loader again before anything else.
                    println!("prepare_flash: ok (the breakpoint did not stop the loader)");
                    flash.finish_flash(&mut session)?;
                }
                Err(error) => println!("prepare_flash: {error}"),
            }
            let pc = session
                .core(0)?
                .read_core_reg::<u32>(probe_rs::RegisterId(0xFE08))?;
            session.core(0)?.clear_hw_breakpoint(BEFORE_MAGIC)?;
            let after = state(&mut session)?;
            println!("cores after:  {}", after.cores.join(", "));
            println!("CPU0 PC after: {pc:#010x}");
            let differences = compare(&before, &after);
            println!("{differences} difference(s)");
        }
        _ => return Err("the mode is `wdt` or `start-failure`".into()),
    }
    Ok(())
}
