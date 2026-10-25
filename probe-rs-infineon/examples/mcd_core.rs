//! Controls one core through an MCD library: its state, run, stop and step, its registers as
//! the library maps them, and breakpoints as MCD triggers. Nothing goes through the OCDS
//! registers of probe-rs, so it works for any core the library offers.
//!
//! ```text
//! cargo run -p probe-rs-infineon --features mcd --example mcd_core -- "<library>[|<config>]" <command>...
//! ```
//!
//! The configuration picks the core as for the MCD probe (`ProbeRsCore=1`). Each further
//! argument is one command, run in order:
//!
//! ```text
//! info                 the core, its register groups and what its triggers can do
//! state                the execution state
//! halt | run           stop or start the core (this core only)
//! step [n]             n instructions (1)
//! wait [ms]            until the core is stopped (2000 ms)
//! sleep <ms>
//! regs [group]         the register groups, or the registers of one (ID or name) with values
//! reg <name>[=value]   read or write a register
//! break <address>      a hardware breakpoint; prints its trigger ID
//! unbreak [id]         remove one, or all this run set (the run removes them when it ends)
//! mem <address> [n]    n 32-bit words of the device memory (1)
//! ```
//!
//! For example, on the TC275 with the flash demo halted at main by `start_in_flash`:
//!
//! ```text
//! mcd_core "C:\Program Files\DAS64\clients\mcdxdas.dll|ProbeRsCore=0" state "reg PC" "step 3" "reg PC"
//! ```

use std::time::{Duration, Instant};

use probe_rs::{
    architecture::tricore::bus::{AddressMap, TriCoreBusAccess},
    probe::DebugProbe,
};
use probe_rs_infineon::mcd::{McdEndpoint, McdProbe};

type Error = Box<dyn std::error::Error>;

fn number(text: &str) -> Result<u64, Error> {
    let text = text.trim();
    Ok(
        match text.strip_prefix("0x").or_else(|| text.strip_prefix("0X")) {
            Some(hex) => u64::from_str_radix(&hex.replace('_', ""), 16)?,
            None => text.replace('_', "").parse()?,
        },
    )
}

fn main() -> Result<(), Error> {
    tracing_subscriber::fmt()
        .with_env_filter(tracing_subscriber::EnvFilter::from_default_env())
        .init();
    let mut args = std::env::args().skip(1);
    let serial = args.next().unwrap_or_default();
    let commands = args.collect::<Vec<_>>();
    let endpoint = McdEndpoint::parse(&serial, |name| std::env::var(name).ok())?;
    let mut probe = McdProbe::load(endpoint)?;
    probe.attach()?;
    println!(
        "core '{}' on {}",
        probe.core_name(),
        probe.access_hardware()
    );
    let mut breakpoints = Vec::new();
    let result = commands
        .iter()
        .try_for_each(|command| run(&mut probe, &mut breakpoints, command));
    if result.is_err() {
        println!();
    }
    // The breakpoints of this run do not outlive it.
    for id in breakpoints {
        match probe.remove_trigger(id) {
            Ok(()) => println!("trigger {id:#x} removed"),
            Err(error) => println!("trigger {id:#x} not removed: {error}"),
        }
    }
    probe.detach()?;
    result
}

fn run(probe: &mut McdProbe, breakpoints: &mut Vec<u32>, command: &str) -> Result<(), Error> {
    let words = command.split_whitespace().collect::<Vec<_>>();
    print!("> {command}: ");
    match words[..] {
        ["info"] => {
            println!("run control {}", probe.has_run_control());
            match probe.register_groups() {
                Ok(groups) => {
                    for group in groups {
                        println!(
                            "    group {} '{}', {} register(s)",
                            group.id, group.name, group.registers
                        );
                    }
                }
                Err(error) => println!("    register groups: {error}"),
            }
            match probe.trigger_info() {
                Ok(info) => println!(
                    "    triggers: types {:#x}, actions {:#x}, {} trigger(s), software breakpoints {}",
                    info.types, info.actions, info.count, info.software_breakpoints
                ),
                Err(error) => println!("    triggers: {error}"),
            }
        }
        ["state"] => {
            let state = probe.core_state()?;
            println!(
                "{}{}{}{}",
                state.execution,
                if state.stopped_since() {
                    ", stopped since the last query"
                } else {
                    ""
                },
                if state.trigger != 0 {
                    format!(", by trigger {:#x}", state.trigger)
                } else {
                    String::new()
                },
                [state.stop_reason, state.info]
                    .iter()
                    .filter(|t| !t.is_empty())
                    .map(|t| format!(" ({t})"))
                    .collect::<String>()
            );
        }
        ["halt"] => {
            probe.stop_core(false)?;
            println!("ok");
        }
        ["run"] => {
            probe.run_core(false)?;
            println!("ok");
        }
        ["step"] | ["step", _] => {
            let count = words.get(1).map(|n| number(n)).transpose()?.unwrap_or(1);
            probe.step_instructions(u32::try_from(count)?)?;
            println!("ok");
        }
        ["wait"] | ["wait", _] => {
            let ms = words.get(1).map(|n| number(n)).transpose()?.unwrap_or(2000);
            let start = Instant::now();
            loop {
                let state = probe.core_state()?;
                if state.execution.is_stopped() {
                    println!(
                        "{} after {} ms{}",
                        state.execution,
                        start.elapsed().as_millis(),
                        if state.trigger != 0 {
                            format!(", by trigger {:#x}", state.trigger)
                        } else {
                            String::new()
                        }
                    );
                    break;
                }
                if start.elapsed() > Duration::from_millis(ms) {
                    return Err(format!("still {} after {ms} ms", state.execution).into());
                }
                std::thread::sleep(Duration::from_millis(10));
            }
        }
        ["sleep", ms] => {
            std::thread::sleep(Duration::from_millis(number(ms)?));
            println!("ok");
        }
        ["regs"] => {
            println!();
            for group in probe.register_groups()? {
                println!(
                    "    group {} '{}', {} register(s)",
                    group.id, group.name, group.registers
                );
            }
        }
        ["regs", group] => {
            let groups = probe.register_groups()?;
            let id = match number(group) {
                Ok(id) => u32::try_from(id)?,
                Err(_) => {
                    groups
                        .iter()
                        .find(|g| g.name.to_lowercase().contains(&group.to_lowercase()))
                        .ok_or_else(|| format!("no register group '{group}'"))?
                        .id
                }
            };
            let registers = probe.registers(id)?;
            println!("{} register(s)", registers.len());
            for register in registers {
                let value = if register.bits <= 64 {
                    match probe.read_register_u64(&register) {
                        Ok(value) => format!("{value:#0width$x}", width = register.bytes() * 2 + 2),
                        Err(error) => format!("({error})"),
                    }
                } else {
                    match probe.read_register(&register) {
                        Ok(bytes) => bytes.iter().rev().map(|b| format!("{b:02x}")).collect(),
                        Err(error) => format!("({error})"),
                    }
                };
                println!(
                    "    {:<12} {value}   ({} bits at {:#x}, space {})",
                    register.name, register.bits, register.address, register.space
                );
            }
        }
        ["reg", what] => match what.split_once('=') {
            Some((name, value)) => {
                let register = probe.find_register(name)?;
                probe.write_register_u64(&register, number(value)?)?;
                let back = probe.read_register_u64(&register)?;
                println!("{} = {back:#x}", register.name);
            }
            None => {
                let register = probe.find_register(what)?;
                let value = probe.read_register_u64(&register)?;
                println!("{} = {value:#x}", register.name);
            }
        },
        ["break", address] => {
            let id = probe.set_breakpoint(number(address)?)?;
            breakpoints.push(id);
            println!("trigger {id:#x}");
        }
        ["unbreak"] => {
            for id in std::mem::take(breakpoints) {
                probe.remove_trigger(id)?;
            }
            println!("ok");
        }
        ["unbreak", id] => {
            let id = u32::try_from(number(id)?)?;
            probe.remove_trigger(id)?;
            breakpoints.retain(|b| *b != id);
            println!("ok");
        }
        ["mem", address] | ["mem", address, _] => {
            let address = u32::try_from(number(address)?)?;
            let count = words.get(2).map(|n| number(n)).transpose()?.unwrap_or(1);
            let mut bytes = vec![0; 4 * count as usize];
            probe.read_block(AddressMap::Target, address, &mut bytes)?;
            let words = bytes
                .chunks(4)
                .map(|w| {
                    format!(
                        "{:08x}",
                        u32::from_le_bytes(w.try_into().unwrap_or_default())
                    )
                })
                .collect::<Vec<_>>();
            println!("{}", words.join(" "));
        }
        _ => {
            return Err(
                format!("`{command}` is not a command (see the example's documentation)").into(),
            );
        }
    }
    Ok(())
}
