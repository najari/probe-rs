//! Points CPU0 of an AURIX (TC275, TC36x) at the entry of an image in PFLASH or RAM, without a reset.
//!
//! For images the boot code does not start, like the demo in the PF1 test sector: it checks
//! that the flash holds the image (segments in RAM, like the demo in the CPU0 PSPR, are written
//! instead), halts CPU0 and sets PC = entry, A10 = the stack top, A11 = PCXI = 0 (the entry is
//! the outermost frame) and clears ICR.IE. CPU0 stays halted, so a debugger can attach (DAP
//! `attach` request) and set breakpoints. The application's context on CPU0 is abandoned:
//! reset the device afterwards.
//!
//! ```text
//! cargo run -p probe-rs-infineon --example start_in_flash -- <elf> [stack top, default 0x7000C000] [--core N]
//! ```
//!
//! `--core N` starts the image on core N instead of CPU0, with the stack in the core's own DSPR
//! (its local alias, default 0xD000C000), so several cores can run the same code (one DAP
//! session for several cores, PRD test T-M8). Each core keeps the context save areas its
//! application set up.
//!
//! A core the application never started (boot halt, `SYSCON.BHALT`, like CPU1 of the TC36x test
//! board) is first taken out of boot halt into a debug halt: a halt request in `DBGSR`, then
//! `BHALT` cleared. It has no context save areas either, so it gets a free list of 256 (16 KiB of
//! its DSPR from local address 0xD0004000, below the demos' data at 0xD000A000) in `FCX`/`LCX`.
//!
//! The chip is detected; `PROBE_RS_CHIP` names it instead, for example `TC36x`.

use std::time::Duration;

use probe_rs::{
    MemoryInterface, Permissions, RegisterId,
    architecture::tricore::registers::{
        CSFR_A0, CSFR_FCX, CSFR_ICR, CSFR_PC, CSFR_PCXI, CSFR_SYSCON, SYSCON_BHALT,
    },
    probe::list::Lister,
};

/// The CSFRs of core `n` on the bus (TC27x and TC3xx).
fn csfr_base(core: usize) -> u64 {
    0xF881_0000 + 0x2_0000 * core as u64
}
/// `DBGSR`, and the value that requests a halt.
const DBGSR: u64 = 0xFD00;
const DBGSR_HALT: u32 = 0x6;
const CSFR_LCX: u64 = 0xFE3C;
/// The context save areas of a core taken out of boot halt: local DSPR addresses.
const CSA_START: u32 = 0xD000_4000;
const CSA_COUNT: u32 = 256;

/// A context save area's link word: segment (bits 19:16) and offset (bits 15:0) of its address.
fn csa_link(address: u32) -> u32 {
    ((address >> 12) & 0xF_0000) | ((address >> 6) & 0xFFFF)
}

/// Takes core `n` out of boot halt into a debug halt and gives it a free list of context save
/// areas. Nothing happens for a core that is not in boot halt.
fn release_boot_halt(core: &mut probe_rs::Core, n: usize) -> Result<bool> {
    let syscon = core.read_word_32(csfr_base(n) + CSFR_SYSCON as u64)?;
    if syscon & SYSCON_BHALT == 0 {
        return Ok(false);
    }
    // The halt request is taken as soon as the core leaves boot halt, before it runs code.
    core.write_word_32(csfr_base(n) + DBGSR, DBGSR_HALT)?;
    core.write_word_32(csfr_base(n) + CSFR_SYSCON as u64, syscon & !SYSCON_BHALT)?;
    core.halt(Duration::from_millis(500))?;

    // The list, each area's first word the link to the next, the last 0. The DSPR of core n is
    // its local 0xD000_0000, and 0x7000_0000 - n * 0x1000_0000 on the bus (CPU0-CPU4).
    let global = |local: u32| (local - 0xD000_0000 + 0x7000_0000 - 0x1000_0000 * n as u32) as u64;
    for i in 0..CSA_COUNT {
        let area = CSA_START + 64 * i;
        let next = if i + 1 < CSA_COUNT {
            csa_link(area + 64)
        } else {
            0
        };
        core.write_word_32(global(area), next)?;
    }
    core.write_word_32(csfr_base(n) + CSFR_FCX as u64, csa_link(CSA_START))?;
    // LCX: the depletion trap comes with a few areas left for the trap handler.
    core.write_word_32(
        csfr_base(n) + CSFR_LCX,
        csa_link(CSA_START + 64 * (CSA_COUNT - 4)),
    )?;
    println!(
        "CPU{n} was in boot halt: halted, {CSA_COUNT} context save areas at {CSA_START:#010x}"
    );
    Ok(true)
}

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

fn main() -> Result<()> {
    let mut args = std::env::args().skip(1).collect::<Vec<_>>();
    let core_index = match args.iter().position(|a| a == "--core") {
        Some(i) => {
            let core = args
                .get(i + 1)
                .ok_or("--core needs a core index")?
                .parse()?;
            args.drain(i..i + 2);
            core
        }
        None => 0usize,
    };
    let mut args = args.into_iter();
    let path = args
        .next()
        .ok_or("usage: start_in_flash <elf> [stack top] [--core N]")?;
    let stack = match args.next() {
        Some(s) => u32::from_str_radix(s.trim_start_matches("0x"), 16)?,
        None if core_index == 0 => 0x7000_C000,
        None => 0xD000_C000,
    };

    let elf = std::fs::read(&path)?;
    if elf.get(..4) != Some(b"\x7fELF".as_slice()) || elf[4] != 1 || elf[5] != 1 {
        return Err("not an ELF32 little-endian file".into());
    }
    let u16_at = |o: usize| u16::from_le_bytes([elf[o], elf[o + 1]]) as usize;
    let u32_at = |o: usize| u32::from_le_bytes(elf[o..o + 4].try_into().unwrap());
    let entry = u32_at(24);

    probe_rs_infineon::register_plugin();
    // `PROBE_RS_PROBE` selects another probe, for example the MCD probe.
    let selector = std::env::var("PROBE_RS_PROBE").unwrap_or_else(|_| "058b:0000".to_string());
    let probe = Lister::new().open(selector.parse::<probe_rs::probe::DebugProbeSelector>()?)?;
    let chip = std::env::var("PROBE_RS_CHIP")
        .map(probe_rs::config::TargetSelector::from)
        .unwrap_or(probe_rs::config::TargetSelector::Auto);
    let mut session = probe.attach(chip, Permissions::default())?;
    println!("{}", session.target().name);
    let mut core = session.core(core_index)?;
    if core_index > 0 && core_index <= 4 {
        release_boot_halt(&mut core, core_index)?;
    }
    core.halt(Duration::from_millis(500))?;

    // A stale image would run garbage.
    let (phoff, phent, phnum) = (u32_at(28) as usize, u16_at(42), u16_at(44));
    for i in 0..phnum {
        let ph = phoff + i * phent;
        if u32_at(ph) != 1 {
            continue;
        }
        let (offset, address, size, mem_size) = (
            u32_at(ph + 4) as usize,
            u32_at(ph + 8),
            u32_at(ph + 16) as usize,
            u32_at(ph + 20) as usize,
        );
        // PFLASH is 0x8... (cached) and 0xA...; the rest is RAM (scratchpads, LMU).
        if !matches!(address >> 28, 0x8 | 0xA) {
            let mut data = elf[offset..offset + size].to_vec();
            data.resize(mem_size, 0);
            core.write(address as u64, &data)?;
            println!("RAM @{address:#010x} {mem_size} B: written");
            continue;
        }
        let mut flash = vec![0; size];
        core.read(address as u64, &mut flash)?;
        if flash != elf[offset..offset + size] {
            return Err(
                format!("flash at {address:#010x} does not hold {path}: flash it first").into(),
            );
        }
        println!("flash @{address:#010x} {size} B: matches");
    }

    core.write_core_reg(RegisterId(CSFR_PC), entry)?;
    core.write_core_reg(RegisterId(CSFR_A0 + 4 * 10), stack)?;
    core.write_core_reg(RegisterId(CSFR_A0 + 4 * 11), 0u32)?;
    core.write_core_reg(RegisterId(CSFR_PCXI), 0u32)?;
    let icr: u32 = core.read_core_reg(RegisterId(CSFR_ICR))?;
    core.write_core_reg(RegisterId(CSFR_ICR), icr & !(1 << 15))?;
    println!(
        "CPU{core_index} halted at {entry:#010x}, A10 = {stack:#010x}, A11 = PCXI = 0, ICR.IE off"
    );
    Ok(())
}
