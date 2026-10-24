//! Hardware check of TriCore debugging on a TC36x (TriBoard TC3X5) through a TAS server.
//!
//! Halts each core where the application is, reads its registers and scratchpads, single
//! steps, stops at a hardware breakpoint on the halt address and lets the application run
//! again. Nothing is written to memory; with `--reset` the device is also reset and halted,
//! then reset and run.
//!
//! ```text
//! cargo run -p probe-rs-infineon --example tc36x_check -- [--reset]
//! ```

use std::time::Duration;

use probe_rs::{
    CoreStatus, Error, HaltReason, MemoryInterface, Permissions, RegisterId,
    architecture::tricore::{
        bus::{AddressMap, TriCoreError},
        ocds,
        registers::{CSFR_A0, CSFR_ICR, CSFR_PC, CSFR_PCXI},
    },
    probe::{DebugProbeSelector, list::Lister},
};

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

/// CSFR base, global DSPR and size of each core.
const CORES: [(u32, u32); 2] = [(0xF881_0000, 0x7000_0000), (0xF883_0000, 0x6000_0000)];
const DSPR_SIZE: u32 = 0x3_0000;
const SCU_CHIPID: u32 = 0xF003_6140;
const CSFR_CORE_ID: u32 = 0xFE1C;

struct Report {
    failed: usize,
}

impl Report {
    fn check(&mut self, what: &str, ok: bool, detail: String) {
        println!("[{}] {what}: {detail}", if ok { "PASS" } else { "FAIL" });
        if !ok {
            self.failed += 1;
        }
    }
}

fn main() -> Result<()> {
    let reset = std::env::args().any(|a| a == "--reset");
    probe_rs_infineon::register_plugin();
    let selector: DebugProbeSelector = std::env::var("PROBE_RS_PROBE")
        .unwrap_or_else(|_| "058b:0000".to_string())
        .parse()?;
    let probe = Lister::new().open(selector)?;
    println!("{}", probe.get_name());
    let mut session = probe.attach("TC36x", Permissions::default())?;
    let mut report = Report { failed: 0 };

    let bus = session.get_tricore_bus()?;
    let chip_id = bus.read_u32(AddressMap::Target, SCU_CHIPID)?;
    let stm_a = bus.read_u32(AddressMap::Target, ocds::STM0_TIM0SV_TC3XX)?;
    std::thread::sleep(Duration::from_millis(20));
    let stm_b = bus.read_u32(AddressMap::Target, ocds::STM0_TIM0SV_TC3XX)?;
    report.check("SCU_CHIPID", chip_id != 0, format!("{chip_id:#010x}"));
    report.check(
        "STM0 (0xF0001050) counts",
        stm_b != stm_a,
        format!("{stm_a:#010x} -> {stm_b:#010x}"),
    );
    for (index, (csfr, _)) in CORES.iter().enumerate() {
        let core_id = bus.read_u32(AddressMap::Target, csfr + CSFR_CORE_ID)?;
        let dbgsr = bus.read_u32(AddressMap::Target, csfr + ocds::DBGSR as u32)?;
        report.check(
            &format!("CPU{index} CORE_ID"),
            core_id & 0x7 == index as u32,
            format!("{core_id:#x}, DBGSR {dbgsr:#010x}"),
        );
    }

    for (index, (_, dspr)) in CORES.iter().enumerate() {
        let mut core = session.core(index)?;
        let before = core.status()?;
        if before == CoreStatus::Sleeping {
            // Boot halt: the application has not started the core. A halt leaves it sleeping,
            // a step is refused, and its registers can be read.
            let halt = core.halt(Duration::from_millis(500)).is_ok();
            let still = core.status()? == CoreStatus::Sleeping;
            let step = core.step();
            let pc: u32 = core.read_core_reg(RegisterId(CSFR_PC))?;
            report.check(
                &format!("CPU{index} in boot halt"),
                halt && still
                    && matches!(step, Err(Error::TriCore(TriCoreError::CoreInBootHalt(_)))),
                format!("sleeping, halt leaves it, step refused, PC {pc:#010x}"),
            );
            continue;
        }
        let info = core.halt(Duration::from_millis(500))?;
        let halted = core.status()? == CoreStatus::Halted(HaltReason::Request);
        report.check(
            &format!("CPU{index} halt"),
            halted,
            format!("was {before:?}, PC {:#010x}", info.pc),
        );
        let pcxi: u32 = core.read_core_reg(RegisterId(CSFR_PCXI))?;
        let icr: u32 = core.read_core_reg(RegisterId(CSFR_ICR))?;
        let sp: u32 = core.read_core_reg(RegisterId(CSFR_A0 + 4 * 10))?;
        println!("       PCXI {pcxi:#010x}, ICR {icr:#010x}, SP {sp:#010x}");

        // The local DSPR window (0xD...) is this core's DSPR; its last word is in range.
        let global = core.read_word_32((*dspr + 0x100) as u64)?;
        let local = core.read_word_32(0xD000_0100)?;
        let last = core.read_word_32((*dspr + DSPR_SIZE - 4) as u64);
        report.check(
            &format!("CPU{index} DSPR local alias, 192 KiB"),
            global == local && last.is_ok(),
            format!("{global:#010x} == {local:#010x}, last word {last:x?}"),
        );

        let mut pcs = vec![info.pc];
        for _ in 0..5 {
            pcs.push(core.step()?.pc);
        }
        let status = core.status()?;
        report.check(
            &format!("CPU{index} 5 single steps"),
            status == CoreStatus::Halted(HaltReason::Step),
            format!("{pcs:#x?}"),
        );

        let pc: u32 = core.read_core_reg(RegisterId(CSFR_PC))?;
        core.set_hw_breakpoint(pc as u64)?;
        core.run()?;
        let hit = core.wait_for_core_halted(Duration::from_secs(2)).is_ok();
        let status = core.status()?;
        let at: u32 = core.read_core_reg(RegisterId(CSFR_PC))?;
        report.check(
            &format!("CPU{index} hardware breakpoint at {pc:#010x}"),
            hit && at == pc,
            format!("{status:?}, PC {at:#010x}"),
        );
        core.clear_all_hw_breakpoints()?;
        core.run()?;
        let running = core.status()? == CoreStatus::Running;
        report.check(&format!("CPU{index} run again"), running, String::new());
    }

    if reset {
        let mut core = session.core(0)?;
        let info = core.reset_and_halt(Duration::from_millis(500))?;
        report.check(
            "reset and halt",
            core.core_halted()?,
            format!("PC {:#010x}", info.pc),
        );
        core.reset()?;
        std::thread::sleep(Duration::from_millis(300));
        let running = core.status()? == CoreStatus::Running;
        report.check("reset and run", running, String::new());
    }
    drop(session);

    println!("{} check(s) failed", report.failed);
    if report.failed > 0 {
        std::process::exit(1);
    }
    Ok(())
}
