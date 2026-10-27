//! The HSM core, a Cortex-M3, run through its debug registers in the debug memory window.
//!
//! probe-rs' Armv7m core is built on an ARM debug port (`ArmMemoryInterface`), which the AURIX
//! does not have, so this drives the few registers directly (ARMv7-M Architecture Reference
//! Manual, C1.6): `DHCSR` halts, runs and single steps the core and tells its state, `DCRSR` and
//! `DCRDR` move a core register, `DFSR` tells why the core halted. Breakpoints are FPB
//! comparators with Infineon's high address bits ([`HsmFpb`]).
//!
//! Not tried on a device yet: that writes from the window set `DHCSR.C_DEBUGEN` like an ARM
//! debug port does (the core itself cannot) is assumed; no HSM with HSM debug enabled was at
//! hand. There is no reset here: what an HSM reset does to the device is not documented.

use std::time::{Duration, Instant};

use crate::{Error, MemoryInterface};

use super::{HsmFpb, HsmWindow, other};

/// `DFSR`, the debug fault status: why the core halted (bits are cleared by writing 1).
pub const DFSR: u64 = 0xE000_ED30;
/// `DHCSR`, the debug halting control and status.
pub const DHCSR: u64 = 0xE000_EDF0;
/// `DCRSR`, the core register selector.
pub const DCRSR: u64 = 0xE000_EDF4;
/// `DCRDR`, the core register data.
pub const DCRDR: u64 = 0xE000_EDF8;

/// `DHCSR.DBGKEY`: a write without it is ignored.
const DBGKEY: u32 = 0xA05F << 16;
const C_DEBUGEN: u32 = 1;
const C_HALT: u32 = 1 << 1;
const C_STEP: u32 = 1 << 2;
const C_MASKINTS: u32 = 1 << 3;
const S_REGRDY: u32 = 1 << 16;
const S_HALT: u32 = 1 << 17;
const S_SLEEP: u32 = 1 << 18;
const S_LOCKUP: u32 = 1 << 19;
const S_RESET_ST: u32 = 1 << 25;
/// `DCRSR.REGWnR`: a write of the register.
const REGWNR: u32 = 1 << 16;
/// `DFSR`: `HALTED`, `BKPT`, `DWTTRAP`, `VCATCH`, `EXTERNAL`.
const DFSR_HALTED: u32 = 1;
const DFSR_BKPT: u32 = 1 << 1;
const DFSR_DWTTRAP: u32 = 1 << 2;
const DFSR_VCATCH: u32 = 1 << 3;
const DFSR_EXTERNAL: u32 = 1 << 4;
const DFSR_ALL: u32 = 0x1F;
/// `FP_CTRL.KEY` without `ENABLE`: disables the FPB.
const FP_CTRL_DISABLE: u32 = 0b10;
/// How long a register transfer may take (`S_REGRDY`).
const REGISTER_TIMEOUT: Duration = Duration::from_millis(100);

/// A core register by its `DCRSR.REGSEL` number.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct HsmRegister(u8);

impl HsmRegister {
    /// The stack pointer (R13), the one in use.
    pub const SP: Self = Self(13);
    /// The link register (R14).
    pub const LR: Self = Self(14);
    /// The DebugReturnAddress: where the core goes on when it runs.
    pub const PC: Self = Self(15);
    /// The program status register.
    pub const XPSR: Self = Self(16);
    /// The main stack pointer.
    pub const MSP: Self = Self(17);
    /// The process stack pointer.
    pub const PSP: Self = Self(18);
    /// `CONTROL` (bits 31:24), `FAULTMASK` (23:16), `BASEPRI` (15:8) and `PRIMASK` (7:0).
    pub const SPECIAL: Self = Self(20);

    /// The registers of a Cortex-M3 without FPU, in `REGSEL` order.
    pub const ALL: [Self; 20] = [
        Self(0),
        Self(1),
        Self(2),
        Self(3),
        Self(4),
        Self(5),
        Self(6),
        Self(7),
        Self(8),
        Self(9),
        Self(10),
        Self(11),
        Self(12),
        Self::SP,
        Self::LR,
        Self::PC,
        Self::XPSR,
        Self::MSP,
        Self::PSP,
        Self::SPECIAL,
    ];

    /// R0-R12.
    pub fn r(n: u8) -> Option<Self> {
        (n <= 12).then_some(Self(n))
    }

    /// The `DCRSR.REGSEL` value.
    pub fn regsel(self) -> u32 {
        u32::from(self.0)
    }

    /// The name of the register.
    pub fn name(self) -> String {
        match self.0 {
            0..=12 => format!("R{}", self.0),
            13 => "SP".into(),
            14 => "LR".into(),
            15 => "PC".into(),
            16 => "xPSR".into(),
            17 => "MSP".into(),
            18 => "PSP".into(),
            _ => "CONTROL/FAULTMASK/BASEPRI/PRIMASK".into(),
        }
    }
}

/// The state of the core, from `DHCSR`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HsmCoreStatus {
    /// `C_DEBUGEN`: halting debug is enabled.
    pub debug_enabled: bool,
    /// `S_HALT`.
    pub halted: bool,
    /// `S_SLEEP`: the core waits for an interrupt or event.
    pub sleeping: bool,
    /// `S_LOCKUP`: the core is locked up after an unrecoverable exception.
    pub locked_up: bool,
    /// `S_RESET_ST`: the core was reset since `DHCSR` was last read (cleared by the read).
    pub reset: bool,
}

impl HsmCoreStatus {
    /// From the value of `DHCSR`.
    pub fn from_dhcsr(dhcsr: u32) -> Self {
        Self {
            debug_enabled: dhcsr & C_DEBUGEN != 0,
            halted: dhcsr & S_HALT != 0,
            sleeping: dhcsr & S_SLEEP != 0,
            locked_up: dhcsr & S_LOCKUP != 0,
            reset: dhcsr & S_RESET_ST != 0,
        }
    }
}

/// Why the core halted, from `DFSR`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HsmHaltReason {
    /// `BKPT`: a breakpoint, an FPB comparator or a `BKPT` instruction.
    Breakpoint,
    /// `DWTTRAP`: a DWT watchpoint.
    Watchpoint,
    /// `VCATCH`: a vector catch, for example the reset vector.
    VectorCatch,
    /// `EXTERNAL`: the external debug request, for example a halt of the halt group.
    External,
    /// `HALTED`: a halt request or a single step; `DFSR` does not tell them apart.
    Halted,
    /// No bit is set.
    Unknown,
}

impl HsmHaltReason {
    /// From the value of `DFSR`; with several bits set, the first in this order.
    pub fn from_dfsr(dfsr: u32) -> Self {
        if dfsr & DFSR_BKPT != 0 {
            Self::Breakpoint
        } else if dfsr & DFSR_DWTTRAP != 0 {
            Self::Watchpoint
        } else if dfsr & DFSR_VCATCH != 0 {
            Self::VectorCatch
        } else if dfsr & DFSR_EXTERNAL != 0 {
            Self::External
        } else if dfsr & DFSR_HALTED != 0 {
            Self::Halted
        } else {
            Self::Unknown
        }
    }
}

/// The HSM core through `host`, the bus of a TriCore core.
///
/// It keeps nothing but the window page and the FPB layout: the breakpoints are read back from
/// the FPB, so a core made again (as a session does for every access) sees the same ones.
pub struct HsmCore<H: MemoryInterface> {
    window: HsmWindow<H>,
    /// The FPB, read on its first use.
    fpb: Option<HsmFpb>,
}

impl<H: MemoryInterface> HsmCore<H> {
    /// The HSM core through `host`, the bus of a TriCore core.
    pub fn new(host: H) -> Self {
        Self {
            window: HsmWindow::new(host),
            fpb: None,
        }
    }

    /// The HSM address space.
    pub fn memory(&mut self) -> &mut HsmWindow<H> {
        &mut self.window
    }

    /// The HSM address space, for reading through a shared reference.
    pub fn memory_ref(&self) -> &HsmWindow<H> {
        &self.window
    }

    /// Forgets the window page and the FPB layout: after a reset of the HSM.
    pub fn invalidate(&mut self) {
        self.window.invalidate();
        self.fpb = None;
    }

    /// The state of the core (`DHCSR`).
    pub fn status(&mut self) -> Result<HsmCoreStatus, Error> {
        Ok(HsmCoreStatus::from_dhcsr(self.window.read_word_32(DHCSR)?))
    }

    fn control(&mut self, bits: u32) -> Result<(), Error> {
        self.window.write_word_32(DHCSR, DBGKEY | C_DEBUGEN | bits)
    }

    fn wait(&mut self, timeout: Duration, bit: u32) -> Result<(), Error> {
        let start = Instant::now();
        loop {
            if self.window.read_word_32(DHCSR)? & bit != 0 {
                return Ok(());
            }
            if start.elapsed() >= timeout {
                return Err(Error::Timeout);
            }
            std::thread::sleep(Duration::from_millis(1));
        }
    }

    /// Waits up to `timeout` for the core to halt.
    pub fn wait_halted(&mut self, timeout: Duration) -> Result<(), Error> {
        self.wait(timeout, S_HALT)
    }

    fn require_halted(&mut self, what: &str) -> Result<(), Error> {
        if self.status()?.halted {
            Ok(())
        } else {
            Err(other(format!("the HSM core must be halted to {what}")))
        }
    }

    /// Halts the core (and enables halting debug), waiting up to `timeout` for it.
    pub fn halt(&mut self, timeout: Duration) -> Result<(), Error> {
        if self.status()?.halted {
            return Ok(());
        }
        self.control(C_HALT)?;
        self.wait_halted(timeout)
    }

    /// Lets the core run, with interrupts as the program has them.
    pub fn run(&mut self) -> Result<(), Error> {
        if !self.status()?.halted {
            return Ok(());
        }
        self.window.write_word_32(DFSR, DFSR_ALL)?;
        // C_MASKINTS may only change while halted.
        self.control(C_HALT)?;
        self.control(0)
    }

    /// Disables halting debug (`C_DEBUGEN` cleared): a halted core runs on. The comparators are
    /// left as they are.
    pub fn disable_debug(&mut self) -> Result<(), Error> {
        self.window.write_word_32(DFSR, DFSR_ALL)?;
        self.window.write_word_32(DHCSR, DBGKEY)
    }

    /// Executes one instruction with interrupts masked and returns the new PC. A breakpoint at
    /// the PC is stepped over: the FPB is disabled for the step.
    pub fn step(&mut self, timeout: Duration) -> Result<u32, Error> {
        self.require_halted("single step")?;
        let pc = self.read_register(HsmRegister::PC)?;
        let over_breakpoint =
            self.breakpoints_enabled()? && self.hw_breakpoints()?.contains(&Some(pc));
        if over_breakpoint {
            self.window
                .write_word_32(HsmFpb::FP_CTRL, FP_CTRL_DISABLE)?;
        }
        self.window.write_word_32(DFSR, DFSR_ALL)?;
        let stepped = self
            .control(C_HALT | C_MASKINTS)
            .and_then(|()| self.control(C_STEP | C_MASKINTS))
            .and_then(|()| self.wait_halted(timeout));
        // Put back what the step changed, also after it failed.
        let unmasked = self.control(C_HALT);
        let enabled = if over_breakpoint {
            self.set_breakpoints_enabled(true)
        } else {
            Ok(())
        };
        stepped.and(unmasked).and(enabled)?;
        self.read_register(HsmRegister::PC)
    }

    /// Why the core halted.
    pub fn halt_reason(&mut self) -> Result<HsmHaltReason, Error> {
        Ok(HsmHaltReason::from_dfsr(self.window.read_word_32(DFSR)?))
    }

    /// Reads a core register; the core must be halted.
    pub fn read_register(&mut self, register: HsmRegister) -> Result<u32, Error> {
        self.require_halted("read a register")?;
        self.window.write_word_32(DCRSR, register.regsel())?;
        self.wait(REGISTER_TIMEOUT, S_REGRDY)?;
        self.window.read_word_32(DCRDR)
    }

    /// Writes a core register; the core must be halted.
    pub fn write_register(&mut self, register: HsmRegister, value: u32) -> Result<(), Error> {
        self.require_halted("write a register")?;
        self.window.write_word_32(DCRDR, value)?;
        self.window
            .write_word_32(DCRSR, register.regsel() | REGWNR)?;
        self.wait(REGISTER_TIMEOUT, S_REGRDY)
    }

    fn fpb(&mut self) -> Result<HsmFpb, Error> {
        if let Some(fpb) = self.fpb {
            return Ok(fpb);
        }
        let fpb = HsmFpb::read(&mut self.window)?;
        self.fpb = Some(fpb);
        Ok(fpb)
    }

    /// The code comparators usable as breakpoints.
    pub fn breakpoint_units(&mut self) -> Result<usize, Error> {
        Ok(self.fpb()?.comparators())
    }

    /// Whether the FPB is enabled (`FP_CTRL.ENABLE`).
    pub fn breakpoints_enabled(&mut self) -> Result<bool, Error> {
        Ok(self.window.read_word_32(HsmFpb::FP_CTRL)? & 1 != 0)
    }

    /// Enables or disables the FPB; the comparators keep their addresses.
    pub fn set_breakpoints_enabled(&mut self, enabled: bool) -> Result<(), Error> {
        let fpb = self.fpb()?;
        if enabled {
            fpb.enable(&mut self.window)
        } else {
            self.window.write_word_32(HsmFpb::FP_CTRL, FP_CTRL_DISABLE)
        }
    }

    /// The address each comparator breaks on, read from the FPB.
    pub fn hw_breakpoints(&mut self) -> Result<Vec<Option<u32>>, Error> {
        let fpb = self.fpb()?;
        (0..fpb.comparators())
            .map(|unit| fpb.breakpoint(&mut self.window, unit))
            .collect()
    }

    /// Sets comparator `unit` to break at `address`.
    pub fn set_hw_breakpoint_unit(&mut self, unit: usize, address: u32) -> Result<(), Error> {
        let fpb = self.fpb()?;
        fpb.set_breakpoint(&mut self.window, unit, address)
    }

    /// Clears comparator `unit`.
    pub fn clear_hw_breakpoint_unit(&mut self, unit: usize) -> Result<(), Error> {
        let fpb = self.fpb()?;
        fpb.clear_breakpoint(&mut self.window, unit)
    }

    /// Sets a breakpoint at `address` on a free comparator and enables the FPB; one that is set
    /// already keeps its comparator. Returns the comparator.
    pub fn set_hw_breakpoint(&mut self, address: u32) -> Result<usize, Error> {
        let breakpoints = self.hw_breakpoints()?;
        let unit = match breakpoints.iter().position(|&b| b == Some(address)) {
            Some(unit) => unit,
            None => {
                let unit = breakpoints
                    .iter()
                    .position(Option::is_none)
                    .ok_or_else(|| {
                        other(format!(
                            "all {} HSM breakpoints are in use",
                            breakpoints.len()
                        ))
                    })?;
                self.set_hw_breakpoint_unit(unit, address)?;
                unit
            }
        };
        self.set_breakpoints_enabled(true)?;
        Ok(unit)
    }

    /// Clears the breakpoint at `address`, if there is one.
    pub fn clear_hw_breakpoint(&mut self, address: u32) -> Result<(), Error> {
        let breakpoints = self.hw_breakpoints()?;
        match breakpoints.iter().position(|&b| b == Some(address)) {
            Some(unit) => self.clear_hw_breakpoint_unit(unit),
            None => Ok(()),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{
        super::{
            HostRef,
            fake::{Cpu, Host},
        },
        *,
    };

    const TIMEOUT: Duration = Duration::from_millis(20);

    fn core(host: &mut Host) -> HsmCore<HostRef<'_, Host>> {
        HsmCore::new(HostRef(host))
    }

    #[test]
    fn a_running_core_is_halted_and_reports_why() {
        let mut host = Host::with_cpu(Cpu::default());
        let mut core = core(&mut host);
        let status = core.status().unwrap();
        assert!(!status.debug_enabled && !status.halted);
        core.halt(TIMEOUT).unwrap();
        assert!(core.status().unwrap().halted);
        assert_eq!(core.halt_reason().unwrap(), HsmHaltReason::Halted);
        // Halted already: nothing more is written.
        core.halt(TIMEOUT).unwrap();
        assert_eq!(host.hsm_writes(0xE000_EDF0), [0xA05F_0003]);
    }

    #[test]
    fn a_halt_that_is_not_answered_times_out() {
        let mut host = Host::with_cpu(Cpu {
            ignores_halt: true,
            ..Cpu::default()
        });
        let mut core = core(&mut host);
        assert!(matches!(core.halt(TIMEOUT), Err(Error::Timeout)));
    }

    #[test]
    fn disabling_debug_lets_a_halted_core_run() {
        let mut host = Host::with_cpu(Cpu::halted_at(0x8000_4000));
        core(&mut host).disable_debug().unwrap();
        let cpu = host.cpu.as_ref().unwrap();
        assert!(!cpu.debugen && !cpu.halted);
    }

    #[test]
    fn registers_are_moved_through_dcrsr_and_dcrdr() {
        let mut cpu = Cpu::halted_at(0x8000_4000);
        cpu.regs[0] = 0x1234_5678;
        cpu.regs[20] = 0x0000_0001;
        let mut host = Host::with_cpu(cpu);
        let mut core = core(&mut host);
        assert_eq!(core.read_register(HsmRegister::PC).unwrap(), 0x8000_4000);
        assert_eq!(
            core.read_register(HsmRegister::r(0).unwrap()).unwrap(),
            0x1234_5678
        );
        assert_eq!(core.read_register(HsmRegister::SPECIAL).unwrap(), 1);
        core.write_register(HsmRegister::r(1).unwrap(), 0xCAFE_F00D)
            .unwrap();
        assert_eq!(host.cpu.as_ref().unwrap().regs[1], 0xCAFE_F00D);
        assert_eq!(host.hsm_writes(0xE000_EDF4), [15, 0, 20, 1 | (1 << 16)]);

        assert_eq!(HsmRegister::r(13), None);
        assert_eq!(HsmRegister::ALL.len(), 20);
        assert_eq!(HsmRegister::XPSR.name(), "xPSR");
    }

    #[test]
    fn registers_of_a_running_core_are_refused() {
        let mut host = Host::with_cpu(Cpu {
            debugen: true,
            ..Cpu::default()
        });
        let mut core = core(&mut host);
        assert!(core.read_register(HsmRegister::PC).is_err());
        assert!(core.write_register(HsmRegister::PC, 0).is_err());
        assert!(core.step(TIMEOUT).is_err());
        assert!(host.hsm_writes(0xE000_EDF4).is_empty());
    }

    #[test]
    fn a_step_masks_interrupts_only_while_it_steps() {
        let mut host = Host::with_cpu(Cpu::halted_at(0x8000_4000));
        let mut core = core(&mut host);
        assert_eq!(core.step(TIMEOUT).unwrap(), 0x8000_4002);
        assert_eq!(core.step(TIMEOUT).unwrap(), 0x8000_4004);
        let cpu = host.cpu.as_ref().unwrap();
        assert_eq!(cpu.steps, [true, true], "masked while stepping");
        assert!(!cpu.maskints && cpu.halted, "unmasked and halted after");
        assert_eq!(
            host.hsm_writes(0xE000_EDF0),
            [0xA05F_000B, 0xA05F_000D, 0xA05F_0003].repeat(2)
        );
    }

    #[test]
    fn a_step_on_a_breakpoint_steps_over_it() {
        let mut host = Host::with_cpu(Cpu::halted_at(0x8000_4000));
        let mut core = core(&mut host);
        core.set_hw_breakpoint(0x8000_4000).unwrap();
        assert_eq!(core.step(TIMEOUT).unwrap(), 0x8000_4002);
        // The FPB was off for the step and is on again.
        assert_eq!(host.hsm_writes(0xE000_2000), [0b11, 0b10, 0b11]);
        assert_eq!(host.hsm[&0xE000_2000] & 1, 1);
    }

    #[test]
    fn a_run_stops_at_a_breakpoint_in_the_flash() {
        let mut host = Host::with_cpu(Cpu::halted_at(0x8000_4000));
        let mut core = core(&mut host);
        assert_eq!(core.set_hw_breakpoint(0x8000_4100).unwrap(), 0);
        core.run().unwrap();
        assert!(core.status().unwrap().halted);
        assert_eq!(core.halt_reason().unwrap(), HsmHaltReason::Breakpoint);
        assert_eq!(core.read_register(HsmRegister::PC).unwrap(), 0x8000_4100);
        // FP_COMPH0 holds the high bits of the flash address.
        assert_eq!(host.hsm[&0xE000_2028], 0b100);
        // DFSR was cleared before the run.
        assert_eq!(host.hsm_writes(0xE000_ED30), [0x1F]);
    }

    #[test]
    fn breakpoints_take_free_comparators_and_are_read_back() {
        let mut host = Host::with_cpu(Cpu::halted_at(0x8000_4000));
        let mut core = core(&mut host);
        for n in 0..6 {
            assert_eq!(
                core.set_hw_breakpoint(0x8000_4000 + 4 * n).unwrap(),
                n as usize
            );
        }
        assert_eq!(
            core.set_hw_breakpoint(0x8000_4008).unwrap(),
            2,
            "set already"
        );
        assert!(
            core.set_hw_breakpoint(0x8000_5000).is_err(),
            "all six in use"
        );
        core.clear_hw_breakpoint(0x8000_4008).unwrap();
        core.clear_hw_breakpoint(0x8000_9000).unwrap();
        assert_eq!(core.hw_breakpoints().unwrap()[2], None);
        assert_eq!(core.set_hw_breakpoint(0x2000_1000).unwrap(), 2);
        // A core made again reads the same breakpoints from the FPB.
        let mut again = HsmCore::new(HostRef(&mut host));
        assert_eq!(again.breakpoint_units().unwrap(), 6);
        assert_eq!(
            again.hw_breakpoints().unwrap(),
            [
                Some(0x8000_4000),
                Some(0x8000_4004),
                Some(0x2000_1000),
                Some(0x8000_400C),
                Some(0x8000_4010),
                Some(0x8000_4014),
            ]
        );
        assert!(again.breakpoints_enabled().unwrap());
        assert_eq!(host.hsm[&0xE000_2030], 0b001, "FP_COMPH2 for the RAM");
    }

    #[test]
    fn halt_reasons_follow_dfsr() {
        assert_eq!(HsmHaltReason::from_dfsr(0b00011), HsmHaltReason::Breakpoint);
        assert_eq!(HsmHaltReason::from_dfsr(0b00100), HsmHaltReason::Watchpoint);
        assert_eq!(
            HsmHaltReason::from_dfsr(0b01000),
            HsmHaltReason::VectorCatch
        );
        assert_eq!(HsmHaltReason::from_dfsr(0b10000), HsmHaltReason::External);
        assert_eq!(HsmHaltReason::from_dfsr(0b00001), HsmHaltReason::Halted);
        assert_eq!(HsmHaltReason::from_dfsr(0), HsmHaltReason::Unknown);
        let status = HsmCoreStatus::from_dhcsr(0x0203_0003);
        assert!(status.debug_enabled && status.halted && status.reset && !status.locked_up);
    }
}
