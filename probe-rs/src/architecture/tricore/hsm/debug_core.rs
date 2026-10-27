//! The HSM as a core of a TriCore session: [`TriCoreHsm`] is the [`CoreInterface`] that
//! `Session::core` returns for a core with `!TriCoreHsm` access options.
//!
//! It runs [`HsmCore`] on the TriCore bus of the session ([`TriCoreBusMemory`]); the core is a
//! Cortex-M3 (`type: armv7m`), so the Cortex-M register file, unwinding and gdb description
//! apply. A session makes it again for every access: what it must keep is in
//! [`TriCoreHsmState`], the breakpoints are read back from the FPB.

use std::time::Duration;

use probe_rs_target::{Architecture, CoreType, InstructionSet};

use super::{HsmCore, HsmHaltReason, HsmRegister, HsmStatus};
use crate::{
    CoreInformation, CoreInterface, CoreRegister, CoreStatus, Error, HaltReason, MemoryInterface,
    architecture::{
        arm::core::registers::cortex_m::{CORTEX_M_CORE_REGISTERS, FP, PC, RA, SP},
        tricore::bus::{AddressMap, BusTransfer, TriCoreBusAccess, execute_all},
    },
    core::{
        BreakpointCause,
        registers::{CoreRegisters, RegisterId, RegisterValue},
    },
    memory::CoreMemoryInterface,
};

/// The TriCore bus of a session as a [`MemoryInterface`] of 32-bit device accesses (map 0),
/// the width of the HSM bridge registers and window.
pub struct TriCoreBusMemory<'probe> {
    bus: &'probe mut dyn TriCoreBusAccess,
}

impl<'probe> TriCoreBusMemory<'probe> {
    /// The memory of `bus`.
    pub fn new(bus: &'probe mut dyn TriCoreBusAccess) -> Self {
        Self { bus }
    }

    fn only_words(what: &str) -> Error {
        Error::Other(format!(
            "the HSM bridge is accessed in 32-bit words, not with {what}"
        ))
    }

    fn address(address: u64) -> Result<u32, Error> {
        u32::try_from(address)
            .map_err(|_| Error::Other(format!("address {address:#x} out of range")))
    }
}

impl MemoryInterface for TriCoreBusMemory<'_> {
    fn supports_native_64bit_access(&mut self) -> bool {
        false
    }

    fn read_64(&mut self, _: u64, _: &mut [u64]) -> Result<(), Error> {
        Err(Self::only_words("64-bit reads"))
    }

    fn read_32(&mut self, address: u64, data: &mut [u32]) -> Result<(), Error> {
        let start = Self::address(address)?;
        let mut transfers = (0..data.len() as u32)
            .map(|i| BusTransfer::read(AddressMap::Target, start.wrapping_add(4 * i), 4))
            .collect::<Vec<_>>();
        let limits = self.bus.limits();
        execute_all(&mut *self.bus, &mut transfers, limits)?;
        for (word, transfer) in data.iter_mut().zip(&transfers) {
            *word = transfer.read_u32()?;
        }
        Ok(())
    }

    fn read_16(&mut self, _: u64, _: &mut [u16]) -> Result<(), Error> {
        Err(Self::only_words("16-bit reads"))
    }

    fn read_8(&mut self, _: u64, _: &mut [u8]) -> Result<(), Error> {
        Err(Self::only_words("8-bit reads"))
    }

    fn write_64(&mut self, _: u64, _: &[u64]) -> Result<(), Error> {
        Err(Self::only_words("64-bit writes"))
    }

    fn write_32(&mut self, address: u64, data: &[u32]) -> Result<(), Error> {
        let start = Self::address(address)?;
        let mut transfers = data
            .iter()
            .enumerate()
            .map(|(i, value)| {
                BusTransfer::write_u32(AddressMap::Target, start.wrapping_add(4 * i as u32), *value)
            })
            .collect::<Vec<_>>();
        let limits = self.bus.limits();
        execute_all(&mut *self.bus, &mut transfers, limits)?;
        transfers
            .iter()
            .try_for_each(BusTransfer::check)
            .map_err(Error::from)
    }

    fn write_16(&mut self, _: u64, _: &[u16]) -> Result<(), Error> {
        Err(Self::only_words("16-bit writes"))
    }

    fn write_8(&mut self, _: u64, _: &[u8]) -> Result<(), Error> {
        Err(Self::only_words("8-bit writes"))
    }

    fn supports_8bit_transfers(&self) -> Result<bool, Error> {
        Ok(false)
    }

    fn flush(&mut self) -> Result<(), Error> {
        Ok(())
    }
}

/// What the HSM core of a session keeps between accesses.
#[derive(Debug, Default)]
pub struct TriCoreHsmState {
    /// Whether the debugger enabled the breakpoints (`FP_CTRL.ENABLE`).
    breakpoints_enabled: bool,
}

impl TriCoreHsmState {
    /// The state of an HSM core the session did not touch yet.
    pub fn new() -> Self {
        Self::default()
    }
}

/// The HSM core of a TriCore session.
pub struct TriCoreHsm<'probe, H: MemoryInterface = TriCoreBusMemory<'probe>> {
    hsm: HsmCore<H>,
    state: &'probe mut TriCoreHsmState,
}

impl<'probe> TriCoreHsm<'probe> {
    /// The HSM core on the TriCore bus of the session, or `None` when its debug window is closed
    /// (the HSM firmware did not enable HSM debug, or the HSM does not boot).
    pub(crate) fn attach(
        bus: &'probe mut dyn TriCoreBusAccess,
        state: &'probe mut TriCoreHsmState,
    ) -> Option<Self> {
        Self::attach_on(TriCoreBusMemory::new(bus), state)
    }
}

impl<'probe, H: MemoryInterface> TriCoreHsm<'probe, H> {
    fn attach_on(mut host: H, state: &'probe mut TriCoreHsmState) -> Option<Self> {
        if !HsmStatus::read(&mut host).debug_open() {
            return None;
        }
        Some(Self {
            hsm: HsmCore::new(host),
            state,
        })
    }

    fn register(id: RegisterId) -> Result<HsmRegister, Error> {
        HsmRegister::ALL
            .into_iter()
            .find(|register| register.regsel() == u32::from(id.0))
            .ok_or_else(|| Error::Other(format!("the HSM core has no register {id:?}")))
    }

    fn address(address: u64) -> Result<u32, Error> {
        u32::try_from(address)
            .map_err(|_| Error::Other(format!("HSM breakpoint address {address:#x} out of range")))
    }

    fn pc(&mut self) -> Result<CoreInformation, Error> {
        Ok(CoreInformation {
            pc: u64::from(self.hsm.read_register(HsmRegister::PC)?),
        })
    }
}

impl From<HsmHaltReason> for HaltReason {
    fn from(reason: HsmHaltReason) -> Self {
        match reason {
            HsmHaltReason::Breakpoint => HaltReason::Breakpoint(BreakpointCause::Hardware),
            HsmHaltReason::Watchpoint => HaltReason::Watchpoint,
            HsmHaltReason::VectorCatch => HaltReason::Exception,
            HsmHaltReason::External => HaltReason::External,
            // A halt request and a step are the same bit.
            HsmHaltReason::Halted => HaltReason::Request,
            HsmHaltReason::Unknown => HaltReason::Unknown,
        }
    }
}

impl<H: MemoryInterface> CoreMemoryInterface for TriCoreHsm<'_, H> {
    type ErrorType = Error;

    fn memory(&self) -> &dyn MemoryInterface<Self::ErrorType> {
        self.hsm.memory_ref()
    }

    fn memory_mut(&mut self) -> &mut dyn MemoryInterface<Self::ErrorType> {
        self.hsm.memory()
    }
}

impl<H: MemoryInterface> CoreInterface for TriCoreHsm<'_, H> {
    fn wait_for_core_halted(&mut self, timeout: Duration) -> Result<(), Error> {
        self.hsm.wait_halted(timeout)
    }

    fn core_halted(&mut self) -> Result<bool, Error> {
        Ok(self.hsm.status()?.halted)
    }

    fn status(&mut self) -> Result<CoreStatus, Error> {
        let status = self.hsm.status()?;
        Ok(if status.halted {
            CoreStatus::Halted(self.hsm.halt_reason()?.into())
        } else if status.locked_up {
            CoreStatus::LockedUp
        } else if status.sleeping {
            CoreStatus::Sleeping
        } else {
            CoreStatus::Running
        })
    }

    fn halt(&mut self, timeout: Duration) -> Result<CoreInformation, Error> {
        self.hsm.halt(timeout)?;
        self.pc()
    }

    fn run(&mut self) -> Result<(), Error> {
        self.hsm.run()
    }

    fn reset(&mut self) -> Result<(), Error> {
        // What an HSM reset does to the device is not documented.
        Err(Error::NotImplemented("reset of the HSM core"))
    }

    fn reset_and_halt(&mut self, _timeout: Duration) -> Result<CoreInformation, Error> {
        Err(Error::NotImplemented("reset of the HSM core"))
    }

    fn step(&mut self) -> Result<CoreInformation, Error> {
        let pc = self.hsm.step(Duration::from_millis(100))?;
        Ok(CoreInformation { pc: u64::from(pc) })
    }

    fn read_core_reg(&mut self, address: RegisterId) -> Result<RegisterValue, Error> {
        let register = Self::register(address)?;
        Ok(RegisterValue::from(self.hsm.read_register(register)?))
    }

    fn write_core_reg(&mut self, address: RegisterId, value: RegisterValue) -> Result<(), Error> {
        let register = Self::register(address)?;
        let value: u32 = value.try_into()?;
        self.hsm.write_register(register, value)
    }

    fn available_breakpoint_units(&mut self) -> Result<u32, Error> {
        Ok(self.hsm.breakpoint_units()? as u32)
    }

    fn hw_breakpoints(&mut self) -> Result<Vec<Option<u64>>, Error> {
        Ok(self
            .hsm
            .hw_breakpoints()?
            .into_iter()
            .map(|address| address.map(u64::from))
            .collect())
    }

    fn enable_breakpoints(&mut self, state: bool) -> Result<(), Error> {
        self.hsm.set_breakpoints_enabled(state)?;
        self.state.breakpoints_enabled = state;
        Ok(())
    }

    fn set_hw_breakpoint(&mut self, unit_index: usize, addr: u64) -> Result<(), Error> {
        let address = Self::address(addr)?;
        self.hsm.set_hw_breakpoint_unit(unit_index, address)
    }

    fn clear_hw_breakpoint(&mut self, unit_index: usize) -> Result<(), Error> {
        self.hsm.clear_hw_breakpoint_unit(unit_index)
    }

    fn registers(&self) -> &'static CoreRegisters {
        &CORTEX_M_CORE_REGISTERS
    }

    fn program_counter(&self) -> &'static CoreRegister {
        &PC
    }

    fn frame_pointer(&self) -> &'static CoreRegister {
        &FP
    }

    fn stack_pointer(&self) -> &'static CoreRegister {
        &SP
    }

    fn return_address(&self) -> &'static CoreRegister {
        &RA
    }

    fn hw_breakpoints_enabled(&self) -> bool {
        self.state.breakpoints_enabled
    }

    fn architecture(&self) -> Architecture {
        Architecture::Arm
    }

    fn core_type(&self) -> CoreType {
        CoreType::Armv7m
    }

    fn instruction_set(&mut self) -> Result<InstructionSet, Error> {
        Ok(InstructionSet::Thumb2)
    }

    fn fpu_support(&mut self) -> Result<bool, Error> {
        // A Cortex-M3.
        Ok(false)
    }

    fn floating_point_register_count(&mut self) -> Result<usize, Error> {
        Ok(0)
    }

    fn reset_catch_set(&mut self) -> Result<(), Error> {
        Err(Error::NotImplemented("reset_catch_set"))
    }

    fn reset_catch_clear(&mut self) -> Result<(), Error> {
        Err(Error::NotImplemented("reset_catch_clear"))
    }

    /// Clears the breakpoints and disables halting debug: a halted HSM runs on, as after a
    /// Cortex-M session.
    fn debug_core_stop(&mut self) -> Result<(), Error> {
        for unit in 0..self.hsm.breakpoint_units()? {
            self.hsm.clear_hw_breakpoint_unit(unit)?;
        }
        self.hsm.set_breakpoints_enabled(false)?;
        self.state.breakpoints_enabled = false;
        self.hsm.disable_debug()
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

    fn attach<'a>(
        host: &'a mut Host,
        state: &'a mut TriCoreHsmState,
    ) -> Option<TriCoreHsm<'a, HostRef<'a, Host>>> {
        TriCoreHsm::attach_on(HostRef(host), state)
    }

    #[test]
    fn a_closed_window_is_no_core() {
        let mut host = Host {
            closed: true,
            ..Host::with_cpu(Cpu::default())
        };
        let mut state = TriCoreHsmState::new();
        assert!(attach(&mut host, &mut state).is_none());
        assert!(host.writes.is_empty(), "attaching only reads");
    }

    #[test]
    fn the_core_interface_runs_the_hsm() {
        let mut cpu = Cpu::halted_at(0x8000_4000);
        cpu.regs[13] = 0x2000_1000;
        let mut host = Host::with_cpu(cpu);
        let mut state = TriCoreHsmState::new();
        let mut core = attach(&mut host, &mut state).unwrap();
        assert_eq!(
            core.status().unwrap(),
            CoreStatus::Halted(HaltReason::Unknown)
        );
        assert_eq!(
            core.read_core_reg(SP.id()).unwrap(),
            RegisterValue::U32(0x2000_1000)
        );
        core.write_core_reg(RegisterId(0), RegisterValue::U32(7))
            .unwrap();
        assert!(core.read_core_reg(RegisterId(19)).is_err(), "no REGSEL 19");
        assert_eq!(core.step().unwrap().pc, 0x8000_4002);
        assert_eq!(
            core.status().unwrap(),
            CoreStatus::Halted(HaltReason::Request)
        );

        assert_eq!(core.available_breakpoint_units().unwrap(), 6);
        core.set_hw_breakpoint(1, 0x8000_4100).unwrap();
        core.enable_breakpoints(true).unwrap();
        assert!(core.hw_breakpoints_enabled());
        assert_eq!(core.hw_breakpoints().unwrap()[1], Some(0x8000_4100));
        core.run().unwrap();
        assert_eq!(
            core.status().unwrap(),
            CoreStatus::Halted(HaltReason::Breakpoint(BreakpointCause::Hardware))
        );
        assert_eq!(
            core.halt(Duration::from_millis(20)).unwrap().pc,
            0x8000_4100
        );
        assert!(matches!(core.reset(), Err(Error::NotImplemented(_))));
        assert_eq!(core.core_type(), CoreType::Armv7m);
        assert!(!core.fpu_support().unwrap());

        core.debug_core_stop().unwrap();
        assert!(!core.hw_breakpoints_enabled());
        assert_eq!(core.hw_breakpoints().unwrap(), [None; 6]);
        let cpu = host.cpu.as_ref().unwrap();
        assert_eq!(cpu.regs[0], 7);
        assert!(
            !cpu.debugen && !cpu.halted,
            "the HSM runs on after the session"
        );
    }

    #[test]
    fn the_core_memory_is_the_hsm_address_space() {
        let mut host = Host::with_cpu(Cpu::halted_at(0x8000_4000));
        host.hsm.insert(0x2000_0010, 0xDEAD_BEEF);
        let mut state = TriCoreHsmState::new();
        let mut core = attach(&mut host, &mut state).unwrap();
        assert_eq!(core.read_word_32(0x2000_0010).unwrap(), 0xDEAD_BEEF);
        let mut bytes = [0u8; 2];
        core.read(0x2000_0011, &mut bytes).unwrap();
        assert_eq!(bytes, [0xBE, 0xAD]);
    }
}
