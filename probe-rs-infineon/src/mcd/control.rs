//! The core of an MCD probe controlled through the MCD library: execution control, the
//! register map the library describes, and breakpoints as MCD triggers.
//!
//! The TriCore core of probe-rs does none of this through MCD; it drives the OCDS registers
//! over the bus. This is the path for cores whose debug registers probe-rs does not know but
//! an MCD library does (the TC4x PPU and CDSP, the SCR): the library halts, steps and reads them,
//! and the probe only needs the core's name. Disassembly and unwinding stay core specific.

use std::fmt;

use super::{McdError, McdProbe, ffi};

/// The execution state of a core, as the MCD library reports it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum McdExecution {
    /// The library does not know.
    Unknown,
    Running,
    Halted,
    /// Halted in debug mode, the state of a core stopped by a debugger.
    Debug,
    /// A state of the library's own.
    Custom(u32),
}

impl McdExecution {
    fn from_raw(state: u32) -> Self {
        match state {
            ffi::MCD_CORE_STATE_UNKNOWN => Self::Unknown,
            ffi::MCD_CORE_STATE_RUNNING => Self::Running,
            ffi::MCD_CORE_STATE_HALTED => Self::Halted,
            ffi::MCD_CORE_STATE_DEBUG => Self::Debug,
            other => Self::Custom(other),
        }
    }

    /// Whether the core is stopped (halted or in debug mode).
    pub fn is_stopped(self) -> bool {
        matches!(self, Self::Halted | Self::Debug)
    }
}

impl fmt::Display for McdExecution {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Unknown => f.write_str("unknown"),
            Self::Running => f.write_str("running"),
            Self::Halted => f.write_str("halted"),
            Self::Debug => f.write_str("debug"),
            Self::Custom(state) => write!(f, "state {state:#x}"),
        }
    }
}

/// The state of a core: [`McdProbe::core_state`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct McdCoreState {
    pub execution: McdExecution,
    /// The MCD core events since the last query.
    pub events: u32,
    /// The trigger that stopped the core, 0 for none.
    pub trigger: u32,
    /// Why the core stopped, in the library's words.
    pub stop_reason: String,
    /// More about the state, in the library's words.
    pub info: String,
}

impl McdCoreState {
    /// Whether the core stopped at least once since the last query.
    pub fn stopped_since(&self) -> bool {
        self.events & ffi::MCD_CORE_EVENT_STOPPED != 0
    }
}

/// A register group of the core.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct McdRegisterGroup {
    pub id: u32,
    pub name: String,
    pub registers: u32,
}

/// A register of the core, as the library describes it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct McdRegister {
    pub name: String,
    pub group: u32,
    /// The size in bits.
    pub bits: u32,
    /// The address, in the memory space `space`.
    pub address: u64,
    pub space: u32,
    /// Simple, compound or partial ([`ffi::MCD_REG_TYPE_SIMPLE`] …).
    pub kind: u32,
}

impl McdRegister {
    fn from_raw(info: &ffi::mcd_register_info_st) -> Self {
        Self {
            name: ffi::c_string(&info.regname),
            group: info.reg_group_id,
            bits: info.regsize,
            address: info.addr.address,
            space: info.addr.mem_space_id,
            kind: info.reg_type,
        }
    }

    /// The size in bytes.
    pub fn bytes(&self) -> usize {
        self.bits.div_ceil(8) as usize
    }
}

/// What the library's triggers can do.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct McdTriggerInfo {
    /// The trigger types (an MCD bit mask: 1 instruction pointer, 2 read, 4 write, 8 either).
    pub types: u32,
    /// The trigger actions (an MCD bit mask: 1 stop this core).
    pub actions: u32,
    /// How many triggers there are, 0 when the library does not say.
    pub count: u32,
    /// Whether the library also patches code for breakpoints.
    pub software_breakpoints: bool,
}

/// The MCD booleans: `u8` on Linux and macOS, `u32` elsewhere.
fn mcd_bool(value: bool) -> ffi::mcd_bool_t {
    ffi::mcd_bool_t::from(value)
}

impl McdProbe {
    fn open_core(&self) -> Result<*const ffi::mcd_core_st, McdError> {
        if self.core.is_null() {
            Err(McdError::NotConnected)
        } else {
            Ok(self.core)
        }
    }

    fn run_functions(&self) -> Result<ffi::McdRunFunctions, McdError> {
        self.api
            .run_control
            .ok_or(McdError::NotSupported("execution control"))
    }

    fn register_functions(&self) -> Result<ffi::McdRegisterFunctions, McdError> {
        self.api
            .registers
            .ok_or(McdError::NotSupported("register maps"))
    }

    fn trigger_functions(&self) -> Result<ffi::McdTriggerFunctions, McdError> {
        self.api.triggers.ok_or(McdError::NotSupported("triggers"))
    }

    /// Whether the library has the execution control functions.
    pub fn has_run_control(&self) -> bool {
        self.api.run_control.is_some()
    }

    /// The state of the connected core.
    pub fn core_state(&self) -> Result<McdCoreState, McdError> {
        let core = self.open_core()?;
        let run = self.run_functions()?;
        let mut state: ffi::mcd_core_state_st = ffi::zeroed();
        // SAFETY: an open core handle and a valid output structure.
        let ret = unsafe { (run.qry_state)(core, &mut state) };
        self.check("mcd_qry_state_f", ret)?;
        Ok(McdCoreState {
            execution: McdExecution::from_raw(state.state),
            events: state.event,
            trigger: state.trig_id,
            stop_reason: ffi::c_string(&state.stop_str),
            info: ffi::c_string(&state.info_str),
        })
    }

    /// Starts the connected core; `global` starts all cores of the system.
    pub fn run_core(&mut self, global: bool) -> Result<(), McdError> {
        let core = self.open_core()?;
        let run = self.run_functions()?;
        // SAFETY: an open core handle.
        let ret = unsafe { (run.run)(core, mcd_bool(global)) };
        self.check("mcd_run_f", ret)
    }

    /// Stops the connected core; `global` stops all cores of the system.
    pub fn stop_core(&mut self, global: bool) -> Result<(), McdError> {
        let core = self.open_core()?;
        let run = self.run_functions()?;
        // SAFETY: an open core handle.
        let ret = unsafe { (run.stop)(core, mcd_bool(global)) };
        self.check("mcd_stop_f", ret)
    }

    /// Steps the connected core by `count` instructions. The call blocks until they are done.
    pub fn step_instructions(&mut self, count: u32) -> Result<(), McdError> {
        let core = self.open_core()?;
        let run = self.run_functions()?;
        // SAFETY: an open core handle.
        let ret =
            unsafe { (run.step)(core, mcd_bool(false), ffi::MCD_CORE_STEP_TYPE_INSTR, count) };
        self.check("mcd_step_f", ret)
    }

    /// The register groups of the connected core.
    pub fn register_groups(&self) -> Result<Vec<McdRegisterGroup>, McdError> {
        let core = self.open_core()?;
        let registers = self.register_functions()?;
        // SAFETY: an open core handle and room for `count` entries.
        let groups = self.query_list("mcd_qry_reg_groups_f", |start, count, out| unsafe {
            (registers.qry_reg_groups)(core, start, count, out)
        })?;
        Ok(groups
            .iter()
            .map(|group| McdRegisterGroup {
                id: group.reg_group_id,
                name: ffi::c_string(&group.reg_group_name),
                registers: group.n_registers,
            })
            .collect())
    }

    /// The registers of group `group` of the connected core.
    pub fn registers(&self, group: u32) -> Result<Vec<McdRegister>, McdError> {
        let core = self.open_core()?;
        let registers = self.register_functions()?;
        // SAFETY: an open core handle and room for `count` entries.
        let map = self.query_list("mcd_qry_reg_map_f", |start, count, out| unsafe {
            (registers.qry_reg_map)(core, group, start, count, out)
        })?;
        Ok(map.iter().map(McdRegister::from_raw).collect())
    }

    /// The register called `name` (without case) in any group.
    pub fn find_register(&self, name: &str) -> Result<McdRegister, McdError> {
        let groups = self.register_groups()?;
        // A library without groups has the one group 0.
        let ids = if groups.is_empty() {
            vec![0]
        } else {
            groups.iter().map(|g| g.id).collect()
        };
        for id in ids {
            if let Some(register) = self
                .registers(id)?
                .into_iter()
                .find(|r| r.name.eq_ignore_ascii_case(name))
            {
                return Ok(register);
            }
        }
        Err(McdError::NotFound {
            what: "register",
            pick: name.to_string(),
            available: "the registers `mcd_core regs` lists".into(),
        })
    }

    /// Runs one register transaction: reads into `data`, or writes it.
    fn register_transaction(
        &mut self,
        register: &McdRegister,
        data: &mut [u8],
        write: bool,
    ) -> Result<(), McdError> {
        let core = self.open_core()?;
        let width = data.len();
        let mut tx = ffi::mcd_tx_st {
            addr: ffi::mcd_addr_st {
                address: register.address,
                mem_space_id: register.space,
                addr_space_id: 0,
                addr_space_type: ffi::MCD_NOTUSED_ID,
            },
            access_type: if write {
                ffi::MCD_TX_AT_W
            } else {
                ffi::MCD_TX_AT_R
            },
            options: ffi::MCD_TX_OPT_DEFAULT,
            access_width: if matches!(width, 1 | 2 | 4 | 8) {
                width as u8
            } else {
                0
            },
            core_mode: 0,
            data: data.as_mut_ptr(),
            num_bytes: width as u32,
            num_bytes_ok: 0,
        };
        let mut list = ffi::mcd_txlist_st {
            tx: &mut tx,
            num_tx: 1,
            num_tx_ok: 0,
        };
        // SAFETY: the transaction points into `data`, which outlives the call.
        let ret = unsafe { (self.api.execute_txlist)(core, &mut list) };
        self.check("mcd_execute_txlist_f", ret)?;
        if list.num_tx_ok != 1 {
            return Err(McdError::Call {
                function: "mcd_execute_txlist_f",
                code: ffi::MCD_ERR_TXLIST_TX,
                message: format!("register {} not transferred", register.name),
            });
        }
        Ok(())
    }

    /// The value of `register`, little endian as MCD transfers registers.
    pub fn read_register(&mut self, register: &McdRegister) -> Result<Vec<u8>, McdError> {
        let mut data = vec![0; register.bytes()];
        self.register_transaction(register, &mut data, false)?;
        Ok(data)
    }

    /// The value of `register` as a number, for registers of up to 64 bits.
    pub fn read_register_u64(&mut self, register: &McdRegister) -> Result<u64, McdError> {
        let data = self.read_register(register)?;
        Ok(data
            .iter()
            .take(8)
            .rev()
            .fold(0, |n, byte| (n << 8) | u64::from(*byte)))
    }

    /// Writes `value` (its low bytes, as many as the register has) to `register`.
    pub fn write_register_u64(
        &mut self,
        register: &McdRegister,
        value: u64,
    ) -> Result<(), McdError> {
        let mut data = value.to_le_bytes()[..register.bytes().min(8)].to_vec();
        data.resize(register.bytes(), 0);
        self.register_transaction(register, &mut data, true)
    }

    /// What the triggers of the connected core can do.
    pub fn trigger_info(&self) -> Result<McdTriggerInfo, McdError> {
        let core = self.open_core()?;
        let triggers = self.trigger_functions()?;
        let mut info: ffi::mcd_trig_info_st = ffi::zeroed();
        // SAFETY: an open core handle and a valid output structure.
        let ret = unsafe { (triggers.qry_trig_info)(core, &mut info) };
        self.check("mcd_qry_trig_info_f", ret)?;
        Ok(McdTriggerInfo {
            types: info.type_,
            actions: info.action,
            count: info.trig_number,
            software_breakpoints: info.sw_breakpoints != 0,
        })
    }

    /// Sets a hardware breakpoint at `address` (in the device memory space): a trigger on the
    /// instruction pointer that stops the core. Returns the trigger's ID.
    pub fn set_breakpoint(&mut self, address: u64) -> Result<u32, McdError> {
        let core = self.open_core()?;
        let triggers = self.trigger_functions()?;
        let mut trigger = ffi::mcd_trig_simple_core_st {
            struct_size: size_of::<ffi::mcd_trig_simple_core_st>() as u32,
            type_: ffi::MCD_TRIG_TYPE_IP,
            option: ffi::MCD_TRIG_OPT_IMPL_HARDWARE,
            action: ffi::MCD_TRIG_ACTION_DBG_DEBUG,
            action_param: 0,
            modified: mcd_bool(false),
            state_mask: 0,
            addr_start: ffi::mcd_addr_st {
                address,
                mem_space_id: self.mem_space_id,
                addr_space_id: 0,
                addr_space_type: ffi::MCD_NOTUSED_ID,
            },
            addr_range: 0,
        };
        let mut id = 0;
        // SAFETY: an open core handle; the trigger structure and the ID outlive the call.
        let ret = unsafe { (triggers.create_trig)(core, (&raw mut trigger).cast(), &mut id) };
        self.check("mcd_create_trig_f", ret)?;
        if trigger.modified != 0 {
            tracing::warn!(
                "MCD: the library changed the breakpoint at {address:#x}: now at {:#x}, range {}",
                trigger.addr_start.address,
                trigger.addr_range
            );
        }
        self.activate_triggers(triggers)?;
        Ok(id)
    }

    /// Removes the trigger `id`.
    pub fn remove_trigger(&mut self, id: u32) -> Result<(), McdError> {
        let core = self.open_core()?;
        let triggers = self.trigger_functions()?;
        // SAFETY: an open core handle.
        let ret = unsafe { (triggers.remove_trig)(core, id) };
        self.check("mcd_remove_trig_f", ret)?;
        self.activate_triggers(triggers)
    }

    /// Downloads the trigger set, so a change takes effect while the core runs too.
    fn activate_triggers(&self, triggers: ffi::McdTriggerFunctions) -> Result<(), McdError> {
        // SAFETY: an open core handle.
        let ret = unsafe { (triggers.activate_trig_set)(self.core) };
        self.check("mcd_activate_trig_set_f", ret)
    }
}
