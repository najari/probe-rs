use std::any::Any;

use super::{condition::BreakpointCondition, session_data};
use crate::cmd::dap_server::debug_adapter::dap::repl_commands::ReplCommand;
use crate::rpc::{Key, RttClient};
use probe_rs_rpc::rtt_client::ScanRegion as WireScanRegion;

/// `(channel number, channel name)` pairs returned while attaching to RTT.
pub(crate) type ChannelNames = Vec<(u32, String)>;
use crate::cmd::dap_server::server::debug_rtt;
use crate::util::stream_decoder::DefmtState;
use probe_rs::CoreStatus;

/// [CoreData] is used to cache data needed by the debugger, on a per-core basis.
pub struct CoreData {
    pub core_index: usize,
    /// Track the last status observed through the RPC backend.
    ///
    /// Periodic RPC status queries detect asynchronous transitions such as a
    /// breakpoint halt and notify the DAP client. Requests that resume or step
    /// the target update this field as part of their adapter bookkeeping so
    /// the next query can identify the subsequent transition without emitting
    /// a duplicate event for the request's own state change.
    pub last_known_status: CoreStatus,
    pub target_name: String,
    /// Metadata-only display cache of the server-owned stack state.
    ///
    /// This must be invalidated before any operation that can change the
    /// target's registers or execution state, and replaced only after a
    /// complete server unwind succeeds.
    pub stack_frames: Vec<probe_rs_debug::stack_frame::StackFrame>,
    /// Whether `stack_frames` is the unwind of the current halt (set by
    /// [`Self::replace_stack_frame_cache`], cleared by [`Self::invalidate_stack_frame_cache`]).
    pub stack_frames_valid: bool,
    pub breakpoints: Vec<session_data::ActiveBreakpoint>,
    /// The data breakpoints set with `setDataBreakpoints`.
    pub data_breakpoints: Vec<DataBreakpoint>,
    /// Why the halt that a breakpoint condition let stop is reported as it is (a condition that
    /// could not be checked). The `stopped` event adds it to its description, then clears it.
    pub stop_note: Option<String>,
    /// The addresses of the breakpoints with a condition that were removed since the last poll.
    /// The core may have halted at one before it was removed, and the poll has not checked that
    /// halt yet: the core runs on, as it would have without a breakpoint there.
    pub removed_conditions: Vec<u64>,
    /// A data breakpoint with a condition was removed since the last poll (the same for a
    /// watchpoint halt).
    pub removed_data_condition: bool,
    pub rtt_scan_ranges: WireScanRegion,
    pub rtt_connection: Option<debug_rtt::RttConnection>,
    /// defmt data of the program binary, parsed on the first RTT attach.
    ///
    /// The parse reads the whole ELF file, thus each RTT attach after a reset
    /// reuses this. The outer `Option` is `None` until the first parse, the
    /// inner `Option` is `None` for a program without defmt data. A new
    /// program binary clears the cache in
    /// [`super::session_data::SessionData::load_rtt_location`].
    pub defmt_state: Option<Option<DefmtState>>,
    /// Cache of the server-side RTT client handle between attach attempts,
    /// so we only call `create_rtt` once per core (RPC backend).
    pub rtt_remote_handle: Option<Key<RttClient>>,
    pub repl_commands: Vec<ReplCommand>,
    pub test_data: Box<dyn Any>,
    /// The `entryFunction` of a `launch`, run to after each reset.
    pub entry_breakpoint: Option<EntryBreakpoint>,
}

/// Where the condition of a breakpoint of a core is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum ConditionSlot {
    /// The breakpoint at this index of [`CoreData::breakpoints`].
    Breakpoint(usize),
    /// The data breakpoint at this index of [`CoreData::data_breakpoints`].
    Data(usize),
}

/// A data breakpoint (watchpoint) of a core.
#[derive(Clone, Debug)]
pub struct DataBreakpoint {
    pub address: u64,
    /// The condition and hit count. A watchpoint halt does not say which watchpoint it was, so
    /// each data breakpoint of the core is checked at one.
    pub condition: Option<BreakpointCondition>,
}

/// A temporary breakpoint on the entry function, set after a reset and removed when hit.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EntryBreakpoint {
    pub function: String,
    pub address: u64,
    /// Set while the hardware breakpoint is on the target.
    pub armed: bool,
}

/// The address of the function `name` in the symbol table of `elf`.
///
/// On ARM the Thumb bit of the symbol value is cleared, to give the address of the first
/// instruction.
pub(crate) fn function_address(elf: &[u8], name: &str) -> Result<u64, String> {
    use object::{Object, ObjectSymbol, SymbolKind};

    let file = object::File::parse(elf).map_err(|e| format!("cannot read the ELF file: {e}"))?;
    let symbol = file
        .symbols()
        .find(|s| s.kind() == SymbolKind::Text && s.name() == Ok(name))
        .ok_or_else(|| format!("no function `{name}` in the symbol table"))?;
    let address = symbol.address();
    Ok(match file.architecture() {
        object::Architecture::Arm => address & !1,
        _ => address,
    })
}

impl CoreData {
    pub(crate) fn condition(&self, slot: ConditionSlot) -> Option<&BreakpointCondition> {
        match slot {
            ConditionSlot::Breakpoint(i) => self.breakpoints.get(i)?.condition.as_ref(),
            ConditionSlot::Data(i) => self.data_breakpoints.get(i)?.condition.as_ref(),
        }
    }

    pub(crate) fn condition_mut(
        &mut self,
        slot: ConditionSlot,
    ) -> Option<&mut BreakpointCondition> {
        match slot {
            ConditionSlot::Breakpoint(i) => self.breakpoints.get_mut(i)?.condition.as_mut(),
            ConditionSlot::Data(i) => self.data_breakpoints.get_mut(i)?.condition.as_mut(),
        }
    }

    /// Keeps the breakpoints for which `keep` is true, and the addresses of the removed ones that
    /// had a condition in [`Self::removed_conditions`].
    pub(crate) fn retain_breakpoints(
        &mut self,
        mut keep: impl FnMut(&session_data::ActiveBreakpoint) -> bool,
    ) {
        let mut removed = Vec::new();
        self.breakpoints.retain(|bp| {
            let kept = keep(bp);
            if !kept && bp.condition.is_some() {
                removed.push(bp.address);
            }
            kept
        });
        self.removed_conditions.extend(removed);
    }

    pub(crate) fn invalidate_stack_frame_cache(&mut self) {
        self.stack_frames.clear();
        self.stack_frames_valid = false;
    }

    pub(crate) fn replace_stack_frame_cache(
        &mut self,
        frames: Vec<probe_rs_debug::stack_frame::StackFrame>,
    ) {
        self.stack_frames = frames;
        self.stack_frames_valid = true;
    }
}

#[test]
fn stack_frame_display_cache_is_invalidated_before_replacement() {
    use probe_rs::RegisterValue;
    use probe_rs_debug::{ObjectRef, registers::DebugRegisters, stack_frame::StackFrame};

    fn frame(id: i64) -> StackFrame {
        StackFrame {
            id: ObjectRef::from(id),
            function_name: format!("frame-{id}"),
            source_location: None,
            registers: DebugRegisters::default(),
            pc: RegisterValue::U32(0),
            frame_base: None,
            is_inlined: false,
            local_variables: None,
            canonical_frame_address: None,
        }
    }

    let mut core_data = CoreData {
        core_index: 0,
        last_known_status: CoreStatus::Unknown,
        target_name: String::new(),
        stack_frames: vec![frame(1)],
        stack_frames_valid: true,
        breakpoints: vec![],
        data_breakpoints: vec![],
        stop_note: None,
        removed_conditions: vec![],
        removed_data_condition: false,
        rtt_scan_ranges: WireScanRegion::Ranges(vec![]),
        rtt_connection: None,
        defmt_state: None,
        rtt_remote_handle: None,
        repl_commands: vec![],
        test_data: Box::new(()),
        entry_breakpoint: None,
    };

    core_data.invalidate_stack_frame_cache();
    assert!(core_data.stack_frames.is_empty());

    core_data.replace_stack_frame_cache(vec![frame(2)]);
    assert_eq!(core_data.stack_frames.len(), 1);
    assert_eq!(core_data.stack_frames[0].id, ObjectRef::from(2));
}

#[test]
fn entry_function_is_found_in_the_symbol_table() {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../probe-rs-debug/tests/debug-unwind-tests/RP2040_full_unwind.elf");
    let elf = std::fs::read(path).unwrap();

    // A Thumb function: the symbol value is odd, the address of the instruction is not.
    let main = function_address(&elf, "main").unwrap();
    assert_ne!(main, 0);
    assert_eq!(main & 1, 0);

    let missing = function_address(&elf, "no_such_function").unwrap_err();
    assert_eq!(
        missing,
        "no function `no_such_function` in the symbol table"
    );
    assert!(function_address(b"not an elf file", "main").is_err());
}
