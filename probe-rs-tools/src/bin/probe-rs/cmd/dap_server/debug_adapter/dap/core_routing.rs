//! One DAP session for several cores.
//!
//! With more than one entry in `coreConfigs`, each core is a DAP thread (id = core index) and
//! every request goes to one core: the core of its `threadId`, of the frame id or variables
//! reference it names (noted when a response handed them out), or of the `@core` suffix of a
//! memory reference or data id. Requests that name no core go to the focus core, the core of
//! the last `stopped` event. A few requests go to every core (see [`FAN_OUT`]); their responses
//! are collected and merged into one.
//!
//! With one core nothing changes: every request goes to it, responses and events are sent as
//! they are.

use std::{
    collections::{HashMap, HashSet},
    sync::{Arc, Mutex, MutexGuard},
};

use serde_json::Value;

use super::dap_types::{Request, Response};
use crate::cmd::dap_server::{
    debug_adapter::protocol::{BoxedAdapter, ProtocolAdapter},
    server::configuration::ConsoleLog,
};

/// Separates the core from an address in memory references and data ids: `0x7000a004@1`.
pub(crate) const CORE_SEPARATOR: char = '@';

/// The requests that go to every core.
pub(crate) const FAN_OUT: [&str; 5] = [
    "setBreakpoints",
    "setInstructionBreakpoints",
    "setDataBreakpoints",
    "configurationDone",
    "disconnect",
];

/// The requests about the session, not one core: a core that cannot be reached does not refuse
/// them.
pub(crate) const SESSION_REQUESTS: [&str; 7] = [
    "threads",
    "disconnect",
    "terminate",
    "loadedSources",
    "source",
    "modules",
    "completions",
];

/// The routing state, shared between the [`RoutingAdapter`] (which sees every response and
/// event) and the request loop (which picks the core of each request).
#[derive(Debug, Default)]
pub(crate) struct CoreRouting {
    /// The configured cores, in the order of `coreConfigs`.
    cores: Vec<usize>,
    /// The core of the request being handled.
    current: Option<usize>,
    /// The core that handed out each frame id and variables reference.
    references: HashMap<i64, usize>,
    /// The core of the last `stopped` event.
    focus: Option<usize>,
    /// The cores that reported a halt and have not run since.
    halted: HashSet<usize>,
    /// The last `stopped` event of each core that stopped on its own (not by the halt group),
    /// while the core stays halted.
    own_stops: HashMap<usize, serde_json::Map<String, Value>>,
    /// While a request goes to several cores: its command and the responses collected.
    captured: Option<(String, Vec<Response>)>,
    /// The cores that cannot be reached now, and why (the HSM of an AURIX while its debug
    /// window is closed). Requests that go to every core leave them out.
    unavailable: HashMap<usize, String>,
}

/// `text` without its `@core` suffix, and the core.
pub(crate) fn split_core(text: &str) -> (&str, Option<usize>) {
    match text.rsplit_once(CORE_SEPARATOR) {
        Some((address, core)) => match core.trim().parse() {
            Ok(core) => (address, Some(core)),
            Err(_) => (text, None),
        },
        None => (text, None),
    }
}

/// `text` with the `@core` suffix of `core`.
pub(crate) fn with_core(text: &str, core: usize) -> String {
    format!("{}{CORE_SEPARATOR}{core}", split_core(text).0)
}

impl CoreRouting {
    pub(crate) fn set_cores(&mut self, cores: Vec<usize>) {
        self.cores = cores;
        self.references.clear();
        self.focus = None;
        self.halted.clear();
        self.own_stops.clear();
        self.unavailable.clear();
    }

    /// Notes that `core` cannot be reached now (`Some(why)`), or can again (`None`). Returns
    /// whether that changed.
    pub(crate) fn set_unavailable(&mut self, core: usize, why: Option<String>) -> bool {
        match why {
            Some(why) => self.unavailable.insert(core, why).is_none(),
            None => self.unavailable.remove(&core).is_some(),
        }
    }

    /// Why `core` cannot be reached now, if it cannot.
    pub(crate) fn unavailable(&self, core: usize) -> Option<&str> {
        self.unavailable.get(&core).map(String::as_str)
    }

    /// Notes that `core` runs.
    fn running(&mut self, core: usize) {
        self.halted.remove(&core);
        self.own_stops.remove(&core);
    }

    /// Whether every configured core reported a halt and has not run since (a core that cannot
    /// be reached is left out).
    fn all_halted(&self) -> bool {
        self.cores
            .iter()
            .filter(|core| !self.unavailable.contains_key(core))
            .all(|core| self.halted.contains(core))
    }

    pub(crate) fn cores(&self) -> &[usize] {
        &self.cores
    }

    /// Whether requests are routed: more than one core is configured.
    pub(crate) fn is_multi_core(&self) -> bool {
        self.cores.len() > 1
    }

    /// The core for a request that names none: the core of the last `stopped` event, else the
    /// first configured core.
    pub(crate) fn focus_core(&self) -> Option<usize> {
        self.focus.or_else(|| self.cores.first().copied())
    }

    pub(crate) fn set_current(&mut self, core: Option<usize>) {
        self.current = core;
    }

    fn configured(&self, core: usize) -> Option<usize> {
        self.cores.contains(&core).then_some(core)
    }

    /// The core a request is for, and the request with the `@core` suffixes taken off.
    pub(crate) fn route(&self, request: &Request) -> (Option<usize>, Request) {
        let mut request = request.clone();
        let Some(Value::Object(arguments)) = request.arguments.as_mut() else {
            return (self.focus_core(), request);
        };
        let number = |key: &str| arguments.get(key).and_then(Value::as_i64);

        let mut core = None;
        if let Some(thread) = number("threadId") {
            core = usize::try_from(thread)
                .ok()
                .and_then(|c| self.configured(c));
        }
        if core.is_none()
            && let Some(frame) = number("frameId")
        {
            core = self.references.get(&frame).copied();
        }
        if core.is_none()
            && let Some(reference) = number("variablesReference")
            && reference != 0
        {
            core = self.references.get(&reference).copied();
        }
        if core.is_none()
            && let Some(explicit) = number("core")
        {
            core = usize::try_from(explicit)
                .ok()
                .and_then(|c| self.configured(c));
        }
        for key in ["memoryReference", "dataId"] {
            if let Some(Value::String(text)) = arguments.get_mut(key) {
                let (address, suffix) = split_core(text);
                if let Some(suffix) = suffix {
                    let address = address.to_string();
                    *text = address;
                    core = core.or_else(|| self.configured(suffix));
                }
            }
        }
        (core.or_else(|| self.focus_core()), request)
    }

    /// Starts collecting the responses of `command` instead of sending them.
    pub(crate) fn start_capture(&mut self, command: &str) {
        self.captured = Some((command.to_string(), Vec::new()));
    }

    /// Stops collecting, and returns the responses collected.
    pub(crate) fn take_captured(&mut self) -> Vec<Response> {
        self.captured.take().map(|(_, r)| r).unwrap_or_default()
    }

    /// Notes the references a response hands out and adds the core to its memory references
    /// and data ids.
    fn observe_response(&mut self, response: &mut Response) {
        if !self.is_multi_core() {
            return;
        }
        let Some(core) = self.current else {
            return;
        };
        // A core that continues or steps runs until its next `stopped` event.
        if response.success
            && matches!(
                response.command.as_str(),
                "continue" | "next" | "stepIn" | "stepOut" | "stepBack" | "reverseContinue"
            )
        {
            self.running(core);
        }
        let Some(body) = response.body.as_mut() else {
            return;
        };
        let mut note = |value: Option<&Value>| {
            if let Some(reference) = value.and_then(Value::as_i64)
                && reference != 0
            {
                self.references.insert(reference, core);
            }
        };
        let tag = |object: &mut serde_json::Map<String, Value>, key: &str| {
            if let Some(Value::String(text)) = object.get_mut(key) {
                *text = with_core(text, core);
            }
        };
        match response.command.as_str() {
            "stackTrace" => {
                for frame in array(body, "stackFrames") {
                    note(frame.get("id"));
                    // A `disassemble` or instruction breakpoint at the frame's instruction
                    // pointer goes to the core of the frame.
                    if let Value::Object(object) = frame {
                        tag(object, "instructionPointerReference");
                    }
                }
            }
            "scopes" => {
                for scope in array(body, "scopes") {
                    note(scope.get("variablesReference"));
                }
            }
            "variables" => {
                for variable in array(body, "variables") {
                    note(variable.get("variablesReference"));
                    if let Value::Object(object) = variable {
                        tag(object, "memoryReference");
                    }
                }
            }
            "evaluate" | "setVariable" => {
                note(body.get("variablesReference"));
                if let Value::Object(object) = body {
                    tag(object, "memoryReference");
                }
            }
            "dataBreakpointInfo" => {
                if let Value::Object(object) = body {
                    tag(object, "dataId");
                }
            }
            // A continue answers for its own thread: the other cores report that they run
            // (or stay halted at their breakpoints) with their own events.
            "continue" => {
                if let Value::Object(object) = body {
                    object.insert("allThreadsContinued".to_string(), Value::Bool(false));
                }
            }
            _ => {}
        }
    }

    /// Notes the core of a `stopped` event. With several cores, each core reports its own halt,
    /// and an event never claims that all threads continued.
    ///
    /// A client may keep only the state of the last `stopped` event (Zed does), so when every
    /// core is halted the last event must say `allThreadsStopped`. VS Code, given that, clears
    /// every call stack and reads back only the one of the event's thread. So the claim goes on
    /// the stop of a core that stopped on its own: on that event when it is the last halt, else
    /// (the halt group stopped the last core) on a copy of the focus core's stop, returned here
    /// to be sent after the event.
    fn observe_event(&mut self, event_type: &str, body: &mut Option<Value>) -> Option<Value> {
        if !self.is_multi_core() {
            return None;
        }
        let Some(Value::Object(body)) = body.as_mut() else {
            return None;
        };
        let thread = body
            .get("threadId")
            .and_then(Value::as_i64)
            .and_then(|thread| usize::try_from(thread).ok());
        match event_type {
            "stopped" => {
                // A core that the halt group stopped (`external`) did not stop on its own: the
                // client keeps its focus on the core that did (VS Code focuses the thread of each
                // stop otherwise, and a step would go to the last core the group stopped).
                let by_group = body.get("reason").and_then(Value::as_str) == Some("external");
                if by_group {
                    body.insert("preserveFocusHint".to_string(), Value::Bool(true));
                } else if thread.is_some() {
                    self.focus = thread.or(self.focus);
                }
                body.insert("allThreadsStopped".to_string(), Value::Bool(false));
                let thread = thread?;
                self.halted.insert(thread);
                if !by_group {
                    self.own_stops.insert(thread, body.clone());
                }
                if !self.all_halted() {
                    return None;
                }
                if !by_group {
                    body.insert("allThreadsStopped".to_string(), Value::Bool(true));
                    return None;
                }
                let mut again = self.own_stops.get(&self.focus?)?.clone();
                again.insert("allThreadsStopped".to_string(), Value::Bool(true));
                Some(Value::Object(again))
            }
            "continued" => {
                if let Some(thread) = thread {
                    self.running(thread);
                }
                body.insert("allThreadsContinued".to_string(), Value::Bool(false));
                None
            }
            _ => None,
        }
    }
}

/// The elements of the array `key` of `body`, if any.
fn array<'a>(body: &'a mut Value, key: &str) -> impl Iterator<Item = &'a mut Value> {
    body.get_mut(key)
        .and_then(Value::as_array_mut)
        .into_iter()
        .flatten()
}

/// One response for a request that went to several cores, from the response of each core.
///
/// Breakpoint responses are merged per breakpoint: the first core that verified it gives the
/// entry, and its message names the cores that took it. Other responses succeed when every core
/// succeeded; else the first failure is the response.
pub(crate) fn merge_responses(
    command: &str,
    responses: Vec<(usize, Response)>,
) -> Option<Response> {
    let (_, first) = responses.first()?.clone();
    if let Some((_, failure)) = responses.iter().find(|(_, r)| !r.success)
        && (!command.starts_with("set") || responses.iter().all(|(_, r)| !r.success))
    {
        return Some(failure.clone());
    }
    if command != "setBreakpoints" {
        return Some(first);
    }

    let succeeded = responses
        .iter()
        .filter(|(_, r)| r.success)
        .collect::<Vec<_>>();
    let per_core = succeeded
        .iter()
        .map(|(core, r)| (*core, breakpoints_of(r)))
        .collect::<Vec<_>>();
    let count = per_core.iter().map(|(_, b)| b.len()).max().unwrap_or(0);
    let merged = (0..count)
        .map(|i| {
            merge_breakpoint(
                per_core
                    .iter()
                    .filter_map(|(core, b)| Some((*core, b.get(i)?.clone())))
                    .collect(),
            )
        })
        .collect::<Vec<_>>();

    let mut response = succeeded.first().map(|(_, r)| r.clone()).unwrap_or(first);
    response.body = Some(serde_json::json!({ "breakpoints": merged }));
    Some(response)
}

/// The `breakpoints` of a breakpoint response.
fn breakpoints_of(response: &Response) -> Vec<Value> {
    response
        .body
        .as_ref()
        .and_then(|b| b.get("breakpoints"))
        .and_then(Value::as_array)
        .cloned()
        .unwrap_or_default()
}

/// One breakpoint from the answers of the cores that were asked to set it: the first core that
/// verified it gives the entry, and its message names the cores that took it.
fn merge_breakpoint(per_core: Vec<(usize, Value)>) -> Value {
    let verified_on = per_core
        .iter()
        .filter(|(_, bp)| bp.get("verified").and_then(Value::as_bool).unwrap_or(false))
        .map(|(core, _)| *core)
        .collect::<Vec<_>>();
    let mut breakpoint = per_core
        .iter()
        .find(|(core, _)| verified_on.first() == Some(core))
        .or_else(|| per_core.first())
        .map(|(_, bp)| bp.clone())
        .unwrap_or(Value::Null);
    if !verified_on.is_empty()
        && let Value::Object(object) = &mut breakpoint
    {
        let cores = verified_on
            .iter()
            .map(usize::to_string)
            .collect::<Vec<_>>()
            .join(", ");
        let message = object
            .get("message")
            .and_then(Value::as_str)
            .map(|m| format!("{m} (cores {cores})"))
            .unwrap_or_else(|| format!("Set on cores {cores}"));
        object.insert("message".to_string(), Value::String(message));
    }
    breakpoint
}

/// The `setInstructionBreakpoints` request of one core: the breakpoints whose instruction
/// reference names the core, or no core (those go to every core), with the suffix taken off,
/// and the position of each in the request.
pub(crate) fn instruction_breakpoints_of_core(
    request: &Request,
    core: usize,
) -> (Request, Vec<usize>) {
    let mut request = request.clone();
    let mut slots = Vec::new();
    if let Some(Value::Object(arguments)) = request.arguments.as_mut()
        && let Some(Value::Array(breakpoints)) = arguments.get_mut("breakpoints")
    {
        let mut kept = Vec::new();
        for (slot, mut breakpoint) in std::mem::take(breakpoints).into_iter().enumerate() {
            let mut of_core = true;
            if let Some(Value::String(reference)) = breakpoint.get_mut("instructionReference") {
                let (address, suffix) = split_core(reference);
                if let Some(suffix) = suffix {
                    of_core = suffix == core;
                    *reference = address.to_string();
                }
            }
            if of_core {
                slots.push(slot);
                kept.push(breakpoint);
            }
        }
        *breakpoints = kept;
    }
    (request, slots)
}

/// One `setInstructionBreakpoints` response from the responses of the cores: each breakpoint
/// back at its position in the request, merged from the cores that were asked to set it.
pub(crate) fn merge_instruction_breakpoints(
    request: &Request,
    responses: Vec<(usize, Response)>,
    slots: &HashMap<usize, Vec<usize>>,
) -> Option<Response> {
    let count = request
        .arguments
        .as_ref()
        .and_then(|a| a.get("breakpoints"))
        .and_then(Value::as_array)
        .map_or(0, Vec::len);
    if responses.iter().all(|(_, r)| !r.success) {
        return responses.into_iter().next().map(|(_, r)| r);
    }
    let mut per_slot = vec![Vec::new(); count];
    let mut base = None;
    for (core, response) in responses.into_iter().filter(|(_, r)| r.success) {
        for (breakpoint, &slot) in breakpoints_of(&response)
            .into_iter()
            .zip(slots.get(&core).into_iter().flatten())
        {
            if let Some(entries) = per_slot.get_mut(slot) {
                entries.push((core, breakpoint));
            }
        }
        base.get_or_insert(response);
    }
    let merged = per_slot
        .into_iter()
        .map(|entries| {
            if entries.is_empty() {
                serde_json::json!({
                    "verified": false,
                    "message": "The instruction reference names no core of this session that can be reached",
                })
            } else {
                merge_breakpoint(entries)
            }
        })
        .collect::<Vec<_>>();
    let mut response = base?;
    response.body = Some(serde_json::json!({ "breakpoints": merged }));
    Some(response)
}

/// The `setDataBreakpoints` request of one core: the data breakpoints whose data id names the
/// core (or no core, for the focus core) with the suffix taken off, and the position of each
/// in the request.
pub(crate) fn data_breakpoints_of_core(
    request: &Request,
    core: usize,
    focus: Option<usize>,
) -> (Request, Vec<usize>) {
    let mut request = request.clone();
    let mut slots = Vec::new();
    if let Some(Value::Object(arguments)) = request.arguments.as_mut()
        && let Some(Value::Array(breakpoints)) = arguments.get_mut("breakpoints")
    {
        let mut kept = Vec::new();
        for (slot, mut breakpoint) in std::mem::take(breakpoints).into_iter().enumerate() {
            let mut of_core = focus == Some(core);
            if let Some(Value::String(data_id)) = breakpoint.get_mut("dataId") {
                let (address, suffix) = split_core(data_id);
                if let Some(suffix) = suffix {
                    of_core = suffix == core;
                    *data_id = address.to_string();
                }
            }
            if of_core {
                slots.push(slot);
                kept.push(breakpoint);
            }
        }
        *breakpoints = kept;
    }
    (request, slots)
}

/// One `setDataBreakpoints` response from the responses of the cores: each breakpoint back at
/// its position in the request.
pub(crate) fn merge_data_breakpoints(
    request: &Request,
    responses: Vec<(usize, Response)>,
    slots: &HashMap<usize, Vec<usize>>,
) -> Option<Response> {
    let count = request
        .arguments
        .as_ref()
        .and_then(|a| a.get("breakpoints"))
        .and_then(Value::as_array)
        .map_or(0, Vec::len);
    if responses.iter().all(|(_, r)| !r.success) {
        return responses.into_iter().next().map(|(_, r)| r);
    }
    let mut merged = vec![serde_json::json!({ "verified": false }); count];
    let mut base = None;
    for (core, response) in responses.into_iter().filter(|(_, r)| r.success) {
        let breakpoints = response
            .body
            .as_ref()
            .and_then(|b| b.get("breakpoints"))
            .and_then(Value::as_array)
            .cloned()
            .unwrap_or_default();
        for (breakpoint, &slot) in breakpoints
            .into_iter()
            .zip(slots.get(&core).into_iter().flatten())
        {
            if let Some(entry) = merged.get_mut(slot) {
                *entry = breakpoint;
            }
        }
        base.get_or_insert(response);
    }
    let mut response = base?;
    response.body = Some(serde_json::json!({ "breakpoints": merged }));
    Some(response)
}

/// The protocol adapter of a session: passes everything to the transport, and lets
/// [`CoreRouting`] see the responses and events on the way.
pub(crate) struct RoutingAdapter {
    pub(crate) inner: BoxedAdapter,
    pub(crate) routing: Arc<Mutex<CoreRouting>>,
}

impl RoutingAdapter {
    fn routing(&self) -> MutexGuard<'_, CoreRouting> {
        self.routing
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
    }
}

impl ProtocolAdapter for RoutingAdapter {
    fn listen_for_request(&mut self) -> anyhow::Result<Option<Request>> {
        self.inner.listen_for_request()
    }

    fn input_ready(&mut self) -> bool {
        self.inner.input_ready()
    }

    fn dyn_send_event(
        &mut self,
        event_type: &str,
        mut event_body: Option<Value>,
    ) -> anyhow::Result<()> {
        let again = self.routing().observe_event(event_type, &mut event_body);
        self.inner.dyn_send_event(event_type, event_body)?;
        if let Some(again) = again {
            self.inner.dyn_send_event(event_type, Some(again))?;
        }
        Ok(())
    }

    fn send_raw_response(&mut self, mut response: Response) -> anyhow::Result<()> {
        {
            let mut routing = self.routing();
            if let Some((_, captured)) = routing.captured.as_mut() {
                captured.push(response);
                return Ok(());
            }
            routing.observe_response(&mut response);
        }
        self.inner.send_raw_response(response)
    }

    fn remove_pending_request(&mut self, request_seq: i64) -> Option<String> {
        // Each core answers a request that went to all of them: the request stays pending
        // until the merged response is sent.
        if let Some((command, _)) = self.routing().captured.as_ref() {
            return Some(command.clone());
        }
        self.inner.remove_pending_request(request_seq)
    }

    fn has_pending_request(&self, request_seq: i64) -> bool {
        self.inner.has_pending_request(request_seq)
    }

    fn set_console_log_level(&mut self, log_level: ConsoleLog) {
        self.inner.set_console_log_level(log_level)
    }

    fn console_log_level(&self) -> ConsoleLog {
        self.inner.console_log_level()
    }

    fn get_next_seq(&mut self) -> i64 {
        self.inner.get_next_seq()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn request(command: &str, arguments: Value) -> Request {
        Request {
            arguments: Some(arguments),
            command: command.to_string(),
            seq: 1,
            type_: "request".to_string(),
        }
    }

    fn response(command: &str, success: bool, body: Value) -> Response {
        Response {
            body: Some(body),
            command: command.to_string(),
            message: None,
            request_seq: 1,
            seq: 2,
            success,
            type_: "response".to_string(),
        }
    }

    fn routing(cores: &[usize]) -> CoreRouting {
        let mut routing = CoreRouting::default();
        routing.set_cores(cores.to_vec());
        routing
    }

    #[test]
    fn core_suffixes_are_split_and_added() {
        assert_eq!(split_core("0x7000a004@1"), ("0x7000a004", Some(1)));
        assert_eq!(split_core("0x7000a004/4@2"), ("0x7000a004/4", Some(2)));
        assert_eq!(split_core("0x7000a004"), ("0x7000a004", None));
        assert_eq!(with_core("0x7000a004@1", 2), "0x7000a004@2");
    }

    #[test]
    fn an_unavailable_core_is_left_out_of_all_halted() {
        let mut routing = routing(&[0, 1, 2]);
        routing.halted.extend([0, 1]);
        assert!(!routing.all_halted(), "core 2 has not halted");
        assert!(routing.set_unavailable(2, Some("Core 2 is not enabled.".into())));
        assert!(
            !routing.set_unavailable(2, Some("again".into())),
            "no change"
        );
        assert_eq!(routing.unavailable(2), Some("again"), "the latest reason");
        assert!(routing.all_halted(), "the unavailable core is left out");
        assert!(routing.set_unavailable(2, None));
        assert!(!routing.set_unavailable(2, None));
        assert_eq!(routing.unavailable(2), None);
        assert!(!routing.all_halted());
    }

    #[test]
    fn a_thread_id_names_its_core() {
        let routing = routing(&[0, 1, 2]);
        let (core, _) = routing.route(&request("next", json!({ "threadId": 2 })));
        assert_eq!(core, Some(2));
        // An unknown thread goes to the focus core.
        let (core, _) = routing.route(&request("next", json!({ "threadId": 7 })));
        assert_eq!(core, Some(0));
    }

    #[test]
    fn references_go_back_to_the_core_that_handed_them_out() {
        let mut routing = routing(&[0, 1, 2]);
        routing.set_current(Some(1));
        let mut stack = response(
            "stackTrace",
            true,
            json!({ "stackFrames": [{ "id": 11 }, { "id": 12 }] }),
        );
        routing.observe_response(&mut stack);
        let mut scopes = response(
            "scopes",
            true,
            json!({ "scopes": [{ "variablesReference": 21 }] }),
        );
        routing.observe_response(&mut scopes);
        routing.set_current(Some(2));
        let mut variables = response(
            "variables",
            true,
            json!({ "variables": [{ "variablesReference": 31, "memoryReference": "0x7000a004" }] }),
        );
        routing.observe_response(&mut variables);
        assert_eq!(
            variables.body.unwrap()["variables"][0]["memoryReference"],
            "0x7000a004@2"
        );

        assert_eq!(
            routing
                .route(&request("scopes", json!({ "frameId": 12 })))
                .0,
            Some(1)
        );
        assert_eq!(
            routing
                .route(&request("variables", json!({ "variablesReference": 21 })))
                .0,
            Some(1)
        );
        assert_eq!(
            routing
                .route(&request("variables", json!({ "variablesReference": 31 })))
                .0,
            Some(2)
        );
    }

    #[test]
    fn a_memory_reference_names_its_core_and_loses_the_suffix() {
        let routing = routing(&[0, 1, 2]);
        let (core, routed) = routing.route(&request(
            "readMemory",
            json!({ "memoryReference": "0x7000a004@2", "count": 4 }),
        ));
        assert_eq!(core, Some(2));
        assert_eq!(routed.arguments.unwrap()["memoryReference"], "0x7000a004");
    }

    #[test]
    fn the_stop_that_leaves_every_core_halted_says_so() {
        let mut routing = routing(&[0, 1, 2]);
        // The event's `allThreadsStopped`, and the thread and reason of the stop sent after it.
        let stop = |routing: &mut CoreRouting, reason: &str, thread: i64| {
            let mut body = Some(json!({ "reason": reason, "threadId": thread }));
            let again = routing.observe_event("stopped", &mut body).map(|again| {
                assert_eq!(again["allThreadsStopped"], true);
                (again["threadId"].clone(), again["reason"].clone())
            });
            (body.unwrap()["allThreadsStopped"].clone(), again)
        };
        // An all-stop: the core that hit its breakpoint, then the cores the group stopped. The
        // last of those leaves every core halted: the breakpoint's stop is sent again, saying so.
        assert_eq!(stop(&mut routing, "breakpoint", 0), (json!(false), None));
        assert_eq!(stop(&mut routing, "external", 1), (json!(false), None));
        assert_eq!(
            stop(&mut routing, "external", 2),
            (json!(false), Some((json!(0), json!("breakpoint"))))
        );

        // A continue lets its core run: the next halt of another core leaves it out. A core
        // that stops on its own as the last one says so on its own event.
        routing.set_current(Some(0));
        let mut resumed = response("continue", true, json!({}));
        routing.observe_response(&mut resumed);
        assert_eq!(stop(&mut routing, "pause", 1), (json!(false), None));
        assert_eq!(stop(&mut routing, "breakpoint", 0), (json!(true), None));

        // So does a `continued` event, and its body never claims that every thread runs.
        let mut continued = Some(json!({ "threadId": 2, "allThreadsContinued": true }));
        routing.observe_event("continued", &mut continued);
        assert_eq!(continued.unwrap()["allThreadsContinued"], false);
        assert_eq!(stop(&mut routing, "pause", 1), (json!(false), None));
        assert_eq!(stop(&mut routing, "step", 2), (json!(true), None));
    }

    #[test]
    fn requests_without_a_core_go_to_the_core_that_stopped_last() {
        let mut routing = routing(&[0, 1, 2]);
        let mut stopped =
            Some(json!({ "reason": "breakpoint", "threadId": 1, "allThreadsStopped": true }));
        routing.observe_event("stopped", &mut stopped);
        assert_eq!(stopped.unwrap()["allThreadsStopped"], false);
        let (core, _) = routing.route(&request("evaluate", json!({ "expression": "g_result" })));
        assert_eq!(core, Some(1));
    }

    #[test]
    fn a_core_the_group_stopped_keeps_the_focus_on_the_core_that_stopped() {
        let mut routing = routing(&[0, 1, 2]);
        let mut hit = Some(json!({ "reason": "breakpoint", "threadId": 0 }));
        routing.observe_event("stopped", &mut hit);
        let mut external = Some(json!({ "reason": "external", "threadId": 2 }));
        routing.observe_event("stopped", &mut external);
        assert_eq!(external.unwrap()["preserveFocusHint"], true);
        assert!(hit.unwrap().get("preserveFocusHint").is_none());
        let (core, _) = routing.route(&request("evaluate", json!({ "expression": "g_result" })));
        assert_eq!(core, Some(0));
    }

    #[test]
    fn one_core_changes_nothing() {
        let mut routing = routing(&[1]);
        routing.set_current(Some(1));
        let mut variables = response(
            "variables",
            true,
            json!({ "variables": [{ "memoryReference": "0x7000a004" }] }),
        );
        routing.observe_response(&mut variables);
        assert_eq!(
            variables.body.unwrap()["variables"][0]["memoryReference"],
            "0x7000a004"
        );
        let mut stopped = Some(json!({ "threadId": 1, "allThreadsStopped": true }));
        routing.observe_event("stopped", &mut stopped);
        assert_eq!(stopped.unwrap()["allThreadsStopped"], true);
    }

    #[test]
    fn breakpoints_of_several_cores_are_merged_per_breakpoint() {
        let responses = vec![
            (
                0,
                response(
                    "setBreakpoints",
                    true,
                    json!({ "breakpoints": [{ "id": 1, "verified": true }, { "verified": false, "message": "no code" }] }),
                ),
            ),
            (
                1,
                response(
                    "setBreakpoints",
                    true,
                    json!({ "breakpoints": [{ "id": 1, "verified": true }, { "id": 2, "verified": true }] }),
                ),
            ),
        ];
        let merged = merge_responses("setBreakpoints", responses).unwrap();
        let breakpoints = &merged.body.unwrap()["breakpoints"];
        assert_eq!(breakpoints[0]["message"], "Set on cores 0, 1");
        assert_eq!(breakpoints[1]["id"], 2);
        assert_eq!(breakpoints[1]["message"], "Set on cores 1");
    }

    #[test]
    fn data_breakpoints_go_to_the_core_of_their_data_id() {
        let request = request(
            "setDataBreakpoints",
            json!({ "breakpoints": [
                { "dataId": "0x7000a004/4@1", "accessType": "write" },
                { "dataId": "0x7000a000/4" },
                { "dataId": "0x7000a008/4@2" }
            ] }),
        );
        let (core1, slots1) = data_breakpoints_of_core(&request, 1, Some(0));
        assert_eq!(slots1, vec![0]);
        assert_eq!(
            core1.arguments.as_ref().unwrap()["breakpoints"][0]["dataId"],
            "0x7000a004/4"
        );
        let (_, slots0) = data_breakpoints_of_core(&request, 0, Some(0));
        assert_eq!(slots0, vec![1]);
        let (_, slots2) = data_breakpoints_of_core(&request, 2, Some(0));
        assert_eq!(slots2, vec![2]);

        let slots = HashMap::from([(0, slots0), (1, slots1), (2, slots2)]);
        let responses = vec![
            (
                0,
                response(
                    "setDataBreakpoints",
                    true,
                    json!({ "breakpoints": [{ "id": 10, "verified": true }] }),
                ),
            ),
            (
                1,
                response(
                    "setDataBreakpoints",
                    true,
                    json!({ "breakpoints": [{ "id": 11, "verified": true }] }),
                ),
            ),
            (
                2,
                response(
                    "setDataBreakpoints",
                    true,
                    json!({ "breakpoints": [{ "id": 12, "verified": false }] }),
                ),
            ),
        ];
        let merged = merge_data_breakpoints(&request, responses, &slots).unwrap();
        let breakpoints = &merged.body.unwrap()["breakpoints"];
        assert_eq!(breakpoints[0]["id"], 11);
        assert_eq!(breakpoints[1]["id"], 10);
        assert_eq!(breakpoints[2]["id"], 12);
    }

    #[test]
    fn a_frame_names_its_core_in_its_instruction_pointer() {
        let mut routing = routing(&[0, 1, 2]);
        routing.set_current(Some(1));
        let mut stack = response(
            "stackTrace",
            true,
            json!({ "stackFrames": [{ "id": 11, "instructionPointerReference": "0x8003a552" }] }),
        );
        routing.observe_response(&mut stack);
        let reference =
            stack.body.unwrap()["stackFrames"][0]["instructionPointerReference"].clone();
        assert_eq!(reference, "0x8003a552@1");

        // A disassembly there goes to that core, without the suffix.
        let (core, routed) = routing.route(&request(
            "disassemble",
            json!({ "memoryReference": reference, "instructionOffset": -24, "instructionCount": 80 }),
        ));
        assert_eq!(core, Some(1));
        assert_eq!(routed.arguments.unwrap()["memoryReference"], "0x8003a552");
    }

    #[test]
    fn instruction_breakpoints_go_to_the_core_of_their_reference_or_to_every_core() {
        let request = request(
            "setInstructionBreakpoints",
            json!({ "breakpoints": [
                { "instructionReference": "0x8021c028@1", "offset": 8 },
                { "instructionReference": "0x8021c030" },
                { "instructionReference": "0x8021c040@7" }
            ] }),
        );
        let (core1, slots1) = instruction_breakpoints_of_core(&request, 1);
        assert_eq!(slots1, vec![0, 1]);
        let breakpoints = &core1.arguments.as_ref().unwrap()["breakpoints"];
        assert_eq!(breakpoints[0]["instructionReference"], "0x8021c028");
        assert_eq!(breakpoints[0]["offset"], 8);
        let (core0, slots0) = instruction_breakpoints_of_core(&request, 0);
        assert_eq!(slots0, vec![1]);
        assert_eq!(
            core0.arguments.as_ref().unwrap()["breakpoints"][0]["instructionReference"],
            "0x8021c030"
        );

        let slots = HashMap::from([(0, slots0), (1, slots1)]);
        let responses = vec![
            (
                0,
                response(
                    "setInstructionBreakpoints",
                    true,
                    json!({ "breakpoints": [{ "id": 20, "verified": true }] }),
                ),
            ),
            (
                1,
                response(
                    "setInstructionBreakpoints",
                    true,
                    json!({ "breakpoints": [{ "id": 10, "verified": true }, { "id": 21, "verified": true }] }),
                ),
            ),
        ];
        let merged = merge_instruction_breakpoints(&request, responses, &slots).unwrap();
        let breakpoints = &merged.body.unwrap()["breakpoints"];
        assert_eq!(breakpoints[0]["id"], 10);
        assert_eq!(breakpoints[0]["message"], "Set on cores 1");
        assert_eq!(breakpoints[1]["id"], 20);
        assert_eq!(breakpoints[1]["message"], "Set on cores 0, 1");
        assert_eq!(breakpoints[2]["verified"], false);
    }

    #[test]
    fn an_rtt_window_goes_to_the_core_it_names() {
        // A client opens the RTT channel of the core the channel's event named.
        let routing = routing(&[0, 1, 2]);
        let (core, _) = routing.route(&request(
            "rttWindowOpened",
            json!({ "channelNumber": 0, "windowIsOpen": true, "core": 1 }),
        ));
        assert_eq!(core, Some(1));
    }

    #[test]
    fn a_request_to_every_core_fails_when_one_core_fails() {
        let responses = vec![
            (0, response("configurationDone", true, Value::Null)),
            (
                1,
                response("configurationDone", false, json!({ "error": "x" })),
            ),
        ];
        assert!(
            !merge_responses("configurationDone", responses)
                .unwrap()
                .success
        );
    }
}
