use postcard_rpc::header::VarHeader;
use probe_rs_debug::{
    DebugInfo, DebugRegisters, ObjectRef, StackFrameInfo, Variable, VariableCache,
    VariableLocation, VariableName,
};
use probe_rs_rpc::debug_vars::{
    ClearCoreDebugStateRequest, EvaluateRequest, EvaluateResponse, LiveWatchRequest,
    LiveWatchResponse, LoadSvdRequest, LoadSvdResponse, ScopesRequest, ScopesResponse,
    SetVariableRequest, SetVariableResult, VariablesRequest, VariablesResponse,
    WireEvaluateResponse, WireLiveWatchValue, WireScope, WireSetVariableResponse, WireVariable,
};

use crate::rpc::functions::{RpcContext, convert::lift};
use probe_rs_rpc::RpcResult;

/// The memory reference of a variable. The client reads and writes the memory of the variable
/// through this address, so a location that is not an address must not have one.
fn memory_reference(location: &VariableLocation) -> Option<String> {
    location.address().map(|address| format!("{address:#010x}"))
}

/// The DAP protocol uses three related values to determine how to invoke the `Variables` request.
/// This function retrieves that information from the `DebugInfo::VariableCache` and returns it as
/// (`variable_reference`, `named_child_variables_cnt`, `indexed_child_variables_cnt`)
fn variable_reference(parent: &Variable, cache: &VariableCache) -> (ObjectRef, i64, i64) {
    if !parent.is_valid() {
        return (ObjectRef::Invalid, 0, 0);
    }
    let mut named = 0;
    let mut indexed = 0;
    for child in cache.get_children(parent.variable_key()) {
        if child.is_indexed() {
            indexed += 1;
        } else {
            named += 1;
        }
    }
    if named > 0 || indexed > 0 {
        (parent.variable_key(), named, indexed)
    } else if parent.variable_node_type.is_deferred() && parent.to_string(cache) != "()" {
        (parent.variable_key(), 0, 0)
    } else {
        (ObjectRef::Invalid, 0, 0)
    }
}

pub async fn scopes(
    ctx: &mut RpcContext,
    _header: VarHeader,
    request: ScopesRequest,
) -> ScopesResponse {
    let states = ctx.debug_states();
    let mut guard = states.lock().await;
    let state = guard
        .get_mut(&request.sessid)
        .ok_or("No debug state for session")?;
    let core_state = state
        .per_core
        .get_mut(&(request.core as usize))
        .ok_or("No debug state for core")?;
    let frame_ref = ObjectRef::from(request.frame_id as i64);
    let mut scopes: Vec<WireScope> = Vec::new();

    if let Some(static_cache) = &core_state.static_variables {
        scopes.push(WireScope {
            name: "Static".to_string(),
            presentation_hint: Some("statics".to_string()),
            variables_reference: i64::from(static_cache.root_variable().variable_key()),
            expensive: true,
            line: None,
            column: None,
        });
    }

    if let Some(svd_cache) = &core_state.svd_variables {
        scopes.push(WireScope {
            name: "Peripherals".to_string(),
            presentation_hint: None,
            variables_reference: i64::from(svd_cache.root_variable_key()),
            expensive: true,
            line: None,
            column: None,
        });
    }

    if let Some(frame) = core_state.stack_frames.iter().find(|f| f.id == frame_ref) {
        // Registers scope: reuse the frame id as its variables_reference.
        scopes.push(WireScope {
            name: "Registers".to_string(),
            presentation_hint: Some("registers".to_string()),
            variables_reference: i64::from(frame.id),
            expensive: true,
            line: None,
            column: None,
        });

        if let Some(locals) = &frame.local_variables {
            let line = frame
                .source_location
                .as_ref()
                .and_then(|l| l.line.map(|l| l as i64));
            let column = frame.source_location.as_ref().and_then(|l| {
                l.column.map(|c| match c {
                    probe_rs_debug::ColumnType::LeftEdge => 0,
                    probe_rs_debug::ColumnType::Column(c) => c as i64,
                })
            });
            scopes.push(WireScope {
                name: "Variables".to_string(),
                presentation_hint: Some("locals".to_string()),
                variables_reference: i64::from(locals.root_variable().variable_key()),
                expensive: false,
                line,
                column,
            });
        }
    }

    Ok(scopes)
}

pub async fn variables(
    ctx: &mut RpcContext,
    _header: VarHeader,
    request: VariablesRequest,
) -> VariablesResponse {
    let states = ctx.debug_states();
    let mut guard = states.lock().await;
    let debug_info = guard
        .get(&request.sessid)
        .and_then(|s| s.debug_info_for(Some(request.core as usize)));
    let Some(debug_info) = debug_info else {
        Err("No debug state for session")?
    };

    // Take the session/core AFTER locking debug_states: `probe_rs::Core` is
    // `!Send`, so it must not be held across the `debug_states().lock().await`
    // (the spawned server future must remain `Send`).
    let mut session = ctx.session(request.sessid).await;
    let mut core = lift(session.core(request.core as usize))?;

    let Some(state) = guard.get_mut(&request.sessid) else {
        Err("No debug state for session")?
    };
    let Some(core_state) = state.per_core.get_mut(&(request.core as usize)) else {
        Err("No debug state for core")?
    };

    let variable_ref = ObjectRef::from(request.variables_reference as i64);

    if let Some(frame) = core_state
        .stack_frames
        .iter()
        .find(|f| f.id == variable_ref)
    {
        let regs: Vec<WireVariable> = frame
            .registers
            .0
            .iter()
            .map(|register| WireVariable {
                name: register.get_register_name(),
                evaluate_name: Some(register.get_register_name()),
                memory_reference: None,
                indexed_variables: None,
                named_variables: None,
                type_: Some(format!("{}", VariableName::RegistersRoot)),
                value: register.value.unwrap_or_default().to_string(),
                variables_reference: 0,
                byte_size: None,
            })
            .collect();
        return Ok(regs);
    }

    if let Some(svd_cache) = core_state.svd_variables.as_ref()
        && svd_cache.get_variable_by_key(variable_ref).is_some()
    {
        let dap_variables: Vec<WireVariable> = svd_cache
            .get_children(variable_ref)
            .into_iter()
            .map(|variable| {
                let child_count = svd_cache.get_children(variable.variable_key()).len() as i64;
                WireVariable {
                    name: variable.name().to_string(),
                    evaluate_name: None,
                    memory_reference: variable.memory_reference(),
                    indexed_variables: Some(0),
                    named_variables: Some(child_count),
                    type_: variable.type_name(),
                    value: variable.get_value(&mut core),
                    variables_reference: i64::from(variable.variable_key()),
                    // Peripheral registers are 32-bit words.
                    byte_size: variable.memory_reference().map(|_| 4),
                }
            })
            .collect();
        return Ok(dap_variables);
    }

    let mut parent_variable: Option<Variable> = None;
    let mut variable_cache: Option<&mut VariableCache> = None;
    let mut frame_info: Option<StackFrameInfo<'_>> = None;
    let cloned_registers = core_state.stack_frames.first().map(|f| f.registers.clone());
    if let Some(search_cache) = core_state.static_variables.as_mut()
        && let Some(search_variable) = search_cache.get_variable_by_key(variable_ref)
    {
        parent_variable = Some(search_variable);
        variable_cache = Some(search_cache);
        if let Some(regs) = &cloned_registers {
            let top_frame = core_state.stack_frames.first().unwrap();
            frame_info = Some(StackFrameInfo {
                registers: regs,
                frame_base: top_frame.frame_base,
                canonical_frame_address: top_frame.canonical_frame_address,
                caller: None,
            });
        }
    }

    let mut caller_registers: Option<DebugRegisters> = None;
    let mut caller_frame_base = None;
    let mut caller_cfa = None;
    let mut found_frame = None;
    if parent_variable.is_none() {
        for i in 0..core_state.stack_frames.len() {
            let has_variable = core_state.stack_frames[i]
                .local_variables
                .as_ref()
                .is_some_and(|cache| cache.get_variable_by_key(variable_ref).is_some());
            if !has_variable {
                continue;
            }
            if let Some(caller_frame) = core_state.stack_frames.get(i + 1) {
                caller_registers = Some(caller_frame.registers.clone());
                caller_frame_base = caller_frame.frame_base;
                caller_cfa = caller_frame.canonical_frame_address;
            }
            found_frame = Some(i);
            break;
        }
    }

    let caller = caller_registers.as_ref().map(|registers| StackFrameInfo {
        registers,
        frame_base: caller_frame_base,
        canonical_frame_address: caller_cfa,
        caller: None,
    });

    if let Some(i) = found_frame {
        let frame = &mut core_state.stack_frames[i];
        if let Some(search_cache) = frame.local_variables.as_mut()
            && let Some(search_variable) = search_cache.get_variable_by_key(variable_ref)
        {
            parent_variable = Some(search_variable);
            variable_cache = Some(search_cache);
            frame_info = Some(StackFrameInfo {
                registers: &frame.registers,
                frame_base: frame.frame_base,
                canonical_frame_address: frame.canonical_frame_address,
                caller: caller.as_ref(),
            });
        }
    }

    let Some(variable_cache) = variable_cache else {
        Err(format!(
            "No variable information found for {}!",
            request.variables_reference
        ))?
    };

    if let Some(parent) = parent_variable.as_mut()
        && parent.variable_node_type.is_deferred()
        && !variable_cache.has_children(parent)
        && let Some(frame_info) = frame_info
    {
        lift(debug_info.cache_deferred_variables(variable_cache, &mut core, parent, &frame_info))?;
    }

    Ok(variable_cache
        .get_children(variable_ref)
        .filter(|variable| match &request.filter {
            Some(filter) => match filter.as_str() {
                "indexed" => variable.is_indexed(),
                "named" => !variable.is_indexed(),
                _ => false,
            },
            None => true,
        })
        .map(|variable| {
            let (variables_reference, named_cnt, indexed_cnt) =
                variable_reference(variable, variable_cache);
            WireVariable {
                name: variable.name.to_string(),
                evaluate_name: None,
                memory_reference: memory_reference(&variable.memory_location),
                indexed_variables: Some(indexed_cnt),
                named_variables: Some(named_cnt),
                type_: Some(variable.type_name()),
                value: variable.to_string(variable_cache),
                variables_reference: i64::from(variables_reference),
                byte_size: variable.byte_size,
            }
        })
        .collect())
}

pub async fn clear_core_debug_state(
    ctx: &mut RpcContext,
    _header: VarHeader,
    request: ClearCoreDebugStateRequest,
) -> RpcResult<()> {
    let states = ctx.debug_states();
    let mut guard = states.lock().await;
    if let Some(state) = guard.get_mut(&request.sessid) {
        state.clear_core(request.core as usize);
    }
    Ok(())
}

/// Replace the per-core SVD state from `request.path`, or clear it when no
/// path is supplied. A parse failure also clears the previous cache so stale
/// peripheral metadata is never retained after a requested reload.
pub async fn load_svd(
    ctx: &mut RpcContext,
    _header: VarHeader,
    request: LoadSvdRequest,
) -> LoadSvdResponse {
    let parsed = request
        .path
        .as_deref()
        .map(|path| crate::rpc::svd::parse_svd_file(std::path::Path::new(path)))
        .transpose();

    ctx.with_server_debug_state_mut(request.sessid, |state| match parsed {
        Ok(svd_variables) => {
            state.replace_svd(request.core as usize, svd_variables);
            Ok(())
        }
        Err(error) => {
            state.replace_svd(request.core as usize, None);
            Err(crate::rpc::functions::convert::rpc_error_debug(error))
        }
    })
    .await
}

/// Set a local/static variable's value server-side. The client only relays
/// the parent key, name, and new value. Searches per-frame locals then
/// statics by name+parent, runs `Variable::update_value` against the live
/// `Core`, and returns the response fields for the DAP `setVariable` body.
pub async fn set_variable(
    ctx: &mut RpcContext,
    _header: VarHeader,
    request: SetVariableRequest,
) -> SetVariableResult {
    let states = ctx.debug_states();
    let mut guard = states.lock().await;

    let mut session = ctx.session(request.sessid).await;
    let mut core = lift(session.core(request.core as usize))?;

    let Some(state) = guard.get_mut(&request.sessid) else {
        Err("No debug state for session")?
    };
    let Some(core_state) = state.per_core.get_mut(&(request.core as usize)) else {
        Err("No debug state for core")?
    };

    let parent_key = ObjectRef::from(request.parent_key);

    // A peripheral register or field of the SVD file.
    if let Some(svd_cache) = core_state.svd_variables.as_ref()
        && svd_cache.get_variable_by_key(parent_key).is_some()
    {
        let Some(variable) = svd_cache.get_variable_by_name_and_parent(&request.name, parent_key)
        else {
            Err(format!("No peripheral variable {} found", request.name))?
        };
        let value = variable.set_value(&mut core, &request.value)?;
        let children = svd_cache.get_children(variable.variable_key()).len() as i64;
        return Ok(WireSetVariableResponse {
            value,
            type_: variable.type_name(),
            variables_reference: i64::from(variable.variable_key()),
            named_variables: Some(children),
            indexed_variables: Some(0),
            memory_reference: variable.memory_reference(),
        });
    }

    let variable_name = VariableName::Named(request.name.clone());

    let mut cache_variable: Option<Variable> = None;
    let mut variable_cache: Option<&mut VariableCache> = None;
    for frame in core_state.stack_frames.iter_mut() {
        if let Some(search_cache) = frame.local_variables.as_mut()
            && let Some(search_variable) =
                search_cache.get_variable_by_name_and_parent(&variable_name, parent_key)
        {
            cache_variable = Some(search_variable);
            variable_cache = Some(search_cache);
            break;
        }
    }
    if cache_variable.is_none()
        && let Some(search_cache) = core_state.static_variables.as_mut()
        && let Some(search_variable) =
            search_cache.get_variable_by_name_and_parent(&variable_name, parent_key)
    {
        cache_variable = Some(search_variable);
        variable_cache = Some(search_cache);
    }

    let (Some(cache_variable), Some(variable_cache)) = (cache_variable, variable_cache) else {
        Err(format!(
            "No variable information found for {variable_name}!"
        ))?
    };

    cache_variable
        .update_value(&mut core, variable_cache, request.value.clone())
        .map_err(|e| e.to_string())?;
    let (vr, named, indexed) = variable_reference(&cache_variable, variable_cache);
    Ok(WireSetVariableResponse {
        value: request.value,
        type_: Some(format!("{:?}", cache_variable.type_name())),
        variables_reference: i64::from(vr),
        named_variables: Some(named),
        indexed_variables: Some(indexed),
        memory_reference: memory_reference(&cache_variable.memory_location),
    })
}

/// Resolve a DAP evaluate expression against a `VariableCache`, expanding the
/// single-root deferred case first. Returns `None` if the expression names no
/// variable in `cache`.
fn resolve_expression(
    debug_info: &DebugInfo,
    core: &mut probe_rs::Core,
    cache: &mut VariableCache,
    expression: &str,
    frame_info: &StackFrameInfo<'_>,
) -> Option<WireEvaluateResponse> {
    if cache.len() == 1 {
        let mut root = cache.root_variable().clone();
        if root.variable_node_type.is_deferred() && !cache.has_children(&root) {
            debug_info
                .cache_deferred_variables(cache, core, &mut root, frame_info)
                .ok()?;
        }
    }
    let mut variable = if let Ok(key) = expression.parse::<ObjectRef>() {
        cache.get_variable_by_key(key)
    } else {
        cache.get_variable_by_name(&VariableName::Named(expression.to_string()))
    }?;
    let (vr, named, indexed) = variable_reference(&variable, cache);
    variable.extract_value(core, cache);
    cache.update_variable(&variable).ok()?;
    Some(WireEvaluateResponse {
        result: variable.to_string(cache),
        type_: Some(variable.type_name()),
        variables_reference: i64::from(vr),
        named_variables: Some(named),
        indexed_variables: Some(indexed),
        memory_reference: memory_reference(&variable.memory_location),
        byte_size: variable.byte_size,
    })
}

/// One step of a path expression after its first name.
#[derive(Debug, Clone, PartialEq, Eq)]
enum PathStep {
    /// `.member`
    Member(String),
    /// `[index]`
    Index(u64),
    /// `->member`
    Arrow(String),
}

/// A path expression: the `*`s in front, the first name and the steps, as in `OsTcb[1].state`,
/// `p->next->state` or `*p`. `None` when the expression is something else (a cast, arithmetic).
fn parse_path(expression: &str) -> Option<(usize, String, Vec<PathStep>)> {
    fn name(text: &str) -> Option<(&str, &str)> {
        let end = text
            .find(|c: char| !(c.is_ascii_alphanumeric() || c == '_'))
            .unwrap_or(text.len());
        let (name, rest) = text.split_at(end);
        (!name.is_empty() && !name.starts_with(|c: char| c.is_ascii_digit()))
            .then_some((name, rest))
    }
    let mut text = expression.trim();
    let mut derefs = 0;
    while let Some(rest) = text.strip_prefix('*') {
        derefs += 1;
        text = rest.trim_start();
    }
    let (root, mut text) = name(text)?;
    let mut steps = Vec::new();
    loop {
        text = text.trim_start();
        if text.is_empty() {
            break;
        }
        if let Some(rest) = text.strip_prefix("->") {
            let (member, rest) = name(rest.trim_start())?;
            steps.push(PathStep::Arrow(member.to_string()));
            text = rest;
        } else if let Some(rest) = text.strip_prefix('.') {
            let (member, rest) = name(rest.trim_start())?;
            steps.push(PathStep::Member(member.to_string()));
            text = rest;
        } else {
            let (index, rest) = text.strip_prefix('[')?.split_once(']')?;
            let index = index.trim();
            let index = match index
                .strip_prefix("0x")
                .or_else(|| index.strip_prefix("0X"))
            {
                Some(hex) => u64::from_str_radix(hex, 16).ok()?,
                None => index.parse().ok()?,
            };
            steps.push(PathStep::Index(index));
            text = rest;
        }
    }
    Some((derefs, root.to_string(), steps))
}

/// The children of `variable`, read from the target first when they are deferred.
fn children_of(
    debug_info: &DebugInfo,
    core: &mut probe_rs::Core,
    cache: &mut VariableCache,
    variable: &mut probe_rs_debug::Variable,
    frame_info: &StackFrameInfo<'_>,
) -> Vec<probe_rs_debug::Variable> {
    if variable.variable_node_type.is_deferred() && !cache.has_children(variable) {
        // A pointer's value first: its target's members are placed from it (unread, they would
        // be read at the pointer's own address).
        variable.extract_value(core, cache);
        let _ = cache.update_variable(variable);
        let _ = debug_info.cache_deferred_variables(cache, core, variable, frame_info);
    }
    cache
        .get_children(variable.variable_key())
        .cloned()
        .collect()
}

/// [`resolve_expression`] for a path expression ([`parse_path`]) as well: the first name in
/// `cache`, then members, array elements and the targets of pointers. The error says which part
/// could not be found.
fn resolve_path(
    debug_info: &DebugInfo,
    core: &mut probe_rs::Core,
    cache: &mut VariableCache,
    expression: &str,
    frame_info: &StackFrameInfo<'_>,
) -> Result<WireEvaluateResponse, String> {
    let Some((derefs, root, steps)) = parse_path(expression) else {
        return Err(
            "not supported: only names, a.b, a[n], p->b and *p (no casts or arithmetic)".into(),
        );
    };
    if derefs == 0 && steps.is_empty() {
        return resolve_expression(debug_info, core, cache, &root, frame_info)
            .ok_or_else(|| "not a global variable".to_string());
    }
    // The root as `resolve_expression` finds it (expanding a deferred cache root first).
    if resolve_expression(debug_info, core, cache, &root, frame_info).is_none() {
        return Err(format!("`{root}` is not a global variable"));
    }
    let mut current = cache
        .get_variable_by_name(&VariableName::Named(root.clone()))
        .ok_or_else(|| format!("`{root}` is not a global variable"))?;
    let mut path = root;
    let deref = |current: &mut probe_rs_debug::Variable,
                 cache: &mut VariableCache,
                 core: &mut probe_rs::Core,
                 path: &str|
     -> Result<probe_rs_debug::Variable, String> {
        // The target is a child `*name`; a pointer to a struct has the members of its target as
        // its own children instead.
        let children = children_of(debug_info, core, cache, current, frame_info);
        if let Some(target) = children
            .iter()
            .find(|child| matches!(&child.name, VariableName::Named(n) if n.starts_with('*')))
        {
            return Ok(target.clone());
        }
        if children.is_empty() {
            Err(format!("`{path}` is not a pointer with a target"))
        } else {
            Ok(current.clone())
        }
    };
    for step in &steps {
        current = match step {
            PathStep::Member(member) | PathStep::Arrow(member) => {
                let arrow = matches!(step, PathStep::Arrow(_));
                let mut parent = if arrow {
                    deref(&mut current, cache, core, &path)?
                } else {
                    current
                };
                path = format!("{path}{}{member}", if arrow { "->" } else { "." });
                let found = children_of(debug_info, core, cache, &mut parent, frame_info)
                    .into_iter()
                    .find(|child| child.name == VariableName::Named(member.clone()));
                found.ok_or_else(|| format!("`{path}`: no such member"))?
            }
            PathStep::Index(index) => {
                let index = *index;
                let found = children_of(debug_info, core, cache, &mut current, frame_info)
                    .into_iter()
                    .find(|child| child.name == VariableName::Indexed(index));
                path = format!("{path}[{index}]");
                found.ok_or_else(|| format!("`{path}`: no such element"))?
            }
        };
    }
    for _ in 0..derefs {
        let target = deref(&mut current, cache, core, &path)?;
        path = format!("*{path}");
        current = target;
    }
    let (vr, named, indexed) = variable_reference(&current, cache);
    current.extract_value(core, cache);
    cache
        .update_variable(&current)
        .map_err(|error| error.to_string())?;
    Ok(WireEvaluateResponse {
        result: current.to_string(cache),
        type_: Some(current.type_name()),
        variables_reference: i64::from(vr),
        named_variables: Some(named),
        indexed_variables: Some(indexed),
        memory_reference: memory_reference(&current.memory_location),
        byte_size: current.byte_size,
    })
}

/// Reads global variables by name without halting the core (a live watch).
///
/// Every request builds a fresh static scope cache, so each value is read from the target now,
/// whether the core runs or not. Only variables with a static location qualify: there are no
/// stack frames of a running core.
pub async fn live_watch(
    ctx: &mut RpcContext,
    _header: VarHeader,
    request: LiveWatchRequest,
) -> LiveWatchResponse {
    let debug_info = ctx
        .with_server_debug_state(request.sessid, |state| {
            state.debug_info_for(Some(request.core as usize))
        })
        .await;
    let Some(debug_info) = debug_info else {
        Err("No debug information for the session (the core configuration has no program binary)")?
    };

    let mut session = ctx.session(request.sessid).await;
    let mut core = lift(session.core(request.core as usize))?;
    let registers = DebugRegisters::default();
    let frame = StackFrameInfo {
        registers: &registers,
        frame_base: None,
        canonical_frame_address: None,
        caller: None,
    };
    let mut cache = debug_info.create_static_scope_cache();

    Ok(request
        .expressions
        .into_iter()
        .map(|expression| {
            match resolve_path(&debug_info, &mut core, &mut cache, &expression, &frame) {
                Ok(response) => WireLiveWatchValue {
                    expression,
                    value: Some(response.result),
                    type_: response.type_,
                    error: None,
                },
                Err(error) => WireLiveWatchValue {
                    expression,
                    value: None,
                    type_: None,
                    error: Some(error),
                },
            }
        })
        .collect())
}

pub async fn evaluate(
    ctx: &mut RpcContext,
    _header: VarHeader,
    request: EvaluateRequest,
) -> EvaluateResponse {
    let states = ctx.debug_states();
    let mut guard = states.lock().await;
    let debug_info = guard
        .get(&request.sessid)
        .and_then(|s| s.debug_info_for(Some(request.core as usize)));
    let Some(debug_info) = debug_info else {
        Err("No debug state for session")?
    };

    let mut session = ctx.session(request.sessid).await;
    let mut core = lift(session.core(request.core as usize))?;

    let Some(state) = guard.get_mut(&request.sessid) else {
        Err("No debug state for session")?
    };
    let Some(core_state) = state.per_core.get_mut(&(request.core as usize)) else {
        Err("No debug state for core")?
    };

    let invalid = || WireEvaluateResponse {
        result: format!("<invalid expression {:?}>", request.expression),
        type_: None,
        variables_reference: 0,
        named_variables: None,
        indexed_variables: None,
        memory_reference: None,
        byte_size: None,
    };

    let frame_ref = match request.frame_id {
        Some(id) => ObjectRef::from(id as i64),
        None => core_state
            .stack_frames
            .first()
            .map(|f| f.id)
            .unwrap_or(ObjectRef::Invalid),
    };
    if matches!(frame_ref, ObjectRef::Invalid) {
        return Ok(invalid());
    }
    let Some(frame_index) = core_state
        .stack_frames
        .iter()
        .position(|f| f.id == frame_ref)
    else {
        return Ok(invalid());
    };

    if let Some(reg) = core_state.stack_frames[frame_index]
        .registers
        .get_register_by_name(&request.expression)
        .and_then(|r| r.value)
    {
        return Ok(WireEvaluateResponse {
            result: format!("{reg}"),
            type_: Some(format!("{}", VariableName::RegistersRoot)),
            variables_reference: 0,
            named_variables: None,
            indexed_variables: None,
            memory_reference: None,
            byte_size: None,
        });
    }

    let frame_base = core_state.stack_frames[frame_index].frame_base;
    let cfa = core_state.stack_frames[frame_index].canonical_frame_address;
    let frame_regs = core_state.stack_frames[frame_index].registers.clone();
    let caller_registers = core_state
        .stack_frames
        .get(frame_index + 1)
        .map(|frame| frame.registers.clone());
    let caller_frame_base = core_state
        .stack_frames
        .get(frame_index + 1)
        .and_then(|frame| frame.frame_base);
    let caller_cfa = core_state
        .stack_frames
        .get(frame_index + 1)
        .and_then(|frame| frame.canonical_frame_address);
    let caller = caller_registers.as_ref().map(|registers| StackFrameInfo {
        registers,
        frame_base: caller_frame_base,
        canonical_frame_address: caller_cfa,
        caller: None,
    });

    if let Some(cache) = core_state.stack_frames[frame_index]
        .local_variables
        .as_mut()
        && let Some(resp) = resolve_expression(
            &debug_info,
            &mut core,
            cache,
            &request.expression,
            &StackFrameInfo {
                registers: &frame_regs,
                frame_base,
                canonical_frame_address: cfa,
                caller: caller.as_ref(),
            },
        )
    {
        return Ok(resp);
    }

    if let Some(cache) = core_state.static_variables.as_mut()
        && let Some(resp) = {
            let (top_base, top_cfa, top_regs) = core_state
                .stack_frames
                .first()
                .map(|f| (f.frame_base, f.canonical_frame_address, f.registers.clone()))
                .unwrap_or((None, None, DebugRegisters::default()));
            resolve_expression(
                &debug_info,
                &mut core,
                cache,
                &request.expression,
                &StackFrameInfo {
                    registers: &top_regs,
                    frame_base: top_base,
                    canonical_frame_address: top_cfa,
                    caller: None,
                },
            )
        }
    {
        return Ok(resp);
    }

    // A path into the global variables (`OsTcb[1].state`, `p->state`), as `liveWatch` reads it.
    if parse_path(&request.expression)
        .is_some_and(|(derefs, _, steps)| derefs > 0 || !steps.is_empty())
        && let Some(cache) = core_state.static_variables.as_mut()
    {
        let (top_base, top_cfa, top_regs) = core_state
            .stack_frames
            .first()
            .map(|f| (f.frame_base, f.canonical_frame_address, f.registers.clone()))
            .unwrap_or((None, None, DebugRegisters::default()));
        let frame = StackFrameInfo {
            registers: &top_regs,
            frame_base: top_base,
            canonical_frame_address: top_cfa,
            caller: None,
        };
        return Ok(
            match resolve_path(&debug_info, &mut core, cache, &request.expression, &frame) {
                Ok(resp) => resp,
                Err(error) => WireEvaluateResponse {
                    result: format!("<{error}>"),
                    ..invalid()
                },
            },
        );
    }

    Ok(invalid())
}

#[cfg(test)]
mod tests {
    use super::{PathStep, parse_path};

    #[test]
    fn path_expressions() {
        assert_eq!(parse_path("g_ticks"), Some((0, "g_ticks".into(), vec![])));
        assert_eq!(
            parse_path("OsTcb[1].state"),
            Some((
                0,
                "OsTcb".into(),
                vec![PathStep::Index(1), PathStep::Member("state".into())]
            ))
        );
        assert_eq!(
            parse_path(" * p -> next ->state"),
            Some((
                1,
                "p".into(),
                vec![
                    PathStep::Arrow("next".into()),
                    PathStep::Arrow("state".into())
                ]
            ))
        );
        assert_eq!(
            parse_path("a[0x10]"),
            Some((0, "a".into(), vec![PathStep::Index(16)]))
        );
        for unsupported in ["(int)x", "a + 1", "a[i]", "&x", "1x", "a.", "a[1"] {
            assert_eq!(parse_path(unsupported), None, "{unsupported}");
        }
    }
}
