//! TriCore TC3xx: the system timer of a core stops while the core is halted
//! (`suspendPeripherals`).
//!
//! STMx stops in soft suspend (`OCS.SUS` = 2) while CPUx is suspended, its `CPUxSUSOUT` (TC3xx
//! User's Manual, STM OCS). A halt alone does not suspend a core: trigger line 1 of the OCDS
//! trigger switch suspends the cores it targets (`CBS_TL1ST`). So the session routes the halt
//! state of each of its cores to line 1 (`CBS_TRCn.HALT` = 1), makes its cores the only cores
//! the line suspends (the HSM and the cores of no session are taken off, which the reset value
//! has on), and puts their system timers in soft suspend. With several cores in the session, a
//! halt of one suspends the others (they halt). The halt group of `allStop` uses other fields of
//! `CBS_TRCn` (`BRKOUT`, `BRKIN`) and line 7.
//!
//! The registers are in the debug reset domain: an application reset (a launch, a restart)
//! keeps them. The session puts back every word it changed when it ends, and the journal of
//! the cores keeps them meanwhile: a session after a killed one puts them back where they
//! still hold what this one wrote (`CBS_TRCn` and the system timer's `OCS` with core n,
//! `CBS_TL1ST` with the first core).
//!
//! On the TC36x test board halting CPU0 then stops STM0 (`SUSSTA` 1), and STM0 in soft suspend
//! without the routing runs on. The TC27x does not suspend its peripherals from the line (STM0
//! ran on), and the GTM of the TC36x does not either: it takes the OTGS's global suspend, which
//! the TC3xx manual does not describe (probe-rs-infineon's `gtm_suspend` has what was tried).

use super::session_data::SessionData;

/// `CBS_TRC0`, the trigger switch routing of CPU0; `CBS_TRCn` follows every 4 bytes.
const CBS_TRC0: u64 = 0xF000_0420;
/// `CBS_TL1ST`: the suspend targets of trigger line 1.
const CBS_TL1ST: u64 = 0xF000_0494;
/// `HALT` of `CBS_TRCn` (bits 3:0): the line that carries the core's halt state.
const TRC_HALT: u32 = 0xF;
const SUSPEND_LINE: u32 = 1;
/// `CBS_TL1ST.C0`-`C5` and `HSM`.
const TL1ST_CORES: u32 = 0x3F | (1 << 31);
/// The cores `CBS_TL1ST` names: `C0`-`C5`.
const TL1ST_CORE_COUNT: usize = 6;
/// `OCS.SUS` (bits 27:24), its write protection `SUS_P` (bit 28), soft suspend.
const OCS_SUS: u32 = 0xF << 24;
const OCS_SUS_P: u32 = 1 << 28;
const OCS_SOFT_SUSPEND: u32 = 2 << 24;
/// `OCS` of a system timer, from its base.
const STM_OCS: u64 = 0xE8;
/// `OCS.SUSSTA` (bit 29): the timer is suspended now.
const OCS_SUSSTA: u32 = 1 << 29;

/// The system timers of a chip, STMx of CPUx, or why the option is not for the chip.
pub(crate) fn system_timers(chip: Option<&str>) -> Result<Vec<u64>, &'static str> {
    let chip = chip.unwrap_or_default().to_ascii_uppercase();
    if chip.starts_with("TC36") {
        Ok(vec![0xF000_1000, 0xF000_1100])
    } else if chip.starts_with("TC27") {
        Err("the TC27x does not suspend its peripherals from the trigger line")
    } else {
        Err("it is known for the TC36x")
    }
}

/// `CBS_TRCn` with the core's halt state on the suspend line, the other fields as they are.
pub(crate) fn routed(trc: u32) -> u32 {
    (trc & !TRC_HALT) | SUSPEND_LINE
}

/// `CBS_TL1ST` with `cores` as the only cores (and no HSM) that line 1 suspends; the other
/// targets (DMA, HSSL, bus masters) as they are.
pub(crate) fn targets(tl1st: u32, cores: &[usize]) -> u32 {
    cores
        .iter()
        .filter(|&&core| core < TL1ST_CORE_COUNT)
        .fold(tl1st & !TL1ST_CORES, |value, &core| value | (1 << core))
}

/// The `OCS` write that sets soft suspend (the other fields keep their value: their write
/// protection bits are 0).
pub(crate) fn soft_suspend() -> u32 {
    OCS_SUS_P | OCS_SOFT_SUSPEND
}

/// The `OCS` write that puts back the sensitivity of `ocs`.
pub(crate) fn sensitivity_of(ocs: u32) -> u32 {
    OCS_SUS_P | (ocs & OCS_SUS)
}

/// One word the session wrote: where, and the word that puts it back.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Restore {
    pub address: u64,
    pub value: u32,
}

async fn read(session_data: &SessionData, core: usize, address: u64) -> Result<u32, String> {
    session_data
        .backend
        .read_memory_32(core, address, 1)
        .await
        .map_err(|error| format!("{address:#010x}: {error}"))?
        .first()
        .copied()
        .ok_or_else(|| format!("{address:#010x}: nothing read"))
}

async fn write(
    session_data: &SessionData,
    core: usize,
    address: u64,
    value: u32,
) -> Result<(), String> {
    session_data
        .backend
        .write_memory_8(core, address, value.to_le_bytes().to_vec())
        .await
        .map_err(|error| format!("{address:#010x}: {error}"))
}

/// Sets the suspend for the cores `cores` of the session (a core in boot halt is left out: its
/// halt state would hold the line); the words to put back are kept in the session. Returns
/// what was done, for the console.
pub(crate) async fn apply(
    session_data: &mut SessionData,
    chip: Option<&str>,
    cores: &[usize],
) -> Result<String, String> {
    let timers = system_timers(chip)?;
    let Some(&bus) = cores.first() else {
        return Err("the session has no core".into());
    };
    // A core in boot halt is left out: its halt state would hold the line.
    let mut routed_cores = Vec::new();
    for &core in cores {
        let sleeping = session_data
            .backend
            .status(core)
            .await
            .is_ok_and(|status| status == probe_rs::CoreStatus::Sleeping);
        if !sleeping && core < timers.len() {
            routed_cores.push(core);
        }
    }
    if routed_cores.is_empty() {
        return Err("no core of the session can be routed (in boot halt)".into());
    }
    let mut restore = Vec::new();
    // The words for the journal of each core: (address, original, written).
    let mut journal = std::collections::BTreeMap::<usize, Vec<(u32, u32, u32)>>::new();
    let outcome = async {
        for &core in &routed_cores {
            let ocs_address = timers[core] + STM_OCS;
            let ocs = read(session_data, bus, ocs_address).await?;
            write(session_data, bus, ocs_address, soft_suspend()).await?;
            restore.push(Restore {
                address: ocs_address,
                value: sensitivity_of(ocs),
            });
            // As it reads back while the timer runs (the journal compares what it reads).
            let written = read(session_data, bus, ocs_address).await? & !OCS_SUSSTA;
            journal.entry(core).or_default().push((
                ocs_address as u32,
                sensitivity_of(ocs),
                written,
            ));
        }
        let tl1st = read(session_data, bus, CBS_TL1ST).await?;
        let set = targets(tl1st, &routed_cores);
        write(session_data, bus, CBS_TL1ST, set).await?;
        restore.push(Restore {
            address: CBS_TL1ST,
            value: tl1st,
        });
        journal
            .entry(bus)
            .or_default()
            .push((CBS_TL1ST as u32, tl1st, set));
        // The run writes of each core put this value back, not the reset value.
        for &core in cores {
            session_data
                .backend
                .set_tricore_options(core, None, None, None, Some(Some(set)), None)
                .await
                .map_err(|error| format!("core {core}: {error}"))?;
        }
        for &core in &routed_cores {
            let address = CBS_TRC0 + 4 * core as u64;
            let trc = read(session_data, bus, address).await?;
            write(session_data, bus, address, routed(trc)).await?;
            restore.push(Restore {
                address,
                value: trc,
            });
            // First in the core's journal: the halt group's entry for `CBS_TRCn` finds the word
            // as the group wrote it only once this one is back.
            journal
                .entry(core)
                .or_default()
                .insert(0, (address as u32, trc, routed(trc)));
        }
        Ok::<_, String>(())
    }
    .await;
    // What was written, also when a step failed: the journals keep it.
    for (core, words) in journal {
        if let Err(error) = session_data
            .backend
            .set_tricore_options(core, None, None, None, None, Some(words))
            .await
        {
            tracing::warn!("`suspendPeripherals`: no journal for core {core}: {error}");
        }
    }
    session_data.peripheral_suspend.extend(restore);
    outcome?;
    let cores = routed_cores
        .iter()
        .map(|core| core.to_string())
        .collect::<Vec<_>>()
        .join(", ");
    Ok(format!(
        "`suspendPeripherals`: the system timer of core {cores} stops while the core is halted (trigger line 1)."
    ))
}

/// Puts back every word [`apply`] changed, the routing first.
pub(crate) async fn restore(session_data: &mut SessionData) -> Result<(), String> {
    let restore = std::mem::take(&mut session_data.peripheral_suspend);
    let Some(bus) = session_data.core_data.first().map(|c| c.core_index) else {
        return Ok(());
    };
    if restore.is_empty() {
        return Ok(());
    }
    let mut failed = None;
    let cores = session_data
        .core_data
        .iter()
        .map(|c| c.core_index)
        .collect::<Vec<_>>();
    for core in cores {
        if let Err(error) = session_data
            .backend
            .set_tricore_options(core, None, None, None, Some(None), Some(Vec::new()))
            .await
        {
            failed.get_or_insert(format!("core {core}: {error}"));
        }
    }
    for word in restore.iter().rev() {
        if let Err(error) = write(session_data, bus, word.address, word.value).await {
            failed.get_or_insert(error);
        }
    }
    failed.map_or(Ok(()), Err)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_routing_keeps_the_halt_group() {
        // The halt group's BRKOUT and BRKIN on line 7, HALT on line 2 before.
        assert_eq!(routed(0x0070_0072), 0x0070_0071);
        assert_eq!(routed(0x2), 0x1);
    }

    #[test]
    fn the_line_suspends_the_cores_of_the_session() {
        // The board's reset value: CPU0-CPU5 and the HSM.
        assert_eq!(targets(0x8000_003F, &[0]), 0x1);
        assert_eq!(targets(0x8000_003F, &[0, 1]), 0x3);
        // DMA stays a target.
        assert_eq!(targets(0x2000_0002, &[0]), 0x2000_0001);
        assert_eq!(targets(0, &[7]), 0);
    }

    #[test]
    fn the_sensitivity_goes_back() {
        assert_eq!(soft_suspend(), 0x1200_0000);
        assert_eq!(sensitivity_of(0x0000_0000), 0x1000_0000);
        assert_eq!(sensitivity_of(0x2100_0000), 0x1100_0000);
    }

    #[test]
    fn the_chips() {
        assert_eq!(
            system_timers(Some("TC36x")),
            Ok(vec![0xF000_1000, 0xF000_1100])
        );
        assert!(system_timers(Some("TC275")).is_err());
        assert!(system_timers(None).is_err());
    }
}
