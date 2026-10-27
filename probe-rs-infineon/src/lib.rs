//! Infineon AURIX (TriCore) support for probe-rs.
//!
//! - [`tas`]: a debug probe that reaches the device through an Infineon TAS server.
//! - `mcd` (feature `mcd`): a debug probe that loads an MCD API library.
//! - [`hsm`]: the HSM (Cortex-M3) through its debug memory window: memory, run control,
//!   registers and breakpoints.
//! - The AURIX targets (`targets/*.yaml`) and their debug sequences.
//!
//! Call [`register_plugin`] once at startup.

use std::sync::Arc;

use probe_rs::{
    Error,
    architecture::tricore::{ocds, sequences::TriCoreDebugSequence},
    config::{DebugSequence, Registry},
    flashing::DebugFlashSequence,
    plugin,
    probe::{DebugProbe, Probe},
    vendor::Vendor,
};
use probe_rs_target::{Chip, ChipFamily, CoreType};

pub mod device_config;
pub mod flash;
pub mod hsm;
#[cfg(feature = "mcd")]
pub mod mcd;
pub mod mcds;
pub mod svd;
pub mod tas;

/// Registers the AURIX targets, the AURIX vendor and the probe drivers (TAS, and MCD with the
/// `mcd` feature) with probe-rs.
pub fn register_plugin() {
    let targets = targets();
    plugin::register_plugin(plugin::Plugin {
        vendors: &[&InfineonAurix],
        image_formats: &[],
        targets: &targets,
        probe_drivers: &[
            &tas::TasFactory,
            #[cfg(feature = "mcd")]
            &mcd::McdFactory,
        ],
    });
}

fn targets() -> Vec<ChipFamily> {
    const AURIX_TARGETS: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/targets.bincode"));

    bincode::serde::decode_from_slice(AURIX_TARGETS, bincode::config::standard())
        .expect("Failed to deserialize builtin targets. This is a bug")
        .0
}

/// The part number field (bits 27:12) of an AURIX JTAG ID code, and the target it identifies.
const PART_NUMBERS: &[(u32, &str)] = &[
    // TC27x: C-step 0x401DA083, D-step 0x501DA083.
    (0x01DA, "TC275"),
    // TC36x: A-step 0x10209083.
    (0x0209, "TC36x"),
];

/// The target name for a JTAG ID code, if it is a known AURIX device.
pub fn target_by_jtag_id(jtag_id: u32) -> Option<&'static str> {
    // Infineon JEP106 code (bank 1, 0x41) in bits 11:1.
    if (jtag_id >> 1) & 0x7FF != 0x041 {
        return None;
    }
    let part = (jtag_id >> 12) & 0xFFFF;
    PART_NUMBERS
        .iter()
        .find(|(number, _)| *number == part)
        .map(|(_, name)| *name)
}

/// The debug sequence of AURIX chips.
#[derive(Debug)]
pub struct AurixSequence {
    /// The chip name of the target.
    chip: String,
    flash: Option<Arc<dyn DebugFlashSequence>>,
    /// The address of STM0 `TIM0SV`.
    stm0_tim0sv: u32,
}

impl AurixSequence {
    /// Creates the AURIX debug sequence of the chip `name`.
    pub fn create(name: &str) -> Arc<dyn TriCoreDebugSequence> {
        let flash = flash::FlashDevice::for_chip(name)
            .map(|device| Arc::new(flash::AurixFlash::new(device)) as Arc<dyn DebugFlashSequence>);
        let stm0_tim0sv = if is_tc3xx(name) {
            ocds::STM0_TIM0SV_TC3XX
        } else {
            ocds::STM0_TIM0SV
        };
        Arc::new(Self {
            chip: name.to_string(),
            flash,
            stm0_tim0sv,
        })
    }
}

/// Whether `name` is a chip of the AURIX TC3xx family.
fn is_tc3xx(name: &str) -> bool {
    name.to_ascii_uppercase().starts_with("TC3")
}

impl TriCoreDebugSequence for AurixSequence {
    /// Refuses a device of another known chip: the boards of a test setup are swapped on one
    /// TAS server, and a TC275 session on a TC36x (or the other way) would use the wrong cores,
    /// memory map and flash loader.
    fn on_connect(
        &self,
        bus: &mut dyn probe_rs::architecture::tricore::bus::TriCoreBusAccess,
    ) -> Result<(), Error> {
        let jtag_id = bus.jtag_id()?;
        match target_by_jtag_id(jtag_id) {
            Some(found) if !found.eq_ignore_ascii_case(&self.chip) => Err(Error::Other(format!(
                "the target is {}, but the connected device (JTAG ID {jtag_id:#010x}) is a \
                 {found}: use the {found} configuration",
                self.chip
            ))),
            _ => Ok(()),
        }
    }

    fn debug_flash_sequence(&self) -> Option<Arc<dyn DebugFlashSequence>> {
        self.flash.clone()
    }

    fn stm0_tim0sv(&self) -> u32 {
        self.stm0_tim0sv
    }
}

/// The Infineon AURIX vendor.
#[derive(docsplay::Display)]
struct InfineonAurix;

impl Vendor for InfineonAurix {
    fn try_create_debug_sequence(&self, chip: &Chip) -> Option<DebugSequence> {
        if chip
            .cores
            .iter()
            .all(|core| core.core_type == CoreType::TriCore)
        {
            Some(DebugSequence::TriCore(AurixSequence::create(&chip.name)))
        } else {
            None
        }
    }

    fn try_detect_chip_from_probe(
        &self,
        _registry: &Registry,
        probe: &mut Probe,
    ) -> Result<Option<String>, Error> {
        let bus = if probe.try_into::<tas::TasProbe>().is_some() {
            probe
                .try_into::<tas::TasProbe>()
                .and_then(|tas| tas.try_as_tricore_bus())
        } else {
            mcd_bus(probe)
        };
        let Some(bus) = bus else {
            return Ok(None);
        };
        let jtag_id = bus.jtag_id()?;
        let target = target_by_jtag_id(jtag_id);
        tracing::debug!("AURIX JTAG ID {jtag_id:#010x}: {target:?}");
        Ok(target.map(String::from))
    }
}

#[cfg(feature = "mcd")]
fn mcd_bus(
    probe: &mut Probe,
) -> Option<&mut dyn probe_rs::architecture::tricore::bus::TriCoreBusAccess> {
    probe
        .try_into::<mcd::McdProbe>()
        .and_then(|mcd| mcd.try_as_tricore_bus())
}

#[cfg(not(feature = "mcd"))]
fn mcd_bus(
    _probe: &mut Probe,
) -> Option<&mut dyn probe_rs::architecture::tricore::bus::TriCoreBusAccess> {
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tc275_is_detected_by_its_jtag_id() {
        assert_eq!(target_by_jtag_id(0x401D_A083), Some("TC275"));
        assert_eq!(target_by_jtag_id(0x501D_A083), Some("TC275"));
        // Not Infineon.
        assert_eq!(target_by_jtag_id(0x4BA0_0477), None);
    }

    #[test]
    fn tc36x_is_detected_by_its_jtag_id() {
        assert_eq!(target_by_jtag_id(0x1020_9083), Some("TC36x"));
    }

    #[test]
    fn tc3xx_chips_read_the_tc3xx_system_timer() {
        assert_eq!(
            AurixSequence::create("TC36x").stm0_tim0sv(),
            ocds::STM0_TIM0SV_TC3XX
        );
        assert_eq!(
            AurixSequence::create("TC275").stm0_tim0sv(),
            ocds::STM0_TIM0SV
        );
    }

    #[test]
    fn the_targets_load() {
        let families = targets();
        let tc27x = families.iter().find(|f| f.name == "TC27x").unwrap();
        assert_eq!(tc27x.variants[0].name, "TC275");
        assert_eq!(tc27x.variants[0].cores.len(), 3);
        assert!(
            tc27x.variants[0]
                .cores
                .iter()
                .all(|c| c.core_type == CoreType::TriCore)
        );
        let tc36x = families.iter().find(|f| f.name == "TC36x").unwrap();
        assert_eq!(tc36x.variants[0].name, "TC36x");
        assert_eq!(tc36x.variants[0].cores.len(), 2);
    }
}
