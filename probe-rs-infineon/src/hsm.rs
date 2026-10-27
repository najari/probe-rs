//! The HSM (Hardware Security Module) of the AURIX, a Cortex-M3, seen from the debugger.
//!
//! The implementation lives in probe-rs ([`probe_rs::architecture::tricore::hsm`]), where a
//! session can make the HSM one of its cores: a target variant with a core of
//! `core_access_options: !TriCoreHsm {}` (`TC36x_HSM`) returns it from `Session::core`. This
//! module offers the same items for direct use on the TriCore bus.

pub use probe_rs::architecture::tricore::hsm::*;
