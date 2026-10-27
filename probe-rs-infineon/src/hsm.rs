//! The HSM (Hardware Security Module) of the AURIX, a Cortex-M3, seen from the debugger.
//!
//! AURIX has no ARM debug port: the debugger reaches the HSM through the HSM block on the SPB
//! (`0xF0040000`-`0xF005FFFF`, TC3xx and TC27x). The host side bridge register `DBGBASE` picks
//! the upper 16 bits of an HSM address and the 64 KiB debug memory window `DBGMEM` at
//! `0xF0050000` reads and writes there, 32 bits at a time ([`HsmWindow`]). All the Cortex-M
//! debug registers (DWT, FPB, `DHCSR`-`DEMCR`) are in the HSM page `0xE0000000`.
//!
//! The window only opens once the HSM firmware enabled HSM debug (`DBGCTRL.HSM = 11`, HSM side);
//! until then `DBGBASE` and the window fail with a bus error ([`HsmStatus`]).
//!
//! The FPB of the HSM is extended by Infineon with `FP_COMPH0`-`FP_COMPH7`: the FPB revision 1
//! of the Cortex-M3 only compares the address bits 28:2, but the HSM code runs from the PFLASH at
//! `0x80000000`. [`HsmFpb`] writes the three missing address bits 31:29 there.
//!
//! Sources: the Infineon HSM sample code, AP32391 `hsm_Interface.h` (host side bridge registers,
//! `DBGMEM` `0xF0050000`-`0xF005FFFF`) and `bridge.h` (`HSM_DBGMEM_Type`, 32 bits), AP32349
//! `debugextension_hsm.h` and `debugextension_hsm_internal.h` (`FP_COMPH0`-`7` at `0xE0002028`,
//! 3 writable bits). That `FP_COMPHn` holds the address bits 31:29 is inferred (the three bits
//! that `FP_COMPn.COMP` lacks), not written there; no HSM with HSM debug enabled was tried yet.

use probe_rs::{Error, MemoryInterface};

/// `HSM_ID`, the module identifier of the bridge (host side).
pub const HSM_ID: u64 = 0xF004_0008;
/// `DBGBASE`: the upper 16 bits (31:16) of the HSM address the window shows.
pub const DBGBASE: u64 = 0xF004_1010;
/// `HSMOTGB`, the OCDS suspend and trigger bus control of the HSM.
pub const HSMOTGB: u64 = 0xF004_1020;
/// `DBGMEM`, the debug memory window.
pub const DBGMEM: u64 = 0xF005_0000;
/// The size of the window and of the HSM pages it shows.
pub const WINDOW_SIZE: u32 = 0x1_0000;

fn other(message: impl Into<String>) -> Error {
    Error::Other(message.into())
}

/// What the debugger can reach of the HSM, read without changing anything.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HsmStatus {
    /// `HSM_ID`, if the bridge answers (TC36x with the HSM not booted: yes; TC275: no).
    pub id: Option<u32>,
    /// `DBGBASE`, if it can be read: the HSM firmware enabled HSM debug.
    pub dbgbase: Option<u32>,
}

impl HsmStatus {
    /// Reads `HSM_ID` and `DBGBASE` through `host`, the bus of a TriCore core. A bus error on
    /// either is an answer, not an error.
    pub fn read<H: MemoryInterface + ?Sized>(host: &mut H) -> Self {
        Self {
            id: host.read_word_32(HSM_ID).ok(),
            dbgbase: host.read_word_32(DBGBASE).ok(),
        }
    }

    /// Whether the debug window is open.
    pub fn debug_open(&self) -> bool {
        self.dbgbase.is_some()
    }
}

/// The HSM address space, through `DBGBASE` and the debug memory window on `host`, the bus of a
/// TriCore core.
///
/// Every access is made of aligned 32-bit window accesses, the width of `DBGMEM`. 8 and 16 bit
/// writes read, change and write back the word they are in: debug registers such as `DHCSR`
/// must be written as whole words.
pub struct HsmWindow<'a, H: MemoryInterface + ?Sized> {
    host: &'a mut H,
    /// The page `DBGBASE` was last set to; `None` until it is set, or when it may have changed.
    page: Option<u32>,
}

impl<'a, H: MemoryInterface + ?Sized> HsmWindow<'a, H> {
    pub fn new(host: &'a mut H) -> Self {
        Self { host, page: None }
    }

    /// Forgets the page `DBGBASE` shows, so that the next access sets it again: after a reset
    /// of the HSM, or when something else may have written `DBGBASE`.
    pub fn invalidate(&mut self) {
        self.page = None;
    }

    /// The host address of the HSM word at `address` (aligned), after setting `DBGBASE` to its
    /// page when it shows another one.
    fn select(&mut self, address: u32) -> Result<u64, Error> {
        let page = address & !(WINDOW_SIZE - 1);
        if self.page != Some(page) {
            self.page = None;
            self.host.write_word_32(DBGBASE, page)?;
            self.page = Some(page);
        }
        Ok(DBGMEM + u64::from(address & (WINDOW_SIZE - 1)))
    }

    fn hsm_address(address: u64, len: usize) -> Result<u32, Error> {
        let start = u32::try_from(address)
            .map_err(|_| other(format!("{address:#x} is not an HSM address (32 bits)")))?;
        let last = (len as u64).saturating_sub(1);
        if u64::from(start) + last > u64::from(u32::MAX) {
            return Err(other(format!(
                "{len} bytes at {start:#010x} go past the end of the HSM address space"
            )));
        }
        Ok(start)
    }

    /// Reads aligned words, one page at a time.
    fn read_words(&mut self, address: u32, data: &mut [u32]) -> Result<(), Error> {
        let mut address = address;
        let mut rest = data;
        while !rest.is_empty() {
            let in_page = ((WINDOW_SIZE - (address & (WINDOW_SIZE - 1))) / 4) as usize;
            let (chunk, tail) = rest.split_at_mut(in_page.min(rest.len()));
            let host = self.select(address)?;
            self.host.read_32(host, chunk)?;
            address = address.wrapping_add(chunk.len() as u32 * 4);
            rest = tail;
        }
        Ok(())
    }

    /// Writes aligned words, one page at a time.
    fn write_words(&mut self, address: u32, data: &[u32]) -> Result<(), Error> {
        let mut address = address;
        let mut rest = data;
        while !rest.is_empty() {
            let in_page = ((WINDOW_SIZE - (address & (WINDOW_SIZE - 1))) / 4) as usize;
            let (chunk, tail) = rest.split_at(in_page.min(rest.len()));
            let host = self.select(address)?;
            self.host.write_32(host, chunk)?;
            address = address.wrapping_add(chunk.len() as u32 * 4);
            rest = tail;
        }
        Ok(())
    }

    /// Reads `data.len()` bytes at any `address` through the words that hold them.
    fn read_bytes(&mut self, address: u64, data: &mut [u8]) -> Result<(), Error> {
        if data.is_empty() {
            return Ok(());
        }
        let start = Self::hsm_address(address, data.len())?;
        let first = start & !3;
        let skip = (start - first) as usize;
        let mut words = vec![0u32; (skip + data.len()).div_ceil(4)];
        self.read_words(first, &mut words)?;
        let bytes: Vec<u8> = words.iter().flat_map(|word| word.to_le_bytes()).collect();
        data.copy_from_slice(&bytes[skip..skip + data.len()]);
        Ok(())
    }

    /// Writes `data` at any `address`: the words it covers only in part are read first.
    fn write_bytes(&mut self, address: u64, data: &[u8]) -> Result<(), Error> {
        if data.is_empty() {
            return Ok(());
        }
        let start = Self::hsm_address(address, data.len())?;
        let first = start & !3;
        let skip = (start - first) as usize;
        let count = (skip + data.len()).div_ceil(4);
        let mut words = vec![0u32; count];
        if skip != 0 {
            self.read_words(first, &mut words[..1])?;
        }
        let tail = (skip + data.len()) % 4;
        if tail != 0 && (count > 1 || skip == 0) {
            let last = first + (count as u32 - 1) * 4;
            self.read_words(last, &mut words[count - 1..])?;
        }
        let mut bytes: Vec<u8> = words.iter().flat_map(|word| word.to_le_bytes()).collect();
        bytes[skip..skip + data.len()].copy_from_slice(data);
        let words: Vec<u32> = bytes
            .as_chunks::<4>()
            .0
            .iter()
            .map(|word| u32::from_le_bytes(*word))
            .collect();
        self.write_words(first, &words)
    }
}

fn aligned(address: u64, alignment: usize) -> Result<(), Error> {
    if address.is_multiple_of(alignment as u64) {
        Ok(())
    } else {
        // `MemoryNotAlignedError` is not public outside probe-rs.
        Err(other(format!(
            "memory access to address {address:#x} was not aligned to {alignment} bytes"
        )))
    }
}

impl<H: MemoryInterface + ?Sized> MemoryInterface for HsmWindow<'_, H> {
    fn supports_native_64bit_access(&mut self) -> bool {
        false
    }

    fn read_64(&mut self, address: u64, data: &mut [u64]) -> Result<(), Error> {
        aligned(address, 8)?;
        let mut words = vec![0u32; data.len() * 2];
        self.read_32(address, &mut words)?;
        for (value, pair) in data.iter_mut().zip(words.as_chunks::<2>().0) {
            *value = u64::from(pair[0]) | (u64::from(pair[1]) << 32);
        }
        Ok(())
    }

    fn read_32(&mut self, address: u64, data: &mut [u32]) -> Result<(), Error> {
        aligned(address, 4)?;
        let start = Self::hsm_address(address, data.len() * 4)?;
        self.read_words(start, data)
    }

    fn read_16(&mut self, address: u64, data: &mut [u16]) -> Result<(), Error> {
        aligned(address, 2)?;
        let mut bytes = vec![0u8; data.len() * 2];
        self.read_bytes(address, &mut bytes)?;
        for (value, pair) in data.iter_mut().zip(bytes.as_chunks::<2>().0) {
            *value = u16::from_le_bytes(*pair);
        }
        Ok(())
    }

    fn read_8(&mut self, address: u64, data: &mut [u8]) -> Result<(), Error> {
        self.read_bytes(address, data)
    }

    fn write_64(&mut self, address: u64, data: &[u64]) -> Result<(), Error> {
        aligned(address, 8)?;
        let words: Vec<u32> = data
            .iter()
            .flat_map(|value| [*value as u32, (*value >> 32) as u32])
            .collect();
        self.write_32(address, &words)
    }

    fn write_32(&mut self, address: u64, data: &[u32]) -> Result<(), Error> {
        aligned(address, 4)?;
        let start = Self::hsm_address(address, data.len() * 4)?;
        self.write_words(start, data)
    }

    fn write_16(&mut self, address: u64, data: &[u16]) -> Result<(), Error> {
        aligned(address, 2)?;
        let bytes: Vec<u8> = data.iter().flat_map(|value| value.to_le_bytes()).collect();
        self.write_bytes(address, &bytes)
    }

    fn write_8(&mut self, address: u64, data: &[u8]) -> Result<(), Error> {
        self.write_bytes(address, data)
    }

    fn supports_8bit_transfers(&self) -> Result<bool, Error> {
        // Made of 32-bit window accesses.
        Ok(true)
    }

    fn flush(&mut self) -> Result<(), Error> {
        self.host.flush()
    }
}

/// The flash patch and breakpoint unit of the HSM, with Infineon's `FP_COMPH` extension.
///
/// Only the code comparators are used, as instruction breakpoints. A comparator is written
/// `FP_COMPHn` first and `FP_COMPn` (with its enable bit) last, and cleared the other way
/// round, so that it never matches with the high address bits of another breakpoint.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HsmFpb {
    /// `FP_CTRL.NUM_CODE`, at most [`HsmFpb::COMPH_COUNT`] of them are usable.
    code_comparators: usize,
}

impl HsmFpb {
    /// `FP_CTRL`.
    pub const FP_CTRL: u64 = 0xE000_2000;
    /// `FP_COMP0`; `FP_COMPn` follows every 4 bytes.
    pub const FP_COMP0: u64 = 0xE000_2008;
    /// `FP_COMPH0` (Infineon); `FP_COMPHn` follows every 4 bytes.
    pub const FP_COMPH0: u64 = 0xE000_2028;
    /// The number of `FP_COMPH` registers.
    pub const COMPH_COUNT: usize = 8;

    /// `FP_CTRL.ENABLE` and `FP_CTRL.KEY` (the write enable of `ENABLE`).
    const CTRL_ENABLE: u32 = 0b11;
    /// `FP_COMPn.ENABLE`.
    const COMP_ENABLE: u32 = 1;
    /// `FP_COMPn.COMP`: the address bits 28:2.
    const COMP_ADDRESS: u32 = 0x1FFF_FFFC;
    /// `FP_COMPn.REPLACE` (bits 31:30): break on the lower (`01`) or upper (`10`) halfword.
    const REPLACE_LOWER: u32 = 0b01 << 30;
    const REPLACE_UPPER: u32 = 0b10 << 30;
    const REPLACE_BOTH: u32 = 0b11 << 30;

    /// Reads `FP_CTRL` through `memory` (an [`HsmWindow`]): a revision 1 FPB is expected, the
    /// one of the Cortex-M3.
    pub fn read(memory: &mut dyn MemoryInterface) -> Result<Self, Error> {
        Self::from_ctrl(memory.read_word_32(Self::FP_CTRL)?)
    }

    /// From the value of `FP_CTRL`.
    pub fn from_ctrl(ctrl: u32) -> Result<Self, Error> {
        let revision = ctrl >> 28;
        if revision != 0 {
            return Err(other(format!(
                "the HSM FPB is of revision {} (FP_CTRL {ctrl:#010x}), not 1 as the Cortex-M3's",
                revision + 1
            )));
        }
        let code_comparators = (((ctrl >> 8) & 0x70) | ((ctrl >> 4) & 0xF)) as usize;
        Ok(Self { code_comparators })
    }

    /// The comparators usable as breakpoints.
    pub fn comparators(&self) -> usize {
        self.code_comparators.min(Self::COMPH_COUNT)
    }

    /// `FP_COMPn` and `FP_COMPHn` for an enabled breakpoint at `address` (an even address:
    /// the HSM runs Thumb code).
    pub fn breakpoint_configuration(address: u32) -> Result<(u32, u32), Error> {
        if address & 1 != 0 {
            return Err(other(format!(
                "{address:#010x} is odd: an HSM breakpoint is on a halfword"
            )));
        }
        let replace = if address & 2 == 0 {
            Self::REPLACE_LOWER
        } else {
            Self::REPLACE_UPPER
        };
        let comp = replace | (address & Self::COMP_ADDRESS) | Self::COMP_ENABLE;
        Ok((comp, address >> 29))
    }

    /// The address a comparator breaks on, if it is enabled.
    pub fn breakpoint_address(comp: u32, comph: u32) -> Option<u32> {
        if comp & Self::COMP_ENABLE == 0 {
            return None;
        }
        let upper = match comp & Self::REPLACE_BOTH {
            Self::REPLACE_UPPER => 2,
            // `00` remaps, it does not break; `01` and `11` include the lower halfword.
            0 => return None,
            _ => 0,
        };
        Some(((comph & 0b111) << 29) | (comp & Self::COMP_ADDRESS) | upper)
    }

    fn check(&self, unit: usize) -> Result<(), Error> {
        if unit >= self.comparators() {
            return Err(other(format!(
                "the HSM FPB has {} code comparators, not {}",
                self.comparators(),
                unit + 1
            )));
        }
        Ok(())
    }

    fn registers(unit: usize) -> (u64, u64) {
        let offset = 4 * unit as u64;
        (Self::FP_COMP0 + offset, Self::FP_COMPH0 + offset)
    }

    /// Enables the FPB (`FP_CTRL.ENABLE` with `KEY`).
    pub fn enable(&self, memory: &mut dyn MemoryInterface) -> Result<(), Error> {
        memory.write_word_32(Self::FP_CTRL, Self::CTRL_ENABLE)
    }

    /// Sets comparator `unit` to break at `address`.
    pub fn set_breakpoint(
        &self,
        memory: &mut dyn MemoryInterface,
        unit: usize,
        address: u32,
    ) -> Result<(), Error> {
        self.check(unit)?;
        let (comp, comph) = Self::breakpoint_configuration(address)?;
        let (comp_register, comph_register) = Self::registers(unit);
        // Disabled while the high bits change, then enabled with the address.
        memory.write_word_32(comp_register, 0)?;
        memory.write_word_32(comph_register, comph)?;
        memory.write_word_32(comp_register, comp)
    }

    /// Clears comparator `unit`.
    pub fn clear_breakpoint(
        &self,
        memory: &mut dyn MemoryInterface,
        unit: usize,
    ) -> Result<(), Error> {
        self.check(unit)?;
        let (comp_register, comph_register) = Self::registers(unit);
        memory.write_word_32(comp_register, 0)?;
        memory.write_word_32(comph_register, 0)
    }

    /// The address comparator `unit` breaks on, if any.
    pub fn breakpoint(
        &self,
        memory: &mut dyn MemoryInterface,
        unit: usize,
    ) -> Result<Option<u32>, Error> {
        self.check(unit)?;
        let (comp_register, comph_register) = Self::registers(unit);
        let comp = memory.read_word_32(comp_register)?;
        let comph = memory.read_word_32(comph_register)?;
        Ok(Self::breakpoint_address(comp, comph))
    }
}

#[cfg(test)]
mod tests {
    use std::collections::HashMap;

    use super::*;

    /// The host bus of a TriCore core with the HSM bridge: `DBGBASE` and the window over a
    /// sparse HSM memory. Every host write is logged.
    #[derive(Default)]
    struct Host {
        /// HSM words by HSM address.
        hsm: HashMap<u32, u32>,
        /// The value of `DBGBASE`.
        dbgbase: u32,
        /// HSM debug is not enabled: `DBGBASE` and the window fail.
        closed: bool,
        writes: Vec<(u64, u32)>,
    }

    impl Host {
        fn bus_error(address: u64) -> Error {
            other(format!("bus error at {address:#010x}"))
        }

        fn read(&mut self, address: u64) -> Result<u32, Error> {
            match address {
                HSM_ID => Ok(0x00BE_C002),
                DBGBASE if !self.closed => Ok(self.dbgbase),
                a if (DBGMEM..DBGMEM + u64::from(WINDOW_SIZE)).contains(&a) && !self.closed => {
                    let hsm = (self.dbgbase & 0xFFFF_0000) | (a - DBGMEM) as u32;
                    Ok(self.hsm.get(&hsm).copied().unwrap_or(0))
                }
                _ => Err(Self::bus_error(address)),
            }
        }

        fn write(&mut self, address: u64, value: u32) -> Result<(), Error> {
            self.writes.push((address, value));
            match address {
                DBGBASE if !self.closed => {
                    self.dbgbase = value & 0xFFFF_0000;
                    Ok(())
                }
                a if (DBGMEM..DBGMEM + u64::from(WINDOW_SIZE)).contains(&a) && !self.closed => {
                    let hsm = (self.dbgbase & 0xFFFF_0000) | (a - DBGMEM) as u32;
                    self.hsm.insert(hsm, value);
                    Ok(())
                }
                _ => Err(Self::bus_error(address)),
            }
        }

        fn dbgbase_writes(&self) -> Vec<u32> {
            self.writes
                .iter()
                .filter(|(address, _)| *address == DBGBASE)
                .map(|(_, value)| *value)
                .collect()
        }
    }

    impl MemoryInterface for Host {
        fn supports_native_64bit_access(&mut self) -> bool {
            false
        }
        fn read_64(&mut self, _: u64, _: &mut [u64]) -> Result<(), Error> {
            unreachable!("the window reads 32 bits")
        }
        fn read_32(&mut self, address: u64, data: &mut [u32]) -> Result<(), Error> {
            for (i, word) in data.iter_mut().enumerate() {
                *word = self.read(address + 4 * i as u64)?;
            }
            Ok(())
        }
        fn read_16(&mut self, _: u64, _: &mut [u16]) -> Result<(), Error> {
            unreachable!("the window reads 32 bits")
        }
        fn read_8(&mut self, _: u64, _: &mut [u8]) -> Result<(), Error> {
            unreachable!("the window reads 32 bits")
        }
        fn write_64(&mut self, _: u64, _: &[u64]) -> Result<(), Error> {
            unreachable!("the window writes 32 bits")
        }
        fn write_32(&mut self, address: u64, data: &[u32]) -> Result<(), Error> {
            for (i, word) in data.iter().enumerate() {
                self.write(address + 4 * i as u64, *word)?;
            }
            Ok(())
        }
        fn write_16(&mut self, _: u64, _: &[u16]) -> Result<(), Error> {
            unreachable!("the window writes 32 bits")
        }
        fn write_8(&mut self, _: u64, _: &[u8]) -> Result<(), Error> {
            unreachable!("the window writes 32 bits")
        }
        fn supports_8bit_transfers(&self) -> Result<bool, Error> {
            Ok(false)
        }
        fn flush(&mut self) -> Result<(), Error> {
            Ok(())
        }
    }

    #[test]
    fn the_status_tells_a_closed_window_from_an_open_one() {
        let mut host = Host {
            closed: true,
            ..Host::default()
        };
        // TC36x, HSM not booted (2026-10-27): HSM_ID reads, DBGBASE is a bus error.
        let status = HsmStatus::read(&mut host);
        assert_eq!(status.id, Some(0x00BE_C002));
        assert!(!status.debug_open());
        host.closed = false;
        host.dbgbase = 0xE000_0000;
        assert_eq!(HsmStatus::read(&mut host).dbgbase, Some(0xE000_0000));
        assert!(host.writes.is_empty(), "the status only reads");
    }

    #[test]
    fn a_debug_register_is_read_through_its_page() {
        let mut host = Host::default();
        host.hsm.insert(0xE000_EDF0, 0x0003_0003);
        let mut window = HsmWindow::new(&mut host);
        // DHCSR: DBGBASE 0xE0000000, host 0xF005EDF0.
        assert_eq!(window.read_word_32(0xE000_EDF0).unwrap(), 0x0003_0003);
        assert_eq!(window.read_word_32(0xE000_EDFC).unwrap(), 0);
        window.write_word_32(0xE000_EDF0, 0xA05F_0003).unwrap();
        assert_eq!(host.dbgbase_writes(), [0xE000_0000], "the page is set once");
        assert!(host.writes.contains(&(0xF005_EDF0, 0xA05F_0003)));
        assert_eq!(host.hsm[&0xE000_EDF0], 0xA05F_0003);
    }

    #[test]
    fn an_access_across_pages_sets_each_page() {
        let mut host = Host::default();
        host.hsm.insert(0x2000_FFFC, 0x1111_1111);
        host.hsm.insert(0x2001_0000, 0x2222_2222);
        let mut window = HsmWindow::new(&mut host);
        let mut words = [0u32; 2];
        window.read_32(0x2000_FFFC, &mut words).unwrap();
        assert_eq!(words, [0x1111_1111, 0x2222_2222]);
        window.write_32(0x2000_FFFC, &[3, 4]).unwrap();
        assert_eq!(host.hsm[&0x2000_FFFC], 3);
        assert_eq!(host.hsm[&0x2001_0000], 4);
        assert_eq!(
            host.dbgbase_writes(),
            [0x2000_0000, 0x2001_0000, 0x2000_0000, 0x2001_0000]
        );
    }

    #[test]
    fn bytes_and_halfwords_read_change_and_write_words() {
        let mut host = Host::default();
        host.hsm.insert(0x2000_0000, 0x4433_2211);
        host.hsm.insert(0x2000_0004, 0x8877_6655);
        let mut window = HsmWindow::new(&mut host);
        let mut bytes = [0u8; 4];
        window.read_8(0x2000_0002, &mut bytes).unwrap();
        assert_eq!(bytes, [0x33, 0x44, 0x55, 0x66]);
        assert_eq!(window.read_word_16(0x2000_0006).unwrap(), 0x8877);
        window.write_8(0x2000_0003, &[0xAA, 0xBB]).unwrap();
        window.write_word_16(0x2000_0000, 0xCCDD).unwrap();
        // Unaligned through the default `write`, which uses `write_8` at the edges.
        window.write(0x2000_0005, &[0xEE]).unwrap();
        assert_eq!(host.hsm[&0x2000_0000], 0xAA33_CCDD);
        assert_eq!(host.hsm[&0x2000_0004], 0x8877_EEBB);
        assert!(
            host.writes
                .iter()
                .all(|(address, _)| *address == DBGBASE || address % 4 == 0),
            "the window is only written as aligned words"
        );
    }

    #[test]
    fn a_closed_window_fails_and_is_set_again_later() {
        let mut host = Host {
            closed: true,
            ..Host::default()
        };
        let mut window = HsmWindow::new(&mut host);
        assert!(window.read_word_32(0xE000_EDF0).is_err());
        assert_eq!(window.page, None, "a failed DBGBASE write leaves no page");
        window.host.closed = false;
        window.read_word_32(0xE000_EDF0).unwrap();
        window.invalidate();
        window.read_word_32(0xE000_EDF0).unwrap();
        assert_eq!(
            host.dbgbase_writes(),
            [0xE000_0000, 0xE000_0000, 0xE000_0000]
        );
    }

    #[test]
    fn addresses_past_32_bits_are_refused() {
        let mut host = Host::default();
        let mut window = HsmWindow::new(&mut host);
        assert!(window.read_word_32(0x1_0000_0000).is_err());
        let mut words = [0u32; 2];
        assert!(window.read_32(0xFFFF_FFFC, &mut words).is_err());
        assert!(window.read_word_32(0x2000_0002).is_err(), "unaligned word");
        assert!(host.writes.is_empty());
    }

    #[test]
    fn a_flash_breakpoint_needs_the_high_bits() {
        // HSM code in the PFLASH (0x80000000): FP_COMPH = 0b100.
        assert_eq!(
            HsmFpb::breakpoint_configuration(0x8000_4000).unwrap(),
            (0x4000_4001, 0b100)
        );
        assert_eq!(
            HsmFpb::breakpoint_configuration(0x8001_8102).unwrap(),
            (0x8001_8101, 0b100)
        );
        // HSM RAM (0x20000000): FP_COMPH = 0b001, which the plain Cortex-M3 FPB cannot reach.
        assert_eq!(
            HsmFpb::breakpoint_configuration(0x2000_1000).unwrap(),
            (0x4000_1001, 0b001)
        );
        assert!(HsmFpb::breakpoint_configuration(0x8000_4001).is_err());
        for address in [
            0x8000_4000,
            0x8001_8102,
            0x2000_1000,
            0x0000_0100,
            0xE000_0002,
        ] {
            let (comp, comph) = HsmFpb::breakpoint_configuration(address).unwrap();
            assert_eq!(HsmFpb::breakpoint_address(comp, comph), Some(address));
        }
        assert_eq!(
            HsmFpb::breakpoint_address(0x4000_4000, 0b100),
            None,
            "disabled"
        );
        assert_eq!(
            HsmFpb::breakpoint_address(0x0000_4001, 0b100),
            None,
            "remap"
        );
    }

    #[test]
    fn the_comparator_count_comes_from_fp_ctrl() {
        // NUM_LIT 2, NUM_CODE 6: the Cortex-M3's.
        assert_eq!(HsmFpb::from_ctrl(0x0000_0260).unwrap().comparators(), 6);
        // NUM_CODE[6:4] in bits 14:12; more than 8 are limited by FP_COMPH.
        assert_eq!(HsmFpb::from_ctrl(0x0000_1020).unwrap().comparators(), 8);
        assert!(HsmFpb::from_ctrl(0x1000_0260).is_err(), "revision 2");
    }

    #[test]
    fn breakpoints_are_set_and_cleared_through_the_window() {
        let mut host = Host::default();
        host.hsm.insert(0xE000_2000, 0x0000_0260);
        let mut window = HsmWindow::new(&mut host);
        let fpb = HsmFpb::read(&mut window).unwrap();
        fpb.enable(&mut window).unwrap();
        fpb.set_breakpoint(&mut window, 5, 0x8000_4002).unwrap();
        assert_eq!(fpb.breakpoint(&mut window, 5).unwrap(), Some(0x8000_4002));
        assert_eq!(fpb.breakpoint(&mut window, 0).unwrap(), None);
        assert!(fpb.set_breakpoint(&mut window, 6, 0x8000_4000).is_err());
        fpb.clear_breakpoint(&mut window, 5).unwrap();
        assert_eq!(fpb.breakpoint(&mut window, 5).unwrap(), None);

        let window_writes: Vec<(u64, u32)> = host
            .writes
            .iter()
            .copied()
            .filter(|(address, _)| *address != DBGBASE)
            .collect();
        // FP_CTRL; FP_COMP5 off, FP_COMPH5, FP_COMP5 on; FP_COMP5 off, FP_COMPH5 0.
        assert_eq!(
            window_writes,
            [
                (0xF005_2000, 0b11),
                (0xF005_201C, 0),
                (0xF005_203C, 0b100),
                (0xF005_201C, 0x8000_4001),
                (0xF005_201C, 0),
                (0xF005_203C, 0),
            ]
        );
        assert_eq!(host.dbgbase_writes(), [0xE000_0000], "all in one page");
    }
}
