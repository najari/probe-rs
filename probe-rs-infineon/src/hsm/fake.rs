//! A fake HSM bridge for the unit tests: the host bus of a TriCore core with `DBGBASE` and the
//! window over a sparse HSM memory, and optionally a Cortex-M3 behind it that answers the
//! debug registers.

use std::collections::HashMap;

use probe_rs::{Error, MemoryInterface};

use super::{DBGBASE, DBGMEM, HSM_ID, HsmFpb, WINDOW_SIZE, other};

const DFSR: u32 = 0xE000_ED30;
const DHCSR: u32 = 0xE000_EDF0;
const DCRSR: u32 = 0xE000_EDF4;
const DCRDR: u32 = 0xE000_EDF8;
const FP_CTRL: u32 = 0xE000_2000;

/// A Cortex-M3 as far as its debug registers go. A step advances the PC by 2; a run stops at
/// the first enabled FPB comparator, or keeps running.
#[derive(Debug, Default)]
pub(super) struct Cpu {
    /// R0-R12, SP, LR, DebugReturnAddress (PC), xPSR, MSP, PSP, -, special.
    pub(super) regs: [u32; 21],
    pub(super) debugen: bool,
    pub(super) halted: bool,
    pub(super) maskints: bool,
    pub(super) dfsr: u32,
    pub(super) dcrdr: u32,
    pub(super) regrdy: bool,
    /// A halt request is not answered (S_HALT stays 0).
    pub(super) ignores_halt: bool,
    /// For each step taken: whether interrupts were masked.
    pub(super) steps: Vec<bool>,
}

impl Cpu {
    pub(super) fn halted_at(pc: u32) -> Self {
        let mut cpu = Self {
            debugen: true,
            halted: true,
            ..Self::default()
        };
        cpu.regs[15] = pc;
        cpu
    }

    fn dhcsr(&self) -> u32 {
        u32::from(self.debugen)
            | (u32::from(self.halted) << 1)
            | (u32::from(self.maskints) << 3)
            | (u32::from(self.regrdy) << 16)
            | (u32::from(self.halted) << 17)
    }

    /// The breakpoint the FPB in `memory` holds at `pc`, or the first one, if it is enabled.
    fn breakpoint(memory: &HashMap<u32, u32>, pc: Option<u32>) -> Option<u32> {
        if memory.get(&FP_CTRL).copied().unwrap_or(0) & 1 == 0 {
            return None;
        }
        (0..8u32)
            .filter_map(|unit| {
                let comp = memory.get(&(0xE000_2008 + 4 * unit)).copied().unwrap_or(0);
                let comph = memory.get(&(0xE000_2028 + 4 * unit)).copied().unwrap_or(0);
                HsmFpb::breakpoint_address(comp, comph)
            })
            .find(|address| pc.is_none_or(|pc| pc == *address))
    }

    fn write_dhcsr(&mut self, value: u32, memory: &HashMap<u32, u32>) {
        if value >> 16 != 0xA05F {
            return;
        }
        self.debugen = value & 1 != 0;
        if !self.debugen {
            self.halted = false;
            return;
        }
        if self.halted {
            self.maskints = value & 0b1000 != 0;
        }
        let halt = value & 0b10 != 0;
        let step = value & 0b100 != 0;
        if halt {
            if !self.halted && !self.ignores_halt {
                self.halted = true;
                self.dfsr |= 1;
            }
        } else if step && self.halted {
            if Self::breakpoint(memory, Some(self.regs[15])).is_some() {
                // The breakpoint at the PC matches again: no progress.
                self.dfsr |= 0b10;
            } else {
                self.regs[15] += 2;
                self.dfsr |= 1;
            }
            self.steps.push(self.maskints);
        } else {
            self.halted = false;
            if let Some(address) = Self::breakpoint(memory, None) {
                self.regs[15] = address;
                self.halted = true;
                self.dfsr |= 0b10;
            }
        }
    }

    fn write_dcrsr(&mut self, value: u32) {
        if !self.halted {
            self.regrdy = false;
            return;
        }
        let regsel = (value & 0x1F) as usize;
        if value & (1 << 16) != 0 {
            self.regs[regsel] = self.dcrdr;
        } else {
            self.dcrdr = self.regs[regsel];
        }
        self.regrdy = true;
    }
}

/// The host bus of a TriCore core with the HSM bridge. Every host write is logged.
#[derive(Default)]
pub(super) struct Host {
    /// HSM words by HSM address (what no [`Cpu`] register answers).
    pub(super) hsm: HashMap<u32, u32>,
    /// The value of `DBGBASE`.
    pub(super) dbgbase: u32,
    /// HSM debug is not enabled: `DBGBASE` and the window fail.
    pub(super) closed: bool,
    pub(super) cpu: Option<Cpu>,
    pub(super) writes: Vec<(u64, u32)>,
}

impl Host {
    pub(super) fn with_cpu(cpu: Cpu) -> Self {
        let mut host = Self {
            cpu: Some(cpu),
            ..Self::default()
        };
        // NUM_LIT 2, NUM_CODE 6: the Cortex-M3's FPB.
        host.hsm.insert(FP_CTRL, 0x0000_0260);
        host
    }

    fn bus_error(address: u64) -> Error {
        other(format!("bus error at {address:#010x}"))
    }

    fn in_window(&self, address: u64) -> Option<u32> {
        ((DBGMEM..DBGMEM + u64::from(WINDOW_SIZE)).contains(&address) && !self.closed)
            .then(|| (self.dbgbase & 0xFFFF_0000) | (address - DBGMEM) as u32)
    }

    fn read(&mut self, address: u64) -> Result<u32, Error> {
        match address {
            HSM_ID => return Ok(0x00BE_C002),
            DBGBASE if !self.closed => return Ok(self.dbgbase),
            _ => {}
        }
        let hsm = self
            .in_window(address)
            .ok_or_else(|| Self::bus_error(address))?;
        if let Some(cpu) = &self.cpu {
            match hsm {
                DHCSR => return Ok(cpu.dhcsr()),
                DCRDR => return Ok(cpu.dcrdr),
                DFSR => return Ok(cpu.dfsr),
                _ => {}
            }
        }
        Ok(self.hsm.get(&hsm).copied().unwrap_or(0))
    }

    fn write(&mut self, address: u64, value: u32) -> Result<(), Error> {
        self.writes.push((address, value));
        if address == DBGBASE && !self.closed {
            self.dbgbase = value & 0xFFFF_0000;
            return Ok(());
        }
        let hsm = self
            .in_window(address)
            .ok_or_else(|| Self::bus_error(address))?;
        if let Some(cpu) = &mut self.cpu {
            match hsm {
                DHCSR => {
                    cpu.write_dhcsr(value, &self.hsm);
                    return Ok(());
                }
                DCRSR => {
                    cpu.write_dcrsr(value);
                    return Ok(());
                }
                DCRDR => {
                    cpu.dcrdr = value;
                    return Ok(());
                }
                DFSR => {
                    cpu.dfsr &= !value;
                    return Ok(());
                }
                // ENABLE is written with KEY; NUM_CODE and NUM_LIT stay.
                FP_CTRL if value & 0b10 != 0 => {
                    let ctrl = self.hsm.entry(FP_CTRL).or_default();
                    *ctrl = (*ctrl & !1) | (value & 1);
                    return Ok(());
                }
                _ => {}
            }
        }
        self.hsm.insert(hsm, value);
        Ok(())
    }

    pub(super) fn dbgbase_writes(&self) -> Vec<u32> {
        self.writes
            .iter()
            .filter(|(address, _)| *address == DBGBASE)
            .map(|(_, value)| *value)
            .collect()
    }

    /// The values written to an HSM address through the window, in order.
    pub(super) fn hsm_writes(&self, hsm: u32) -> Vec<u32> {
        let mut dbgbase = 0;
        let mut values = Vec::new();
        for &(address, value) in &self.writes {
            if address == DBGBASE {
                dbgbase = value & 0xFFFF_0000;
            } else if address >= DBGMEM
                && (dbgbase | (address - DBGMEM) as u32) == hsm
                && address < DBGMEM + u64::from(WINDOW_SIZE)
            {
                values.push(value);
            }
        }
        values
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
