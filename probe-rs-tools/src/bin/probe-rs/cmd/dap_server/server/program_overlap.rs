//! A session for several cores writes each distinct program binary once. Two binaries that load
//! into the same flash sector would lose the first one: the flash loader fills the rest of each
//! sector it writes (with 0, or the old content with `restore_unwritten_bytes` only for the
//! bytes that were there before the session). The check runs before anything is written.

use std::path::{Path, PathBuf};

use object::elf;
use object::read::elf::{FileHeader, ProgramHeader};

/// The address ranges (start, length) that a program binary loads: its loadable segments with
/// data in the file, at their load (physical) addresses.
pub(crate) fn load_ranges(data: &[u8]) -> Result<Vec<(u64, u64)>, String> {
    match object::FileKind::parse(data).map_err(|e| e.to_string())? {
        object::FileKind::Elf32 => ranges::<elf::FileHeader32<object::Endianness>>(data),
        object::FileKind::Elf64 => ranges::<elf::FileHeader64<object::Endianness>>(data),
        kind => Err(format!("not an ELF file ({kind:?})")),
    }
}

fn ranges<H: FileHeader<Endian = object::Endianness>>(
    data: &[u8],
) -> Result<Vec<(u64, u64)>, String> {
    let header = H::parse(data).map_err(|e| e.to_string())?;
    let endian = header.endian().map_err(|e| e.to_string())?;
    let headers = header
        .program_headers(endian, data)
        .map_err(|e| e.to_string())?;
    Ok(headers
        .iter()
        .filter(|ph| ph.p_type(endian) == elf::PT_LOAD)
        .map(|ph| (ph.p_paddr(endian).into(), ph.p_filesz(endian).into()))
        .filter(|&(_, len)| len > 0)
        .collect())
}

/// The flash sectors (base, size) of the chip, where they are known (the AURIX PFLASH, at the
/// non-cached addresses: see [`probe_rs_infineon::flash::to_uncached`]).
fn sectors(chip: Option<&str>) -> Option<Vec<(u64, u64)>> {
    let device = probe_rs_infineon::flash::FlashDevice::for_chip(chip?)?;
    Some(
        device
            .sectors()
            .into_iter()
            .map(|s| (s.base as u64, s.size as u64))
            .collect(),
    )
}

/// Refuses program binaries (path, load ranges) of which two load the same bytes, or, where the
/// chip's flash sectors are known, the same flash sector.
pub(crate) fn check(
    chip: Option<&str>,
    binaries: &[(PathBuf, Vec<(u64, u64)>)],
) -> Result<(), String> {
    let sectors = sectors(chip);
    let touched = |ranges: &[(u64, u64)]| -> Vec<u64> {
        let Some(sectors) = &sectors else {
            return Vec::new();
        };
        let mut bases = Vec::new();
        for &(start, len) in ranges {
            let start = probe_rs_infineon::flash::to_uncached(start);
            let end = start + len;
            for &(base, size) in sectors {
                if start < base + size && base < end && !bases.contains(&base) {
                    bases.push(base);
                }
            }
        }
        bases
    };
    for (i, (first, first_ranges)) in binaries.iter().enumerate() {
        for (second, second_ranges) in &binaries[i + 1..] {
            for &(a, a_len) in first_ranges {
                for &(b, b_len) in second_ranges {
                    if a < b + b_len && b < a + a_len {
                        return Err(format!(
                            "{} and {} both load {:#x}: one program binary would overwrite the other",
                            name(first),
                            name(second),
                            a.max(b)
                        ));
                    }
                }
            }
            let second_sectors = touched(second_ranges);
            if let Some(base) = touched(first_ranges)
                .into_iter()
                .find(|base| second_sectors.contains(base))
            {
                return Err(format!(
                    "{} and {} both write the flash sector at {base:#x}: writing the second one \
                     would erase the first one there. Link them to different sectors, or into one \
                     program binary",
                    name(first),
                    name(second)
                ));
            }
        }
    }
    Ok(())
}

fn name(path: &Path) -> String {
    path.file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_else(|| path.display().to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn binary(name: &str, ranges: &[(u64, u64)]) -> (PathBuf, Vec<(u64, u64)>) {
        (PathBuf::from(name), ranges.to_vec())
    }

    #[test]
    fn binaries_in_different_sectors_pass() {
        // TC27x PF1: 0x8021C000 is the last 16 KiB sector, 0x80220000 the first 32 KiB one.
        let binaries = [
            binary("a.elf", &[(0x8021_C000, 0x100)]),
            binary("b.elf", &[(0x8022_0000, 0x100)]),
        ];
        assert_eq!(check(Some("TC275"), &binaries), Ok(()));
    }

    #[test]
    fn binaries_in_one_sector_are_refused() {
        // 0x8021C000 and 0x8021F000 are in the same 16 KiB sector (0xA021C000, non-cached).
        let binaries = [
            binary("a.elf", &[(0x8021_C000, 0x100)]),
            binary("b.elf", &[(0x8021_F000, 0x100)]),
        ];
        let error = check(Some("TC275"), &binaries).unwrap_err();
        assert!(
            error.contains("a.elf and b.elf both write the flash sector at 0xa021c000"),
            "{error}"
        );
    }

    #[test]
    fn the_cached_and_non_cached_address_are_the_same_sector() {
        let binaries = [
            binary("a.elf", &[(0x8021_C000, 0x10)]),
            binary("b.elf", &[(0xA021_D000, 0x10)]),
        ];
        assert!(check(Some("TC275"), &binaries).is_err());
    }

    #[test]
    fn overlapping_bytes_are_refused_without_a_sector_map() {
        let binaries = [
            binary("a.elf", &[(0x2000_0000, 0x100)]),
            binary("b.elf", &[(0x2000_0080, 0x100)]),
            binary("c.elf", &[(0x2000_0200, 0x100)]),
        ];
        let error = check(Some("nRF52840_xxAA"), &binaries).unwrap_err();
        assert!(
            error.contains("a.elf and b.elf both load 0x20000080"),
            "{error}"
        );
        assert_eq!(check(None, &binaries[1..]), Ok(()));
    }

    #[test]
    fn the_load_ranges_are_the_loadable_segments_at_their_load_addresses() {
        // A minimal 32-bit little endian ELF: one PT_LOAD with 4 bytes at paddr 0x80000000
        // (vaddr 0xD0000000, as initialized data), one empty PT_LOAD.
        let mut elf = vec![0u8; 52 + 2 * 32 + 4];
        elf[..4].copy_from_slice(b"\x7fELF");
        elf[4] = 1; // 32 bit
        elf[5] = 1; // little endian
        elf[6] = 1;
        elf[16..18].copy_from_slice(&2u16.to_le_bytes()); // ET_EXEC
        elf[18..20].copy_from_slice(&44u16.to_le_bytes()); // EM_TRICORE
        elf[20..24].copy_from_slice(&1u32.to_le_bytes());
        elf[28..32].copy_from_slice(&52u32.to_le_bytes()); // e_phoff
        elf[40..42].copy_from_slice(&52u16.to_le_bytes()); // e_ehsize
        elf[42..44].copy_from_slice(&32u16.to_le_bytes()); // e_phentsize
        elf[44..46].copy_from_slice(&2u16.to_le_bytes()); // e_phnum
        let ph = |elf: &mut Vec<u8>, at: usize, offset: u32, vaddr: u32, paddr: u32, size: u32| {
            elf[at..at + 4].copy_from_slice(&1u32.to_le_bytes()); // PT_LOAD
            elf[at + 4..at + 8].copy_from_slice(&offset.to_le_bytes());
            elf[at + 8..at + 12].copy_from_slice(&vaddr.to_le_bytes());
            elf[at + 12..at + 16].copy_from_slice(&paddr.to_le_bytes());
            elf[at + 16..at + 20].copy_from_slice(&size.to_le_bytes());
            elf[at + 20..at + 24].copy_from_slice(&size.to_le_bytes());
        };
        ph(&mut elf, 52, 116, 0xD000_0000, 0x8000_0000, 4);
        ph(&mut elf, 84, 120, 0x7000_0000, 0x7000_0000, 0);
        assert_eq!(load_ranges(&elf), Ok(vec![(0x8000_0000, 4)]));
    }
}
