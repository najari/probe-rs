//! AURIX program flash (PFLASH) programming, for the TC27x and the TC36x, and data flash
//! (DFLASH) programming for the TC27x.
//!
//! A small loader (`loader/m25_flash_loader.c`) runs on CPU0 from PSPR and executes the flash
//! command sequences; the host drives it through a mailbox in DSPR. The command sequences are
//! the ones of the Infineon iLLD `IfxFlash` driver (Boost Software License 1.0), the same on
//! the TC2xx and the TC3xx; the loader is built once per flash controller ([`FlashDevice`]).
//!
//! The TC27x PFLASH has logical sectors of 16 KiB to 256 KiB, the TC36x PFLASH 16 KiB ones;
//! pages are 32 bytes on both. An erased TC27x page does not read as a value: it fails with an
//! uncorrectable ECC error. So the flash is read page by page, and a page that can not be read
//! is erased.
//!
//! The TC27x DFLASH (DF0, 0xAF000000, 384 KiB) has 48 logical sectors of 8 KiB and pages of
//! 8 bytes. The host handles it in 32-byte units like the PFLASH, and the loader writes each
//! unit as four pages. Erased DFLASH reads as 0 without an error, so a page of zeros counts as
//! erased: it is not written, and reads back as erased. The UCBs (0xAF100000) are not part of
//! the map and are never written.
//!
//! A sector the image touches is erased and the image's pages are written, the rest of a
//! partly written page is 0 (the erased value in the target description). Bytes the image does
//! not set stay erased, unless `--restore-unwritten` asked probe-rs to read them back first.
//! A sector that already holds exactly that content is not erased.
//!
//! `prepare_flash` saves the CPU0 registers and the RAM the loader uses, and halts all cores.
//! `finish_flash` restores them and resumes the cores that were running; so does a
//! `prepare_flash` that fails part way (probe-rs runs no `finish_flash` then). The loader
//! disables the CPU0 and the safety watchdog and puts their disable request back when it
//! exits. The application in flash has changed by then, so reset the device after flashing.

use std::{
    collections::{BTreeMap, BTreeSet},
    path::PathBuf,
    sync::Mutex,
    time::{Duration, Instant, SystemTime},
};

use base64::Engine as _;

use probe_rs::{
    CoreStatus, Error, MemoryInterface, RegisterId, Session,
    architecture::tricore::registers::{CSFR_A0, CSFR_ICR, CSFR_PC, TRICORE_CORE_REGISTERS},
    flashing::{DebugFlashSequence, FlashLayout, FlashPage},
};
use probe_rs_target::NvmRegion;

use crate::device_config;

/// The TC27x loader (TC2xx flash module), built by `loader/build.ps1`.
const LOADER_TC27X: &[u8] = include_bytes!("../loader/m25_flash_loader_tc27x.bin");
/// The TC3xx loader (DMU), built by `loader/build.ps1`.
const LOADER_TC3XX: &[u8] = include_bytes!("../loader/m25_flash_loader_tc3xx.bin");

const PSPR: u32 = 0x7010_0000;
const MAILBOX: u32 = 0x7000_F000;
const PAYLOAD: u32 = 0x7000_A000;
const PAYLOAD_SIZE: usize = 0x2000;
const LOADER_SP: u32 = 0x7000_E000;
const MAGIC: u32 = 0x4D32_354D;
/// The RAM the loader uses: payload, stack, data and mailbox in DSPR, code in PSPR.
const LOADER_RAM: [(u32, usize); 2] = [(0x7000_A000, 0x6000), (PSPR, 0x2000)];

/// The PFLASH program page size.
pub const PAGE: usize = 32;

const MB_MAGIC: u32 = 0;
const MB_CMD: u32 = 1;
const MB_ARG0: u32 = 2;
const MB_ARG1: u32 = 3;
const MB_ARG2: u32 = 4;
const MB_STATUS: u32 = 5;
const MB_DONE: u32 = 6;
const MB_PROGRESS: u32 = 7;

const CMD_ERASE: u32 = 1;
const CMD_PROGRAM: u32 = 2;
const CMD_EXIT: u32 = 3;

const ERASE_TIMEOUT: Duration = Duration::from_secs(30);
const PROGRAM_TIMEOUT: Duration = Duration::from_secs(10);

/// One logical PFLASH sector (non-cached address).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Sector {
    /// The first address.
    pub base: u32,
    /// The size in bytes.
    pub size: u32,
}

/// The 54 logical sectors of the TC27x PFLASH: per bank of 2 MiB, 8 x 16 KiB, 8 x 32 KiB,
/// 4 x 64 KiB, 3 x 128 KiB and 4 x 256 KiB.
pub fn tc27x_sectors() -> Vec<Sector> {
    const RUNS: [(u32, u32); 5] = [
        (0x4000, 8),
        (0x8000, 8),
        (0x1_0000, 4),
        (0x2_0000, 3),
        (0x4_0000, 4),
    ];
    let mut sectors = Vec::new();
    for bank in [0xA000_0000u32, 0xA020_0000] {
        let mut base = bank;
        for (size, count) in RUNS {
            for _ in 0..count {
                sectors.push(Sector { base, size });
                base += size;
            }
        }
    }
    sectors
}

/// The 256 logical sectors of 16 KiB of the TC36x PFLASH: PF0 and PF1 of 2 MiB each, at the
/// 3 MiB bank stride of the TC3xx.
pub fn tc36x_sectors() -> Vec<Sector> {
    [0xA000_0000u32, 0xA030_0000]
        .into_iter()
        .flat_map(|bank| {
            (0..128).map(move |i| Sector {
                base: bank + i * 0x4000,
                size: 0x4000,
            })
        })
        .collect()
}

/// The 48 logical sectors of 8 KiB of the TC27x DFLASH (DF0).
pub fn tc27x_dflash_sectors() -> Vec<Sector> {
    (0..48)
        .map(|i| Sector {
            base: DFLASH_BASE + i * 0x2000,
            size: 0x2000,
        })
        .collect()
}

/// The start of the DFLASH (DF0).
pub const DFLASH_BASE: u32 = 0xAF00_0000;
/// The start of the UCBs, which the flash programming never writes.
const UCB_BASE: u32 = 0xAF10_0000;

/// Whether `address` is in the data flash address range (below the UCBs).
pub fn is_dflash(address: u32) -> bool {
    (DFLASH_BASE..UCB_BASE).contains(&address)
}

/// A device that a flash loader of probe-rs programs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlashDevice {
    /// The TC27x (TC2xx flash module).
    Tc27x,
    /// The TC36x (TC3xx DMU).
    Tc36x,
}

impl FlashDevice {
    /// The device of a chip name of the AURIX targets.
    pub fn for_chip(name: &str) -> Option<Self> {
        let name = name.to_ascii_uppercase();
        if name.starts_with("TC27") {
            Some(Self::Tc27x)
        } else if name.starts_with("TC36") {
            Some(Self::Tc36x)
        } else {
            None
        }
    }

    /// The device of a JTAG ID; the step (bits 31:28) does not matter.
    pub fn for_jtag_id(jtag_id: u32) -> Option<Self> {
        match jtag_id & 0x0FFF_FFFF {
            0x001D_A083 => Some(Self::Tc27x),
            0x0020_9083 => Some(Self::Tc36x),
            _ => None,
        }
    }

    /// The device family name.
    pub fn name(self) -> &'static str {
        match self {
            Self::Tc27x => "TC27x",
            Self::Tc36x => "TC36x",
        }
    }

    /// The flash loader that the AURIX Flasher names for the device (`flaxFileName`).
    fn flax_file_name(self) -> &'static str {
        match self {
            Self::Tc27x => device_config::FLAX_TC2XX,
            Self::Tc36x => "flax_TC36x.hex",
        }
    }

    fn loader(self) -> &'static [u8] {
        match self {
            Self::Tc27x => LOADER_TC27X,
            Self::Tc36x => LOADER_TC3XX,
        }
    }

    /// `WDTCPU0_CON0` and `WDTS_CON0`, which hold ENDINIT; `CON1` (the disable request
    /// `DR`, bit 3) follows each.
    fn watchdogs(self) -> (u64, u64) {
        match self {
            Self::Tc27x => (0xF003_6100, 0xF003_60F0),
            Self::Tc36x => (0xF003_624C, 0xF003_62A8),
        }
    }

    /// The built-in PFLASH sector map.
    pub fn sectors(self) -> Vec<Sector> {
        match self {
            Self::Tc27x => tc27x_sectors(),
            Self::Tc36x => tc36x_sectors(),
        }
    }

    /// The built-in DFLASH sector map: empty where the loader does not program the DFLASH.
    pub fn dflash_sectors(self) -> Vec<Sector> {
        match self {
            Self::Tc27x => tc27x_dflash_sectors(),
            // The TC3xx loader waits for the PFLASH banks only.
            Self::Tc36x => Vec::new(),
        }
    }
}

/// The non-cached alias of a cached PFLASH address (`0x8...` to `0xA...`).
pub fn to_uncached(address: u64) -> u64 {
    if address >> 28 == 0x8 {
        address | 0x2000_0000
    } else {
        address
    }
}

fn sector_at(sectors: &[Sector], address: u32) -> Option<Sector> {
    sectors
        .iter()
        .copied()
        .find(|s| address >= s.base && address - s.base < s.size)
}

/// The content of one page: `None` when the page is erased.
pub type PageImage = Option<[u8; PAGE]>;

/// New data for one page: the bytes to write, `None` where the page keeps its content.
pub type PagePatch = [Option<u8>; PAGE];

/// Collects the bytes `layout` writes, per non-cached page address.
///
/// These are the image data, and the fill bytes of the pages the data touches. probe-rs fills
/// the rest of a sector too: with the old content for `--restore-unwritten`, else with the
/// erased value 0. A page with only fill bytes is written when one of them is not 0; an
/// all-zero fill page stays erased.
pub fn page_patches(layout: &FlashLayout) -> BTreeMap<u32, PagePatch> {
    let data = spans_to_patches(
        layout.pages(),
        layout.data_blocks().iter().map(|b| (b.address(), b.size())),
    );
    let fills = spans_to_patches(
        layout.pages(),
        layout.fills().iter().map(|f| (f.address(), f.size())),
    );
    let mut patches = data;
    for (address, fill) in fills {
        match patches.get_mut(&address) {
            Some(page) => {
                for (byte, fill) in page.iter_mut().zip(fill) {
                    if byte.is_none() {
                        *byte = fill;
                    }
                }
            }
            None if fill.iter().any(|b| b.is_some_and(|b| b != 0)) => {
                patches.insert(address, fill);
            }
            None => {}
        }
    }
    patches
}

fn spans_to_patches(
    pages: &[FlashPage],
    spans: impl Iterator<Item = (u64, u64)>,
) -> BTreeMap<u32, PagePatch> {
    let mut patches = BTreeMap::new();
    for (start, size) in spans {
        let end = start + size;
        let mut address = start;
        while address < end {
            let Some(page) = find_page(pages, address) else {
                break;
            };
            let page_end = page.address() + page.size() as u64;
            let n = (end.min(page_end) - address) as usize;
            let offset = (address - page.address()) as usize;
            for (i, byte) in page.data()[offset..offset + n].iter().enumerate() {
                let target = to_uncached(address + i as u64) as u32;
                let patch: &mut PagePatch = patches
                    .entry(target & !(PAGE as u32 - 1))
                    .or_insert([None; PAGE]);
                patch[target as usize % PAGE] = Some(*byte);
            }
            address += n as u64;
        }
    }
    patches
}

fn find_page(pages: &[FlashPage], address: u64) -> Option<&FlashPage> {
    let index = pages.partition_point(|p| p.address() + p.size() as u64 <= address);
    pages
        .get(index)
        .filter(|p| p.address() <= address && address < p.address() + p.size() as u64)
}

/// The pages of a sector once it is written: the patched pages, with 0 in the bytes a patch
/// does not set, and erased pages elsewhere. In the DFLASH a page of zeros is erased.
///
/// `before` is the sector as it was ([`read_pages`]). A page left erased where the PFLASH held
/// a page of zeros keeps the zeros: [`page_patches`] cannot tell the zeros of
/// `--restore-unwritten` from fill bytes, and erased PFLASH does not read as zeros (an ECC
/// error).
pub fn sector_image(
    sector: Sector,
    patches: &BTreeMap<u32, PagePatch>,
    before: &[PageImage],
) -> Vec<PageImage> {
    let dflash = is_dflash(sector.base);
    (0..sector.size as usize / PAGE)
        .map(|index| {
            let address = sector.base + (index * PAGE) as u32;
            patches
                .get(&address)
                .map(|patch| patch.map(|byte| byte.unwrap_or(0)))
                .or_else(|| {
                    before
                        .get(index)
                        .copied()
                        .flatten()
                        .filter(|page| *page == [0; PAGE])
                })
                .filter(|page| !(dflash && *page == [0; PAGE]))
        })
        .collect()
}

/// Reads `size` bytes of flash at `address` page by page: a page that can not be read (an
/// erased PFLASH page) or a DFLASH page of zeros is `None`. Large reads are tried first and
/// split only where they fail.
///
/// A read that fails could also be a broken connection, which would make the pages erased:
/// after a failed read of more than a page the CPU0 DSPR is read, and when that fails too, so
/// does this function.
pub fn read_pages(
    memory: &mut dyn MemoryInterface,
    address: u32,
    size: usize,
) -> Result<Vec<PageImage>, Error> {
    let mut pages = vec![None; size / PAGE];
    fn walk(
        memory: &mut dyn MemoryInterface,
        base: u32,
        address: u32,
        size: usize,
        step: usize,
        top: bool,
        pages: &mut [PageImage],
    ) -> Result<(), Error> {
        let mut offset = 0;
        while offset < size {
            let n = step.min(size - offset);
            let at = address + offset as u32;
            let mut buffer = vec![0; n];
            if memory.read(at as u64, &mut buffer).is_ok() {
                for (i, chunk) in buffer.as_chunks::<PAGE>().0.iter().enumerate() {
                    let index = (at - base) as usize / PAGE + i;
                    pages[index] = Some(*chunk);
                }
            } else {
                if n > PAGE || top {
                    memory.read_word_32(MAILBOX as u64).map_err(|error| {
                        other(format!(
                            "reading the flash at {at:#010x} failed, and so did a read of RAM \
                             ({error}): the connection to the device is broken"
                        ))
                    })?;
                }
                if n > PAGE {
                    let step = if n > 512 { 512 } else { PAGE };
                    walk(memory, base, at, n, step, false, pages)?;
                }
            }
            offset += n;
        }
        Ok(())
    }
    walk(memory, address, address, size, 0x1000, true, &mut pages)?;
    if is_dflash(address) {
        for page in &mut pages {
            if *page == Some([0; PAGE]) {
                *page = None;
            }
        }
    }
    Ok(pages)
}

/// What the loader changes, saved by [`AurixFlash::start_loader`] as it goes (a part, when it
/// fails part way).
#[derive(Debug, Default)]
struct SavedState {
    registers: Vec<(RegisterId, u32)>,
    ram: Vec<(u32, Vec<u8>)>,
    /// Whether each core ran; a core that ran was halted.
    running: Vec<bool>,
    /// `WDTCPU0_CON1` and `WDTS_CON1` before the loader disabled the watchdogs.
    watchdogs: Option<(u32, u32)>,
    /// Whether CPU0 was run with the loader.
    started: bool,
}

/// `CON1.DR` of a watchdog: the disable request.
const WDT_DR: u32 = 1 << 3;

/// Keeps the first error of several steps that are all tried.
fn keep_first(failed: &mut Option<Error>, result: Result<(), Error>) {
    if let Err(error) = result {
        failed.get_or_insert(error);
    }
}

/// SCU_CHIPID, which tells the device step and variant apart.
const SCU_CHIPID: u64 = 0xF003_6140;

/// The flash sectors of the connected device.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FlashMap {
    /// The device.
    pub device: FlashDevice,
    /// Where the map comes from: an AURIX Flasher device file, or the built-in map.
    pub source: String,
    /// The logical PFLASH sectors.
    pub sectors: Vec<Sector>,
    /// The logical DFLASH sectors; empty where the loader does not program the DFLASH.
    pub dflash: Vec<Sector>,
    /// The bases of the sectors that must not be written.
    pub restricted: BTreeSet<u32>,
}

impl FlashMap {
    /// The built-in map of `device`.
    pub fn builtin(device: FlashDevice) -> Self {
        Self {
            device,
            source: format!("the built-in {} map", device.name()),
            sectors: device.sectors(),
            dflash: device.dflash_sectors(),
            restricted: BTreeSet::new(),
        }
    }

    /// The map for a device with these IDs: from its AURIX Flasher device file in `dirs` if
    /// there is one, else the built-in map. Fails for a device no loader can program.
    pub fn for_device(
        dirs: &[std::path::PathBuf],
        jtag_id: u32,
        chip_id: Option<u32>,
    ) -> Result<Self, Error> {
        let config = device_config::find_in(dirs, jtag_id, chip_id);
        let Some(device) = FlashDevice::for_jtag_id(jtag_id) else {
            return Err(other(match &config {
                Some(config) => format!(
                    "{} uses the flash controller of {}; probe-rs has flash loaders for the \
                     TC27x and the TC36x only",
                    config.device_name, config.flax_file_name
                ),
                None => format!(
                    "probe-rs has flash loaders for the TC27x and the TC36x only, not for JTAG \
                     ID {jtag_id:#010x}"
                ),
            }));
        };
        let Some(config) = config else {
            return Ok(Self::builtin(device));
        };
        let name = &config.device_name;
        if config.flax_file_name != device.flax_file_name() {
            return Err(other(format!(
                "{name} uses the flash controller of {}, not the one of the {} ({})",
                config.flax_file_name,
                device.name(),
                device.flax_file_name()
            )));
        }
        let pflash = config
            .program_flash()
            .filter(|m| !m.sectors.is_empty())
            .ok_or_else(|| {
                other(format!(
                    "{} has no ProgramFlash sectors",
                    config.file.display()
                ))
            })?;
        let sectors = pflash
            .sectors
            .iter()
            .map(|s| Sector {
                base: s.base,
                size: s.size,
            })
            .collect::<Vec<_>>();
        if sectors != device.sectors() {
            tracing::warn!(
                "The PFLASH map of {name} differs from the built-in {} map; using {}",
                device.name(),
                config.file.display()
            );
        }
        // The DFLASH sectors of the file, for a device whose DFLASH the loader programs. Only
        // sectors in the DFLASH range count (never a UCB); a file without them leaves the
        // built-in map.
        let builtin_dflash = device.dflash_sectors();
        let dflash_config = config
            .data_flash()
            .filter(|_| !builtin_dflash.is_empty())
            .map(|m| {
                m.sectors
                    .iter()
                    .filter(|s| s.size > 0 && is_dflash(s.base) && is_dflash(s.base + s.size - 1))
                    .copied()
                    .collect::<Vec<_>>()
            })
            .filter(|sectors| !sectors.is_empty());
        let dflash = match &dflash_config {
            Some(config_sectors) => {
                let dflash = config_sectors
                    .iter()
                    .map(|s| Sector {
                        base: s.base,
                        size: s.size,
                    })
                    .collect::<Vec<_>>();
                if dflash != builtin_dflash {
                    tracing::warn!(
                        "The DFLASH map of {name} differs from the built-in {} map; using {}",
                        device.name(),
                        config.file.display()
                    );
                }
                dflash
            }
            None => builtin_dflash,
        };
        Ok(Self {
            device,
            source: format!("{name} ({})", config.file.display()),
            restricted: pflash
                .sectors
                .iter()
                .chain(dflash_config.iter().flatten())
                .filter(|s| s.restricted)
                .map(|s| s.base)
                .collect(),
            sectors,
            dflash,
        })
    }

    fn sector(&self, address: u64) -> Result<Sector, Error> {
        let uncached = to_uncached(address);
        let sector = u32::try_from(uncached)
            .ok()
            .and_then(|a| sector_at(&self.sectors, a).or_else(|| sector_at(&self.dflash, a)))
            .ok_or_else(|| {
                other(if u32::try_from(uncached).is_ok_and(is_dflash) && self.dflash.is_empty() {
                    format!(
                        "{address:#010x} is in the DFLASH, which probe-rs programs on the TC27x \
                         only"
                    )
                } else {
                    format!("{address:#010x} is not in the PFLASH or the DFLASH")
                })
            })?;
        if self.restricted.contains(&sector.base) {
            return Err(other(format!(
                "sector {:#010x} is restricted in {}",
                sector.base, self.source
            )));
        }
        Ok(sector)
    }
}

/// The environment variable that turns sector backups on: the directory to keep them in.
///
/// Each flash download then saves every sector it erases, as it was before, in
/// `<dir>/<UTC time of the download>/sector_<base>.json`: `{ base, size, blank, b64 }`, with
/// `blank` 1 for the erased pages (the format the `pflash_check` example compares with).
pub const BACKUP_ENV: &str = "PROBE_RS_TRICORE_FLASH_BACKUP";

/// Programs the PFLASH of a TC27x or a TC36x through the RAM loader.
#[derive(Debug)]
pub struct AurixFlash {
    /// The device of the target description.
    device: FlashDevice,
    saved: Mutex<Option<SavedState>>,
    /// The pages written in this session, which must read back as data.
    programmed: Mutex<BTreeSet<u32>>,
    /// The sector map of the connected device, found when the loader starts.
    map: Mutex<Option<FlashMap>>,
    /// Where the sectors of this download are backed up ([`BACKUP_ENV`]).
    backup: Mutex<Option<PathBuf>>,
}

/// The JSON backup of a sector: its pages as read before the erase (`None`: erased).
pub fn sector_backup(sector: Sector, pages: &[PageImage]) -> String {
    let blank = pages
        .iter()
        .map(|page| u8::from(page.is_none()))
        .collect::<Vec<_>>();
    let data = pages
        .iter()
        .flat_map(|page| page.unwrap_or([0; PAGE]))
        .collect::<Vec<_>>();
    serde_json::json!({
        "base": sector.base,
        "size": sector.size,
        "blank": blank,
        "b64": base64::engine::general_purpose::STANDARD.encode(data),
    })
    .to_string()
}

/// `YYYY-MM-DD_HHMMSSZ` of a time (UTC), for the backup directory of a download.
pub fn utc_stamp(time: SystemTime) -> String {
    let seconds = time
        .duration_since(SystemTime::UNIX_EPOCH)
        .map_or(0, |d| d.as_secs());
    let (days, rest) = (seconds / 86_400, seconds % 86_400);
    // The civil date of a day number (H. Hinnant, days from civil).
    let z = days as i64 + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1_460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = doy - (153 * mp + 2) / 5 + 1;
    let month = if mp < 10 { mp + 3 } else { mp - 9 };
    let year = yoe + era * 400 + i64::from(month <= 2);
    format!(
        "{year:04}-{month:02}-{day:02}_{:02}{:02}{:02}Z",
        rest / 3600,
        rest / 60 % 60,
        rest % 60
    )
}

fn other(message: impl Into<String>) -> Error {
    Error::Other(message.into())
}

impl AurixFlash {
    /// The flash programming of `device`.
    pub fn new(device: FlashDevice) -> Self {
        Self {
            device,
            saved: Mutex::default(),
            programmed: Mutex::default(),
            map: Mutex::default(),
            backup: Mutex::default(),
        }
    }

    /// Saves `pages`, the content of `sector` before its erase, when backups are on
    /// ([`BACKUP_ENV`]). A backup that cannot be written stops the download before the erase.
    fn save_backup(&self, sector: Sector, pages: &[PageImage]) -> Result<(), Error> {
        let Some(dir) = self
            .backup
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clone()
        else {
            return Ok(());
        };
        let path = dir.join(format!("sector_{:08x}.json", sector.base));
        std::fs::create_dir_all(&dir)
            .and_then(|()| std::fs::write(&path, sector_backup(sector, pages)))
            .map_err(|error| {
                other(format!(
                    "the backup of sector {:#010x} to {} failed ({error}); the sector was not \
                     erased",
                    sector.base,
                    path.display()
                ))
            })?;
        tracing::info!(
            "Sector {:#010x} backed up to {}",
            sector.base,
            path.display()
        );
        Ok(())
    }

    /// Checks that the loader can program the device and takes its sector map.
    fn check_device(&self, session: &mut Session) -> Result<(), Error> {
        let jtag_id = session.get_tricore_bus()?.jtag_id()?;
        let chip_id = session.core(0)?.read_word_32(SCU_CHIPID).ok();
        let map = FlashMap::for_device(&device_config::config_dirs(), jtag_id, chip_id)?;
        if map.device != self.device {
            return Err(other(format!(
                "the target is a {}, but the connected device (JTAG ID {jtag_id:#010x}) is a {}",
                self.device.name(),
                map.device.name()
            )));
        }
        tracing::info!(
            "Flash map: {} ({} PFLASH and {} DFLASH sectors; JTAG ID {jtag_id:#010x}, chip ID {})",
            map.source,
            map.sectors.len(),
            map.dflash.len(),
            chip_id.map_or("?".into(), |c| format!("{c:#010x}"))
        );
        *self.map.lock().unwrap_or_else(|e| e.into_inner()) = Some(map);
        Ok(())
    }

    /// The sector map of the device (the built-in one before the loader started).
    fn flash_map(&self) -> FlashMap {
        self.map
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clone()
            .unwrap_or_else(|| FlashMap::builtin(self.device))
    }

    fn mailbox_read(session: &mut Session, index: u32) -> Result<u32, Error> {
        session.core(0)?.read_word_32((MAILBOX + 4 * index) as u64)
    }

    fn mailbox_write(session: &mut Session, index: u32, value: u32) -> Result<(), Error> {
        session
            .core(0)?
            .write_word_32((MAILBOX + 4 * index) as u64, value)
    }

    fn loader_running(session: &mut Session) -> Result<bool, Error> {
        Ok(Self::mailbox_read(session, MB_MAGIC)? == MAGIC)
    }

    /// Runs one loader command and returns its status (0 = ok).
    fn command(
        session: &mut Session,
        cmd: u32,
        args: [u32; 3],
        timeout: Duration,
    ) -> Result<u32, Error> {
        if !Self::loader_running(session)? {
            return Err(other("the flash loader is not running"));
        }
        Self::mailbox_write(session, MB_DONE, 0)?;
        Self::mailbox_write(session, MB_PROGRESS, 0)?;
        Self::mailbox_write(session, MB_ARG0, args[0])?;
        Self::mailbox_write(session, MB_ARG1, args[1])?;
        Self::mailbox_write(session, MB_ARG2, args[2])?;
        // The loader takes the command and clears it at once: no read back.
        Self::mailbox_write(session, MB_CMD, cmd)?;

        let start = Instant::now();
        while Self::mailbox_read(session, MB_DONE)? != 1 {
            if start.elapsed() > timeout {
                return Err(other(format!("flash loader command {cmd} timed out")));
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        let status = Self::mailbox_read(session, MB_STATUS)?;
        let progress = Self::mailbox_read(session, MB_PROGRESS)?;
        tracing::debug!("loader command {cmd} {args:#x?}: status {status:#x}, progress {progress}");
        Ok(status)
    }

    fn erase(session: &mut Session, sector: Sector) -> Result<(), Error> {
        tracing::info!(
            "Erasing sector {:#010x} ({} KiB)",
            sector.base,
            sector.size / 1024
        );
        let status = Self::command(session, CMD_ERASE, [sector.base, 1, 0], ERASE_TIMEOUT)?;
        if status != 0 {
            return Err(other(format!(
                "erasing {:#010x} failed (status {status:#x})",
                sector.base
            )));
        }
        Ok(())
    }

    /// Writes the pages that hold data, in runs of consecutive pages that fit the payload buffer.
    fn program_pages(
        &self,
        session: &mut Session,
        base: u32,
        pages: &[PageImage],
    ) -> Result<(), Error> {
        let mut index = 0;
        while index < pages.len() {
            if pages[index].is_none() {
                index += 1;
                continue;
            }
            let start = index;
            let mut data = Vec::new();
            while index < pages.len() && data.len() < PAYLOAD_SIZE {
                let Some(page) = pages[index] else { break };
                data.extend_from_slice(&page);
                index += 1;
            }
            let address = base + (start * PAGE) as u32;
            session.core(0)?.write(PAYLOAD as u64, &data)?;
            let status = Self::command(
                session,
                CMD_PROGRAM,
                [address, data.len() as u32, PAYLOAD],
                PROGRAM_TIMEOUT,
            )?;
            if status != 0 {
                return Err(other(format!(
                    "programming {address:#010x} failed (status {status:#x})"
                )));
            }
            let mut programmed = self.programmed.lock().unwrap_or_else(|e| e.into_inner());
            for page in 0..data.len() / PAGE {
                programmed.insert(address + (page * PAGE) as u32);
            }
        }
        Ok(())
    }

    /// Halts the cores and starts the loader on CPU0. What it changes goes into `saved` before
    /// it is changed, so that [`Self::stop_loader`] puts back what was done when a step fails.
    fn start_loader(&self, session: &mut Session, saved: &mut SavedState) -> Result<(), Error> {
        self.check_device(session)?;

        for id in 0..session.list_cores().len() {
            let mut core = session.core(id)?;
            // A core the application has not started (boot halt) sleeps: the halt leaves it as
            // it is, and it is not run afterwards.
            saved.running.push(core.status()? == CoreStatus::Running);
            // The application may reset the device when a core stops answering.
            core.halt(Duration::from_millis(500))?;
        }

        let mut core = session.core(0)?;
        let mut registers = Vec::new();
        for register in TRICORE_CORE_REGISTERS.all_registers() {
            registers.push((register.id, core.read_core_reg::<u32>(register.id)?));
        }
        let mut ram = Vec::new();
        for (address, size) in LOADER_RAM {
            let mut data = vec![0; size];
            core.read(address as u64, &mut data)?;
            ram.push((address, data));
        }
        let (wdtcpu0_con0, wdts_con0) = self.device.watchdogs();
        let watchdogs = (
            core.read_word_32(wdtcpu0_con0 + 4)?,
            core.read_word_32(wdts_con0 + 4)?,
        );
        saved.registers = registers;
        saved.ram = ram;
        saved.watchdogs = Some(watchdogs);

        let loader = self.device.loader();
        tracing::info!(
            "Starting the {} flash loader ({} bytes)",
            self.device.name(),
            loader.len()
        );
        core.write(PSPR as u64, loader)?;
        core.write_word_32((MAILBOX + 4 * MB_MAGIC) as u64, 0)?;
        core.write_core_reg(RegisterId(CSFR_PC), PSPR)?;
        core.write_core_reg(RegisterId(CSFR_A0 + 4 * 10), LOADER_SP)?;
        // No application interrupt may run while a flash bank is busy.
        let icr: u32 = core.read_core_reg(RegisterId(CSFR_ICR))?;
        core.write_core_reg(RegisterId(CSFR_ICR), icr & !(1 << 15))?;
        saved.started = true;
        core.run()?;
        drop(core);

        let start = Instant::now();
        while !Self::loader_running(session)? {
            if start.elapsed() > Duration::from_secs(3) {
                return Err(other("the flash loader did not start"));
            }
            std::thread::sleep(Duration::from_millis(20));
        }
        Ok(())
    }

    /// Stops the loader and puts back what [`Self::start_loader`] saved. Every step is tried,
    /// also after one failed: the first error is returned.
    fn stop_loader(&self, session: &mut Session) -> Result<(), Error> {
        let Some(saved) = self.saved.lock().unwrap_or_else(|e| e.into_inner()).take() else {
            return Ok(());
        };
        let mut failed = None;
        if !saved.started {
            // Nothing ran: the cores halted so far run again, the rest was not changed.
            for (id, running) in saved.running.iter().enumerate() {
                if *running {
                    keep_first(&mut failed, session.core(id).and_then(|mut core| core.run()));
                }
            }
            return failed.map_or(Ok(()), Err);
        }

        match Self::loader_running(session) {
            Ok(true) => keep_first(
                &mut failed,
                Self::command(session, CMD_EXIT, [0; 3], Duration::from_secs(2)).map(drop),
            ),
            Ok(false) => {}
            Err(error) => keep_first(&mut failed, Err(error)),
        }
        // The loader must have set ENDINIT again, and put back the disable requests.
        let mut watchdogs = None;
        match session.core(0) {
            Ok(mut core) => {
                keep_first(&mut failed, core.halt(Duration::from_millis(500)).map(drop));
                let (wdtcpu0_con0, wdts_con0) = self.device.watchdogs();
                let words = [wdtcpu0_con0, wdtcpu0_con0 + 4, wdts_con0, wdts_con0 + 4]
                    .map(|address| core.read_word_32(address));
                match words {
                    [Ok(con0), Ok(con1), Ok(s_con0), Ok(s_con1)] => {
                        watchdogs = Some((con0, con1, s_con0, s_con1));
                    }
                    words => {
                        for word in words {
                            keep_first(&mut failed, word.map(drop));
                        }
                    }
                }
                for (address, data) in &saved.ram {
                    keep_first(&mut failed, core.write(*address as u64, data));
                }
                for (id, value) in &saved.registers {
                    keep_first(&mut failed, core.write_core_reg(*id, *value));
                }
            }
            Err(error) => keep_first(&mut failed, Err(error)),
        }

        for (id, running) in saved.running.iter().enumerate() {
            if *running {
                keep_first(&mut failed, session.core(id).and_then(|mut core| core.run()));
            }
        }

        if let Some(error) = failed {
            return Err(error);
        }
        if let Some((cpu0, cpu0_con1, safety, safety_con1)) = watchdogs {
            if cpu0 & 1 == 0 || safety & 1 == 0 {
                return Err(other(format!(
                    "ENDINIT is still cleared after flashing (WDTCPU0_CON0 {cpu0:#010x}, \
                     WDTS_CON0 {safety:#010x}): reset the device"
                )));
            }
            if let Some((before, safety_before)) = saved.watchdogs
                && ((cpu0_con1 ^ before) | (safety_con1 ^ safety_before)) & WDT_DR != 0
            {
                return Err(other(format!(
                    "the watchdogs are not enabled as before flashing (WDTCPU0_CON1 \
                     {cpu0_con1:#010x}, was {before:#010x}; WDTS_CON1 {safety_con1:#010x}, \
                     was {safety_before:#010x}): reset the device"
                )));
            }
        }
        tracing::info!("Flash loader stopped; reset the device to run the new image");
        Ok(())
    }
}

impl DebugFlashSequence for AurixFlash {
    fn prepare_flash(&self, session: &mut Session) -> Result<(), Error> {
        self.programmed
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clear();
        *self.backup.lock().unwrap_or_else(|e| e.into_inner()) = std::env::var_os(BACKUP_ENV)
            .filter(|dir| !dir.is_empty())
            .map(|dir| PathBuf::from(dir).join(utc_stamp(SystemTime::now())));
        let mut saved = SavedState::default();
        let started = self.start_loader(session, &mut saved);
        *self.saved.lock().unwrap_or_else(|e| e.into_inner()) = Some(saved);
        if let Err(error) = started {
            // probe-rs runs no `finish_flash` after a failed `prepare_flash`.
            if let Err(restore) = self.stop_loader(session) {
                tracing::warn!("Putting back the state from before the flash loader: {restore}");
            }
            return Err(error);
        }
        Ok(())
    }

    fn finish_flash(&self, session: &mut Session) -> Result<(), Error> {
        self.stop_loader(session)
    }

    fn erase_all(&self, _session: &mut Session) -> Result<(), Error> {
        Err(Error::NotImplemented("chip erase of the AURIX PFLASH"))
    }

    fn supports_chip_erase(&self) -> bool {
        false
    }

    fn erase_sector(&self, session: &mut Session, address: u64) -> Result<(), Error> {
        let sector = self.flash_map().sector(address)?;
        if sector.base as u64 != to_uncached(address) {
            return Err(other(format!(
                "{address:#010x} is not the start of a sector"
            )));
        }
        Self::erase(session, sector)
    }

    fn program(&self, session: &mut Session, address: u64, data: &[u8]) -> Result<(), Error> {
        // Only used when `program_image` is not: write the data as given.
        let base = to_uncached(address) as u32;
        self.flash_map().sector(address)?;
        if !base.is_multiple_of(PAGE as u32) {
            return Err(other(format!("{address:#010x} is not page aligned")));
        }
        let pages = data
            .chunks(PAGE)
            .map(|chunk| {
                let mut page = [0; PAGE];
                page[..chunk.len()].copy_from_slice(chunk);
                Some(page)
            })
            .collect::<Vec<_>>();
        self.program_pages(session, base, &pages)
    }

    fn program_image(
        &self,
        session: &mut Session,
        regions: &[(&NvmRegion, &FlashLayout)],
    ) -> Result<bool, Error> {
        let mut patches = BTreeMap::new();
        for (_, layout) in regions {
            patches.extend(page_patches(layout));
        }

        let map = self.flash_map();
        let mut touched = BTreeMap::new();
        for page in patches.keys() {
            let sector = map.sector(*page as u64)?;
            touched.insert(sector.base, sector);
        }

        for sector in touched.into_values() {
            let backup = {
                let mut core = session.core(0)?;
                read_pages(&mut core, sector.base, sector.size as usize)?
            };
            let image = sector_image(sector, &patches, &backup);
            if image == backup {
                tracing::info!("Sector {:#010x} already holds the image", sector.base);
                continue;
            }
            self.save_backup(sector, &backup)?;
            Self::erase(session, sector)?;
            self.program_pages(session, sector.base, &image)?;
            tracing::info!(
                "Sector {:#010x}: {} page(s) written",
                sector.base,
                image.iter().filter(|p| p.is_some()).count()
            );
        }
        Ok(true)
    }

    fn verify(&self, session: &mut Session, address: u64, data: &[u8]) -> Result<bool, Error> {
        let base = to_uncached(address) as u32;
        let pages = {
            let mut core = session.core(0)?;
            read_pages(&mut core, base, data.len() / PAGE * PAGE)?
        };
        let programmed = self.programmed.lock().unwrap_or_else(|e| e.into_inner());
        for (index, (page, expected)) in pages.iter().zip(data.as_chunks::<PAGE>().0).enumerate() {
            let address = base + (index * PAGE) as u32;
            let ok = match page {
                Some(actual) => actual == expected,
                // An erased page stands for bytes the image does not set; it must not be one
                // that was just written.
                None => !programmed.contains(&address) && expected.iter().all(|b| *b == 0),
            };
            if !ok {
                tracing::error!("Verify failed at {address:#010x}");
                return Ok(false);
            }
        }
        Ok(true)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn backup_directories_are_named_by_the_utc_time() {
        let at = |seconds| SystemTime::UNIX_EPOCH + Duration::from_secs(seconds);
        assert_eq!(utc_stamp(at(0)), "1970-01-01_000000Z");
        assert_eq!(utc_stamp(at(951_868_800)), "2000-03-01_000000Z");
        assert_eq!(utc_stamp(at(1_709_251_199)), "2024-02-29_235959Z");
        assert_eq!(utc_stamp(at(1_792_755_296)), "2026-10-23_113456Z");
    }

    #[test]
    fn a_sector_backup_keeps_data_and_erased_pages() {
        let sector = Sector {
            base: 0xA021_C000,
            size: 3 * PAGE as u32,
        };
        let pages = [Some([0x11; PAGE]), None, Some([0x22; PAGE])];
        let json: serde_json::Value = serde_json::from_str(&sector_backup(sector, &pages)).unwrap();
        assert_eq!(json["base"], 0xA021_C000u32);
        assert_eq!(json["size"], 96);
        assert_eq!(json["blank"], serde_json::json!([0, 1, 0]));
        let data = base64::engine::general_purpose::STANDARD
            .decode(json["b64"].as_str().unwrap())
            .unwrap();
        assert_eq!(data.len(), 96);
        assert!(data[..32].iter().all(|b| *b == 0x11) && data[64..].iter().all(|b| *b == 0x22));
    }

    #[test]
    fn the_sector_map_has_54_sectors_and_covers_4_mib() {
        let sectors = tc27x_sectors();
        assert_eq!(sectors.len(), 54);
        assert_eq!(
            sectors[0],
            Sector {
                base: 0xA000_0000,
                size: 0x4000
            }
        );
        let last = sectors.last().unwrap();
        assert_eq!(last.base + last.size, 0xA040_0000);
        for pair in sectors.windows(2) {
            assert_eq!(pair[0].base + pair[0].size, pair[1].base);
        }
        // The test sector of the demo.
        assert_eq!(
            sector_at(&sectors, 0xA021_C0B0),
            Some(Sector {
                base: 0xA021_C000,
                size: 0x4000
            })
        );
        assert_eq!(sector_at(&sectors, 0xA040_0000), None);
    }

    #[test]
    fn cached_addresses_map_to_the_uncached_alias() {
        assert_eq!(to_uncached(0x8021_C000), 0xA021_C000);
        assert_eq!(to_uncached(0xA021_C000), 0xA021_C000);
        assert_eq!(to_uncached(0x7000_0000), 0x7000_0000);
    }

    #[test]
    fn only_patched_pages_are_written() {
        let sector = Sector {
            base: 0xA021_C000,
            size: 0x4000,
        };
        let mut patches = BTreeMap::new();
        let mut patch = [None; PAGE];
        patch[1] = Some(0xAB);
        patches.insert(sector.base + 64, patch);

        let image = sector_image(sector, &patches, &[]);
        assert_eq!(image.len(), 512);
        let mut expected = [0; PAGE];
        expected[1] = 0xAB;
        assert_eq!(image[2], Some(expected));
        assert_eq!(image.iter().filter(|p| p.is_some()).count(), 1);
    }

    /// A directory with one AURIX Flasher style device file (made-up content).
    fn device_dir(
        name: &str,
        flax: &str,
        jtag: &str,
        sectors: &[(u32, u32, bool)],
    ) -> std::path::PathBuf {
        let dir =
            std::env::temp_dir().join(format!("probe-rs-flash-map-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        let sectors = sectors
            .iter()
            .map(|(base, size, restricted)| {
                format!(
                    r#"{{ "sectorStartAddress": "{base:#010X}", "sectorSize": "{size:#X}", "restricted": {restricted} }}"#
                )
            })
            .collect::<Vec<_>>()
            .join(",");
        let json = format!(
            r#"{{ "deviceName": "{name}", "flaxFileName": "{flax}",
                 "registers": [ {{ "registerName": "JTAGID", "registerValue": ["{jtag}"] }},
                                {{ "registerName": "ChipID", "registerValue": ["0x47427060"] }} ],
                 "memory": [ {{ "memoryName": "ProgramFlash", "memoryStartAddress": "0xA0000000",
                                "memoryEndAddress": "0xA03FFFFF", "logicalSectors": [{sectors}] }} ] }}"#
        );
        std::fs::write(dir.join(format!("{name}.json")), json).unwrap();
        dir
    }

    fn builtin_runs() -> Vec<(u32, u32, bool)> {
        tc27x_sectors()
            .iter()
            .map(|s| (s.base, s.size, false))
            .collect()
    }

    #[test]
    fn the_map_comes_from_the_matching_device_file() {
        let dir = device_dir(
            "TC27x_test",
            "flax_TC2xx.hex",
            "0x401DA083",
            &builtin_runs(),
        );
        let map = FlashMap::for_device(std::slice::from_ref(&dir), 0x401D_A083, Some(0x4742_7060))
            .unwrap();
        assert!(map.source.starts_with("TC27x_test"), "{}", map.source);
        assert_eq!(map.device, FlashDevice::Tc27x);
        assert_eq!(map.sectors, tc27x_sectors());
        assert_eq!(map.sector(0x8021_C0B0).unwrap().base, 0xA021_C000);
        std::fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn restricted_sectors_are_refused() {
        let mut runs = builtin_runs();
        runs[0].2 = true;
        let dir = device_dir("TC27x_restricted", "flax_TC2xx.hex", "0x401DA083", &runs);
        let map = FlashMap::for_device(std::slice::from_ref(&dir), 0x401D_A083, None).unwrap();
        let error = map.sector(0xA000_0100).unwrap_err().to_string();
        assert!(error.contains("restricted"), "{error}");
        assert!(map.sector(0xA000_4000).is_ok());
        std::fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn devices_without_a_loader_are_refused() {
        let tc3 = device_dir(
            "TC37x_test",
            "flax_TC37x.hex",
            "0x10207083",
            &[(0xA000_0000, 0x4000, false)],
        );
        let error = FlashMap::for_device(std::slice::from_ref(&tc3), 0x1020_7083, None)
            .unwrap_err()
            .to_string();
        assert!(error.contains("flax_TC37x.hex"), "{error}");
        std::fs::remove_dir_all(tc3).unwrap();

        let tc26 = device_dir(
            "TC26x_test",
            "flax_TC2xx.hex",
            "0x201E8083",
            &[(0xA000_0000, 0x4000, false)],
        );
        let error = FlashMap::for_device(std::slice::from_ref(&tc26), 0x201E_8083, None)
            .unwrap_err()
            .to_string();
        assert!(error.contains("TC36x only"), "{error}");
        std::fs::remove_dir_all(tc26).unwrap();

        // A TC36x with a device file of another flash controller.
        let wrong = device_dir(
            "TC36x_wrong",
            "flax_TC2xx.hex",
            "0x10209083",
            &[(0xA000_0000, 0x4000, false)],
        );
        let error = FlashMap::for_device(std::slice::from_ref(&wrong), 0x1020_9083, None)
            .unwrap_err()
            .to_string();
        assert!(error.contains("flax_TC36x.hex"), "{error}");
        std::fs::remove_dir_all(wrong).unwrap();
    }

    #[test]
    fn without_a_device_file_the_builtin_map_is_used() {
        assert_eq!(
            FlashMap::for_device(&[], 0x401D_A083, None).unwrap(),
            FlashMap::builtin(FlashDevice::Tc27x)
        );
        assert_eq!(
            FlashMap::for_device(&[], 0x1020_9083, None).unwrap(),
            FlashMap::builtin(FlashDevice::Tc36x)
        );
        // TC37x: no loader.
        assert!(FlashMap::for_device(&[], 0x1020_7083, None).is_err());
    }

    #[test]
    fn the_tc36x_map_has_two_banks_of_16_kib_sectors() {
        let sectors = tc36x_sectors();
        assert_eq!(sectors.len(), 256);
        assert!(sectors.iter().all(|s| s.size == 0x4000));
        assert_eq!(sectors[127].base + sectors[127].size, 0xA020_0000);
        assert_eq!(sectors[128].base, 0xA030_0000);
        assert_eq!(sectors.last().unwrap().base + 0x4000, 0xA050_0000);
        // The gap between the banks is not flash.
        assert_eq!(sector_at(&sectors, 0xA021_C000), None);
        assert_eq!(
            sector_at(&sectors, 0xA04F_C010),
            Some(Sector {
                base: 0xA04F_C000,
                size: 0x4000
            })
        );
    }

    #[test]
    fn the_tc27x_dflash_has_48_sectors_of_8_kib_and_no_ucb() {
        let sectors = tc27x_dflash_sectors();
        assert_eq!(sectors.len(), 48);
        assert_eq!(sectors[0].base, 0xAF00_0000);
        assert_eq!(sectors.last().unwrap().base + 0x2000, 0xAF06_0000);
        let map = FlashMap::builtin(FlashDevice::Tc27x);
        assert_eq!(
            map.sector(0xAF00_2010).unwrap(),
            Sector {
                base: 0xAF00_2000,
                size: 0x2000
            }
        );
        // PFLASH sectors are still found, the UCBs and the end of DF0 are not flash.
        assert_eq!(map.sector(0x8021_C000).unwrap().base, 0xA021_C000);
        for address in [0xAF06_0000u64, 0xAF10_0000, 0xAF10_0400] {
            let error = map.sector(address).unwrap_err().to_string();
            assert!(error.contains("not in the PFLASH or the DFLASH"), "{error}");
        }
        assert!(is_dflash(0xAF05_FFFF) && !is_dflash(0xAF10_0000) && !is_dflash(0xA000_0000));
    }

    #[test]
    fn the_tc36x_dflash_is_refused() {
        let map = FlashMap::builtin(FlashDevice::Tc36x);
        assert!(map.dflash.is_empty());
        let error = map.sector(0xAF00_0000).unwrap_err().to_string();
        assert!(error.contains("on the TC27x only"), "{error}");
    }

    #[test]
    fn a_page_of_zeros_that_was_in_the_pflash_stays() {
        let sector = Sector {
            base: 0xA021_C000,
            size: 0x4000,
        };
        // Programmed zeros at page 0 (an application's data), erased at page 1, data at page 2;
        // the image sets page 3 only.
        let mut before = vec![None; 512];
        before[0] = Some([0; PAGE]);
        before[2] = Some([0x11; PAGE]);
        let mut patches = BTreeMap::new();
        patches.insert(sector.base + 3 * PAGE as u32, [Some(0x22); PAGE]);
        let image = sector_image(sector, &patches, &before);
        assert_eq!(image[0], Some([0; PAGE]));
        assert_eq!(image[1], None);
        assert_eq!(image[2], None);
        assert_eq!(image[3], Some([0x22; PAGE]));
    }

    /// Flash in which the pages of `erased` fail to read, and everything fails when the
    /// connection is `broken`.
    #[derive(Default)]
    struct FakeFlash {
        erased: BTreeSet<u32>,
        broken: bool,
        reads: usize,
    }

    impl FakeFlash {
        fn check(&mut self, address: u64, len: usize) -> Result<(), Error> {
            self.reads += 1;
            let page = |a: u64| (a as u32) & !(PAGE as u32 - 1);
            if self.broken || (address..address + len as u64).any(|a| self.erased.contains(&page(a)))
            {
                return Err(other("read failed"));
            }
            Ok(())
        }
    }

    impl MemoryInterface for FakeFlash {
        fn supports_native_64bit_access(&mut self) -> bool {
            true
        }
        fn read_64(&mut self, address: u64, data: &mut [u64]) -> Result<(), Error> {
            self.check(address, data.len() * 8)
        }
        fn read_32(&mut self, address: u64, data: &mut [u32]) -> Result<(), Error> {
            self.check(address, data.len() * 4)
        }
        fn read_16(&mut self, address: u64, data: &mut [u16]) -> Result<(), Error> {
            self.check(address, data.len() * 2)
        }
        fn read_8(&mut self, address: u64, data: &mut [u8]) -> Result<(), Error> {
            self.check(address, data.len())?;
            data.fill(0x5A);
            Ok(())
        }
        fn write_64(&mut self, _: u64, _: &[u64]) -> Result<(), Error> {
            unimplemented!()
        }
        fn write_32(&mut self, _: u64, _: &[u32]) -> Result<(), Error> {
            unimplemented!()
        }
        fn write_16(&mut self, _: u64, _: &[u16]) -> Result<(), Error> {
            unimplemented!()
        }
        fn write_8(&mut self, _: u64, _: &[u8]) -> Result<(), Error> {
            unimplemented!()
        }
        fn supports_8bit_transfers(&self) -> Result<bool, Error> {
            Ok(true)
        }
        fn flush(&mut self) -> Result<(), Error> {
            Ok(())
        }
    }

    #[test]
    fn pages_that_fail_to_read_are_erased_unless_the_connection_is_broken() {
        let base = 0xA040_0000;
        let mut flash = FakeFlash::default();
        flash.erased.insert(base + 0x20);
        flash.erased.insert(base + 0x3FE0);
        let pages = read_pages(&mut flash, base, 0x4000).unwrap();
        assert_eq!(pages[1], None);
        assert_eq!(pages[511], None);
        assert_eq!(pages[0], Some([0x5A; PAGE]));
        assert_eq!(pages.iter().filter(|p| p.is_none()).count(), 2);

        // A broken connection fails every read: that is no erased sector.
        let mut flash = FakeFlash {
            broken: true,
            ..FakeFlash::default()
        };
        assert!(read_pages(&mut flash, base, 0x4000).is_err());
        // It is found at the first read.
        assert_eq!(flash.reads, 2);
    }

    #[test]
    fn dflash_pages_of_zeros_are_erased() {
        let sector = Sector {
            base: 0xAF00_2000,
            size: 0x2000,
        };
        let mut patches = BTreeMap::new();
        patches.insert(sector.base, [Some(0); PAGE]);
        let mut patch = [None; PAGE];
        patch[3] = Some(0x5A);
        patches.insert(sector.base + PAGE as u32, patch);

        let image = sector_image(sector, &patches, &[]);
        assert_eq!(image.len(), 256);
        assert_eq!(image[0], None);
        let mut expected = [0; PAGE];
        expected[3] = 0x5A;
        assert_eq!(image[1], Some(expected));
        assert_eq!(image.iter().filter(|p| p.is_some()).count(), 1);

        // In the PFLASH a page of zeros is data.
        let pflash = Sector {
            base: 0xA021_C000,
            size: 0x4000,
        };
        let mut patches = BTreeMap::new();
        patches.insert(pflash.base, [Some(0); PAGE]);
        assert_eq!(sector_image(pflash, &patches, &[])[0], Some([0; PAGE]));
    }

    #[test]
    fn the_dflash_map_comes_from_the_device_file() {
        let dir = device_dir(
            "TC27x_dflash",
            "flax_TC2xx.hex",
            "0x401DA083",
            &builtin_runs(),
        );
        // Add a DataFlash memory: two sectors (the second restricted) and a UCB-range sector,
        // which is ignored.
        let file = dir.join("TC27x_dflash.json");
        let json = std::fs::read_to_string(&file).unwrap().replace(
            "] } ] }",
            r#"] },
               { "memoryName": "DataFlash", "memoryStartAddress": "0xAF000000",
                 "memoryEndAddress": "0xAF003FFF", "logicalSectors": [
                   { "sectorStartAddress": "0xAF000000", "sectorSize": "0x2000", "restricted": false },
                   { "sectorStartAddress": "0xAF002000", "sectorSize": "0x2000", "restricted": true },
                   { "sectorStartAddress": "0xAF100000", "sectorSize": "0x400", "restricted": false } ] } ] }"#,
        );
        std::fs::write(&file, json).unwrap();
        let map = FlashMap::for_device(std::slice::from_ref(&dir), 0x401D_A083, None).unwrap();
        assert_eq!(map.dflash.len(), 2);
        assert_eq!(map.sector(0xAF00_0100).unwrap().base, 0xAF00_0000);
        let error = map.sector(0xAF00_2000).unwrap_err().to_string();
        assert!(error.contains("restricted"), "{error}");
        assert!(map.sector(0xAF10_0000).is_err());
        assert!(map.sector(0xAF00_4000).is_err());
        std::fs::remove_dir_all(dir).unwrap();

        // A file without a DataFlash memory keeps the built-in DFLASH map.
        let dir = device_dir(
            "TC27x_no_dflash",
            "flax_TC2xx.hex",
            "0x401DA083",
            &builtin_runs(),
        );
        let map = FlashMap::for_device(std::slice::from_ref(&dir), 0x401D_A083, None).unwrap();
        assert_eq!(map.dflash, tc27x_dflash_sectors());
        std::fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn chips_and_jtag_ids_name_their_flash_device() {
        assert_eq!(FlashDevice::for_chip("TC275"), Some(FlashDevice::Tc27x));
        assert_eq!(FlashDevice::for_chip("TC36x"), Some(FlashDevice::Tc36x));
        assert_eq!(FlashDevice::for_chip("TC375"), None);
        assert_eq!(
            FlashDevice::for_jtag_id(0x501D_A083),
            Some(FlashDevice::Tc27x)
        );
        assert_eq!(
            FlashDevice::for_jtag_id(0x1020_9083),
            Some(FlashDevice::Tc36x)
        );
    }

    #[test]
    fn the_loaders_are_the_reviewed_builds() {
        // A change of a loader binary must be deliberate: rebuild with loader/build.ps1 and
        // update this check.
        // TC27x SHA-256 B01DE77697D2A6994F1BB8A07CAC7BE06C98FB88713DFB472638B15312E300AF.
        assert_eq!(LOADER_TC27X.len(), 1280);
        // TC3xx SHA-256 DAD758F0DCB352D3227DE92DBF11400B7F3DEC275D5E23D473CDDB961BD7DDB1.
        assert_eq!(LOADER_TC3XX.len(), 1128);
    }
}
