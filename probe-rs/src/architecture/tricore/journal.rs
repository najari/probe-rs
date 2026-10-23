//! A record of what a session wrote into the device for its debugging, so that a later session
//! can put it back when this one ends without doing so (the debugger was killed): the `DEBUG`
//! instructions it planted, the trigger units it armed and the halt group routing.
//!
//! While a session has such values on a core, it keeps a file with their addresses, the values
//! found there before and the values written, locked. The lock goes with the process. A session
//! that finds a file of the same core without a lock has found the record of a session that is
//! gone.
//!
//! One line per entry: `address original [written]`, hexadecimal. Without `written` the entry is
//! a `DEBUG` instruction (the journals of earlier versions have only these).

use std::{
    fs::{self, File, OpenOptions},
    io::{Read, Seek, SeekFrom, Write},
    path::{Path, PathBuf},
    sync::atomic::{AtomicU32, Ordering},
};

/// One value a session wrote: the bus address, the bytes found there before (put back), and the
/// bytes written, `None` for a `DEBUG` instruction (its length follows the original's).
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct Entry {
    pub(crate) address: u32,
    pub(crate) original: Vec<u8>,
    pub(crate) planted: Option<Vec<u8>>,
}

impl Entry {
    /// A `DEBUG` instruction planted at `address` over `original`.
    pub(crate) fn debug_instruction(address: u32, original: Vec<u8>) -> Self {
        Self {
            address,
            original,
            planted: None,
        }
    }

    /// A register at `address` that held `original` and was set to `written`.
    pub(crate) fn word(address: u32, original: u32, written: u32) -> Self {
        Self {
            address,
            original: original.to_le_bytes().to_vec(),
            planted: Some(written.to_le_bytes().to_vec()),
        }
    }
}

fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

fn unhex(text: &str) -> Option<Vec<u8>> {
    (0..text.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(text.get(i..i + 2)?, 16).ok())
        .collect()
}

/// Numbers the journals of this process.
static NEXT_JOURNAL: AtomicU32 = AtomicU32::new(0);

/// The journal of one core of this session.
#[derive(Debug)]
pub(crate) struct Journal {
    dir: Option<PathBuf>,
    core: u8,
    /// The file and its path while there are entries. The file is locked.
    file: Option<(PathBuf, File)>,
    /// The entries in the file.
    written: Vec<Entry>,
}

impl Journal {
    /// The journal of core `core`, kept in `dir`; `None` keeps no journal.
    pub(crate) fn new(dir: Option<PathBuf>, core: u8) -> Self {
        Self {
            dir,
            core,
            file: None,
            written: Vec::new(),
        }
    }

    /// Where journals are kept by default.
    pub(crate) fn default_dir() -> PathBuf {
        std::env::temp_dir().join("probe-rs-tricore-breakpoints")
    }

    fn prefix(&self) -> String {
        format!("core{}-", self.core)
    }

    /// Records `entries`, the values written now. Without entries, the file goes.
    /// A journal that cannot be written is not an error of the debugging: it is logged.
    pub(crate) fn record(&mut self, entries: &[Entry]) {
        if self.dir.is_none() || entries == self.written.as_slice() {
            return;
        }
        if entries.is_empty() {
            if let Some((path, file)) = self.file.take() {
                drop(file);
                if let Err(error) = fs::remove_file(&path) {
                    tracing::debug!("TriCore: removing {}: {error}", path.display());
                }
            }
            self.written.clear();
            return;
        }
        match self.write(entries) {
            Ok(()) => self.written = entries.to_vec(),
            Err(error) => tracing::warn!(
                "TriCore core {}: cannot record the breakpoints ({error}); if the debugger is \
                 killed, load the program again and reset the device",
                self.core
            ),
        }
    }

    fn write(&mut self, entries: &[Entry]) -> std::io::Result<()> {
        if self.file.is_none() {
            let dir = self.dir.as_ref().expect("checked by the caller");
            fs::create_dir_all(dir)?;
            let path = dir.join(format!(
                "{}{}-{}.txt",
                self.prefix(),
                std::process::id(),
                NEXT_JOURNAL.fetch_add(1, Ordering::Relaxed)
            ));
            let file = OpenOptions::new()
                .read(true)
                .write(true)
                .create_new(true)
                .open(&path)?;
            file.try_lock().map_err(std::io::Error::other)?;
            self.file = Some((path, file));
        }
        let (_, file) = self.file.as_mut().expect("opened above");
        let mut text = String::new();
        for entry in entries {
            text.push_str(&format!("{:08x} {}", entry.address, hex(&entry.original)));
            if let Some(planted) = &entry.planted {
                text.push_str(&format!(" {}", hex(planted)));
            }
            text.push('\n');
        }
        file.set_len(0)?;
        file.seek(SeekFrom::Start(0))?;
        file.write_all(text.as_bytes())?;
        file.sync_data()
    }

    /// Takes the entries of the journals of this core that no running session holds, and
    /// removes their files.
    pub(crate) fn take_abandoned(&self) -> Vec<Entry> {
        let Some(dir) = &self.dir else {
            return Vec::new();
        };
        let Ok(files) = fs::read_dir(dir) else {
            return Vec::new();
        };
        let own = self.file.as_ref().map(|(path, _)| path.as_path());
        let mut entries = Vec::new();
        for file in files.flatten() {
            let path = file.path();
            let name = file.file_name().to_string_lossy().into_owned();
            if !name.starts_with(&self.prefix()) || !name.ends_with(".txt") || Some(&*path) == own {
                continue;
            }
            // A locked journal is held by a running session.
            if let Some(found) = read_abandoned(&path) {
                tracing::debug!(
                    "TriCore core {}: {} left by a session that is gone: {} entries",
                    self.core,
                    path.display(),
                    found.len()
                );
                entries.extend(found);
                if let Err(error) = fs::remove_file(&path) {
                    tracing::debug!("TriCore: removing {}: {error}", path.display());
                }
            }
        }
        entries
    }

    /// Lets the file go without removing it, as when the process is killed (tests).
    #[cfg(test)]
    pub(crate) fn abandon(&mut self) {
        self.file = None;
        self.written.clear();
    }
}

/// `key` with every character that is not a letter, a digit, `-` or `.` replaced by `_`, for a
/// file or directory name.
pub(crate) fn file_name_part(key: &str) -> String {
    key.chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || c == '-' || c == '.' {
                c
            } else {
                '_'
            }
        })
        .collect()
}

/// The entries of the journal at `path`, unless a running session holds it.
fn read_abandoned(path: &Path) -> Option<Vec<Entry>> {
    let mut file = OpenOptions::new().read(true).write(true).open(path).ok()?;
    file.try_lock().ok()?;
    let mut text = String::new();
    file.read_to_string(&mut text).ok()?;
    Some(
        text.lines()
            .filter_map(|line| {
                let mut fields = line.split_whitespace();
                let address = u32::from_str_radix(fields.next()?, 16).ok()?;
                let original = unhex(fields.next()?)?;
                let planted = match fields.next() {
                    Some(written) => Some(unhex(written)?),
                    None => None,
                };
                Some(Entry {
                    address,
                    original,
                    planted,
                })
            })
            .collect(),
    )
}
