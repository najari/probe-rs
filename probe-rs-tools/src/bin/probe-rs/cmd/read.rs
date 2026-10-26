use anyhow::Context;
use ihex::Record;
use itertools::Itertools;

use probe_rs_rpc::core_ops::WireCoreStatus;
use probe_rs_rpc_client::{CoreInterface, RpcClient};

use crate::CoreOptions;
use crate::util::cli;
use crate::util::common_options::{ProbeOptions, ReadWriteBitWidth, ReadWriteOptions};
use std::io::Write;
use std::path::PathBuf;

#[derive(clap::ValueEnum, Clone, Copy)]
enum OutputFormat {
    /// Intel Hex Format
    Ihex,
    /// Simple list of hexadecimal numbers
    SimpleHex,
    /// Hexadecimal numbers formatted into a table
    HexTable,
    /// The raw binary
    Binary,
}

impl OutputFormat {
    fn write(
        self,
        dst: &mut dyn Write,
        address: u64,
        width: ReadWriteBitWidth,
        data: &[u8],
    ) -> anyhow::Result<()> {
        match self {
            OutputFormat::Binary => Self::write_binary(dst, data),
            OutputFormat::Ihex => Self::write_ihex(dst, address, data),
            OutputFormat::SimpleHex => Self::write_simple_hex(dst, width, data),
            OutputFormat::HexTable => Self::write_hex_table(dst, address, width, data),
        }
    }

    fn write_simple_hex(
        dst: &mut dyn Write,
        width: ReadWriteBitWidth,
        data: &[u8],
    ) -> anyhow::Result<()> {
        let bytes = match width {
            ReadWriteBitWidth::B8 => 1,
            ReadWriteBitWidth::B16 => 2,
            ReadWriteBitWidth::B32 => 4,
            ReadWriteBitWidth::B64 => 8,
        };

        let mut first = true;
        for window in data.chunks(bytes) {
            if first {
                first = false;
            } else {
                write!(dst, " ")?;
            }

            for byte in window.iter().rev() {
                write!(dst, "{byte:02x}")?;
            }
        }

        writeln!(dst)?;
        Ok(())
    }

    fn write_hex_table(
        dst: &mut dyn Write,
        mut address: u64,
        width: ReadWriteBitWidth,
        data: &[u8],
    ) -> anyhow::Result<()> {
        let bytes_in_line = match width {
            ReadWriteBitWidth::B8 => 8,
            ReadWriteBitWidth::B16 => 16,
            ReadWriteBitWidth::B32 | ReadWriteBitWidth::B64 => 32,
        };
        for window in data.chunks(bytes_in_line) {
            write!(dst, "{address:08x}: ")?;
            Self::write_simple_hex(dst, width, window)?;
            address += bytes_in_line as u64;
        }

        Ok(())
    }

    fn write_binary(dst: &mut dyn Write, data: &[u8]) -> anyhow::Result<()> {
        dst.write_all(data)?;

        Ok(())
    }

    fn write_ihex(dst: &mut dyn Write, address: u64, data: &[u8]) -> anyhow::Result<()> {
        let mut running_address = address;
        let mut records = vec![];

        for chunk in &data.iter().copied().chunks(255) {
            let address_msbs: u16 = (running_address >> 16)
                .try_into()
                .context("Hex format only supports addressing up to 32 bits")?;

            records.push(Record::ExtendedLinearAddress(address_msbs));

            records.push(Record::Data {
                offset: (running_address & 0xFFFF) as u16,
                value: chunk.collect(),
            });
            running_address += 255;
        }
        records.push(Record::EndOfFile);
        let hexdata = ihex::create_object_file_representation(&records)?;
        dst.write_all(hexdata.as_bytes())?;

        Ok(())
    }
}

/// Read from target memory address
///
/// e.g. probe-rs read b32 0x400E1490 2
///      Reads 2 32-bit words from address 0x400E1490
///
/// Default output is a space separated list of hex values padded to the read word width.
/// e.g. 2 words
///     00 00 (8-bit)
///     00000000 00000000 (32-bit)
///     0000000000000000 0000000000000000 (64-bit)
///
/// If the --output argument is provided, readback data is instead saved to a file as hex/bin.
/// In this case, the read word width has no effect except determining the total number of bytes
#[derive(clap::Parser)]
#[clap(verbatim_doc_comment)]
pub struct Cmd {
    #[clap(flatten)]
    shared: CoreOptions,

    #[clap(flatten)]
    probe_options: ProbeOptions,

    #[clap(flatten)]
    read_write_options: ReadWriteOptions,

    /// Number of words to read from the target
    words: usize,

    /// File to output binary data to
    #[arg(long, short)]
    output: Option<PathBuf>,

    /// Format of the outputted binary data
    #[clap(value_enum, default_value_t=OutputFormat::HexTable)]
    #[arg(long, short)]
    format: OutputFormat,
}

impl Cmd {
    pub async fn run(self, client: RpcClient) -> anyhow::Result<()> {
        let session = cli::attach_probe(&client, self.probe_options, None, false).await?;
        // A read can halt a core to reach the memory (RISC-V, Xtensa): such a core runs again
        // afterwards. A core that was halted before, by a debugger or another tool, stays
        // halted. Without the states, every halted core runs, as before.
        let resume = match session.cores_status(None).await {
            Ok(before) => Some(cores_to_resume(&before.statuses)),
            Err(error) => {
                tracing::warn!(
                    "The core states could not be read ({error}): every halted core runs after the read"
                );
                None
            }
        };
        let core = session.core(self.shared.core);

        let data = Self::read_memory(
            core,
            self.read_write_options.address,
            self.read_write_options.width,
            self.words,
        )
        .await?;

        match self.output {
            Some(path) => Self::save_to_file(
                self.read_write_options.address,
                &data,
                path,
                self.read_write_options.width,
                self.format,
            )?,
            None => Self::print_to_console(
                self.read_write_options.address,
                &data,
                self.read_write_options.width,
                self.format,
            )?,
        };

        match resume {
            Some(cores) if cores.is_empty() => {}
            // Only the cores halted by now are run.
            Some(cores) => {
                session.resume_cores(Some(cores)).await?;
            }
            None => session.resume_all_cores().await?,
        }

        Ok(())
    }

    async fn read_memory(
        core: CoreInterface,
        address: u64,
        width: ReadWriteBitWidth,
        nwords: usize,
    ) -> anyhow::Result<Vec<u8>> {
        let bytes = match width {
            ReadWriteBitWidth::B8 => {
                let values = core.read_memory_8(address, nwords).await?;
                values
                    .into_iter()
                    .flat_map(|v| v.to_le_bytes())
                    .collect::<Vec<_>>()
            }
            ReadWriteBitWidth::B16 => {
                let values = core.read_memory_16(address, nwords).await?;
                values
                    .into_iter()
                    .flat_map(|v| v.to_le_bytes())
                    .collect::<Vec<_>>()
            }
            ReadWriteBitWidth::B32 => {
                let values = core.read_memory_32(address, nwords).await?;
                values
                    .into_iter()
                    .flat_map(|v| v.to_le_bytes())
                    .collect::<Vec<_>>()
            }
            ReadWriteBitWidth::B64 => {
                let values = core.read_memory_64(address, nwords).await?;
                values
                    .into_iter()
                    .flat_map(|v| v.to_le_bytes())
                    .collect::<Vec<_>>()
            }
        };
        Ok(bytes)
    }

    fn save_to_file(
        address: u64,
        data: &[u8],
        path: PathBuf,
        width: ReadWriteBitWidth,
        format: OutputFormat,
    ) -> anyhow::Result<()> {
        let mut file = std::fs::File::create(path)?;
        format.write(&mut file, address, width, data)?;
        Ok(())
    }

    fn print_to_console(
        address: u64,
        data: &[u8],
        width: ReadWriteBitWidth,
        format: OutputFormat,
    ) -> anyhow::Result<()> {
        let mut stdout = std::io::stdout();
        format.write(&mut stdout, address, width, data)?;
        Ok(())
    }
}

/// The cores to run after a read, given their states before it: those that were not halted.
fn cores_to_resume(before: &[(u32, WireCoreStatus)]) -> Vec<u32> {
    before
        .iter()
        .filter(|(_, status)| !matches!(status, WireCoreStatus::Halted(_)))
        .map(|(core, _)| *core)
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use probe_rs_rpc::core_ops::WireHaltReason;

    #[test]
    fn cores_halted_before_the_read_stay_halted() {
        let before = [
            // Halted by a debugger (or `start_in_flash`): not run.
            (0, WireCoreStatus::Halted(WireHaltReason::Request)),
            (1, WireCoreStatus::Running),
            (
                2,
                WireCoreStatus::Halted(WireHaltReason::Breakpoint(
                    probe_rs_rpc::core_ops::WireBreakpointCause::Hardware,
                )),
            ),
            (3, WireCoreStatus::Sleeping),
            (4, WireCoreStatus::Unknown),
            (5, WireCoreStatus::LockedUp),
        ];
        // The rest run again if the read halted them (the server runs halted cores only).
        assert_eq!(cores_to_resume(&before), [1, 3, 4, 5]);
        assert!(cores_to_resume(&before[..1]).is_empty());
        assert!(cores_to_resume(&[]).is_empty());
    }
}
