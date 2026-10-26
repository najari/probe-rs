//! Server-side CMSIS-SVD parser and peripheral variable cache.
//!
//! The RPC server owns the live `probe_rs::Core`, so peripheral register
//! reads happen here. The DAP client never builds an SVD cache; it uploads the
//! SVD file (via the temp-file endpoints) and calls `load_svd`, which parses
//! the file and stores the resulting [`SvdVariableCache`] in the per-core
//! [`crate::rpc::debug_state::CoreDebugState`].

use std::collections::BTreeMap;
use std::io::Read;
use std::path::Path;

use probe_rs::MemoryInterface;
use probe_rs_debug::{DebugError, ObjectRef, get_object_reference};

/// Cache of SVD-derived variables for one core, structured down to the field
/// level. Built once per debug session from the CMSIS-SVD file; only the
/// register/field *values* are re-read from the target on each query.
#[derive(Debug, Clone, PartialEq)]
pub struct SvdVariableCache {
    root_variable_key: ObjectRef,
    variable_hash_map: BTreeMap<ObjectRef, Variable>,
}

impl SvdVariableCache {
    /// Create a new, empty cache with a root variable.
    pub(crate) fn new_svd_cache() -> Self {
        let root_variable_key = get_object_reference();
        let root_variable = Variable {
            variable_key: root_variable_key,
            parent_key: ObjectRef::Invalid,
            name: "Peripheral variable".to_string(),
            variable_kind: SvdVariable::Root,
        };

        SvdVariableCache {
            root_variable_key,
            variable_hash_map: BTreeMap::from([(root_variable_key, root_variable)]),
        }
    }

    /// All children of `parent_key` (top-level variables when `parent_key` is
    /// [`ObjectRef::Invalid`]).
    pub fn get_children(&self, parent_key: ObjectRef) -> Vec<&Variable> {
        self.variable_hash_map
            .values()
            .filter(|child_variable| child_variable.parent_key == parent_key)
            .collect::<Vec<_>>()
    }

    /// Look up a variable by its key.
    pub fn get_variable_by_key(&self, variable_key: ObjectRef) -> Option<&Variable> {
        self.variable_hash_map.get(&variable_key)
    }

    /// The root variable's key (the `variables_reference` for the Peripherals
    /// scope).
    pub fn root_variable_key(&self) -> ObjectRef {
        self.root_variable_key
    }

    fn add_variable(
        &mut self,
        parent_key: ObjectRef,
        name: String,
        variable: SvdVariable,
    ) -> Result<ObjectRef, DebugError> {
        let variable_key = get_object_reference();
        let cache_variable = Variable {
            variable_key,
            parent_key,
            name,
            variable_kind: variable,
        };

        // Validate that the parent_key exists ...
        if !self.variable_hash_map.contains_key(&parent_key) {
            return Err(DebugError::Other(format!(
                "SvdVariableCache: Attempted to add a new variable: {} with non existent `parent_key`: {:?}. Please report this as a bug",
                cache_variable.name, parent_key
            )));
        }

        if let Some(old_variable) = self
            .variable_hash_map
            .insert(cache_variable.variable_key, cache_variable.clone())
        {
            return Err(DebugError::Other(format!(
                "Attempt to insert a new `SvdVariable`:{:?} with a duplicate cache key: {:?}. Please report this as a bug.",
                cache_variable.name, old_variable.variable_key
            )));
        }

        Ok(cache_variable.variable_key)
    }

    /// Retrieve a variable by name (logs a warning if more than one matches).
    pub(crate) fn get_variable_by_name_and_parent(
        &self,
        variable_name: &str,
        parent_key: ObjectRef,
    ) -> Option<&Variable> {
        let child_variables = self
            .variable_hash_map
            .values()
            .filter(|child_variable| {
                child_variable.name == variable_name && child_variable.parent_key == parent_key
            })
            .collect::<Vec<&Variable>>();

        match &child_variables[..] {
            [] => None,
            [variable] => Some(*variable),
            [.., last] => {
                tracing::error!(
                    "Found {} variables with parent_key={:?} and name={}. Please report this as a bug.",
                    child_variables.len(),
                    parent_key,
                    variable_name
                );
                Some(last)
            }
        }
    }
}

/// A single SVD variable (root / peripheral group / peripheral / register / field).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Variable {
    variable_key: ObjectRef,
    parent_key: ObjectRef,
    name: String,
    pub variable_kind: SvdVariable,
}

impl Variable {
    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn variable_key(&self) -> ObjectRef {
        self.variable_key
    }

    /// Memory reference, compatible with DAP.
    pub fn memory_reference(&self) -> Option<String> {
        match self.variable_kind {
            SvdVariable::SvdRegister {
                address,
                restricted_read: false,
                ..
            } => Some(format!("{address:#010X}")),
            SvdVariable::SvdField {
                address,
                restricted_read: false,
                ..
            } => Some(format!("{address:#010X}")),
            _ => None,
        }
    }

    pub fn type_name(&self) -> Option<String> {
        self.variable_kind.type_name()
    }

    /// Read the variable's value from the target via `MemoryInterface`.
    pub fn get_value(&self, memory: &mut dyn MemoryInterface) -> String {
        self.variable_kind.get_value(memory)
    }

    /// Write `text` to the register or field on the target, and return the value read back.
    ///
    /// A register is written whole. A field is written by reading its register, replacing the
    /// field's bits and writing the register back, so a field of a register that cannot be
    /// read without side effects is refused. Values are hexadecimal (`0x...`), binary
    /// (`0b...`) or decimal, and must fit the register or field.
    pub fn set_value(
        &self,
        memory: &mut dyn MemoryInterface,
        text: &str,
    ) -> Result<String, String> {
        match &self.variable_kind {
            SvdVariable::SvdRegister { address, size, .. } => {
                let value = parse_value(text, (*size).clamp(1, 32))?;
                let width = register_access(*address, *size)?;
                write_register(memory, *address, width, value)
                    .map_err(|e| format!("writing {address:#010X} failed: {e}"))?;
            }
            SvdVariable::SvdField {
                address,
                register_size,
                restricted_read,
                bit_range_lower_bound,
                bit_range_upper_bound,
                ..
            } => {
                if *restricted_read {
                    return Err(format!(
                        "{} is part of a register that cannot be read without side effects; \
                         write the whole register instead",
                        self.name
                    ));
                }
                let (lower, upper) = (*bit_range_lower_bound, *bit_range_upper_bound);
                let field = parse_value(text, upper.saturating_sub(lower))?;
                // The register is read and written with its own size at its own address: a
                // wider access would also read and write the registers next to it.
                let width = register_access(*address, *register_size)?;
                if upper > width {
                    return Err(format!(
                        "{} (bits {lower}..{upper}) does not fit its {width}-bit register",
                        self.name
                    ));
                }
                let old = read_register(memory, *address, width)
                    .map_err(|e| format!("reading {address:#010X} failed: {e}"))?;
                let new = merge_field(old, lower, upper, field)?;
                write_register(memory, *address, width, new)
                    .map_err(|e| format!("writing {address:#010X} failed: {e}"))?;
            }
            _ => return Err(format!("{} is not a register or a field", self.name)),
        }
        Ok(self.get_value(memory))
    }
}

/// The access width of a register of `size` bits at `address`: 8, 16 or 32 bits, at an address
/// aligned to it.
fn register_access(address: u64, size: u32) -> Result<u32, String> {
    let width = match size {
        0..=8 => 8,
        9..=16 => 16,
        _ => 32,
    };
    aligned(address, u64::from(width / 8))?;
    Ok(width)
}

fn read_register(
    memory: &mut dyn MemoryInterface,
    address: u64,
    width: u32,
) -> Result<u32, probe_rs::Error> {
    match width {
        8 => memory.read_word_8(address).map(u32::from),
        16 => memory.read_word_16(address).map(u32::from),
        _ => memory.read_word_32(address),
    }
}

fn write_register(
    memory: &mut dyn MemoryInterface,
    address: u64,
    width: u32,
    value: u32,
) -> Result<(), probe_rs::Error> {
    match width {
        8 => memory.write_word_8(address, value as u8),
        16 => memory.write_word_16(address, value as u16),
        _ => memory.write_word_32(address, value),
    }
}

fn aligned(address: u64, size: u64) -> Result<(), String> {
    if address.is_multiple_of(size) {
        Ok(())
    } else {
        Err(format!("{address:#010X} is not aligned to {size} bytes"))
    }
}

/// Parses a register or field value of `width` bits: `0x...`, `0b...` or decimal, with optional
/// `_` separators.
fn parse_value(text: &str, width: u32) -> Result<u32, String> {
    let text = text.trim().replace('_', "");
    let lower = text.to_ascii_lowercase();
    let parsed = if let Some(hex) = lower.strip_prefix("0x") {
        u64::from_str_radix(hex, 16)
    } else if let Some(bin) = lower.strip_prefix("0b") {
        u64::from_str_radix(bin, 2)
    } else {
        lower.parse::<u64>()
    }
    .map_err(|_| format!("'{text}' is not a number (use 0x.., 0b.. or decimal)"))?;
    let max = if width >= 32 {
        u64::from(u32::MAX)
    } else {
        (1u64 << width) - 1
    };
    if parsed > max {
        return Err(format!("{text} does not fit in {width} bit(s)"));
    }
    Ok(parsed as u32)
}

/// Replaces bits `lower..upper` (upper exclusive) of `register` with `field`.
fn merge_field(register: u32, lower: u32, upper: u32, field: u32) -> Result<u32, String> {
    if lower >= upper || upper > 32 {
        return Err(format!("invalid bit range {lower}..{upper}"));
    }
    let width = upper - lower;
    let mask = if width == 32 {
        u32::MAX
    } else {
        ((1u32 << width) - 1) << lower
    };
    Ok((register & !mask) | ((field << lower) & mask))
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SvdVariable {
    Root,
    SvdRegister {
        address: u64,
        restricted_read: bool,
        description: Option<String>,
        size: u32,
    },
    SvdField {
        /// The address of the field's register.
        address: u64,
        /// The size of the field's register, in bits.
        register_size: u32,
        restricted_read: bool,
        bit_range_lower_bound: u32,
        bit_range_upper_bound: u32,
        description: Option<String>,
    },
    SvdPeripheral {
        base_address: u64,
        description: Option<String>,
    },
    SvdPeripheralGroup {
        description: Option<String>,
    },
}

impl SvdVariable {
    fn get_value(&self, memory: &mut dyn MemoryInterface) -> String {
        match &self {
            SvdVariable::Root => "".to_string(),
            SvdVariable::SvdPeripheral { description, .. }
            | SvdVariable::SvdPeripheralGroup { description } => {
                description.as_ref().cloned().unwrap_or_default()
            }
            SvdVariable::SvdRegister {
                address,
                restricted_read,
                size,
                ..
            } => {
                if *restricted_read {
                    format!("Register cannot be read without side effects @ {address:#010X}")
                } else {
                    let size_bytes = (*size / 8).max(1);
                    let bits_alignment_offset = (*address % size_bytes as u64) as u32 * 8;
                    let value = match *size {
                        0..=8 => memory
                            .read_word_8(*address)
                            .map(|val| format!("{val:#04X}")),
                        9..=16 => {
                            let aligned_address = *address & !0b1;
                            memory
                                .read_word_16(aligned_address)
                                .map(|val| format!("{:#06X}", val >> bits_alignment_offset))
                        }
                        _ => {
                            let aligned_address = *address & !0b11;
                            memory
                                .read_word_32(aligned_address)
                                .map(|val| format!("{:#010X}", val >> bits_alignment_offset))
                        }
                    };

                    match value {
                        Ok(value) => value,
                        Err(error) => format!(
                            "Unable to read peripheral register value @ {address:#010X} : {error:?}"
                        ),
                    }
                }
            }
            SvdVariable::SvdField {
                address,
                register_size,
                restricted_read,
                bit_range_lower_bound,
                bit_range_upper_bound,
                ..
            } => {
                if *restricted_read {
                    format!("Field cannot be read without side effects @ {address:#010X}")
                } else {
                    // The register with its own size: a wider read would also read the
                    // registers next to it (some have side effects when read).
                    let value = match register_access(*address, *register_size) {
                        Ok(width) => read_register(memory, *address, width),
                        Err(error) => return error,
                    };

                    match value {
                        Ok(register_u32_value) => {
                            let mut bit_value: u32 = register_u32_value;
                            bit_value <<= 32 - bit_range_upper_bound;
                            bit_value >>= 32 - (bit_range_upper_bound - bit_range_lower_bound);
                            format!(
                                "{:0width$b} @ {:#010X}:{}..{}",
                                bit_value,
                                address,
                                bit_range_lower_bound,
                                bit_range_upper_bound,
                                width = (*bit_range_upper_bound - *bit_range_lower_bound) as usize
                            )
                        }
                        Err(error) => format!(
                            "Unable to read peripheral register field value @ {address:#010X} : {error:?}"
                        ),
                    }
                }
            }
        }
    }

    fn type_name(&self) -> Option<String> {
        match &self {
            SvdVariable::SvdRegister { description, .. }
            | SvdVariable::SvdField { description, .. } => description.clone(),
            SvdVariable::SvdPeripheral { .. } => Some("Peripheral".to_string()),
            SvdVariable::SvdPeripheralGroup { .. } => Some("Peripheral Group".to_string()),
            SvdVariable::Root => None,
        }
    }
}

/// Read and parse an SVD file at `path` (plain, or gzip compressed with a `.gz` extension), then
/// build the variable cache.
#[tracing::instrument(skip_all)]
pub fn parse_svd_file(path: &Path) -> Result<SvdVariableCache, DebugError> {
    let mut svd_xml = String::new();
    let file = std::fs::File::open(path)?;
    if path
        .extension()
        .is_some_and(|extension| extension.eq_ignore_ascii_case("gz"))
    {
        flate2::read::GzDecoder::new(file).read_to_string(&mut svd_xml)?;
    } else {
        std::io::BufReader::new(file).read_to_string(&mut svd_xml)?;
    }
    let svd_xml = without_empty_access(svd_xml);

    let device = svd_parser::parse_with_config(
        &svd_xml,
        // `expand_properties` pushes each peripheral's / the device's
        // `defaultRegisterProperties` (including `access`) down onto the individual
        // registers, so a register whose `access` is still `None` afterwards is one
        // that is genuinely unspecified anywhere in the SVD tree.
        &svd_parser::Config::default()
            .expand(true)
            .expand_properties(true)
            .ignore_enums(true),
    )
    .map_err(|error| {
        DebugError::Other(format!(
            "Unable to parse CMSIS-SVD file: {path:?}. {error:?}"
        ))
    })?;

    build_svd_cache(&device)
}

/// Removes empty `<access>` elements, which `svd_parser` refuses. The Infineon TC3xx SVD files
/// have hundreds of them on fields; an absent `access` means the same (the field takes the
/// access of its register).
fn without_empty_access(svd_xml: String) -> String {
    const EMPTY: [&str; 3] = ["<access />", "<access/>", "<access></access>"];
    if !EMPTY.iter().any(|empty| svd_xml.contains(empty)) {
        return svd_xml;
    }
    EMPTY
        .iter()
        .fold(svd_xml, |xml, empty| xml.replace(empty, ""))
}

/// Build an [`SvdVariableCache`] from a parsed SVD `Device`.
#[tracing::instrument(skip_all)]
fn build_svd_cache(
    peripheral_device: &svd_parser::svd::Device,
) -> Result<SvdVariableCache, DebugError> {
    let mut svd_cache = SvdVariableCache::new_svd_cache();
    let device_root_variable_key = svd_cache.root_variable_key();

    let device_default_access = peripheral_device.default_register_properties.access;

    for peripheral in &peripheral_device.peripherals {
        let current_peripheral_group_name = peripheral.group_name.as_ref();

        let peripheral_parent_key;

        // Adding the Peripheral Group Name as an additional level in the structure helps to keep the 'variable tree' more compact,
        // but more importantly, it helps to avoid having duplicate variable names that conflict with hal crates.
        if let Some(peripheral_group_name) = &peripheral.group_name {
            // Before we create a new group variable, check if we have one by that name already.
            peripheral_parent_key = match svd_cache
                .get_variable_by_name_and_parent(peripheral_group_name, device_root_variable_key)
            {
                Some(existing_peripheral_group_variable) => {
                    existing_peripheral_group_variable.variable_key()
                }
                None => svd_cache.add_variable(
                    device_root_variable_key,
                    peripheral_group_name.clone(),
                    SvdVariable::SvdPeripheralGroup {
                        description: peripheral.description.clone(),
                    },
                )?,
            };
        } else {
            peripheral_parent_key = device_root_variable_key;
        }

        let peripheral_name = if let Some(peripheral_group) = current_peripheral_group_name {
            format!("{}.{}", peripheral_group, peripheral.name)
        } else {
            peripheral.name.clone()
        };

        let peripheral_key = svd_cache.add_variable(
            peripheral_parent_key,
            peripheral_name.clone(),
            SvdVariable::SvdPeripheral {
                base_address: peripheral.base_address,
                description: peripheral.description.clone(),
            },
        )?;

        for register in peripheral.all_registers() {
            let register_address = peripheral.base_address + register.address_offset as u64;

            // After property expansion above, a `None` access means no `<access>` was
            // declared anywhere in the tree (register, peripheral or device). Treat such
            // registers as readable: many recent SVDs (e.g. STM32H5) omit `<access>`
            // entirely, and defaulting to restricted made every register unviewable in
            // the debugger. Registers with a `read_action` are still restricted, so
            // side-effecting reads remain protected.
            let mut register_has_restricted_read = register.read_action.is_some()
                || register
                    .properties
                    .access
                    .map(|a| !a.can_read())
                    .or_else(|| device_default_access.map(|a| !a.can_read()))
                    .unwrap_or(false);

            let register_name = format!("{peripheral_name}.{}", register.name);

            let mut field_variables = Vec::new();

            for field in register.fields() {
                let field_has_restricted_read = register_has_restricted_read
                    || field.read_action.is_some()
                    || field
                        .access
                        .map(|a| !a.can_read())
                        .or_else(|| device_default_access.map(|a| !a.can_read()))
                        .unwrap_or(register_has_restricted_read);

                let field_variable = (
                    format!("{}.{}", register_name, field.name),
                    SvdVariable::SvdField {
                        address: register_address,
                        register_size: register.properties.size.unwrap_or(32),
                        restricted_read: field_has_restricted_read,
                        bit_range_lower_bound: field.bit_offset(),
                        bit_range_upper_bound: (field.bit_offset() + field.bit_width()),
                        description: field.description.clone(),
                    },
                );

                // If any of the fields in the register have restricted read, then the register has restricted read.
                register_has_restricted_read |= field_has_restricted_read;

                field_variables.push(field_variable);
            }

            let register_variable_key = svd_cache.add_variable(
                peripheral_key,
                format!("{peripheral_name}.{}", register.name),
                SvdVariable::SvdRegister {
                    address: register_address,
                    restricted_read: register_has_restricted_read,
                    description: register.description.clone(),
                    size: register.properties.size.unwrap_or(32),
                },
            )?;

            for (variable_name, variable) in field_variables {
                // TODO: Extend the Variable definition, so that we can resolve the EnumeratedValues for fields.
                svd_cache.add_variable(register_variable_key, variable_name, variable)?;
            }
        }
    }

    Ok(svd_cache)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_access_elements_are_removed() {
        let xml = "<field><access /><name>PS0</name><access/></field><access>read-only</access>";
        assert_eq!(
            without_empty_access(xml.to_string()),
            "<field><name>PS0</name></field><access>read-only</access>"
        );
    }

    #[test]
    fn values_are_parsed_and_checked_against_the_width() {
        assert_eq!(parse_value("0x1F", 8), Ok(0x1F));
        assert_eq!(parse_value("0b1010", 4), Ok(0b1010));
        assert_eq!(parse_value("1_000", 16), Ok(1000));
        assert_eq!(parse_value("0xFFFFFFFF", 32), Ok(u32::MAX));
        assert!(parse_value("0x100", 8).is_err());
        assert!(parse_value("4", 2).is_err());
        assert!(parse_value("ten", 8).is_err());
    }

    #[test]
    fn a_field_replaces_only_its_bits() {
        // Bits 4..8 of 0xFFFF_0F0F become 0b1010.
        assert_eq!(merge_field(0xFFFF_0F0F, 4, 8, 0b1010), Ok(0xFFFF_0FAF));
        // A one-bit field.
        assert_eq!(merge_field(0, 31, 32, 1), Ok(0x8000_0000));
        assert_eq!(merge_field(u32::MAX, 0, 1, 0), Ok(0xFFFF_FFFE));
        // The whole register.
        assert_eq!(merge_field(0x1234_5678, 0, 32, 0xCAFE), Ok(0xCAFE));
        assert!(merge_field(0, 8, 8, 0).is_err());
    }

    /// Memory that records each access (address, width in bits, write) and holds bytes.
    #[derive(Default)]
    struct Recorder {
        bytes: BTreeMap<u64, u8>,
        accesses: Vec<(u64, u32, bool)>,
    }

    impl Recorder {
        fn load(&mut self, address: u64, data: &[u8], width: u32, write: bool) {
            self.accesses.push((address, width, write));
            for (i, b) in data.iter().enumerate() {
                self.bytes.insert(address + i as u64, *b);
            }
        }
        fn get(&mut self, address: u64, len: usize, width: u32) -> Vec<u8> {
            self.accesses.push((address, width, false));
            (0..len as u64)
                .map(|i| *self.bytes.get(&(address + i)).unwrap_or(&0))
                .collect()
        }
    }

    impl MemoryInterface for Recorder {
        fn supports_native_64bit_access(&mut self) -> bool {
            false
        }
        fn read_64(&mut self, address: u64, data: &mut [u64]) -> Result<(), probe_rs::Error> {
            for (i, word) in data.iter_mut().enumerate() {
                let b = self.get(address + 8 * i as u64, 8, 64);
                *word = u64::from_le_bytes(b.try_into().unwrap());
            }
            Ok(())
        }
        fn read_32(&mut self, address: u64, data: &mut [u32]) -> Result<(), probe_rs::Error> {
            for (i, word) in data.iter_mut().enumerate() {
                let b = self.get(address + 4 * i as u64, 4, 32);
                *word = u32::from_le_bytes(b.try_into().unwrap());
            }
            Ok(())
        }
        fn read_16(&mut self, address: u64, data: &mut [u16]) -> Result<(), probe_rs::Error> {
            for (i, word) in data.iter_mut().enumerate() {
                let b = self.get(address + 2 * i as u64, 2, 16);
                *word = u16::from_le_bytes(b.try_into().unwrap());
            }
            Ok(())
        }
        fn read_8(&mut self, address: u64, data: &mut [u8]) -> Result<(), probe_rs::Error> {
            let b = self.get(address, data.len(), 8);
            data.copy_from_slice(&b);
            Ok(())
        }
        fn write_64(&mut self, address: u64, data: &[u64]) -> Result<(), probe_rs::Error> {
            for (i, w) in data.iter().enumerate() {
                self.load(address + 8 * i as u64, &w.to_le_bytes(), 64, true);
            }
            Ok(())
        }
        fn write_32(&mut self, address: u64, data: &[u32]) -> Result<(), probe_rs::Error> {
            for (i, w) in data.iter().enumerate() {
                self.load(address + 4 * i as u64, &w.to_le_bytes(), 32, true);
            }
            Ok(())
        }
        fn write_16(&mut self, address: u64, data: &[u16]) -> Result<(), probe_rs::Error> {
            for (i, w) in data.iter().enumerate() {
                self.load(address + 2 * i as u64, &w.to_le_bytes(), 16, true);
            }
            Ok(())
        }
        fn write_8(&mut self, address: u64, data: &[u8]) -> Result<(), probe_rs::Error> {
            self.load(address, data, 8, true);
            Ok(())
        }
        fn supports_8bit_transfers(&self) -> Result<bool, probe_rs::Error> {
            Ok(true)
        }
        fn flush(&mut self) -> Result<(), probe_rs::Error> {
            Ok(())
        }
    }

    fn field(address: u64, register_size: u32, lower: u32, upper: u32) -> Variable {
        Variable {
            variable_key: ObjectRef::Invalid,
            parent_key: ObjectRef::Invalid,
            name: "P.R.F".into(),
            variable_kind: SvdVariable::SvdField {
                address,
                register_size,
                restricted_read: false,
                bit_range_lower_bound: lower,
                bit_range_upper_bound: upper,
                description: None,
            },
        }
    }

    #[test]
    fn a_field_is_accessed_with_the_size_of_its_register() {
        // A 16-bit register at 0x1002, next to another one at 0x1000.
        let mut memory = Recorder::default();
        memory
            .bytes
            .extend([(0x1000, 0xAA), (0x1001, 0xBB), (0x1002, 0x0F), (0x1003, 0)]);
        let value = field(0x1002, 16, 12, 16)
            .set_value(&mut memory, "0x5")
            .unwrap();
        assert_eq!(
            memory.accesses,
            [(0x1002, 16, false), (0x1002, 16, true), (0x1002, 16, false)]
        );
        assert_eq!(memory.bytes[&0x1003], 0x50);
        assert_eq!(memory.bytes[&0x1000], 0xAA);
        assert!(value.starts_with("0101 @"), "{value}");

        // An 8-bit register with a field up to bit 8: a byte access.
        let mut memory = Recorder::default();
        field(0x2001, 8, 4, 8).set_value(&mut memory, "1").unwrap();
        assert!(
            memory
                .accesses
                .iter()
                .all(|&(a, w, _)| a == 0x2001 && w == 8)
        );

        // A field beyond its register, and a register at an address it cannot be at.
        let mut memory = Recorder::default();
        assert!(field(0x3000, 8, 4, 12).set_value(&mut memory, "1").is_err());
        assert!(field(0x3002, 32, 0, 4).set_value(&mut memory, "1").is_err());
        assert!(memory.accesses.is_empty());
    }

    /// The Infineon TC27x SVD parses from its gzip file, and every peripheral of the device
    /// becomes a variable.
    #[test]
    fn a_gzip_compressed_aurix_svd_parses() {
        let path = probe_rs_infineon::svd::find_svd("TC275", &[]).unwrap();
        let start = std::time::Instant::now();
        let cache = parse_svd_file(&path).unwrap();
        let names = cache
            .get_children(cache.root_variable_key())
            .into_iter()
            .map(|v| v.name().to_string())
            .collect::<Vec<_>>();
        println!(
            "{} top-level entries in {:?}: {}",
            names.len(),
            start.elapsed(),
            names.join(" ")
        );
        assert!(names.iter().any(|n| n.contains("CPU0")), "{names:?}");
        assert!(names.iter().any(|n| n.contains("SCU")), "{names:?}");
    }
}
