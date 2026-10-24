//! Tests of the MCD probe against fake MCD functions linked into the test binary.

use std::{
    collections::HashMap,
    ffi::{CStr, c_char},
    sync::{Mutex, MutexGuard},
};

use probe_rs::architecture::tricore::bus::{
    AddressMap, BusTransfer, ResetKind, TransferStatus, TriCoreBusAccess, TriCoreError,
};

use super::{ffi::*, *};

const JTAG_ID: u32 = 0x401D_A083;
/// Reads at this address fail.
const BAD_ADDRESS: u64 = 0xA021_C000;

/// What the fake library saw, and its memory.
#[derive(Default)]
struct Fake {
    memory: HashMap<u64, u8>,
    calls: Vec<String>,
    /// The number of transactions of every list.
    lists: Vec<usize>,
    resets: Vec<(u32, u32)>,
    last_error: (u32, String),
    open: bool,
    /// The memory spaces of every core: (id, name, type). Transactions must use the first that
    /// is not registers, a cache or virtual, or 0 without one.
    spaces: Vec<(u32, &'static str, u32)>,
}

impl Fake {
    fn device_space(&self) -> u32 {
        self.spaces
            .iter()
            .find(|(_, _, mem_type)| mem_type & 0xD == 0)
            .map_or(0, |(id, ..)| *id)
    }
}

static FAKE: Mutex<Option<Fake>> = Mutex::new(None);
/// Serialises the tests, which share the fake library.
static TEST: Mutex<()> = Mutex::new(());

fn fake() -> MutexGuard<'static, Option<Fake>> {
    FAKE.lock().unwrap_or_else(|e| e.into_inner())
}

fn with<R>(f: impl FnOnce(&mut Fake) -> R) -> R {
    f(fake().as_mut().unwrap())
}

fn fail(code: u32, text: &str) -> mcd_return_et {
    with(|s| s.last_error = (code, text.into()));
    3 // MCD_RET_ACT_HANDLE_ERROR
}

unsafe extern "C" fn initialize(
    version: *const mcd_api_version_st,
    info: *mut mcd_impl_version_info_st,
) -> mcd_return_et {
    let version = unsafe { &*version };
    assert_eq!((version.v_api_major, version.v_api_minor), (1, 1));
    let info = unsafe { &mut *info };
    set_c_string(&mut info.vendor, b"fake");
    with(|s| s.calls.push("initialize".into()));
    0
}

unsafe extern "C" fn exit() {
    with(|s| s.calls.push("exit".into()));
}

static mut SERVER: mcd_server_st = mcd_server_st {
    instance: ptr::null_mut(),
    host: ptr::null(),
    config_string: ptr::null(),
};

unsafe extern "C" fn open_server(
    _key: *const c_char,
    config: *const c_char,
    server: *mut *mut mcd_server_st,
) -> mcd_return_et {
    let config = if config.is_null() {
        "<none>".to_string()
    } else {
        unsafe { CStr::from_ptr(config) }
            .to_string_lossy()
            .into_owned()
    };
    with(|s| s.calls.push(format!("open_server {config}")));
    unsafe { *server = &raw mut SERVER };
    0
}

unsafe extern "C" fn close_server(_server: *const mcd_server_st) -> mcd_return_et {
    with(|s| s.calls.push("close_server".into()));
    0
}

/// Answers a list query of the MCD API from `entries`: a count of 0 asks for their number.
unsafe fn answer<T: Copy>(
    entries: &[T],
    start: u32,
    count: *mut u32,
    out: *mut T,
) -> mcd_return_et {
    unsafe {
        if *count == 0 {
            *count = entries.len() as u32;
            return 0;
        }
        let rest = entries.get(start as usize..).unwrap_or_default();
        let n = rest.len().min(*count as usize);
        std::ptr::copy_nonoverlapping(rest.as_ptr(), out, n);
        *count = n as u32;
    }
    0
}

/// The devices of the fake server: (device, access hardware, JTAG ID, number of cores).
const DEVICES: [(&str, &str, u32, usize); 2] = [
    ("TC27x", "DAS JTAG AURDUINO 46YPEADW", JTAG_ID, 3),
    ("TC36x", "DAS JTAG TriBoard 0815", 0x1020_6083, 2),
];

unsafe extern "C" fn qry_systems(
    start: u32,
    count: *mut u32,
    info: *mut mcd_core_con_info_st,
) -> mcd_return_et {
    let mut system: mcd_core_con_info_st = zeroed();
    set_c_string(&mut system.system, b"Real HW");
    unsafe { answer(&[system], start, count, info) }
}

unsafe extern "C" fn qry_devices(
    _system: *const mcd_core_con_info_st,
    start: u32,
    count: *mut u32,
    info: *mut mcd_core_con_info_st,
) -> mcd_return_et {
    let devices = DEVICES.map(|(name, acc_hw, jtag_id, _)| {
        let mut device: mcd_core_con_info_st = zeroed();
        set_c_string(&mut device.system, b"Real HW");
        set_c_string(&mut device.device, name.as_bytes());
        set_c_string(&mut device.acc_hw, acc_hw.as_bytes());
        device.device_type = jtag_id;
        device
    });
    unsafe { answer(&devices, start, count, info) }
}

unsafe extern "C" fn qry_cores(
    device: *const mcd_core_con_info_st,
    start: u32,
    count: *mut u32,
    info: *mut mcd_core_con_info_st,
) -> mcd_return_et {
    let device = unsafe { *device };
    let name = c_string(&device.device);
    let (.., cores) = DEVICES.iter().find(|d| d.0 == name).unwrap();
    let cores = (0..*cores)
        .map(|i| {
            let mut core = device;
            let kind = if i == 0 { "1.6E" } else { "1.6P" };
            set_c_string(
                &mut core.core,
                format!("Core {i}: CPU{i} TriCore {kind}").as_bytes(),
            );
            core.core_id = i as u32;
            core.core_type = 44;
            core
        })
        .collect::<Vec<_>>();
    unsafe { answer(&cores, start, count, info) }
}

unsafe extern "C" fn qry_mem_spaces(
    _core: *const mcd_core_st,
    start: u32,
    count: *mut u32,
    out: *mut mcd_memspace_st,
) -> mcd_return_et {
    let spaces = with(|s| s.spaces.clone())
        .into_iter()
        .map(|(id, name, mem_type)| {
            let mut space: mcd_memspace_st = zeroed();
            space.mem_space_id = id;
            set_c_string(&mut space.mem_space_name, name.as_bytes());
            space.mem_type = mem_type;
            space.max_addr = 0xFFFF_FFFF;
            space
        })
        .collect::<Vec<_>>();
    unsafe { answer(&spaces, start, count, out) }
}

unsafe extern "C" fn open_core(
    info: *const mcd_core_con_info_st,
    core: *mut *mut mcd_core_st,
) -> mcd_return_et {
    let name = c_string(unsafe { &(*info).core });
    let device = c_string(unsafe { &(*info).device });
    with(|s| {
        s.open = true;
        s.calls.push(format!("open_core {device} {name}"));
    });
    // Like a library, the fake owns the core object; the tests leak it.
    let info: &'static mcd_core_con_info_st = Box::leak(Box::new(unsafe { *info }));
    let core_obj = Box::leak(Box::new(mcd_core_st {
        instance: ptr::null_mut(),
        core_con_info: info,
    }));
    unsafe { *core = core_obj };
    0
}

unsafe extern "C" fn close_core(_core: *const mcd_core_st) -> mcd_return_et {
    with(|s| {
        s.open = false;
        s.calls.push("close_core".into());
    });
    0
}

unsafe extern "C" fn qry_error_info(_core: *const mcd_core_st, info: *mut mcd_error_info_st) {
    let (code, text) = with(|s| s.last_error.clone());
    unsafe {
        (*info).error_code = code;
        set_c_string(&mut (*info).error_str, text.as_bytes());
    }
}

unsafe extern "C" fn qry_max_payload_size(
    _core: *const mcd_core_st,
    max_payload: *mut u32,
) -> mcd_return_et {
    unsafe { *max_payload = 4096 };
    0
}

unsafe extern "C" fn execute_txlist(
    _core: *const mcd_core_st,
    txlist: *mut mcd_txlist_st,
) -> mcd_return_et {
    let txlist = unsafe { &mut *txlist };
    let txs = unsafe { std::slice::from_raw_parts_mut(txlist.tx, txlist.num_tx as usize) };
    let payload: u32 = txs.iter().map(|tx| tx.num_bytes).sum();
    assert!(txs.len() <= MCD_TX_NUM_MAX && payload <= 4096 || txs.len() == 1);
    with(|s| s.lists.push(txs.len()));
    txlist.num_tx_ok = 0;
    for tx in txs.iter_mut() {
        let data = unsafe { std::slice::from_raw_parts_mut(tx.data, tx.num_bytes as usize) };
        if tx.access_width != 0 {
            assert_eq!(tx.access_width as u32, tx.num_bytes);
        }
        let address = tx.addr.address;
        if tx.addr.mem_space_id != with(|s| s.device_space()) {
            return fail(MCD_ERR_TXLIST_TX, "unknown memory space");
        }
        if tx.access_type == MCD_TX_AT_R {
            if (address..address + data.len() as u64).contains(&BAD_ADDRESS) {
                return fail(
                    MCD_ERR_TXLIST_READ,
                    "Read failure for a transaction of the txlist",
                );
            }
            with(|s| {
                for (i, byte) in data.iter_mut().enumerate() {
                    *byte = *s.memory.get(&(address + i as u64)).unwrap_or(&0);
                }
            });
        } else {
            with(|s| {
                for (i, byte) in data.iter().enumerate() {
                    s.memory.insert(address + i as u64, *byte);
                }
            });
        }
        tx.num_bytes_ok = tx.num_bytes;
        txlist.num_tx_ok += 1;
    }
    0
}

unsafe extern "C" fn qry_rst_classes(
    _core: *const mcd_core_st,
    classes: *mut u32,
) -> mcd_return_et {
    unsafe { *classes = 0x7 };
    0
}

unsafe extern "C" fn rst(_core: *const mcd_core_st, class: u32, halt: mcd_bool_t) -> mcd_return_et {
    // `mcd_bool_t` is `u8` on Linux and macOS.
    #[allow(clippy::useless_conversion)]
    let halt = u32::from(halt);
    with(|s| s.resets.push((class, halt)));
    0
}

const FUNCTIONS: McdFunctions = McdFunctions {
    initialize,
    exit,
    open_server,
    close_server,
    qry_systems,
    qry_devices,
    qry_cores,
    open_core,
    close_core,
    qry_mem_spaces: Some(qry_mem_spaces),
    qry_error_info,
    qry_max_payload_size,
    execute_txlist,
    qry_rst_classes,
    rst,
};

/// Sets up a fresh fake library and a probe for it, with `config` as in a selector.
fn probe(config: &str) -> (MutexGuard<'static, ()>, McdProbe) {
    probe_with(config, FUNCTIONS)
}

fn probe_with(config: &str, functions: McdFunctions) -> (MutexGuard<'static, ()>, McdProbe) {
    let guard = TEST.lock().unwrap_or_else(|e| e.into_inner());
    *fake() = Some(Fake {
        spaces: vec![(0, "Code and Data", 0)],
        ..Fake::default()
    });
    let endpoint = McdEndpoint::parse(&format!("fake.dll|{config}"), |_| None).unwrap();
    (guard, McdProbe::with_functions(endpoint, functions))
}

#[test]
fn endpoints_are_parsed() {
    let none = |_: &str| None;
    assert_eq!(
        McdEndpoint::parse(r"C:\tools\mcd_tas.dll", none).unwrap(),
        McdEndpoint {
            library: r"C:\tools\mcd_tas.dll".into(),
            config: String::new(),
            target: McdTarget::default(),
        }
    );
    let endpoint = McdEndpoint::parse(
        r"C:\tools\mcd_tas.dll|McdHostName=box McdServerPort=1",
        none,
    )
    .unwrap();
    assert_eq!(endpoint.config, "McdHostName=box McdServerPort=1");
    assert_eq!(
        McdEndpoint::parse(&endpoint.serial(), none).unwrap(),
        endpoint
    );

    assert!(McdEndpoint::parse("", none).is_err());
    let env = |name: &str| match name {
        LIBRARY_ENV => Some("from_env.dll".to_string()),
        CONFIG_ENV => Some("McdHostName=env".to_string()),
        _ => None,
    };
    assert_eq!(
        McdEndpoint::parse("", env).unwrap(),
        McdEndpoint {
            library: "from_env.dll".into(),
            config: "McdHostName=env".into(),
            target: McdTarget::default(),
        }
    );
    // An explicit, even empty, configuration wins over the environment.
    assert_eq!(McdEndpoint::parse("x.dll|", env).unwrap().config, "");
}

#[test]
fn the_probe_keys_are_taken_out_of_the_configuration() {
    let none = |_: &str| None;
    let endpoint = McdEndpoint::parse(
        "x.dll|McdHostName=box proBeRsDevice=TriBoard McdServerPort=1 ProbeRsCore=2",
        none,
    )
    .unwrap();
    assert_eq!(endpoint.config, "McdHostName=box McdServerPort=1");
    assert_eq!(
        endpoint.target,
        McdTarget {
            system: None,
            device: Some(McdPick::Name("TriBoard".into())),
            core: Some(McdPick::Index(2)),
        }
    );
    assert_eq!(
        endpoint.serial(),
        "x.dll|McdHostName=box McdServerPort=1 ProbeRsDevice=TriBoard ProbeRsCore=2"
    );
    assert_eq!(
        McdEndpoint::parse(&endpoint.serial(), none).unwrap(),
        endpoint
    );

    // The keys alone, without case; a key without a value is refused.
    let endpoint = McdEndpoint::parse("x.dll|PROBERSSYSTEM=0", none).unwrap();
    assert_eq!(endpoint.config, "");
    assert_eq!(endpoint.target.system, Some(McdPick::Index(0)));
    assert_eq!(endpoint.serial(), "x.dll|ProbeRsSystem=0");
    let error = McdEndpoint::parse("x.dll|ProbeRsCore=", none).unwrap_err();
    assert!(error.contains("ProbeRsCore"), "{error}");

    // A configuration without them is passed on as written.
    let endpoint = McdEndpoint::parse("x.dll|A=1   B=2", none).unwrap();
    assert_eq!(endpoint.config, "A=1   B=2");
}

#[test]
fn the_factory_only_opens_its_selector() {
    let tas: DebugProbeSelector = "058b:0000".parse().unwrap();
    assert!(matches!(
        McdFactory.open(&tas),
        Err(DebugProbeError::ProbeCouldNotBeCreated(
            ProbeCreationError::NotFound
        ))
    ));
    // The rest of a Windows path survives the selector parser.
    let missing: DebugProbeSelector = r"058b:0001:C:\no\such\mcd.dll".parse().unwrap();
    assert_eq!(
        missing.serial_number.as_deref(),
        Some(r"C:\no\such\mcd.dll")
    );
    let error = McdFactory.open(&missing).unwrap_err().to_string();
    assert!(error.contains("could not be loaded"), "{error}");
}

#[test]
fn attach_opens_the_first_core_and_moves_memory() {
    let (_guard, mut probe) = probe("McdHostName=localhost");
    assert!(probe.try_as_tricore_bus().is_none());
    probe.attach().unwrap();
    assert_eq!(probe.core_name(), "Core 0: CPU0 TriCore 1.6E");
    assert_eq!(probe.access_hardware(), "DAS JTAG AURDUINO 46YPEADW");
    assert_eq!(probe.jtag_id().unwrap(), JTAG_ID);
    assert_eq!(
        probe.limits(),
        BusLimits {
            max_transfers: 64,
            max_block_size: 1024
        }
    );

    // An unaligned range larger than one list (4 KiB payload).
    let data = (0..9000).map(|i| (i * 7) as u8).collect::<Vec<_>>();
    probe
        .write_block(AddressMap::Target, 0x7000_0003, &data)
        .unwrap();
    let mut back = vec![0; data.len()];
    probe
        .read_block(AddressMap::Target, 0x7000_0003, &mut back)
        .unwrap();
    assert_eq!(back, data);
    assert!(with(|s| s.lists.len()) > 2);

    probe.write_u32(AddressMap::Target, 0xF881_FD00, 6).unwrap();
    assert_eq!(probe.read_u32(AddressMap::Target, 0xF881_FD00).unwrap(), 6);

    probe.detach().unwrap();
    assert!(probe.try_as_tricore_bus().is_none());
    assert_eq!(
        with(|s| s.calls.clone()),
        [
            "initialize",
            "open_server McdHostName=localhost",
            "open_core TC27x Core 0: CPU0 TriCore 1.6E",
            "close_core",
            "close_server",
            "exit"
        ]
    );
}

#[test]
fn a_failed_read_stops_the_batch() {
    let (_guard, mut probe) = probe("");
    probe.attach().unwrap();
    // An empty string, not NULL: the Infineon library reads the string.
    assert_eq!(with(|s| s.calls[1].clone()), "open_server ");

    let mut transfers = [
        BusTransfer::read(AddressMap::Target, 0xA021_BFF8, 8),
        BusTransfer::read(AddressMap::Target, BAD_ADDRESS as u32, 32),
        BusTransfer::read(AddressMap::Target, 0xA021_C020, 32),
    ];
    probe.execute(&mut transfers).unwrap();
    assert_eq!(transfers[0].status, Some(TransferStatus::Ok));
    assert_eq!(
        transfers[1].status,
        Some(TransferStatus::Failed(STATUS_READ_FAILED))
    );
    assert_eq!(transfers[2].status, None);
    assert!(matches!(
        transfers[1].check(),
        Err(TriCoreError::TransferFailed {
            code: STATUS_READ_FAILED,
            ..
        })
    ));
}

#[test]
fn other_address_maps_are_refused() {
    let (_guard, mut probe) = probe("");
    probe.attach().unwrap();
    let mut transfers = [
        BusTransfer::read(AddressMap::Target, 0x7000_0000, 4),
        BusTransfer::read(AddressMap::Server, 0x5000_0090, 4),
        BusTransfer::read(AddressMap::Target, 0x7000_0000, 4),
    ];
    probe.execute(&mut transfers).unwrap();
    assert_eq!(transfers[0].status, Some(TransferStatus::Ok));
    assert_eq!(
        transfers[1].status,
        Some(TransferStatus::Failed(STATUS_MAP_NOT_AVAILABLE))
    );
    assert_eq!(transfers[2].status, None);
}

#[test]
fn resets_halt_with_the_system_reset_class() {
    let (_guard, mut probe) = probe("");
    probe.target_reset_assert().unwrap();
    probe.attach().unwrap();
    probe.reset(ResetKind::ResetAndHalt).unwrap();
    assert_eq!(with(|s| s.resets.clone()), [(0x2, 1), (0x2, 1)]);
}

#[test]
fn a_dropped_probe_closes_the_library() {
    let (_guard, mut probe) = probe("");
    probe.attach().unwrap();
    drop(probe);
    assert!(!with(|s| s.open));
    assert_eq!(with(|s| s.calls.last().cloned()).as_deref(), Some("exit"));
}

#[test]
fn the_configuration_picks_the_device_and_the_core() {
    let (_guard, mut probe) = probe("ProbeRsDevice=triboard ProbeRsCore=1");
    probe.attach().unwrap();
    assert_eq!(probe.core_name(), "Core 1: CPU1 TriCore 1.6P");
    assert_eq!(probe.access_hardware(), "DAS JTAG TriBoard 0815");
    assert_eq!(probe.jtag_id().unwrap(), 0x1020_6083);
    // The library does not see the probe's keys.
    assert_eq!(
        with(|s| s.calls[1..3].to_vec()),
        ["open_server ", "open_core TC36x Core 1: CPU1 TriCore 1.6P"]
    );
}

#[test]
fn a_missing_core_names_the_cores_there_are() {
    let (_guard, mut probe) = probe("ProbeRsCore=CPU7");
    let error = messages(&probe.attach().unwrap_err());
    assert!(error.contains("no core 'CPU7'"), "{error}");
    assert!(
        error.contains("0 = Core 0: CPU0 TriCore 1.6E, 1 = Core 1: CPU1 TriCore 1.6P"),
        "{error}"
    );
    // The failed attach closed the library again.
    assert!(!with(|s| s.open));
    assert_eq!(with(|s| s.calls.last().cloned()).as_deref(), Some("exit"));
}

#[test]
fn a_device_index_past_the_end_is_refused() {
    let (_guard, mut probe) = probe("ProbeRsDevice=2");
    let error = messages(&probe.attach().unwrap_err());
    assert!(error.contains("no device '2'"), "{error}");
    assert!(
        error.contains("1 = TC36x / DAS JTAG TriBoard 0815"),
        "{error}"
    );
}

/// The messages of an error and its sources.
fn messages(error: &dyn std::error::Error) -> String {
    let mut text = error.to_string();
    let mut source = error.source();
    while let Some(error) = source {
        text = format!("{text}: {error}");
        source = error.source();
    }
    text
}

#[test]
fn transactions_use_the_memory_space_of_the_device_memory() {
    let (_guard, mut probe) = probe("");
    with(|s| s.spaces = vec![(1, "Registers", 0x1), (4, "Cache", 0x8), (7, "AM0", 0x10)]);
    probe.attach().unwrap();
    probe.write_u32(AddressMap::Target, 0x7000_0000, 5).unwrap();
    assert_eq!(probe.read_u32(AddressMap::Target, 0x7000_0000).unwrap(), 5);
}

#[test]
fn a_library_without_memory_spaces_gets_space_0() {
    let functions = McdFunctions {
        qry_mem_spaces: None,
        ..FUNCTIONS
    };
    let (_guard, mut probe) = probe_with("", functions);
    with(|s| s.spaces.clear());
    probe.attach().unwrap();
    assert_eq!(probe.read_u32(AddressMap::Target, 0x7000_0000).unwrap(), 0);
}

#[test]
fn the_inventory_lists_every_core_and_closes_again() {
    let (_guard, mut probe) = probe("ProbeRsCore=9");
    let entries = probe.inventory().unwrap();
    assert_eq!(entries.len(), 5);
    assert_eq!(entries[0].indices, (0, 0, 0));
    assert_eq!(entries[4].indices, (0, 1, 1));
    assert_eq!(entries[4].core, "Core 1: CPU1 TriCore 1.6P");
    assert_eq!(entries[4].device, "TC36x");
    assert_eq!(entries[4].access_hardware, "DAS JTAG TriBoard 0815");
    assert_eq!(entries[4].jtag_id, 0x1020_6083);
    assert_eq!(
        entries[0].memory_spaces,
        Ok(vec![McdMemorySpace {
            id: 0,
            name: "Code and Data".into(),
            mem_type: 0,
            device_memory: true,
        }])
    );
    assert!(!with(|s| s.open));
    assert_eq!(with(|s| s.calls.last().cloned()).as_deref(), Some("exit"));
    // The probe still attaches afterwards (and the bad pick of its own target shows then).
    assert!(probe.attach().is_err());
}
