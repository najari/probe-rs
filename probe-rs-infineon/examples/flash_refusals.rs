//! PRD test T-12: the flash sequence refuses invalid requests before the loader runs a flash
//! command. It starts the loader, asks for an unaligned erase, an erase and a program outside
//! the PFLASH, an unaligned erase in the DFLASH, erases after the DFLASH and in a UCB, and
//! stops the loader again. No flash is changed.
//!
//! ```text
//! cargo run -p probe-rs-infineon --example flash_refusals
//! ```
//!
//! The chip (TC275 or TC36x) is detected.

use probe_rs::{Permissions, flashing::DebugFlashSequence, probe::list::Lister};
use probe_rs_infineon::flash::{AurixFlash, FlashDevice};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    probe_rs_infineon::register_plugin();
    let probe = Lister::new().open("058b:0000".parse::<probe_rs::probe::DebugProbeSelector>()?)?;
    let mut session = probe.attach(
        probe_rs::config::TargetSelector::Auto,
        Permissions::default(),
    )?;
    let device =
        FlashDevice::for_chip(&session.target().name).ok_or("no flash loader for this chip")?;
    let flash = AurixFlash::new(device);
    let mut failed = 0;

    let sectors = device.sectors();
    let inside = sectors[1].base as u64 + 0x10;
    let last = sectors.last().unwrap();
    let after = (last.base + last.size) as u64;

    flash.prepare_flash(&mut session)?;
    let cases: [(String, Result<(), probe_rs::Error>); 6] = [
        (
            format!("erase inside a sector ({inside:#010x})"),
            flash.erase_sector(&mut session, inside),
        ),
        (
            format!("erase after the PFLASH ({after:#010x})"),
            flash.erase_sector(&mut session, after),
        ),
        (
            "erase inside a DFLASH sector (0xAF000010)".to_string(),
            flash.erase_sector(&mut session, 0xAF00_0010),
        ),
        (
            "erase after the DFLASH (0xAF060000)".to_string(),
            flash.erase_sector(&mut session, 0xAF06_0000),
        ),
        (
            "erase in a UCB (0xAF100000)".to_string(),
            flash.erase_sector(&mut session, 0xAF10_0000),
        ),
        (
            format!("program after the PFLASH ({:#010x})", after - 0x2000_0000),
            flash.program(&mut session, after - 0x2000_0000, &[0; 32]),
        ),
    ];
    for (what, result) in cases {
        let refused = result.is_err();
        println!(
            "[{}] T-12 {what}: {}",
            if refused { "PASS" } else { "FAIL" },
            match result {
                Err(e) => format!("refused: {e}"),
                Ok(()) => "accepted".to_string(),
            }
        );
        failed += (!refused) as usize;
    }
    flash.finish_flash(&mut session)?;
    drop(session);

    println!("{failed} check(s) failed");
    if failed > 0 {
        std::process::exit(1);
    }
    Ok(())
}
