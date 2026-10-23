//! What one status poll of all cores costs (one DAP session for several cores, PRD test T-M13):
//! the status of each core asked on its own, against [`probe_rs::Session::status_many`], which
//! reads the debug status of all cores in one request. The cores are in the halt group as with
//! `allStop`, so each status check also looks at the group. The cores keep running; the group
//! routing is put back when the session ends. Read only otherwise.
//!
//! ```text
//! cargo run -p probe-rs-infineon --example poll_cost
//! ```
//!
//! `PROBE_RS_PROBE` selects another probe.

use std::time::{Duration, Instant};

use probe_rs::{
    CoreStatus, Permissions,
    probe::{DebugProbeSelector, list::Lister},
};

fn median(samples: &mut [Duration]) -> f64 {
    samples.sort_unstable();
    samples[samples.len() / 2].as_secs_f64() * 1000.0
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    probe_rs_infineon::register_plugin();
    let selector: DebugProbeSelector = std::env::var("PROBE_RS_PROBE")
        .unwrap_or_else(|_| "058b:0000".to_string())
        .parse()?;
    let mut session = Lister::new().open(selector)?.attach(
        probe_rs::config::TargetSelector::Auto,
        Permissions::default(),
    )?;
    let cores = (0..session.list_cores().len()).collect::<Vec<_>>();
    println!("{}, cores {cores:?}", session.target().name);
    for &core in &cores {
        session.set_tricore_halt_group(core, true)?;
    }

    // A first round settles the status of each core.
    for &core in &cores {
        session.core(core)?.status()?;
    }

    // `--requests`: one poll each way with the TAS requests logged, to count them.
    if std::env::args().any(|a| a == "--requests") {
        tracing_subscriber::fmt()
            .with_env_filter("probe_rs_infineon::tas::protocol=trace")
            .with_writer(std::io::stderr)
            .init();
        eprintln!("== each core on its own");
        for &core in &cores {
            session.core(core)?.status()?;
        }
        eprintln!("== status_many");
        for status in session.status_many(&cores) {
            status?;
        }
        eprintln!("== end");
        return Ok(());
    }

    const ROUNDS: usize = 50;
    let (mut each, mut batch) = (Vec::new(), Vec::new());
    let mut statuses = Vec::new();
    for _ in 0..ROUNDS {
        let start = Instant::now();
        for &core in &cores {
            session.core(core)?.status()?;
        }
        each.push(start.elapsed());

        let start = Instant::now();
        statuses = session
            .status_many(&cores)
            .into_iter()
            .collect::<Result<Vec<_>, _>>()?;
        batch.push(start.elapsed());
    }
    let each = median(&mut each);
    let batch = median(&mut batch);
    println!(
        "one poll of {} cores: each core on its own {each:.2} ms, status_many {batch:.2} ms ({:.1} times faster)",
        cores.len(),
        each / batch
    );
    println!("statuses: {statuses:?}");
    if statuses.iter().any(|s| *s != CoreStatus::Running) {
        return Err("the cores must run".into());
    }
    Ok(())
}
