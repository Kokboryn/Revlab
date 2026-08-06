use std::f64::consts::PI;
use revlab_core::{SimDuration, SimTime};
use revlab_kernel::{Kernel, Port};
use revlab_kernel::plant::engine::{Engine, EngineBuilder};
use revlab_kernel::plant::{friction::ChenFlynn, fuel::Fuel, geometry::Geometry};
use revlab_kernel::sensors::{crank_wheel::CrankWheel, Fault};
use revlab_kernel::ecu::idle_governor::IdleGovernor;
use revlab_kernel::telemetry::CsvLogger;

const IDLE_RPM: f64 = 800.0;
const RUN_S: u64 = 20;

fn main() -> std::io::Result<()> {

    let seed = std::env::args().nth(1)
        .and_then(|s| s.parse().ok()).unwrap_or(0xC0FFEE);

    let mut k = Kernel::new(seed);

    // Ports. q_cmd is pre-loaded so the engine has fuel at t=0 before the governor's first execution at 1 ms
    let q_cmd: Port     = k.bus.alloc(6.0);
    let omega: Port     = k.bus.alloc(IDLE_RPM * 2.0 * PI / 60.0);
    let theta: Port     = k.bus.alloc(0.0);
    let n_meas: Port     = k.bus.alloc(IDLE_RPM);

    let geom = Geometry::ea288_16tdi();
    eprintln!("displacement {:.0} cc    inertia {:.4} kg·m²", geom.displacement() * 1e6, geom.inertia_est());
    eprintln!("friction at idle {:.1} Nm", ChenFlynn::DI_DIESEL.torque(&geom, 800.0, 140e5));

    let par = EngineBuilder::new(geom, Fuel::DIESEL_B7)
        .build();
    k.add(Box::new(Engine::new(par, q_cmd, omega, theta, IDLE_RPM)));

    let wheel = CrankWheel::new(omega, n_meas)
        .arm_fault(SimTime::ZERO + SimDuration::from_millis(10_000),
            Fault::Drift { per_sec: 20.0 });
    k.add(Box::new(wheel));

    k.add(Box::new(IdleGovernor::new(n_meas, q_cmd, IDLE_RPM)));

    k.add(Box::new(CsvLogger::new(
        "run.csv",
        vec![("omega".into(), omega),
             ("n_meas".into(), n_meas),
             ("q_cmd".into(), q_cmd)],
        SimDuration::from_millis(10),
    )?));

    k.run_until(SimTime::ZERO + SimDuration::from_millis(RUN_S * 1000));
    eprintln!("done, {} s simulated -> run.csv", RUN_S);
    Ok(())
}