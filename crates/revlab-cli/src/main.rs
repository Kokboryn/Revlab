mod scenario;

use std::f64::consts::PI;
use revlab_core::{SimDuration, SimTime};
use revlab_kernel::{Kernel, Port};
use revlab_kernel::plant::engine::{Engine, EngineBuilder};
use revlab_kernel::plant::{friction::ChenFlynn, fuel::Fuel, geometry::Geometry};
use revlab_kernel::sensors::{crank_wheel::CrankWheel};
use revlab_kernel::ecu::{Ecu, Rate, idle::IdleTask};
use revlab_kernel::ecu::torque::{TorqueArbiter, TorqueToFuel};
use revlab_kernel::telemetry::CsvLogger;
use revlab_kernel::ecu::diag::{SpeedPlausibility, LimpMode};
use revlab_kernel::sensors::cam_wheel::CamWheel;
use scenario::{Scenario, Event, parse_args};

const IDLE_RPM: f64 = 800.0;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = parse_args()?;
    let sc = Scenario::by_name(&args.scenario).ok_or_else(|| format!("unknown scenario '{}'; try --list", args.scenario))?;
    eprintln!("scenario {} - {}", sc.name, sc.about);

    let mut k = Kernel::new(args.seed);

    // Ports. q_cmd is preloaded so the engine has fuel at t=0 before the governor's first execution at 1 ms
    let q_cmd: Port     = k.bus.alloc(6.0);
    let omega: Port     = k.bus.alloc(IDLE_RPM * 2.0 * PI / 60.0);
    let theta: Port     = k.bus.alloc(0.0);
    let n_meas: Port     = k.bus.alloc(IDLE_RPM);
    let t_arb: Port     = k.bus.alloc(0.0);
    let n_cam: Port     = k.bus.alloc(IDLE_RPM);
    let dtc: Port = k.bus.alloc(0.0);

    let geom = Geometry::ea288_16tdi();
    eprintln!("displacement {:.0} cc    inertia {:.4} kg·m²", geom.displacement() * 1e6, geom.inertia_est());
    eprintln!("friction at idle {:.1} Nm", ChenFlynn::DI_DIESEL.torque(&geom, 800.0, 140e5));

    let par = EngineBuilder::new(geom, Fuel::DIESEL_B7)
        .build();
    k.add(Box::new(Engine::new(par, q_cmd, omega, theta, IDLE_RPM)));

    let mut crank = CrankWheel::new(omega, n_meas);
    let mut cam = CamWheel::new(omega, n_cam);
    for e in &sc.events {
        let at = |s: f64| SimTime::ZERO
            + SimDuration::from_millis((s * 1000.0) as u64);
        match *e {
            Event::CrankFault { at_s, fault } => crank = crank.arm_fault(at(at_s), fault),
            Event::CamFault { at_s, fault } => cam = cam.arm_fault(at(at_s), fault),
        }
    }
    k.add(Box::new(crank));
    k.add(Box::new(cam));

    k.add(Box::new(
        Ecu::new(n_meas, n_cam, q_cmd, t_arb,dtc, 6.0)
            .task(Rate::Ms10, Box::new(SpeedPlausibility::default()))
            .task(Rate::Ms10, Box::new(LimpMode { torque_max: 40.0 }))
            .task(Rate::Ms10, Box::new(IdleTask::new(IDLE_RPM, 17.3)))
            .task(Rate::Ms10, Box::new(TorqueArbiter))
            .task(Rate::Ms10, Box::new(TorqueToFuel::di_diesel(4.0)))
    ));
    k.add(Box::new(CsvLogger::new(
        &args.out,
        vec![("omega".into(), omega),
             ("n_crank".into(), n_meas),
             ("n_cam".into(), n_cam),
             ("dtc".into(), dtc),
             ("t_arb".into(), t_arb),
             ("q_cmd".into(), q_cmd)],
        SimDuration::from_millis(10),
    )?));

    k.run_until(SimTime::ZERO + SimDuration::from_millis(sc.duration_s * 1000));
    eprintln!("done, {} s simulated -> {}", sc.duration_s, args.out);
    Ok(())
}