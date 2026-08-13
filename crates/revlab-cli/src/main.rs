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
use revlab_kernel::ecu::observer::SpeedObserver;
use revlab_kernel::sensors::cam_wheel::CamWheel;
use scenario::{Scenario, Event, parse_args};
use revlab_kernel::plant::{environment::Environment, intake::IntakeManifold};
use revlab_kernel::plant::load::LoadProfile;
use revlab_kernel::plant::turbo::Turbo;
use revlab_kernel::plant::exhaust::ExhaustManifold;

const IDLE_RPM: f64 = 800.0;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = parse_args()?;
    let sc = Scenario::by_name(&args.scenario).ok_or_else(|| format!("unknown scenario '{}'; try --list", args.scenario))?;
    eprintln!("scenario {} - {}", sc.name, sc.about);

    let mut k = Kernel::new(args.seed);

    // Ports. q_cmd is preloaded so the engine has fuel at t=0 before the governor's first execution at 1 ms
    let q_cmd: Port         = k.bus.alloc(6.0);
    let omega: Port         = k.bus.alloc(IDLE_RPM * 2.0 * PI / 60.0);
    let theta: Port         = k.bus.alloc(0.0);
    let n_meas: Port        = k.bus.alloc(IDLE_RPM);
    let t_arb: Port         = k.bus.alloc(0.0);
    let n_cam: Port         = k.bus.alloc(IDLE_RPM);
    let dtc: Port           = k.bus.alloc(0.0);
    let n_model: Port       = k.bus.alloc(IDLE_RPM);
    let p_amb: Port         = k.bus.alloc(101_325.0);
    let t_amb: Port         = k.bus.alloc(293.15);
    let p_im: Port          = k.bus.alloc(101_325.0);
    let t_im: Port          = k.bus.alloc(293.15);
    let m_dot_air: Port     = k.bus.alloc(0.0);
    let m_dot_maf: Port     = k.bus.alloc(0.0);
    let afr: Port           = k.bus.alloc(999.0);
    let t_load: Port        = k.bus.alloc(0.0);
    let m_comp: Port        = k.bus.alloc(0.0);
    let p_em: Port          = k.bus.alloc(101_325.0);
    let t_em: Port          = k.bus.alloc(500.0);
    let n_tc: Port          = k.bus.alloc(0.0);
    let m_turb: Port        = k.bus.alloc(0.0);
    let m_fuel: Port        = k.bus.alloc(0.0);
    let vnt: Port           = k.bus.alloc(1.0);     // vanes open, no control yet
    let t_charge: Port      = k.bus.alloc(293.15);
    let speed_req: Port     = k.bus.alloc(IDLE_RPM);

    let geom = Geometry::ea288_16tdi();
    eprintln!("displacement {:.0} cc    inertia {:.4} kg·m²", geom.displacement() * 1e6, geom.inertia_est());
    eprintln!("friction at idle {:.1} Nm", ChenFlynn::DI_DIESEL.torque(&geom, 800.0, 140e5));

    k.add(Box::new(Environment::standard(p_amb, t_amb)));
    k.add(Box::new(Turbo::vnt_small_diesel(p_amb, t_amb, p_im, p_em, t_em, vnt, m_comp, t_charge,m_turb, n_tc)));
    k.add(Box::new(IntakeManifold::new(0.0025, t_charge, m_dot_air, m_comp, p_im, t_im, m_dot_maf, 101_325.0, 293.15)));

    k.add(Box::new(ExhaustManifold::new(0.0015, m_dot_air, m_fuel, t_im, m_turb, p_em, t_em, 101_325.0, 500.0)));

    k.add(Box::new(IntakeManifold::new(0.0025, t_amb, m_dot_air, m_comp, p_im, t_im, m_dot_maf, 101_325.0, 293.15)));

    let par = EngineBuilder::new(geom, Fuel::DIESEL_B7)
        .build();
    k.add(Box::new(Engine::new(par, q_cmd, omega, theta, p_im, t_im, m_dot_air, afr, m_fuel, t_load, IDLE_RPM)));

    let mut load_steps: Vec<(SimTime, f64)> = Vec::new();
    let mut speed_steps: Vec<(SimTime, f64)> = vec![(SimTime::ZERO, IDLE_RPM)];
    let mut crank = CrankWheel::new(omega, n_meas);
    let mut cam = CamWheel::new(omega, n_cam);
    for e in &sc.events {
        let at = |s: f64| SimTime::ZERO + SimDuration::from_millis((s * 1000.0) as u64);
        match *e {
            Event::CrankFault { at_s, fault } => crank = crank.arm_fault(at(at_s), fault),
            Event::CamFault { at_s, fault } => cam = cam.arm_fault(at(at_s), fault),
            Event::Load { at_s, torque } => load_steps.push((at(at_s), torque)),
            Event::Speed { at_s, rpm } => speed_steps.push((at(at_s), rpm)),
        }
    }
    k.add(Box::new(LoadProfile::new(load_steps, t_load)));
    k.add(Box::new(LoadProfile::new(speed_steps, speed_req)));
    k.add(Box::new(crank));
    k.add(Box::new(cam));

    k.add(Box::new(
        Ecu::new(n_meas, n_cam, speed_req, q_cmd, t_arb, dtc, n_model, 6.0)
            .task(Rate::Ms10, Box::new(SpeedObserver::di_diesel_1_6()))
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
             ("n_model".into(), n_model),
             ("dtc".into(), dtc),
             ("t_arb".into(), t_arb),
             ("q_cmd".into(), q_cmd),
             ("p_im".into(), p_im),
             ("m_air".into(), m_dot_air),
             ("afr".into(), afr),
             ("t_load".into(), t_load),
             ("p_em".into(), p_em),
             ("t_em".into(), t_em),
             ("n_tc".into(), n_tc)],
        SimDuration::from_millis(10),
    )?));

    k.run_until(SimTime::ZERO + SimDuration::from_millis(sc.duration_s * 1000));
    drop(k);
    eprintln!("done, {} s simulated -> {}", sc.duration_s, args.out);

    if args.plot {
        let title = format!("Revlab - {} (seed {})", sc.name, args.seed);
        match std::process::Command::new("python")
            .args(["tools/plot.py", &args.out, "--title", &title])
            .status()
        {
            Ok(s) if s.success() => {}
            Ok(_) => eprintln!("plot failed (see traceback above)"),
            Err(e) => eprintln!("could not run tools/plot.py: {e}"),
        }
    }
    Ok(())
}