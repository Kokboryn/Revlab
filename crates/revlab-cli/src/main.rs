mod scenario;
mod keyboard;

use std::f64::consts::PI;
use revlab_core::{SimDuration, SimTime};
use revlab_kernel::{Kernel, Port};
use revlab_kernel::plant::engine::{Engine, EngineBuilder, EnginePorts};
use revlab_kernel::plant::{friction::ChenFlynn, fuel::Fuel, geometry::Geometry};
use revlab_kernel::sensors::{crank_wheel::CrankWheel};
use revlab_kernel::ecu::{Ecu, EcuPorts, Rate, idle::IdleTask};
use revlab_kernel::ecu::torque::{TorqueArbiter, TorqueToFuel};
use revlab_kernel::telemetry::CsvLogger;
use revlab_kernel::ecu::diag::{SpeedPlausibility, LimpMode};
use revlab_kernel::ecu::observer::SpeedObserver;
use revlab_kernel::sensors::cam_wheel::CamWheel;
use scenario::{Scenario, Event, parse_args};
use revlab_kernel::plant::{environment::Environment, intake::IntakeManifold};
use revlab_kernel::plant::load::LoadProfile;
use revlab_kernel::plant::turbo::{Turbo, TurboPorts};
use revlab_kernel::plant::exhaust::{ExhaustManifold, ExhaustPorts};
use revlab_kernel::sensors::map_maf::AnalogSensor;
use revlab_kernel::ecu::airpath::{AirEstimator, SmokeLimiter};
use revlab_kernel::ecu::driver::DriverDemand;
use revlab_kernel::ecu::loss::{LossModel, WarmupComp};
use revlab_kernel::ecu::limits::RevLimiter;
use revlab_kernel::pacer::Pacer;
use revlab_kernel::plant::clutch::{Clutch, ClutchPorts};
use revlab_kernel::plant::intake::IntakePorts;
use revlab_kernel::plant::thermal::ThermalSystem;
use revlab_kernel::plant::road_load::{RoadLoad, RoadLoadPar, RoadLoadPorts};
use revlab_kernel::plant::driveline::{Driveline, DrivelinePorts};

struct RawGuard;
impl Drop for RawGuard {
    fn drop(&mut self) { let _ = keyboard::Keyboard::leave_raw(); }
}

const IDLE_RPM: f64 = 800.0;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = parse_args()?;
    let sc = Scenario::by_name(&args.scenario).ok_or_else(|| format!("unknown scenario '{}'; try --list", args.scenario))?;
    eprintln!("scenario {} - {}", sc.name, sc.about);
    // Rolling start. The clutch owns the input shaft, so one initial value sets both engine side and
    // vehicle side conditions consistently.
    const R_WHEEL: f64 = 0.314;
    const GEAR_RATIOS: [f64; 7] = [13.633, 7.777, 5.252, 4.011, 3.067, 2.413, 1.957];

    let start_gear = sc.events.iter()
        .filter_map(|e| match e { Event::Gear { at_s, gear } if *at_s <= 0.0 => Some(*gear as usize), _ => None })
        .last()
        .unwrap_or(0);

    let omega_in_init = if start_gear >= 1 && start_gear <= 7 {
        sc.start_kmh / 3.6 / R_WHEEL * GEAR_RATIOS[start_gear - 1]
    } else { 0.0 };

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
    let p_im_s: Port        = k.bus.alloc(101_325.0);
    let t_im_s: Port        = k.bus.alloc(293.15);
    let q_lim: Port         = k.bus.alloc(0.0);
    let m_air_est: Port     = k.bus.alloc(0.0);
    let t_loss: Port        = k.bus.alloc(0.0);
    let t_ind_req: Port     = k.bus.alloc(0.0);
    let pedal: Port         = k.bus.alloc(0.0);
    let crank_valid: Port   = k.bus.alloc(1.0);
    let cam_valid: Port     = k.bus.alloc(1.0);
    let m_maf_s: Port       = k.bus.alloc(0.0);
    let n_tc_s: Port        = k.bus.alloc(0.0);
    let t_em_s: Port        = k.bus.alloc(500.0);
    let p_amb_s: Port       = k.bus.alloc(101_325.0);
    let t_cool: Port        = k.bus.alloc(293.15);
    let t_oil: Port         = k.bus.alloc(293.15);
    let visc_mult: Port     = k.bus.alloc(3.2);
    let t_cool_s: Port      = k.bus.alloc(293.15);
    let freeze: Port        = k.bus.alloc(0.0);
    let t_bias: Port        = k.bus.alloc(0.0);
    let gear: Port          = k.bus.alloc(0.0);     // 0 = neutral; no scenario shifts yet
    let f_road: Port        = k.bus.alloc(0.0);
    let j_ref: Port         = k.bus.alloc(0.0);
    let v_veh: Port         = k.bus.alloc(0.0);
    let n_wheel: Port       = k.bus.alloc(0.0);
    let grade: Port         = k.bus.alloc(0.0);
    let brake: Port         = k.bus.alloc(0.0);
    let headwind: Port      = k.bus.alloc(0.0);
    let eta_ind: Port       = k.bus.alloc(0.40);
    let q_coolant: Port     = k.bus.alloc(0.0);
    let q_fric: Port        = k.bus.alloc(0.0);
    let speed_source: Port  = k.bus.alloc(0.0);
    let t_clutch: Port      = k.bus.alloc(0.0);
    let slip: Port          = k.bus.alloc(0.0);
    let q_clutch: Port      = k.bus.alloc(0.0);
    let clutch_cmd: Port    = k.bus.alloc(0.0);
    let omega_in: Port      = k.bus.alloc(0.0);
    let t_out: Port         = k.bus.alloc(0.0);
    let t_disc: Port        = k.bus.alloc(293.15);
    let wear_um: Port     = k.bus.alloc(0.0);
    let glaze: Port         = k.bus.alloc(0.0);

    let geom = Geometry::ea288_16tdi();
    eprintln!("displacement {:.0} cc    inertia {:.4} kg·m²", geom.displacement() * 1e6, geom.inertia_est());
    eprintln!("friction at idle {:.1} Nm", ChenFlynn::DI_DIESEL.torque(&geom, 800.0, 140e5));

    // Thermocouple in the exhaust stream: ~2 s time constant. That lag is real and is why EGT protection
    // acts on a model, not the sensor.
    k.add(Box::new(AnalogSensor::new(t_em, t_em_s, 2.000, 3.0, 1300.0, 500.0)));

    k.add(Box::new(AnalogSensor::new(n_tc, n_tc_s, 0.10, 200.0, 300_000.0, 0.0)));
    k.add(Box::new(AnalogSensor::new(p_amb, p_amb_s, 0.200, 100.0, 120_000.0, 101_325.0)));
    k.add(Box::new(AnalogSensor::new(t_cool, t_cool_s, 1.000, 0.3, 400.0, 293.15)));
    
    k.add(Box::new(ThermalSystem::ea288(q_coolant, q_fric, t_amb, t_cool, t_oil, visc_mult, 293.15)));

    k.add(Box::new(Environment::standard(p_amb, t_amb)));
    k.add(Box::new(Turbo::vnt_small_diesel(TurboPorts {
        p_amb, t_amb, p_im, p_em, t_em, vnt_cmd: vnt, m_comp, t_comp: t_charge, m_turb, n_tc,
    })));
    k.add(Box::new(IntakeManifold::new(0.0025, IntakePorts {
        t_up: t_charge, m_dot_eng: m_dot_air, m_comp_in: m_comp, p: p_im, t: t_im, m_dot_in: m_dot_maf,
    }, 101_325.0, 293.15)));

    k.add(Box::new(ExhaustManifold::new(0.0015, ExhaustPorts {
        m_air: m_dot_air, m_fuel, t_im, m_turb, p: p_em, t: t_em, eta_ind, q_coolant
    }, 101_325.0, 500.0)));

    // Hotwire MAF: fast element, but noisy and prone to error during fast flow changes - which is why
    // real ECUs blend it with speed-density rather than trusting it alone
    k.add(Box::new(AnalogSensor::new(m_dot_maf, m_maf_s, 0.020, 0.4e-3, 0.5, 0.0)));
    
    k.add(Box::new(Clutch::dq200_k1(ClutchPorts { omega_eng: omega, cmd: clutch_cmd, t_out, j_ref,
        omega_in, t_clutch, slip, q_clutch, v_veh, t_disc, t_amb, wear_um, glaze
    }, omega_in_init, 293.15)));

    let par = EngineBuilder::new(geom, Fuel::DIESEL_B7)
        .build();
    k.add(Box::new(Engine::new(par, EnginePorts { q_cmd, p_im, t_im, t_load, t_clutch, visc_mult,
        omega, theta, m_dot_air, afr, m_fuel, eta_ind, q_fric,
    }, IDLE_RPM)));

    let mut load_steps: Vec<(SimTime, f64)> = Vec::new();
    let mut speed_steps: Vec<(SimTime, f64)> = vec![(SimTime::ZERO, IDLE_RPM)];
    let mut pedal_steps: Vec<(SimTime, f64)> = vec![(SimTime::ZERO, 0.0)];
    let mut crank = CrankWheel::new(omega, n_meas, crank_valid,);
    let mut cam = CamWheel::new(omega, n_cam, cam_valid,);
    let mut gear_steps: Vec<(SimTime, f64)> = vec![(SimTime::ZERO, 0.0)];
    let mut clutch_steps: Vec<(SimTime, f64)> = vec![(SimTime::ZERO, 0.0)];
    let mut grade_steps: Vec<(SimTime, f64)> = vec![(SimTime::ZERO, 0.0)];
    let mut brake_steps: Vec<(SimTime, f64)> = vec![(SimTime::ZERO, 0.0)];
    for e in &sc.events {
        let at = |s: f64| SimTime::ZERO + SimDuration::from_millis((s * 1000.0) as u64);
        match *e {
            Event::CrankFault { at_s, fault }   => crank = crank.arm_fault(at(at_s), fault),
            Event::CamFault { at_s, fault }     => cam = cam.arm_fault(at(at_s), fault),
            Event::Load { at_s, torque }         => load_steps.push((at(at_s), torque)),
            Event::Speed { at_s, rpm }           => speed_steps.push((at(at_s), rpm)),
            Event::Pedal { at_s, position }      => pedal_steps.push((at(at_s), position)),
            Event::Gear { at_s, gear: g  }       => gear_steps.push((at(at_s), g)),
            Event::Clutch {at_s, cmd }           => clutch_steps.push((at(at_s), cmd)),
            Event::Grade { at_s, rad }           => grade_steps.push((at(at_s), rad)),
            Event::Brake { at_s, cmd }           => brake_steps.push((at(at_s), cmd)),
        }
    }

    let quit = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
    let _raw = if args.live {
        keyboard::Keyboard::enter_raw()?;
        Some(RawGuard)
    } else { None };

    if args.live {
        k.add(Box::new(Pacer::new(1.0)));
        k.add(Box::new(keyboard::Keyboard::new(pedal, t_load, quit.clone())));
    } else {
        if let Some(sp) = args.speed { k.add(Box::new(Pacer::new(sp))); }
        k.add(Box::new(LoadProfile::new(load_steps, t_load)));
        k.add(Box::new(LoadProfile::new(pedal_steps, pedal)));
    }
    k.add(Box::new(LoadProfile::new(speed_steps, speed_req)));
    k.add(Box::new(crank));
    k.add(Box::new(cam));

    k.add(Box::new(AnalogSensor::new(p_im, p_im_s, 0.005, 300.0, 300_000.0, 101_325.0)));
    k.add(Box::new(AnalogSensor::new(t_im, t_im_s, 0.500, 0.5, 400.0, 293.15)));

    k.add(Box::new(RoadLoad::new(RoadLoadPar::passat_b8_16tdi(), RoadLoadPorts {
        v_veh, grade, headwind, brake, p_amb, t_amb, f_road,
    })));
    k.add(Box::new(Driveline::dq200_passat(RoadLoadPar::passat_b8_16tdi(), DrivelinePorts {
        omega_in, gear, f_road, v_veh, n_wheel, t_out, j_ref,
    })));
    k.add(Box::new(
        Ecu::new(EcuPorts {
            n_crank: n_meas,
            n_cam,
            speed_req,
            p_im: p_im_s,
            t_im: t_im_s,
            m_maf: m_maf_s,
            q_cmd,
            in_pedal: pedal,
            t_arb,
            dtc,
            n_model,
            q_lim,
            m_air_est,
            t_loss,
            t_ind_req,
            cam_valid,
            crank_valid,
            t_ect_c: t_cool_s,
            freeze,
            t_bias,
            speed_source
        }, 6.0)
            .task(Rate::Ms10, Box::new(WarmupComp::di_diesel_1_6()))
            .task(Rate::Ms10, Box::new(SpeedObserver::di_diesel_1_6()))
            .task(Rate::Ms10, Box::new(SpeedPlausibility::default()))
            .task(Rate::Ms10, Box::new(LimpMode { torque_max: 40.0 }))
            .task(Rate::Ms10, Box::new(DriverDemand::di_diesel_1_6()))
            .task(Rate::Ms10, Box::new(IdleTask::new(0.0)))
            .task(Rate::Ms10, Box::new(AirEstimator::di_diesel_1_6()))
            .task(Rate::Ms10, Box::new(SmokeLimiter::di_diesel_1_6()))
            .task(Rate::Ms10, Box::new(RevLimiter::ea288()))
            .task(Rate::Ms10, Box::new(TorqueArbiter))
            .task(Rate::Ms10, Box::new(LossModel::di_diesel_1_6()))
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
             ("n_tc".into(), n_tc),
             ("q_lim".into(), q_lim),
             ("m_air_est".into(), m_air_est), 
             ("t_loss".into(), t_loss),
             ("t_ind_req".into(), t_ind_req),
             ("pedal".into(), pedal),
             ("m_maf_s".into(), m_maf_s),
             ("t_cool".into(), t_cool),
             ("t_ect".into(), t_cool_s),
             ("t_oil".into(), t_oil),
             ("visc_mult".into(), visc_mult),
             ("freeze".into(), freeze),
             ("crank_valid".into(), crank_valid),
             ("cam_valid".into(), cam_valid),
             ("t_bias".into(), t_bias),
             ("gear".into(), gear),
             ("v_veh".into(), v_veh),
             ("n_wheel".into(), n_wheel),
             ("f_road".into(), f_road),
             ("eta_ind".into(), eta_ind),
             ("speed_source".into(), speed_source),
             ("t_out".into(), t_out),
             ("j_ref".into(), j_ref),
             ("omega_in".into(), omega_in),
             ("slip".into(), slip),
             ("t_clutch".into(), t_clutch),
             ("q_clutch".into(), q_clutch),
             ("clutch_cmd".into(), clutch_cmd),
             ("t_disc".into(), t_disc),
             ("wear_um".into(), wear_um),
             ("glaze".into(), glaze),],
        SimDuration::from_millis(10),
    )?));
    
    // Last on purpose: inserting a component earlier shifts every later component's index, which reorders
    // same timestamp RNG draws and changes the noise realization in every scenario.
    k.add(Box::new(LoadProfile::new(gear_steps, gear)));
    k.add(Box::new(LoadProfile::new(clutch_steps, clutch_cmd)
        .ramped(SimDuration::from_millis(1500))));
    k.add(Box::new(LoadProfile::new(grade_steps, grade)));
    k.add(Box::new(LoadProfile::new(brake_steps, brake)));

    let end = SimTime::ZERO + SimDuration::from_millis(sc.duration_s * 1000);
    let chunk = SimDuration::from_millis(if args.live { 100 } else { 1000 });
    let wall = std::time::Instant::now();
    let mut t = SimTime::ZERO;

    if args.live {
        eprintln!(" up/down = pedal     left/right = load   space = lift    q = quit\r")
    }

    while args.live || t < end {
        t = if args.live { t + chunk } else { std::cmp::min(t + chunk, end) };
        k.run_until(t);

        if args.live {
            let rpm = k.bus.get(omega) * 60.0 / (2.0 * PI);
            let boost = (k.bus.get(p_im) - 101_325.0) / 1e5;
            eprint!("\r {:5.0} rpm | {:+5.2} bar | AFR {:5.1} | EGT {:4.0} c | pedal {:3.0}% | load {:3.0} Nm ",
            rpm, boost, k.bus.get(afr).min(999.0), k.bus.get(t_em) - 273.15, k.bus.get(pedal) * 100.0, k.bus.get(t_load));
            if quit.load(std::sync::atomic::Ordering::Relaxed) { break; }
        } else {
            eprint!("\r {:.0}/{} s simulated ({:.0}x real time)     ",
                    t.as_secs_f64(), sc.duration_s,
                    t.as_secs_f64() / wall.elapsed().as_secs_f64().max(1e-9));
        }
    }
    eprintln!();

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