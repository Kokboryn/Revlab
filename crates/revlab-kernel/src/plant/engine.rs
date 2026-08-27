use std::f64::consts::PI;
use revlab_core::{SimDuration};
use crate::{Component, Ctx, Port, Trigger};
use super::{efficiency::Efficiency, friction::ChenFlynn, fuel::Fuel, geometry::Geometry};
use super::gas::R_AIR;

/// Flat, resolved parameters. The solver never does lookups or unit conversion in the loop - everything is resolved once at build time.
pub struct EnginePar {
    pub inertia: f64,
    pub cylinders: f64,
    pub lhv: f64,
    pub geom: Geometry,
    pub fric: ChenFlynn,
    pub eta: Box<dyn Efficiency>,
    pub stall_rad_s: f64,
    pub p_max_nom: f64,     // Pa. nominal peak cylinder pressure
    pub q_max: f64,         // mg/stroke, physical injector limit
}

pub struct EngineBuilder {
    geom: Geometry,
    fuel: Fuel,
    fric: ChenFlynn,
    eta: Box<dyn Efficiency>,
}

impl EngineBuilder {
    pub fn new(geom: Geometry, fuel: Fuel) -> Self {
        EngineBuilder {
            geom, fuel, fric: ChenFlynn::DI_DIESEL, eta: Box::new(super::efficiency::MapEta::di_diesel_typical()),
        }
    }

    pub fn efficiency(mut self, e: Box<dyn Efficiency>) -> Self {
        self.eta = e; self
    }

    pub fn build(self) -> EnginePar {
        EnginePar {
            inertia: self.geom.inertia_est(),
            cylinders: self.geom.cylinders as f64,
            lhv: self.fuel.lhv,
            geom: self.geom,
            fric: self.fric,
            eta: self.eta,
            stall_rad_s: 30.0,
            p_max_nom: 140e5,
            q_max: 60.0,
        }
    }
}


/// Every port the engine touches, named. Construction is by field name, so the ordering mistakes that
/// plague a 12-argument constructor become impossible. Same reasoning as EcuPorts, applied per subsystem;
/// the plant components are independent, so a driveline should add its own struct rather than widen one
/// everybody shares.
#[derive(Copy, Clone)]
pub struct EnginePorts {
    // inputs
    pub q_cmd: Port,
    pub p_im: Port,
    pub t_im: Port,
    pub t_load: Port,       // scripted external load, from LoadProfile
    pub t_clutch: Port,        // vehicle inertia reflected through the gearing
    pub visc_mult: Port,
    // outputs
    pub omega: Port,
    pub theta: Port,
    pub m_dot_air: Port,
    pub afr: Port,
    pub m_fuel: Port,
    pub eta_ind: Port,
    pub q_fric: Port,       // W, friction work becoming heat in the oil
}

pub struct Engine {
    omega: f64,     // rad/s
    theta: f64,     // rad, wrapped
    running: bool,
    p: EnginePar,
    ports: EnginePorts,
    dt: f64,
}

impl Engine {
    pub const STEP: SimDuration = SimDuration::from_millis(1);

    pub fn new(p: EnginePar, ports: EnginePorts, idle_rpm: f64) -> Self {
        Engine {
            omega: idle_rpm * 2.0 * PI / 60.0,
            theta: 0.0,
            running: true,
            p, ports,
            dt: Self::STEP.as_secs_f64(),
        }
    }

    pub fn rpm(&self) -> f64 { self.omega * 60.0 / (2.0 * PI) }

    fn indicated_eta(&self, q_mg: f64) -> f64 {
        let q = q_mg.clamp(0.0, self.p.q_max);
        self.p.eta.eta(self.rpm(), q / self.p.q_max)
    }
    /// T_ind = q · n_cyl · LHV · n_i / 4π
    /// The 4π is the four stroke cycle: one burn per two revolutions
    fn indicated_torque(&self, q_mg: f64) -> f64 {
        if !self.running { return 0.0; }
        let q = q_mg.clamp(0.0, self.p.q_max);
        let m_kg = q * 1e-6 * self.p.cylinders;
        m_kg * self.p.lhv * self.indicated_eta(q_mg) / (4.0 * PI)
    }

    fn friction_torque(&self) -> f64 {
        self.p.fric.torque(&self.p.geom, self.rpm().max(0.0), self.p.p_max_nom)
    }

    /// Speed-density: ṁ = η_vol · V_d · N · p / (120 · R · T). The 120 is 2 revolutions per cycle x 60 s/min
    fn air_flow(&self, p_im: f64, t_im: f64) -> f64 {
        let eta_vol = 0.90; // TODO: map over (N, p_im) Diesels sit high and flat - no throttling loss
        eta_vol * self.p.geom.displacement() * self.rpm() * p_im / (120.0 * R_AIR * t_im)
    }
}

impl Component for Engine {
    fn triggers(&self) -> Vec<Trigger> {
        vec![Trigger::Periodic { period: Self::STEP, offset: SimDuration::ZERO }]
    }

    fn step(&mut self, _trig: u16, ctx: &mut Ctx<'_>) {
        let q = ctx.bus.get(self.ports.q_cmd);
        let t_load = ctx.bus.get(self.ports.t_load) + ctx.bus.get(self.ports.t_clutch);
        let visc = ctx.bus.get(self.ports.visc_mult).max(1.0);
        let t_net = self.indicated_torque(q) - self.friction_torque() * visc - t_load;

        // The clutch owns the input shaft, so the vehicle is no longer part of this inertia. Engine
        // speed is its own degree of freedom again -- which is what lets it stall against a closed
        // clutch, and rev freely against an open one.
        self.omega += t_net / self.p.inertia * self.dt;
        if self.omega < self.p.stall_rad_s {
            self.running = false;
            self.omega = self.omega.max(0.0);
        }
        self.theta = (self.theta + self.omega * self.dt) % (2.0 * PI);

        ctx.bus.set(self.ports.omega, self.omega);
        ctx.bus.set(self.ports.theta, self.theta);
        let _ = ctx.now;
        let p_im = ctx.bus.get(self.ports.p_im);
        let t_im = ctx.bus.get(self.ports.t_im);
        let m_air = self.air_flow(p_im, t_im).max(0.0);
        // fuel mass flow: q per stroke x cylinders x cycles per second
        let m_fuel = q.clamp(0.0, self.p.q_max) * 1e-6 * self.p.cylinders * self.rpm() / 120.0;
        ctx.bus.set(self.ports.m_dot_air, m_air);
        ctx.bus.set(self.ports.afr, if m_fuel > 1e-9 { m_air / m_fuel } else { 999.0 });
        ctx.bus.set(self.ports.m_fuel, m_fuel);
        ctx.bus.set(self.ports.eta_ind, self.indicated_eta(q));
        ctx.bus.set(self.ports.q_fric, self.friction_torque() * visc * self.omega);
    }
}