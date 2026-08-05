use std::f64::consts::PI;
use revlab_core::{SimDuration, SimTime};
use crate::{Component, Ctx, Port, Trigger};
use super::{efficiency::Efficiency, friction::ChenFlynn, fuel::Fuel, geometry::Geometry};

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
            geom, fuel, fric: ChenFlynn::DI_DIESEL, eta: Box::new(super::efficiency::ConstEta(0.40)),
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

pub struct Engine {
    omega: f64,     // ras/s
    theta: f64,     // rad, wrapped
    running: bool,
    p: EnginePar,
    q_cmd: Port,
    omega_out: Port,
    theta_out: Port,
    dt: f64,
}

impl Engine {
    pub const STEP: SimDuration = SimDuration::from_millis(1);

    pub fn new(p: EnginePar, q_cmd: Port, omega_out: Port, theta_out: Port, idle_rpm: f64) -> Self {
        Engine {
            omega: idle_rpm * 2.0 * PI / 60.0,
            theta: 0.0,
            running: true,
            p, q_cmd, omega_out, theta_out, dt: Self::STEP.as_secs_f64(),
        }
    }

    pub fn rpm(&self) -> f64 { self.omega * 60.0 / (2.0 * PI) }

    /// T_ind = q · n_cyl · LHV · n_i / 4π
    /// The 4π is the four stroke cycle: one burn per two revolutions
    fn indicated_torque(&self, q_mg: f64) -> f64 {
        if !self.running { return 0.0; }
        let q = q_mg.clamp(0.0, self.p.q_max);
        let load = q / self.p.q_max;
        let m_kg = q * 1e-6 * self.p.cylinders;
        m_kg * self.p.lhv * self.p.eta.eta(self.rpm(), load) / (4.0 * PI)
    }

    fn friction_torque(&self) -> f64 {
        self.p.fric.torque(&self.p.geom, self.rpm().max(0.0), self.p.p_max_nom)
    }
}

impl Component for Engine {
    fn triggers(&self) -> Vec<Trigger> {
        vec![Trigger::Periodic { period: Self::STEP, offset: SimDuration::ZERO }]
    }

    fn step(&mut self, _trig: u16, ctx: &mut Ctx<'_>) {
        let q = ctx.bus.get(self.q_cmd);
        let t_net = self.indicated_torque(q) - self.friction_torque();

        // Semi implicit Euler: update ω first, integrate 0 from the new ω
        self.omega += t_net / self.p.inertia * self.dt;
        if self.omega < self.p.stall_rad_s {
            self.running = false;
            self.omega = self.omega.max(0.0);
        }
        self.theta = (self.theta + self.omega * self.dt) % (2.0 * PI);

        ctx.bus.set(self.omega_out, self.omega);
        ctx.bus.set(self.theta_out, self.theta);
        let _ = ctx.now;
    }
}