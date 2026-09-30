use std::f64::consts::PI;
use revlab_core::SimDuration;
use crate::{Component, Ctx, Port, Trigger};
use super::road_load::RoadLoadPar;

// Shift fork and synchronizer on one shaft. The TCU asks for a gear; the plant decides when it has one
#[derive(Copy, Clone, PartialEq, Debug)]
enum Fork {
    Neutral,
    Travel { to: usize, left: f64 },    // s of fork travel remaining, shaft free
    Sync { to: usize },                 // cone dragging the shaft to the gear's speed
    Engaged(usize),
}

#[derive(Copy, Clone)]
pub struct GearboxPorts {
    // inputs
    pub t_c1: Port,     // K1 torque, odd gears
    pub t_c2: Port,     // K2 torque, even gears
    pub sel1: Port,     // fork request, shaft 1: 1, 3, 5, 7. 0 = no command, fork holds its detent
    pub sel2: Port,     // dork request, shaft 2: 2, 4, 6
    pub f_road: Port,
    // outputs
    pub omega_in1: Port,
    pub omega_in2: Port,
    pub n_in1: Port,    // rpm, for the shaft speed sensors
    pub n_in2: Port,
    pub eng1: Port,     // gear actually engaged on shaft 1, 0 while moving -- the fork position sensor
    pub eng2: Port,
    pub v_veh: Port,
    pub n_wheel: Port,
}

/// DQ200 layout: two input shafts, each with its own clutch. Odd gears on shaft 1, even on shaft 2.
/// The vehicle owns the speed state; a shaft it neutral spins on its own inertia against its clutch.
/// With both shafts geared -- which is exactly the state during a shift -- they are two views of one
/// state rather than two integrators that could drift apart.
pub struct Gearbox {
    /// Overall ratios, engine revolutions per wheel revolution
    pub gear_ratios: [f64; 7],
    pub eta: f64,
    pub j_shaft: f64,       // kg·m², each input shaft and its gear set
    pub t_travel: f64,      // s, fork travel from the detent to the synchro
    pub t_sync: f64,        // Nm at the shaft, synchronizer cone capacity
    pub w_sync_tol: f64,    // rad/s, the dog teeth drop in below this speed difference
    pub t_dog_hold: f64,    // Nm, a dog carrying more than this can't be pulled out
    pub p: RoadLoadPar,
    w_wheel: f64,       // rad/s
    w_free: [f64; 2],   // rad/s, shaft speed while no gear is selected on it
    fork: [Fork; 2],
    ports: GearboxPorts,
    dt: f64,
}
impl Gearbox {
    pub const STEP: SimDuration = SimDuration::from_millis(1);

    pub fn dq200_passat(p: RoadLoadPar, ports: GearboxPorts, v_init_kmh: f64, gear_init: usize) -> Self {
        let w_wheel = v_init_kmh / 3.6 / p.r_wheel;
        let mut gb = Gearbox {
            gear_ratios: [13.633, 7.777, 5.252, 4.011, 3.067, 2.413, 1.957],
            eta: 0.94,
            j_shaft: 0.02,
            // Starting values, not measured: a hydraulic fork actuator travels in a few tens of ms,
            // and a small passenger-car synchro holds on the order of 10 Nm at the input shaft
            t_travel: 0.060,
            t_sync: 10.0,
            w_sync_tol: 2.0,
            t_dog_hold: 5.0,
            w_wheel,
            w_free: [0.0; 2],
            fork: [Fork::Neutral; 2],
            p, ports,
            dt: Self::STEP.as_secs_f64(),
        };
        // A rolling start is an initial condition: the starting gear is already in and its shaft turning
        let k = if gear_init % 2 == 1 { 0 } else { 1 };
        if let Some(i) = gb.ratio_on(k, gear_init) {
            gb.fork[k] = Fork::Engaged(gear_init);
            gb.w_free[k] = w_wheel * i;
        }
        gb
    }

    /// Ratio if `g` physically lives on shaft `k`. Asking for an even gear on the odd shaft gets you
    /// neutral: the plant is honest about its layout.
    fn ratio_on(&self, k: usize, g: usize) -> Option<f64> {
        let on_shaft = if k == 0 { g % 2 == 1 } else { g % 2 == 0 };
        (g >= 1 && g <= 7 && on_shaft).then(|| self.gear_ratios[g - 1])
    }

    fn fork_step(&mut self, k: usize, req: usize, tc: f64) {
        // An unpowered fork stays in its detent, and a request the layout can't satisfy is ignored.
        // Either way the fork carries on with whatever it was already doing
        let req = if self.ratio_on(k, req).is_some() { req } else {
            match self.fork[k] {
                Fork::Engaged(g) | Fork::Travel { to: g, .. } | Fork::Sync { to: g } => g,
                Fork::Neutral => return,
            }
        };
        self.fork[k] = match self.fork[k] {
            // A loaded dog is held in by its own torque: the fork can't pull it out
            Fork::Engaged(g) if g == req || tc.abs() > self.t_dog_hold => Fork::Engaged(g),
            Fork::Engaged(_) | Fork::Neutral => Fork::Travel { to: req, left: self.t_travel },
            Fork::Travel { left, .. } if left > self.dt => Fork::Travel { to: req, left: left - self.dt },
            Fork::Travel { .. } => Fork::Sync { to: req },
            Fork::Sync { to } if to != req => Fork::Travel { to: req, left: self.t_travel },
            Fork::Sync { to } => {
                let w_gear = self.w_wheel * self.gear_ratios[to - 1];
                if (self.w_free[k] - w_gear).abs() < self.w_sync_tol { Fork::Engaged(to) }
                else { Fork::Sync { to } }
            }
        };
    }

    fn engaged(&self, k: usize) -> usize {
        match self.fork[k] { Fork::Engaged(g) => g, _ => 0 }
    }
}

impl Component for Gearbox {
    fn triggers(&self) -> Vec<Trigger> {
        vec![Trigger::Periodic { period: Self::STEP, offset: SimDuration::from_micros(450) }]
    }

    fn step(&mut self, _t: u16, ctx: &mut Ctx<'_>) {
        let sel = [ctx.bus.get(self.ports.sel1).round().max(0.0) as usize,
                            ctx.bus.get(self.ports.sel2).round().max(0.0) as usize];
        let tc = [ctx.bus.get(self.ports.t_c1), ctx.bus.get(self.ports.t_c2)];
        let f_road = ctx.bus.get(self.ports.f_road);
        for k in 0..2 { self.fork_step(k, sel[k], tc[k]); }

        let r = self.p.r_wheel;
        let mut t_wheel = -f_road * r;
        let mut j = self.p.mass * r * r + self.p.j_wheels;
        let mut ratio = [None; 2];

        for k in 0..2 {
            match self.fork[k] {
                Fork::Engaged(g) => {
                    let i = self.gear_ratios[g - 1];
                    ratio[k] = Some(i);
                    t_wheel += tc[k] * i * self.eta;
                    j += self.j_shaft * i * i;
                }
                Fork::Sync { to } => {
                    // The cone drags the shaft toward the gear's speed and reacts against the wheels
                    let i = self.gear_ratios[to - 1];
                    // Friction, not a motor: the cone carries at most t_sync, and only as much as
                    // it takes to close the gap -- zero when there is none
                    let w_gear = self.w_wheel * i;
                    let t_s = ((w_gear - self.w_free[k]) * self.j_shaft / self.dt)
                        .clamp(-self.t_sync, self.t_sync);
                    self.w_free[k] += (tc[k] + t_s) / self.j_shaft * self.dt;
                    t_wheel -= t_s * i;
                }
                Fork::Neutral | Fork::Travel { .. } => {
                    self.w_free[k] += tc[k] / self.j_shaft * self.dt;
                }
            }
        }

        // No floor: a clutch that cannot hold the grade lets the car roll back
        self.w_wheel += t_wheel / j * self.dt;

        let mut w_in = [0.0; 2];
        for k in 0..2 {
            w_in[k] = match ratio[k] {
                Some(i) => self.w_wheel * i,
                None => self.w_free[k],
            };
            // A shaft that goes to neutral carries on from where it was
            self.w_free[k] = w_in[k];
        }

        let rpm = 60.0 / (2.0 * PI);
        ctx.bus.set(self.ports.omega_in1, w_in[0]);
        ctx.bus.set(self.ports.omega_in2, w_in[1]);
        ctx.bus.set(self.ports.n_in1, w_in[0] * rpm);
        ctx.bus.set(self.ports.n_in2, w_in[1] * rpm);
        ctx.bus.set(self.ports.v_veh, self.w_wheel * r);
        ctx.bus.set(self.ports.n_wheel, self.w_wheel * rpm);
        ctx.bus.set(self.ports.eng1, self.engaged(0) as f64);
        ctx.bus.set(self.ports.eng2, self.engaged(1) as f64);
    }
}