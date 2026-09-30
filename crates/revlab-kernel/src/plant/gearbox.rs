use std::f64::consts::PI;
use revlab_core::SimDuration;
use crate::{Component, Ctx, Port, Trigger};
use super::road_load::RoadLoadPar;

#[derive(Copy, Clone)]
pub struct GearboxPorts {
    // inputs
    pub t_c1: Port,     // K1 torque, odd gears
    pub t_c2: Port,     // K2 torque, even gears
    pub sel1: Port,     // gear selected on shaft 1: 0, 1, 3, 5, 7
    pub sel2: Port,     // gear selected on shaft 2: 0, 2, 4, 6
    pub f_road: Port,
    // outputs
    pub omega_in1: Port,
    pub omega_in2: Port,
    pub n_in1: Port,    // rpm, for the shaft speed sensors
    pub n_in2: Port,
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
    pub j_shaft: f64,   // kg·m², each input shaft and its gear set
    pub p: RoadLoadPar,
    w_wheel: f64,       // rad/s
    w_free: [f64; 2],   // rad/s, shaft speed while no gear is selected on it
    ports: GearboxPorts,
    dt: f64,
}
impl Gearbox {
    pub const STEP: SimDuration = SimDuration::from_millis(1);

    pub fn dq200_passat(p: RoadLoadPar, ports: GearboxPorts, v_init_kmh: f64) -> Self {
        Gearbox {
            gear_ratios: [13.633, 7.777, 5.252, 4.011, 3.067, 2.413, 1.957],
            eta: 0.94,
            j_shaft: 0.02,
            w_wheel: v_init_kmh / 3.6 / p.r_wheel,
            w_free: [0.0; 2],
            p, ports,
            dt: Self::STEP.as_secs_f64(),
        }
    }

    /// Ratio if `g` physically lives on shaft `k`. Asking for an even gear on the odd shaft gets you
    /// neutral: the plant is honest about its layout.
    fn ratio_on(&self, k: usize, g: usize) -> Option<f64> {
        let on_shaft = if k == 0 { g % 2 == 1 } else { g % 2 == 0 };
        (g >= 1 && g <= 7 && on_shaft).then(|| self.gear_ratios[g - 1])
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

        let r = self.p.r_wheel;
        let mut t_wheel = -f_road * r;
        let mut j = self.p.mass * r * r + self.p.j_wheels;
        let mut ratio = [None; 2];

        for k in 0..2 {
            ratio[k] = self.ratio_on(k, sel[k]);
            match ratio[k] {
                Some(i) => {
                    t_wheel += tc[k] * i * self.eta;
                    j += self.j_shaft * i * i;
                }
                None => self.w_free[k] += tc[k] / self.j_shaft * self.dt,
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
    }
}