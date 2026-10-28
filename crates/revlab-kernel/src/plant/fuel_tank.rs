//! Fuel tank: drains by what the injectors deliver. The sender reports the true level; damping it is
//! the cluster's job, like in the car

use crate::{Component, Ctx, Port, Trigger};
use revlab_core::SimDuration;


pub struct FuelTankPorts {
    pub q_cmd: Port,    // in: injected mg per stroke per cylinder
    pub omega: Port,    // in: crank speed, rad/s
    pub fuel_l: Port,   // out: Liters in the tank
    pub fuel_lph: Port, // out: flow to the engine, l/h
}

pub struct FuelTank {
    p: FuelTankPorts,
    level_l: f64,
    rho: f64,           // kg/l
    cylinders: f64,
}

const DT: f64 = 0.01;

impl FuelTank {
    /// Passat B8: 66 l. Diesel (EN 590) is 0.820-0.845 kg/l at 15 °C
    pub fn passat_b8(p: FuelTankPorts, fill: f64) -> Self {
        Self { p, level_l: 66.0 * fill.clamp(0.0, 1.0), rho: 0.835, cylinders: 4.0 }
    }
}

impl Component for FuelTank {
    fn triggers(&self) -> Vec<Trigger> {
        vec![Trigger::Periodic {period: SimDuration::from_millis(10), offset: SimDuration::from_micros(600) }]
    }

    fn step(&mut self, _trig: u16, ctx: &mut Ctx<'_>) {
        let q = ctx.bus.get(self.p.q_cmd).max(0.0);
        let omega = ctx.bus.get(self.p.omega).max(0.0);
        // Four stroke: each cylinder injects once every two revolutions
        let strokes_per_s = self.cylinders * omega / (4.0 * std::f64::consts::PI);
        let lps = q * 1e-6 * strokes_per_s / self.rho;
        self.level_l = (self.level_l - lps * DT).max(0.0);
        ctx.bus.set(self.p.fuel_l, self.level_l);
        ctx.bus.set(self.p.fuel_lph, lps * 3600.0);
    }
}