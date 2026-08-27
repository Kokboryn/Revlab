use std::f64::consts::PI;
use revlab_core::SimDuration;
use crate::{Component, Ctx, Port, Trigger};
use super::road_load::RoadLoadPar;


#[derive(Copy, Clone)]
pub struct DrivelinePorts {
    // inputs
    pub omega_in: Port,
    pub gear: Port,         // 0 = neutral, 1..7
    pub f_road: Port,
    // outputs
    pub v_veh: Port,
    pub n_wheel: Port,
    pub t_out: Port,
    pub j_ref: Port,
}
pub struct Driveline {
    /// Overall ratios, engine revolutions per wheel revolution: gear ratio and final drive collapsed
    /// together. The DQ200 runs gears 1-4 and 5-7 on two output shafts with different final drives,
    /// so a single scalar cannot represent it -- and only the product is ever needed here.
    pub gear_ratios: [f64; 7],
    pub eta: f64,           // gearbox + final drive mechanical efficiency
    pub p: RoadLoadPar,     // shares r_wheel, mass, j_wheels
    ports: DrivelinePorts,
}

impl Driveline {
    pub const STEP: SimDuration = SimDuration::from_millis(1);

    pub fn dq200_passat(p: RoadLoadPar, ports: DrivelinePorts) -> Self {
        // Measured from the vehicle: overall ratios = published DQ200 gear set x a single final drive
        // of 3.617, which gears 4,5 and 7 all agree on to within 1%.
        Driveline {
            gear_ratios: [13.633, 7.777, 5.252, 4.011, 3.067, 2.413, 1.957],
            eta: 0.94,
            p, ports,
        }
    }
}
impl Component for Driveline {
    fn triggers(&self) -> Vec<Trigger> {
        vec![Trigger::Periodic { period: Self::STEP, offset: SimDuration::from_micros(450) }]
    }
    fn step(&mut self, _t: u16, ctx: &mut Ctx<'_>) {
        let gear = ctx.bus.get(self.ports.gear).round().max(0.0) as usize;
        if gear == 0 || gear > self.gear_ratios.len() {
            ctx.bus.set(self.ports.t_out, 0.0);
            ctx.bus.set(self.ports.j_ref, 0.0);
            return;
        }
        let i = self.gear_ratios[gear - 1];
        let omega = ctx.bus.get(self.ports.omega_in);
        let f_road = ctx.bus.get(self.ports.f_road);

        ctx.bus.set(self.ports.v_veh, omega / i * self.p.r_wheel);
        ctx.bus.set(self.ports.n_wheel, omega / i * 60.0 / (2.0 * PI));
        ctx.bus.set(self.ports.t_out, f_road * self.p.r_wheel / (i * self.eta));
        ctx.bus.set(self.ports.j_ref, (self.p.mass * self.p.r_wheel.powi(2) + self.p.j_wheels) / (i * i * self.eta));
    }
}