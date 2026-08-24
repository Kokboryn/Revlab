use revlab_core::SimDuration;
use crate::{Component, Ctx, Port, Trigger};
use super::gas::R_AIR;

pub const G: f64 = 9.80665;     // m/s²

#[derive(Copy, Clone)]
pub struct RoadLoadPorts {
    // inputs
    pub v_veh: Port,
    pub grade: Port,        // rad
    pub headwind: Port,     // m/s, + = against
    pub brake: Port,        // 0..1
    pub p_amb: Port,
    pub t_amb: Port,
    // outputs
    pub f_road: Port,       // N, total resistance opposing motion
}


/// Longitudinal resistance on the vehicle. Physical rather than a coast down polynomial, so mass,
/// drag area, and tire choice each show up as themselves.
pub struct RoadLoadPar {
    pub mass: f64,          // kg, kerb + occupants
    pub cd: f64,
    pub frontal_area: f64,  // m²
    pub c_rr: f64,          // rolling resistance coefficient
    pub r_wheel: f64,       // m, loaded rolling radius
    pub j_wheels: f64,      // kg·m², all four plus brake discs
}

impl RoadLoadPar {
    pub fn passat_b8_16tdi() -> Self {
        RoadLoadPar {
            mass: 1582.0,       // 1502 curb (registration) + driver
            cd: 0.27,
            frontal_area: 2.19,
            c_rr: 0.010,
            r_wheel: 0.314,         // 215/50R17 loaded
            j_wheels: 3.2,
        }
    }
}

pub struct RoadLoad {
    pub p: RoadLoadPar,
    pub brake_max: f64,     // N at the contact patch, full pedal
    ports: RoadLoadPorts,
}

impl RoadLoad {
    pub const STEP: SimDuration = SimDuration::from_millis(1);

    pub fn new(p: RoadLoadPar, ports: RoadLoadPorts) -> Self {
        RoadLoad { p, brake_max: 12_000.0, ports}
    }
}

impl Component for RoadLoad {
    fn triggers(&self) -> Vec<Trigger> {
        vec![Trigger::Periodic { period: Self::STEP, offset: SimDuration::from_micros(400) }]
    }

    fn step(&mut self, _t: u16, ctx: &mut Ctx<'_>) {
        let v = ctx.bus.get(self.ports.v_veh);
        let grade = ctx.bus.get(self.ports.grade);          // rad
        let headwind = ctx.bus.get(self.ports.headwind);    // m/s, + = against
        // Density from measured ambient, so altitude changes drag as it should
        let rho = ctx.bus.get(self.ports.p_amb) / (R_AIR * ctx.bus.get(self.ports.t_amb));

        let v_air = v + headwind;
        let f_aero = 0.5 * rho * self.p.cd * self.p.frontal_area * v_air * v_air.abs();
        // signum(0.0) is 1.0 in Rust, which would push a stationary car backwards
        let dir = if v.abs() < 1e-3 { 0.0 } else { v.signum() };
        let f_roll = self.p.c_rr * self.p.mass * G * grade.cos() * dir;
        let f_grade = self.p.mass * G * grade.sin();
        let f_brake = ctx.bus.get(self.ports.brake).clamp(0.0, 1.0) * self.brake_max;

        ctx.bus.set(self.ports.f_road, f_aero + f_roll + f_grade + f_brake * v.signum());
    }
}
