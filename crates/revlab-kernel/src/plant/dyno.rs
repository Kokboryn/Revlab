use std::f64::consts::PI;
use revlab_core::{SimDuration, SimTime};
use crate::{Component, Ctx, Port, Trigger};

#[derive(Copy, Clone)]
pub struct DynoPorts {
    pub omega: Port,    // engine speed, rad/s
    pub t_dyno: Port,   // absorbed torque, Nm (positive loads the engine)
}

/// Eddy-current dynamometer: holds the engine at a scheduled speed by absorbing whatever torque it
/// makes. At steady state the absorbed torque IS the engine's brake torque, which is how a test
/// bench measures it. Absorb only: an eddy-current brake can't drive the engine
pub struct Dyno {
    steps: Vec<(SimTime, f64)>,     // (from, rpm); 0 rpm releases
    pub kp: f64,                    // Nm per rad/s
    pub ki: f64,                    // Nm per rad
    pub t_max: f64,                 // Nm, absorber capacity
    integ: f64,
    ports: DynoPorts,
    dt: f64,
}

impl Dyno {
    pub const STEP: SimDuration = SimDuration::from_millis(1);

    pub fn new(steps: Vec<(SimTime, f64)>, ports: DynoPorts) -> Self {
        // With the engine's 0.095 kg·m²: about 14 rad/s bandwidth, damping 0.7
        Dyno { steps, kp: 2.0, ki: 20.0, t_max: 400.0, integ: 0.0, ports, dt: Self::STEP.as_secs_f64() }
    }
    
    fn target_rpm(&self, now: SimTime) -> f64 {
        self.steps.iter().rev().find(|(t, _)| *t <= now).map(|(_, r)| *r).unwrap_or(0.0)
    }
}

impl Component for Dyno {
    fn triggers(&self) -> Vec<Trigger> {
        vec![Trigger::Periodic { period: Self::STEP, offset: SimDuration::from_micros(400) }]
    }
    
    fn step(&mut self, _t: u16, ctx: &mut Ctx<'_>) {
        let rpm_set = self.target_rpm(ctx.now);
        if rpm_set <= 0.0 {
            self.integ = 0.0;
            ctx.bus.set(self.ports.t_dyno, 0.0);
            return;
        }
        // Positive error = engine too fast = absorb more
        let err = ctx.bus.get(self.ports.omega) - rpm_set * PI / 30.0;
        self.integ = (self.integ + self.ki * err * self.dt).clamp(0.0, self.t_max);
        let t = (self.kp * err + self.integ).clamp(0.0, self.t_max);
        ctx.bus.set(self.ports.t_dyno, t);
    }
}