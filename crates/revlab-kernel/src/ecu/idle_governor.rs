use revlab_core::SimDuration;
use crate::{Component, Ctx, Port, Trigger};

pub struct IdleGovernor {
    pub target_rpm: f64,
    pub q_ff: f64,      // base fueling, mg/stroke - calibration, not a tuning gain
    kp: f64,
    ki: f64,
    integ: f64,
    q_min: f64,
    q_max: f64,
    dt: f64,
    n_meas: Port,
    q_cmd: Port,
}

impl IdleGovernor {
    pub const PERIOD: SimDuration = SimDuration::from_millis(10);

    pub fn new(n_meas: Port, q_cmd: Port, target_rpm: f64) -> Self {
        IdleGovernor {
            target_rpm, q_ff: 6.1, kp: 0.004, ki: 0.15, integ: 0.0, q_min: 0.0, q_max: 60.0, dt: Self::PERIOD.as_secs_f64(), n_meas, q_cmd,
        }
    }
}

impl Component for IdleGovernor {
    fn triggers(&self) -> Vec<Trigger> {
        vec![Trigger::Periodic { period: Self::PERIOD, offset: SimDuration::from_millis(1) }]
    }

    fn step(&mut self, _trig: u16, ctx: &mut Ctx<'_>) {
        let err = self.target_rpm - ctx.bus.get(self.n_meas);
        let trial = self.integ + self.ki * err * self.dt;
        let u = self.q_ff + self.kp * err + trial;
        let q = u.clamp(self.q_min, self.q_max);

        // Clamping anti-windup: only integrate when not saturated
        if (q - u).abs() < 1e-12 { self.integ = trial; }

        ctx.bus.set(self.q_cmd, q);
    }
}