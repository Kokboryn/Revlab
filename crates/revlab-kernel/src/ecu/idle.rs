use super::{EcuState, Task};
use super::torque::{ReqKind, Source, TorqueRequest};

pub struct IdleTask {
    pub target_rpm: f64,
    pub t_ff: f64,      // Nm. base torque to overcome friction
    kp: f64,            // Nm/rpm
    ki: f64,            // Nm/(rpm·s)
    integ: f64,
    t_min: f64,
    t_max: f64,
    dt: f64,
}

impl IdleTask {
    pub fn new(target_rpm: f64, t_ff: f64) -> Self {
        IdleTask {
            target_rpm, t_ff, kp: 0.0114, ki: 0.426, integ: 0.0, t_min: 0.0, t_max: 250.0, dt: 0.010,
        }
    }
}

impl Task for IdleTask {
    fn name(&self) -> &'static str { "IdleGovernor" }

    fn run(&mut self, s: &mut EcuState) {
        let err = self.target_rpm - s.n_eng;
        let trial = self.integ + self.ki * err * self.dt;
        let u = self.t_ff + self.kp * err + trial;
        let t = u.clamp(self.t_min, self.t_max);
        if (t - u).abs() < 1e-12 { self.integ = trial; }

        s.reqs[Source::Idle as usize] = TorqueRequest {
            kind: ReqKind::MinLimit,    // idle control is a floor, not a target
            value: t,
            active: true,
        };
    }
}