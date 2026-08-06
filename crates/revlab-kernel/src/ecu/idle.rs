use super::{EcuState, Task};

pub struct IdleTask {
    pub target_rpm: f64,
    pub q_ff: f64,
    kp: f64,
    ki: f64,
    integ: f64,
    q_min: f64,
    q_max: f64,
    dt: f64,
}

impl IdleTask {
    pub fn new(target_rpm: f64, q_ff: f64) -> Self {
        IdleTask {
            target_rpm, q_ff, kp: 0.004, ki: 0.15, integ: 0.0, q_min: 0.0, q_max: 60.0, dt: 0.010,
        }
    }
}

impl Task for IdleTask {
    fn name(&self) -> &'static str { "IdleGovernor" }

    fn run(&mut self, s: &mut EcuState) {
        let err = self.target_rpm - s.n_eng;
        let trial = self.integ + self.ki * err * self.dt;
        let u = self.q_ff + self.kp * err + trial;
        let q = u.clamp(self.q_min, self.q_max);
        if (q - u).abs() < 1e-12 { self.integ = trial; }
        s.q_cmd = q;
    }
}