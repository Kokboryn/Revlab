use super::{EcuState, Task};
use super::torque::{ReqKind, Source, TorqueRequest};
use revlab_core::SimTime;

pub struct IdleTask {
    pub t_ff: f64,      // Nm. base torque to overcome friction
    kp: f64,            // Nm/rpm
    ki: f64,            // Nm/(rpm·s)
    degraded_scale: f64,    // gain derate on a substitute signal
    integ: f64,
    t_min: f64,
    t_max: f64,
    last_update: Option<SimTime>,
    last_seq: u64,      // init 0
}

impl IdleTask {
    pub fn new(target_rpm: f64, t_ff: f64) -> Self {
        IdleTask {
            t_ff,
            kp: 0.09, ki: 0.426,
            degraded_scale: 0.5,
            integ: 0.0,
            t_min: 0.0, t_max: 250.0,
            last_update: None,
            last_seq: 0,
        }
    }
}

impl Task for IdleTask {
    fn name(&self) -> &'static str { "IdleGovernor" }

    fn run(&mut self, s: &mut EcuState) {
        // No new measurement: hold the previous request. Integrating against a stale sample is what destabilized the cam path.
        if s.n_eng_seq == self.last_seq { return; }
        self.last_seq = s.n_eng_seq;

        let dt = match self.last_update {
            Some(t0) => (s.now - t0).as_secs_f64(),
            None => 0.010,
        };
        self.last_update = Some(s.now);
        if dt <= 0.0 || dt > 0.5 { return; }    // implausible gap: skip

        let scale = if s.degraded { self.degraded_scale } else { 1.0 };
        let err = s.target_rpm - s.n_eng;
        let trial = self.integ + self.ki * scale * err * dt;
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