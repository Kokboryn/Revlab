/// Engagement control. During a launch the target is a slip *speed*, not a clutch position: hold the
/// engine at a launch speed set by the pedal and let the clutch take whatever torque keeps it there.
/// That is what makes a launch smooth regardless of pedal, load or gradient, and it is why a real DSG
/// can pull away on a hill without the driver balancing anything.

use super::{ClutchState, Task, TcuState};
pub struct ClutchControl {
    pub rate_open: f64,     // 1/s, releasing is a valve opening -- fast
    pub rate_max: f64,      // 1/s, actuator slew -- the servo has bandwidth
    pub lock_slip: f64,     // rpm, below which it commits to closed
    pub n_floor: f64,       // rpm, hard floor
    pub n_guard: f64,       // rpm, start protecting here
    // Hold the engine at launch speed rather than commanding position. The clutch takes whatever
    // torque keeps the engine there: more pedal means a higher target and a faster launch, and the
    // guard never has to intervene because the controller is already doing its job.
    pub n_launch: f64,      // rpm, engine speed to hold during engagement
    pub kp_n: f64,          // command per rpm of engine speed error -- the damping
    pub ki_n: f64,          // command per rpm·s -- trims out the steady error
    pub creep_cmd: f64,     // clamp held in D with the brake released
    cmd: f64,
    i_n: f64,               // integrator, tracks the actual command so it can't wind up
}

impl ClutchControl {
    pub fn dq200() -> Self {
        ClutchControl {
            rate_open: 10.0,
            rate_max: 2.0,
            lock_slip: 30.0,
            n_floor: 600.0,
            n_guard: 750.0,
            n_launch: 1400.0,
            kp_n: 4.5e-4,
            ki_n: 2.7e-3,
            creep_cmd: 0.30,
            cmd: 0.0,
            i_n: 0.0,
        }
    }
}

impl Task for ClutchControl {
    fn name(&self) -> &'static str { "ClutchControl" }

    fn run(&mut self, s: &mut TcuState) {
        let slip = s.n_eng - s.n_in;

        let mut err = 0.0;          // stays zero unless the speed loop is in charge
        let mut target = match (s.lever, s.gear) {
            (lever, g) if lever.drives() && g >= 1 => {
                if s.n_in < 50.0 && s.pedal < 0.02 {
                    0.0                             // stopped, pedal up: creep later, open for now
                } else if s.n_in > s.n_eng - self.lock_slip && s.n_in > 300.0 {
                    1.0
                } else {
                    // PI on engine speed. The engine is an integrator -- clutch torque sets its acceleration
                    // -- so pure integral action on top of it has no damping and hunts. The proportional
                    // term is what damps it
                    err = s.n_eng - (self.n_launch + s.pedal * 800.0);
                    self.i_n = (self.i_n + self.ki_n * err * 0.01).clamp(0.0, 1.0);
                    self.i_n + self.kp_n * err
                }
            }
            _ => 0.0,                               // N, P, R until reverse exists
        };

        // Creep. A real automatic holds the clutch at its touch point in D once the brake is released,
        // which is why it crawls forward on the flat -- and why it only rolls back a little on a gradient
        // instead of freewheeling away.
        if s.lever.drives() && s.gear >= 1 && s.brake < 0.05 {
            target = target.max(self.creep_cmd)
        }

        if s.clutch_state == ClutchState::Closed && s.lever.drives() && s.gear >= 1 {
            target = 1.0;
        }

        // Protect on the way in, not after the fact. At 0.095 kg.m2 the engine can fall 1000 rpm inside
        // one task period, so waiting for it to reach the floor is already too late -- back off across
        // a band above it instead
        if s.n_eng < self.n_guard {
            let headroom = ((s.n_eng - self.n_floor) / (self.n_guard - self.n_floor)).clamp(0.0, 1.0);
            target = target.min(headroom);
        }


        // Actuator slew. The mechatronic unit is a servo, not an instant command, and that lag is part
        // of why a DSG engagement feels the way it does.
        // Releasing is a valve opening and engaging is a pump filling, so the mechatronic unit can
        // dump clamp far faster than it can build it. That asymmetry is what lets the engine guard
        // actually save a launch.
        let step_up = self.rate_max * 0.01;
        let step_dn = self.rate_open * 0.01;
        self.cmd = (self.cmd + (target - self.cmd).clamp(-step_dn, step_up)).clamp(0.0, 1.0);


        // Protection is not overridable, same principle as the ECU's arbiter.
        if s.overheat { self.cmd = self.cmd.min(0.0); }

        // Back calculation: the integrator follows what the actuator actually did, so creep, the guard
        // and the slew limit can't leave it wound up, and entering the speed loop from creep is bumpless.
        self.i_n = self.cmd - self.kp_n * err;

        s.clutch_cmd = self.cmd;
        s.clutch_state = if self.cmd < 0.01 { ClutchState::Open }
        else if slip.abs() < self.lock_slip && s.n_in > 300.0 { ClutchState::Closed }
        else { ClutchState::Engaging };
    }
}
