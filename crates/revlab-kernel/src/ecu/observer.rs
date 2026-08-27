use std::f64::consts::PI;
use revlab_core::SimTime;
use super::{EcuState, Task};

/// The ECU's own engine model - an independent third opinion on speed.
///
/// It's constants are the ECU's calibration, not the plant's: j_cal is 5% off the true inertia, and the
/// friction model is fitted so that it BALANCES the ECU's own torque model at idle. That mutual consistency
/// matters more than absolute accuracy - a model biased against its own torque path would integrate away in seconds.
pub struct SpeedObserver {
    pub j_cal: f64,         // kg·m²
    pub eta_cal: f64,
    pub lhv_cal: f64,
    pub cylinders: f64,
    pub fric_a: f64,        // Nm
    pub fric_b: f64,        // Nm·s/rad
    pub correct_gain: f64,  // 1/s, pull toward the trusted sensor
    pub bias_gain: f64,     // Nm per rpm second, integral trim
    pub bias_max: f64,      // Nm, clamp
    last: Option<SimTime>,
}

impl SpeedObserver {
    pub fn di_diesel_1_6() -> Self {
        SpeedObserver {
            j_cal: 0.100,       // true is 0.0952
            eta_cal: 0.19,      // same constant TorqueToFuel uses
            lhv_cal: 42.7e6,
            cylinders: 4.0,
            fric_a: 14.0,
            fric_b: 0.021,      // 14.0 + 0.021·83.8 = 15.76 Nm at idle, matching eta_cal's torque at 6.1 mg
            // PI observer. Speed responds at 95.5 rpm/s per Nm with j_cal, so wn = sqrt(95.5 * bias_gain) = 1.95 rad/s
            // and zeta = 0.26.
            // Underdamped by design for now: detection latency on crank_drift was the binding constraint,
            // not overshoot.
            correct_gain: 1.0,
            bias_gain: 0.04,
            bias_max: 300.0,
            last: None,
        }
    }
}

impl Task for SpeedObserver {
    fn name(&self) -> &'static str { "SpeedObserver" }

    fn run(&mut self, s: &mut EcuState) {
        let dt = match self.last {
            Some(t0) => (s.now - t0).as_secs_f64(),
            None => {
                self.last = Some(s.now);
                s.n_model = s.n_eng;        // initialize on the first sample
                return;
            }
        };
        self.last = Some(s.now);
        if dt <= 0.0 || dt > 0.5 { return }

        // --- predict from the fuel we commanded, less whatever torque the model cannot see:
        // external load, driveline drag, residual calibration error
        let m_kg = s.q_cmd * 1e-6 * self.cylinders;
        let t_ind = m_kg * self.lhv_cal * self.eta_cal / (4.0 * PI);
        let omega = s.n_model * 2.0 * PI / 60.0;
        // LossModel runs later in the 10ms table, so warmup_mult is one cycle old. It moves over minutes;
        // 10 ms is nothing.
        let t_fric = (self.fric_a + self.fric_b * omega) * s.warmup_mult;
        s.n_model += (t_ind - t_fric - s.t_bias) / self.j_cal * dt * 60.0 / (2.0 * PI);

        // --- correct toward the trusted sensor, UNLESS a fault is being evaluated.
        // Both terms freeze: the integrator would otherwise learn a lying sensor as load and make the
        // model agree with it, destroying the third opinion.
        if !s.freeze_adaptation {
            let err = s.n_eng - s.n_model;
            s.n_model += self.correct_gain * err * dt;
            s.t_bias -= self.bias_gain * err * dt;
            s.t_bias = s.t_bias.clamp(-self.bias_max, self.bias_max);
        }

        s.n_model = s.n_model.max(0.0);
        // A frozen correction must not license unbounded divergence. Past this band the model is not
        // a usable third opinion, and diag must not arbitrate on it.
        let plausible = (s.crank_valid || s.cam_valid) && (s.n_model - s.n_eng).abs() < 1000.0;
        s.model_valid = plausible;

        // Re-anchor rather than integrate into nonsense. Past the plausibility band the correction
        // cannot recover the model on its own -- and during a launch transient it is frozen half the
        // time, so it does not even try. Real ECUs reinitialise a diverged model instead. The bias goes
        // with it: whatever it learned belongs to the diverged trajectory.
        if !plausible && (s.crank_valid || s.cam_valid) {
            s.n_model = s.n_eng;
            s.t_bias = 0.0;
        }
    }
}