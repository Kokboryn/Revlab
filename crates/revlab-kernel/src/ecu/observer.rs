use std::f64::consts::PI;
use revlab_core::SimTime;
use super::{EcuState, Task};
use super::torque_model::TorqueModel;

/// The ECU's own engine model - an independent third opinion on speed.
///
/// Its inertia is the ECU's calibration, 5% off the true value. Torque comes from the ECU's own torque
/// path: the calibrated model TorqueToFuel inverts, less LossModel's friction and pumping, but evaluated
/// at the model's own speed, never the sensor's. That mutual consistency matters more than absolute
/// accuracy, a model biased against its own torque path would integrate away in seconds
pub struct SpeedObserver {
    pub j_cal: f64,         // kg·m²
    pub correct_gain: f64,  // 1/s, pull toward the trusted sensor
    pub bias_gain: f64,     // Nm per rpm second, integral trim
    pub bias_max: f64,      // Nm, clamp
    pub ratios: [f64; 7],   // overall ratios incl. final drive, calibration copy
    pub mass_cal: f64,      // kg, vehicle with driver
    pub r_wheel: f64,       // m, coded rolling radius
    pub f_road_max: f64,    // N, clamp on the learned road force
    pub model: TorqueModel, // The ECU's calibrated torque model: the same one TorqueToFuel inverts
    pub fric_a: f64,        // Nm | copies of LossModel's fit, evaluated at the model's own speed:
    pub fric_b: f64,        // Nm | under a crank fault n_eng is the faulty reading, and a third
    pub fric_c: f64,        // Nm | opinion must not borrow it
    pub displacement: f64,  // m³, pumping term
    last: Option<SimTime>,
}

impl SpeedObserver {
    pub fn di_diesel_1_6() -> Self {
        SpeedObserver {
            j_cal: 0.100,       // true is 0.0952
            // PI observer. Speed responds at 95.5 rpm/s per Nm with j_cal, so wn = sqrt(95.5 * bias_gain)
            // = 1.95 rad/s and zeta = 0.26.
            // Underdamped by design for now: detection latency on crank_drift was the binding constraint,
            // not overshoot.
            correct_gain: 1.0,
            bias_gain: 0.04,
            bias_max: 300.0,
            ratios: [13.633, 7.777, 5.252, 4.011, 3.067, 2.413, 1.957],
            mass_cal: 1550.0,    // 1459 kg kerb + driver
            r_wheel: 0.314,
            f_road_max: 8000.0,
            last: None,
            model: TorqueModel::ea288_cal(),
            fric_a: 15.9,
            fric_b: 0.0133,
            fric_c: 4.1e-5,
            displacement: 1.598e-3,
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

        // --- driveline state, from the TCU over CAN. Prepare still runs on the old gear, locked
        let g = s.tcu_gear as usize;
        let handing_over = s.tcu_shift >= 2;
        let locked = s.tcu_clutch == 2 && !handing_over && (1..=7).contains(&g);
        let slipping = !locked && (handing_over || s.tcu_clutch == 1);
        let i = if locked { self.ratios[g - 1] } else { 1.0 };

        // Locked: the car is part of the inertia, reflected through the gear, and the load is road force
        // Slipping: the engine alone, against the clutch torque the TCU reports. Open: the engine alone
        let (j_eff, t_drive) = if locked {
            (self.j_cal + self.mass_cal * self.r_wheel * self.r_wheel / (i * i), s.f_road_est * self.r_wheel / i)
        } else if slipping {
            (self.j_cal, s.tcu_t_clutch)
        } else {
            (self.j_cal, 0.0)
        };

        // --- predict from the fuel we commanded, through the ECU's own torque model, less friction
        // and pumping computed at the model's speed
        let t_ind = self.model.t_ind(s.n_model, s.q_cmd);
        let omega = s.n_model * 2.0 * PI / 60.0;
        let t_fric = (self.fric_a + self.fric_b * omega + self.fric_c * omega * omega) * s.warmup_mult;
        let t_pump = (s.p_em_est - s.p_im_meas) * self.displacement / (4.0 * PI);
        s.n_model += (t_ind - t_fric - t_pump - s.t_bias - t_drive) / j_eff * dt * 60.0 / (2.0 * PI);

        // --- correct toward the trusted sensor, UNLESS a fault is being evaluated. Both terms freeze:
        // the integrator would otherwise learn a lying sensor as load and make the model agree with it,
        // destroying the third opinion.
        if !s.freeze_adaptation {
            let err = s.n_eng - s.n_model;
            s.n_model += self.correct_gain * err * dt;
            if locked {
                // Road force, in N so it survives gear changes. The gain scales with the inertia so
                // the loop keeps the same dynamics in every gear
                s.f_road_est -= self.bias_gain * (j_eff / self.j_cal) * err * dt * i / self.r_wheel;
                s.f_road_est = s.f_road_est.clamp(-self.f_road_max, self.f_road_max);
            } else {
                s.t_bias -= self.bias_gain * err * dt;
                s.t_bias = s.t_bias.clamp(-self.bias_max, self.bias_max);
            }
        }

        s.n_model = s.n_model.max(0.0);
        // A frozen correction must not license unbounded divergence. Past this band the model is not
        // a usable third opinion, and diag must not arbitrate on it.
        let plausible = (s.crank_valid || s.cam_valid) && (s.n_model - s.n_eng).abs() < 1000.0;
        s.model_valid = plausible;

        // Re-anchor rather than integrate into nonsense. Whatever both learned terms hold belongs to
        // the diverged trajectory.
        if !plausible && (s.crank_valid || s.cam_valid) {
            s.n_model = s.n_eng;
            s.t_bias = 0.0;
            s.f_road_est = 0.0;
        }
    }
}