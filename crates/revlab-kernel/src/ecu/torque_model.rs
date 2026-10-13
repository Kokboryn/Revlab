//! The ECU's torque model: fuel per stroke <-> indicated torque, through its own efficiency map.
//!
//! Calibrated on a dyno against this engine type, which is why it has the engine's shape -- and against a
//! population, not this engine, which is why every cell reads 3% low. A torque-structure ECU has to be
//! accurate to a few percent, because everything that coordinates with it -- the TCU above all -- takes
//! its torque on trust

use std::f64::consts::PI;
use revlab_core::Map2d;

pub struct TorqueModel {
    eta: Map2d,
    lhv: f64,
    cylinders: f64,
    pub q_max: f64,         // mg/stroke, injector limit
}

impl TorqueModel {
    pub fn ea288_cal() -> Self {
        let rpm = vec![800.0, 1500.0, 2500.0, 3500.0, 4500.0];
        let load = vec![0.0, 0.15, 0.35, 0.70, 1.00];   // fuel per stroke / q_max
        // row-major: one row per load, columns across rpm
        let dyno = [
            0.10, 0.12, 0.13, 0.12, 0.11,
            0.26, 0.31, 0.34, 0.33, 0.31,
            0.36, 0.42, 0.45, 0.44, 0.41,
            0.38, 0.43, 0.45, 0.43, 0.40,
            0.35, 0.39, 0.41, 0.39, 0.36,
        ];
        let z = dyno.iter().map(|e| e * 0.97).collect();
        TorqueModel { eta: Map2d::new(rpm, load, z), lhv: 42.7e6, cylinders: 4.0, q_max: 60.0 }
    }

    /// Indicated torque from fuel per stroke at speed n
    pub fn t_ind(&self, n: f64, q: f64) -> f64 {
        let q = q.clamp(0.0, self.q_max);
        q * 1e-6 * self.cylinders * self.lhv * self.eta.get(n, q / self.q_max) / (4.0 * PI)
    }

    /// Fuel per stroke for an indicated torque. Efficiency depends on load, so there's no closed form; but
    /// torque rises monotonically with fuel across the whole map, so bisection always lands
    pub fn q_for(&self, n: f64, t_ind: f64) -> f64 {
        if t_ind <= 0.0 { return 0.0; }
        if self.t_ind(n, self.q_max) <= t_ind { return self.q_max; }
        let (mut lo, mut hi) = (0.0, self.q_max);
        for _ in 0..30 {
            let mid = 0.5 * (lo + hi);
            if self.t_ind(n, mid) < t_ind { lo = mid; } else { hi = mid; }
        }
        0.5 * (lo + hi)
    }
}