// P17BF-class: clutch overtemperature. A real DSG warns and then protects itself by opening, which
// is unpleasant but cheater than a new pack.

use super::{Task, TcuState};

/// Clutch overtemperature protection. There is no temperature sensor on a DQ200 pack, so the TCU integrates
/// its own estimate from slip and command with its own guessed constants -- and it can be wrong, which
/// is exactly the interesting part. Compare t_disc_est against the plant's disc.
pub struct ClutchThermal {
    pub t_warn: f64,    // K
    pub t_protect: f64, // K
    pub t_clear: f64,   // K, hysteresis so it does not chatter
    pub c_est: f64,     // J/K, the TCU's guess at disc heat capacity
    pub ua_est: f64,    // W/K, its guess at cooling
    pub t_cap_est: f64, // Nm, its guess at full clamp capacity
}

impl ClutchThermal {
    pub fn dq200() -> Self {
        ClutchThermal {
            t_warn:     273.15 + 250.0,
            t_protect:  273.15 + 350.0,
            t_clear:    273.15 + 200.0,
            // Deliberately not the plant's values: a calibration is a guess at a population, not a measurement
            // of this particular clutch
            c_est:      900.0,
            ua_est:     10.0,
            t_cap_est:  320.0,
        }
    }
}

impl Task for ClutchThermal {
    fn name(&self) -> &'static str { "ClutchThermal" }

    fn run(&mut self, s: &mut TcuState) {
        const DT: f64 = 0.1;        // 100 ms task
        const T_AMB: f64 = 293.15;  // no ambient sensor on the gearbox either


        // Slip power from what it commanded and what the shafts are doing. Torque is inferred, never
        // measured
        let slip = (s.n_eng - s.n_in).abs() * 2.0 * std::f64::consts::PI / 60.0;
        let t_est = self.t_cap_est * s.clutch_cmd * s.clutch_cmd;
        let q = if slip > 1.0 { t_est * slip } else { 0.0 };

        s.t_disc_est += (q - self.ua_est * (s.t_disc_est - T_AMB)) / self.c_est * DT;

        if s.t_disc_est > self.t_protect { s.overheat = true; }
        if s.t_disc_est < self.t_clear { s.overheat = false; }
    }
}