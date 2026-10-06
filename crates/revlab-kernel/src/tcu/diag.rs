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
    pub ua_still_est: f64,  // W/K, its guess at cooling at rest
    pub ua_speed_est: f64,  // W/K per m/s, its guess at how much road speed helps
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
            ua_still_est: 10.0,
            ua_speed_est: 1.0,
            t_cap_est:  320.0,
        }
    }
}

impl Task for ClutchThermal {
    fn name(&self) -> &'static str { "ClutchThermal" }

    fn run(&mut self, s: &mut TcuState) {
        const DT: f64 = 0.01;        // 10 ms task: a shift's slip lasts ~0.3 s, too short to integrate at 100 ms
        const T_AMB: f64 = 293.15;  // no ambient sensor on the gearbox either

        // A dry pack is cooled by air through the bell housing, so road speed matters. The TCU has it
        // from the ABS; the constants are still its own guesses
        let ua = self.ua_still_est + self.ua_speed_est * s.v_veh / 3.6;

        // One model per pack. Each sees its own command and the slip across it -- engine against its own
        // shaft -- so heat lands in the clutch that took it, including the oncoming one mid-shift. Torque
        // is inferred from command, never measured
        for k in 0..2 {
            let slip = (s.n_eng - s.n_shaft[k]).abs() * 2.0 * std::f64::consts::PI / 60.0;
            let t_est = self.t_cap_est * s.cmd[k] * s.cmd[k];
            let q = if slip > 1.0 { t_est * slip } else { 0.0 };
            s.t_disc_est[k] += (q - ua * (s.t_disc_est[k] - T_AMB)) / self.c_est * DT;
        }

        // One flag for now, from the hotter pack
        let hottest = s.t_disc_est[0].max(s.t_disc_est[1]);
        if hottest > self.t_protect { s.overheat = true; }
        if hottest < self.t_clear { s.overheat = false; }
    }
}