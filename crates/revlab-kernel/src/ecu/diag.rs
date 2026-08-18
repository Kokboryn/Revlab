use super::torque::{ReqKind, Source, TorqueRequest};
use super::{EcuState, Task};

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Sensor { Crank = 0, Cam = 1 }
pub const N_SENSORS: usize = 2;

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum DtcState { Passed, Pending, Confirmed }

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum SpeedSource { Crank, Cam }

#[derive(Copy, Clone, Debug)]
pub struct FaultEntry {
    pub state: DtcState,
    pub fail_count: i32,
    pub freeze_rpm: f64,
    pub freeze_time_s: f64,
}

impl FaultEntry {
    pub const CLEAR: Self = FaultEntry {
        state: DtcState::Passed, fail_count: 0,
        freeze_rpm: 0.0, freeze_time_s: 0.0,
    };
}

/// Three-way vote: crank, cam, and the ECU's own model.
///
/// Tolerance is dynamic. The cam's disagreement during a transient is predictable - it fires every 180 crank degrees,
/// so its reading is stale by about two periods (2 x 30/N seconds). Rather than blinding the monitor during transients,
/// widen the threshold by exactly the error the lag accounts for. A genuine drift is slow, so it barely widens at all;
/// a 6000 rpm/s load step widens it hugely
pub struct SpeedPlausibility {
    pub threshold_rpm: f64,     // static tolerance
    pub freeze_frac: f64,       // fraction of tolerance that freezes the observer
    pub confirm_count: i32,
    last_n: f64,
    last_t: Option<revlab_core::SimTime>,
    dn_dt: f64,                 // rpm/s, filtered
}

impl Default for SpeedPlausibility {
    fn default() -> Self {
        SpeedPlausibility {
            threshold_rpm: 40.0,
            freeze_frac: 0.25,
            confirm_count: 30,
            last_n: 0.0,
            last_t: None,
            dn_dt: 0.0,
        }
    }
}

impl SpeedPlausibility {
    fn debounce(e: &mut FaultEntry, bad: bool, limit: i32, n: f64, t: f64) -> bool {
        e.fail_count = if bad {
            (e.fail_count + 1).min(limit)
        } else {
            (e.fail_count - 1).max(0)
        };
        if e.fail_count >= limit && e.state != DtcState::Confirmed {
            e.state = DtcState::Confirmed;
            e.freeze_rpm = n;
            e.freeze_time_s = t;
            return true;                    // newly confirmed
        }
        if bad && e.state == DtcState::Passed {
            e.state = DtcState::Pending;
        }
        if !bad && e.fail_count == 0 && e.state == DtcState::Pending {
            e.state = DtcState::Passed;
        }
        false
    }

    /// Two cam periods worth of speed change, in rpm
    fn lag_allowance(&self, n_eng: f64) -> f64 {
        let period = 30.0 / n_eng.max(100.0);
        2.0 * period * self.dn_dt.abs()
    }
}

impl Task for SpeedPlausibility {
    fn name(&self) -> &'static str { "SpeedPlausibility" }

    fn run(&mut self, s: &mut EcuState) {
        // Cannot compare signals that are not there. A stopped engine or an open circuit is not a correlation fault
        if !s.crank_valid || !s.cam_valid {
            s.freeze_adaptation = true;
            return;
        }
        
        // --- rate of change, lightly filtered
        if let Some(t0) = self.last_t {
            let dt = (s.now - t0).as_secs_f64();
            if dt > 1e-6 {
                let raw = (s.n_eng - self.last_n) / dt;
                self.dn_dt += 0.3 * (raw - self.dn_dt);
            }
        }
        self.last_t = Some(s.now);
        self.last_n = s.n_eng;

        let tol = self.threshold_rpm + self.lag_allowance(s.n_eng);
        let delta = (s.n_crank - s.n_cam).abs();

        // Freeze well before the fault threshold: the observer's correction pulls toward the CONTROL sensor,
        // so a fault there contaminates the model meant to arbitrate it.
        s.freeze_adaptation = delta > tol * self.freeze_frac;
        let disagree = delta > tol;

        let now = s.now.as_secs_f64();
        let n = s.n_eng;

        if !disagree || !s.model_valid {
            Self::debounce(&mut s.fault_mem[Sensor::Crank as usize], false, self.confirm_count, n, now);
            Self::debounce(&mut s.fault_mem[Sensor::Cam as usize], false, self.confirm_count, n, now);
            s.unattributable = false;
            return;
        }

        let d_crank = (s.n_crank - s.n_model).abs();
        let d_cam   = (s.n_cam   - s.n_model).abs();

        // Comparative, not absolute: attribute only when one sensor is decisively closer to the model.
        let decisive = (d_crank - d_cam).abs() > tol * self.freeze_frac;
        let unattributable = !decisive;

        let crank_bad = !unattributable && d_crank > d_cam;
        let cam_bad   = !unattributable && d_cam > d_crank;

        if Self::debounce(&mut s.fault_mem[Sensor::Crank as usize], crank_bad, self.confirm_count, n, now) {
            s.speed_source = SpeedSource::Cam;  // substitute
        }
        Self::debounce(&mut s.fault_mem[Sensor::Cam as usize], cam_bad, self.confirm_count, n, now);
        // Cam confirmed faulty: stay on crank. No substitution - that is the whole point of attributing the fault

        s.unattributable = unattributable;
    }
}

/// Torque ceiling while any confirmed fault is present
pub struct LimpMode { pub torque_max: f64 }

impl Task for LimpMode {
    fn name(&self) -> &'static str { "LimpMode" }

    fn run(&mut self, s: &mut EcuState) {
        s.degraded = s.fault_mem.iter()
            .any(|e| e.state == DtcState::Confirmed) || s.unattributable;
        s.reqs[Source::Protect as usize] = TorqueRequest {
            kind: ReqKind::MaxLimit,
            value: self.torque_max,
            active: s.degraded,
        };
    }
}