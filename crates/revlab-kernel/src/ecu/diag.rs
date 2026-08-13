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
/// The previous two-signal version could detect disagreement but not attribute it, so it always substituted cam - and discarded a healthy crank signal whenever cam was the faulty one. The model breaks the tie.
pub struct SpeedPlausibility {
    pub threshold_rpm: f64,     // 40.0 - fault
    pub freeze_rpm: f64,        // 10.0 - stop trusting the correction
    pub confirm_count: i32,
}

impl Default for SpeedPlausibility {
    fn default() -> Self {
        SpeedPlausibility { threshold_rpm: 40.0, confirm_count: 30, freeze_rpm: 10.0 }
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
        if !bad && e.fail_count == 0 && e.state == DtcState::Pending {
            e.state = DtcState::Passed;
        }
        false
    }
}

impl Task for SpeedPlausibility {
    fn name(&self) -> &'static str { "SpeedPlausibility" }

    fn run(&mut self, s: &mut EcuState) {
        let delta = (s.n_crank - s.n_cam).abs();

        // Freeze well before the fault threshold. The correction pulls the model toward the CONTROL sensor, so a fault in that sensor contaminates the very model meant to arbitrate it.
        // Going open-loop at the first hint keeps the model physics dominated across the detection window.
        s.freeze_adaptation = delta > self.freeze_rpm;
        let disagree = delta > self.threshold_rpm;

        if !disagree || !s.model_valid {
            let now = s.now.as_secs_f64();
            let n = s.n_eng;
            Self::debounce(&mut s.fault_mem[Sensor::Crank as usize], false, self.confirm_count, n, now);
            Self::debounce(&mut s.fault_mem[Sensor::Cam as usize], false, self.confirm_count, n, now);
            return;
        }

        let d_crank = (s.n_crank - s.n_model).abs();
        let d_cam   = (s.n_cam   - s.n_model).abs();

        // Comparative, not absolute: attribute only when one sensor is decisively closer to the mode. Both merely being far away means nothing is trustworthy.
        let decisive = (d_crank - d_cam).abs() > self.freeze_rpm;
        let unattributable = !decisive;

        let now = s.now.as_secs_f64();
        let n = s.n_eng;
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