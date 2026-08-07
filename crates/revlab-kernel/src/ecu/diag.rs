use super::{EcuState, Task};

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Dtc { P0016CrankCamCorrelation }

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum DtcState { Passed, Pending, Confirmed }

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum SpeedSource { Crank, Cam }

/// Fault memory entry. Real ECUs debounce before confirming and heal slowly afterward, so a single noisy sample never lights a lamp
#[derive(Copy, Clone, Debug)]
pub struct FaultEntry {
    pub state: DtcState,
    pub fail_count: i32,
    pub freeze_rpm: f64,        // freeze-frame: conditions at confirmation
    pub freeze_time_s: f64,
}

impl FaultEntry {
    pub const CLEAR: Self = FaultEntry {
        state: DtcState::Passed, fail_count: 0, freeze_rpm: 0.0, freeze_time_s: 0.0,
    };
}

pub struct SpeedPlausibility {
    pub threshold_rpm: f64,
    pub confirm_count: i32,
}

impl Default for SpeedPlausibility {
    fn default() -> Self {
        // Cam is coarse, so the threshold must clear its own noise floor
        SpeedPlausibility { threshold_rpm: 40.0, confirm_count: 30 }
    }
}

impl Task for SpeedPlausibility {
    fn name(&self) -> &'static str { "SpeedPlausibility" }

    fn run(&mut self, s: &mut EcuState) {
        let e = &mut s.fault_mem;
        let bad = (s.n_crank - s.n_cam).abs() > self.threshold_rpm;

        e.fail_count = if bad {
            (e.fail_count + 1).min(self.confirm_count)
        } else {
            (e.fail_count - 1).max(0)
        };

        if e.fail_count >= self.confirm_count && e.state != DtcState::Confirmed {
            e.state = DtcState::Confirmed;
            e.freeze_rpm = s.n_crank;
            e.freeze_time_s = s.now.as_secs_f64();
            // Substitute the surviving signal. Degraded not dead
            s.speed_source = SpeedSource::Cam;
        } else if bad && e.state == DtcState::Passed {
            e.state = DtcState::Pending;
        }
    }
}