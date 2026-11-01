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

/// Puts the crank on the cam's time base. A cam reading is the mean speed over its last lobe interval
/// (30/N s: 13.5 ms at idle) and then holds until the next edge, so against the crank's live reading
/// it is up to 1.5 intervals late, hundreds of rpm between two healthy sensors on a tip-in. Averaged
/// over the same window the two agree to within noise, so the monitor compares  like with like instead
/// of widening its tolerance by the lag. Runs every 1 ms, so an edge is never more than 1 ms old here
pub struct CamWindow {
    hist: [(f64, f64); 128],    // (crank, model) once per ms, ring: covers a cam window down to ~235 rpm
    head: usize,
    filled: usize,
    last_cam: f64,
    primed: bool,
}

impl CamWindow {
    pub fn new() -> Self {
        CamWindow { hist: [(0.0, 0.0); 128], head: 0, filled: 0, last_cam: 0.0, primed: false }
    }
}

impl Task for CamWindow {
    fn name(&self) -> &'static str { "CamWindow" }

    fn run(&mut self, s: &mut EcuState) {
        let len_hist = self.hist.len();
        self.hist[self.head] = (s.n_crank, s.n_model);
        self.head = (self.head + 1) % len_hist;
        self.filled = (self.filled + 1).min(len_hist);
        if !self.primed {
            s.n_crank_camwin = s.n_crank;
            s.n_model_camwin = s.n_model;
            s.n_cam_camwin = s.n_cam;
            self.last_cam = s.n_cam;
            self.primed = true;
            return;
        }
        // Every cam edge carries fresh noise, so a reading that hasn't changes is no new edge
        if s.n_cam == self.last_cam { return; }
        self.last_cam = s.n_cam;
        // The reading covers the interval that ended at this edge, and its length follows from the reading
        let len = ((30.0 / s.n_cam.max(1.0)) * 1000.0).round() as usize;
        let len = len.clamp(1, self.filled);
        let (mut c, mut m) = (0.0, 0.0);
        for i in 1..=len {
            let (ci, mi) = self.hist[(self.head + len_hist - i) % len_hist];
            c += ci;
            m += mi;
        }
        s.n_crank_camwin = c / len as f64;
        s.n_model_camwin = m / len as f64;
        // The pair travels together: a check that runs between a cam edge and this task must not compare
        // the new reading aa=gainst the previous edge's window
        s.n_cam_camwin = s.n_cam;
    }
}

impl FaultEntry {
    pub const CLEAR: Self = FaultEntry {
        state: DtcState::Passed, fail_count: 0,
        freeze_rpm: 0.0, freeze_time_s: 0.0,
    };
}

/// Three-way vote: crank, cam, and the ECU's own model.
///
/// The cam is compared against the crank averaged over the cam's own window (CamWindow), so a transient
/// doesn't open a gap between two healthy sensors. Only the timing slop left after matching, edge
/// quantization and the crank's tooth windows, widens the tolerance, so the monitor stays sharp through
/// a load step instead of going blind for it
pub struct SpeedPlausibility {
    pub threshold_rpm: f64,     // static tolerance
    pub t_resid: f64,           // s, timing left after matching windows, as rpm per rpm/s
    pub freeze_frac: f64,       // fraction of tolerance that freezes the observer
    pub confirm_count: i32,
    last_n: f64,
    last_t: Option<revlab_core::SimTime>,
    dn_dt: f64,                 // rpm/s, filtered
    pub stale_s: f64,           // s, a reading frozen this long is a dead capture
    last_run: Option<revlab_core::SimTime>,
    last_crank: f64,
    last_cam: f64,
    crank_still_s: f64,
    cam_still_s: f64,
}

impl Default for SpeedPlausibility {
    fn default() -> Self {
        SpeedPlausibility {
            threshold_rpm: 40.0,
            t_resid: 0.004,
            freeze_frac: 0.25,
            confirm_count: 30,
            last_n: 0.0,
            last_t: None,
            dn_dt: 0.0,
            // A healthy crank never repeats a reading for more than 10 ms; the cam holds up to 40 ms
            // at idle, since it only updates every 180 degrees
            stale_s: 0.15,
            last_run: None,
            last_crank: -1.0,
            last_cam: -1.0,
            crank_still_s: 0.0,
            cam_still_s: 0.0,
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

    /// What the matched windows still can't line up, in rpm
    fn lag_allowance(&self) -> f64 {
        self.t_resid * self.dn_dt.abs()
    }
}

impl Task for SpeedPlausibility {
    fn name(&self) -> &'static str { "SpeedPlausibility" }

    fn run(&mut self, s: &mut EcuState) {
        let dt_run = self.last_run.map(|t0| (s.now - t0).as_secs_f64()).unwrap_or(0.0);
        self.last_run = Some(s.now);

        // --- capture freshness. A healthy speed signal is re-measured every edge and never repeats
        // bit for bit; one that stops changing is a dead capture, whatever its valid flag says
        self.crank_still_s = if s.n_crank == self.last_crank { self.crank_still_s + dt_run } else { 0.0 };
        self.cam_still_s = if s.n_cam == self.last_cam { self.cam_still_s + dt_run } else { 0.0 };
        self.last_crank = s.n_crank;
        self.last_cam = s.n_cam;
        let crank_ok = s.crank_valid && self.crank_still_s < self.stale_s;
        let cam_ok = s.cam_valid && self.cam_still_s < self.stale_s;

        // Both gone at once is a stopped engine, not a sensor fault
        if !crank_ok && !cam_ok {
            s.freeze_adaptation = true;
            return;
        }

        // One gone: losing the signal is its own evidence. Debounce it as that sensor's fault
        let (now, n) = (s.now.as_secs_f64(), s.n_eng);
        if !crank_ok || !cam_ok {
            if Self::debounce(&mut s.fault_mem[Sensor::Crank as usize], !crank_ok, self.confirm_count, n, now) {
                s.speed_source = SpeedSource::Cam;
            }
            Self::debounce(&mut s.fault_mem[Sensor::Cam as usize], !cam_ok, self.confirm_count, n, now);
        }

        // A confirmed fault has been acted on: the trusted sensor is chosen, so the observer may follow
        // it again. Re-voting against a condemned sensor would freeze the model for good and let it
        // wander off
        if s.fault_mem.iter().any(|e| e.state == DtcState::Confirmed) {
            s.freeze_adaptation = false;
            s.unattributable = false;
            return;
        }
        if !crank_ok || !cam_ok {
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

        let tol = self.threshold_rpm + self.lag_allowance();
        let delta = (s.n_crank_camwin - s.n_cam_camwin).abs();

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

        // Each sensor against the model over the same time it measured: crank and model live, cam and
        // model over the cam's window
        let d_crank = (s.n_crank - s.n_model).abs();
        let d_cam   = (s.n_cam_camwin   - s.n_model_camwin).abs();

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