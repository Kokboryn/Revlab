use std::f64::consts::PI;
use super::{EcuState, Task};

/// Fixed slot per producer - no allocation, and the arbiter's input set is known at compile time, as on real firmware
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Source { Idle = 0, Driver = 1, Smoke = 2, Protect = 3, RevLimit = 4 }
pub const N_SOURCES: usize = 5;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ReqKind {
    /// A setpoint. Highest target wins.
    Target,
    /// "at least this much" - idle speed control, catalyst heating
    MinLimit,
    /// "no more than this" - smoke limit, component protection, TCU shift torque reduction
    MaxLimit,
}

#[derive(Clone, Debug)]
pub struct TorqueRequest {
    pub kind: ReqKind,
    pub value: f64,     // Nm at the crank
    pub active: bool,
}

impl TorqueRequest {
    pub const INACTIVE: Self = TorqueRequest { kind: ReqKind::Target, value: 0.0, active: false };
}

/// Selects one torque from all pending requests
pub struct TorqueArbiter;

impl Task for TorqueArbiter {
    fn name(&self) -> &'static str { "TorqueArbiter" }

    fn run(&mut self, s: &mut EcuState) {
        let act = |k: ReqKind, s: &EcuState| -> Option<f64> {
            s.reqs.iter()
                .filter(|r| r.active && r.kind == k)
                .map(|r| r.value)
                .fold(None, |acc: Option<f64>, v| Some(match acc {
                    None => v,
                    Some(a) if k == ReqKind::MaxLimit => a.min(v),
                    Some(a) => a.max(v),
                }))
        };

        // Order is load-bearing: targets, then minimum guarantees, then maximum limits LAST so protection can never be overridden.
        let mut t = act(ReqKind::Target, s).unwrap_or(0.0);
        if let Some(lo) = act(ReqKind::MinLimit, s) { t = t.max(lo) }
        if let Some(hi) = act(ReqKind::MaxLimit, s) { t = t.min(hi) }
        s.t_arb = t;
    }
}

/// Inverse torque model: arbitrated torque -> fuel quantity
///
/// Deliberately approximate. The ECU has no access to the plant's efficiency map or friction correlation. So eta_cal is a single calibrated constant and the resulting bias is what closed look control exists to absorb
pub struct TorqueToFuel {
    pub eta_cal: f64,
    pub lhv_cal: f64,
    pub cylinders: f64,
    pub q_max: f64,
}

impl TorqueToFuel {
    pub fn di_diesel(cylinders: f64) -> Self {
        TorqueToFuel { eta_cal: 0.19, lhv_cal: 42.7e6, cylinders, q_max: 60.0 }
    }
}

impl Task for TorqueToFuel {
    fn name(&self) -> &'static str { "TorqueToFuel" }

    fn run(&mut self, s: &mut EcuState) {
        // q = T · 4π / (n_cyl · LHV · η). in mg/stroke
        let denom = self.cylinders * self.lhv_cal * self.eta_cal;
        let q = s.t_ind_req * 4.0 * PI / denom * 1e6;
        s.q_cmd = q.clamp(0.0, self.q_max);
    }
}