use super::torque_model::TorqueModel;
use super::{EcuState, Task};

/// Fixed slot per producer - no allocation, and the arbiter's input set is known at compile time, as on real firmware
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Source { Idle = 0, Driver = 1, Smoke = 2, Protect = 3, RevLimit = 4, Tcu = 5 }
pub const N_SOURCES: usize = 6;

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

/// One arbitration pass over the request slots, optionally leaving one source out
fn arbitrate(reqs: &[TorqueRequest], skip: Option<Source>) -> f64 {
    let act = |k :ReqKind| -> Option<f64> {
        reqs.iter().enumerate()
            .filter(|(i, r)| r.active && r.kind == k && skip.map_or(true, |x| *i != x as usize))
            .map(|(_, r)| r.value)
            .fold(None, |acc: Option<f64>, v| Some(match acc {
                None => v,
                Some(a) if k == ReqKind::MaxLimit => a.min(v),
                Some(a) => a.max(v)
            }))
    };
    // Order is load-bearing: targets, then minimum guarantees, then maximum limits LAST so protection
    // can never be overridden
    let mut t = act(ReqKind::Target).unwrap_or(0.0);
    if let Some(lo) = act(ReqKind::MinLimit) { t = t.max(lo) }
    if let Some(hi) = act(ReqKind::MaxLimit) { t = t.min(hi) }
    t
}

/// Selects one torque from all pending requests
pub struct TorqueArbiter;

impl Task for TorqueArbiter {
    fn name(&self) -> &'static str { "TorqueArbiter" }

    fn run(&mut self, s: &mut EcuState) {
        s.t_arb = arbitrate(&s.reqs, None);
        // What the engine would make without the TCU's intervention, every other limit still applied.
        // The TCU sizes its own requests from it, so a shift follows the driver's torque instead of
        // a figure frozen when the shift began
        s.t_no_icpt = arbitrate(&s.reqs, Some(Source::Tcu))
    }
}

/// Inverse torque model: arbitrated torque -> fuel quantity, through the ECU's calibrated torque model.
/// Also reports what the engine actually makes on that fuel, which is what goes out on CAN
pub struct TorqueToFuel {
    pub model: TorqueModel,
}

impl TorqueToFuel {
    pub fn ea288() -> Self { TorqueToFuel { model: TorqueModel::ea288_cal() } }
}

impl Task for TorqueToFuel {
    fn name(&self) -> &'static str { "TorqueToFuel" }

    fn run(&mut self, s: &mut EcuState) {
        s.q_cmd = self.model.q_for(s.n_eng, s.t_ind_req);
        // What the engine actually delivers once the fuel has been clamped, net of the ECU's friction
        // estimate. This, not the request, is what a real ECU broadcasts as actual torque: fuel cut can't
        // make negative indicated torque, and the smoke and injector limits take the top off
        s.t_act = self.model.t_ind(s.n_eng, s.q_cmd) - s.t_loss;
    }
}

/// Torque intervention from the TCU (Gearbox_1). During a shift's inertia phase the TCU asks for a cut
/// (a ceiling) on upshifts and a raise (a floor) on downshifts, so the engine moves its own inertia and
/// the clutch carries a steady torque. Max limits are applied last, so protection still wins
pub struct TcuIntervention;

impl Task for TcuIntervention {
    fn name(&self) -> &'static str { "TcuIntervention" }
    
    fn run(&mut self, s: &mut EcuState) {
        s.reqs[Source::Tcu as usize] = match s.tcu_icpt_kind {
            1 => TorqueRequest { kind: ReqKind::MaxLimit, value: s.tcu_icpt_t, active: true },
            2 => TorqueRequest { kind: ReqKind::MinLimit, value: s.tcu_icpt_t, active: true },
            _ => TorqueRequest::INACTIVE,
        };
    }
}