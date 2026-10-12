use super::torque::{ReqKind, Source, TorqueRequest};
use super::{EcuState, Task};

/// Driver demand: pedal position -> crank torque target.
///
/// At closed pedal the target is NEGATIVE - overrun. That is what lets the idle governor's MinLimit
/// act as a floor rather than the only request, and it is why a real engine slows when you lift off.
/// Below n_cut the drag fades out (a dashpot), so the engine arrives at idle rather than falling through
/// it.
pub struct DriverDemand {
    pub t_max: f64,         // Nm at full pedal
    pub t_overrun: f64,     // Nm at closed pedal (negative)
    pub n_cut: f64,         // rpm, overrun drag is full above this and fades to zero at the idle target
}

impl DriverDemand {
    pub fn di_diesel_1_6() -> Self {
        DriverDemand { t_max: 250.0, t_overrun: -40.0, n_cut: 1300.0 }
    }
}

impl Task for DriverDemand {
    fn name(&self) -> &'static str { "DriverDemand" }
    
    fn run(&mut self, s: &mut EcuState) {
        let pedal = s.pedal.clamp(0.0, 1.0);
        // Overrun fuel cut holds full drag only well above idle. Below n_cut the drag fades to nothing
        // at the idle target, so a freed engine settles onto idle instead of falling through it. Fuel
        // comes back where the faded drag no longer covers friction, around 1100 rpm
        let span = (self.n_cut - s.target_rpm).max(1.0);
        let fade = ((s.n_eng - s.target_rpm) / span).clamp(0.0, 1.0);
        let t_closed = self.t_overrun * fade;
        let t = t_closed + pedal * (self.t_max - t_closed);
        s.reqs[Source::Driver as usize] = TorqueRequest {
            kind: ReqKind::Target,
            value: t,
            active: true,
        };
    }
}