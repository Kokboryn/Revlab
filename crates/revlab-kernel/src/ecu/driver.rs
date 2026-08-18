use super::torque::{ReqKind, Source, TorqueRequest};
use super::{EcuState, Task};

/// Driver demand: pedal position -> crank torque target.
///
/// At closed pedal the target is NEGATIVE - overrun. That is what lets the idle governor's MinLimit
/// act as a floor rather than the only request, and it is why a real engine slows when you lift off
pub struct DriverDemand {
    pub t_max: f64,         // Nm at full pedal
    pub t_overrun: f64,     // Nm at closed pedal (negative)
}

impl DriverDemand {
    pub fn di_diesel_1_6() -> Self {
        DriverDemand { t_max: 250.0, t_overrun: -40.0 }
    }
}

impl Task for DriverDemand {
    fn name(&self) -> &'static str { "DriverDemand" }
    
    fn run(&mut self, s: &mut EcuState) {
        let pedal = s.pedal.clamp(0.0, 1.0);
        let t = self.t_overrun + pedal * (self.t_max - self.t_overrun);
        s.reqs[Source::Driver as usize] = TorqueRequest {
            kind: ReqKind::Target,
            value: t,
            active: true,
        };
    }
}