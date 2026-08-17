use super::torque::{ReqKind, Source, TorqueRequest};
use super::{EcuState, Task};

/// Rev limiter as a torque ceiling ramped over a band, not a hard fuel cut. Modern diesels do it
/// this way - a hard cut is violent on the driveline and the resulting oscillation is worse than
/// the overspeed
pub struct RevLimiter {
    pub n_soft: f64,    // rpm - start pulling torque
    pub n_hard: f64,    // rpm - zero torque
    pub t_max: f64,     // Nm, ceiling below n_soft
}

impl RevLimiter {
    /// EA288 cuts around 4500-4800
    pub fn ea288() -> Self {
        RevLimiter { n_soft: 4300.0, n_hard: 4700.0, t_max: 250.0 }
    }
}

impl Task for RevLimiter {
    fn name(&self) -> &'static str { "RevLimiter" }
    
    fn run(&mut self, s: &mut EcuState) {
        if s.n_eng <= self.n_soft {
            s.reqs[Source::RevLimit as usize] = TorqueRequest::INACTIVE;
            return;
        }
        let frac = ((self.n_hard - s.n_eng) / (self.n_hard - self.n_soft)).clamp(0.0, 1.0);
        
        // Ceiling is expressed at the CRANK, like every other request. Above n_hard this goes negative,
        // which is correct: the only way to stop an engine already over the limit is to command fuel cut,
        // not merely zero torque.
        let ceiling = self.t_max * frac - (1.0 - frac) * s.t_loss;
        
        s.reqs[Source::RevLimit as usize] = TorqueRequest {
            kind: ReqKind::MaxLimit,
            value: ceiling,
            active: true,
        };
    }
}