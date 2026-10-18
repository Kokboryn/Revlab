use super::torque::{ReqKind, Source, TorqueRequest};
use super::{EcuState, Task};
use revlab_core::Map1d;

/// Driver demand: pedal position -> crank torque target.
///
/// At closed pedal the target is NEGATIVE - overrun. That is what lets the idle governor's MinLimit
/// act as a floor rather than the only request, and it is why a real engine slows when you lift off.
/// Below n_cut the drag fades out (a dashpot), so the engine arrives at idle rather than falling through
/// it.
pub struct DriverDemand {
    pub t_max_curve: Map1d, // Full load torque at the crank over rpm: the engine's rated curve
    pub t_overrun: f64,     // Nm at closed pedal (negative)
    pub n_cut: f64,         // rpm, overrun drag is full above this and fades to zero at the idle target
    pub p_drag: f64,        // pedal over which overrun drag blends out; above it the request is positive
}

impl DriverDemand {
    pub fn di_diesel_1_6() -> Self {
        DriverDemand { 
            // 250 Nm plateau, then 88 kW constant power (DCXA rating)
            t_max_curve: Map1d::new(
                vec![1000.0, 1500.0, 1750.0, 3300.0, 3600.0, 4000.0, 4500.0],
                vec![180.0, 240.0, 250.0, 250.0, 233.4, 210.1, 185.0],
        ), t_overrun: -40.0, n_cut: 1300.0, p_drag: 0.04 }
    }
}

impl Task for DriverDemand {
    fn name(&self) -> &'static str { "DriverDemand" }
    
    fn run(&mut self, s: &mut EcuState) {
        let t_max = self.t_max_curve.get(s.n_eng);
        let pedal = s.pedal.clamp(0.0, 1.0);
        // Overrun fuel cut holds full drag only well above idle. Below n_cut the drag fades to nothing
        // at the idle target, so a freed engine settles onto idle instead of falling through it. Fuel
        // comes back where the faded drag no longer covers friction, around 1100 rpm
        let span = (self.n_cut - s.target_rpm).max(1.0);
        let fade = ((s.n_eng - s.target_rpm) / span).clamp(0.0, 1.0);
        let t_closed = self.t_overrun * fade;
        // The first few percent of travel blend the overrun drag out; above that the pedal asks for its
        // share of full torque. A straight line from overrun to full spends the first seventh of the
        // pedal cancelling drag, which makes light throttle coast
        let t = if pedal < self.p_drag {
            t_closed * (1.0 - pedal / self.p_drag)
        } else {
            t_max * (pedal - self.p_drag) / (1.0 - self.p_drag)
        };
        s.reqs[Source::Driver as usize] = TorqueRequest {
            kind: ReqKind::Target,
            value: t,
            active: true,
        };
    }
}