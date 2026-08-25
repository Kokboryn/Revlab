use revlab_core::Map1d;
use super::{EcuState, Task};

/// ECT derived friction correction, published for every model that needs it. Own task because it depends
/// only on coolant temperature, so it can run at the top of the table where consumers get a fresh value
pub struct WarmupComp(pub Map1d);

impl WarmupComp {
    pub fn di_diesel_1_6() -> Self {
        // ECT indexed friction correction, measured against the plant across a full warmup: the multiplier
        // the governor actually needed was 1.734 at 20 C, 1.445 at 38, 1.225 at 71, 1.139 at 86. The curve
        // is convex, so the earlier straight line was over by 0.12 in the middle and under by 0.10 at the
        // top. The 100 C point is extrapolated -- nominal idle plateaus at 85 and never reaches it.
        WarmupComp(Map1d::new(
            vec![20.0, 38.0, 71.0, 86.0, 100.0],
            vec![1.734, 1.445, 1.225, 1.139, 1.060],
        ))
    }
}

impl Task for WarmupComp {
    fn name(&self) -> &'static str { "WarmupComp" }
    fn run(&mut self, s: &mut EcuState) {
        s.warmup_mult = self.0.get(s.t_ect_c);
    }
}

/// Crank torque -> indicated torque
///
/// Torque requests are made at the CRANK, because that is what the driver and the driveline care about.
/// Combustion has to produce that plus everything lost on the way out: friction, pumping, accessories.
///
/// The ECU's loss model is its own fitted approximation, not the plant's Chen-Flynn correlation.
/// It's wrong, and closed-loop control covers the difference.
pub struct LossModel {
    pub fric_a: f64,        // Nm, constant
    pub fric_b: f64,        // Nm.s/rad, speed dependent
    pub accessory: f64,     // Nm, alternator/pump/AC
}

impl LossModel {
    pub fn di_diesel_1_6() -> Self {
        // Base fit is the warm engine friction at 800 rpm; WarmupComp scales it by an ECT indexed 
        // multiplier that does not reach unity in the operating range.
        LossModel {
            fric_a: 11.0,
            fric_b: 0.021,
            accessory: 3.0,
        }
    }
}

impl Task for LossModel {
    fn name(&self) -> &'static str { "LossModel" }
    
    fn run(&mut self, s: &mut EcuState) {
        let omega = s.n_eng * std::f64::consts::PI / 30.0;
        let base = self.fric_a + self.fric_b * omega + self.accessory;
        s.t_loss = base * s.warmup_mult;
        s.t_ind_req = s.t_arb + s.t_loss;
    }
}