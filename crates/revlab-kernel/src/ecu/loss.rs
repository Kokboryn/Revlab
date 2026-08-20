use super::{EcuState, Task};

/// ECT derived friction correction, published for every model that needs it. Own task because it depends
/// only on coolant temperature, so it can run at the top of the table where consumers get a fresh value
pub struct WarmupComp { pub slope: f64, pub ref_c: f64 }

impl WarmupComp {
    // warmup_slope fitted from the same run: t_ind_req/t_loss was 1.74 at 20 C, 1.47 at 39 C, 1.24 at 66 C -- linear
    // at -0.0109 per degC, hitting 1.00 at 90 C.
    pub fn di_diesel_1_6() -> Self { WarmupComp { slope: 0.0109, ref_c: 90.0 } }
}

impl Task for WarmupComp {
    fn name(&self) -> &'static str { "WarmupComp" }

    fn run(&mut self, s: &mut EcuState) {
        s.warmup_mult = 1.0 + self.slope * (self.ref_c - s.t_ect_c).max(0.0);
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
        // Base fit is a WARM engine: at the 90 C reference WarmupComp returns unity.
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