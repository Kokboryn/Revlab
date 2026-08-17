use super::{EcuState, Task};

/// Crank torque -> indicated torque
///
/// Torque requests are made at the CRANK, because that is what the driver and the driveline care about.
/// Combustion has to produce that plus everything lost on the way out;: friction, pumping, accessories.
///
/// The ECU's loss model is its own fitted approximation, not the plant's Chen-Flynn correlation.
/// It's wrong and closed-loop control covers the difference.
pub struct LossModel {
    pub fric_a: f64,        // Nm, constant
    pub fric_b: f64,        // Nm.s/rad, speed dependent
    pub accessory: f64,     // Nm, alternator/pump/AC
}

impl LossModel {
    pub fn di_diesel_1_6() -> Self {
        // Fitted at idle to ~15.8 Nm, which is what the ECU's torque model believes it makes at 6.1 mg/stroke
        LossModel { fric_a: 11.0, fric_b: 0.021, accessory: 3.0 }
    }
}

impl Task for LossModel {
    fn name(&self) -> &'static str { "LossModel" }
    
    fn run(&mut self, s: &mut EcuState) {
        let omega = s.n_eng * std::f64::consts::PI / 30.0;
        s.t_loss = self.fric_a + self.fric_b * omega + self.accessory;
        s.t_ind_req = s.t_arb + s.t_loss;
    }
}