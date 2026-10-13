use revlab_core::Map1d;
use super::{EcuState, Task};

/// ECT derived friction correction, published for every model that needs it. Own task because it depends
/// only on coolant temperature, so it can run at the top of the table where consumers get a fresh value
pub struct WarmupComp(pub Map1d);

impl WarmupComp {
    pub fn di_diesel_1_6() -> Self {
        // ECT indexed friction multiplier, fitted to the engine's measured friction across a warmup:
        // 2.20 at 20 C down to 1.43 at 70. The 86 and 100 C points are extrapolated along the flattening
        // curve; a warm run will show how close they are. The earlier map was fitted to what the idle
        // governor needed, which also soaked up the friction slope and efficiency errors
        WarmupComp(Map1d::new(
            vec![20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 86.0, 100.0],
            vec![2.20, 1.87, 1.75, 1.62, 1.53, 1.43, 1.30, 1.22],
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
    pub fric_c: f64,        // Nm.s²/rad², grows with speed squared, as Chen-Flynn friction does
    pub accessory: f64,     // Nm, alternator/pump/AC
}

impl LossModel {
    pub fn di_diesel_1_6() -> Self {
        // Warm friction fitted to the engine across 1000-4500 rpm: quadratic in speed, as the correlation
        // it comes from is. The plant has no accessory load yet, so neither does the model
        LossModel {
            fric_a: 15.9,
            fric_b: 0.0133,
            fric_c: 4.1e-5,
            accessory: 0.0,
        }
    }
}

impl Task for LossModel {
    fn name(&self) -> &'static str { "LossModel" }
    
    fn run(&mut self, s: &mut EcuState) {
        let omega = s.n_eng * std::f64::consts::PI / 30.0;
        let base = self.fric_a + self.fric_b * omega + self.fric_c * omega * omega + self.accessory;
        s.t_loss = base * s.warmup_mult;
        s.t_ind_req = s.t_arb + s.t_loss;
    }
}