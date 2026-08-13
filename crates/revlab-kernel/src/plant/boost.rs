use revlab_core::SimDuration;
use crate::{Component, Ctx, Port, Trigger};

/// Placeholder for the compressor: supplies whatever flow holds the manifold at a target pressure.
/// Stands in for the naturally aspirated case, where the inlet is effectively unrestricted and the manifold sits at ambient.
/// Replaced by Turbo, which occupies the same socket.
pub struct FixedBoost {
    pub p_target: f64,      // Pa
    pub gain: f64,          // kg/(s·Pa)
    p_im: Port,
    m_dot_eng: Port,
    m_comp_out: Port,
}

impl FixedBoost {
    pub const STEP: SimDuration = SimDuration::from_millis(1);

    pub fn new(p_target: f64, p_im: Port, m_dot_eng: Port, m_comp_out: Port) -> Self {
        FixedBoost {
            p_target,
            gain: 1.0e-5,
            p_im, m_dot_eng, m_comp_out,
        }
    }
}

impl Component for FixedBoost {
    fn triggers(&self) -> Vec<Trigger> {
        vec![Trigger::Periodic { period: Self::STEP, offset: SimDuration::from_micros(250) }]
    }

    fn step(&mut self, _t: u16, ctx: &mut Ctx<'_>) {
        // Feedforward engine demand, then correct the residual error.
        let err = self.p_target - ctx.bus.get(self.p_im);
        let m = ctx.bus.get(self.m_dot_eng) + self.gain * err;
        ctx.bus.set(self.m_comp_out, m.max(0.0));
    }
}