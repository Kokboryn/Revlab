use revlab_core::SimDuration;
use crate::{Component, Ctx, Port, Trigger};
use super::gas::{R_AIR};

#[derive(Copy, Clone)]
pub struct IntakePorts {
    // inputs
    pub t_up: Port,
    pub m_dot_eng: Port,    // written by Engine - last step's consumption
    pub m_comp_in: Port,
    // outputs
    pub p: Port,
    pub t: Port,
    pub m_dot_in: Port,     // what a MAF would see
}

/// Intake manifold as a filling and emptying volume.
///
/// dp/dt = (R·T/V)·(ṁ_in − ṁ_out)
///
/// Time constant is V/(ṁ·R·T/p) - a few tens of ms at idle, much faster at high flow. This is what makes manifold pressure lag throttle or boost changes
pub struct IntakeManifold {
    p: f64,             // Pa
    t: f64,             // K
    pub volume: f64,    // m³, plenum + runners
    ports: IntakePorts,
    dt: f64,
}

impl IntakeManifold {
    pub const STEP: SimDuration = SimDuration::from_millis(1);

    pub fn new(volume: f64, ports: IntakePorts, p_init: f64, t_init: f64) -> Self {
        IntakeManifold { p: p_init, t: t_init, volume, ports, dt: Self::STEP.as_secs_f64(), }
    }
}

impl Component for IntakeManifold {
    fn triggers(&self) -> Vec<Trigger> {
        vec![Trigger::Periodic { period: Self::STEP, offset: SimDuration::from_micros(300) }]
    }

    fn step(&mut self, _t: u16, ctx: &mut Ctx<'_>) {
        let t_up = ctx.bus.get(self.ports.t_up);
        let m_out = ctx.bus.get(self.ports.m_dot_eng);
        let m_in = ctx.bus.get(self.ports.m_comp_in);

        // The manifold's eigenvalue near a unity pressure ratio is about -8900 1/s (0.11 ms), so explicit Euler at the 1 ms component step is unstable and rings at the step frequency. Sub-step
        const N_SUB: u32 = 10;
        let h = self.dt / N_SUB as f64;

        for _ in 0..N_SUB {
            self.p += R_AIR * self.t / self.volume * (m_in - m_out) * h;
            self.p = self.p.max(5_000.0);
        }

        // --- temperature: incoming charge mixes with what's there. Enthalpy balance would be better; this is adequate until the intercooler exists and T actually varies.
        self.t = t_up;

        ctx.bus.set(self.ports.p, self.p);
        ctx.bus.set(self.ports.t, self.t);
        ctx.bus.set(self.ports.m_dot_in, m_in);
    }
}