use revlab_core::SimDuration;
use crate::{Component, Ctx, Port, Trigger};
use super::gas::{psi, R_AIR};

/// Intake manifold as a filling and emptying volume.
///
/// dp/dt = (R·T/V)·(ṁ_in − ṁ_out)
///
/// Time constant is V/(ṁ·R·T/p) - a few tens of ms at idle, much faster at high flow. This is what makes manifold pressure lag throttle or boost changes
pub struct IntakeManifold {
    p: f64,             // Pa
    t: f64,             // K
    pub volume: f64,    // m³, plenum + runners
    pub cd_a: f64,      // m², effective throttle/inlet area
    p_up: Port,         // upstream (compressor outlet; ambient for now)
    t_up: Port,
    m_dot_eng: Port,    // written by Engine — last step's consumption
    p_out: Port,
    t_out: Port,
    m_dot_in_out: Port, // what a MAF would see
    dt: f64,
}

impl IntakeManifold {
    pub const STEP: SimDuration = SimDuration::from_millis(1);

    pub fn new(volume: f64, p_up: Port, t_up: Port, m_dot_eng: Port, p_out: Port, t_out: Port, m_dot_in_out: Port, p_init: f64, t_init: f64) -> Self {
        IntakeManifold {
            p: p_init, t: t_init, volume,
            cd_a: 1.6e-3,           // ~45 mm bore, Cd = 0.9 - diesel, no throttle plate
            p_up, t_up, m_dot_eng, p_out, t_out, m_dot_in_out, dt: Self::STEP.as_secs_f64(),
        }
    }
}

impl Component for IntakeManifold {
    fn triggers(&self) -> Vec<Trigger> {
        vec![Trigger::Periodic { period: Self::STEP, offset: SimDuration::from_micros(300) }]
    }

    fn step(&mut self, _t: u16, ctx: &mut Ctx<'_>) {
        let p_up = ctx.bus.get(self.p_up);
        let t_up = ctx.bus.get(self.t_up);
        let m_out = ctx.bus.get(self.m_dot_eng);

        // The manifold's eigenvalue near unity pressure ratio is about -8900 1/s (0.11 ms), so explicit Euler at the 1 ms component step is unstable and rings at the step frequency. Sub-step
        const N_SUB: u32 = 10;
        let h = self.dt / N_SUB as f64;
        let mut m_in = 0.0;

        for _ in 0..N_SUB {
            m_in = if p_up > self.p {
                self.cd_a * p_up / (R_AIR * t_up).sqrt() * psi(self.p / p_up)
            } else {
                0.0
            };
            self.p += R_AIR * self.t / self.volume * (m_in - m_out) * h;
            self.p = self.p.max(5_000.0);
        }

        // --- temperature: incoming charge mixes with what's there. Enthalpy balance would be better; this is adequate until the intercooler exists and T actually varies.
        self.t = t_up;

        ctx.bus.set(self.p_out, self.p);
        ctx.bus.set(self.t_out, self.t);
        ctx.bus.set(self.m_dot_in_out, m_in);
    }
}