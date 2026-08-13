use revlab_core::SimDuration;
use crate::{Component, Ctx, Port, Trigger};

pub const R_EXH: f64 = 287.0;       // close enough to air at these AFRs
pub const CP_EXH: f64 = 1150.0;     // J/(kg·K), hot combustion products
pub const GAMMA_EXH: f64 = 1.33;

/// Exhaust manifold: a filling and emptying volume like the intake, plus a temperature state. Gas leaves the cylinder carrying the fuel energy that did NOT become indicated work.
pub struct ExhaustManifold {
    p: f64,
    t: f64,
    pub volume: f64,        // m³
    pub eta_ind_nom: f64,   // fraction of fuel energy that became work
    pub t_wall: f64,        // K, manifold wall - sink for heat loss
    pub h_loss: f64,        // W/K, lumped wall heat transfer
    m_air_in: Port,
    m_fuel_in: Port,
    t_im: Port,
    m_turb_out_in: Port,    // turbine flow, from the turbo component
    p_out: Port,
    t_out: Port,
    dt: f64,
}

impl ExhaustManifold {
    pub const STEP: SimDuration = SimDuration::from_millis(1);

    #[allow(clippy::too_many_arguments)]
    pub fn new(volume: f64, m_air_in: Port, m_fuel_in: Port, t_im: Port,
                m_turb_out_in: Port, p_out: Port, t_out: Port,
                p_init: f64, t_init: f64) -> Self {
        ExhaustManifold {
            p: p_init, t: t_init, volume,
            eta_ind_nom: 0.40,
            t_wall: 450.0,
            h_loss: 3.0,
            m_air_in, m_fuel_in, t_im, m_turb_out_in, p_out, t_out,
            dt: Self::STEP.as_secs_f64(),
        }
    }
}

impl Component for ExhaustManifold {
    fn triggers(&self) -> Vec<Trigger> {
        vec![Trigger::Periodic { period: Self::STEP, offset: SimDuration::from_micros(350) }]
    }

    fn step(&mut self, _t: u16, ctx: &mut Ctx<'_>) {
        const LHV: f64 = 42.7e6;
        let m_air   = ctx.bus.get(self.m_air_in);
        let m_fuel  = ctx.bus.get(self.m_fuel_in);
        let t_im    = ctx.bus.get(self.t_im);
        let m_out   = ctx.bus.get(self.m_turb_out_in);
        let m_in    = m_air + m_fuel;

        // --- port temperature: energy not converted to work heats the gas
        let t_port = if m_in > 1e-6 {
            t_im + m_fuel * LHV * (1.0 - self.eta_ind_nom) / (m_in * CP_EXH)
        } else { t_im };

        const N_SUB: u32 = 10;
        let h = self.dt / N_SUB as f64;
        for _ in 0..N_SUB {
            let m_gas = self.p * self.volume / (R_EXH * self.t);
            let d_mix = m_in * (t_port - self.t) / m_gas;
            let d_wall = -self.h_loss * (self.t - self.t_wall) / (m_gas * CP_EXH);
            self.t = (self.t + (d_mix + d_wall) * h).clamp(250.0, 1300.0);
            self.p = (self.p + R_EXH * self.t / self.volume * (m_in - m_out) * h).max(5_000.0);
        }

        ctx.bus.set(self.p_out, self.p);
        ctx.bus.set(self.t_out, self.t);
    }
}