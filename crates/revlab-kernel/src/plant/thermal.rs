use revlab_core::SimDuration;
use crate::{Component, Ctx, Port, Trigger};

/// Engine thermal state: coolant/block as one lumped mass, oil as a second that lags it.
///
/// Of the fuel energy, roughly 40% becomes work, 30% leaves in the exhaust, and 30% goes into the coolant.
/// Only that last share appears here - the exhaust share is already accounted for in the exhaust manifold's
/// port temperature
pub struct ThermalSystem {
    t_cool: f64,                // K
    t_oil: f64,                 // K
    pub c_block: f64,           // J/K, coolant + block metal
    pub c_oil: f64,             // J/K
    pub frac_to_coolant: f64,
    pub ua_rad: f64,            // W/K, radiator at full flow
    pub ua_oil: f64,            // W/K, block-to-oil coupling
    pub t_stat_open: f64,       // K, thermostat starts opening
    pub t_stat_full: f64,       // K, fully open
    m_fuel_in: Port,
    t_amb_in: Port,
    t_cool_out: Port,
    t_oil_out: Port,
    visc_mult_out: Port,        // friction multiplier from oil viscosity
    dt: f64,
}

impl ThermalSystem {
    pub const STEP: SimDuration = SimDuration::from_millis(100);

    pub fn ea288(m_fuel_in: Port, t_amb_in: Port, t_cool_out: Port,
                 t_oil_out: Port, visc_mult_out: Port, t_init: f64) -> Self {
        ThermalSystem {
            t_cool: t_init,
            t_oil: t_init,  // ~6 L coolant (25 kJ/K) plus ~120 kg of block (60 kJ/K)
            c_block: 85_000.0,
            c_oil: 8_000.0,     // ~4.5 L oil plus sump metal
            frac_to_coolant: 0.30,
            ua_rad: 900.0,
            ua_oil: 350.0,
            t_stat_open: 273.15 + 85.0,
            t_stat_full: 273.15 + 95.0,
            m_fuel_in, t_amb_in, t_cool_out, t_oil_out, visc_mult_out,
            dt: Self::STEP.as_secs_f64(),
        }
    }

    /// 0.0 closed, 1.0 fully open. Below the opening temperature the coolant bypasses the radiator
    /// entirely, which is what makes warmup as fast as it is
    fn thermostat(&self) -> f64 {
        ((self.t_cool - self.t_stat_open) / (self.t_stat_full - self.t_stat_open)).clamp(0.0, 1.0)
    }

    /// Friction multiplier from oil viscosity. Cold oil is dramatically thicker: roughly 3x the
    /// friction at 0 C against fully warm. Exponential fit, 1.0 at 90 C.
    fn visc_mult(&self) -> f64 {
        let t_c = self.t_oil - 273.15;
        (1.0 + 2.2 * (-(t_c - 0.0) / 38.0).exp()).clamp(1.0, 4.0)
    }
}

impl Component for ThermalSystem {
    fn triggers(&self) -> Vec<Trigger> {
        // 100 ms: thermal time constants are minutes, so this is already three orders of magnitude
        // faster than the dynamics.
        vec![Trigger::Periodic {period: Self::STEP, offset: SimDuration::from_micros(700) }]
    }

    fn step(&mut self, _t: u16, ctx: &mut Ctx<'_>) {
        const LHV: f64 = 42.7e6;
        let m_fuel = ctx.bus.get(self.m_fuel_in);
        let t_amb = ctx.bus.get(self.t_amb_in);

        let q_in = m_fuel * LHV * self.frac_to_coolant;
        let q_rad = self.ua_rad * self.thermostat() * (self.t_cool - t_amb);
        // Small convective loss even with the thermostat shut
        let q_shell = 25.0 * (self.t_cool - t_amb);
        let q_oil = self.ua_oil * (self.t_cool - self.t_oil);

        self.t_cool += (q_in - q_rad - q_shell - q_oil) / self.c_block * self.dt;
        self.t_oil += (q_oil - 15.0 * (self.t_oil - t_amb)) / self.c_oil * self.dt;

        self.t_cool = self.t_cool.max(t_amb);
        self.t_oil = self.t_oil.max(t_amb);

        ctx.bus.set(self.t_cool_out, self.t_cool);
        ctx.bus.set(self.t_oil_out, self.t_oil);
        ctx.bus.set(self.visc_mult_out, self.visc_mult());
    }
}