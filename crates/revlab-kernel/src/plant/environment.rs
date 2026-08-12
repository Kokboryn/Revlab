use revlab_core::SimDuration;
use crate::{Component, Ctx, Port, Trigger};
use super::gas::{R_AIR, T0_C};

/// Ambient conditions. First-class so they can be varied at runtime - weather, altitude, headwind - rather than baked into the compressor
pub struct Environment {
    pub p_amb: f64,         // Pa, static
    pub t_amb: f64,         // K
    pub headwind: f64,       // m/s, positive = into the intake
    p_out: Port,
    t_out: Port,
}

impl Environment {
    /// ISA sea level, 20 °C, still air
    pub fn standard(p_out: Port, t_out: Port) -> Self {
        Environment {
            p_amb: 101_325.0,
            t_amb: T0_C + 20.0,
            headwind: 0.0,
            p_out, t_out,
        }
    }

    pub fn at_altitude(mut self, metres: f64) -> Self {
        // barometric formula, constant lapse rate
        self.p_amb = 101_325.0 * (1.0 - 2.25577e-5 * metres).powf(5.25588);
        self.t_amb = T0_C + 15.0 - 0.0065 * metres;
        self
    }

    /// Stagnation pressure at the intake. Incompressible form is fine below ~100 m/s: even 30 m/s adds only ~550 Pa, about 0.5%.
    fn ram_pressure(&self) -> f64 {
        let rho = self.p_amb / (R_AIR * self.t_amb);
        self.p_amb + 0.5 * rho * self.headwind * self.headwind
    }
}

impl Component for Environment {
    fn triggers(&self) -> Vec<Trigger> {
        vec![Trigger::Periodic {
            period: SimDuration::from_millis(100),
            offset: SimDuration::from_micros(100),
        }]
    }

    fn step(&mut self, _t: u16, ctx: &mut Ctx<'_>) {
        ctx.bus.set(self.p_out, self.ram_pressure());
        ctx.bus.set(self.t_out, self.t_amb);
    }
}