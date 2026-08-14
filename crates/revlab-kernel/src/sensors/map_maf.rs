use revlab_core::SimDuration;
use crate::{Component, Ctx, Port, Trigger};
use super::Fault;

/// Analogue sensor with first order lag, noise, quantization and faults. Covers, MAP, MAF, IAT - anything read through an ADC
pub struct AnalogSensor {
    filtered: f64,
    pub tau: f64,       // s, sensor time constant
    pub noise: f64,     // stddev in engineering units
    pub lsb: f64,       // quantization step (full scale / 4096)
    pub scale: f64,     // truth -> reading
    fault: Fault,
    fault_since: Option<revlab_core::SimTime>,
    armed: Option<(revlab_core::SimTime, Fault)>,
    truth_in: Port,
    out: Port,
    dt: f64,
}

impl AnalogSensor {
    pub const STEP: SimDuration = SimDuration::from_millis(1);

    pub fn new(truth_in: Port, out: Port, tau: f64, noise: f64, full_scale: f64, init: f64) -> Self {
        AnalogSensor {
            filtered: init,
            tau, noise,
            lsb: full_scale / 4096.0,       // 12 bit ADC
            scale: 1.0,
            fault: Fault::None, fault_since: None, armed: None,
            truth_in, out,
            dt: Self::STEP.as_secs_f64(),
        }
    }

    pub fn arm_fault(mut self, at: revlab_core::SimTime, f: Fault) -> Self {
        self.armed = Some((at, f));
        self
    }
}

impl Component for AnalogSensor {
    fn triggers(&self) -> Vec<Trigger> {
        vec![Trigger::Periodic { period: Self::STEP, offset: SimDuration::from_micros(400) }]
    }

    fn step(&mut self, _t: u16, ctx: &mut Ctx<'_>) {
        if let Some((t, f)) = self.armed {
            if ctx.now >= t {
                self.fault = f;
                self.fault_since = Some(ctx.now);
                self.armed = None;
            }
        }

        let truth = ctx.bus.get(self.truth_in) * self.scale;

        // first order lag: the physical sensing element's response
        let a =self.dt / (self.tau + self.dt);
        self.filtered += a * (truth - self.filtered);

        let mut v = self.filtered + ctx.rng.normal() * self.noise;
        v = (v / self.lsb).round() * self.lsb;      // ADC quantization

        let ta = self.fault_since
            .map(|t0| (ctx.now - t0).as_secs_f64()).unwrap_or(0.0);
        if let Some(out) = self.fault.apply(v, ta) {
            ctx.bus.set(self.out, out);
        }
    }
}