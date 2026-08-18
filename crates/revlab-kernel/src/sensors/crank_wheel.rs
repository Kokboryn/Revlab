use std::f64::consts::PI;
use revlab_core::{SimDuration, SimTime};
use crate::{Component, Ctx, Port, Trigger};
use super::Fault;

const TEETH: u32 = 60;
const PRESENT: u32 = 58;        // 60 - 2 wheel
const TOOTH_RAD: f64 = 2.0 * PI / TEETH as f64;

/// Models the toothed wheel, not an "rpm sensor" Speed is *inferred* from tooth period, so staleness and quantization emerge rather than being bolted on
pub struct CrankWheel {
    tooth: u32,
    last_edge: SimTime,
    step_rad: f64,              // angular step that produced the current edge
    fault: Fault,
    fault_since: Option<SimTime>,
    armed: Option<(SimTime, Fault)>,
    timer_tick_ns: u64,         // capture timer resolution
    noise_rpm: f64,
    omega_in: Port,
    n_meas_out: Port,
    valid_out: Port,
}

impl CrankWheel {
    pub fn new(omega_in: Port, n_meas_out: Port, valid_out: Port,) -> Self {
        CrankWheel {
            tooth: 0,
            last_edge: SimTime::ZERO,
            step_rad: TOOTH_RAD,
            fault: Fault::None,
            fault_since: None,
            armed: None,
            timer_tick_ns: 50,  // 20 MHz capture timer
            noise_rpm: 1.5,
            omega_in, n_meas_out, valid_out,
        }
    }

    pub fn arm_fault(mut self, at: SimTime, f: Fault) -> Self {
        self.armed = Some((at, f));
        self
    }

    pub fn inject(&mut self, f: Fault, now: SimTime) {
        self.fault = f;
        self.fault_since = Some(now);
    }

    /// Angular gap from the current tooth to the next physical tooth.
    fn next_step(&self) -> (u32, f64) {
        let next = (self.tooth + 1) % TEETH;
        if next >= PRESENT {
            (0, 3.0 * TOOTH_RAD)    // jump the two tooth gap
        } else {
            (next, TOOTH_RAD)
        }
    }
}

impl Component for CrankWheel {
    fn triggers(&self) -> Vec<Trigger> { vec![Trigger::SelfPaced] }

    fn step(&mut self, _trig: u16, ctx: &mut Ctx<'_>) {
        if let Some((t, f)) = self.armed {
            if ctx.now >= t {
                self.fault = f;
                self.fault_since = Some(ctx.now);
                self.armed = None;
            }
        }

        let omega = ctx.bus.get(self.omega_in);
        if omega <= 1.0 {
            ctx.bus.set(self.valid_out, 0.0);
            self.last_edge = ctx.now;
            ctx.schedule_in(SimDuration::from_millis(50), 0);
            return;
        }

        // --- measure: period since previous edge, quantized by the timer
        let raw_ns = (ctx.now - self.last_edge).as_nanos();
        let q_ns = (raw_ns / self.timer_tick_ns) * self.timer_tick_ns;
        self.last_edge = ctx.now;

        if q_ns > 0 {
            let dt = q_ns as f64 * 1e-9;
            let mut n_rpm = (self.step_rad / dt) * 60.0 / (2.0 * PI);
            n_rpm += ctx.rng.normal() * self.noise_rpm;

            let t_active = self.fault_since
                .map(|t0| (ctx.now - t0).as_secs_f64()).unwrap_or(0.0);
            match self.fault.apply(n_rpm, t_active) {
                Some(v) => {
                    ctx.bus.set(self.n_meas_out, v);
                    ctx.bus.set(self.valid_out, 1.0);
                }
                None => ctx.bus.set(self.valid_out, 0.0),      // open circuit
            }
        }

        let (next_tooth, step) = self.next_step();
        self.tooth = next_tooth;
        self.step_rad = step;
        let ns = (step / omega * 1e9) as u64;
        ctx.schedule_in(SimDuration::from_nanos(ns.max(1)), 0);
    }
}