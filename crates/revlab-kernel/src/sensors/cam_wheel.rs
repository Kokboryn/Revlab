use std::f64::consts::PI;
use revlab_core::{SimDuration, SimTime};
use crate::{Component, Ctx, Port, Trigger};
use super::Fault;

const LOBES: u32 = 4;                           // one per cylinder
const LOBE_RAD: f64 = 2.0 * PI / LOBES as f64;  // cam radians per event

/// Camshaft position sensor, Turns at half crank speed, with far fewer teeth than the crank wheel - so it is coarse and laggy (~37 ms at idle), but independent. That independence is what makes it useful for plausibility checking
pub struct CamWheel {
    last_edge: SimTime,
    fault: Fault,
    fault_since: Option<SimTime>,
    armed: Option<(SimTime, Fault)>,
    noise_rpm: f64,
    omega_in: Port,     // true Crank omega
    n_cam_out: Port,    // crank equivalent rpm
    valid_out: Port,
}

impl CamWheel {
    pub fn new(omega_in: Port, n_cam_out:Port, valid_out: Port,) -> Self {
        CamWheel {
            last_edge: SimTime::ZERO,
            fault: Fault::None, fault_since: None,
            armed: None,
            noise_rpm: 3.0,
            omega_in, n_cam_out, valid_out,
        }
    }

    pub fn arm_fault(mut self, at: SimTime, f: Fault) -> Self {
        self.armed = Some((at, f));
        self
    }
}

impl Component for CamWheel {
    fn triggers(&self) -> Vec<Trigger> { vec![Trigger::SelfPaced] }

    fn step(&mut self, _t: u16, ctx: &mut Ctx<'_>) {
        if let Some((t, f)) = self.armed {
            if ctx.now >= t {
                self.fault = f;
                self.fault_since = Some(ctx.now);
                self.armed = None;
            }
        }

        // --- reschedule / stall check FIRST, so a stopped engine never reaches the measurement path
        // and invents a period from the 50 ms poll interval

        let omega = ctx.bus.get(self.omega_in);
        if omega <= 1.0 {
            ctx.bus.set(self.valid_out, 0.0);
            self.last_edge = ctx.now;       // no stale gap after a restart
            ctx.schedule_in(SimDuration::from_millis(50), 0);
            return;
        }

        let dt_ns = (ctx.now - self.last_edge).as_nanos();
        self.last_edge = ctx.now;
        if dt_ns > 0 {
            let dt = dt_ns as f64 * 1e-9;
            // cam omega -> crank omega is x2 on a four stroke
            let omega_crank = (LOBE_RAD / dt) * 2.0;
            let mut n = omega_crank * 60.0 / (2.0 * PI);
            n += ctx.rng.normal() * self.noise_rpm;

            let ta = self.fault_since
                .map(|t0| (ctx.now - t0).as_secs_f64()).unwrap_or(0.0);
            match self.fault.apply(n, ta) {
                Some(v) => {
                    ctx.bus.set(self.n_cam_out, v);
                    ctx.bus.set(self.valid_out, 0.0);
                }
                // Open circuit: no signal at all. Indistinguishable from a stopped engine at the ECU,
                // which is correct
                None => ctx.bus.set(self.valid_out, 0.0),
            }
        }

        let ns = (LOBE_RAD / (omega / 2.0) * 1e9) as u64;
        ctx.schedule_in(SimDuration::from_nanos(ns.max(1)), 0);
    }
}