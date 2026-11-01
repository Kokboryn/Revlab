use std::f64::consts::PI;
use revlab_core::{SimDuration, SimTime};
use crate::{Component, Ctx, Port, Trigger};
use super::Fault;

const LOBES: u32 = 4;                           // one per cylinder
const LOBE_RAD: f64 = 2.0 * PI / LOBES as f64;  // cam radians per event
const EDGE_RAD: f64 = 2.0 * LOBE_RAD;           // crank radians per lobe: the cam turns at half speed

/// Camshaft position sensor, Turns at half crank speed, with far fewer teeth than the crank wheel - so it is coarse and laggy (~37 ms at idle), but independent. That independence is what makes it useful for plausibility checking
pub struct CamWheel {
    last_edge: SimTime, // interpolated time of the last lobe edge
    last_wake: SimTime,
    angle: f64,         // crank radians turned since the last edge
    last_omega: f64,
    running: bool,
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
            last_edge: SimTime::ZERO, last_wake: SimTime::ZERO, angle: 0.0, last_omega: 0.0,
            running: false, fault: Fault::None, fault_since: None, armed: None, noise_rpm: 3.0,
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

        // An edge is the lobe passing the sensor: the crank has really turned 100 degrees since the
        // last one. Scheduling the next edge from the current speed instead would assume the speed
        // holds, and the reading would be the speed at the previous edge rather than the average across
        // the interval
        let omega = ctx.bus.get(self.omega_in);
        if omega <= 1.0 {
            ctx.bus.set(self.valid_out, 0.0);
            self.running = false;
            ctx.schedule_in(SimDuration::from_millis(1), 0);
            return;
        }

        let dt = (ctx.now - self.last_wake).as_secs_f64();
        let step = 0.5 * (omega + self.last_omega) * dt;        // trapezoid: crank radians since the last poll
        if self.angle + step >= EDGE_RAD {
            // The lobe passed inside this poll: place the edge where the angle crossed
            let frac = (EDGE_RAD - self.angle) / step;
            let t_edge = self.last_wake + SimDuration::from_nanos((frac * dt * 1e9) as u64);
            let period = (t_edge - self.last_edge).as_secs_f64();
            self.last_edge = t_edge;
            self.angle = self.angle + step - EDGE_RAD;
            if period > 0.0 {
                let mut n = EDGE_RAD / period * 60.0 / (2.0 * PI);
                n += ctx.rng.normal() * self.noise_rpm;

                let ta = self.fault_since
                    .map(|t0| (ctx.now - t0).as_secs_f64()).unwrap_or(0.0);
                match self.fault.apply(n, ta) {
                    Some(v) => {
                        ctx.bus.set(self.n_cam_out, v);
                        ctx.bus.set(self.valid_out, 1.0);
                    }
                    // Open circuit: no signal at all, Indistinguishable from a stopped engine at the
                    // ECU, which is correct
                    None => ctx.bus.set(self.valid_out, 0.0),
                }
            }
        } else {
            self.angle += step;
        }
        self.last_wake = ctx.now;
        self.last_omega = omega;
        ctx.schedule_in(SimDuration::from_millis(1), 0);
    }
}