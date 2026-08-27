use revlab_core::{SimDuration, SimTime};
use crate::{Component, Ctx, Port, Trigger};

/// External load on the crank - accessories, dyno, eventually the driveline. A step schedule for now; a driveline component later
pub struct LoadProfile {
    steps: Vec<(SimTime, f64)>,  // sorted by time
    ramp: Option<SimDuration>,   // None = step, Some(d) = linear over d
    current: f64,
    from: f64,                   // value when the current ramp began
    at: SimTime,                 // when it began
    next: usize,
    t_load_out: Port,
}

impl LoadProfile {
    pub fn new(mut steps: Vec<(SimTime, f64)>, t_load_out: Port) -> Self {
        steps.sort_by_key(|(t, _)| *t);
        LoadProfile { steps, ramp: None, from: 0.0, at: SimTime::ZERO, current: 0.0, next: 0, t_load_out }
    }
    
    /// Linear ramp to each new value over `d`. Steps are right for gear selection and fault injection,
    /// where the real thing is discontinuous. Pedal, clutch and load are not: a step demands infinite
    /// actuator bandwidth, and a clutch step put 115 kW through a dry pack in 100 ms.
    pub fn ramped(mut self, d: SimDuration) -> Self {
        self.ramp = Some(d);
        self
    }
}

impl Component for LoadProfile {
    fn triggers(&self) -> Vec<Trigger> {
        vec![Trigger::Periodic { period: SimDuration::from_millis(1), offset: SimDuration::from_micros(150) }]
    }

    fn step(&mut self, _t: u16, ctx: &mut Ctx<'_>) {
        while self.next < self.steps.len() && ctx.now >= self.steps[self.next].0 {
            self.from = self.current;
            self.at = self.steps[self.next].0;
            self.next += 1;
        }
        
        let target = if self.next == 0 { self.from } else { self.steps[self.next - 1].1 };
        self.current = match self.ramp {
            None => target,
            Some(d) => {
                let elapsed = (ctx.now - self.at).as_secs_f64();
                let total = d.as_secs_f64();
                if elapsed > total { target }
                else { self.from + (target - self.from) * (elapsed / total) }
            }
        };
        
        ctx.bus.set(self.t_load_out, self.current);
    }
}