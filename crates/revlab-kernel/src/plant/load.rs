use revlab_core::{SimDuration, SimTime};
use crate::{Component, Ctx, Port, Trigger};

/// External load on the crank - accessories, dyno, eventually the driveline. A step schedule for now; a driveline component later
pub struct LoadProfile {
    steps: Vec<(SimTime, f64)>,  // sorted by time
    current: f64,
    next: usize,
    t_load_out: Port,
}

impl LoadProfile {
    pub fn new(mut steps: Vec<(SimTime, f64)>, t_load_out: Port) -> Self {
        steps.sort_by_key(|(t, _)| *t);
        LoadProfile { steps, current: 0.0, next: 0, t_load_out }
    }
}

impl Component for LoadProfile {
    fn triggers(&self) -> Vec<Trigger> {
        vec![Trigger::Periodic { period: SimDuration::from_millis(1), offset: SimDuration::from_micros(150) }]
    }

    fn step(&mut self, _t: u16, ctx: &mut Ctx<'_>) {
        while self.next < self.steps.len() && ctx.now >= self.steps[self.next].0 {
            self.current = self.steps[self.next].1;
            self.next += 1;
        }
        ctx.bus.set(self.t_load_out, self.current);
    }
}