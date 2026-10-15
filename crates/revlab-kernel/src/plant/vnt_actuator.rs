use revlab_core::SimDuration;
use crate::{Component, Ctx, Port, Trigger};

#[derive(Copy, Clone)]
pub struct VntActuatorPorts {
    pub cmd: Port,  // demanded position from the ECU: 0 = closed, 1 = open
    pub pos: Port,  // actual position: to the turbo, and to the ECU's position sensor
}

/// VNT positioner: the actuator's own position loop, seen from outside as first order lag behind a
/// slew limit. Moving the vanes take time, which is what makes boost control a control problem and
/// not a lookup
pub struct VntActuator {
    pos: f64,
    pub tau: f64,       // s, position loop time constant
    pub rate: f64,      // 1/s, slew limit: full travel take 1/rate seconds
    ports: VntActuatorPorts,
    dt: f64,
}

impl VntActuator {
    pub const STEP: SimDuration = SimDuration::from_millis(1);

    pub fn ea288(ports: VntActuatorPorts, pos_init: f64) -> Self {
        VntActuator { pos: pos_init.clamp(0.0, 1.0), tau: 0.04, rate: 5.0, ports, dt: Self::STEP.as_secs_f64() }
    }
}

impl Component for VntActuator {
    fn triggers(&self) -> Vec<Trigger> {
        // After the ECU's 1 ms task (200 µs), before the turbo (250 µs)
        vec![Trigger::Periodic { period: Self::STEP, offset: SimDuration::from_micros(220) }]
    }
    
    fn step(&mut self, _t: u16, ctx: &mut Ctx<'_>) {
        let cmd = ctx.bus.get(self.ports.cmd).clamp(0.0, 1.0);
        let v = ((cmd - self.pos) / self.tau).clamp(-self.rate, self.rate);
        self.pos = (self.pos + v * self.dt).clamp(0.0, 1.0);
        ctx.bus.set(self.ports.pos, self.pos);
    }
}