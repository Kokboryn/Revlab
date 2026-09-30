pub mod clutch_ctl;
pub mod diag;

use revlab_core::{SimDuration, SimTime};
use crate::{Component, Ctx, Port, Trigger};

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Rate { Ms10, Ms100 }

impl Rate {
    fn period(&self) -> SimDuration {
        match self {
            Rate::Ms10 => SimDuration::from_millis(10),
            Rate::Ms100 => SimDuration::from_millis(100),
        }
    }
    /// Staggered clear of the ECU's offsets so the two never share a timestamp
    fn offset(self) -> SimDuration {
        match self {
            Rate::Ms10 => SimDuration::from_micros(3500),
            Rate::Ms100 => SimDuration::from_micros(4500),
        }
    }
    fn from_trig(t: u16) -> Rate {
        match t { 0 => Rate::Ms10, _ => Rate::Ms100 }
    }
}

/// What the driver is asking for. The TCU decides whether to grant it
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Lever { Park, Reverse, Neutral, Drive }

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum ClutchState { Open, Engaging, Closed }

/// The TCU's RAM image. Same rule as the ECU: tasks see this and nothing else. It has no access to plant
/// state, and no access to the ECU's RAM either -- what passes between them is what a CAN frame would
/// carry
pub struct TcuState {
    pub now: SimTime,
    pub lever: Lever,
    pub n_eng: f64,                 // rpm, from the engine speed line
    pub n_in: f64,                  // rpm, input shaft sensor
    pub v_veh: f64,                 // km/h, from wheel speed
    pub pedal: f64,
    pub brake: f64,
    pub gear: usize,                // engaged
    pub clutch_state: ClutchState,
    pub clutch_cmd: f64,            // 0..1, what the actuator is asked for
    pub t_disc_est: f64,            // K, the TCU's own thermal model
    pub overheat: bool,
}

pub trait Task: Send {
    fn name(&self) -> &'static str;
    fn run(&mut self, s: &mut TcuState);
}

/// Every port TCU touches. Inputs are sensor lines and the selector; outputs are actuator commands
/// and status. Nothing here reaches plant state.
#[derive(Copy, Clone)]
pub struct TcuPorts {
    // inputs
    pub lever: Port,
    pub n_eng: Port,    // rpm, engine speed (CAN in a real car)
    pub n_in1: Port,    // rpm, shaft 1 speed sensor, odd gears
    pub n_in2: Port,    // rpm, shaft 2 speed sensor, even gears
    pub v_veh: Port,    // rpm, wheel speed sensor
    pub pedal: Port,
    pub brake: Port,
    // outputs: actuators
    pub sel1: Port,     // gear selected on shaft 1
    pub sel2: Port,     // gear selected on shaft 2
    pub cmd1: Port,     // K1 clamp
    pub cmd2: Port,     // K2 clamp
    // outputs: status, for telemetry only
    pub clutch_cmd: Port,   // command to whichever pack carries the engaged gear
    pub gear: Port,
    pub clutch_state: Port,
    pub t_disc_est: Port,
    pub overheat: Port,
}

pub struct Tcu {
    state: TcuState,
    tasks: Vec<(Rate, Box<dyn Task>)>,
    p: TcuPorts,
}

impl Tcu {
    pub fn new(p: TcuPorts, gear_init: usize) -> Self {
        Tcu {
            state: TcuState {
                now: SimTime::ZERO,
                lever: Lever::Neutral,
                n_eng: 0.0, n_in: 0.0, v_veh: 0.0, pedal: 0.0, brake: 0.0,
                gear: gear_init,
                clutch_state: ClutchState::Open,
                clutch_cmd: 0.0,
                t_disc_est: 293.15,
                overheat: false,
            },
            tasks: Vec::new(),
            p,
        }
    }

    /// Registration order is execution order within a rate
    pub fn task(mut self, rate: Rate, t: Box<dyn Task>) -> Self {
        self.tasks.push((rate, t));
        self
    }
}

impl Lever {
    pub fn from_port(v: f64) -> Lever {
        match v.round() as i32 {
            0 => Lever::Park, 1 => Lever::Reverse, 3 => Lever::Drive, _ => Lever::Neutral,
        }
    }
    pub fn to_port(self) -> f64 {
        match self { Lever::Park => 0.0, Lever::Reverse => 1.0, Lever::Neutral => 2.0, Lever::Drive => 3.0 }
    }
}

impl Component for Tcu {
    fn triggers(&self) -> Vec<Trigger> {
        [Rate::Ms10, Rate::Ms100].iter()
            .map(|r| Trigger::Periodic { period: r.period(), offset: r.offset() })
            .collect()
    }

    fn step(&mut self, trig: u16, ctx: &mut Ctx<'_>) {
        let rate = Rate::from_trig(trig);

        // --- input latch
        self.state.now      = ctx.now;
        self.state.lever    = Lever::from_port(ctx.bus.get(self.p.lever));
        self.state.n_eng    = ctx.bus.get(self.p.n_eng);
        // Watch the shaft that carries the engaged gear
        self.state.n_in     = if self.state.gear % 2 == 1 {
            ctx.bus.get(self.p.n_in1)
        } else {
            ctx.bus.get(self.p.n_in2)
        };
        self.state.v_veh    = ctx.bus.get(self.p.v_veh);
        self.state.pedal    = ctx.bus.get(self.p.pedal);
        self.state.brake    = ctx.bus.get(self.p.brake);

        // --- application
        for (r, t) in self.tasks.iter_mut() {
            if *r == rate { t.run(&mut self.state); }
        }

        // A real TCU knows its gearbox: odd gears on K1, even on K2. The pack that carries the engaged
        // gear gets the command, the other stays open
        let g = self.state.gear;
        let odd = g % 2 == 1;

        // --- output drivers
        ctx.bus.set(self.p.sel1, if odd { g as f64 } else { 0.0 });
        ctx.bus.set(self.p.sel2, if !odd && g > 0 { g as f64 } else { 0.0 });
        ctx.bus.set(self.p.cmd1, if odd { self.state.clutch_cmd } else { 0.0 });
        ctx.bus.set(self.p.cmd2, if !odd && g > 0 { self.state.clutch_cmd } else { 0.0 });
        ctx.bus.set(self.p.clutch_cmd, self.state.clutch_cmd);
        ctx.bus.set(self.p.gear, self.state.gear as f64);
        ctx.bus.set(self.p.clutch_state, match self.state.clutch_state {
            ClutchState::Open => 0.0, ClutchState::Engaging => 1.0, ClutchState::Closed => 2.0,
        });
        ctx.bus.set(self.p.t_disc_est, self.state.t_disc_est);
        ctx.bus.set(self.p.overheat, if self.state.overheat { 1.0 } else { 0.0 });
    }
}