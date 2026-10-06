pub mod clutch_ctl;
pub mod shift_ctl;
pub mod diag;

use revlab_core::{SimDuration, SimTime};
use crate::{Component, Ctx, Port, Trigger};
use std::f64::consts::PI;

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
pub enum Lever { Park, Reverse, Neutral, Drive, Manual }

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
    pub n_shaft: [f64; 2],          // rpm, both input shaft sensors
    pub eng: [usize; 2],            // gear engaged per shaft, from the fork position
    pub sel: [usize; 2],            // fork requests
    pub cmd: [f64; 2],              // K1, K2 clamp
    pub tip_up: u32,                // running count of tiptronic + presses
    pub shift_phase: u8,            // 0 idle, 1 prepare, 2 release, 3 torque, 4 inertia
    pub v_veh: f64,                 // km/h, from wheel speed
    pub pedal: f64,
    pub brake: f64,
    pub gear: usize,                // engaged
    pub clutch_state: ClutchState,
    pub clutch_cmd: f64,            // 0..1, what the actuator is asked for
    pub t_disc_est: [f64; 2],            // K, the TCU's own thermal model, per pack
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
    pub eng1: Port,     // fork position sensor, shaft 1: gear engaged, 0 while moving
    pub eng2: Port,
    pub n_wheel: Port,    // rpm, wheel speed sensor (ABS over CAN in a real car)
    pub pedal: Port,
    pub brake: Port,
    pub tip_up: Port,
    // outputs: actuators
    pub sel1: Port,     // gear selected on shaft 1
    pub sel2: Port,     // gear selected on shaft 2
    pub cmd1: Port,     // K1 clamp
    pub cmd2: Port,     // K2 clamp
    // outputs: status, for telemetry only
    pub clutch_cmd: Port,   // command to whichever pack carries the engaged gear
    pub gear: Port,
    pub clutch_state: Port,
    pub t_disc_est1: Port,
    pub t_disc_est2: Port,
    pub overheat: Port,
    pub shift_phase: Port,
}

pub struct Tcu {
    state: TcuState,
    tasks: Vec<(Rate, Box<dyn Task>)>,
    p: TcuPorts,
    r_wheel: f64,                   // m, coded rolling radius -- the TCU's belief, not the plant's tire

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
                t_disc_est: [293.15; 2],
                overheat: false,
                n_shaft: [0.0; 2],
                eng: [0; 2],
                sel: [0; 2],
                cmd: [0.0; 2],
                tip_up: 0,
                shift_phase: 0,
            },
            tasks: Vec::new(),
            p,
            r_wheel: 0.314,
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
            0 => Lever::Park, 1 => Lever::Reverse, 3 => Lever::Drive, 4 => Lever::Manual, _ => Lever::Neutral,
        }
    }
    pub fn to_port(self) -> f64 {
        match self { Lever::Park => 0.0, Lever::Reverse => 1.0, Lever::Neutral => 2.0, Lever::Drive => 3.0, Lever::Manual => 4.0 }
    }
    /// Positions where the car is meant to be driven: D, and the tiptronic gate beside it
    pub fn drives(self) -> bool { matches!(self, Lever::Drive | Lever::Manual) }
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
        self.state.n_shaft = [ctx.bus.get(self.p.n_in1), ctx.bus.get(self.p.n_in2)];
        self.state.eng      = [ctx.bus.get(self.p.eng1).round().max(0.0) as usize,
                                ctx.bus.get(self.p.eng2).round().max(0.0) as usize];
        // Watch the shaft that carries the engaged gear
        self.state.n_in     = self.state.n_shaft[if self.state.gear % 2 == 1 { 0 } else { 1 }];
        // Wheel rpm to road speed through the coded tire size, the way a real TCU does it
        self.state.v_veh    = ctx.bus.get(self.p.n_wheel) * 2.0 * PI / 60.0 * self.r_wheel * 3.6;
        self.state.pedal    = ctx.bus.get(self.p.pedal);
        self.state.brake    = ctx.bus.get(self.p.brake);
        self.state.tip_up = ctx.bus.get(self.p.tip_up).round().max(0.0) as u32;

        // --- application
        for (r, t) in self.tasks.iter_mut() {
            if *r == rate { t.run(&mut self.state); }
        }

        // ShiftControl owns both forks and both clutches
        ctx.bus.set(self.p.sel1, self.state.sel[0] as f64);
        ctx.bus.set(self.p.sel2, self.state.sel[1] as f64);
        ctx.bus.set(self.p.cmd1, self.state.cmd[0]);
        ctx.bus.set(self.p.cmd2, self.state.cmd[1]);
        ctx.bus.set(self.p.clutch_cmd, self.state.clutch_cmd);
        ctx.bus.set(self.p.gear, self.state.gear as f64);
        ctx.bus.set(self.p.clutch_state, match self.state.clutch_state {
            ClutchState::Open => 0.0, ClutchState::Engaging => 1.0, ClutchState::Closed => 2.0,
        });
        ctx.bus.set(self.p.t_disc_est1, self.state.t_disc_est[0]);
        ctx.bus.set(self.p.t_disc_est2, self.state.t_disc_est[1]);
        ctx.bus.set(self.p.overheat, if self.state.overheat { 1.0 } else { 0.0 });
        ctx.bus.set(self.p.shift_phase, self.state.shift_phase as f64);
    }
}