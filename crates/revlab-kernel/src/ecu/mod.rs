pub mod idle;
pub mod torque;
pub mod diag;

use revlab_core::{SimDuration, SimTime};
use crate::{Component, Ctx, Port, Trigger};

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Rate { Ms1, Ms10, Ms100}

impl Rate {
    fn period(self) -> SimDuration {
        match self {
            Rate::Ms1 => SimDuration::from_millis(1),
            Rate::Ms10 => SimDuration::from_millis(10),
            Rate::Ms100 => SimDuration::from_millis(100),
        }
    }
    /// Staggered so rates never share a timestamp with each other or with the 1 ms plant step
    fn offset(self) -> SimDuration {
        match self {
            Rate::Ms1 => SimDuration::from_micros(200),
            Rate::Ms10 => SimDuration::from_millis(1),
            Rate::Ms100 => SimDuration::from_millis(2),
        }
    }
    fn from_trig(t: u16) -> Rate {
        match t { 0 => Rate::Ms1, 1 => Rate::Ms10, _ => Rate::Ms100 }
    }
}

/// The ECU's RAM image. Shared across all tasks - this is the only thing application code may touch. No sim bus, no plant state
pub struct EcuState {
    pub now: SimTime,
    // --- inputs, written by inout processing
    pub n_crank: f64,    // raw
    pub n_cam: f64,      // raw
    pub n_eng: f64,      // selected - what the control path uses
    pub speed_source: diag::SpeedSource,
    pub fault_mem: diag::FaultEntry,
    pub reqs: [torque::TorqueRequest; torque::N_SOURCES],
    // --- outputs, read by output drivers
    pub q_cmd: f64,     // mg/stroke
    pub t_arb: f64,     // Nm, arbitrated
}

pub trait Task: Send {
    fn name(&self) -> &'static str;
    fn run(&mut self, s: &mut EcuState);
}

pub struct Ecu {
    state: EcuState,
    tasks: Vec<(Rate, Box<dyn Task>)>,
    in_n_crank: Port,
    in_n_cam: Port,
    out_q_cmd: Port,
    out_t_arb: Port,
}

impl Ecu {
    pub fn new(in_n_crank: Port, in_n_cam: Port, out_q_cmd: Port, out_t_arb: Port, q_init: f64) -> Self {
        Ecu {
            state: EcuState {
                now: SimTime::ZERO,
                n_crank: 0.0,
                n_cam: 0.0,
                n_eng: 0.0,
                speed_source: diag::SpeedSource::Crank,
                fault_mem: diag::FaultEntry::CLEAR,
                reqs: [torque::TorqueRequest::INACTIVE; torque::N_SOURCES],
                t_arb: 0.0,
                q_cmd: q_init,
            },
            tasks: Vec::new(),
            in_n_crank, in_n_cam, out_q_cmd, out_t_arb,
        }
    }

    /// Registration order is execution order within a rate
    pub fn task(mut self, rate: Rate, t: Box<dyn Task>) -> Self {
        self.tasks.push((rate, t));
        self
    }
}

impl Component for Ecu {
    fn triggers(&self) -> Vec<Trigger> {
        [Rate::Ms1, Rate::Ms10, Rate::Ms100].iter()
            .map(|r| Trigger::Periodic { period: r.period(), offset: r.offset() })
            .collect()
    }

    fn step(&mut self, trig: u16, ctx: &mut Ctx<'_>) {
        let rate = Rate::from_trig(trig);

        // --- input processing
        self.state.now = ctx.now;
        self.state.n_crank = ctx.bus.get(self.in_n_crank);
        self.state.n_cam = ctx.bus.get(self.in_n_cam);
        self.state.n_eng = match self.state.speed_source {
            diag::SpeedSource::Crank => self.state.n_crank,
            diag::SpeedSource::Cam => self.state.n_cam,
        };

        // --- application
        for (r, t) in self.tasks.iter_mut() {
            if *r == rate { t.run(&mut self.state); }
        }

        // --- output drivers
        ctx.bus.set(self.out_q_cmd, self.state.q_cmd);
        ctx.bus.set(self.out_t_arb, self.state.t_arb);
    }
}