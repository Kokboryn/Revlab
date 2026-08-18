pub mod idle;
pub mod torque;
pub mod diag;
pub mod observer;
pub mod airpath;
pub mod loss;
pub mod driver;
pub mod limits;

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
    pub n_crank: f64,    // raw
    pub n_cam: f64,      // raw
    pub n_eng: f64,      // selected - what the control path uses
    pub n_eng_seq: u64,
    pub speed_source: diag::SpeedSource,
    pub reqs: [torque::TorqueRequest; torque::N_SOURCES],
    pub n_model: f64,
    pub model_valid: bool,
    pub freeze_adaptation: bool,
    pub unattributable: bool,
    pub fault_mem: [diag::FaultEntry; diag::N_SENSORS],
    pub degraded: bool,
    pub q_cmd: f64,     // mg/stroke
    pub t_arb: f64,     // Nm, arbitrated
    pub target_rpm: f64,
    pub p_im_meas: f64,
    pub t_im_meas: f64,
    pub m_air_est: f64,
    pub q_smoke_limit: f64,
    pub t_loss: f64,
    pub t_ind_req: f64,
    pub pedal: f64,
    pub crank_valid: bool,
    pub cam_valid: bool,
    pub m_maf_meas: f64,
    m_air_est_sd: f64,
}

pub trait Task: Send {
    fn name(&self) -> &'static str;
    fn run(&mut self, s: &mut EcuState);
}

/// Every port the ECU touches, named. Construction is by field name, so the ordering mistakes that
/// plague a 16-argument constructor become impossible
#[derive(Copy, Clone)]
pub struct EcuPorts {
    // inputs
    pub n_crank: Port,
    pub n_cam: Port,
    pub speed_req: Port,
    pub p_im: Port,
    pub t_im: Port,
    pub in_pedal: Port,
    pub crank_valid: Port,
    pub cam_valid: Port,
    pub m_maf: Port,
    // outputs
    pub q_cmd: Port,
    pub t_arb: Port,
    pub dtc: Port,
    pub n_model: Port,
    pub q_lim: Port,
    pub m_air_est: Port,
    pub t_loss: Port,
    pub t_ind_req: Port,
}

pub struct Ecu {
    state: EcuState,
    tasks: Vec<(Rate, Box<dyn Task>)>,
    p: EcuPorts,
}

impl Ecu {
    pub fn new(p: EcuPorts, q_init: f64) -> Self {
        Ecu {
            state: EcuState {
                now: SimTime::ZERO,
                n_crank: 0.0,
                n_cam: 0.0,
                n_eng: 0.0,
                n_eng_seq: 0,
                speed_source: diag::SpeedSource::Crank,
                fault_mem: [diag::FaultEntry::CLEAR; diag::N_SENSORS],
                degraded: false,
                reqs: [torque::TorqueRequest::INACTIVE; torque::N_SOURCES],
                t_arb: 0.0,
                q_cmd: q_init,
                n_model: 0.0,
                target_rpm: 800.0,
                p_im_meas: 101325.0,
                t_im_meas: 293.15,
                m_air_est: 0.0,
                q_smoke_limit: 0.0,
                model_valid: false,
                freeze_adaptation: false,
                unattributable: false,
                t_loss: 0.0,
                t_ind_req: 0.0,
                pedal: 0.0,
                crank_valid: true,
                cam_valid: true,
                m_maf_meas: 0.0,
                m_air_est_sd: 0.5,
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

impl Component for Ecu {
    fn triggers(&self) -> Vec<Trigger> {
        [Rate::Ms1, Rate::Ms10, Rate::Ms100].iter()
            .map(|r| Trigger::Periodic { period: r.period(), offset: r.offset() })
            .collect()
    }

    fn step(&mut self, trig: u16, ctx: &mut Ctx<'_>) {
        let rate = Rate::from_trig(trig);

        // --- input processing
        self.state.now          = ctx.now;
        self.state.n_crank      = ctx.bus.get(self.p.n_crank);
        self.state.n_cam        = ctx.bus.get(self.p.n_cam);
        self.state.target_rpm   = ctx.bus.get(self.p.speed_req);
        self.state.p_im_meas    = ctx.bus.get(self.p.p_im);
        self.state.t_im_meas    = ctx.bus.get(self.p.t_im);
        self.state.pedal        = ctx.bus.get(self.p.in_pedal);
        self.state.crank_valid  = ctx.bus.get(self.p.crank_valid) > 0.5;
        self.state.cam_valid    = ctx.bus.get(self.p.cam_valid) > 0.5;
        self.state.m_maf_meas   = ctx.bus.get(self.p.m_maf);

        let n_new = match self.state.speed_source {
            diag::SpeedSource::Crank if self.state.crank_valid => Some(self.state.n_crank),
            diag::SpeedSource::Cam   if self.state.cam_valid => Some(self.state.n_cam),
            _ => None
        };
        if let Some(n) = n_new {
            // Real firmware gets a "new capture" flag from the timer unit. A bit-identical value means the port was not rewritten.
            if n != self.state.n_eng {
                self.state.n_eng = n;
                self.state.n_eng_seq += 1;
            }
        }

        // --- application
        for (r, t) in self.tasks.iter_mut() {
            if *r == rate { t.run(&mut self.state); }
        }

        // --- output drivers
        let worst = self.state.fault_mem.iter()
            .map(|e| match e.state {
                diag::DtcState::Passed => 0.0,
                diag::DtcState::Pending => 1.0,
                diag::DtcState::Confirmed => 2.0,
            })
            .fold(0.0_f64, f64::max);
        ctx.bus.set(self.p.q_cmd, self.state.q_cmd);
        ctx.bus.set(self.p.t_arb, self.state.t_arb);
        ctx.bus.set(self.p.dtc, worst);
        ctx.bus.set(self.p.n_model, self.state.n_model);
        ctx.bus.set(self.p.q_lim, self.state.q_smoke_limit);
        ctx.bus.set(self.p.m_air_est, self.state.m_air_est);
        ctx.bus.set(self.p.t_loss, self.state.t_loss);
        ctx.bus.set(self.p.t_ind_req, self.state.t_ind_req);
    }
}