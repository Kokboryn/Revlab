pub mod idle;
pub mod torque;
pub mod diag;
pub mod observer;
pub mod airpath;
pub mod loss;
pub mod driver;
pub mod limits;
pub mod torque_model;
pub mod boost;

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
    pub q_cmd: f64,         // mg/stroke
    pub t_arb: f64,         // Nm, arbitrated
    pub target_rpm: f64,
    pub p_im_meas: f64,
    pub t_im_meas: f64,
    pub m_air_est: f64,
    pub q_smoke_limit: f64,
    pub t_loss: f64,
    pub t_ind_req: f64,
    pub t_act: f64,         // Nm at the crank: what the engine actually makes, by the CUS's own model
    pub t_no_icpt: f64,     // Nm at the crank, arbitration without the TCU's request
    pub tcu_icpt_kind: u8,  // TCU torque intervention: 0 none, 1 cut (ceiling), 2 raise (floor)
    pub tcu_icpt_t: f64,    // Nm at the crank
    pub tcu_gear: u8,       // engaged gear, from the TCU over CAN
    pub tcu_clutch: u8,     // driveline as the TCU measures it: 0 open, 1 slipping, 2 locked
    pub tcu_shift: u8,      // shift phase: 0 idle, 1 prepare, 2+ clutches handing over
    pub tcu_t_clutch: f64,  // Nm, what the clutches take from the engine
    pub f_road_est: f64,    // N, road force learned by the observer while the driveline is locked
    pub gb1_lost: bool,             // Gearbox_1 timed out
    pub gb1_dtc: diag::DtcState,    // stored lost-communication code for the TCU
    pub p_set: f64,         // Pa, boost setpoint
    pub vnt_cmd: f64,       // VNT position demand: 0 closed, 1 open
    pub pedal: f64,
    pub crank_valid: bool,
    pub cam_valid: bool,
    pub m_maf_meas: f64,
    m_air_est_sd: f64,
    pub t_ect_c: f64,
    pub warmup_mult: f64,
    pub t_bias: f64,
    pub p_em_est: f64,      // Pa, modeled exhaust manifold pressure
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
    pub t_ect_c: Port,
    pub tcu_icpt_kind: Port,
    pub tcu_icpt_t: Port,
    pub tcu_gear: Port,
    pub tcu_clutch: Port,
    pub tcu_shift: Port,
    pub tcu_t_clutch: Port,
    pub gb1_ctr: Port,
    // outputs
    pub q_cmd: Port,
    pub t_arb: Port,
    pub dtc: Port,
    pub n_model: Port,
    pub q_lim: Port,
    pub m_air_est: Port,
    pub t_loss: Port,
    pub t_ind_req: Port,
    pub t_act: Port,
    pub vnt_cmd: Port,
    pub p_set: Port,
    pub freeze: Port,
    pub t_bias: Port,
    pub speed_source: Port,
    pub p_em_est: Port,
    pub f_road_est: Port,
    pub dtc_comm: Port,     // U0101, lost communication with the TCU
    pub t_no_icpt: Port,
}

pub struct Ecu {
    state: EcuState,
    tasks: Vec<(Rate, Box<dyn Task>)>,
    p: EcuPorts,
    rx_gb1: crate::can::RxMonitor,
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
                t_act: 0.0,
                t_no_icpt: 0.0,
                tcu_icpt_kind: 0,
                tcu_icpt_t: 0.0,
                p_set: 101325.0,
                vnt_cmd: 1.0,
                pedal: 0.0,
                crank_valid: true,
                cam_valid: true,
                m_maf_meas: 0.0,
                m_air_est_sd: 0.5,
                t_ect_c: 20.0,
                warmup_mult: 1.0,
                t_bias: 0.0,
                p_em_est: 101_325.0,
                tcu_gear: 0,
                tcu_clutch: 0,
                tcu_shift: 0,
                tcu_t_clutch: 0.0,
                f_road_est: 0.0,
                gb1_lost: false,
                gb1_dtc: diag::DtcState::Passed,
            },
            tasks: Vec::new(),
            p,
            rx_gb1: crate::can::RxMonitor::new(0.1),
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
        self.state.now              = ctx.now;
        self.state.n_crank          = ctx.bus.get(self.p.n_crank);
        self.state.n_cam            = ctx.bus.get(self.p.n_cam);
        self.state.target_rpm       = ctx.bus.get(self.p.speed_req);
        self.state.p_im_meas        = ctx.bus.get(self.p.p_im);
        self.state.t_im_meas        = ctx.bus.get(self.p.t_im);
        self.state.pedal            = ctx.bus.get(self.p.in_pedal);
        self.state.crank_valid      = ctx.bus.get(self.p.crank_valid) > 0.5;
        self.state.cam_valid        = ctx.bus.get(self.p.cam_valid) > 0.5;
        self.state.m_maf_meas       = ctx.bus.get(self.p.m_maf);
        self.state.t_ect_c          = ctx.bus.get(self.p.t_ect_c) - 273.15;
        self.state.tcu_icpt_kind    = ctx.bus.get(self.p.tcu_icpt_kind).round().max(0.0) as u8;
        self.state.tcu_icpt_t       = ctx.bus.get(self.p.tcu_icpt_t);
        self.state.tcu_gear         = ctx.bus.get(self.p.tcu_gear).round().max(0.0) as u8;
        self.state.tcu_clutch       = ctx.bus.get(self.p.tcu_clutch).round().max(0.0) as u8;
        self.state.tcu_shift        = ctx.bus.get(self.p.tcu_shift).round().max(0.0) as u8;
        self.state.tcu_t_clutch     = ctx.bus.get(self.p.tcu_t_clutch);
        self.state.gb1_lost = self.rx_gb1.lost(ctx.bus.get(self.p.gb1_ctr), ctx.now);
        if self.state.gb1_lost {
            // Stale Gearbox_1 values are worse than none: a torque cut frozen mid-shift would hold
            // the engine down indefinitely.
            self.state.tcu_icpt_kind = 0;
            self.state.tcu_icpt_t = 0.0;
            self.state.tcu_gear = 0;
            self.state.tcu_clutch = 0;
            self.state.tcu_shift = 0;
            self.state.tcu_t_clutch = 0.0;
            self.state.gb1_dtc = diag::DtcState::Confirmed;
        }

        let n_new = match self.state.speed_source {
            diag::SpeedSource::Crank if self.state.crank_valid => Some(self.state.n_crank),
            diag::SpeedSource::Cam if self.state.cam_valid => Some(self.state.n_cam),
            // The selected sensor has lost its signal: fall back to the other at once. Holding the
            // last reading leaves the governor blind, and the engine runs away
            _ if self.state.cam_valid => Some(self.state.n_cam),
            _ if self.state.crank_valid => Some(self.state.n_crank),
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
        ctx.bus.set(self.p.t_act, self.state.t_act);
        ctx.bus.set(self.p.vnt_cmd, self.state.vnt_cmd);
        ctx.bus.set(self.p.p_set, self.state.p_set);
        ctx.bus.set(self.p.freeze, if self.state.freeze_adaptation { 1.0 } else { 0.0 });
        ctx.bus.set(self.p.t_bias, self.state.t_bias);
        ctx.bus.set(self.p.speed_source, match self.state.speed_source {
            diag::SpeedSource::Crank => 0.0,
            diag::SpeedSource::Cam => 1.0,
        });
        ctx.bus.set(self.p.p_em_est, self.state.p_em_est);
        ctx.bus.set(self.p.f_road_est, self.state.f_road_est);
        ctx.bus.set(self.p.dtc_comm, if self.state.gb1_dtc == diag::DtcState::Confirmed { 2.0 } else { 0.0 });
        ctx.bus.set(self.p.t_no_icpt, self.state.t_no_icpt);
    }
}