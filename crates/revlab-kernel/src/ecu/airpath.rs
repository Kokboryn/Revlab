use std::f64::consts::PI;
use super::torque::{ReqKind, Source, TorqueRequest};
use super::{EcuState, Task};

/// Cylinder air charge estimate, then the smoke limit.
///
/// The ECU has no access to plant air flow - it runs the speed density equation itself, from MAP and IAT, with its own volumetric
/// efficiency calibration. That estimate is deliberately imperfect
pub struct AirEstimator {
    pub eta_vol_cal: f64,
    pub displacement: f64,
    pub tau: f64,           // s, crossover between the two sources
    blended: f64,
    last: Option<revlab_core::SimTime>,
}

impl Task for AirEstimator {
    fn name(&self) -> &'static str { "AirEstimator" }

    fn run(&mut self, s: &mut EcuState) {
        const R_AIR: f64 = 287.05;
        let sd = self.eta_vol_cal * self.displacement * s.n_eng * s.p_im_meas / (120.0 * R_AIR * s.t_im_meas.max(200.0));

        let dt = match self.last {
            Some(t0) => (s.now - t0).as_secs_f64(),
            None => { self.last = Some(s.now); self.blended = sd; s.m_air_est_sd = sd; s.m_air_est = sd.max(0.0); return; }
        };
        self.last = Some(s.now);
        if dt <= 0.0 || dt > 0.5 { return; }

        // Complementary filter: speed density carries the fast content, MAF correct the slow bias
        let a = dt / (self.tau + dt);
        self.blended = self.blended + (sd - s.m_air_est_sd) + a * (s.m_maf_meas - self.blended);
        s.m_air_est_sd = sd;
        s.m_air_est = self.blended.max(0.0);
    }
}

impl AirEstimator {
    pub fn di_diesel_1_6() -> Self {
        AirEstimator {
            eta_vol_cal: 0.88,
            displacement: 1.598e-3,
            tau: 0.5,
            blended: 0.0,
            last: None,
        }
    }
}

pub struct SmokeLimiter {
    pub afr_min: f64,       // soot threshold, above stoichiometric
    pub cylinders: f64,
    pub lhv_cal: f64,
    pub eta_ind_cal: f64,
    pub q_max: f64,         // mg/stroke, injector limit
}
impl SmokeLimiter {
    pub fn di_diesel_1_6() -> Self {
        SmokeLimiter {
            afr_min: 18.0,
            cylinders: 4.0,
            lhv_cal: 42.7e6,
            eta_ind_cal: 0.19,
            q_max: 60.0,
        }
    }
}

impl Task for SmokeLimiter {
    fn name(&self) -> &'static str { "Smoke Limiter" }

    fn run(&mut self, s: &mut EcuState) {
        // No usable air estimate yet: do not limit. A limiter that defaults to zero fuel turns a startup
        // transient into a stall.
        if s.m_air_est < 1e-4 {
            s.reqs[Source::Smoke as usize] = TorqueRequest::INACTIVE;
            return;
        }
        // Air estimate comes from AirEstimator, which blends MAF against speed density. Two sources
        // of truth for the same quantity is what made the blend inert.

        // --- air per stroke -> maximum fuel per stroke at afr_min
        let cycles_per_s = s.n_eng / 120.0;
        let q_air_mg = if cycles_per_s > 1e-3 {
            s.m_air_est / (cycles_per_s * self.cylinders) * 1e6
        } else { 0.0 };
        let q_limit = (q_air_mg / self.afr_min).min(self.q_max);
        s.q_smoke_limit = q_limit;

        // --- express as a torque ceiling so the arbiter resolves it
        let t_limit_ind = q_limit * 1e-6 * self.cylinders * self.lhv_cal * self.eta_ind_cal / (4.0 * PI);

        s.reqs[Source::Smoke as usize] = TorqueRequest {
            kind: ReqKind::MaxLimit,
            value: t_limit_ind - s.t_loss,
            active: true,
        };
    }
}