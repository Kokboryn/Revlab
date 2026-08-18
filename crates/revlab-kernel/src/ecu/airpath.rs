use std::f64::consts::PI;
use super::torque::{ReqKind, Source, TorqueRequest};
use super::{EcuState, Task};

/// Cylinder air charge estimate, then the smoke limit.
///
/// The ECU has no access to plant air flow - it runs the speed density equation itself, from MAP and IAT, with its own volumetric
/// efficiency calibration. That estimate is deliberately imperfect
pub struct SmokeLimiter {
    pub afr_min: f64,       // soot threshold, above stoichiometric
    pub eta_vol_cal: f64,
    pub displacement: f64,  // m³
    pub cylinders: f64,
    pub lhv_cal: f64,
    pub eta_ind_cal: f64,
    pub q_max: f64,         // mg/stroke, injector limit
}

impl SmokeLimiter {
    pub fn di_diesel_1_6() -> Self {
        SmokeLimiter {
            afr_min: 18.0,
            eta_vol_cal: 0.88,      // plant runs 0.90 - deliberate mismatch
            displacement: 1.598e-3,
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
        const R_AIR: f64 = 287.05;

        // --- speed-density, ECU's own calibration
        let m_air = self.eta_vol_cal * self.displacement * s.n_eng * s.p_im_meas / (120.0 * R_AIR * s.t_im_meas.max(200.0));
        s.m_air_est = m_air.max(0.0);

        // --- air per stroke -> maximum fuel per stroke at afr_min
        let cycles_per_s = s.n_eng / 120.0;
        let q_air_mg = if cycles_per_s > 1e-3 {
            s.m_air_est / (cycles_per_s * self.cylinders) * 1e6
        } else { 0.0 };
        let q_limit = (q_air_mg / self.afr_min).min(self.q_max);
        s.q_smoke_limit = q_limit;

        // --- express as a torque ceiling so the arbiter resolves it
        let t_limit_ind = q_limit * 1e-6 * self.cylinders * self.lhv_cal * self.eta_ind_cal / (4.0 * PI);
        let t_limit_crank = t_limit_ind - s.t_loss;

        s.reqs[Source::Smoke as usize] = TorqueRequest {
            kind: ReqKind::MaxLimit,
            value: t_limit_crank,
            active: true,
        };
    }
}