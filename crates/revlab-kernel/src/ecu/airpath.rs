use super::torque_model::TorqueModel;
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
    pub model: TorqueModel, // the same calibration TorqueToFuel inverts, so the limit lands where it means
}
impl SmokeLimiter {
    pub fn di_diesel_1_6() -> Self {
        SmokeLimiter { afr_min: 18.0, cylinders: 4.0, model: TorqueModel::ea288_cal() }
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
        let q_limit = (q_air_mg / self.afr_min).min(self.model.q_max);
        s.q_smoke_limit = q_limit;

        // --- express as a torque ceiling so the arbiter resolves it
        let t_limit_ind = self.model.t_ind(s.n_eng, q_limit);

        s.reqs[Source::Smoke as usize] = TorqueRequest {
            kind: ReqKind::MaxLimit,
            value: t_limit_ind - s.t_loss,
            active: true,
        };
    }
}

/// Exhaust manifold pressure, modeled. There is no sensor upstream of the turbine, so the ECU computes
/// it the way the plant makes it: turbine flow (air + fuel) through an orifice whose area follows the
/// vane demand, at an exhaust temperature mapped over fuel quantity. Geometry and gas constants are
/// calibration copies, close to the real turbo but not identical
pub struct ExhaustModel {
    pub a_t_min: f64,       // m²
    pub a_t_max: f64,
    pub gamma: f64,
    pub r_exh: f64,
    pub t_em_base: f64,     // K at zero fuel
    pub t_em_per_mg: f64,   // K per mg/stroke
    pub t_em_max: f64,
    pub cylinders: f64,
    pub p_amb: f64,         // Pa; no baro sensor modeled yet
}

impl ExhaustModel {
    pub fn ea288() -> Self {
        ExhaustModel {
            a_t_min: 1.25e-4, a_t_max: 6.3e-4, gamma: 1.35, r_exh: 287.0, t_em_base: 430.0,
            t_em_per_mg: 13.0, t_em_max: 1060.0, cylinders: 4.0, p_amb: 101_325.0,
        }
    }

    fn psi(&self, pr: f64) -> f64 {
        let g = self.gamma;
        let pr_crit = (2.0 / (g + 1.0)).powf(g / (g - 1.0));
        let pr = pr.clamp(pr_crit, 1.0);
        (2.0 * g / (g - 1.0) * (pr.powf(2.0 / g) - pr.powf((g + 1.0) / g))).max(0.0).sqrt()
    }
}

impl Task for ExhaustModel {
    fn name(&self) -> &'static str { "ExhaustModel" }

    fn run(&mut self, s: &mut EcuState) {
        let q = s.q_cmd.max(0.0);
        let m_t = s.m_air_est + q * 1e-6 * self.cylinders * s.n_eng / 120.0;
        let t_em = (self.t_em_base + self.t_em_per_mg * q).min(self.t_em_max);
        let a = self.a_t_min + s.vnt_cmd.clamp(0.0, 1.0) * (self.a_t_max - self.a_t_min);
        let k = a / (self.r_exh * t_em).sqrt();
        // Orifice flow rises monotonically with upstream pressure, so bisection always lands
        let (mut lo, mut hi) = (self.p_amb, 6.0 * self.p_amb);
        if k * hi * self.psi(self.p_amb / hi) < m_t { s.p_em_est = hi; return; }
        for _ in 0..30 {
            let mid = 0.5 * (lo + hi);
            if k * mid * self.psi(self.p_amb / mid) < m_t { lo = mid; } else { hi = mid; }
        }
        s.p_em_est = 0.5 * (lo + hi);
    }
}