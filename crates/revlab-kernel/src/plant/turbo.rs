use std::f64::consts::PI;
use revlab_core::SimDuration;
use crate::{Component, Ctx, Port, Trigger};
use super::gas::{psi, CP_AIR, GAMMA, R_AIR};
use super::exhaust::{CP_EXH, GAMMA_EXH, R_EXH};

#[derive(Copy, Clone)]
pub struct TurboPorts {
    // inputs
    pub p_amb: Port,
    pub t_amb: Port,
    pub p_im: Port,     // compressor discharge = manifold pressure
    pub p_em: Port,
    pub t_em: Port,
    pub vnt_pos: Port,  // Actual vane position from the actuator: 0 = closed, 1 = open
    // outputs
    pub m_comp: Port,
    pub t_comp: Port,   // post-intercooler charge temperature
    pub m_turb: Port,
    pub n_tc: Port,     // rpm, for the sensor
}
/// Turbocharger: compressor, turbine, and shaft as one device.
///
/// The compressor map is a normalized ellipse in (flow, head) rather than a measured table - the right shape, not a specific turbo.
/// Swapping in real map data later means replacing `flow_param` only
pub struct Turbo {
    omega: f64,                 // rad/s, shaft
    pub j_tc: f64,              // kg·m²
    pub r_c: f64,               // m, compressor-wheel radius
    pub psi_max: f64,           // peak head coefficient
    pub phi_max: f64,           // choke flow coefficient
    pub eta_c: f64,
    pub eta_t_peak: f64,        // turbine efficiency at the best vane position
    pub vnt_best: f64,          // vane position of peak efficiency
    pub eta_t_drop: f64,        // efficiency lost per (position error)²: nozzle losses closed, incidence losses open
    pub a_c_eff: f64,           // m², compressor flow path as an orifice, when the engine draws below ambient
    pub eta_mech: f64,
    pub a_t_min: f64,           // m², VNT vanes closed
    pub a_t_max: f64,           // m², VNT vanes open
    pub ic_eff: f64,            // intercooler effectiveness
    pub u_tip_max: f64,         // m/s, compressor tip speed limit; the shaft clamp follows from it
    ports: TurboPorts,
    dt: f64,
}

impl Turbo {
    pub const STEP: SimDuration = SimDuration::from_millis(1);

    pub fn vnt_small_diesel(ports: TurboPorts) -> Self {
        Turbo {
            omega: 2000.0,      // idling, barely turning
            j_tc: 2.8e-5,       // bigger wheels, slower spool
            r_c: 0.0225,        // 45 mm wheel: rated flow at ~90% of the speed limit
            psi_max: 1.0,
            phi_max: 0.20,
            eta_c: 0.70,
            eta_t_peak: 0.72,
            vnt_best: 0.6,
            eta_t_drop: 0.4,
            a_c_eff: 5.0e-4,
            eta_mech: 0.98,
            a_t_min: 1.2e-4,
            a_t_max: 6.5e-4,    // m². vanes open: keeps authority at rated flow
            u_tip_max: 520.0,
            ic_eff: 0.65,
            ports,
            dt: Self::STEP.as_secs_f64(),
        }
    }

    pub fn rpm(&self) -> f64 { self.omega * 60.0 / (2.0 * PI) }

    /// Ellipse map: (phi/ph_max)² + (psi/psi_max)² = 1
    /// Head rises with tip speed squared, so flow collapses when the pressure ratio exceeds what the
    /// current shaft speed can sustain
    ///
    /// When the engine draws below ambient, the slow wheel isn't the bottleneck: air goes through the
    /// housing as through any restriction, and the wheel windmills. The map only caps flow when the
    /// compressor has to push against boost
    fn comp_flow(&self, p_in: f64, t_in: f64, p_out: f64) -> f64 {
        let m_free = if p_out < p_in {
            self.a_c_eff * p_in / (R_AIR * t_in).sqrt() * psi(p_out / p_in)
        } else { 0.0 };
        let u = self.omega * self.r_c;
        if u < 1.0 { return m_free; }
        let pr = (p_out / p_in).max(1.0);
        let head = CP_AIR * t_in * (pr.powf((GAMMA - 1.0) / GAMMA) - 1.0);
        let psi_c = 2.0 * head / (u * u);
        if psi_c >= self.psi_max { return m_free; }     // past surge/stall
        let phi = self.phi_max * (1.0 - (psi_c / self.psi_max).powi(2)).max(0.0).sqrt();
        let rho = p_in / (R_AIR * t_in);
        (phi * rho * u * PI * self.r_c * self.r_c).max(m_free)
    }
}

impl Component for Turbo {
    fn triggers(&self) -> Vec<Trigger> {
        vec![Trigger::Periodic { period: Self::STEP, offset: SimDuration::from_micros(250) }]
    }

    fn step(&mut self, _t: u16, ctx: &mut Ctx<'_>) {
        let p_amb = ctx.bus.get(self.ports.p_amb);
        let t_amb = ctx.bus.get(self.ports.t_amb);
        let p_im  = ctx.bus.get(self.ports.p_im);
        let p_em  = ctx.bus.get(self.ports.p_em);
        let t_em  = ctx.bus.get(self.ports.t_em);
        let vnt   = ctx.bus.get(self.ports.vnt_pos).clamp(0.0, 1.0);

        // --- compressor
        let m_c = self.comp_flow(p_amb, t_amb, p_im).max(0.0);
        let pr_c = (p_im / p_amb).max(1.0);
        let d_t_isen = t_amb * (pr_c.powf((GAMMA - 1.0) / GAMMA) - 1.0);
        let t_after_comp = t_amb + d_t_isen / self.eta_c;
        let p_c = m_c * CP_AIR * d_t_isen / self.eta_c;

        // --- intercooler: effectiveness against ambient
        let t_charge = t_after_comp - self.ic_eff * (t_after_comp - t_amb);

        // --- turbine
        let a_t = self.a_t_min + vnt * (self.a_t_max - self.a_t_min);
        let eta_t = self.eta_t_peak - self.eta_t_drop * (vnt - self.vnt_best).powi(2);
        let m_t = if p_em > p_amb {
            a_t * p_em / (R_EXH * t_em).sqrt() * psi(p_amb / p_em)
        } else { 0.0 };
        let pr_t = (p_amb / p_em).clamp(1e-3, 1.0);
        let p_t = m_t * CP_EXH * t_em * eta_t * (1.0 - pr_t.powf((GAMMA_EXH - 1.0) / GAMMA_EXH));

        // --- shaft. Stiff at low speed (P/(J·omega) blows up), so sub-step and hold a floor
        const N_SUB: u32 = 10;
        let h = self.dt / N_SUB as f64;
        for _ in 0..N_SUB {
            let w = self.omega.max(200.0);
            self.omega += (p_t * self.eta_mech - p_c) / (self.j_tc * w) * h;
            self.omega = self.omega.clamp(200.0, self.u_tip_max / self.r_c);
        }

        ctx.bus.set(self.ports.m_comp, m_c);
        ctx.bus.set(self.ports.t_comp, t_charge);
        ctx.bus.set(self.ports.m_turb, m_t);
        ctx.bus.set(self.ports.n_tc, self.rpm());
    }
}