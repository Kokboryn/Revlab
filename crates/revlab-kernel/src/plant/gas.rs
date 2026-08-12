/// Dry air properties. Accurate below ~1000 K; combustion products differ enough that the exhaust side will want its own constants
pub const R_AIR: f64 = 287.05;      // J/(kg·K)
pub const CP_AIR: f64 = 1005.0;     // J/(kg·K)
pub const GAMMA: f64 = 1.400;
pub const T0_C: f64 = 273.15;

/// Critical pressure ratio - below this the orifice is choked and mass flow stops responding to downstream pressure.
pub const PR_CRIT: f64 = 0.5283;

/// Compressible flow function for an isentropic orifice. 'pr' is downstream/upstream pressure.
pub fn psi(pr: f64) -> f64 {
    let pr = pr.clamp(0.0, 1.0);
    if pr <= PR_CRIT {
        // choked: flow depends only on upstream conditions
        GAMMA.sqrt() * (2.0 / (GAMMA + 1.0)).powf((GAMMA + 1.0) / (2.0 * (GAMMA - 1.0)))
    } else {
        let a = pr.powf(2.0 / GAMMA);
        let b = pr.powf((GAMMA + 1.0) / GAMMA);
        (2.0 * GAMMA / (GAMMA - 1.0) * (a - b)).max(0.0).sqrt()
    }
}