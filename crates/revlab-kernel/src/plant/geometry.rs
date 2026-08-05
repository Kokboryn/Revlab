use std::f64::consts::PI;

/// Primary measured inputs. Everything in `derived` comes from these.
#[derive(Copy, Clone, Debug)]
pub struct Geometry {
    pub bore: f64,                  // m
    pub stroke: f64,                // m
    pub cylinders: u32,
    pub conrod: f64,                // m, center to center
    pub compression_ratio: f64,
    pub recip_mass_per_cyl: f64,    // kg, piston + rings + pin + small end
    pub flywheel_mass: f64,         // kg (DMF primary side)
    pub flywheel_radius: f64,       // m, radius of gyration
}

impl Geometry {
    /// EA288 1.6 TDI. Bore/stroke give 1598 cc - a useful sanity check.
    pub fn ea288_16tdi() -> Self {
        Geometry {
            bore: 0.0795, stroke: 0.0805, cylinders: 4, conrod: 0.144, compression_ratio: 16.2, recip_mass_per_cyl: 0.75, flywheel_mass: 11.0, flywheel_radius: 0.115,
        }
    }

    pub fn crank_radius(&self) -> f64 { self.stroke / 2.0 }

    /// Swept volume, m³
    pub fn displacement(&self) -> f64 {
        PI / 4.0 * self.bore * self.bore * self.stroke * self.cylinders as f64
    }

    /// Mean piston speed, m/s - the input Chen-Flynn friction wants
    pub fn mean_piston_speed(&self, n_rpm: f64) -> f64 {
        2.0 * self.stroke * n_rpm / 60.0
    }

    /// Rotational inertia, kg·m². Flywheel disc dominates; reciprocating mass is added as a speed-averaged equivalent ( it is actually crank angle dependent, which mean value fidelity ignores)
    pub fn inertia_est(&self) -> f64 {
        let j_fly = 0.5 * self.flywheel_mass * self.flywheel_radius * self.flywheel_radius;
        let r = self.crank_radius();
        let j_recip = 0.5 * self.recip_mass_per_cyl * r * r * self.cylinders as f64;
        j_fly + j_recip + 0.02  // crank, rods, front pulley
    }
}