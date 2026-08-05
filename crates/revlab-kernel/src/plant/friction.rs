use super::geometry::Geometry;

/// Chen-Flynn: FMEP - a + b·p_max + c·v_p + d·v_p². The form is physical, the coefficients are fitted per engine family.
#[derive(Copy, Clone, Debug)]
pub struct ChenFlynn {
    pub a: f64,     // Pa, constant
    pub b: f64,     // dimensionless, × peak cylinder pressure
    pub c: f64,     // Pa.s/m
    pub d: f64,     // Pa·s²/m²
}

impl ChenFlynn {
    /// Typical DI diesel coefficients. Fitted, not derived.
    pub const DI_DIESEL: Self = ChenFlynn { a: 0.13e5, b: 0.008, c: 0.04e5, d: 0.005e5 };

    pub fn fmep(&self, p_max: f64, v_piston: f64) -> f64 {
        self.a + self.b * p_max + self.c * v_piston + self.d * v_piston * v_piston
    }

    /// Friction torque, Nm. Four-stroke: one cycle per two revolutions.
    pub fn torque(&self, g: &Geometry, n_rpm: f64, p_max: f64) -> f64 {
        let fmep = self.fmep(p_max, g.mean_piston_speed(n_rpm));
        fmep * g.displacement() / (4.0 * std::f64::consts::PI)
    }
}