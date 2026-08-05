/// Indicates efficiency. Swap the implementation to raise fidelity without touching the solver.
pub trait Efficiency: Send {
    fn eta(&self, n_rpm: f64, load: f64) -> f64;
}

/// v0 placeholder. Real η_i runs ~0.25 at idle to ~0.445 near peak - this is deliberately crude and will be replaced by a map
pub struct ConstEta(pub f64);

impl Efficiency for ConstEta {
    fn eta(&self, _n_rpm: f64, _load: f64) -> f64 { self.0 }
}