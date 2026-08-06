use revlab_core::Map2d;
/// Indicated efficiency vs speed and load. Fitted surface, typical of a small DI diesel - not measured from a specific engine
pub trait Efficiency: Send {
    fn eta(&self, n_rpm: f64, load: f64) -> f64;
}

pub struct MapEta(pub Map2d);

impl MapEta {
    pub fn di_diesel_typical() -> Self {
        let rpm = vec![800.0, 1500.0, 2500.0, 3500.0, 4500.0];
        let load = vec![0.0, 0.15, 0.35, 0.70, 1.00];
        // row-major: one row per load, columns across rpm
        let z = vec![
            0.10, 0.12, 0.13, 0.12, 0.11,   // load 0.00 - pumping/friction dominated
            0.26, 0.31, 0.34, 0.33, 0.31,   // load 0.15
            0.36, 0.42, 0.45, 0.44, 0.41,   // load 0.35 - best BSFC island
            0.38, 0.43, 0.45, 0.43, 0.40,   // load 0.70
            0.35, 0.39, 0.41, 0.39, 0.36,   // load 1.0 - rich, smoke limited
        ];
        MapEta(Map2d::new(rpm, load, z))
    }
}

impl Efficiency for MapEta {
    fn eta(&self, n_rpm: f64, load: f64) -> f64 { self.0.get(n_rpm, load) }
}