pub mod crank_wheel;
pub mod cam_wheel;
pub mod map_maf;

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Fault {
    None,
    StuckAt(f64),
    Offset(f64),
    Drift { per_sec: f64 },
    OpenCircuit,
}

impl Fault {
    /// t_active is seconds since the fault was injected
    pub fn apply(&self, truth: f64, t_active: f64) -> Option<f64> {
        match *self {
            Fault::None => Some(truth),
            Fault::StuckAt(v) => Some(v),
            Fault::Offset(o) => Some(truth + o),
            Fault::Drift { per_sec } => Some(truth + per_sec * t_active),
            Fault::OpenCircuit => None,
        }
    }
}