use std::ops::{Add, AddAssign, Sub};

#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Debug, Default, Hash)]
pub struct SimTime(u64);        // ns since start of the run

#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Debug, Hash)]
pub struct SimDuration(u64);    // ns

impl SimDuration {
    pub const ZERO: Self = SimDuration(0);
    pub const fn from_nanos(n: u64) -> Self { SimDuration(n) }
    pub const fn from_micros(u: u64) -> Self { SimDuration(u * 1_000) }
    pub const fn from_millis(m: u64) -> Self { SimDuration(m * 1_000_000) }

    /// Integer division: 1 GHz / hz
    pub const fn from_hz(hz: u32) -> Self { SimDuration(1_000_000_000 / hz as u64) }

    pub const fn as_nanos(self) -> u64 { self.0 }
    /// For physics only.
    pub fn as_secs_f64(self) -> f64 { self.0 as f64 * 1e-9 }
}

impl SimTime {
    pub const ZERO: Self = SimTime(0);
    pub const fn as_nanos(self) -> u64 { self.0 }
    pub fn as_secs_f64(self) -> f64 { self.0 as f64 * 1e-9 }
}

impl Add<SimDuration> for SimTime {
    type Output = SimTime;
    fn add(self, d: SimDuration) -> SimTime { SimTime(self.0 + d.0)}
}
impl AddAssign<SimDuration> for SimTime {
    fn add_assign(&mut self, d: SimDuration) { self.0 += d.0; }
}

impl Sub for SimTime {
    type Output = SimDuration;
    fn sub(self, rhs: SimTime) -> SimDuration { SimDuration(self.0 - rhs.0) }
}