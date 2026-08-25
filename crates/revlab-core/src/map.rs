/// Returns (lower index, fraction within the interval). Shared by both map types: clamping at
/// the edges, never extrapolating.
fn locate(bp: &[f64], v: f64) -> (usize, f64) {
    if v <= bp[0] { return (0, 0.0); }
    if v >= bp[bp.len() - 1] { return (bp.len() - 2, 1.0) }
    // Linear scan: those maps are small, and it avoids the branch order surprises of partition_point on floats
    let mut i = 0;
    while i + 2 < bp.len() && v >= bp[i + 1] { i += 1; }
    (i, (v - bp[i]) / (bp[i + 1] - bp [i]))
}

/// 1D breakpoint map with linear interpolation and edge clamping.
#[derive(Clone, Debug)]
pub struct Map1d {
    x_bp: Vec<f64>,     // ascending
    y: Vec<f64>,
}

impl Map1d {
    pub fn new(x_bp: Vec<f64>, y: Vec<f64>) -> Self {
        assert!(x_bp.len() >= 2, "map needs at least two breakpoints");
        assert_eq!(y.len(), x_bp.len(), "map dimensions mismatch");
        assert!(x_bp.windows(2).all(|w| w[0] < w[1]), "x breakpoints not ascending");
        Map1d { x_bp, y }
    }

    pub fn get(&self, x: f64) -> f64 {
        let (i, fx) = locate(&self.x_bp, x);
        self.y[i] + (self.y[i + 1] - self.y[i]) * fx
    }
}

/// 2D breakpoint map with bilinear interpolation and edge clamping. Clamping (not extrapolation) matches real ECU behavior - outside the calibrated range you get the edge value, never an invented one
#[derive(Clone, Debug)]
pub struct Map2d {
    x_bp: Vec<f64>,     // ascending
    y_bp: Vec<f64>,     // ascending
    z: Vec<f64>,        // row-major, len = y_bp.len() * x_bp.len()
}

impl Map2d {
    pub fn new(x_bp: Vec<f64>, y_bp: Vec<f64>, z: Vec<f64>) -> Self {
        assert_eq!(z.len(), x_bp.len() * y_bp.len(), "map dimensions mismatch");
        assert!(x_bp.len() >= 2 && y_bp.len() >=2, "map needs at least two breakpoints per axis");
        assert!(x_bp.windows(2).all(|w| w[0] < w[1]), "x breakpoints not ascending");
        assert!(y_bp.windows(2).all(|w| w[0] < w[1]), "y breakpoints not ascending");
        Map2d { x_bp, y_bp, z }
    }

    pub fn get(&self, x: f64, y: f64) -> f64 {
        let (i, fx) = locate(&self.x_bp, x);
        let (j, fy) = locate(&self.y_bp, y);
        let nx = self.x_bp.len();
        let z00 = self.z[j * nx + i];
        let z10 = self.z[j * nx + i + 1];
        let z01 = self.z[(j + 1) * nx + i];
        let z11 = self.z[(j + 1) * nx + i + 1];
        let a = z00 + (z10 - z00) * fx;
        let b = z01 + (z11 - z01) * fx;
        a + (b - a) * fy
    }
}