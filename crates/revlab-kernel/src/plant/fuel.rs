/// Fuel properties. Not an engine property - swapping this is how B7 / HVO / gasoline eventually enter the model
#[derive(Copy, Clone, Debug)]
pub struct Fuel {
    pub lhv: f64,           // J/kg, lower heating value
    pub density: f64,       // kg/m³ @ 15 °C
    pub stoich_afr: f64,    // kg air / kg fuel
    pub cetane: f64,
}

impl Fuel {
    pub const DIESEL_B7: Self = Fuel {
        lhv: 42.7e6, density: 835.0, stoich_afr: 14.5, cetane: 51.0,
    };

    pub const HVO: Self = Fuel {
        lhv: 44.0e6, density: 780.0, stoich_afr: 14.9, cetane: 75.0,
    };
}