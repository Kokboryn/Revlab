//! Instrument cluster: the gauges' own computer. It damps the fuel sender, runs the trip computer,
//! shows the indicated speed and keeps the clock. In the car wheel speed and consumption arrive over
//! CAN; here it reads the same signals from the bus for now

use crate::{Component, Ctx, Port, Trigger};
use revlab_core::SimDuration;

pub struct ClusterPorts {
    pub fuel_l: Port,       // in: tank sender, l
    pub fuel_lph: Port,     // in: consumption, l/h
    pub n_wheel: Port,      // in: wheel speed, rpm
    pub t_amb: Port,        // in: outside air sensor. K
    pub fuel_frac: Port,    // out: gauge position 0..1
    pub range_km: Port,
    pub clock_s: Port,      // out: time of day, s since midnight
    pub t_amb_disp: Port,   // out: displayed outside temperature, K
    pub v_disp: Port,       // out: indicated speed, km/h
    pub trip_km: Port,
    pub cons_avg: Port,     // out: trip average, l/100 km
}

pub struct Cluster {
    p: ClusterPorts,
    capacity_l: f64,
    r_wheel: f64,           // coded rolling radius
    speedo_gain: f64,       // indicated / true: a speedometer may never read low
    gauge_tau: f64,
    amb_tau: f64,
    range_dist: f64,        // km over which the range estimate forgets old driving
    clock_start: f64,
    started: bool,
    elapsed: f64,
    gauge_l: f64,
    amb_k: f64,
    cons_hist: f64,         // l/100 km behind the range estimate
    trip_km: f64,
    trip_l: f64,
}

const DT: f64 = 0.02;

impl Cluster {
    pub fn passat_b8(p: ClusterPorts, clock_start: f64) -> Self {
        Self {
            p, capacity_l: 66.0, r_wheel: 0.314, speedo_gain: 1.03, gauge_tau: 20.0, amb_tau: 20.0,
            range_dist: 30.0, clock_start, started: false, elapsed: 0.0, gauge_l: 0.0, amb_k: 0.0,
            cons_hist: 5.5, trip_km: 0.0, trip_l: 0.0,
        }
    }
}

impl Component for Cluster {
    fn triggers(&self) -> Vec<Trigger> {
        // Just before the streamer's slot, so the dash always gets this cycle's values
        vec![Trigger::Periodic { period: SimDuration::from_millis(20), offset: SimDuration::from_micros(4800) }]
    }

    fn step(&mut self, _trig: u16, ctx: &mut Ctx<'_>) {
        let fuel = ctx.bus.get(self.p.fuel_l);
        let lph = ctx.bus.get(self.p.fuel_lph);
        let v = ctx.bus.get(self.p.n_wheel).abs() * std::f64::consts::TAU / 60.0 * self.r_wheel;
        let v = if v < 0.5 { 0.0 } else { v };  // below ~2 km/h the cluster shows zero, so noise can't add distance
        let amb = ctx.bus.get(self.p.t_amb);
        if !self.started {
            // Wakes up showing where things stand instead of sweeping up from empty
            self.gauge_l = fuel;
            self.amb_k = amb;
            self.started = true;
        }
        self.elapsed += DT;
        self.gauge_l += (fuel - self.gauge_l) * DT / self.gauge_tau;
        self.amb_k += (amb - self.amb_k) * DT / self.amb_tau;

        let ds_km = v * DT / 1000.0;
        let dl = lph * DT / 3600.0;
        self.trip_km += ds_km;
        self.trip_l += dl;
        // Distance weighted history, written per increment so a crawl never divides by zero. Idling
        // burns fuel without distance, so it pushes the history up, as does in the car
        self.cons_hist += (dl * 100.0 - self.cons_hist * ds_km) / self.range_dist;
        let avg = if self.trip_km > 0.1 { self.trip_l / self.trip_km * 100.0 } else { 0.0 };

        ctx.bus.set(self.p.fuel_frac, (self.gauge_l / self.capacity_l).clamp(0.0, 1.0));
        ctx.bus.set(self.p.range_km, self.gauge_l / self.cons_hist.max(1.0) * 100.0);
        ctx.bus.set(self.p.clock_s, (self.clock_start + self.elapsed) % 86_400.0);
        ctx.bus.set(self.p.t_amb_disp, self.amb_k);
        ctx.bus.set(self.p.v_disp, v * 3.6 * self.speedo_gain);
        ctx.bus.set(self.p.trip_km, self.trip_km);
        ctx.bus.set(self.p.cons_avg, avg);
    }
}