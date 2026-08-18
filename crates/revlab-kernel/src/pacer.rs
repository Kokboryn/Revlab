use std::time::{Duration, Instant};
use revlab_core::{SimDuration, SimTime};
use crate::{Component, Ctx, Trigger};

/// Throttles simulation time to wall-clock time
///
/// This is the ONLY place in the codebase that reads the wall clock, and it is deliberately inert:
/// it writes no port and touches no state, so it cannot affect the numerical result. A run paced to
/// real time and the same run unpaced produce identical telemetry. That is why sim time was made the
/// source of truth rather than being derived from the clock - real-time is an optional governor, not a
/// different mode
pub struct Pacer {
    /// 1.0 = real time, 0.25 = quarter speed, 10.0 = ten times faster.
    pub speed: f64,
    period: SimDuration,
    start_wall: Option<Instant>,
    start_sim: SimTime,
    worst_lag: f64,         // s, how far behind we ever fell
}

impl Pacer {
    pub fn new(speed: f64) -> Self {
        Pacer {
            speed: speed.max(1e-6),
            // 10 ms granularity: fine enough to look smooth, coarse enough that sleep overhead stays negligible
            period: SimDuration::from_millis(10),
            start_wall: None,
            start_sim: SimTime::ZERO,
            worst_lag: 0.0,
        }
    }

    /// Seconds the sim ever fell behind the requested pace. Non-zero means the machine could not keep up
    pub fn worst_lag(&self) -> f64 { self.worst_lag }
}

impl Component for Pacer {
    fn triggers(&self) -> Vec<Trigger> {
        vec![Trigger::Periodic {period: self.period, offset: SimDuration::from_micros(900) }]
    }

    fn step(&mut self, _trig: u16, ctx: &mut Ctx<'_>) {
        let wall0 = match self.start_wall {
            Some(w) => w,
            None => {
                // First execution defines the origin, so start-up cost is not counted as lag
                self.start_wall = Some(Instant::now());
                self.start_sim = ctx.now;
                return;
            }
        };

        let sim_elapsed = (ctx.now - self.start_sim).as_secs_f64() / self.speed;
        let wall_elapsed = wall0.elapsed().as_secs_f64();

        if sim_elapsed > wall_elapsed {
            std::thread::sleep(Duration::from_secs_f64(sim_elapsed - wall_elapsed));
        } else {
            let lag = wall_elapsed - sim_elapsed;
            if lag > self.worst_lag { self.worst_lag = lag; }
        }
    }
}