use revlab_core::SimDuration;
use crate::{Component, Ctx, Port, Trigger};

#[derive(Copy, Clone)]
pub struct ClutchPorts {
    // inputs
    pub omega_eng: Port,    // crank speed, one tick old
    pub cmd: Port,          // 0 = fully open, 1 = fully clamped
    pub t_out: Port,        // reaction from the driveline at the input shaft
    pub j_ref: Port,        // vehicle inertia reflected onto the input shaft
    pub v_veh: Port,
    pub t_amb: Port,
    // outputs
    pub omega_in: Port,     // transmission input shaft
    pub t_clutch: Port,     // torque on the crank, positive = retarding
    pub slip: Port,         // rad/s, engine minus input
    pub q_clutch: Port,     // W, friction power
    pub t_disc: Port,
}

/// One dry clutch of a dual clutch pack. Owns the input shaft speed, so with the clutch open the engine
/// and the vehicle are genuinely independent -- the second degree of freedom the rigit driveline could not have.
///
/// Lok is a stiff spring damper rather than a solved constraint: the bus is f64 slots, so an iterative
/// constraint solve across components is not practical. Stiffness is set so residual twist stays under
/// a degree, which is indistinguishable from locket at this timestep.
pub struct Clutch {
    omega_in: f64,          // rad/s
    theta_rel: f64,         // rad, accumulated twist while gripping
    t_disc: f64,            // K, lining and pressure plate as one lumped mass
    pub j_in: f64,          // kg·m², input shaft + gearset, engine side of the diff
    pub t_cap_cold: f64,    // Nm, torque capacity at full clamp with fresh cool lining
    pub c_disc: f64,        // J/K
    pub ua_still: f64,      // W/K, bell housing to ambient at rest
    pub ua_speed: f64,      // W/K, per m/s, forced convection with road speed
    pub t_fade_start: f64,  // K, mu begins falling
    pub t_fade_end: f64,    // K, mu at its floor
    pub mu_floor: f64,      // fraction of cold mu when fully faded
    pub k_lock: f64,        // Nm/rad
    pub c_lock: f64,        // Nm·s/rad
    ports: ClutchPorts,
    dt: f64,
}

impl Clutch {
    pub const STEP: SimDuration = SimDuration::from_millis(1);

    pub fn dq200_k1(ports: ClutchPorts, omega_in_init: f64, t_amb_init: f64) -> Self {
        Clutch {
            omega_in: omega_in_init,
            theta_rel: 0.0,
            t_disc: t_amb_init,
            j_in: 0.02,
            t_cap_cold: 330.0,
            // ~2 kg of lining and pressure plate. One 39.5 kJ launch is a 40 C rise, which is why
            // repeated hill starts are what kills these.
            c_disc: 1000.0,
            // A dry pack is cooled by air through the bell housing, so cooling depends on road speed.
            // That is exactly why failure happens in traffic and not on a freeway.
            ua_still: 8.0,
            ua_speed: 1.2,
            t_fade_start: 273.15 + 250.0,
            t_fade_end: 273.15 + 450.0,
            mu_floor: 0.55,
            k_lock: 4000.0,
            c_lock: 40.0,
            ports,
            dt: Self::STEP.as_secs_f64(),
        }
    }

    /// Organic linings lose grip as they heat: the binder starts to break down and outgas. Falls to
    /// mu_floor and stays there -- recoverable here, since permanent loss is wear rather than fade.
    fn mu_frac(&self) -> f64 {
        let x = (self.t_disc - self.t_fade_start) / (self.t_fade_end - self.t_fade_start);
        1.0 - x.clamp(0.0, 1.0) * (1.0 - self.mu_floor)
    }
}

impl Component for Clutch {
    fn triggers(&self) -> Vec<Trigger> {
        vec![Trigger::Periodic { period: Self::STEP, offset: SimDuration::ZERO }]
    }

    fn step(&mut self, _trig: u16, ctx: &mut Ctx<'_>) {
        let omega_eng = ctx.bus.get(self.ports.omega_eng);
        let cmd = ctx.bus.get(self.ports.cmd).clamp(0.0, 1.0);
        let t_out = ctx.bus.get(self.ports.t_out);
        let slip = omega_eng - self.omega_in;

        // Capacity rises with clamp force. Squared because the plate travel closes the gap before
        // it starts loading: the first half of the pedal does almost nothing, which is what makes a
        // clutch driveable.
        let cap = self.t_cap_cold * self.mu_frac() * cmd * cmd;

        // Spring damper first, then decide whether the pack can hold it
        self.theta_rel += slip * self.dt;
        let t_stick = self.k_lock * self.theta_rel + self.c_lock * slip;

        let t_c = if t_stick.abs() <= cap {
            t_stick                                 // gripping
        } else {
            // Slipping: Coulomb at capacity, opposing relative motion. Reset the twist so re-grip starts
            // from zero rather than a wound spring
            self.theta_rel = 0.0;
            cap * slip.signum()
        };

        // In neutral j_ref is 0 and the input shaft carries only its own inertia, so it spins up freely
        // against the clutch -- correct, nothing is connected.
        let j_tot = (self.j_in + ctx.bus.get(self.ports.j_ref)).max(1e-4);
        self.omega_in += (t_c - t_out) / j_tot * self.dt;
        // No floor: a clutch that has faded below the grade load lets the car roll back, and that is the
        // failure this models
        ctx.bus.set(self.ports.omega_in, self.omega_in);
        ctx.bus.set(self.ports.t_clutch, t_c);
        ctx.bus.set(self.ports.slip, slip);
        // Friction power. Zero while gripping, kilowatts during a launch -- this is what feeds the
        // clutch thermal state in the next stage.
        ctx.bus.set(self.ports.q_clutch, if t_stick.abs() > cap { (t_c * slip).abs() } else { 0.0 });

        // Thermal state. Slip power in, forced convection out, scaled by road speed.
        let q_in = if t_stick.abs() > cap { (t_c * slip).abs() } else { 0.0 };
        let v_veh = ctx.bus.get(self.ports.v_veh).abs();
        let ua = self.ua_still + self.ua_speed * v_veh;
        let t_amb = ctx.bus.get(self.ports.t_amb);
        self.t_disc += (q_in - ua * (self.t_disc - t_amb)) / self.c_disc * self.dt;

        ctx.bus.set(self.ports.t_disc, self.t_disc);
        ctx.bus.set(self.ports.q_clutch, q_in);
    }
}