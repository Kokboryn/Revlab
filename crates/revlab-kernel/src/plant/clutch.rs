use revlab_core::SimDuration;
use crate::{Component, Ctx, Port, Trigger};

#[derive(Copy, Clone)]
pub struct ClutchPorts {
    // inputs
    pub omega_eng: Port,    // crank speed, one tick old
    pub omega_in: Port,     // input shaft speed, owned by the gearbox
    pub cmd: Port,          // 0 = fully open, 1 = fully clamped
    pub v_veh: Port,
    pub t_amb: Port,
    // outputs
    pub t_clutch: Port,     // torque on the crank, positive = retarding
    pub slip: Port,         // rad/s, engine minus input
    pub q_clutch: Port,     // W, friction power
    pub t_disc: Port,
    pub glaze: Port,        // 0..1, permanent mu loss
    pub wear_um: Port,      // µm of lining consumed, cumulative
}

/// One dry clutch of a dual clutch pack: a torque device between the crank and one input shaft. It
/// owns no speed state -- the gearbox owns the vehicle, and the shaft's speed follows from whatever
/// gear is selected on it.
///
/// Lock is a stiff spring damper rather than a solved constraint: the bus is f64 slots, so an iterative
/// constraint solve across components is not practical. Stiffness is set so residual twist stays under
/// a degree, which is indistinguishable from locked at this timestep.
pub struct Clutch {
    theta_rel: f64,         // rad, accumulated twist while gripping
    t_disc: f64,            // K, lining and pressure plate as one lumped mass
    pub t_cap_cold: f64,    // Nm, torque capacity at full clamp with fresh cool lining
    pub c_disc: f64,        // J/K
    pub ua_still: f64,      // W/K, bell housing to ambient at rest
    pub ua_speed: f64,      // W/K, per m/s, forced convection with road speed
    pub t_fade_start: f64,  // K, mu begins falling
    pub t_fade_end: f64,    // K, mu at its floor
    pub mu_floor: f64,      // fraction of cold mu when fully faded
    pub k_lock: f64,        // Nm/rad
    pub c_lock: f64,        // Nm·s/rad
    pub thickness0: f64,    // m, lining when new
    pub thickness: f64,     // m, remaining friction material
    pub travel: f64,        // m, actuator travel from touch point to full clamp
    pub glaze: f64,         // 0..1, fraction of cold mu permanently lost
    pub k_wear: f64,        // m per joule at reference temperature
    pub k_glaze: f64,       // per K per second above t_glaze_start
    pub glaze_max: f64,
    pub t_glaze_start: f64, // K, above which surface damage accumulates
    ports: ClutchPorts,
    dt: f64,
}

impl Clutch {
    pub const STEP: SimDuration = SimDuration::from_millis(1);

    pub fn dq200(ports: ClutchPorts, t_amb_init: f64,
                    wear_um_init: f64, glaze_init: f64) -> Self {
        Clutch {
            theta_rel: 0.0,
            t_disc: t_amb_init,
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
            // ~150,000 km of normal use is a millimetre or so of lining, against a few hundred MJ of
            // cumulative slip. Numbers to be tuned once the first long run shows what actually accumulates.
            thickness0: 3.5e-3,
            thickness: 3.5e-3 - wear_um_init * 1e-6,
            travel: 8.0e-3,
            glaze: glaze_init,
            // Fitted to service life rather than to any single run: a dry pack loses roughly 1 mm
            // of lining over ~150,000 km, which is about 3 GJ of cumulative slip energy -- 75,000
            // launches at ~25 kJ plus the shifts between them. At the 100 C reference; the temperature
            // factor does the rest
            k_wear: 3.3e-13,
            k_glaze: 1e-4,
            glaze_max: 0.35,
            t_glaze_start: 273.15 + 300.0,
            ports,
            dt: Self::STEP.as_secs_f64(),
        }
    }

    /// Organic linings lose grip as they heat: the binder starts to break down and outgas. Falls to
    /// mu_floor and stays there -- recoverable here, since permanent loss is wear rather than fade.
    fn mu_frac(&self) -> f64 {
        let x = (self.t_disc - self.t_fade_start) / (self.t_fade_end - self.t_fade_start);
        let thermal = 1.0 - x.clamp(0.0, 1.0) * (1.0 - self.mu_floor);  // recovers on cooling
        thermal * (1.0 - self.glaze)                                                   // does not
    }
}

impl Component for Clutch {
    fn triggers(&self) -> Vec<Trigger> {
        vec![Trigger::Periodic { period: Self::STEP, offset: SimDuration::ZERO }]
    }

    fn step(&mut self, _trig: u16, ctx: &mut Ctx<'_>) {
        let omega_eng = ctx.bus.get(self.ports.omega_eng);
        let cmd_raw = ctx.bus.get(self.ports.cmd).clamp(0.0, 1.0);
        // Lost lining means the plate has further to travel before it loads, so the same actuator
        // position gives less clamp. This is the bite point moving
        let worn = ((self.thickness0 - self.thickness) / self.travel).clamp(0.0, 1.0);
        let cmd = (cmd_raw - worn).clamp(0.0, 1.0);
        let omega_in = ctx.bus.get(self.ports.omega_in);
        let slip = omega_eng - omega_in;

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

        ctx.bus.set(self.ports.t_clutch, t_c);
        ctx.bus.set(self.ports.slip, slip);
        // Friction power. Zero while gripping, kilowatts during a launch

        // Thermal state. Slip power in, forced convection out, scaled by road speed.
        let q_in = if t_stick.abs() > cap { (t_c * slip).abs() } else { 0.0 };
        let v_veh = ctx.bus.get(self.ports.v_veh).abs();
        let ua = self.ua_still + self.ua_speed * v_veh;
        let t_amb = ctx.bus.get(self.ports.t_amb);
        self.t_disc += (q_in - ua * (self.t_disc - t_amb)) / self.c_disc * self.dt;

        // Archard-style: material removed in proportion to friction energy, rising steeply with
        // temperature -- roughly doubling per 50 C for organic linings, so a hill hold at 300 C
        // removes material orders of magnitude faster than ordinary engagement.
        let temp_factor = 2f64.powf((self.t_disc - 373.15) / 50.0);
        self.thickness = (self.thickness - self.k_wear * q_in * temp_factor * self.dt).max(0.0);

        // Glazing is permanent: the binder does not un-decompose when it cools. This is what separates
        // fade, which recovers, from damage, which does not
        if self.t_disc > self.t_glaze_start {
            self.glaze = (self.glaze + self.k_glaze * (self.t_disc - self.t_glaze_start) * self.dt)
                .min(self.glaze_max);
        }

        ctx.bus.set(self.ports.t_disc, self.t_disc);
        ctx.bus.set(self.ports.q_clutch, q_in);
        ctx.bus.set(self.ports.wear_um, (self.thickness0 - self.thickness) * 1e6);
        ctx.bus.set(self.ports.glaze, self.glaze);
    }
}