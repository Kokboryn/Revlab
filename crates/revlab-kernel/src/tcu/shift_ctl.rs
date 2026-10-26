/// Shift sequencing. Owns both forks and both clutches: which gear each shaft holds, and how clamp is
/// split between K1 and K2.
///
/// A power on upshift runs Release -> Torque -> Inertia. Without an engine torque signal (that arrives
/// with the CAN interface) the TCU measures torque itself: it backs the driving clutch off until it just
/// starts to slip, and the clamp at that moment is what the engine is making.

use std::f64::consts::PI;
use super::{ClutchState, Lever, Task, TcuState};

/// Shaft index that carries gear `g`: odd gears on shaft 1 (0), even gears on shaft 2 (1)
pub fn shaft_of(g: usize) -> usize { if g % 2 == 1 { 0 } else { 1 } }

/// Piecewise linear lookup over pedal
fn interp(t: &[(f64, f64)], x: f64) -> f64 {
    if x <= t[0].0 { return t[0].1; }
    for w in t.windows(2) {
        let ((x0, y0), (x1, y1)) = (w[0], w[1]);
        if x <= x1 { return y0 + (y1 - y0) * (x - x0) / (x1 - x0); }
    }
    t[t.len() - 1].1
}

#[derive(Copy, Clone, Debug, PartialEq)]
enum Phase {
    Idle,
    /// Target gear not in yet on the idle shaft: wait for the fork
    Prepare { to: usize },
    /// Driving clutch backs off until it just slips -- the clamp there measures engine torque
    Release { to: usize },
    /// Capacity hands across; engine speed stays put
    Torque { to: usize, x: f64, c_learn: f64, t_est: f64 },
    /// Off going open; on coming pulls the engine onto its shaft -- down after a power on upshift, up
    /// after a power off downshift
    Inertia { to: usize, t: f64, slip0: f64, t_est: f64 },
    /// Inertia first: the offgoing clutch slips so the engine's own torque moves it onto the new
    /// shaft -- up on a power on downshift, down on a power off upshift
    Flare { to: usize, t: f64, slip0: f64, t_est: f64, tf: f64 },
    /// Torque second: engine is at the new shaft's speed, so the on coming clutch grips with almost no
    /// slip while the off going one empties
    Catch { to: usize, x: f64, c_off0: f64 },
}

pub struct ShiftControl {
    pub t_cap_est: f64,     // Nm, the TCU's belief of full clamp capacity, same as its thermal model
    pub j_eng_est: f64,     // kg·m², engine and flywheel
    pub rate: f64,          // 1/s, actuator slew during a shift
    pub slip_detect: f64,   // rpm, slip that marks the torque point
    pub t_torque: f64,      // s, torque phase
    pub t_inertia: f64,     // s, engine speed ramp
    pub margin: f64,        // extra off going capacity, so it never slips mid-handover
    pub lock_slip: f64,     // rpm, on coming slip that ends the shift
    pub kp: f64,            // command per rpm of engine speed error
    pub ki: f64,            // command per rpm·s
    pub ratios: [f64; 7],   // overall ratios -- the TCU knows its own gearbox
    pub n_up_min: f64,      // rpm, refuse an upshift that would land below this (lugging)
    pub n_dn_max: f64,      // rpm, refuse a downshift that would land above this (over-rev)
    pub map_up: [(f64, f64); 5],    // (pedal, shaft rpm) upshift line
    pub map_dn: [(f64, f64); 5],    // (pedal, shaft rpm) downshift line
    pub land_margin: f64,           // rpm an upshift must land above the new gear's downshift line
    pub kd_pedal: f64,              // pedal above which the box steps down on its own (kickdown)
    pub kd_land: f64,               // rpm a kickdown step may land at, at most
    pub t_confirm: f64,             // s a map request must persist before it is acted on
    pub min_hold: f64,              // s in a gear before the map may shift again
    pub mass_est: f64,              // kg, vehicle with driver, the TCU's calibration
    pub r_wheel_est: f64,           // m, coded rolling radius
    pub eta_est: f64,               // driveline efficiency
    pub crr_est: f64,               // rolling resistance coefficient
    pub cda_est: f64,               // m², drag area
    pub grade_dead: f64,            // %, climb below this is treated as estimate error
    pub a_hold: f64,                // m/s², on a climb the next gear must still manage this
    pub n_hill_max: f64,            // rpm, above this an upshift goes regardless: no camping on the limiter
    phase: Phase,
    i_n: f64,
    seen_up: u32,
    seen_dn: u32,
    pre_dn: bool,                   // preselect the gear below rather than above
    want: usize,                    // gear the map is asking for, 0 = none
    want_t: f64,
    since_shift: f64,
    v_f: f64,                       // km/h, filtered road speed
    a_f: f64,                       // m/s², filtered acceleration
    f_drive_f: f64,                 // N, drive force lagged to match a_f
    primed: bool,
    cmd: [f64; 2],
}

impl ShiftControl {
    pub fn dq200() -> Self {
        ShiftControl {
            t_cap_est: 320.0,
            j_eng_est: 0.10,
            rate: 10.0,
            slip_detect: 20.0,
            t_torque: 0.15,
            t_inertia: 0.30,
            margin: 0.15,
            lock_slip: 30.0,
            kp: 4.5e-4,
            ki: 2.7e-3,
            ratios: [13.633, 7.777, 5.252, 4.011, 3.067, 2.413, 1.957],
            n_up_min: 1100.0,
            n_dn_max: 4400.0,
            // Starting values, not VW's calibration: early upshifts on light pedal, late on full, and a
            // downshift line low enough that the engine never labors
            map_up: [(0.0, 1600.0), (0.25, 1900.0), (0.5, 2500.0), (0.75, 3200.0), (1.0, 4100.0)],
            map_dn: [(0.0, 1050.0), (0.25, 1150.0), (0.5, 1400.0), (0.75, 1700.0), (1.0, 2000.0)],
            land_margin: 250.0,
            kd_pedal: 0.95,
            kd_land: 3800.0,
            t_confirm: 0.2,
            min_hold: 1.0,
            mass_est: 1550.0,
            r_wheel_est: 0.314,
            eta_est: 0.92,
            crr_est: 0.011,
            cda_est: 0.62,
            grade_dead: 2.0,
            a_hold: 0.0,
            n_hill_max: 4300.0,
            phase: Phase::Idle,
            i_n: 0.0,
            seen_up: 0,
            seen_dn: 0,
            pre_dn: false,
            want: 0,
            want_t: 0.0,
            since_shift: 0.0,
            v_f: 0.0,
            a_f: 0.0,
            f_drive_f: 0.0,
            primed: false,
            cmd: [0.0; 2],
        }
    }
}

impl Task for ShiftControl {
    fn name(&self) -> &'static str { "ShiftControl" }

    fn run(&mut self, s: &mut TcuState) {
        const DT: f64 = 0.01;
        // Tips arrive as a running count, so nothing has to clear a pulse
        let tipped = s.tip_up != self.seen_up;
        self.seen_up = s.tip_up;
        let tipped_dn = s.tip_dn != self.seen_dn;
        self.seen_dn = s.tip_dn;

        if !s.lever.drives() { self.phase = Phase::Idle; }
        let g = s.gear;
        if g == 0 {
            s.sel = [0, 0];
            s.cmd = [0.0, 0.0];
            self.cmd = s.cmd;
            s.shift_phase = 0;
            return;
        }
        let k = shaft_of(g);
        let shifting = self.phase != Phase::Idle;

        // Acceleration from the TCU's own road speed, filtered twice: the wheel sensor's noise would
        // swamp a raw derivative
        if !self.primed { self.v_f = s.v_veh; self.primed = true; }
        let v_prev = self.v_f;
        self.v_f += (s.v_veh - self.v_f) * DT / 0.2;
        self.a_f += ((self.v_f - v_prev) / 3.6 / DT - self.a_f) * DT / 0.5;

        // Road resistance: what the engine pushes against, less what goes into accelerating the car
        // and the engine's own inertia, reflected through the gear. Whatever exceeds a flat road's
        // rolling and air drag is climb. Learned only with the gear locked, the brakes off and a
        // second clear of the last shift: that's where the force balance holds
        let i_g = self.ratios[g - 1];
        let r = self.r_wheel_est;
        let m_eff = self.mass_est + self.j_eng_est * i_g * i_g / (r * r);
        // Engine torque is instant, acceleration comes through two filters: lag the force to match
        self.f_drive_f += (s.t_eng * i_g * self.eta_est / r - self.f_drive_f) * DT / 0.7;
        let f_res = self.f_drive_f - m_eff * self.a_f;
        if !shifting && s.clutch_state == ClutchState::Closed && s.brake < 0.01 && self.since_shift > 1.0
            && !s.e1_lost {
            let v = s.v_veh / 3.6;
            let f_flat = self.crr_est * self.mass_est * 9.81 + 0.6 * self.cda_est * v * v;
            let grade = (f_res - f_flat) / (self.mass_est * 9.81) * 100.0;
            s.grade_est += (grade - s.grade_est) * DT / 2.0;
        }
        // On a climb, upshift only into a gear that can still hold speed at this pedal: same engine
        // torque, through the new ratio, against the same resistance. On the flat the check stays
        // out of the way
        let hill_hold = g < 7 && s.grade_est > self.grade_dead && {
            let i_up = self.ratios[g];
            let m_up = self.mass_est + self.j_eng_est * i_up * i_up / (r * r);
            (self.f_drive_f * i_up / i_g - f_res) / m_up < self.a_hold
        };

        // Shift map: lines in shaft rpm of the current gear against pedal
        let p = s.pedal.clamp(0.0, 1.0);
        let n_up = interp(&self.map_up, p);
        let n_dn = interp(&self.map_dn, p);

        // Preselect where the next shift will go. In D, if the map already wants a shift, that decides --
        // a kickdown chain or a tip-out chain knows its next step. Otherwise, gaining speed means up and
        // losing it means down; in between, where the shaft sits in the band, with a dead band so the fork
        // doesn't shuttle
        let auto = s.lever == Lever::Drive;
        let dn_land_pre = if g > 1 { s.n_in * self.ratios[g - 2] / self.ratios[g - 1] } else { f64::MAX };
        if auto && g > 1 && (s.n_in < n_dn || (p >= self.kd_pedal && dn_land_pre <= self.kd_land)) {
            self.pre_dn = true;
        } else if auto && g < 7 && s.n_in > n_up {
            self.pre_dn = false;
        } else if self.a_f > 0.3 { self.pre_dn = false; }
        else if self.a_f < -0.3 { self.pre_dn = true; }
        else {
            let x = (s.n_in - n_dn) / (n_up - n_dn);
            if x < 0.3 { self.pre_dn = true; } else if x > 0.7 { self.pre_dn = false; }
        }

        // Default: the driving clutch follows ClutchControl, the idle shaft holds the preselected gear
        let mut tgt = [0.0; 2];
        // No intervention unless a phase below asks for one
        s.icpt_kind = 0;
        s.icpt_t = 0.0;
        tgt[k] = s.clutch_cmd;
        s.sel[k] = g;
        s.sel[1 - k] = if g == 7 || (self.pre_dn && g > 1) { g - 1 } else { g + 1 };

        self.phase = match self.phase {
            Phase::Idle => {
                self.since_shift += DT;
                // Where each shift would put the engine
                let up_land = if g < 7 { s.n_eng * self.ratios[g] / self.ratios[g - 1] } else { 0.0 };
                let dn_land = if g > 1 { s.n_eng * self.ratios[g - 2] / self.ratios[g - 1] } else { f64::MAX };
                let closed = s.clutch_state == ClutchState::Closed;

                // In D the map asks. An upshift has to land clear of the new gear's downshift line, or the
                // box would hunt between the two
                let ask = if s.lever == Lever::Drive && closed {
                    let kick = p >= self.kd_pedal && dn_land <= self.kd_land;
                    if g < 7 && s.n_in > n_up && up_land >= n_dn + self.land_margin
                        && (!hill_hold || s.n_in > self.n_hill_max) { g + 1 }
                    else if g > 1 && (s.n_in < n_dn || kick) && dn_land <= self.n_dn_max { g - 1 }
                    else { 0 }
                } else { 0 };
                // A request has to persist, so a pedal blip or a bump in the road doesn't shift the box
                if ask != self.want { self.want = ask; self.want_t = 0.0; } else { self.want_t += DT; }
                let auto_go = self.want != 0 && self.want_t >= self.t_confirm && self.since_shift >= self.min_hold;

                let manual = s.lever == Lever::Manual && closed;
                // Without Engine_1 the TCU can't see the engine through a handover: no new shifts
                let to = if s.e1_lost { 0 }
                    else if manual && tipped && up_land >= self.n_up_min { g + 1 }
                    else if manual && tipped_dn && dn_land <= self.n_dn_max { g - 1 }
                    else if auto_go { self.want }
                    else { 0 };


                if to != 0 {
                    self.since_shift = 0.0;
                    self.want = 0;
                    self.want_t = 0.0;
                    Phase::Prepare { to }
                } else { Phase::Idle }
            }

            Phase::Prepare { to } => {
                s.sel[shaft_of(to)] = to;
                if s.eng[shaft_of(to)] == to { Phase::Release { to } } else { Phase::Prepare { to } }
            }
            Phase::Release { to } => {
                s.sel[shaft_of(to)] = to;
                // The engine's torque comes over CAN, so back straight off to the clamp that carries it
                // plus margin, no hunting. Slip still wins if it shows first: the ECU's figure is a model,
                // and this guard is also what's left when the frame stops arriving
                let t_can = s.t_eng.abs();
                let c_can = (t_can * (1.0 + self.margin) / self.t_cap_est).sqrt().min(1.0);
                let slipped = (s.n_eng - s.n_shaft[k]).abs() > self.slip_detect;
                if slipped || self.cmd[k] <= c_can + 1e-9 || self.cmd[k] <= 0.0 {
                    let (c_learn, t_est) = if slipped {
                        // Detection lags the slip point by a task: measure from the step before
                        let c = (self.cmd[k] + self.rate * DT).min(1.0);
                        (c, (self.t_cap_est * c * c).max(5.0))
                    } else {
                        ((t_can / self.t_cap_est).sqrt(), t_can.max(5.0))
                    };
                    tgt[k] = if slipped { c_learn } else { c_can };
                    // A slipping clutch passes torque from its faster side to its slower side. Power on
                    // upshifts and power off downshifts can hand torque across first; the other two
                    // have to move engine speed first
                    let torque_first = (to > g) == (s.pedal > 0.05);
                    if torque_first {
                        Phase::Torque { to, x: 0.0, c_learn, t_est }
                    } else {
                        let slip0 = s.n_shaft[shaft_of(to)] - s.n_eng;
                        let w0 = slip0.abs() * 2.0 * PI / 60.0;
                        let a_eng = t_est / self.j_eng_est;
                        let tf = self.t_inertia.max(2.0 * w0 / (0.8 * a_eng));
                        self.i_n = 0.0;
                        Phase::Flare { to, t: 0.0, slip0, t_est, tf }
                    }
                } else {
                    tgt[k] = (self.cmd[k] - self.rate * DT).max(c_can);
                    Phase::Release { to }
                }
            }
            Phase::Torque { to, x, c_learn, t_est } => {
                let ko = shaft_of(to);
                s.sel[ko] = to;
                let x = (x + DT / self.t_torque).min(1.0);
                // The handover follows what the engine makes now, scaled from the clamp learned at
                // release, so the calibration it learned stays in. Bounded: a bad frame mustn't swing
                // the clamps
                let scale = if s.e1_lost { 1.0 } else { (s.t_eng.abs().max(5.0) / t_est).sqrt().clamp(0.7, 1.4) };
                let c = {c_learn * scale}.min(1.0);
                tgt[ko] = c * x.sqrt();
                tgt[k] = (c * ((1.0 - x) * (1.0 + self.margin)).sqrt()).min(1.0);
                if x >= 1.0 {
                    self.i_n = 0.0;
                    Phase::Inertia { to, t: 0.0, slip0: s.n_eng - s.n_shaft[ko], t_est }
                } else { Phase::Torque { to, x, c_learn, t_est } }
            }
            Phase::Inertia { to, t, slip0, t_est } => {
                let ko = shaft_of(to);
                s.sel[ko] = to;
                let t = t + DT;
                // +1 after a power on upshift (engine comes down onto the shaft), -1 after a power off
                // downshift (engine comes up). The engine's torque has the same sign in both cases, so
                // one law covers both
                let dir = slip0.signum();
                // The reference runs on past the shaft instead of stopping on it: stopping there leaves
                // the engine balanced just short of sync while the shaft slowly catches up
                let n_ref = s.n_shaft[ko] + slip0 * (1.0 - t / self.t_inertia);
                let alpha = slip0 / self.t_inertia * 2.0 * PI / 60.0;
                // Ask the engine to move its own inertia: a cut on an upshift, a raise coasting down. The
                // clutch then only carries the torque already going through, and output torque stays level
                // instead of surging or thumping
                s.icpt_kind = if dir > 0.0 { 1 } else { 2 };
                // Sized from what the driver is asking for right now, not the torque frozen at release:
                // the clutch then carries the driver's torque through the shift
                let t_ref = if s.e1_lost { dir * t_est } else { s.t_drv };
                s.icpt_t = t_ref - self.j_eng_est * alpha;
                // Feedforward from the engine's actual torque, live over CAN: whatever the ECU managed to
                // deliver, the clutch carries that plus whatever moving the engine still needs
                // A shift already runing when Engine_1 drops finishes on the torque it learned at release
                let t_e = if s.e1_lost { dir * t_est } else { s.t_eng };
                let t_ff = (dir * t_e + self.j_eng_est * alpha * dir).max(0.0);
                let err = s.n_eng - n_ref;
                self.i_n = (self.i_n + self.ki * err * dir * DT).clamp(-0.3, 0.3);
                tgt[k] = 0.0;
                tgt[ko] = ((t_ff / self.t_cap_est).sqrt() + dir * self.kp * err + self.i_n).clamp(0.0, 1.0);
                if dir * (s.n_eng - s.n_shaft[ko]) < self.lock_slip || t > 3.0 * self.t_inertia {
                    // Synchronized: the new gear drives, and ClutchControl takes its clutch from the next run
                    s.gear = to;
                    s.clutch_state = ClutchState::Closed;
                    Phase::Idle
                } else { Phase::Inertia { to, t, slip0, t_est } }
            }

            Phase::Flare { to, t, slip0, t_est, tf } => {
                let ko = shaft_of(to);
                s.sel[ko] = to;
                let t = t + DT;
                // +1 on a power on downshift (engine comes up), -1 on a power off upshift (engine comes
                // down). The engine's torque has the same sign as the slip in both, so one law covers both
                let dir = slip0.signum();
                // Ease out: slip closes quickly at first and arrives at the shaft with zero rate. Here the
                // off going clutch steers and the on coming one starts from nothing, so the engine has to
                // be standing at the shaft when they swap -- crossing at full rate overshoots it
                let tau = (t / tf).min(1.0);
                let n_ref = s.n_shaft[ko] - slip0 * (1.0 - tau) * (1.0 - tau);
                let alpha = 2.0 * slip0 * (1.0 - tau) / tf * 2.0 * PI / 60.0;
                // Power on, ask for the extra torque that spins the engine up, so the off going clutch keeps
                // carrying all of it, no sag, if the engine has torque to spare. Coasting there's nothing
                // to ask for: the engine is already on fuel cut, and only friction brings it down
                if dir > 0.0 {
                    s.icpt_kind = 2;
                    let t_ref = if s.e1_lost { t_est } else { s.t_drv };
                    s.icpt_t = t_ref + self.j_eng_est * alpha;
                }
                // Feedforward from live CAN torque: what the engine makes, less what moving it takes
                let t_e = if s.e1_lost { dir * t_est } else { s.t_eng };
                let t_ff = (dir * t_e - self.j_eng_est * alpha * dir).max(0.0);
                let err = s.n_eng - n_ref;
                self.i_n = (self.i_n + self.ki * err * dir * DT).clamp(-0.3, 0.3);
                tgt[k] = ((t_ff / self.t_cap_est).sqrt() + dir * self.kp * err + self.i_n).clamp(0.0, 1.0);
                tgt[ko] = 0.0;
                // Hand over only once the ramp has landed: the engine has to be standing at the shaft,
                // held by the off going clutch, not passing through it
                if (tau >= 1.0 && dir * (s.n_shaft[ko] - s.n_eng) < self.lock_slip) || t > 3.0 * tf {
                    Phase::Catch { to, x: 0.0, c_off0: self.cmd[k] }
                } else { Phase::Flare { to, t, slip0, t_est, tf } }
            }

            Phase::Catch { to, x, c_off0 } => {
                    let ko = shaft_of(to);
                    s.sel[ko] = to;
                    let x = (x + DT / self.t_torque).min(1.0);
                    // Sized from the clamp that was actually holding the engine at the end of the flare.
                    // The clutches are the same design, so the same clamp carries the same torque --
                    // whatever the estimate before the shift said
                    tgt[ko] = (c_off0 * (x * (1.0 + self.margin)).sqrt()).min(1.0);
                    tgt[k] = c_off0 * (1.0 - x).sqrt();
                    if x >= 1.0 {
                        s.gear = to;
                        s.clutch_state = ClutchState::Closed;
                        Phase::Idle
                    } else { Phase::Catch {to, x, c_off0 } }
                }
        };

        for j in 0..2 {
            self.cmd[j] = if shifting {
                let step = self.rate * DT;
                (self.cmd[j] + (tgt[j] - self.cmd[j]).clamp(-step, step)).clamp(0.0, 1.0)
            } else { tgt[j] };
        }
        // Protection is not overridable
        if s.overheat { self.cmd = [0.0; 2]; self.phase = Phase::Idle; }
        s.cmd = self.cmd;
        s.shift_phase = match self.phase {
            Phase::Idle => 0, Phase::Prepare { .. } => 1, Phase::Release { .. } => 2,
            Phase::Torque { .. } => 3, Phase::Inertia { .. } => 4,
            Phase::Flare { .. } => 4, Phase::Catch { .. } => 3,
        };
    }
}