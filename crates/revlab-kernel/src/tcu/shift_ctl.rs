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

#[derive(Copy, Clone, Debug, PartialEq)]
enum Phase {
    Idle,
    /// Target gear not in yet on the idle shaft: wait for the fork
    Prepare { to: usize },
    /// Driving clutch backs off until it just slips -- the clamp there measures engine torque
    Release { to: usize },
    /// Capacity hands across; engine speed stays put
    Torque { to: usize, x: f64, c_learn: f64, t_est: f64 },
    /// Off going open; on coming pulls the engine down onto its shaft
    Inertia { to: usize, t: f64, slip0: f64, t_est: f64 },
    /// Power on downshift, inertia first: the off going clutch slips so engine's own surplus spins it
    /// up onto the new shaft
    Flare { to: usize, t: f64, slip0: f64, t_est: f64 },
    /// Power on downshift, torque second: engine is at the new shaft's speed, so the oncoming clutch
    /// grips with almost no slip while the offgoing one empties
    Catch { to: usize, x: f64, c_off0: f64, t_est: f64 },
}

pub struct ShiftControl {
    pub t_cap_est: f64,     // Nm, the TCU's belief of full clamp capacity, same as its thermal model
    pub j_eng_est: f64,     // kg·m², engine and flywheel
    pub rate: f64,          // 1/s, actuator slew during a shift
    pub release_rate: f64,  // 1/s, how fast the driving clutch backs off looking for slip
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
    phase: Phase,
    i_n: f64,
    seen_up: u32,
    seen_dn: u32,
    cmd: [f64; 2],
}

impl ShiftControl {
    pub fn dq200() -> Self {
        ShiftControl {
            t_cap_est: 320.0,
            j_eng_est: 0.10,
            rate: 10.0,
            release_rate: 4.0,
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
            phase: Phase::Idle,
            i_n: 0.0,
            seen_up: 0,
            seen_dn: 0,
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

        // Default: the driving clutch follows ClutchControl, the idle shaft preselects the next gear up
        let mut tgt = [0.0; 2];
        tgt[k] = s.clutch_cmd;
        s.sel[k] = g;
        s.sel[1 - k] = if g == 7 { 6 } else { g + 1 };

        self.phase = match self.phase {
            Phase::Idle => {
                // Where each shift would put the engine. A tio that would lug or overrev it is refused,
                // as a real tiptronic does
                let up_land = if g < 7 { s.n_eng * self.ratios[g] / self.ratios[g - 1] } else { 0.0 };
                let dn_land = if g > 1 { s.n_eng * self.ratios[g - 2] / self.ratios[g - 1] } else { f64::MAX };
                // Power on shifts from a closed clutch only, for now
                let ready = s.lever == Lever::Manual && s.clutch_state == ClutchState::Closed && s.pedal > 0.05;
                if ready && tipped && up_land >= self.n_up_min { Phase::Prepare { to: g + 1 }
                } else if ready && tipped_dn && dn_land <= self.n_dn_max { Phase::Prepare { to: g - 1 }
                } else { Phase::Idle }
            }
            Phase::Prepare { to } => {
                s.sel[shaft_of(to)] = to;
                if s.eng[shaft_of(to)] == to { Phase::Release { to } } else { Phase::Prepare { to } }
            }
            Phase::Release { to } => {
                s.sel[shaft_of(to)] = to;
                if s.n_eng - s.n_shaft[k] > self.slip_detect || self.cmd[k] <= 0.0 {
                    let c_learn = self.cmd[k];
                    tgt[k] = c_learn;
                    let t_est = (self.t_cap_est * c_learn * c_learn).max(5.0);
                    if to > g {
                        Phase::Torque { to, x: 0.0, c_learn, t_est }
                    } else {
                        // Downshift: the new shaft is faster than the engine, so it has to come up first
                        self.i_n = 0.0;
                        Phase::Flare { to, t: 0.0, slip0: s.n_shaft[shaft_of(to)] - s.n_eng, t_est }
                    }
                } else {
                    tgt[k] = self.cmd[k] - self.release_rate * DT;
                    Phase::Release { to }
                }
            }
            Phase::Torque { to, x, c_learn, t_est } => {
                let ko = shaft_of(to);
                s.sel[ko] = to;
                let x = (x + DT / self.t_torque).min(1.0);
                // Capacity goes with clamp squared, so the clamps follow square roots: capacity crosses
                // over linearly, with no hole in the middle for the engine to flare into
                tgt[ko] = c_learn * x.sqrt();
                tgt[k] = (c_learn * ((1.0 - x) * (1.0 + self.margin)).sqrt()).min(1.0);
                if x >= 1.0 {
                    self.i_n = 0.0;
                    Phase::Inertia { to, t: 0.0, slip0: s.n_eng - s.n_shaft[ko], t_est }
                } else { Phase::Torque { to, x, c_learn, t_est } }
            }
            Phase::Inertia { to, t, slip0, t_est } => {
                let ko = shaft_of(to);
                s.sel[ko] = to;
                let t = t + DT;
                // The reference runs on past the shaft instead of stopping on it: stopping there leaves the engine balanced
                // just above sync while the shaft slowly catches up
                let n_ref = s.n_shaft[ko] + slip0 * (1.0 - t / self.t_inertia);
                let alpha = slip0 / self.t_inertia * 2.0 * PI / 60.0;
                // Feedforward: engine torque, plus what it takes to decelerate the engine along the ramp
                let t_ff = (t_est + self.j_eng_est * alpha).max(0.0);
                let err = s.n_eng - n_ref;
                self.i_n = (self.i_n + self.ki * err * DT).clamp(-0.3, 0.3);
                tgt[k] = 0.0;
                tgt[ko] = ((t_ff / self.t_cap_est).sqrt() + self.kp * err + self.i_n).clamp(0.0, 1.0);
                if s.n_eng - s.n_shaft[ko] < self.lock_slip || t > 3.0 * self.t_inertia {
                    // Synchronized: the new gear drives, and ClutchControl takes its clutch from the next run
                    s.gear = to;
                    s.clutch_state = ClutchState::Closed;
                    Phase::Idle
                } else { Phase::Inertia { to, t, slip0, t_est } }
            }

            Phase::Flare { to, t, slip0, t_est } => {
                let ko = shaft_of(to);
                s.sel[ko] = to;
                let t = t + DT;
                // Engine speed ramps up onto the new shaft and runs past it, so it crosses on schedule
                let n_ref = s.n_shaft[ko] - slip0 * (1.0 - t / self.t_inertia);
                let alpha = slip0 / self.t_inertia * 2.0 * PI / 60.0;
                // The offgoing clutch carries everything except what it takes to accelerate the engine
                // along the ramp. That shortfall is the sag a kickdown has before it pulls
                let t_ff = (t_est - self.j_eng_est * alpha).max(0.0);
                let err = s.n_eng - n_ref;
                self.i_n = (self.i_n + self.ki * err * DT).clamp(-0.3, 0.3);
                tgt[k] = ((t_ff / self.t_cap_est).sqrt() + self.kp * err + self.i_n).clamp(0.0, 1.0);
                tgt[ko] = 0.0;
                if s.n_eng - s.n_shaft[ko] > -self.lock_slip || t > 3.0 * self.t_inertia {
                    Phase::Catch { to, x: 0.0, c_off0: self.cmd[k], t_est }
                } else { Phase::Flare { to, t, slip0, t_est } }
            }

            Phase::Catch { to, x, c_off0, t_est } => {
                    let ko = shaft_of(to);
                    s.sel[ko] = to;
                    let x = (x + DT / self.t_torque).min(1.0);
                    // Oncoming takes the engine's torque with a margin, so it holds the engine at its shaft
                    // while the offgoing clutch empties
                    tgt[ko] = (x * (1.0 + self.margin) * t_est / self.t_cap_est).sqrt().min(1.0);
                    tgt[k] = c_off0 * (1.0 - x).sqrt();
                    if x >= 1.0 {
                        s.gear = to;
                        s.clutch_state = ClutchState::Closed;
                        Phase::Idle
                    } else { Phase::Catch {to, x, c_off0, t_est} }
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