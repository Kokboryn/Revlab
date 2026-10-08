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
                let manual_closed = s.lever == Lever::Manual && s.clutch_state == ClutchState::Closed;
                // Either direction, pedal up or down -- the sequencer picks the order from both
                if manual_closed && tipped && up_land >= self.n_up_min { Phase::Prepare { to: g + 1 }
                } else if manual_closed && tipped_dn && dn_land <= self.n_dn_max { Phase::Prepare { to: g - 1 }
                } else { Phase::Idle }
            }
            Phase::Prepare { to } => {
                s.sel[shaft_of(to)] = to;
                if s.eng[shaft_of(to)] == to { Phase::Release { to } } else { Phase::Prepare { to } }
            }
            Phase::Release { to } => {
                s.sel[shaft_of(to)] = to;
                // Power on the engine runs away above its shaft; coasting it falls below. Either way the
                // clamp at that moment measures the torque going through
                if (s.n_eng - s.n_shaft[k]).abs() > self.slip_detect || self.cmd[k] <= 0.0 {
                    // Detection lags the slip point by a task: the clamp now is already one release step
                    // below where the clutch let go, and torque goes with clamp squared. Measure from the
                    // step before, the last one that was still holding
                    let c_learn = (self.cmd[k] + self.release_rate * DT).min(1.0);
                    tgt[k] = c_learn;
                    let t_est = (self.t_cap_est * c_learn * c_learn).max(5.0);
                    // A slipping clutch passes torque from its faster side to its slower side. Power on
                    // upshifts and power off downshifts can hand torque across first; the other two have
                    // to move engine speed first
                    let torque_first = (to > g) == (s.pedal > 0.05);
                    if torque_first { Phase::Torque { to, x: 0.0, c_learn, t_est }
                    } else {
                        // Inertia first: engine speed has to move before the new clutch can take torque --
                        // up on a power on downshift, down on a power off upshift
                        self.i_n = 0.0;
                        let slip0= s.n_shaft[shaft_of(to)] - s.n_eng;
                        // The engine moves on its own torque, so the ramp can't ask more than that torque
                        // delivers. Eased, the peak rate is twice the average; keep it under 80% of the
                        // engine's own, so the off going clutch never has to let go completely
                        let w0 = slip0.abs() * 2.0 * PI / 60.0;
                        let a_eng = t_est / self.j_eng_est;
                        let tf = self.t_inertia.max(2.0 * w0 / (0.8 * a_eng));
                        Phase::Flare { to, t: 0.0, slip0, t_est, tf }
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
                // +1 after a power on upshift (engine comes down onto the shaft), -1 after a power off
                // downshift (engine comes up). The engine's torque has the same sign in both cases, so
                // one law covers both
                let dir = slip0.signum();
                // The reference runs on past the shaft instead of stopping on it: stopping there leaves
                // the engine balanced just short of sync while the shaft slowly catches up
                let n_ref = s.n_shaft[ko] + slip0 * (1.0 - t / self.t_inertia);
                let alpha = slip0 / self.t_inertia * 2.0 * PI / 60.0;
                // Feedforward: the torque already going through, plus what it takes to move the engine
                // along the ramp
                let t_ff = (t_est + self.j_eng_est * alpha * dir).max(0.0);
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
                // The off going clutch carries the torque going through, less what it takes to move the
                // engine along the ramp -- the engine's own torque does the moving. Power-on that shortfall
                // is the kickdown sag; coasting it is a moment of lighter engine braking
                let t_ff = (t_est - self.j_eng_est * alpha * dir).max(0.0);
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