use revlab_core::{Map1d, Map2d, SimTime};
use super::torque::Source;
use super::torque_model::TorqueModel;
use super::{EcuState, Task};

/// Closed loop boost control through the VNT vanes
///
/// The setpoint is indexed by the fuel the DRIVER is asking for, not the fuel actually injected.
/// Injected fuel is smoke limited, so it follows boost; a setpoint built from it would sink with the
/// boost it is meant to raise, the same two level trap the open vane turbo fell into. TCU torque cuts
/// don't touch the driver request either, so the turbo stays spooled through a shift
///
/// Vane position = feedforward map + PI on boost error. Closing the vanes raises boost, hence the minus
/// sign.
pub struct BoostControl {
    pub model: TorqueModel,
    pub p_set_map: Map2d,   // Pa absolute over (rpm, demanded load)
    pub vnt_ff_map: Map2d,  // 0 closed .. 1 open, over (rpm, demanded load)
    pub kp: f64,            // 1/kPa
    pub ki: f64,            // 1/(kPa·s)
    pub kd: f64,            // 1/(kPa/s), on measured boost rate
    pub d_tau: f64,         // s, derivative filter
    pub i_band: f64,        // kPa, integrate only inside this error band
    pub integ_max: f64,
    pub vnt_min_map: Map1d,       // closing limit over rpm: at high flow, shut vanes only buy back pressure and overspeed
    pub vnt_max: f64,
    pub n_active: f64,      // rpm, loop off below this (start, idle)
    integ: f64,
    p_prev: f64,
    dpdt: f64,              // kPa/s, filtered
    last: Option<SimTime>
}

impl BoostControl {
    pub fn ea288() -> Self {
        let rpm = vec![800.0, 1250.0, 1750.0, 2500.0, 3250.0, 4000.0, 4600.0];
        let load = vec![0.0, 0.25, 0.5, 0.75, 1.0];     // demanded fuel / q_max
        // row major: one row per load, columns across rpm. kPa absolute
        let p_set_kpa = [
            102.0, 102.0, 103.0, 105.0, 108.0, 110.0, 110.0,
            103.0, 110.0, 125.0, 140.0, 150.0, 150.0, 145.0,
            105.0, 130.0, 165.0, 190.0, 200.0, 195.0, 185.0,
            108.0, 150.0, 210.0, 240.0, 245.0, 235.0, 215.0,
            110.0, 175.0, 250.0, 270.0, 270.0, 245.0, 210.0,
        ];
        // Steady state vane position for each setpoint, from the turbo's own power balance. The PI
        // corrects the rest
        let vnt_ff = vec![
            0.80, 0.80, 0.80, 0.80, 0.80, 0.85, 0.90,
            0.26, 0.12, 0.12, 0.17, 0.27, 0.41, 0.53,
            0.23, 0.12, 0.12, 0.26, 0.43, 0.59, 0.72,
            0.23, 0.12, 0.21, 0.43, 0.64, 0.82, 0.95,
            0.21, 0.12, 0.25, 0.49, 0.71, 0.89, 0.98,
        ];
        BoostControl {
            model: TorqueModel::ea288_cal(),
            p_set_map: Map2d::new(rpm.clone(), load.clone(), p_set_kpa.iter().map(|p| p * 1e3).collect()),
            vnt_ff_map: Map2d::new(rpm, load, vnt_ff),
            kp: 0.006,
            ki: 0.006,
            integ_max: 0.6,
            vnt_min_map: Map1d::new(
                vec![1250.0, 2000.0, 2750.0, 3500.0, 4250.0],
                vec![0.10, 0.10, 0.15, 0.25, 0.40],
            ),
            kd: 0.001,
            d_tau: 0.05,
            i_band: 25.0,
            vnt_max: 1.0,
            n_active: 1000.0,
            integ: 0.0,
            p_prev: 101325.0,
            dpdt: 0.0,
            last: None,
        }
    }
}

impl Task for BoostControl {
    fn name(&self) -> &'static str { "BoostControl" }

    fn run(&mut self, s: &mut EcuState) {
        let dt = match self.last { Some(t0) => (s.now - t0).as_secs_f64(), None => 0.0 };
        self.last = Some(s.now);
        let dt_ok = dt > 0.0 && dt < 0.5;

        // Measured boost rate, filtered, for the D term. Kept current in open loop too, so it has settled on entry
        if dt_ok {
            let raw = (s.p_im_meas - self.p_prev) * 1e-3 / dt;
            self.dpdt += dt / (self.d_tau + dt) * (raw - self.dpdt);
        }
        self.p_prev = s.p_im_meas;

        // What the driver wants, at the cylinder, as fuel: before the smoke limiter and any TCU cut
        let drv = &s.reqs[Source::Driver as usize];
        let t_drv = if drv.active { drv.value } else { 0.0 };
        // Overrun: the driver wants no torque, so use what is actually injected (zero in fuel cut,
        // the idle quantity at idle). Otherwise, friction alone reads as load and the loop builds boost
        // while coasting
        let q_dem = if t_drv > 0.0 { self.model.q_for(s.n_eng, t_drv + s.t_loss) } else { s.q_cmd };
        let load = (q_dem / self.model.q_max).clamp(0.0, 1.0);

        s.p_set = self.p_set_map.get(s.n_eng, load);
        let ff = self.vnt_ff_map.get(s.n_eng, load);
        let vnt_min = self.vnt_min_map.get(s.n_eng);

        if s.n_eng < self.n_active || t_drv <= 0.0 {
            self.integ = 0.0;
            s.vnt_cmd = ff.clamp(vnt_min, self.vnt_max);
            return;
        }

        let e = (s.p_set - s.p_im_meas) * 1e-3;    // kPa, positive = need more boost = close
        // D on the measurement, not the error: a setpoint step at a shift landing must not kick the
        // vanes
        let cmd = (ff - self.kp * e - self.integ + self.kd * self.dpdt).clamp(vnt_min, self.vnt_max);

        // Integrate only near the setpoint, but always let the integral unwind: a band that also froze
        // unwinding locked 35kPa of overboost in for 10 s. Don't wind while a non-smoke limiter (rev
        // limit, TCU cut) holds fuel below demand: the exhaust energy is gone, and no vane position
        // will make it up
        let pushing_closed = cmd <= vnt_min && e > 0.0;
        let pushing_open = cmd >= self.vnt_max && e < 0.0;
        let unwinding = e * self.integ < 0.0;
        let cut_by_other = s.q_cmd < s.q_smoke_limit - 2.0 && q_dem > s.q_cmd + 2.0;
        if dt_ok {
            if unwinding {
                let next = self.integ + self.ki * e * dt;
                self.integ = if next * self.integ < 0.0 { 0.0 } else { next };  // stop at zero; the band takes over
            } else if e.abs() < self.i_band && !cut_by_other && !pushing_closed && !pushing_open {
                self.integ = (self.integ + self.ki * e * dt).clamp(-self.integ_max, self.integ_max);
            }
        }
        s.vnt_cmd = cmd;
    }
}