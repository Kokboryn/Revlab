/// Shift sequencing. Owns both forks and both clutches: which gear each shaft holds, and how clamp is
/// split between K1 and K2. Idle for now -- the driving clutch gets ClutchControl's  command and the idle
/// shaft preselects the next gear up. The handover state machine goes here next.

use super::{Task, TcuState};

/// Shaft index that carries gear `g`: odd gears on shaft 1 (0), even gears on shaft 2 (1)
pub fn shaft_of(g: usize) -> usize { if g % 2 == 1 { 0 } else { 1 } }

pub struct ShiftControl;

impl ShiftControl {
    pub fn dq200() -> Self { ShiftControl }
}

impl Task for ShiftControl {
    fn name(&self) -> &'static str { "ShiftControl" }

    fn run(&mut self, s: &mut TcuState) {
        let g = s.gear;
        if g == 0 {
            s.sel = [0, 0];
            s.cmd = [0.0, 0.0];
            return;
        }
        let k = shaft_of(g);
        // Preselect the next gear up on the idle shaft; 7th has none, so it holds 6th for the way down
        s.sel[k] = g;
        s.sel[1 - k] = if g == 7 { 6 } else { g + 1 };
        s.cmd[k] = s.clutch_cmd;
        s.cmd[1 - k] = 0.0;
    }
}