use std::io;
use crossterm::event::{self, Event, KeyCode, KeyEventKind};
use crossterm::terminal;
use revlab_core::SimDuration;
use revlab_kernel::{Component, Ctx, Port, Trigger};

/// Live keyboard input
///
/// Polls with a zero timeout so it never blocks the sim. Keypress TIMING depends on the wall clock,
/// so a live session is not reproducible on its own - that is what the input recorder exists for.
/// Nothing else in the sim reads the clock.
pub struct Keyboard {
    pedal: f64,
    load: f64,
    pub quit: std::sync::Arc<std::sync::atomic::AtomicBool>,
    pedal_out: Port,
    load_out: Port,
}

impl Keyboard {
    pub fn new(pedal_out: Port, load_out: Port, quit: std::sync::Arc<std::sync::atomic::AtomicBool>) -> Self {
        Keyboard { pedal: 0.0, load: 0.0, pedal_out, load_out, quit }
    }

    /// Raw mode so keys arrive without Enter. Must be undone on exit or the user's shell is left unusable
    pub fn enter_raw() -> io::Result<()> { terminal::enable_raw_mode() }
    pub fn leave_raw() -> io::Result<()> { terminal::disable_raw_mode() }
}

impl Component for Keyboard {
    fn triggers(&self) -> Vec<Trigger> {
        // 20 ms: responsive to a human, cheap enough to poll
        vec![Trigger::Periodic { period: SimDuration::from_millis(20), offset: SimDuration::from_micros(950) }]
    }

    fn step(&mut self, trig: u16, ctx: &mut Ctx<'_>) {
        while event::poll(std::time::Duration::ZERO).unwrap_or(false) {
            if let Ok(Event::Key(k)) = event::read() {
                if k.kind != KeyEventKind::Press { continue; }
                match k.code {
                    KeyCode::Up                         => self.pedal = (self.pedal + 0.10).min(1.0),
                    KeyCode::Down                       => self.pedal = (self.pedal - 0.10).max(0.0),
                    KeyCode::Right                      => self.load = (self.load + 10.0).min(300.0),
                    KeyCode::Left                       => self.load = (self.load - 10.0).max(0.0),
                    KeyCode::Char(' ')                  => { self.pedal = 0.0; }
                    KeyCode::Char('q') | KeyCode::Esc   => { self.quit.store(true, std::sync::atomic::Ordering::Relaxed);
                    }
                    _ => {}
                }
            }
        }
        ctx.bus.set(self.pedal_out, self.pedal);
        ctx.bus.set(self.load_out, self.load);
    }
}