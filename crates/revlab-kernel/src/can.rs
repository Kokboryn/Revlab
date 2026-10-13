//! CAN bus between the control units. Each frame goes out on its own period, carrying signals sampled
//! from the sender at transmit time, quantized to the signal's resolution and held at the receiver until
//! the next frame. That sample and hold, the latency it adds and the resolution are what a real powertrain
//! CAN does to every signal that crosses it, nothing on the receiving side is as fresh or as fine as the
//! sender's own copy.

use revlab_core::SimDuration;
use crate::{Component, Ctx, Port, Trigger};

/// One signal in a frame: read from the sender's port, written to the receiver's
#[derive(Copy, Clone)]
pub struct Signal {
    pub src: Port,
    pub dst: Port,
    pub res: f64,       // quantization step, in the signal's own unit
}

pub struct Frame {
    pub period: SimDuration,
    pub offset: SimDuration,
    pub signals: Vec<Signal>,
}

pub struct CanBus {
    frames: Vec<Frame>,
}

impl CanBus {
    pub fn new() -> Self { CanBus { frames: Vec::new() } }
    
    /// Registration order is trigger order: frane n fires as trigger n
    pub fn frame(mut self, f: Frame) -> Self {
        self.frames.push(f);
        self
    }
}

impl Component for CanBus {
    fn triggers(&self) -> Vec<Trigger> {
        self.frames.iter()
            .map(|f| Trigger::Periodic { period: f.period, offset: f.offset })
            .collect()
    }

    fn step(&mut self, trig: u16, ctx: &mut Ctx<'_>) {
        for s in &self.frames[trig as usize].signals {
            let v = ctx.bus.get(s.src);
            ctx.bus.set(s.dst, (v / s.res).round() * s.res);
        }
    }
}