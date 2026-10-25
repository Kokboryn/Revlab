//! CAN bus between the control units. Each frame goes out on its own period, carrying signals sampled
//! from the sender at transmit time, quantized to the signal's resolution and held at the receiver until
//! the next frame. That sample and hold, the latency it adds and the resolution are what a real powertrain
//! CAN does to every signal that crosses it, nothing on the receiving side is as fresh or as fine as the
//! sender's own copy.

use revlab_core::{SimDuration, SimTime};
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
    counter: Option<Port>,
    mute: Vec<(SimTime, Option<SimTime>)>,
    count: u8,
}

impl Frame {
    pub fn new(period: SimDuration, offset: SimDuration, signals: Vec<Signal>) -> Self {
        Frame { period, offset, signals, counter: None, mute: Vec::new(), count: 0 }
    }
    /// Rolling message counter, 0..15, written to the receiver each time the frame goes out
    pub fn counter(mut self,dst: Port) -> Self { self.counter = Some(dst); self }
    /// Fault injection: the sender goes quiet from..until (None = for good). Nothing reaches the wire,
    /// so receivers hold their last values, which is exactly why they need the counter
    pub fn mute(mut self, windows: Vec<(SimTime, Option<SimTime>)>) -> Self { self.mute = windows; self }
}

/// Receiver side of one frame: watches its rolling counter. A frame whose counter stops moving for the
/// timeout is lost, and the receiver's copies of its signals are stale, however plausible they look
pub struct RxMonitor {
    last: f64,
    seen: Option<SimTime>,
    timeout_s: f64,
}

impl RxMonitor {
    pub fn new(timeout_s: f64) -> Self { RxMonitor { last: f64::NAN, seen: None, timeout_s } }

    /// True while the frame is lost
    pub fn lost(&mut self, ctr: f64, now: SimTime) -> bool {
        if ctr != self.last { self.last = ctr; self.seen = Some(now); }
        match self.seen { Some(t) => (now - t).as_secs_f64() > self.timeout_s, None => false }
    }
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
       let now = ctx.now;
        let f = &mut self.frames[trig as usize];
        if f.mute.iter().any(|&(a, b)| now >= a && b.map_or(true, |b| now < b)) { return; }
        for s in &f.signals {
            let v = ctx.bus.get(s.src);
            ctx.bus.set(s.dst, (v / s.res).round() * s.res);
        }
        f.count = (f.count + 1) % 16;
        if let Some(p) = f.counter { ctx.bus.set(p, f.count as f64); }
    }
}