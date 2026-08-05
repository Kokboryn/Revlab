use std::cmp::Ordering;
use std::collections::BinaryHeap;
use revlab_core::{SimTime, SimDuration, SimRng};

pub mod plant;
pub mod sensors;
pub mod ecu;

#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Debug, Hash)]
pub struct ComponentId(pub u32);

#[derive(Copy, Clone, Debug)]
pub enum Trigger {
    Periodic { period: SimDuration, offset: SimDuration },
    /// Not time schedulable in advance - the owning component reschedules itself via Ctx::schedule_in as speed changes
    SelfPaced,
}

#[derive(Copy, Clone, PartialEq, Eq)]
struct Entry { at: SimTime, comp: ComponentId, trig: u16 }

// BinaryHeap is a max heap: invert every field for min heap ordering
impl Ord for Entry {
    fn cmp(&self, o: &Self) -> Ordering {
        o.at.cmp(&self.at)
            .then_with(|| o.comp.cmp(&self.comp))
            .then_with(|| o.trig.cmp(&self.trig))
    }
}
impl PartialOrd for Entry {
    fn partial_cmp(&self, o: &Self) -> Option<Ordering> { Some(self.cmp(o)) }
}

pub struct Ctx<'a> {
    pub now: SimTime,
    pub rng: &'a mut SimRng,
    pub bus: &'a mut Blackboard,
    me: ComponentId,
    pending: &'a mut Vec<Entry>,
}

impl<'a> Ctx<'a> {
    pub fn schedule_in(&mut self, d: SimDuration, trig: u16) {
        self.pending.push(Entry { at: self.now + d, comp: self.me, trig });
    }
}

pub trait Component {
    fn triggers(&self) -> Vec<Trigger>;
    fn step(&mut self, trig: u16, ctx: &mut Ctx<'_>);
}

/// Flat f64 slots. Ports are handed out at build time, so no lookups and no map iteration order in the hot path
#[derive(Default)]
pub struct Blackboard { slots: Vec<f64> }

#[derive(Copy, Clone, Debug)]
pub struct Port(u32);

impl Blackboard {
    pub fn alloc(&mut self, init: f64) -> Port {
        self.slots.push(init);
        Port(self.slots.len() as u32 - 1)
    }
    #[inline] pub fn get(&self, p: Port) -> f64 { self.slots[p.0 as usize] }
    #[inline] pub fn set(&mut self, p: Port, v: f64) { self.slots[p.0 as usize] = v; }
}

pub struct Kernel {
    components: Vec<Box<dyn Component>>,
    rngs: Vec<SimRng>,
    pub bus: Blackboard,
    queue: BinaryHeap<Entry>,
    pending: Vec<Entry>,
    periods: Vec<Vec<Option<SimDuration>>>, // [comp][trig]
    now: SimTime,
    run_seed: u64,
}

impl Kernel {
    pub fn new(run_seed: u64) -> Self {
        Kernel {
            components: Vec::new(), rngs: Vec::new(), bus: Blackboard::default(), queue: BinaryHeap::new(), pending: Vec::new(), periods: Vec::new(), now: SimTime::ZERO, run_seed,
        }
    }

    pub fn add(&mut self, c: Box<dyn Component>) -> ComponentId {
        let id = ComponentId(self.components.len() as u32);
        let mut row = Vec::new();
        for (i, t) in c.triggers().iter().enumerate() {
            match *t {
                Trigger::Periodic { period, offset } => {
                    self.queue.push(Entry { at: SimTime::ZERO + offset, comp: id, trig: i as u16 });
                    row.push(Some(period));
                }
                Trigger::SelfPaced => row.push(None),
            }
        }
        self.periods.push(row);
        self.rngs.push(SimRng::derive(self.run_seed, id.0));
        self.components.push(c);
        id
    }

    pub fn run_until(&mut self, end: SimTime) {
        while let Some(&e) = self.queue.peek() {
            if e.at > end { break; }
            self.queue.pop();
            self.now = e.at;

            // Split borrow: distinct fields, so this checks out
            let Kernel { components, rngs, bus, pending, .. } = self;
            let mut ctx = Ctx {
                now: e.at,
                rng: &mut rngs[e.comp.0 as usize],
                bus,
                me: e.comp,
                pending,
            };
            components[e.comp.0 as usize].step(e.trig, &mut ctx);

            if let Some(p) = self.periods[e.comp.0 as usize][e.trig as usize] {
                self.queue.push(Entry { at: e.at + p, comp: e.comp, trig: e.trig });
            }
            for p in self.pending.drain(..) { self.queue.push(p); }
        }
    }

    pub fn now(&self) -> SimTime { self.now }
}