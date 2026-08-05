use std::fs::File;
use std::io::{BufWriter, Write};
use revlab_core::SimDuration;
use crate::{Component, Ctx, Port, Trigger};

pub struct CsvLogger {
    w: BufWriter<File>,
    ports: Vec<(String, Port)>,
    period: SimDuration,
}

impl CsvLogger {
    pub fn new(path: &str, ports: Vec<(String, Port)>, period: SimDuration) -> std::io::Result<Self> {
        let mut w = BufWriter::new(File::create(path)?);
        write!(w, "t_s")?;
        for (n, _) in &ports { write!(w, ",{}", n)?; }
        writeln!(w)?;
        Ok(CsvLogger { w, ports, period })
    }
}

impl Component for CsvLogger {
    fn triggers(&self) -> Vec<Trigger> {
        vec![Trigger::Periodic { period: self.period, offset: SimDuration::from_micros(500) }]
    }

    fn step(&mut self, _t: u16, ctx: &mut Ctx<'_>) {
        let _ = write!(self.w, "{:.6}", ctx.now.as_secs_f64());
        for (_, p) in &self.ports {
            let _ = write!(self.w, ",{:.4}", ctx.bus.get(*p));
        }
        let _ = writeln!(self.w);
    }
}