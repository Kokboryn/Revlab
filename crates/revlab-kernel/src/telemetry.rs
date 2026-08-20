use std::fs::File;
use std::io::{BufWriter, Write};
use revlab_core::SimDuration;
use crate::{Component, Ctx, Port, Trigger};

pub struct CsvLogger {
    w: BufWriter<File>,
    ports: Vec<(String, Port)>,
    period: SimDuration,
    failed: bool,
}

impl CsvLogger {
    pub fn new(path: &str, ports: Vec<(String, Port)>, period: SimDuration) -> std::io::Result<Self> {
        let mut w = BufWriter::new(File::create(path)?);
        write!(w, "t_s")?;
        for (n, _) in &ports { write!(w, ",{}", n)?; }
        writeln!(w)?;
        Ok(CsvLogger { w, ports, period, failed: false })
    }

    fn write_row(&mut self, ctx: &mut Ctx<'_>) -> std::io::Result<()> {
        write!(self.w, "{:.6}", ctx.now.as_secs_f64())?;
        for (_, p) in &self.ports {
            write!(self.w, ",{:.4}", ctx.bus.get(*p))?;
        }
        writeln!(self.w)
    }
}

impl Component for CsvLogger {
    fn triggers(&self) -> Vec<Trigger> {
        vec![Trigger::Periodic { period: self.period, offset: SimDuration::from_micros(500) }]
    }

    fn step(&mut self, _trig: u16, ctx: &mut Ctx<'_>) {
        if self.failed { return; }
        if let Err(e) = self.write_row(ctx) {
            // Once. A per-row message on a full disk would bury the run output
            eprintln!("telemetry write failed, logging stopped: {e}");
            self.failed = true;
        }
    }
}