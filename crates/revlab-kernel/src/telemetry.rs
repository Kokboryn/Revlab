/// Live telemetry over UDP to a dashboard on the same machine. Fire and forget: the sim never waits
/// on the receiver and never reads from it, so a run is identical with or without a dashboard attached.
///
/// Two datagram kinds, little-endian:
///     schema: b"RVLS", u32 count, then count names as u16 length + UTF-8 bytes. Sent every second,
///         so a dashboard started late still learns the layout
///     data:   b"RVLD", u32 seq, f64 sim time [s], then count f64 values in schema order
pub struct UdpStreamer {
    sock: Option<UdpSocket>,
    dest: String,
    names: Vec<String>,
    ports: Vec<Port>,
    period: SimDuration,
    seq: u32,
    since_schema: u32,
    schema_every: u32,
}

impl UdpStreamer {
    pub fn new(port: u16, channels: Vec<(String, Port)>, period: SimDuration) -> Self {
        let sock = UdpSocket::bind("127.0.0.1:0").ok();
        if let Some(s) = &sock { let _ = s.set_nonblocking(true); }
        let (names, ports): (Vec<String>, Vec<Port>) = channels.into_iter().unzip();
        let schema_every = (1.0 / period.as_secs_f64()).round().max(1.0) as u32;
        UdpStreamer {
            sock, dest: format!("127.0.0.1:{port}"), names, ports, period, seq: 0,
            since_schema: schema_every, schema_every,       // schema goes out with the first frame
        }
    }
}

impl Component for UdpStreamer {
    fn triggers(&self) -> Vec<Trigger> {
        vec![Trigger::Periodic { period: self.period, offset: SimDuration::from_micros(4900) }]
    }

    fn step(&mut self, _t: u16, ctx: &mut Ctx<'_>) {
        let Some(sock) = &self.sock else { return };
        if self.since_schema >= self.schema_every {
            self.since_schema = 0;
            let mut b = Vec::with_capacity(8 + 16 * self.names.len());
            b.extend_from_slice(b"RVLS");
            b.extend_from_slice(&(self.names.len() as u32).to_le_bytes());
            for n in &self.names {
                b.extend_from_slice(&(n.len() as u16).to_le_bytes());
                b.extend_from_slice(n.as_bytes());
            }
            let _ = sock.send_to(&b, &self.dest);
        }
        self.since_schema += 1;
        
        let mut b = Vec::with_capacity(16 + 8 * self.ports.len());
        b.extend_from_slice(b"RVLD");
        b.extend_from_slice(&self.seq.to_le_bytes());
        b.extend_from_slice(&ctx.now.as_secs_f64().to_le_bytes());
        for &p in &self.ports {
            b.extend_from_slice(&ctx.bus.get(p).to_le_bytes());
        }
        let _ = sock.send_to(&b, &self.dest);
        self.seq = self.seq.wrapping_add(1);
    }
}

use std::fs::File;
use std::io::{BufWriter, Write};
use std::net::UdpSocket;
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