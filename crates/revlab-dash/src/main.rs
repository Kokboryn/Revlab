//! Revlab dashboard. Listens for the sim's UDP telemetry and shows it live. A separate process, so
//! drawing never slows the simulation down

use std::collections::HashMap;
use std::net::UdpSocket;
use std::time::{Duration, Instant};
use eframe::egui;

const PORT: u16 = 39810;

struct Dash {
    sock: UdpSocket,
    names: Vec<String>,
    index: HashMap<String, usize>,
    values: Vec<f64>,
    t: f64,
    last_rx: Option<Instant>,
}

impl Dash {
    fn new() -> std::io::Result<Self> {
        let sock = UdpSocket::bind(("127.0.0.1", PORT))?;
        sock.set_nonblocking(true)?;
        Ok(Dash { sock, names: Vec::new(), index: HashMap::new(), values: Vec::new(), t: 0.0, last_rx: None })
    }

    fn get(&self, name: &str) -> Option<f64> {
        self.index.get(name).and_then(|&i| self.values.get(i).copied()).filter(|v| v.is_finite())
    }

    /// Drain everything that arrived since the last frame; only the newest values matter for a live view
    fn poll(&mut self) {
        let mut buf = [0u8; 65536];
        while let Ok((n, _)) = self.sock.recv_from(&mut buf) {
            if n < 8 { continue; }
            match &buf[..4] {
                b"RVLS" => self.read_schema(&buf[4..n]),
                b"RVLD" => self.read_data(&buf[4..n]),
                _ => {}
            }
        }
    }

    fn read_schema(&mut self, b: &[u8]) {
        let count = u32::from_le_bytes(b[0..4].try_into().unwrap()) as usize;
        let (mut names, mut i) = (Vec::with_capacity(count), 4);
        for _ in 0..count {
            if i + 2 > b.len() { return; }
            let len = u16::from_le_bytes([b[i], b[i + 1]]) as usize;
            i += 2;
            if i + len > b.len() { return; }
            names.push(String::from_utf8_lossy(&b[i..i + len]).into_owned());
            i += len;
        }
        if names != self.names {
            self.index = names.iter().enumerate().map(|(i, n)| (n.clone(), i)).collect();
            self.values = vec![f64::NAN; names.len()];
            self.names = names;
        }
    }

    fn read_data(&mut self, b: &[u8]) {
        if b.len() < 12 { return; }
        let n = (b.len() - 12) / 8;
        if n != self.names.len() { return; }    // schema not known yet, or a different run started
        self.t = f64::from_le_bytes(b[4..12].try_into().unwrap());
        for k in 0..n {
            let o = 12 + 8 * k;
            self.values[k] = f64::from_le_bytes(b[o..o + 8].try_into().unwrap());
        }
        self.last_rx = Some(Instant::now());
    }
}

fn big (ui: &mut egui::Ui, unit: &str, v: Option<f64>, dp: usize) {
    ui.vertical(|ui| {
        let txt = v.map_or("--".to_string(), |x| format!("{:.*}", dp, x));
        ui.label(egui::RichText::new(txt).size(40.0).monospace());
        ui.label(unit);
    });
    ui.add_space(24.0);
}

impl eframe::App for Dash {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.poll();
        let live = self.last_rx.map_or(false, |t| t.elapsed() < Duration::from_millis(500));

        egui::TopBottomPanel::top("status").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.heading("Revlab");
                ui.separator();
                if live {
                    ui.colored_label(egui::Color32::LIGHT_GREEN, format!("live   t = {:2} s", self.t));
                } else if self.names.is_empty() {
                    ui.label(format!("waiting for a sim streaming to port {PORT}"));
                } else {
                    ui.colored_label(egui::Color32::YELLOW, format!("stream stopped at t = {:.2} s", self.t));
                }
            });
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            ui.horizontal(|ui| {
                big(ui, "rpm", self.get("n_crank"), 0);
                big(ui, "km/h", self.get("v_veh").map(|v| v * 3.6), 0);
                big(ui, "gear", self.get("gear"), 0);
                big(ui, "boost, bar", self.get("p_im").map(|p| (p - 101_325.0) / 1e5), 2);
                big(ui, "Nm", self.get("t_act"), 0);
            });
            ui.separator();
            egui::ScrollArea::vertical().show(ui, |ui| {
                egui::Grid::new("all").striped(true).num_columns(2).show(ui, |ui| {
                    for (i, n) in self.names.iter().enumerate() {
                        ui.monospace(n);
                        ui.monospace(format!("{:.4}", self.values[i]));
                        ui.end_row();
                    }
                });
            });
        });

        ctx.request_repaint_after(Duration::from_millis(16));
    }
}

fn main() -> eframe::Result<()> {
    let dash = Dash::new()
        .unwrap_or_else(|e| panic!("can't listen on port {PORT} ({e}). Is another dashboard already open?"));
    eframe::run_native(
        "Revlab Dashboard",
        eframe::NativeOptions::default(),
        Box::new(|_cc| Ok(Box::new(dash))),
    )
}