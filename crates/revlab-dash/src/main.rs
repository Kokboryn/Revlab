//! Revlab dashboard. Listens for the sim's UDP telemetry and shows it live. A separate process, so
//! drawing never slows the simulation down

use std::collections::HashMap;
use std::net::UdpSocket;
use std::time::{Duration, Instant};
use eframe::egui;
use eframe::egui::{pos2, vec2, Align2, FontId};

mod gauges;
const PORT: u16 = 39810;

const TACH: gauges::ArcSpec = gauges::ArcSpec {
    min: 0.0, max: 5000.0, start_deg: 135.0, sweep_deg: 270.0, segments: 50, label_every: 1000.0,
    label_div: 1000.0, hot_from: Some(4000.0), red_from: Some(4600.0),
};

const SPEEDO: gauges::ArcSpec = gauges::ArcSpec {
    min: 0.0, max: 260.0, start_deg: 135.0, sweep_deg: 270.0, segments: 52, label_every: 20.0,
    label_div: 1.0, hot_from: None, red_from: None,
};

struct Dash {
    sock: UdpSocket,
    names: Vec<String>,
    index: HashMap<String, usize>,
    values: Vec<f64>,
    t: f64,
    last_rx: Option<Instant>,
    show_channels: bool,
    rpm_disp: f64,      // smoothed for the needles
    kmh_disp: f64,
}

impl Dash {
    fn new() -> std::io::Result<Self> {
        let sock = UdpSocket::bind(("127.0.0.1", PORT))?;
        sock.set_nonblocking(true)?;
        Ok(Dash { sock, names: Vec::new(), index: HashMap::new(), values: Vec::new(), t: 0.0, last_rx: None,
            show_channels: false, rpm_disp: 0.0, kmh_disp: 0.0, })
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

impl eframe::App for Dash {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.poll();
        let live = self.last_rx.map_or(false, |t| t.elapsed() < Duration::from_millis(500));

        // Needles chase the signal instead of jumping to it
        let k = (ctx.input(|i| i.stable_dt) as f64 * 12.0).min(1.0);
        if let Some(n) = self.get("n_crank") { self.rpm_disp += (n - self.rpm_disp) * k; }
        if let Some(v) = self.get("v_veh") { self.kmh_disp += (v * 3.6 - self.kmh_disp) * k; }

        let lever = self.get("lever").unwrap_or(2.0).round() as i32;
        let sport = lever == 4;
        let th = if sport { gauges::Theme::SPORT } else { gauges::Theme::COMFORT };

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
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.toggle_value(&mut self.show_channels, "channels");
                });
            });
        });

        if self.show_channels {
            egui::SidePanel::right("channels").resizable(true).default_width(260.0).show(ctx, |ui| {
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
        }

        egui::CentralPanel::default().frame(egui::Frame::default().fill(th.bg)).show(ctx, |ui| {
            let rect = ui.max_rect();
            let p = ui.painter_at(rect);
            let r = (rect.width() / 6.4).min(rect.height() / 2.3);
            let (cx, cy) = (rect.center().x, rect.center().y - r * 0.05);
            let c_tach = pos2(cx - r * 2.05, cy);
            let c_spd = pos2(cx + r * 2.05, cy);

            gauges::arc_gauge(&p, c_tach, r, &TACH, self.rpm_disp, &th);
            gauges::arc_gauge(&p, c_spd, r, &SPEEDO, self.kmh_disp, &th);

            // Gauge hearts: digital speed, and rpm under its scale
            p.text(c_spd, Align2::CENTER_CENTER, format!("{:.0}", self.kmh_disp.max(0.0)),
                FontId::monospace(r * 0.36), th.text);
            p.text(c_spd + vec2(0.0, r * 0.30), Align2::CENTER_CENTER, "km/h", FontId::proportional(r * 0.1), th.muted);
            p.text(c_tach, Align2::CENTER_CENTER, format!("{:.0}", self.rpm_disp.max(0.0)),
                FontId::monospace(r * 0.22), th.text);
            p.text(c_tach + vec2(0.0, r * 0.30), Align2::CENTER_CENTER, "rpm x1000", FontId::proportional(r * 0.1), th.muted);

            // Center: gear and mode
            let g = self.get("gear").unwrap_or(0.0).round() as i32;
            let gear_txt = match lever {
                0 => "P".to_string(), 1 => "R".to_string(), 2 => "N".to_string(), 3 => format!("D{g}"),
                _ => format!("M{g}"),
            };
            let cg = pos2(cx, cy - r * 0.45);
            p.text(cg, Align2::CENTER_CENTER, gear_txt, FontId::monospace(r * 0.5), th.accent);
            p.text(cg + vec2(0.0, r * 0.36), Align2::CENTER_CENTER, if sport { "SPORT" } else { "COMFORT" },
                FontId::proportional(r * 0.1), th.muted);

            // Info stack
            let clutch = match (self.get("t_disc_est1"), self.get("t_disc_est2")) {
                (Some(a), Some(b)) => Some(format!("{:.0} / {:.0} °C", a - 273.15, b - 273.15)),
                _ => None,
            };
            let rows = [
                ("Boost", self.get("p_im").map(|x| format!("{:+.2} bar", (x - 101_325.0) / 1e5))),
                ("Torque", self.get("t_act").map(|x| format!("{:.0} Nm", x))),
                ("Coolant", self.get("t_ect").map(|x| format!("{:.0} °C", x - 273.15))),
                ("Clutch", clutch),
            ];
            for (i, (label, val)) in rows.iter().enumerate() {
                let y = cy + r * 0.15 + i as f32 * r * 0.17;
                p.text(pos2(cx - r * 0.05, y), Align2::RIGHT_CENTER, *label, FontId::proportional(r * 0.1), th.muted);
                p.text(pos2(cx + r * 0.05, y), Align2::LEFT_CENTER, val.clone().unwrap_or_else(|| "--".into()),
                    FontId::monospace(r * 0.1), th.text);
            }

            // Lamps: enigne for P0016 or U0101, gearbox for lost frames or an overheated clutch
            let engine = self.get("dtc").map_or(false, |d| d >= 2.0) || self.get("u0101").map_or(false, |d| d >= 2.0);
            let gearbox = self.get("can_dtc").map_or(false, |d| d > 0.0) || self.get("overheat").map_or(false, |d| d > 0.5);
            let ly = cy + r * 0.95;
            gauges::lamp(&p, pos2(cx - r * 0.3, ly), r * 0.12, "ENG", engine, &th);
            gauges::lamp(&p, pos2(cx + r * 0.3, ly), r * 0.12, "GBX", gearbox, &th);
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