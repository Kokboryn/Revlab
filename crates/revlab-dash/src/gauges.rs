//! Cockpit drawing. Everything is painted from primitives, so any part of the look can change without
//! touching the data side

use eframe::egui::{Align2, Color32, FontId, Painter, Pos2, Shape, Stroke, Vec2};

#[derive(Clone, Copy)]
pub struct Theme {
    pub bg: Color32,
    pub dim: Color32,           // unlit segments, lamps off
    pub muted: Color32,         // labels
    pub text: Color32,
    pub accent: Color32,        // lit segments, gear
    pub accent_hot: Color32,    // shift zone
    pub warn: Color32,          // lamps on
}

impl Theme {
    pub const COMFORT: Theme = Theme {
        bg: Color32::from_rgb(8, 10, 14),
        dim: Color32::from_rgb(34, 40, 50),
        muted: Color32::from_rgb(110, 122, 140),
        text: Color32::from_rgb(220, 228, 238),
        accent: Color32::from_rgb(63, 169, 245),
        accent_hot: Color32::from_rgb(170, 225, 255),
        warn: Color32::from_rgb(255, 176, 32),
    };
    pub const SPORT: Theme = Theme {
        bg: Color32::from_rgb(12, 6, 6),
        dim: Color32::from_rgb(52, 32, 32),
        muted: Color32::from_rgb(150, 112, 108),
        text: Color32::from_rgb(242, 230, 226),
        accent: Color32::from_rgb(255, 59, 48),
        accent_hot: Color32::from_rgb(255, 205, 120),
        warn: Color32::from_rgb(255, 176, 32),
    };
}

/// One arc gauge: a sweep of segments from start_deg, clockwise, over sweep_deg. Screen angles: 0 = right,
/// 90 = down
pub struct ArcSpec {
    pub min: f64,
    pub max: f64,
    pub start_deg: f32,
    pub sweep_deg: f32,
    pub segments: usize,
    pub label_every: f64,
    pub label_div: f64,         // label shows value / label_div (rpm in thousands)
    pub hot_from: Option<f64>,
    pub red_from: Option<f64>,
}

fn at(c: Pos2, r: f32, deg: f32) -> Pos2 {
    let a = deg.to_radians();
    c + Vec2::new(a.cos(), a.sin()) * r
}

pub fn arc_gauge(p: &Painter, c: Pos2, r: f32, s: &ArcSpec, value: f64, th: &Theme) {
    let r_in = r * 0.80;
    let frac = ((value - s.min) / (s.max - s.min)).clamp(0.0, 1.0);
    let n = s.segments;
    let seg = s.sweep_deg / n as f32;
    let gap = seg * 0.2;
    for i in 0..n {
        let a0 = s.start_deg + seg * i as f32 + gap * 0.5;
        let a1 = a0 + seg - gap;
        let v_mid = s.min + (s.max - s.min) * (i as f64 + 0.5) / n as f64;
        let lit = (i as f64 + 0.5) / n as f64 <= frac;
        let red = s.red_from.map_or(false, |rf| v_mid >= rf);
        let hot = s.hot_from.map_or(false, |hf| v_mid >= hf);
        let fill = match (lit, red, hot) {
            (true, true, _) => Color32::from_rgb(255, 40, 40),
            (true, false, true) => th.accent_hot,
            (true, false, false) => th.accent,
            (false, true, _) => Color32::from_rgb(70, 16, 16),
            (false, false, _) => th.dim,
        };
        // Tapered: each segment reaches a little further out than the last, so the band reads as rising
        let r_out = r_in + (r - r_in) * (0.45 + 0.55 * (i as f32 / n as f32));
        let pts = vec![at(c, r_in, a0), at(c, r_out, a0), at(c, r_out, a1), at(c, r_in, a1)];
        p.add(Shape::convex_polygon(pts, fill, Stroke::NONE));
    }
    
    let mut v = s.min;
    while v <= s.max + 1e-9 {
        let a = s.start_deg + s.sweep_deg * ((v - s.min) / (s.max - s.min)) as f32;
        p.text(at(c, r_in - r * 0.13, a), Align2::CENTER_CENTER, format!("{:.0}", v / s.label_div),
            FontId::proportional(r * 0.09), th.muted);
        v += s.label_every;
    }
    
    // The blade: a bright mark across the band at the value
    let a = s.start_deg + s.sweep_deg * frac as f32;
    p.line_segment([at(c, r_in - r * 0.05, a), at(c, r * 1.06, a)], Stroke::new(r * 0.022, th.text));
}

pub fn lamp(p: &Painter, c: Pos2, r: f32, label: &str, on: bool, th: &Theme) {
    p.circle_filled(c, r, if on { th.warn } else { th.dim });
    p.text(c, Align2::CENTER_CENTER, label, FontId::proportional(r * 0.62),
        if on { Color32::BLACK } else { th.bg });
}