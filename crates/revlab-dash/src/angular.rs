//! Angular cluster: an eight-sided frame whose flanks are the scales. Speed climbs the left flank,
//! rpm on the right; fuel and coolant sit outside, a mode-tinted screen sits in the middle

use crate::gauges::{lamp, Theme};
use eframe::egui::{pos2, Align2, Color32, FontId, Painter, Pos2, Rect, Shape, Stroke, Vec2};

/// Everything the cluster shows, already in display units. None draws as "--"
pub struct Cockpit {
    pub kmh: f64,
    pub rpm: f64,
    pub gear: String,
    pub mode: &'static str,
    pub fuel_frac: Option<f64>,
    pub coolant_c: Option<f64>,
    pub range_km: Option<f64>,
    pub clock: Option<String>,
    pub ambient_c: Option<f64>,
    pub page_title: &'static str,
    pub rows: Vec<(&'static str, String)>,
    pub eng_lamp: bool,
    pub gbx_lamp: bool,
}

struct Scale {
    min: f64,
    max: f64,
    bars: usize,
    label_every: f64,
    label_div: f64,
    hot_from: Option<f64>,
    red_from: Option<f64>,
}

const SPEED: Scale = Scale { min: 0.0, max: 260.0, bars: 52, label_every: 20.0, label_div: 1.0,
    hot_from: None, red_from: None, };
const RPM: Scale = Scale { min: 0.0, max: 5000.0, bars: 50, label_every: 1000.0, label_div: 1000.0,
    hot_from: Some(4000.0), red_from: Some(4600.0), };

/// A polyline measured by arc length, so a scale can run around corners
struct Path { pts: Vec<Pos2>, cum: Vec<f32>, nrm: Vec<Vec2> }

impl Path {
    fn new(pts: Vec<Pos2>, toward: Pos2) -> Self {
        let mut cum = vec![0.0];
        let mut nrm = Vec::new();
        for w in pts.windows(2) {
            let d = w[1] - w[0];
            cum.push(cum.last().unwrap() + d.length());
            let n = d.normalized().rot90();
            let mid = w[0] + d * 0.5;
            nrm.push(if n.dot(toward - mid) > 0.0 { n } else { -n });
        }
        Self { pts, cum, nrm }
    }

    fn len(&self) -> f32 { *self.cum.last().unwrap() }

    /// Segment that holds arc length s
    fn seg(&self, s: f32) -> usize {
        self.cum.windows(2).position(|w| s <= w[1]).unwrap_or(self.pts.len() - 2)
    }

    /// Point at arc length s, measured along segment i
    fn on(&self, i: usize, s: f32) -> Pos2 {
        self.pts[i] + (self.pts[i + 1] - self.pts[i]).normalized() * (s - self.cum[i])
    }

    /// Corner bisector at vertex j: the line where two segments bars meet
    fn miter(&self, j: usize) -> Vec2 { (self.nrm[j - 1] + self.nrm[j]).normalized() }
}

/// Keep the part of a convex polygon on the same side of the line (through o, along dir) as `keep`
fn clip(poly: Vec<Pos2>, o: Pos2, dir: Vec2, keep: Pos2) -> Vec<Pos2> {
    let side = |p: Pos2| { let v = p - o; dir.x * v.y - dir.y * v.x };
    let sk = if side(keep) >= 0.0 { 1.0 } else { -1.0 };
    let mut out = Vec::with_capacity(poly.len() + 1);
    for i in 0..poly.len() {
        let (a, b) = (poly[i], poly[(i + 1) % poly.len()]);
        let (sa, sb) = (side(a) * sk, side(b) * sk);
        if sa >= 0.0 { out.push(a); }
        if (sa >= 0.0) != (sb >= 0.0) { out.push(a + (b - a) * (sa / (sa - sb))); }
    }
    out
}

fn area(poly: &[Pos2]) -> f32 {
    let n = poly.len();
    (0..n).map(|i| { let (a, b) = (poly[i], poly[(i + 1) % n]); a.x * b.y - b.x * a.y })
        .sum::<f32>().abs() * 0.5
}

fn flank(p: &Painter, path: &Path, depth: f32, s: &Scale, value: f64, th: &Theme) {
    let frac = ((value - s.min) / (s.max - s.min)).clamp(0.0, 1.0) as f32;
    let len = path.len();
    let n = s.bars as f32;
    for i in 0..s.bars {
        let fm = (i as f32 + 0.5) / n;
        // A bar never bends round a corner: it stays on the segment that holds its middle
        let k = path.seg(fm * len);
        let s0 = ((i as f32 + 0.12) / n * len).max(path.cum[k]);
        let s1 = ((i as f32 + 0.88) / n * len).min(path.cum[k + 1]);
        if s1 - s0 < 0.5 { continue; }
        let (a, b, nm) = (path.on(k, s0), path.on(k, s1), path.nrm[k]);
        let depth_i = depth * (0.45 + 0.55 * fm);
        let mut poly = vec![a, b, b + nm * depth_i, a + nm * depth_i];
        // Mitered at the corners like a picture frame, so neighboring edges never overlap
        if k > 0 { poly = clip(poly, path.pts[k], path.miter(k), path.pts[k + 1]); }
        if k + 2 < path.pts.len() { poly = clip(poly, path.pts[k + 1], path.miter(k + 1), path.pts[k]); }
        // Slivers left at a corner read as clutter: drop anything under half a bar
        if poly.len() < 3 || area(&poly) < 0.5 * (s1 - s0) * depth_i { continue; }

        let v_mid = s.min + (s.max - s.min) * fm as f64;
        let lit = fm <= frac;
        let red = s.red_from.map_or(false, |r| v_mid >= r);
        let hot = s.hot_from.map_or(false, |h| v_mid >= h);
        let fill = match (lit, red, hot) {
            (true, true, _) => Color32::from_rgb(255, 40, 40),
            (true, false, true) => th.accent_hot,
            (true, false, false) => th.accent,
            (false, true, _) => Color32::from_rgb(70, 16, 16),
            (false, false, _) => th.dim,
        };
        p.add(Shape::convex_polygon(poly, fill, Stroke::NONE));
    }

    // Labels sit outside the frame: at a convex corner the outward normals spread apart instead of
    // bunching, and nothing inside the pods can collide with them

    let font = depth * 0.55;
    let mut v = s.min;
    while v <= s.max + 1e-9 {
        let sv = len * ((v - s.min) / (s.max - s.min)) as f32;
        let k = path.seg(sv);
        let out = -path.nrm[k];
        let txt = format!("{:.0}", v / s.label_div);
        // Push by the text's own half-size along the normal, so wide labels clear a vertical edge too
        let reach = out.x.abs() * 0.3 * font * txt.len() as f32 + out.y.abs() * 0.5 * font;
        p.text(path.on(k, sv) + out * (depth * 0.3 + reach), Align2::CENTER_CENTER, txt,
               FontId::proportional(font), th.muted);
        v += s.label_every;
    }

    let sv = frac * len;
    let k = path.seg(sv);
    let (pt, nm) = (path.on(k, sv), path.nrm[k]);
    p.line_segment([pt - nm * depth * 0.2, pt + nm * depth * 1.2], Stroke::new(depth *  0.12, th.text));
}

/// Short vertical bar gauge for fuel and coolant
fn column(p: &Painter, x: f32, y_bot: f32, y_top: f32, w: f32, frac: Option<f64>, warn: bool, lo: &str,
            hi: &str, caption: &str, font: f32, th: &Theme) {
    let n = 8;
    let step = (y_bot - y_top) / n as f32;
    for i in 0..n {
        let y1 = y_bot - step * i as f32 - step * 0.12;
        let y0 = y1 - step * 0.76;
        let lit = frac.map_or(false, |f| (i as f64 + 0.5) / n as f64 <= f);
        let fill = if !lit { th.dim } else if warn { th.warn } else { th.accent };
        let pts = vec![pos2(x - w / 2.0, y0), pos2(x + w / 2.0, y0), pos2(x + w / 2.0, y1), pos2(x - w / 2.0, y1)];
        p.add(Shape::convex_polygon(pts, fill, Stroke::NONE));
    }
    p.text(pos2(x, y_top - font), Align2::CENTER_CENTER, hi, FontId::proportional(font), th.muted);
    p.text(pos2(x, y_bot + font), Align2::CENTER_CENTER, lo, FontId::proportional(font), th.muted);
    p.text(pos2(x, y_bot + font * 2.4), Align2::CENTER_CENTER, caption, FontId::proportional(font * 0.8), th.muted);
}

pub fn paint(p: &Painter, rect: Rect, c: &Cockpit, th: &Theme) {
    // Laid out on a 1000 x 333 grid around the center, scaled to the window
    let k = rect.width().min(rect.height() * 3.0) / 1000.0;
    let o = rect.center();
    let at = |x: f32, y: f32| pos2(o.x + x * k, o.y + y * k);
    let depth = 26.0 * k;

    // Frame: half-width 400, half-height 140, inner corners at x = 136, outer edges 84 tall
    let (hw, hh, inner, waist) = (400.0, 140.0, 136.0, 60.0);
    let left = Path::new(vec![at(-inner, hh), at(-hw, waist), at(-hw, -waist), at (-inner, -hh)], at(-250.0, 0.0));
    let right = Path::new(vec![at(inner, hh), at(hw, waist), at(hw, -waist), at(inner, -hh)], at(250.0, 0.0));
    let mut outline = left.pts.clone();
    outline.extend(right.pts.iter().rev());
    p.add(Shape::closed_line(outline, Stroke::new(1.5 * k, th.dim)));

    flank(p, &left, depth, &SPEED, c.kmh, th);
    flank(p, &right, depth, &RPM, c.rpm, th);

    // Pods: digital speed left, gear and mode right
    p.text(at(-262.0, -8.0), Align2::CENTER_CENTER, format!("{:.0}", c.kmh.max(0.0)),
        FontId::monospace(64.0 * k), th.text);
    p.text(at(-262.0, 34.0), Align2::CENTER_CENTER, "km/h", FontId::proportional(13.0 * k), th.muted);
    p.text(at(262.0, -10.0), Align2::CENTER_CENTER, &c.gear, FontId::monospace(64.0 * k), th.accent);
    p.text(at(262.0, 34.0), Align2::CENTER_CENTER, c.mode, FontId::proportional(13.0 * k), th.muted);
    p.text(at(262.0, 54.0), Align2::CENTER_CENTER, format!("{:.0} rpm", c.rpm.max(0.0)),
        FontId::monospace(12.0 * k), th.muted);

    // Center screen, tinted by the drive mode
    let (sx, sy, ch) = (118.0, 120.0, 14.0);
    let screen = vec![at(-sx + ch, -sy), at(sx - ch, -sy), at(sx, -sy + ch), at(sx, sy - ch),
                                 at(sx - ch, sy), at(-sx + ch, sy), at(-sx, sy - ch), at(-sx, -sy + ch)];
    p.add(Shape::convex_polygon(screen, th.accent.gamma_multiply(0.10),
                                 Stroke::new(1.5 * k, th.accent.gamma_multiply(0.6))));
    p.text(at(0.0, -98.0), Align2::CENTER_CENTER, c.page_title, FontId::proportional(14.0 * k), th.accent);
    for (i, (label, val)) in c.rows.iter().enumerate() {
        let y = -66.0 + i as f32 * 26.0;
        p.text(at(-6.0, y), Align2::RIGHT_CENTER, *label, FontId::proportional(13.0 *k), th.muted);
        p.text(at(6.0, y), Align2::LEFT_CENTER, val, FontId::monospace(14.0 * k), th.text);
    }
    lamp(p, at(-40.0, 92.0), 13.0 * k, "ENG", c.eng_lamp, th);
    lamp(p, at(40.0, 92.0), 13.0 * k, "GBX", c.gbx_lamp, th);

    // Outside the frame: fuel left, coolant right
    let font = 12.0 * k;
    column(p, at(-470.0, 0.0).x, at(0.0, 60.0).y, at(0.0, -60.0).y, 10.0 * k, c.fuel_frac,
            c.fuel_frac.map_or(false, |f| f < 0.12), "E", "F", "FUEL", font, th);
    let cool = c.coolant_c.map(|t| ((t - 50.0) / 80.0).clamp(0.0, 1.0));
    column(p, at(470.0, 0.0).x, at(0.0, 60.0).y, at(0.0, -60.0).y, 10.0 * k, cool,
            c.coolant_c.map_or(false, |t| t > 115.0), "C", "H", "TEMP", font, th);

    // Corners: range and clock left, ambient right
    let range = c.range_km.map_or("-- km".into(), |r| format!("{:.0} km", r));
    let clock = c.clock.clone().unwrap_or_else(|| "--:--".into());
    p.text(at(-290.0, 156.0), Align2::CENTER_CENTER, format!("{range}     {clock}"),
            FontId::monospace(14.0 * k), th.text);
    p.text(at(290.0, 156.0), Align2::CENTER_CENTER,
            c.ambient_c.map_or("--.- °C".into(), |t| format!("{:.1} °C", t)), FontId::monospace(14.0 * k), th.text);
}