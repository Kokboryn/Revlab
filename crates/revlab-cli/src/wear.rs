use std::collections::HashMap;

/// Accumulated damage carried between runs. A run is described by (scenario, seed, wear file) -- the
/// file is an input like any other, so replay still holds as long as you keep it.
#[derive(Default, Clone)]
pub struct Wear {
    vals: HashMap<String, f64>,
}

impl Wear {
    pub fn load(path: &str) -> std::io::Result<Self> {
        let mut w = Wear::default();
        let text = match std::fs::read_to_string(path) {
            Ok(t) => t,
            // A missing file is a new vehicle, not an error
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(w),
            Err(e) => return Err(e),
        };
        for (i, line) in text.lines().enumerate() {
            let line = line.split('#').next().unwrap().trim();
            if line.is_empty() { continue; }
            let (k, v) = line.split_once('=').ok_or_else(|| bad(i, "expected key = value"))?;
            let v: f64 = v.trim().parse().map_err(|_| bad(i, "value is not a number"))?;
            w.vals.insert(k.trim().to_string(), v);
        }
        Ok(w)
    }

    pub fn get(&self, key: &str, default: f64) -> f64 {
        *self.vals.get(key).unwrap_or(&default)
    }

    pub fn set(&mut self, key: &str, v: f64) {
        self.vals.insert(key.to_string(), v);
    }

    pub fn save(&self, path: &str) -> std::io::Result<()> {
        let mut keys: Vec<_> = self.vals.keys().collect();
        keys.sort();    // deterministic file, so a diff means something
        let mut out = String::from("# Revlab accumulated wear. Edit at your own risk.\n");
        for k in keys {
            out += &format!("{k} = {:.6e}\n", self.vals[k]);
        }
        std::fs::write(path, out)
    }
}

fn bad(line: usize, msg: &str) -> std::io::Error {
    std::io::Error::new(std::io::ErrorKind::InvalidData, format!("wear file line {}: {msg}", line + 1))
}