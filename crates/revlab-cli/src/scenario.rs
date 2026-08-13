use revlab_kernel::sensors::Fault;

#[derive(Clone, Copy, Debug)]
pub enum Event {
    CrankFault { at_s: f64, fault: Fault },
    CamFault   { at_s: f64, fault: Fault },
    Load       { at_s: f64, torque: f64 },
    Speed      { at_s: f64, rpm: f64 },
}

pub struct Scenario {
    pub name: &'static str,
    pub about: &'static str,
    pub duration_s: u64,
    pub events: Vec<Event>,
}

pub const NAMES: &[(&str, &str)] = &[
    ("nominal",     "no faults - baseline idle"),
    ("crank_drift", "crank sensor drifts +20 rpm/s from t=10s"),
    ("crank_stuck", "crank sensor freezes at 800 rpm from t=10s"),
    ("crank_open", "crank signal lost at t=10s"),
    ("cam_drift", "CAM drifts instead - does the monitor lame the right sensor?"),
    ("load_step", "60 Nm load applied at t=5s - spools the turbo"),
    ("spool", "2500 rpm + 8000 Nm at t=5s - turbo spools"),
];

impl Scenario {
    pub fn by_name(n: &str) -> Option<Scenario> {
        let (duration_s, events): (u64, Vec<Event>) = match n {
            "nominal"       => (20, vec![]),
            "crank_drift"   => (20, vec![Event::CrankFault { at_s: 10.0, fault: Fault::Drift { per_sec: 20.0 } }]),
            "crank_stuck"   => (20, vec![Event::CrankFault { at_s: 10.0, fault: Fault::StuckAt(800.0) }]),
            "crank_open"    => (20, vec![Event::CrankFault { at_s: 10.0, fault: Fault::OpenCircuit }]),
            "cam_drift"     => (20, vec![Event::CamFault { at_s: 10.0, fault: Fault::Drift {per_sec: 20.0 } }]),
            "load_step"     => (20, vec![Event::Load { at_s: 5.0, torque: 60.0 }]),
            "spool"         => (20, vec![Event::Speed { at_s: 5.0, rpm: 2500.0 }, Event::Load { at_s: 5.0, torque: 80.0 }]),
            _ => return None,
        };
        let about = NAMES.iter().find(|(k,_)| *k==n).map(|(_, v)| *v)?;
        Some(Scenario { name: NAMES.iter().find(|(k,_)| *k==n).unwrap().0, about, duration_s, events })
    }
}

pub struct Args {
    pub scenario: String,
    pub seed: u64,
    pub out: String,
    pub plot: bool
}

pub fn parse_args() -> Result<Args, String> {
    let mut a = Args { scenario: "crank_drift".into(), seed: 0xC0FFEE, out: "run.csv".into(), plot: false };
    let mut it = std::env::args().skip(1);
    while let Some(k) = it.next() {
        match k.as_str() {
            "--list" => {
                for (n, d) in NAMES { println!(" {:<12} {}", n, d); }
                std::process::exit(0);
            }
            "--scenario" => a.scenario = it.next().ok_or("--scenario needs a value")?,
            "--seed" => a.seed = it.next().ok_or("--seed needs a value")?.parse().map_err(|_| "--seed must be an integer")?,
            "--out" => a.out = it.next().ok_or("--out needs a value")?,
            "--plot" => a.plot = true,
            other => return Err(format!("unknown argument: {other}")),
        }
    }
    Ok(a)
}