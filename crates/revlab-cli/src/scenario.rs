use revlab_kernel::sensors::Fault;

#[derive(Clone, Copy, Debug)]
pub enum Event {
    CrankFault { at_s: f64, fault: Fault },
    CamFault   { at_s: f64, fault: Fault },
    Load       { at_s: f64, torque: f64 },
    Speed      { at_s: f64, rpm: f64 },
    Pedal      { at_s: f64, position: f64 },    // 0.0 to 1.0
    Gear       { at_s: f64, gear: f64 },    // 0 = neutral, 1..7
    Clutch     { at_s: f64, cmd: f64 },
    Grade      { at_s: f64, rad: f64 },
    Brake      { at_s: f64, cmd: f64 },
}

pub struct Scenario {
    pub name: &'static str,
    pub about: &'static str,
    pub duration_s: u64,
    /// Vehicle speed at t=0. Rolling starts are an initial condition, not an event: with a real
    /// clutch you cannot conjure road speed mid run.
    pub start_kmh: f64,
    pub events: Vec<Event>,
}

pub const NAMES: &[(&str, &str)] = &[
    ("nominal",     "no faults - baseline idle"),
    ("crank_drift", "crank sensor drifts +20 rpm/s from t=10s"),
    ("crank_stuck", "crank sensor freezes at 800 rpm from t=10s"),
    ("crank_open", "crank signal lost at t=10s"),
    ("cam_drift", "CAM drifts instead - does the monitor blame the right sensor?"),
    ("load_step", "60 Nm load applied at t=5s - spools the turbo"),
    ("spool", "2500 rpm + 80 Nm at t=5s - turbo spools"),
    ("pedal_ramp", "pedal to 40% at t=5s, released at t=12s"),
    ("pedal_full", "pedal to 100% at t=5s, no load- watch the rev limit"),
    ("drive_away", "4th engaged at t=2s from idle (~24 km/h), pedal to 50% at t=5s"),
    ("launch", "1st gear, clutch ramped 0->1 over 2s from t=2, pedal 40% at t=2.5"),
    ("hill_hold", "10% grade, 1st gear, clutch slipped to hold station - fade"),
];

impl Scenario {
    pub fn by_name(n: &str) -> Option<Scenario> {
        let (duration_s, start_kmh, events): (u64, f64, Vec<Event>) = match n {
            "nominal"       => (2400, 0.0, vec![]),
            "crank_drift"   => (20, 0.0, vec![Event::CrankFault { at_s: 10.0, fault: Fault::Drift { per_sec: 20.0 } }]),
            "crank_stuck"   => (20, 0.0, vec![Event::CrankFault { at_s: 10.0, fault: Fault::StuckAt(800.0) }]),
            "crank_open"    => (20, 0.0, vec![Event::CrankFault { at_s: 10.0, fault: Fault::OpenCircuit }]),
            "cam_drift"     => (20, 0.0, vec![Event::CamFault { at_s: 10.0, fault: Fault::Drift {per_sec: 20.0 } }]),
            "load_step"     => (60, 0.0, vec![Event::Load { at_s: 5.0, torque: 60.0 }]),
            "spool"         => (20, 0.0, vec![Event::Speed { at_s: 5.0, rpm: 2500.0 }, Event::Load { at_s: 5.0, torque: 80.0 }]),
            "pedal_ramp"    => (30, 0.0, vec![Event::Pedal { at_s: 5.0, position: 0.40 }, Event::Pedal { at_s: 12.0, position: 0.0 }]),
            "pedal_full"    => (60, 0.0, vec![Event::Pedal { at_s: 10.0, position: 1.0 }]),
            "drive_away"    => (30, 23.6, vec![Event::Clutch { at_s: 0.0, cmd: 1.0 }, Event::Gear { at_s: 0.0, gear: 4.0 }, Event::Pedal { at_s: 5.0, position: 0.50 }]),
            "launch"        => (30, 0.0, vec![Event::Gear { at_s: 2.0, gear: 1.0 }, Event::Clutch { at_s: 2.0, cmd: 0.0 }, Event::Clutch { at_s: 4.0, cmd: 1.0 }, Event::Pedal { at_s: 2.5, position: 0.40 }]),
            "hill_hold"     => (180, 0.0, vec![
                Event::Grade { at_s: 0.0, rad: 0.0997 },    // 10%
                Event::Brake { at_s: 0.0, cmd: 0.30 },      // held on the brake first
                Event::Gear { at_s: 2.0, gear: 1.0 },
                Event::Clutch { at_s: 2.5, cmd: 0.348 },
                Event::Pedal { at_s: 2.5, position: 0.12 },
                Event::Brake { at_s: 3.5, cmd: 0.0 },       // release once slipping
            ]),
            _ => return None,
        };
        let about = NAMES.iter().find(|(k,_)| *k==n).map(|(_, v)| *v)?;
        Some(Scenario { name: NAMES.iter().find(|(k,_)| *k==n).unwrap().0, about, duration_s, start_kmh, events })
    }
}

pub struct Args {
    pub scenario: String,
    pub seed: u64,
    pub out: String,
    pub plot: bool,
    pub speed: Option<f64>,     // None = as fast as possible
    pub live: bool,
}

pub fn parse_args() -> Result<Args, String> {
    let mut a = Args { scenario: "crank_drift".into(), seed: 0xC0FFEE, out: "run.csv".into(), plot: false, speed: None, live: false };
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
            "--realtime" => a.speed = Some(1.0),
            "--speed" => a.speed = Some(it.next().ok_or("--speed needs a value")?.parse().map_err(|_| "--speed must be a number")?,),
            "--live" => a.live = true,
            other => return Err(format!("unknown argument: {other}")),
        }
    }
    Ok(a)
}