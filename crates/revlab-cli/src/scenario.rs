use revlab_kernel::sensors::Fault;
use revlab_kernel::tcu::Lever;

#[derive(Clone, Copy, Debug)]
pub enum Event {
    CrankFault { at_s: f64, fault: Fault },
    CamFault   { at_s: f64, fault: Fault },
    Load       { at_s: f64, torque: f64 },
    Speed      { at_s: f64, rpm: f64 },
    Pedal      { at_s: f64, position: f64 },    // 0.0 to 1.0
    Lever      { at_s: f64, lever: Lever },    // P/R/N/D selector
    Grade      { at_s: f64, rad: f64 },
    Brake      { at_s: f64, cmd: f64 },
    TipUp      { at_s: f64 },
    TipDown    { at_s: f64 },
}

pub struct Scenario {
    pub name: &'static str,
    pub about: &'static str,
    pub duration_s: u64,
    /// Vehicle speed at t=0. Rolling starts are an initial condition, not an event: with a real
    /// clutch you cannot conjure road speed mid run.
    pub start_kmh: f64,
    pub start_gear: usize,  // gear engaged at t=0, for rolling starts
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
    ("drive_away", "rolling start at 23.6 km/h in 4th, pedal to 50% at t=5s"),
    ("launch", "select D at t=2s, pedal to 40% - TCU handles engagement"),
    ("hill_start", "10% grade, pull away from rest under TCU control"),
    ("top_speed", "full throttle from rest to top speed, list at 40 s, N at 90 s, roll to a stop"),
    ("upshifts", "full throttle in the tiptronic gate, tip up through 1-2-3-4-5-6-7"),
    ("kickdown", "part throttle in 5th from 50 km/h, then floor it and tip down 5-4-3, then try 2"),
    ("coast_down", "100 km/h in 5th, lift, tip down through 4-3-2-1 while coasting"),
];

impl Scenario {
    pub fn by_name(n: &str) -> Option<Scenario> {
        let (duration_s, start_kmh, start_gear, events): (u64, f64, usize, Vec<Event>) = match n {
            "nominal"       => (2400, 0.0, 0, vec![]),
            "crank_drift"   => (20, 0.0, 0, vec![Event::CrankFault { at_s: 10.0, fault: Fault::Drift { per_sec: 20.0 } }]),
            "crank_stuck"   => (20, 0.0, 0, vec![Event::CrankFault { at_s: 10.0, fault: Fault::StuckAt(800.0) }]),
            "crank_open"    => (20, 0.0, 0, vec![Event::CrankFault { at_s: 10.0, fault: Fault::OpenCircuit }]),
            "cam_drift"     => (20, 0.0, 0, vec![Event::CamFault { at_s: 10.0, fault: Fault::Drift {per_sec: 20.0 } }]),
            "load_step"     => (60, 0.0, 0, vec![Event::Load { at_s: 5.0, torque: 60.0 }]),
            "spool"         => (20, 0.0, 0, vec![Event::Speed { at_s: 5.0, rpm: 2500.0 }, Event::Load { at_s: 5.0, torque: 80.0 }]),
            "pedal_ramp"    => (30, 0.0, 0, vec![Event::Pedal { at_s: 5.0, position: 0.40 }, Event::Pedal { at_s: 12.0, position: 0.0 }]),
            "pedal_full"    => (60, 0.0, 0, vec![Event::Pedal { at_s: 10.0, position: 1.0 }]),
            "drive_away"    => (30, 23.6, 4, vec![Event::Lever { at_s: 0.0, lever: Lever::Drive }, Event::Pedal { at_s: 5.0, position: 0.50 }]),
            "launch"        => (30, 0.0, 1, vec![Event::Lever { at_s: 2.0, lever: Lever::Drive}, Event::Pedal { at_s: 2.5, position: 0.40 }]),
            "hill_start"     => (180, 0.0, 1, vec![
                Event::Grade { at_s: 0.0, rad: 0.0997 },    // 10%
                Event::Brake { at_s: 0.0, cmd: 0.30 },      // held on the brake first
                Event::Lever { at_s: 2.0, lever: Lever::Drive },
                Event::Pedal { at_s: 2.5, position: 0.35 },
                Event::Brake { at_s: 3.5, cmd: 0.0 },       // release once slipping
            ]),
            "top_speed"     => (240, 0.0, 1, vec![
                Event::Lever { at_s: 2.0, lever: Lever::Drive },
                Event::Pedal { at_s: 3.0, position: 1.0 },      // flat out
                Event::Pedal { at_s: 40.0, position: 0.0 },      // lift: engine braking in gear
                Event::Lever { at_s: 90.0, lever: Lever::Neutral }, // clutch opens, coast to rest
            ]),
            "upshifts"      => (80, 0.0, 1, vec![
                Event::Lever { at_s: 2.0, lever: Lever::Manual },
                Event::Pedal { at_s: 3.0, position: 1.0 },
                Event::TipUp { at_s: 7.0 },     // ~3200 rpm in 1st, per top_speed
                Event::TipUp { at_s: 12.0 },
                Event::TipUp { at_s: 20.0 },
                Event::TipUp { at_s: 31.0 },
                Event::TipUp { at_s: 45.0 },
                Event::TipUp { at_s: 62.0 },
            ]),
            "kickdown"      => (20, 50.0, 5, vec![
                Event::Lever { at_s: 0.0, lever: Lever::Manual },
                Event::Pedal { at_s: 0.0, position: 0.4 },      // part throttle in 5th, ~1300 rpm
                Event::Pedal { at_s: 8.0, position: 0.5 },      // floor it...
                Event::TipDown { at_s: 8.0 },                   // ...and tip down: 5 -> 4
                Event::TipDown { at_s: 11.0 },                  // 4 -> 3
                Event::TipDown { at_s: 14.0 },                  // 3 -> 2: refused if it would land above 4400
            ]),
            "coast_down"     => (38, 100.0, 5, vec![
                Event::Lever { at_s: 0.0, lever: Lever::Manual },
                Event::Pedal { at_s: 0.0, position: 0.3 },      // cruise
                Event::Pedal { at_s: 5.0, position: 0.0 },      // lift: engine braking in 5th
                Event::TipDown { at_s: 7.0 },                   // 5 -> 4, ~2450 -> ~3200 rpm
                Event::TipDown { at_s: 12.0 },                  // 4 -> 3
                Event::TipDown { at_s: 24.0 },                  // 3 -> 2 (earlier would overrev)
                Event::TipDown { at_s: 32.0 },                  // 2 -> 1
            ]),
            _ => return None,
        };
        let about = NAMES.iter().find(|(k,_)| *k==n).map(|(_, v)| *v)?;
        Some(Scenario { name: NAMES.iter().find(|(k,_)| *k==n).unwrap().0, about, duration_s, start_kmh, start_gear, events })
    }
}

pub struct Args {
    pub scenario: String,
    pub seed: u64,
    pub plot: bool,
    pub speed: Option<f64>,     // None = as fast as possible
    pub live: bool,
    pub wear: Option<String>,
    pub out: Option<String>,    // None = runs/<scenario>_s<seed>/run.csv
}

pub fn parse_args() -> Result<Args, String> {
    let mut a = Args { scenario: "crank_drift".into(), seed: 0xC0FFEE, out: None, plot: false, speed: None, live: false, wear: None };
    let mut it = std::env::args().skip(1);
    while let Some(k) = it.next() {
        match k.as_str() {
            "--list" => {
                for (n, d) in NAMES { println!(" {:<12} {}", n, d); }
                std::process::exit(0);
            }
            "--scenario" => a.scenario = it.next().ok_or("--scenario needs a value")?,
            "--seed" => a.seed = it.next().ok_or("--seed needs a value")?.parse().map_err(|_| "--seed must be an integer")?,
            "--out" => a.out = Some(it.next().ok_or("--out needs a value")?),
            "--plot" => a.plot = true,
            "--realtime" => a.speed = Some(1.0),
            "--speed" => a.speed = Some(it.next().ok_or("--speed needs a value")?.parse().map_err(|_| "--speed must be a number")?,),
            "--live" => a.live = true,
            "--wear" => a.wear = Some(it.next().ok_or("--wear needs a value")?),
            other => return Err(format!("unknown argument: {other}")),
        }
    }
    Ok(a)
}