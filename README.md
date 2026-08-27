# Revlab

Deterministic multi-rate simulation framework for internal-combustion vehicles, written in Rust. Physically modeled engine, powertrain and sensors driving a realistic ECU over a simulated signal boundary, with bit-exact replay for regression testing. Currently: turbo diesel.

The reference vehicle is a 2017 VW Passat B8 with the EA288 1.6 TDI and a DQ200 seven-speed dual-clutch gearbox.

---

## The idea

The ECU only ever sees what a sensor line would carry — quantized, noisy, delayed, and occasionally lying. It has no access to plant state. That single constraint is what the whole architecture is built around, and it's what makes the interesting failures reproducible:

- A drifting crank sensor can stall the engine while the ECU reports a perfect 800 rpm
- The ECU can be wrong about its own friction and still idle flawlessly, because the integrator quietly absorbs the model error — until the error moves, and then the error is visible in the arbitrated torque
- A plausibility monitor with only two signals can detect that something is wrong but cannot tell you *which* sensor is lying

None of those are scripted. They fall out of the separation.

---

## Layout

```
revlab/
├── crates/
│    ├── revlab-cli/        scenarios and the runner
│    ├── revlab-core/       time base, seeded PRNG, interpolated maps
│    └── revlab-kernel/     scheduler, plant, sensors, ECU, telemetry
└── tools/
     ├── plot.py            telemetry plotting
     ├── run_scenarios.sh   full validation sweep
     └── check_run.py       per-run health metrics
```

`revlab-kernel` is internally split into `plant/`, `sensors/`, and `ecu/`. Nothing under `ecu/` imports from `plant/` — that becomes a crate boundary later so the compiler enforces it.

---

## Determinism

Replay is bit-exact: the same seed produces a byte-identical CSV. That's a load-bearing property, not a nicety — it's what makes a regression baseline meaningful, and it constrains the design:

- **Integer nanosecond time.** Accumulating `f64` seconds drifts, and the drift is platform dependent.
- **Per-component seeded PRNG** (xoshiro256\*\*, hand-rolled so a `cargo update` can't change the number stream). Never `thread_rng`.
- **Deterministic tie breaking.** Simultaneous events resolve on `(time, component id, trigger index)` — never hash or insertion order.
- **No wall clock reads anywhere in the graph.**

One consequence worth knowing: each component's PRNG is derived from its registration index, so inserting a component mid-list changes the noise stream of every component after it. Replay still holds, but cross-commit comparison only survives if new components are appended.

---

## What's modeled

### Plant

|              |                                                                                                                               |
|--------------|-------------------------------------------------------------------------------------------------------------------------------|
| Engine       | mean-value crank dynamics, indicated torque from an efficiency map over (speed, load)                                         |
| Friction     | Chen-Flynn correlation — physical form, coefficients fitted per engine family, scaled by oil viscosity                        |
| Geometry     | measured inputs (bore, stroke, conrod, flywheel) resolving to derived inertia and displacement                                |
| Intake       | filling-and-emptying manifold, sub-stepped for stiffness                                                                      |
| Exhaust      | manifold with a temperature state, driven by the share of fuel energy that became neither work nor in-cylinder heat rejection |
| Turbo        | compressor, turbine and shaft as one device; ellipse compressor map, VNT vane area, shaft inertia                             |
| Thermal      | coolant/block and oil as separate masses, thermostat-gated radiator, oil cooler, friction heat split between the two          |
| Clutch       | one dry pack of a dual-clutch gearbox, owning the transmission input shaft — open, slipping and locked                        |
| Driveline    | measured DQ200 ratios, vehicle inertia reflected onto the input shaft                                                         |
| Road load    | aero, rolling, grade and brake force from physical parameters rather than a coastdown polynomial                              |
| Environment  | ambient pressure and temperature as a component — altitude, weather and headwind can vary at runtime                          |

### Sensors

Modeled as mechanisms, not ideal readings. The 60-2 crank wheel infers speed from tooth period, so measurement staleness (1.25 ms at idle, 0.25 ms at 4000 rpm) and timer quantization emerge rather than being bolted on. The cam wheel fires every 180 crank degrees — coarse and laggy, but *independent*, which is what makes it useful for cross-checking. Analogue sensors (MAP, IAT, MAF, EGT, turbo speed, barometric, coolant) carry first-order lag, noise and ADC quantization, plus a validity line that goes low when there is no signal to read.

Fault injection: stuck-at, offset, drift, open circuit.

### ECU

One component with an internal task table at 1/10/100 ms sharing a single RAM image — inputs latched, tasks run, outputs written. Tasks cannot reach the simulation bus.

- **Torque structure.** Requests carry a *kind*: idle control is a MinLimit, driver demand is a Target, the smoke limiter and rev limiter are MaxLimits. The arbiter applies targets, then minimum guarantees, then maximum limits last so protection can't be overridden.
- **Inverse torque model** that deliberately does *not* invert the plant. The ECU has no access to the efficiency map or friction correlation, so its conversion is a single calibrated constant and the residual bias is what closed-loop control exists to absorb.
- **Loss model** converting crank torque to indicated, with an ECT-indexed warm-up multiplier — the same multiplier the speed observer scales its own friction by, because two models that must agree cannot each own a copy of the calibration.
- **Speed observer** — the ECU's own engine model, integrated from the fuel it commanded using its own inertia, efficiency and friction. A PI structure: proportional correction toward the trusted sensor plus an integral estimate of unmodelled torque, both frozen while a fault is under evaluation, so the integrator cannot learn a lying sensor as load. Accuracy is a bonus; independence is the requirement.
- **Three-way plausibility vote.** Crank and cam are each compared against the model, so a fault can be *attributed* rather than merely detected. Confirming the cam leaves control on the crank; confirming the crank substitutes cam. Per-sensor fault memory with freeze frames, debounce and healing.
- **Dynamic tolerance.** The cam's disagreement during a transient is predictable from its lag, so the threshold widens by exactly the error that lag accounts for — barely at all for a slow drift, hugely for a hard load step.
- **Smoke limiter** capping fuel at the air actually available, estimated by the ECU from MAP and IAT with its own volumetric efficiency calibration, blended against the MAF signal.

---

## Running it

```bash
cargo run -p revlab-cli -- --list
cargo run -p revlab-cli -- --scenario launch --seed 42 --plot
./tools/run_scenarios.sh          # full sweep, replay check, health metrics
```

A run is fully described by `(scenario, seed)`. Flags: `--scenario --seed --out --plot --live --realtime --speed`.

| scenario       |                                                               |
|----------------|---------------------------------------------------------------|
| `nominal`      | baseline idle from cold, 40 minutes                           |
| `crank_drift`  | crank sensor drifts +20 rpm/s from t=10 s                     |
| `crank_stuck`  | crank sensor freezes at 800 rpm                               |
| `crank_open`   | crank signal lost                                             |
| `cam_drift`    | cam drifts instead — does the monitor blame the right sensor? |
| `load_step`    | 60 Nm applied at t=5 s                                        |
| `spool`        | 2500 rpm + 80 Nm at t=5 s — turbo spools                      |
| `pedal_ramp`   | pedal to 40% at t=5 s, released at t=12 s                     |
| `pedal_full`   | pedal to 100%, no load — watch the rev limit                  |
| `drive_away`   | rolling start at 23.6 km/h in 4th, pedal to 50% at t=5 s      |
| `launch`       | 1st gear from rest, clutch ramped in over 1.5 s               |

Plotting needs `matplotlib`.

---

## Sanity

Numbers that fall out of the model rather than being tuned to match:

- 1598 cc, J = 0.0952 kg·m², friction 17.3 Nm at idle
- warm idle holds 800 rpm on **7.09 mg/stroke** at AFR 61 — the band a real EA288 burns, and diesels do idle that lean
- coolant reaches its thermostat and holds at 85.3 °C with no residual drift; oil settles 3.9 °C below it at sustained idle, and the friction multiplier falls from 2.30 cold to 1.26 warm
- EGT runs 382 °C at cold idle to 697 °C smoke-limited, against roughly 700–800 °C for the real engine at full load
- `spool`: turbo 19,547 → 88,496 rpm, AFR floored at 17.8 by the smoke limiter, fuel held at the limit for 63% of the transient and released as boost arrives
- `drive_away`: 2400 rpm gives **70.82 km/h** in 4th, against 70 km/h read off the car's own dash
- `launch`: one clutch engagement puts **39.5 kJ** into the friction surfaces, peaking at 57 kW

The gear ratios are measured from the vehicle, so the 70.82 is a check against reality rather than against the model. Everything else in the list is the model agreeing with itself.

---

## Not yet

Boost and EGR control (VNT is currently pinned open); the second clutch, shift logic and a TCU; aftertreatment; wear and clutch fade. An open crank circuit is not diagnosed — a missing signal is a continuity fault rather than a correlation one, and that monitor doesn't exist yet.

---

## Direction

Revlab is built to eventually be *configured* rather than scripted: an engine defined by its geometry and materials, run under conditions the user chooses, with the consequences of those choices falling out of the physics. That's why geometry, derived quantities and fitted correlations are separated, why ambient conditions are a component rather than constants, and why the ECU carries its own approximate model instead of inverting the plant.

The long-term target is that sustained operation matters — that holding an engine past its limits produces heat soak, oil film breakdown and accumulated wear, rather than just a number that stays high. The thermal states are in; the wear is next.

## License

Source available for review. All rights reserved.