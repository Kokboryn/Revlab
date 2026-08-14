# Revlab

Deterministic multi-rate simulation framework for internal-combustion vehicles, written in Rust. Physically modeled engine, powertrain and sensors driving a realistic ECU over a simulated signal boundary, with bit exact replay for regression testing. Currently: turbo diesel

The reference engine is a VW EA288 1.6 TDI.

---

## The idea

The ECU only ever sees what a sensor line would carry - quantized, noisy, delayed, and occasionally lying. It has no access to plant state. That single constraint is what the whole architecture is built around, and it's what makes the interesting failures reproducible:

- A drifting crank sensor can stall the engine while the ECU reports a perfect 800 rpm
- The ECU can be 9% wrong about its own torque output and still idle flawlessly, because the integrator quietly absorbs the model error
- A plausibility monitor with only two signals can detect that something is wrong but cannot tell you *which* sensor is lying.

None of those are scripted. They fall out of the separation.

---

## Layout

```
revlab/
├── crates/
│    ├── revlab-cli/        scenarios and the runner
│    ├── revlab-core/       time base, seede PRNG, interpolated maps
│    └── revlab-kernel/     scheduler, plant, sensors, ECU, telemetry
└── tools/plot.py           telemetry plotting
```

`revlab-kernel` is internally split into `plant/`, `sensors/`, and `ecu/`. Nothing under `ecu/` imports from `plant/` - that becomes a crate boundary later so the compiler enforces it.

---

## Determinism

Replay is bit-exact: the same seed produces a byte-identical CSV. That's a load-bearing property, nota nicety - it's what makes a regression baseline meaningful, and it constrains the design:

- **Integer nanosecond time.** Accumulating `f64` seconds drifts, and the drift is platform dependent.
- **Per component seeded PRNG** (xoshiro256\*\*, hand-rolled so a `cargo update` can't change the number stream). Never `thread_rng`.
- **Deterministic tie breaking.** Simultaneous events resolve on `(time, component id, trigger index)` - never hash or insertion order
- **No wall clock reads anywhere in the graph.**

---

## What's modeled

### Plant

|          |                                                                                                      |
|----------|------------------------------------------------------------------------------------------------------|
| Engine   | mean value crank dynamics, indicated torque from an effiiency map over (speed, load)                 |
| Friction | Chen-Flynn correlation - physical form, cofficients fitted per engine family                         |
| Geometry | measured inputs (bore, stroke, conrod, flywheel) resolving to derived inertia and displacement       |
| Intake   | filling and emptying manifold, sub stepped for stiffness                                             |
| Exhaust  | manifold with a temperature state, driven by the fuel energy that didn't become work                 |
| Turbo    | compressor, turbine and shaft as one device; ellipse compressor map, VNT vane area, shaft inertia    |
| Environment | ambient pressure and temperature as a component - altitude, weather and headwind can vary at runtime |

### Sensors

Modeled as mechanisms, not ideal readings. The 60 crank wheel infers speed from tooth period, so measurement staleness (1.25 ms at idle, 0.25 ms at 4000 rpm) and timer quantization emerge rather than being bolted on. The cam wheel fires every 180 crank degrees - coarse and laggy, but *independent*, which is what makes it useful for cross-checking. Analogue sensors (MAP, IAT) carry first order lag, noise and 12-bit ADC quantization.

Fault injection: stuck-at, offset, drift, open circuit.

### ECU

One component with an internal task table at 1/10/100 ms sharing a single RAM image - inputes latched, tasks run, outputs written. Tasks cannot reach the simulation bus.

- **Torque structure.** Requests carry a *kind*: idle control is a MinLimit, driver demand will be a Target, the smoke limiter is a MaxLimit. The arbiter applies targets, then minimum guarantees, then maximum limits last so protection can't be overridden.
- **Inverse torque model** that deliberately does *not* invert the plant. The ECU has no access to the efficiency map or friction correlation, so its conversion is a single calibrated constant nd the residual bias is what closed loop control exists to absorb.
- **Speed observer** - the ECU's own engine model, integrated from the fuel it commanded using its own inertia, efficiency and friction. Tracks true speed within ~1 rpm. Accuracy is a bonus; independence is the requirement.
- **Three-way plausibility vote.** Crank and cam are each compared against the model, so a fault can be *attributed* rather than merely detected. Confirming the cam leaves control on the crank; confirming the crank substitutes cam. Per-sensor fault memory with freeze frames, debounce and healing.
- **Dynamic tolerance.** The cam's disagreement during a transient is predictable, so the threshold widens it by 0.4 rpm; a 5000 rpm/s load step widens it by 450.
- **Smoke limiter** capping fuel at the air actually available, estimated by the ECU from MAP and IAT with its own volumetric efficiency calibration.

---

## Running it

```bash
cargo run -p revlab-cli -- --list
cargo run -p revlab-cli -- --scenario spool --seed 42 --plot
```

A run is fully described by `(scenario, seed)`. Flags: `--scenario --seed --out --plot`.

| scenario      |                                                              |
|---------------|--------------------------------------------------------------|
| `nominal`     | baseline idle, no faults                                     |
| `crank_drift` | crank sensor drifts +20 rpm/s from t=10 s                    |
| `crank_stuck` | crank sensor freezes at 800 rpm                              |
| `crank_open`  | crank signal lost                                            |
| `cam_drift`   | cam drifts instead - does te monitor blame the right sensor? |
| `load_step`   | 60 Nm applied at t=5 s                                       |
| `spool`       | 2500 rpm + 80 Nm at t=5 s - turbo spools                     |

Plotting needs `matplotlib`.

---

## Sanity

Numbers that fall out of model rater than being tuned to match:

- 1598 cc, J = 0.952 kg·m², friction 173 Nm at idle
- idle holds 800 rpm on **6.10 mg/stroke** - the band a real EA288 burns
- idle AFR ≈ 71 (diesels idle very lean), falling to 25 under load
- `spool`: turbo 18,880 -> 83,542 rpm, AFR floored at 18.1 by the smoke limiter, torque held down by available air for ~1.5 s and released as boost arrives

That last one is turbo lag as a consequence of air constraint, not a scripted delay.

---

## Not yet

Driver demand and he ECU's crank vs indicated loss model; boost and EGR control (VNT is currently inned open); transmission, driveline and vehicle dynamics; aftertreatment. Sensors fabricate a speed when the engine is stopped, so stall detection and restart aren't possible yet.

---

## Direction

Revlab is built to eventually be *configured* rather than scripted: an engine defined by its geometry and materials, run under conditions the user chooses with consequences of those choices falling out of the physics. That's why geometry and materials, run under conditions the user chooses, with the consequences of those choices falling out of the physics. That's why geometry, derived quantities and fitted correlations are separated, why ambient conditions are a component rather than constants, and why the ECU carries its own approximate model instead of inverting the plant.

The long term target is that sustained operation matters - that holding an engine past its limits produces heat soak, oil film breakdown and accumulated wear, rather than just a number that stays high. That needs thermal states before it needs anything else.

## License

Source available for review. All rights reserved.