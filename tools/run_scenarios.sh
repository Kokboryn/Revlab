#!/usr/bin/env bash
# Validation sweep. Runs each scenario twice to prove byte-exact replay, then reports first pending/confirmed DTC.
# Output lands in runs/
set -euo pipefail

SEED=${SEED:-42}
OUT=${OUT:-runs}
SCENARIOS=(nominal cam_drift crank_drift crank_open crank_stuck load_step spool pedal_ramp pedal_full drive_away launch hill_start)

mkdir -p "$OUT"
cargo build --release

for s in "${SCENARIOS[@]}"; do
    echo "=== $s"
    dir="$OUT/${s}_s$SEED"
    cargo run --release --quiet -- --scenario "$s" --seed "$SEED" --out "$dir/run.csv" --plot 2>/dev/null
    cargo run --release --quiet -- --scenario "$s" --seed "$SEED" --out "$dir/.replay.csv" 2>/dev/null
    if cmp -s "$dir/run.csv" "$dir/.replay.csv"; then
        echo " replay OK"
    else
        echo " replay MISMATCH"
    fi
    rm -f "$dir/.replay.csv"
    python3 tools/check_run.py "$dir/run.csv"
done