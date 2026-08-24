#!/usr/bin/env bash
# Validation sweep. Runs each scenario twice to prove byte-exact replay, then reports first pending/confirmed DTC.
# Output lands in runs/
set -euo pipefail

SEED=${SEED:-42}
OUT=${OUT:-runs}
SCENARIOS=(nominal cam_drift crank_drift crank_open crank_stuck load_step spool pedal_ramp pedal_full drive_away)

mkdir -p "$OUT"
cargo build --release

for s in "${SCENARIOS[@]}"; do
  echo "=== $s"
  cargo run --release --quiet -- --scenario "$s" --seed "$SEED" --out "$OUT/$s.csv" 2>/dev/null
  cargo run --release --quiet -- --scenario "$s" --seed "$SEED" --out "$OUT/.$s.replay" 2>/dev/null
  if cmp -s "$OUT/$s.csv" "$OUT/.$s.replay"; then
    echo " replay OK"
  else
    echo " replay MISMATCH"
  fi
  rm -f "$OUT/.$s.replay"
  python3 tools/check_run.py "$OUT/$s.csv"
done