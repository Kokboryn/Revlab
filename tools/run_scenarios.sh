#!/usr/bin/env bash
# Validation sweep. Runs each scenario twice to prove byte-exact replay, then reports first pending/confirmed DTC.
# Output lands in runs/
set -euo pipefail

SEED=${SEED:-42}
OUT=${OUT:-runs}
PLOT_FLAG=${PLOT:-1}; [[ "$PLOT_FLAG" == 1 ]] && PLOT_FLAG=--plot || PLOT_FLAG=

mkdir -p "$OUT"
cargo build --release
mapfile -t SCENARIOS < <(cargo run --release --quiet -- --list | awk '{print $1}')

for s in "${SCENARIOS[@]}"; do
    echo "=== $s"
    dir="$OUT/${s}_s$SEED"
    mkdir -p "$dir"
    if ! cargo run --release --quiet -- --scenario "$s" --seed "$SEED" --out "$dir/run.csv" $PLOT_FLAG 2>"$dir/stderr.log"; then
      echo " RUN FAILED (see $dir/stderr.log)"
      continue
    fi
    cargo run --release --quiet -- --scenario "$s" --seed "$SEED" --out "$dir/.replay.csv" 2>/dev/null
    if cmp -s "$dir/run.csv" "$dir/.replay.csv"; then
        echo " replay OK"
    else
        echo " replay MISMATCH"
    fi
    rm -f "$dir/.replay.csv"
    python3 tools/check_run.py "$dir/run.csv"
done