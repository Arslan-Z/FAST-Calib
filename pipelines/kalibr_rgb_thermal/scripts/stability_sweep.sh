#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

if [[ $# -lt 1 || $# -gt 3 ]]; then
  echo "Usage: $0 <config-name-or-path> [bag] [output-root]" >&2
  echo "Set KALIBR_SWEEP='0.03:2.0 0.02:2.0 0.015:2.0' to override." >&2
  exit 2
fi

CONFIG=$1
BAG=${2:-}
OUT_ROOT=${3:-}
SWEEP=${KALIBR_SWEEP:-"0.03:2.0 0.02:2.0 0.015:2.0"}

for item in $SWEEP; do
  sync=${item%%:*}
  freq=${item##*:}
  label="sync$(printf '%s' "$sync" | tr -d .)_freq$(printf '%s' "$freq" | tr -d .)"
  out_arg=()
  if [[ -n "$OUT_ROOT" ]]; then
    out_arg=("$OUT_ROOT/$label")
  fi
  echo "[sweep] approx-sync=$sync bag-freq=$freq label=$label"
  KALIBR_APPROX_SYNC=$sync KALIBR_BAG_FREQ=$freq \
    "$SCRIPT_DIR/calibrate.sh" "$CONFIG" "$BAG" "${out_arg[@]}"
done
