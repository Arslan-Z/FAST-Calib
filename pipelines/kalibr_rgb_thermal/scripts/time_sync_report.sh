#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=scripts/lib.sh
source "$SCRIPT_DIR/lib.sh"

if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "Usage: $0 <config-name-or-path> [bag]" >&2
  exit 2
fi

kalibr_pipeline_load_config "$1"
BAG=$(kalibr_resolve_bag "${2:-}")

kalibr_source_ros
python3 "$ROOT/tools/kalibr_time_sync_report.py" \
  --bag "$BAG" \
  --rgb-topic "$RGB_TOPIC" \
  --thermal-topic "$THERMAL_TOPIC" \
  --strict-sync "${KALIBR_APPROX_SYNC:-$DEFAULT_APPROX_SYNC}"
