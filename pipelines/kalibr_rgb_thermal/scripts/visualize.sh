#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=scripts/lib.sh
source "$SCRIPT_DIR/lib.sh"

if [[ $# -lt 3 || $# -gt 4 ]]; then
  echo "Usage: $0 <config-name-or-path> <camchain.yaml> <bag> [output-dir]" >&2
  exit 2
fi

kalibr_pipeline_load_config "$1"
CAMCHAIN=$(readlink -f "$2")
BAG=$(readlink -f "$3")
BAG_STEM=$(basename "${BAG%.bag}")
CAMCHAIN_RUN=$(basename "$(dirname "$CAMCHAIN")")
OUT=${4:-$DATA_ROOT/output/${BAG_STEM}__${CAMCHAIN_RUN}__target_plane_$(date +%Y%m%d_%H%M%S)}
OUT=$(readlink -m "$OUT")

if [[ ! -f "$CAMCHAIN" ]]; then
  echo "Camchain not found: $CAMCHAIN" >&2
  exit 1
fi
if [[ ! -f "$BAG" ]]; then
  echo "Bag not found: $BAG" >&2
  exit 1
fi
if [[ ! -f "$TARGET" ]]; then
  echo "Target YAML not found: $TARGET" >&2
  exit 1
fi

if [[ "${KALIBR_DRY_RUN:-false}" == "true" ]]; then
  printf 'python3 %q --camchain %q --target %q --bag %q --rgb-topic %q --thermal-topic %q --rgb-cam cam0 --thermal-cam cam1 --max-dt %q --max-frames %q --stride %q --output-prefix %q --output-dir %q\n' \
    "$ROOT/tools/visualize_kalibr_target_plane_overlay.py" \
    "$CAMCHAIN" "$TARGET" "$BAG" "$RGB_TOPIC" "$THERMAL_TOPIC" \
    "${KALIBR_VIS_MAX_DT:-$DEFAULT_VIS_MAX_DT}" \
    "${KALIBR_VIS_MAX_FRAMES:-$DEFAULT_VIS_MAX_FRAMES}" \
    "${KALIBR_VIS_STRIDE:-$DEFAULT_VIS_STRIDE}" \
    "$BAG_STEM" "$OUT"
  exit 0
fi

kalibr_source_ros
set +u
source "$ROOT/scripts/source_kalibr_official.sh"
set -u

python3 "$ROOT/tools/visualize_kalibr_target_plane_overlay.py" \
  --camchain "$CAMCHAIN" \
  --target "$TARGET" \
  --bag "$BAG" \
  --rgb-topic "$RGB_TOPIC" \
  --thermal-topic "$THERMAL_TOPIC" \
  --rgb-cam cam0 \
  --thermal-cam cam1 \
  --max-dt "${KALIBR_VIS_MAX_DT:-$DEFAULT_VIS_MAX_DT}" \
  --max-frames "${KALIBR_VIS_MAX_FRAMES:-$DEFAULT_VIS_MAX_FRAMES}" \
  --stride "${KALIBR_VIS_STRIDE:-$DEFAULT_VIS_STRIDE}" \
  --output-prefix "$BAG_STEM" \
  --output-dir "$OUT"
