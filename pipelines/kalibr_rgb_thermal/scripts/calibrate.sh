#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=scripts/lib.sh
source "$SCRIPT_DIR/lib.sh"

if [[ $# -lt 1 || $# -gt 3 ]]; then
  echo "Usage: $0 <config-name-or-path> [bag] [output-dir]" >&2
  exit 2
fi

kalibr_pipeline_load_config "$1"
BAG=$(kalibr_resolve_bag "${2:-}")
OUT=$(kalibr_output_dir "$BAG" "${3:-}")
BAG_STEM=$(basename "${BAG%.bag}")

mapfile -t TOPIC_ARRAY < <(kalibr_split_words "${KALIBR_TOPICS:-$TOPICS}")
mapfile -t MODEL_ARRAY < <(kalibr_split_words "${KALIBR_MODELS:-$MODELS}")
if [[ ${#TOPIC_ARRAY[@]} -ne ${#MODEL_ARRAY[@]} ]]; then
  echo "KALIBR_TOPICS and KALIBR_MODELS must have the same number of entries." >&2
  exit 1
fi
if [[ ! -f "$TARGET" ]]; then
  echo "Target YAML not found: $TARGET" >&2
  exit 1
fi

kalibr_source_ros
kalibr_check_bag_topics "$BAG" "${TOPIC_ARRAY[@]}"

mkdir -p "$OUT"
BAG_LINK="$OUT/input.bag"
ln -sfn "$BAG" "$BAG_LINK"

BAG_FREQ=${KALIBR_BAG_FREQ:-$DEFAULT_BAG_FREQ}
APPROX_SYNC=${KALIBR_APPROX_SYNC:-$DEFAULT_APPROX_SYNC}
VIS_OUT=${KALIBR_VIS_OUT:-$OUT/target_plane_overlay}
KALIBR_BIN=${KALIBR_CALIBRATE_CAMERAS_BIN:-/home/zcy/kalibr_ws/devel/lib/kalibr/kalibr_calibrate_cameras}
kalibr_write_calibration_manifest "$OUT" "$BAG"

if [[ "${KALIBR_DRY_RUN:-false}" == "true" ]]; then
  echo "cd $OUT"
  printf '%q --bag %q --topics' "$KALIBR_BIN" "$BAG_LINK"
  printf ' %q' "${TOPIC_ARRAY[@]}"
  printf ' --models'
  printf ' %q' "${MODEL_ARRAY[@]}"
  printf ' --target %q --bag-freq %q --approx-sync %q --dont-show-report\n' "$TARGET" "$BAG_FREQ" "$APPROX_SYNC"
  if [[ "${KALIBR_SKIP_VIS:-false}" != "true" ]]; then
    printf '%q %q %q %q %q\n' "$ROOT/pipelines/kalibr_rgb_thermal/scripts/visualize.sh" "$CONFIG_PATH" "$OUT/input-camchain.yaml" "$BAG" "$VIS_OUT"
  fi
  exit 0
fi

kalibr_source_runtime
KALIBR_BIN=${KALIBR_CALIBRATE_CAMERAS_BIN:-$KALIBR_BIN}
if [[ ! -x "$KALIBR_BIN" ]]; then
  echo "Kalibr camera calibration binary not found or not executable: $KALIBR_BIN" >&2
  echo "Rebuild Kalibr with: KALIBR_WS=/home/zcy/kalibr_ws ./scripts/build_kalibr_official.sh" >&2
  exit 1
fi
cd "$OUT"
"$KALIBR_BIN" \
  --bag "$BAG_LINK" \
  --topics "${TOPIC_ARRAY[@]}" \
  --models "${MODEL_ARRAY[@]}" \
  --target "$TARGET" \
  --bag-freq "$BAG_FREQ" \
  --approx-sync "$APPROX_SYNC" \
  --dont-show-report

CAMCHAIN="$OUT/input-camchain.yaml"
if [[ "${KALIBR_SKIP_VIS:-false}" != "true" && -f "$CAMCHAIN" ]]; then
  echo
  echo "[kalibr] generating target-plane overlay:"
  echo "[kalibr]   $VIS_OUT"
  if KALIBR_VIS_MAX_DT="${KALIBR_VIS_MAX_DT:-$APPROX_SYNC}" \
      "$ROOT/pipelines/kalibr_rgb_thermal/scripts/visualize.sh" "$CONFIG_PATH" "$CAMCHAIN" "$BAG" "$VIS_OUT"; then
    echo "[kalibr] target-plane overlay ready:"
    echo "[kalibr]   $VIS_OUT/index.html"
  else
    echo "[kalibr] warning: target-plane overlay failed; calibration output remains in $OUT" >&2
  fi
elif [[ ! -f "$CAMCHAIN" ]]; then
  echo "[kalibr] warning: camchain not found, skipping target-plane overlay: $CAMCHAIN" >&2
fi
