#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=scripts/lib.sh
source "$SCRIPT_DIR/lib.sh"

if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "Usage: $0 <config-name-or-path> [output.bag]" >&2
  exit 2
fi

kalibr_pipeline_load_config "$1"
RUN_ID=$(kalibr_run_id)
OUT=${2:-$(kalibr_default_bag_path "$RUN_ID")}
OUT=$(readlink -m "$OUT")
MANIFEST=$(kalibr_record_manifest_path "$RUN_ID")

mapfile -t RECORD_TOPIC_ARRAY < <(kalibr_split_words "$RECORD_TOPICS")
mapfile -t REQUIRED_TOPIC_ARRAY < <(kalibr_split_words "$REQUIRED_RECORD_TOPICS")

DURATION_ARG=()
if [[ -n "${KALIBR_RECORD_DURATION:-}" ]]; then
  DURATION_ARG=(--duration="$KALIBR_RECORD_DURATION")
fi

if [[ "${KALIBR_DRY_RUN:-false}" == "true" ]]; then
  echo "run_id=$RUN_ID"
  echo "bag=$OUT"
  echo "manifest=$MANIFEST"
  printf 'rosbag record --buffsize=1024'
  if [[ ${#DURATION_ARG[@]} -gt 0 ]]; then
    printf ' %q' "${DURATION_ARG[@]}"
  fi
  printf ' -O %q' "$OUT"
  printf ' %q' "${RECORD_TOPIC_ARRAY[@]}"
  printf '\n'
  exit 0
fi

kalibr_source_ros
set +u
source /home/zcy/sensor_ws/devel/setup.bash 2>/dev/null || true
set -u
kalibr_check_live_topics "${REQUIRED_TOPIC_ARRAY[@]}"
mkdir -p "$(dirname "$OUT")"
MANIFEST=$(kalibr_write_record_manifest "$RUN_ID" "$OUT")

echo "[kalibr-record] run_id: $RUN_ID"
echo "[kalibr-record] bag: $OUT"
echo "[kalibr-record] manifest: $MANIFEST"
exec rosbag record --buffsize=1024 "${DURATION_ARG[@]}" -O "$OUT" "${RECORD_TOPIC_ARRAY[@]}"
