#!/usr/bin/env bash

kalibr_pipeline_root() {
  cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd
}

kalibr_pipeline_config_path() {
  local root="$1"
  local config="$2"
  if [[ -f "$config" ]]; then
    readlink -f "$config"
  elif [[ -f "$root/pipelines/kalibr_rgb_thermal/configs/$config.env" ]]; then
    readlink -f "$root/pipelines/kalibr_rgb_thermal/configs/$config.env"
  else
    echo "Unknown Kalibr RGB/thermal config: $config" >&2
    echo "Available configs:" >&2
    find "$root/pipelines/kalibr_rgb_thermal/configs" -maxdepth 1 -type f -name '*.env' -printf '  %f\n' >&2
    return 2
  fi
}

kalibr_pipeline_load_config() {
  if [[ $# -ne 1 ]]; then
    echo "Usage: kalibr_pipeline_load_config <config-name-or-path>" >&2
    return 2
  fi
  ROOT=$(kalibr_pipeline_root)
  export ROOT
  CONFIG_PATH=$(kalibr_pipeline_config_path "$ROOT" "$1")
  export CONFIG_PATH
  # shellcheck source=/dev/null
  source "$CONFIG_PATH"
  TARGET=${KALIBR_TARGET:-$ROOT/$TARGET_REL}
  export TARGET
}

kalibr_source_ros() {
  set +u
  source /opt/ros/noetic/setup.bash
  set -u
}

kalibr_source_runtime() {
  set +u
  source "$ROOT/scripts/source_kalibr_isolated.sh"
  set -u
}

kalibr_latest_bag() {
  {
    find "$DATA_ROOT/runs" -path "*/raw/input.bag" \( -type f -o -type l \) 2>/dev/null || true
    find "$DATA_ROOT" -maxdepth 1 -type f -name "${BAG_PREFIX}_[0-9]*.bag" 2>/dev/null || true
  } | sort | tail -n 1
}

kalibr_resolve_bag() {
  local bag="${1:-}"
  if [[ -z "$bag" ]]; then
    bag=$(kalibr_latest_bag || true)
  fi
  if [[ -z "$bag" || ! -f "$bag" ]]; then
    echo "Missing bag for pipeline '$PIPELINE_NAME'. Pass a bag path or record first." >&2
    return 1
  fi
  realpath -s "$bag"
}

kalibr_split_words() {
  local input="$1"
  read -r -a __kalibr_words <<< "$input"
  printf '%s\n' "${__kalibr_words[@]}"
}

kalibr_output_dir() {
  local bag="$1"
  local requested="${2:-}"
  local bag_stem
  bag_stem=$(basename "${bag%.bag}")
  if [[ -n "$requested" ]]; then
    readlink -m "$requested"
  elif [[ "$bag" == "$DATA_ROOT"/runs/*/raw/input.bag ]]; then
    local run_dir
    run_dir=$(dirname "$(dirname "$bag")")
    readlink -m "$run_dir/results/kalibr/calib_$(date +%Y%m%d_%H%M%S)"
  else
    readlink -m "$DATA_ROOT/output/${bag_stem}__kalibr_$(date +%Y%m%d_%H%M%S)"
  fi
}

kalibr_run_id() {
  local run_id="${KALIBR_RUN_ID:-}"
  if [[ -z "$run_id" ]]; then
    run_id="${PIPELINE_NAME}_$(date +%Y%m%d_%H%M%S)"
  fi
  printf '%s\n' "$run_id"
}

kalibr_default_bag_path() {
  local run_id="$1"
  readlink -m "$DATA_ROOT/runs/$run_id/raw/input.bag"
}

kalibr_record_manifest_path() {
  local run_id="$1"
  readlink -m "$DATA_ROOT/runs/$run_id/run_manifest.env"
}

kalibr_write_record_manifest() {
  local run_id="$1"
  local bag="$2"
  local manifest
  manifest=$(kalibr_record_manifest_path "$run_id")
  mkdir -p "$(dirname "$manifest")"
  {
    printf 'KALIBR_PIPELINE_NAME=%q\n' "$PIPELINE_NAME"
    printf 'KALIBR_RUN_ID=%q\n' "$run_id"
    printf 'KALIBR_BAG_PATH=%q\n' "$bag"
    printf 'KALIBR_DATA_ROOT=%q\n' "$DATA_ROOT"
    printf 'KALIBR_TARGET=%q\n' "$TARGET"
    printf 'KALIBR_RECORD_TOPICS=%q\n' "$RECORD_TOPICS"
    printf 'KALIBR_REQUIRED_RECORD_TOPICS=%q\n' "$REQUIRED_RECORD_TOPICS"
    printf 'KALIBR_RECORD_STARTED_AT=%q\n' "$(date +%Y%m%d_%H%M%S)"
  } > "$manifest"
  echo "$manifest"
}

kalibr_write_calibration_manifest() {
  local out="$1"
  local bag="$2"
  local manifest="$out/calibration_manifest.env"
  {
    printf 'KALIBR_PIPELINE_NAME=%q\n' "$PIPELINE_NAME"
    printf 'KALIBR_BAG_PATH=%q\n' "$bag"
    printf 'KALIBR_OUTPUT_DIR=%q\n' "$out"
    printf 'KALIBR_TARGET=%q\n' "$TARGET"
    printf 'KALIBR_TOPICS=%q\n' "${KALIBR_TOPICS:-$TOPICS}"
    printf 'KALIBR_MODELS=%q\n' "${KALIBR_MODELS:-$MODELS}"
    printf 'KALIBR_BAG_FREQ=%q\n' "${KALIBR_BAG_FREQ:-$DEFAULT_BAG_FREQ}"
    printf 'KALIBR_APPROX_SYNC=%q\n' "${KALIBR_APPROX_SYNC:-$DEFAULT_APPROX_SYNC}"
    printf 'KALIBR_CALIBRATED_AT=%q\n' "$(date +%Y%m%d_%H%M%S)"
  } > "$manifest"
}

kalibr_check_bag_topics() {
  local bag="$1"
  shift
  local info topic
  info=$(rosbag info "$bag")
  for topic in "$@"; do
    if ! grep -Eq "^[[:space:]]*$topic[[:space:]]+" <<< "$info"; then
      echo "Bag does not contain required topic: $topic" >&2
      return 1
    fi
  done
}

kalibr_check_live_topics() {
  local topic
  for topic in "$@"; do
    if ! rostopic list 2>/dev/null | grep -Fxq "$topic"; then
      echo "Missing required live topic: $topic" >&2
      return 1
    fi
  done
}
