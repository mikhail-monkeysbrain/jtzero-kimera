#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
KIMERA_ROOT="${KIMERA_ROOT:-/home/vio/Kimera-VIO}"
PARAMS="$ROOT/params/JTZeroMonoFLU"
SERIAL="${JTZERO_CLEAN_SERIAL:-/dev/ttyAMA0}"
BIN="/tmp/jtzero_clean01_standalone"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="$HOME/jtzero_runs/${STAMP}_CLEAN01_STANDALONE"

find_camera() {
  local d real card
  if [[ -d /dev/v4l/by-id ]]; then
    for d in /dev/v4l/by-id/*Arducam*video-index0 /dev/v4l/by-id/*OV9281*video-index0; do
      [[ -e "$d" ]] || continue
      real="$(readlink -f "$d")"
      if v4l2-ctl -d "$real" --list-formats-ext 2>/dev/null | grep -q "'MJPG'"; then
        printf '%s\n' "$real"; return 0
      fi
    done
  fi
  for d in /dev/video*; do
    [[ -e "$d" ]] || continue
    card="$(v4l2-ctl -d "$d" --all 2>/dev/null | sed -n 's/^[[:space:]]*Card type[[:space:]]*:[[:space:]]*//p' | head -1)"
    case "$card" in
      *Arducam*|*OV9281*)
        if v4l2-ctl -d "$d" --list-formats-ext 2>/dev/null | grep -q "'MJPG'"; then
          printf '%s\n' "$d"; return 0
        fi ;;
    esac
  done
  return 1
}

CAMERA="$(find_camera || true)"
[[ -n "$CAMERA" ]] || { echo "ОШИБКА: OV9281 MJPEG не найдена" >&2; exit 2; }
[[ -e "$SERIAL" ]] || { echo "ОШИБКА: serial $SERIAL не существует" >&2; exit 2; }

mkdir -p "$OUT"

# Explicitly remove experiment-only switches from the environment.
unset JTZERO_MONO_POSE_GATE
unset JTZERO_MONO_POSE_GATE_JUMP_DEG
unset JTZERO_MONO_POSE_GATE_TILT_DEG
unset JTZERO_MONO_POSE_GATE_STATS_FILE
unset JTZERO_GRAVITY_ALIGNED_IMU_INIT
unset JTZERO_DIAG_IMU_INIT
unset JTZERO_STAGED_ZUPT
unset JTZERO_V41_CAMERA_HEIGHT_OFFSET_M
unset JTZERO_V43_CAMERA_HEIGHT_OFFSET_M
unset JTZERO_V26_TARGET_SEC
unset JTZERO_V26_TOLERANCE_SEC

export LD_LIBRARY_PATH="$KIMERA_ROOT/build:/usr/local/lib:${LD_LIBRARY_PATH:-}"

if [[ ! -x "$BIN" || "$ROOT/tools/clean01_standalone.cpp" -nt "$BIN" ]]; then
  bash "$ROOT/tools/build_clean01_standalone.sh" "$BIN"
fi

{
  echo "test=CLEAN01_STANDALONE"
  echo "timestamp=$STAMP"
  echo "truth_distance_mm=500"
  echo "truth_distance_uncertainty_mm=operator_not_entered"
  echo "camera_height_mm=not_used_by_vio"
  echo "gate_enabled=0"
  echo "experimental_imu_init=0"
  echo "staged_zupt=0"
  echo "camera_device=$CAMERA"
  echo "serial_device=$SERIAL"
  echo "params_dir=$PARAMS"
  echo "jtzero_head=$(git -C "$ROOT" rev-parse HEAD)"
  echo "kimera_head=$(git -C "$KIMERA_ROOT" rev-parse HEAD)"
  echo "kimera_diff_sha256=$(git -C "$KIMERA_ROOT" diff | sha256sum | awk '{print $1}')"
  echo "binary_sha256=$(sha256sum "$BIN" | awk '{print $1}')"
} > "$OUT/MANIFEST.txt"

sha256sum   "$PARAMS/LeftCameraParams.yaml"   "$PARAMS/ImuParams.yaml"   "$PARAMS/FrontendParams.yaml"   "$PARAMS/PipelineParams.yaml"   > "$OUT/PARAM_HASHES.txt"

git -C "$KIMERA_ROOT" status --short > "$OUT/KIMERA_STATUS.txt"
git -C "$KIMERA_ROOT" diff > "$OUT/KIMERA_LOCAL_DIFF.patch"
cp "$KIMERA_ROOT/src/pipeline/MonoImuPipeline.cpp" "$OUT/MonoImuPipeline.cpp.snapshot"
cp "$ROOT/tools/clean01_standalone.cpp" "$OUT/clean01_standalone.cpp.snapshot"

set +e
"$BIN" "$PARAMS" "$CAMERA" "$SERIAL" "$OUT" 2>&1 | tee "$OUT/terminal.log"
RC=${PIPESTATUS[0]}
set -e

{
  echo "runner_exit_code=$RC"
  echo "archive=$OUT"
} >> "$OUT/MANIFEST.txt"

(cd "$OUT" && find . -maxdepth 1 -type f ! -name SHA256SUMS.txt -print0 | sort -z | xargs -0 sha256sum) > "$OUT/SHA256SUMS.txt"

echo
echo "======================================================================"
echo "CLEAN-01 STANDALONE ЗАВЕРШЁН"
echo "Архив: $OUT"
echo "Код завершения: $RC"
echo "======================================================================"
exit "$RC"
