#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
KIMERA_ROOT="${KIMERA_ROOT:-/home/vio/Kimera-VIO}"
PARAMS="$ROOT/params/JTZeroMonoFLU"
BIN="/tmp/jtzero_clean01"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="$HOME/jtzero_runs/${STAMP}_CLEAN01_500MM"

find_camera() {
  local d real card
  if [[ -d /dev/v4l/by-id ]]; then
    for d in /dev/v4l/by-id/*Arducam*video-index0 /dev/v4l/by-id/*OV9281*video-index0; do
      [[ -e "$d" ]] || continue
      real="$(readlink -f "$d")"
      if v4l2-ctl -d "$real" --list-formats-ext 2>/dev/null | grep -q "'MJPG'"; then
        printf '%s\n' "$real"
        return 0
      fi
    done
  fi
  for d in /dev/video*; do
    [[ -e "$d" ]] || continue
    card="$(v4l2-ctl -d "$d" --all 2>/dev/null | sed -n 's/^[[:space:]]*Card type[[:space:]]*:[[:space:]]*//p' | head -1)"
    case "$card" in
      *Arducam*OV9281*|*OV9281*USB*|*Arducam*)
        if v4l2-ctl -d "$d" --list-formats-ext 2>/dev/null | grep -q "'MJPG'"; then
          printf '%s\n' "$d"; return 0
        fi ;;
    esac
  done
  return 1
}

mkdir -p "$OUT"

# CLEAN-01 deliberately disables all opt-in legacy diagnostics/gates.
unset JTZERO_MONO_POSE_GATE
unset JTZERO_MONO_POSE_GATE_JUMP_DEG
unset JTZERO_MONO_POSE_GATE_TILT_DEG
unset JTZERO_MONO_POSE_GATE_STATS_FILE
unset JTZERO_V41_CAMERA_HEIGHT_OFFSET_M
unset JTZERO_V43_CAMERA_HEIGHT_OFFSET_M
unset JTZERO_V25_SOURCE
unset JTZERO_V25_BIN
unset JTZERO_V25_PARAMS
unset JTZERO_V25_EXTRA_CXXFLAGS

unset JTZERO_GRAVITY_ALIGNED_IMU_INIT
unset JTZERO_DIAG_IMU_INIT
unset JTZERO_STAGED_ZUPT
unset JTZERO_V26_TARGET_SEC
unset JTZERO_V26_TOLERANCE_SEC
export LD_LIBRARY_PATH="$KIMERA_ROOT/build:/usr/local/lib:${LD_LIBRARY_PATH:-}"

CAMERA="$(find_camera || true)"
if [[ -z "$CAMERA" ]]; then
  echo "ОШИБКА: поток изображения OV9281 не найден." >&2
  v4l2-ctl --list-devices >&2 || true
  exit 2
fi

echo "======================================================================"
echo "JT-ZERO CLEAN-01 — ПРЕДПОЛЁТНАЯ ПРОВЕРКА"
echo "======================================================================"
echo "Это новый архив. Старые результаты и коэффициенты не читаются."
echo "Gate: ВЫКЛЮЧЕН"
echo "Экспериментальная инициализация IMU: ВЫКЛЮЧЕНА"
echo "Staged ZUPT: ВЫКЛЮЧЕН"
echo "Legacy camera-only: НЕ СОБИРАЕТСЯ"
echo "Параметры: $PARAMS"
echo "Камера: $CAMERA"
echo "Архив: $OUT"
echo

if [[ ! -x "$BIN" || "$ROOT/tools/live_mono_imu_500mm_clean01.cpp" -nt "$BIN" ]]; then
  bash "$ROOT/tools/build_clean01.sh" "$BIN"
fi

{
  echo "test=CLEAN01_500MM"
  echo "timestamp=$STAMP"
  echo "truth_distance_mm=500"
  echo "truth_distance_uncertainty_mm=UNSET_OPERATOR_INPUT"
  echo "camera_height_mm=UNSET_OPERATOR_INPUT"
  echo "camera_height_uncertainty_mm=UNSET_OPERATOR_INPUT"
  echo "gate_enabled=0"
  echo "gravity_aligned_imu_init=0"
  echo "diag_imu_init=0"
  echo "staged_zupt=0"
  echo "legacy_camera_only=0"
  echo "camera_device=$CAMERA"
  echo "params_dir=$PARAMS"
  echo "jtzero_head=$(git -C "$ROOT" rev-parse HEAD)"
  echo "jtzero_status_begin"
  git -C "$ROOT" status --short
  echo "jtzero_status_end"
  echo "kimera_head=$(git -C "$KIMERA_ROOT" rev-parse HEAD)"
  echo "kimera_status_begin"
  git -C "$KIMERA_ROOT" status --short
  echo "kimera_status_end"
  echo "kimera_diff_sha256=$(git -C "$KIMERA_ROOT" diff | sha256sum | awk '{print $1}')"
  echo "binary_sha256=$(sha256sum "$BIN" | awk '{print $1}')"
} > "$OUT/MANIFEST.txt"

sha256sum   "$PARAMS/LeftCameraParams.yaml"   "$PARAMS/ImuParams.yaml"   "$PARAMS/FrontendParams.yaml"   "$PARAMS/PipelineParams.yaml"   > "$OUT/PARAM_HASHES.txt"

git -C "$KIMERA_ROOT" diff > "$OUT/KIMERA_LOCAL_DIFF.patch"
cp "$KIMERA_ROOT/src/pipeline/MonoImuPipeline.cpp" "$OUT/MonoImuPipeline.cpp.snapshot"

rm -f   "$HOME/jtzero_clean01_legs.csv"   "$HOME/jtzero_clean01_imu.csv"   "$HOME/jtzero_clean01_camera.csv"   "$HOME/jtzero_clean01_selected.mjpg"   "$HOME/jtzero_clean01_selected_frames.csv"   "$HOME/jtzero_clean01_attitude.csv"   "$HOME/jtzero_clean01_range.csv"   "$HOME/jtzero_clean01_backend.csv"   "$HOME/jtzero_clean01_frontend.csv"   "$HOME/jtzero_clean01_events.csv"   "$HOME/jtzero_clean01_vio_trace.csv"

set +e
"$BIN" "$PARAMS" "$CAMERA" 2>&1 | tee "$OUT/terminal.log"
RC=${PIPESTATUS[0]}
set -e

for f in   jtzero_clean01_legs.csv   jtzero_clean01_imu.csv   jtzero_clean01_camera.csv   jtzero_clean01_selected.mjpg   jtzero_clean01_selected_frames.csv   jtzero_clean01_attitude.csv   jtzero_clean01_range.csv   jtzero_clean01_backend.csv   jtzero_clean01_frontend.csv   jtzero_clean01_events.csv   jtzero_clean01_vio_trace.csv
do
  [[ -e "$HOME/$f" ]] && mv "$HOME/$f" "$OUT/"
done

{
  echo "runner_exit_code=$RC"
  echo "archive=$OUT"
  echo "files_begin"
  find "$OUT" -maxdepth 1 -type f -printf '%f\n' | sort
  echo "files_end"
} >> "$OUT/MANIFEST.txt"

(cd "$OUT" && find . -maxdepth 1 -type f ! -name SHA256SUMS.txt -print0 | sort -z | xargs -0 sha256sum) > "$OUT/SHA256SUMS.txt"

echo
echo "======================================================================"
echo "CLEAN-01 ЗАВЕРШЁН"
echo "Архив: $OUT"
echo "Код завершения: $RC"
echo "======================================================================"
exit "$RC"
