#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="/tmp/jtzero_ground_motion_live"
CAMERA="${JTZERO_GM_CAMERA:-/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC762-video-index0}"
LUNA="${JTZERO_GM_LUNA:-/dev/ttyAMA2}"
FC="${JTZERO_GM_FC:-/dev/ttyAMA0}"
PARAMS="${JTZERO_GM_CAMERA_YAML:-$ROOT/params/JTZeroMonoFLU/LeftCameraParams.yaml}"
OFFSET="${JTZERO_GM_CAMERA_OFFSET_MM:-0}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="${JTZERO_GM_OUT:-$HOME/jtzero_runs/${STAMP}_GROUND_MOTION_LIVE.csv}"
REFRESH="${JTZERO_GM_3D_REFRESH_MS:-250}"

mkdir -p "$(dirname "$OUT")"

if [[ ! -x "$BIN" || "$ROOT/tools/ground_motion_live.cpp" -nt "$BIN" ]]; then
  bash "$ROOT/tools/build_ground_motion_live.sh" "$BIN"
fi

EST_PID=""
VIS_PID=""

cleanup() {
  trap - INT TERM EXIT
  if [[ -n "${VIS_PID:-}" ]] && kill -0 "$VIS_PID" 2>/dev/null; then
    kill "$VIS_PID" 2>/dev/null || true
    sleep 0.2
    kill -KILL "$VIS_PID" 2>/dev/null || true
  fi
  if [[ -n "${EST_PID:-}" ]] && kill -0 "$EST_PID" 2>/dev/null; then
    kill "$EST_PID" 2>/dev/null || true
    sleep 0.3
    kill -KILL "$EST_PID" 2>/dev/null || true
  fi
}
trap cleanup INT TERM EXIT

echo "======================================================================"
echo "JT-ZERO — GROUND MOTION + 3D"
echo "CSV: $OUT"
echo "======================================================================"

"$BIN" "$CAMERA" "$LUNA" "$FC" "$OUT" "$PARAMS" "$OFFSET" &
EST_PID=$!

# Wait until the estimator creates the current-run CSV.
for _ in $(seq 1 100); do
  [[ -s "$OUT" ]] && break
  if ! kill -0 "$EST_PID" 2>/dev/null; then
    wait "$EST_PID" || true
    echo "ОШИБКА: Ground Motion завершился до создания CSV"
    exit 1
  fi
  sleep 0.1
done

if [[ ! -s "$OUT" ]]; then
  echo "ОШИБКА: CSV не появился: $OUT"
  exit 1
fi

python3 "$ROOT/tools/visualize_ground_motion_3d.py"   "$OUT"   --live   --refresh-ms "$REFRESH" &
VIS_PID=$!

echo "Ground Motion PID: $EST_PID"
echo "3D PID:            $VIS_PID"
echo "Закрытие любого из двух окон завершит весь запуск."
echo "Ctrl+C в этом терминале также завершит оба процесса."

set +e
wait -n "$EST_PID" "$VIS_PID"
RC=$?
set -e

echo
echo "Один из процессов завершился. Закрываю второй."
echo "CSV сохранён: $OUT"
exit "$RC"
