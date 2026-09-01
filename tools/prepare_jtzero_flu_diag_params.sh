#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$REPO_ROOT/params/JTZeroMono"
DST="$REPO_ROOT/params/JTZeroMonoFLU"

if [[ ! -d "$SRC" ]]; then
  echo "ERROR: missing source params: $SRC" >&2
  exit 2
fi

if [[ ! -f "$DST/LeftCameraParams.yaml" ]]; then
  echo "ERROR: missing FLU camera params: $DST/LeftCameraParams.yaml" >&2
  exit 2
fi

mkdir -p "$DST/flags"

for f in BackendParams.yaml DisplayParams.yaml FrontendParams.yaml ImuParams.yaml LcdParams.yaml PipelineParams.yaml RightCameraParams.yaml; do
  if [[ ! -f "$SRC/$f" ]]; then
    echo "ERROR: missing generated JTZeroMono param: $SRC/$f" >&2
    echo "Run tools/prepare_jtzero_live_params.sh first." >&2
    exit 2
  fi
  cp "$SRC/$f" "$DST/$f"
done

if [[ -d "$SRC/flags" ]]; then
  rm -rf "$DST/flags"
  mkdir -p "$DST/flags"
  cp -a "$SRC/flags/." "$DST/flags/"
fi

echo "Prepared isolated FLU diagnostic params:"
echo "  $DST"
echo
echo "IMU data transform in live_mono_imu_standstill_flu.cpp:"
echo "  x'= x, y'=-y, z'=-z for accel and gyro"
echo "Camera T_BS:"
echo "  accepted FRD T_BC transformed by R_FLU_FRD=diag(1,-1,-1)"
echo "Gravity remains:"
grep '^n_gravity:' "$DST/ImuParams.yaml" || true
