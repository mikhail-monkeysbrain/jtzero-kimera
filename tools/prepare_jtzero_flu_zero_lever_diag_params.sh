#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$REPO_ROOT/params/JTZeroMonoFLU"
DST="$REPO_ROOT/params/JTZeroMonoFLUZeroLever"

if [[ ! -d "$SRC" ]]; then
  echo "ERROR: missing FLU diagnostic params: $SRC" >&2
  echo "Run: bash tools/prepare_jtzero_flu_diag_params.sh" >&2
  exit 2
fi

if [[ ! -f "$DST/LeftCameraParams.yaml" ]]; then
  echo "ERROR: missing project-owned zero-lever camera params: $DST/LeftCameraParams.yaml" >&2
  exit 2
fi

mkdir -p "$DST/flags"

for f in BackendParams.yaml DisplayParams.yaml FrontendParams.yaml ImuParams.yaml LcdParams.yaml PipelineParams.yaml RightCameraParams.yaml; do
  if [[ ! -f "$SRC/$f" ]]; then
    echo "ERROR: missing source param: $SRC/$f" >&2
    exit 2
  fi
  cp "$SRC/$f" "$DST/$f"
done

if [[ -d "$SRC/flags" ]]; then
  cp -a "$SRC/flags/." "$DST/flags/"
fi

cat <<EOF
Prepared isolated FLU zero-lever diagnostic params:
  $DST

A/B invariant:
  IMU transform: FRD -> FLU, unchanged
  camera rotation R_BC: unchanged from JTZeroMonoFLU
  camera translation t_BC: ONLY changed value, [-0.055 m in Z] -> [0,0,0]
  timing/intrinsics/backend/IMU params: unchanged

This is diagnostic only. Do not promote zero lever arm to final calibration without evidence.
EOF
