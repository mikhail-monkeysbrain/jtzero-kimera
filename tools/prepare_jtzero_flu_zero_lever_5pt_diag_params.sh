#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$REPO_ROOT/params/JTZeroMonoFLUZeroLever"
DST="$REPO_ROOT/params/JTZeroMonoFLUZeroLever5pt"

if [[ ! -d "$SRC" ]]; then
  echo "ERROR: missing source params: $SRC" >&2
  echo "Run: bash tools/prepare_jtzero_flu_zero_lever_diag_params.sh" >&2
  exit 2
fi

if [[ ! -f "$DST/LeftCameraParams.yaml" ]]; then
  echo "ERROR: missing project-owned camera params: $DST/LeftCameraParams.yaml" >&2
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
  rm -rf "$DST/flags"
  mkdir -p "$DST/flags"
  cp -a "$SRC/flags/." "$DST/flags/"
fi

if ! grep -q '^ransac_use_2point_mono:' "$DST/FrontendParams.yaml"; then
  echo "ERROR: ransac_use_2point_mono not found in $DST/FrontendParams.yaml" >&2
  exit 2
fi

sed -i 's/^ransac_use_2point_mono:.*/ransac_use_2point_mono: 0/' "$DST/FrontendParams.yaml"

cat <<EOF
Prepared isolated FLU zero-lever 5-point diagnostic params:
  $DST

A/B invariant versus JTZeroMonoFLUZeroLever:
  IMU transform: unchanged FRD -> FLU
  camera R_BC: unchanged
  camera t_BC: unchanged zero lever
  timing/intrinsics/backend/IMU params: unchanged

ONLY frontend change:
  ransac_use_2point_mono: 1 -> 0
  Kimera will use 5-point mono RANSAC instead of IMU-aided 2-point RANSAC.
EOF

grep '^ransac_use_2point_mono:' "$DST/FrontendParams.yaml"
