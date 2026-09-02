#!/usr/bin/env bash
set -euo pipefail

KIMERA_ROOT="${1:-/home/vio/Kimera-VIO}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC_BASE="$KIMERA_ROOT/params/EurocMono"
DST="$REPO_ROOT/params/JTZeroMono"

if [[ ! -d "$SRC_BASE" ]]; then
  echo "ERROR: Kimera EurocMono params not found: $SRC_BASE" >&2
  exit 2
fi

mkdir -p "$DST/flags"

for f in BackendParams.yaml DisplayParams.yaml FrontendParams.yaml ImuParams.yaml LcdParams.yaml PipelineParams.yaml RightCameraParams.yaml; do
  cp "$SRC_BASE/$f" "$DST/$f"
done

if [[ -d "$SRC_BASE/flags" ]]; then
  cp -a "$SRC_BASE/flags/." "$DST/flags/"
fi

# LeftCameraParams.yaml is project-owned and MUST NOT be overwritten.
if [[ ! -f "$DST/LeftCameraParams.yaml" ]]; then
  echo "ERROR: missing project camera params: $DST/LeftCameraParams.yaml" >&2
  exit 2
fi

# Live operation has no EuRoC ground-truth initial state. Kimera's EurocMono
# template uses autoInitialize: 0, which explicitly requests GT initialization
# and aborts when the initial GT state is identity. For live JT-Zero use IMU
# initialization instead. Kimera assumes the platform is motionless during the
# initialization window.
sed -i 's/^autoInitialize:[[:space:]]*0$/autoInitialize: 1/' "$DST/BackendParams.yaml"

# The JT-Zero runtime already supplies camera and IMU timestamps in one
# CLOCK_MONOTONIC domain. Disable Kimera's additional IMU-rate alignment to
# avoid applying a second timing correction.
sed -i 's/^do_imu_rate_time_alignment:[[:space:]]*1$/do_imu_rate_time_alignment: 0/' "$DST/ImuParams.yaml"

cat <<EOF
Prepared JT-Zero live Kimera params:
  $DST

Camera:
  OV9281 640x480 @ 30 Hz
  accepted Stage 10 intrinsics
  accepted Stage 11 T_BC

IMU:
  200 Hz
  T_BS identity because HIGHRES_IMU is already expressed in body FRD
  Kimera internal IMU-rate time alignment disabled

Backend initialization:
  autoInitialize: 1 (initialize from IMU, not EuRoC ground truth)
  IMPORTANT: keep the stand motionless during startup/initialization

NOTE:
  Noise-density/random-walk values are still inherited from EurocMono and are
  provisional for the first live integration. Do not use first-run accuracy as
  final performance evidence until Matek/ArduPilot IMU noise is characterized.
EOF
