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

# First live integration uses the already validated 200 Hz stream and
# pre-synchronized timestamps from the JT-Zero runtime. Kimera's own rate
# alignment must therefore be disabled to avoid a second timing correction.
python3 - "$DST/ImuParams.yaml" <<'PY'
from pathlib import Path
import sys
p = Path(sys.argv[1])
s = p.read_text()
s = s.replace('do_imu_rate_time_alignment: 1', 'do_imu_rate_time_alignment: 0')
s = s.replace('imu_time_shift: 0.0', 'imu_time_shift: 0.0')
p.write_text(s)
PY

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

NOTE:
  Noise-density/random-walk values are still inherited from EurocMono and are
  provisional for the first live integration. Do not use first-run accuracy as
  final performance evidence until Matek/ArduPilot IMU noise is characterized.
EOF
