#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="${JTZERO_V25_BASE_PARAMS:-$ROOT/params/JTZeroMonoFLU_TBS_Rm1d5_Pm5d5_ARW_003}"
TMP="${JTZERO_V25_ARW_TEST_DIR:-/tmp/jtzero_v25_params_arw_0003}"

rm -rf "$TMP"
cp -a "$SRC" "$TMP"

python3 - "$TMP/ImuParams.yaml" <<'PY'
from pathlib import Path
import re, sys
p=Path(sys.argv[1])
s=p.read_text()
pat=r'(?m)^accelerometer_random_walk:\s*[^#\n]+(?:\s*#.*)?$'
new='accelerometer_random_walk: 0.0003    # JT-ZERO controlled Test 1B.5: 10x tighter than baseline 0.003'
s2,n=re.subn(pat,new,s,count=1)
if n != 1:
    raise SystemExit(f"ERROR: expected one accelerometer_random_walk line, replaced {n}")
p.write_text(s2)
PY

echo "[ARW-TEST] baseline: $SRC"
echo "[ARW-TEST] params:   $TMP"
grep -nE 'accelerometer_random_walk|accelerometer_noise_density|gyroscope_random_walk' "$TMP/ImuParams.yaml"

export JTZERO_V25_PARAMS="$TMP"
export JTZERO_GRAVITY_ALIGNED_IMU_INIT="${JTZERO_GRAVITY_ALIGNED_IMU_INIT:-1}"
export JTZERO_DIAG_IMU_INIT="${JTZERO_DIAG_IMU_INIT:-1}"
export JTZERO_STAGED_ZUPT="${JTZERO_STAGED_ZUPT:-1}"

exec bash "$ROOT/tools/run_v25_auto_camera.sh" "$@"
