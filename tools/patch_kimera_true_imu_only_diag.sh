#!/usr/bin/env bash
set -euo pipefail

TARGET="/home/vio/Kimera-VIO/src/pipeline/MonoImuPipeline.cpp"
MODE="${1:-apply}"
MARKER='JT-ZERO TRUE IMU-ONLY: disable LOW_DISPARITY backend constraints'

if [[ ! -f "$TARGET" ]]; then
  echo "ERROR: missing Kimera source: $TARGET" >&2
  exit 2
fi

if [[ "$MODE" == "status" ]]; then
  if grep -q "$MARKER" "$TARGET"; then
    echo "TRUE IMU-only patch: APPLIED"
  else
    echo "TRUE IMU-only patch: NOT APPLIED"
  fi
  exit 0
fi

if [[ "$MODE" == "restore" ]]; then
  python3 - "$TARGET" "$MARKER" <<'PY'
from pathlib import Path
import sys
p = Path(sys.argv[1])
marker = sys.argv[2]
s = p.read_text()
old = f'''            // {marker}\n            jtzero_diag_measurements->first.kfTrackingStatus_mono_ =\n                TrackingStatus::INVALID;\n'''
if old not in s:
    print("TRUE IMU-only patch already absent")
    raise SystemExit(0)
p.write_text(s.replace(old, ""))
print("Removed TRUE IMU-only LOW_DISPARITY override")
PY
  exit 0
fi

if [[ "$MODE" != "apply" ]]; then
  echo "Usage: bash tools/patch_kimera_true_imu_only_diag.sh [apply|restore|status]" >&2
  exit 2
fi

if grep -q "$MARKER" "$TARGET"; then
  echo "TRUE IMU-only patch already present"
  exit 0
fi

if ! grep -q 'jtzero_diag_measurements->second.clear();' "$TARGET"; then
  echo "ERROR: base JTZERO_DIAG_IMU_ONLY patch not found." >&2
  echo "Apply tools/patch_kimera_mono_imu_only_diag.sh first." >&2
  exit 3
fi

python3 - "$TARGET" "$MARKER" <<'PY'
from pathlib import Path
import sys
p = Path(sys.argv[1])
marker = sys.argv[2]
s = p.read_text()
needle = '''            jtzero_diag_measurements->second.clear();\n'''
insert = needle + f'''            // {marker}\n            // Clearing measurements alone is not enough: LOW_DISPARITY in\n            // TrackerStatusSummary makes VioBackend add ZeroVelocityPrior and\n            // NoMotionFactor. Force INVALID only in diagnostic mode so the\n            // backend receives IMU factors without visual/no-motion constraints.\n            jtzero_diag_measurements->first.kfTrackingStatus_mono_ =\n                TrackingStatus::INVALID;\n'''
if needle not in s:
    raise SystemExit("ERROR: insertion point not found")
p.write_text(s.replace(needle, insert, 1))
print("Applied TRUE IMU-only LOW_DISPARITY override")
PY

echo
echo "Rebuild Kimera-VIO before the next test."
echo "Normal launch remains unchanged; override is active only with JTZERO_DIAG_IMU_ONLY=1."
