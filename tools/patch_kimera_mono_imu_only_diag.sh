#!/usr/bin/env bash
set -euo pipefail

TARGET="/home/vio/Kimera-VIO/src/pipeline/MonoImuPipeline.cpp"
BACKUP="${TARGET}.jtzero_before_imu_only_diag"
MODE="${1:-apply}"

if [[ ! -f "$TARGET" ]]; then
  echo "ERROR: missing Kimera source: $TARGET" >&2
  exit 2
fi

if [[ "$MODE" == "restore" ]]; then
  if [[ ! -f "$BACKUP" ]]; then
    echo "ERROR: backup not found: $BACKUP" >&2
    exit 2
  fi
  cp "$BACKUP" "$TARGET"
  echo "Restored original Kimera source: $TARGET"
  exit 0
fi

if [[ "$MODE" != "apply" ]]; then
  echo "Usage: bash tools/patch_kimera_mono_imu_only_diag.sh [apply|restore]" >&2
  exit 2
fi

if grep -q "JTZERO_DIAG_IMU_ONLY" "$TARGET"; then
  echo "Patch already present in $TARGET"
  exit 0
fi

cp -n "$TARGET" "$BACKUP"

perl -0pi -e 's/#include <string>/#include <cstdlib>\n#include <string>/s' "$TARGET"

perl -0pi -e 's/if \(converted_output->is_keyframe_\) \{\n          \/\/! Only push to Backend input queue if it is a keyframe!\n          backend_input_queue\.push\(std::make_unique<BackendInput>\(\n              converted_output->frame_lkf_\.timestamp_,\n              converted_output->status_mono_measurements_,/if (converted_output->is_keyframe_) {\n          \/\/ JT-ZERO diagnostic: when JTZERO_DIAG_IMU_ONLY is set, keep the\n          \/\/ frontend and IMU preintegration running but remove monocular smart\n          \/\/ landmark measurements before they enter the backend. This isolates\n          \/\/ whether visual smart factors create rotation->translation coupling.\n          auto jtzero_diag_measurements =\n              converted_output->status_mono_measurements_;\n          if (std::getenv("JTZERO_DIAG_IMU_ONLY") &&\n              converted_output->status_mono_measurements_) {\n            jtzero_diag_measurements =\n                std::make_shared<StatusMonoMeasurements>(\n                    *converted_output->status_mono_measurements_);\n            jtzero_diag_measurements->second.clear();\n          }\n\n          \/\/! Only push to Backend input queue if it is a keyframe!\n          backend_input_queue.push(std::make_unique<BackendInput>(\n              converted_output->frame_lkf_.timestamp_,\n              jtzero_diag_measurements,/s' "$TARGET"

if ! grep -q "JTZERO_DIAG_IMU_ONLY" "$TARGET"; then
  echo "ERROR: patch pattern did not match; source left from backup at: $BACKUP" >&2
  cp "$BACKUP" "$TARGET"
  exit 3
fi

echo "Applied JT-ZERO IMU-only diagnostic patch to:"
echo "  $TARGET"
echo
echo "Diagnostic behavior:"
echo "  normal launch: visual smart measurements unchanged"
echo "  JTZERO_DIAG_IMU_ONLY=1: smart mono measurements cleared before backend"
echo
echo "Backup:"
echo "  $BACKUP"
echo
echo "Restore with:"
echo "  bash tools/patch_kimera_mono_imu_only_diag.sh restore"
