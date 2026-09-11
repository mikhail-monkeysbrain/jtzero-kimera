#!/usr/bin/env bash
set -euo pipefail

cd /home/vio/jtzero-kimera-sync || exit 1

# Production camera on the previous JT-Zero SD image.
# If /dev/video11 is not present, keep the stable Arducam by-id path used by
# tools/run_ground_motion_with_3d.sh.
if [[ -e /dev/video11 ]]; then
  export JTZERO_GM_CAMERA="${JTZERO_GM_CAMERA:-/dev/video11}"
fi

export JTZERO_GM_CAMERA_OFFSET_MM="${JTZERO_GM_CAMERA_OFFSET_MM:-0}"
export JTZERO_GM_3D_MODE="${JTZERO_GM_3D_MODE:-fast}"

exec bash tools/run_ground_motion_with_3d.sh
