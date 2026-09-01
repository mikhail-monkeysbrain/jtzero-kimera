#!/usr/bin/env bash
set -euo pipefail

# Low-load launcher for the integrated 500 mm HUD.
# The first HUD build redraws a 1280x900 OpenCV window as fast as the 120 FPS
# capture loop allows, which can starve Kimera backend threads over TigerVNC.
# This launcher gives the process real-time-friendly CPU affinity: Kimera and
# capture may use all cores, while X/TigerVNC load is reduced by asking Qt/X11
# not to synchronize redraws to the display.

export LD_LIBRARY_PATH="/home/vio/Kimera-VIO/build:/usr/local/lib:${LD_LIBRARY_PATH:-}"
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_SOFTWARE=1

exec /tmp/live_mono_imu_500mm_hud "${1:-params/JTZeroMono}"
