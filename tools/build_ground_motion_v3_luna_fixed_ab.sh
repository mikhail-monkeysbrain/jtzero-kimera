#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
MAV_INC=""
for d in /home/vio/Kimera-VIO/Thirdparty/mavlink /home/vio/mavlink/include /usr/local/include; do
  if [ -f "$d/mavlink/v2.0/common/mavlink.h" ]; then MAV_INC="$d"; break; fi
  if [ -f "$d/common/mavlink.h" ]; then MAV_INC="$d/.."; break; fi
done
if [ -z "$MAV_INC" ]; then
  MAV_INC=$(dirname "$(find /home/vio -path '*/mavlink/v2.0/common/mavlink.h' -print -quit 2>/dev/null || true)")
  MAV_INC=${MAV_INC%/mavlink/v2.0/common}
fi
if [ -z "$MAV_INC" ]; then echo "ОШИБКА: MAVLink headers не найдены" >&2; exit 1; fi

g++ -std=c++17 -O2 -pthread tools/ground_motion_live_v3_luna_fixed_ab.cpp \
  -o /tmp/jtzero_ground_motion_v3_luna_fixed_ab \
  -I"$MAV_INC" $(pkg-config --cflags --libs opencv4)
echo "ГОТОВО: /tmp/jtzero_ground_motion_v3_luna_fixed_ab"
