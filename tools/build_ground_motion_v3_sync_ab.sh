#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
KIMERA_ROOT="${KIMERA_ROOT:-/home/vio/Kimera-VIO}"
OUT="${1:-/tmp/jtzero_ground_motion_v3_sync_ab}"
MAVLINK_INC=""
for d in "$KIMERA_ROOT/third_party/mavlink" "$KIMERA_ROOT/third_party/mavlink/include/mavlink/v2.0" "/usr/local/include/mavlink/v2.0"; do
  if [[ -f "$d/common/mavlink.h" ]]; then MAVLINK_INC="-I$d"; break; fi
done
[[ -n "$MAVLINK_INC" ]] || { echo "ОШИБКА: common/mavlink.h не найден"; exit 2; }
g++ -std=c++17 -O2 -DNDEBUG -pthread \
  $(pkg-config --cflags opencv4) \
  $MAVLINK_INC \
  "$ROOT/tools/ground_motion_live_v3_sync_ab.cpp" \
  -o "$OUT" \
  $(pkg-config --libs opencv4) -lpthread
echo "ГОТОВО: $OUT"
