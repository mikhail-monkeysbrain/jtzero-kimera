#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
KIMERA_ROOT="${KIMERA_ROOT:-/home/vio/Kimera-VIO}"
OUT="${1:-/tmp/jtzero_set_ekf_origin}"
MAVLINK_INC=""
for d in "$KIMERA_ROOT/third_party/mavlink" "$KIMERA_ROOT/third_party/mavlink/include/mavlink/v2.0" "/usr/local/include/mavlink/v2.0"; do
  if [[ -f "$d/common/mavlink.h" ]]; then MAVLINK_INC="-I$d"; break; fi
done
[[ -n "$MAVLINK_INC" ]] || { echo "ОШИБКА: common/mavlink.h не найден"; exit 2; }
g++ -std=c++17 -O2 -DNDEBUG -pthread \
  $MAVLINK_INC \
  "$ROOT/tools/set_ekf_origin.cpp" \
  -o "$OUT"
echo "ГОТОВО: $OUT"
