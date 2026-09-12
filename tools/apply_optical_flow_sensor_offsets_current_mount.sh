#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

KIMERA_ROOT="${KIMERA_ROOT:-/home/vio/Kimera-VIO}"
DEVICE="${JTZERO_FLOW_FC:-/dev/ttyAMA0}"
BAUD="${JTZERO_FLOW_FC_BAUD:-460800}"

CAM_X="${JTZERO_CAM_POS_X:-0.0625}"
CAM_Y="${JTZERO_CAM_POS_Y:-0.0000}"
CAM_Z="${JTZERO_CAM_POS_Z:-0.0500}"
LUNA_X="${JTZERO_LUNA_POS_X:--0.1300}"
LUNA_Y="${JTZERO_LUNA_POS_Y:-0.0000}"
LUNA_Z="${JTZERO_LUNA_POS_Z:-0.0260}"

TMP_BIN="/tmp/jtzero_fc_param_batch_checked"

MAVLINK_INC=""
for d in "$KIMERA_ROOT/third_party/mavlink" "$KIMERA_ROOT/third_party/mavlink/include/mavlink/v2.0" "/usr/local/include/mavlink/v2.0"; do
  if [[ -f "$d/common/mavlink.h" && -f "$d/ardupilotmega/mavlink.h" ]]; then MAVLINK_INC="-I$d"; break; fi
done
[[ -n "$MAVLINK_INC" ]] || { echo "ОШИБКА: MAVLink headers не найдены"; exit 2; }

g++ -std=c++17 -O2 -DNDEBUG -Wno-address-of-packed-member \
  $MAVLINK_INC "$ROOT/tools/fc_param_batch_checked.cpp" -o "$TMP_BIN"

cat <<EOF
======================================================================
JT-ZERO — APPLY OPTICAL FLOW / RANGEFINDER SENSOR OFFSETS
======================================================================
BODY FRD: +X forward, +Y right, +Z down

OV9281:
  FLOW_POS_X = $CAM_X
  FLOW_POS_Y = $CAM_Y
  FLOW_POS_Z = $CAM_Z

TF-Luna:
  RNGFND1_POS_X = $LUNA_X
  RNGFND1_POS_Y = $LUNA_Y
  RNGFND1_POS_Z = $LUNA_Z
======================================================================
EOF

read -r -p "Проверить INS_POS1_* и записать offsets? [Enter=yes, Ctrl+C=no] " _

exec "$TMP_BIN" "$DEVICE" "$BAUD" apply-current-mount \
  "$CAM_X" "$CAM_Y" "$CAM_Z" "$LUNA_X" "$LUNA_Y" "$LUNA_Z"
