#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

KIMERA_ROOT="${KIMERA_ROOT:-/home/vio/Kimera-VIO}"
CAMERA="${JTZERO_FLOW_CAMERA:-/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC762-video-index0}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="/home/vio/jtzero_runs/${STAMP}_OF_CAP_DECISION"
REC=/tmp/jtzero_of_cap_decision_record

MAVLINK_INC=""
for d in "$KIMERA_ROOT/third_party/mavlink" "$KIMERA_ROOT/third_party/mavlink/include/mavlink/v2.0" "/usr/local/include/mavlink/v2.0"; do
  if [[ -f "$d/common/mavlink.h" && -f "$d/ardupilotmega/mavlink.h" ]]; then
    MAVLINK_INC="-I$d"
    break
  fi
done
[[ -n "$MAVLINK_INC" ]] || { echo "ОШИБКА: MAVLink headers не найдены" >&2; exit 2; }
[[ -e "$CAMERA" ]] || { echo "ОШИБКА: камера не найдена: $CAMERA" >&2; exit 2; }

cat <<EOF
======================================================================
JT-ZERO — РЕШАЮЩИЙ ТЕСТ FEATURE CAP 500 / 300 / 200
======================================================================
Это НЕ ещё одна ветка диагностики.
Цель — выбрать рабочий cap и закончить этот вопрос.

FC и TF-Luna не нужны.
Будет записано 15 секунд видео OV9281, затем ОДНИ И ТЕ ЖЕ тяжёлые пары
будут replay-нуты для разных feature cap.

ПРОТОКОЛ:
  1) 2 с — аппарат спокойно.
  2) 8-10 с — двигай аппарат заметно энергичнее обычного:
     переносы, изменение высоты, roll/pitch/yaw.
     Сделай несколько быстрых, но контролируемых движений.
     Камеру рукой не закрывать.
  3) последние 3 с — спокойно.

Маршрут не повторять. Точка A/B не нужна.
Dataset: $OUT

КРИТЕРИЙ РЕШЕНИЯ:
  - если cap=200 на ТОЧНО тех же тяжёлых парах почти не теряет valid
    относительно 500 и flow/displacement совпадает — выбираем 200;
  - если 200 теряет заметно больше, а 300 нет — выбираем 300;
  - если оба заметно хуже 500 — оставляем 500.

После этого не продолжаем подбор cap без нового противоречащего факта.
======================================================================
EOF

echo "Собираю recorder..."
g++ -std=c++17 -O2 -DNDEBUG -pthread -Wno-address-of-packed-member   $(pkg-config --cflags opencv4) $MAVLINK_INC   "$ROOT/tools/optical_flow_feature_replay_record.cpp"   -o "$REC" $(pkg-config --libs opencv4) -lpthread

mkdir -p "$OUT"
"$REC" "$CAMERA" "$OUT" 15

echo
echo "Запускаю анализ тяжёлых пар..."
bash "$ROOT/tools/run_optical_flow_heavy_pair_feature_replay.sh" "$OUT" 25

echo
echo "Dataset сохранён:"
echo "  $OUT"
