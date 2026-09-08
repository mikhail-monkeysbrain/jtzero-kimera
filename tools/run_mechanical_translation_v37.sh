#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="/tmp/live_mechanical_translation_fc_accel_v37"
SRC="$ROOT/tools/live_mechanical_translation_fc_accel_v37.cpp"

echo "============================================================"
echo "JT-ZERO: МЕХАНИЧЕСКИЙ ЭТАЛОННЫЙ ТЕСТ ПЕРЕМЕЩЕНИЯ v37"
echo "YAW ДРОНА/СТЕНДА: примерно -90°"
echo "Kimera estimator в измерении не используется."
echo "Снимайте ОДНО непрерывное боковое видео на телефон. В кадре должны быть:"
echo "  1) метка/крест, жёстко закреплённая рядом с FC/IMU"
echo "  2) жёсткая верхняя площадка стенда"
echo "  3) неподвижный ориентир на фоне"
echo "Последовательность: покой A -> A->B 500 мм -> покой B -> B->A 500 мм -> покой A"
echo "============================================================"

export JTZERO_V25_SOURCE="$SRC"
bash "$ROOT/tools/build_v25.sh" "$BIN"

rm -f /home/vio/jtzero_mechanical_translation_v37.csv

LD_LIBRARY_PATH=/home/vio/Kimera-VIO/build:/usr/local/lib:${LD_LIBRARY_PATH:-} "$BIN"

echo
echo "Результат CSV:"
ls -lh /home/vio/jtzero_mechanical_translation_v37.csv
echo
echo "Загрузите ОБА файла:"
echo "  /home/vio/jtzero_mechanical_translation_v37.csv"
echo "  непрерывное боковое видео с телефона"
