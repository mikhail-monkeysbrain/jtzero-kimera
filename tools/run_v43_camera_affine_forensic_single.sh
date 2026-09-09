#!/usr/bin/env bash
set -euo pipefail

ROOT="${HOME}/jtzero-kimera-sync"
RUNS="${HOME}/jtzero_runs"
PARAMS="${ROOT}/params/JTZeroMonoFLU_TBS_Rm1d5_Pm5d5_ARW_003"
SRC="${ROOT}/tools/live_mono_imu_500mm_v43_camera_forensic.cpp"
BIN="/tmp/live_mono_imu_500mm_v43_camera_forensic"
STAMP=$(date +%Y%m%d_%H%M%S)
OUT="${RUNS}/${STAMP}_v43_CAMERA_AFFINE_FORENSIC"

# TF-Luna measures 180 mm in the current static setup.
# Camera sensor plane was physically estimated near 185 mm, therefore +5 mm.
# Override explicitly if a more precise physical measurement is made.
export JTZERO_V41_CAMERA_HEIGHT_OFFSET_M="${JTZERO_V43_CAMERA_HEIGHT_OFFSET_M:-0.005}"
export JTZERO_V25_PARAMS="${PARAMS}"
export JTZERO_V26_TARGET_SEC=7.5
export JTZERO_V26_TOLERANCE_SEC=1.0
export JTZERO_GRAVITY_ALIGNED_IMU_INIT=1
export JTZERO_DIAG_IMU_INIT=1
export JTZERO_STAGED_ZUPT=1
export JTZERO_V25_SOURCE="${SRC}"
unset JTZERO_V25_EXTRA_CXXFLAGS

cd "${ROOT}"

echo "======================================================================"
echo "V43 — ОДИН КОНТРОЛЬНЫЙ ПРОХОД A→B / 500 мм"
echo "ДИАГНОСТИКА CAMERA-ONLY: AFFINE INTERNALS"
echo "ФИЗИЧЕСКИХ ПРОХОДОВ: РОВНО 1"
echo
echo "TF-LUNA DIRECT = дальномер читается напрямую Raspberry Pi через /dev/ttyAMA2."
echo "CAMERA-ONLY = отдельная оценка перемещения по камере."
echo "KIMERA = итоговая визуально-инерциальная оценка по камере + IMU."
echo
echo "ТЕКУЩАЯ ГЕОМЕТРИЯ:"
echo "  TF-Luna static: ~180 мм"
echo "  camera offset:  ${JTZERO_V41_CAMERA_HEIGHT_OFFSET_M} м"
echo
echo "СЦЕНАРИЙ:"
echo "1) Стенд на A. Не двигать."
echo "2) Дождаться STARTUP/WARM-UP."
echo "3) Убедиться, что V43-PREFLIGHT показывает валидную TF-Luna."
echo "4) SPACE/ENTER на A -> плавно двигать к B."
echo "5) Дойти до B примерно за 7.5 с."
echo "6) На упоре B нажать SPACE/ENTER."
echo "7) Дождаться '1AB СОХРАНЁН'."
echo "8) Q/ESC — закрыть GUI."
echo
echo "ЦЕЛЬ: записать scale/rotation/tx/ty и центры inlier-точек на одном проходе."
echo "======================================================================"

# Build only if the binary does not exist or relevant source changed.
if [[ ! -x "${BIN}" || "${SRC}" -nt "${BIN}" ||       "${ROOT}/tools/v25_parts/live_mono_imu_500mm_repeat_hud_v25_part01.inc" -nt "${BIN}" ||       "${ROOT}/tools/v18_parts/live_mono_imu_500mm_repeat_hud_v18_part07b.inc" -nt "${BIN}" ||       "${ROOT}/tools/v25_parts/live_mono_imu_500mm_repeat_hud_v25_part08b.inc" -nt "${BIN}" ]]; then
  echo "[V43] Исходники изменились — собираю бинарник ОДИН РАЗ..."
  bash "${ROOT}/tools/build_v25.sh" "${BIN}"
else
  echo "[V43] Использую уже собранный бинарник: ${BIN}"
fi

export JTZERO_V25_BIN="${BIN}"
rm -f "${HOME}"/jtzero_500mm_v26_[1-4]AB.csv
rm -f "${HOME}"/jtzero_500mm_v25_{legs,backend,frontend,events,camera,attitude,range}.csv
rm -f "${HOME}/jtzero_500mm_v25.csv"
rm -f "${HOME}/jtzero_v43_camera_forensic.csv"

read -r -p "ENTER — запустить единственный V43 проход: " _
set +e
bash "${ROOT}/tools/run_v25_auto_camera.sh" CONTROLLED_AB4
RC=$?
set -e

A=$(bash "${ROOT}/tools/archive_v25_run.sh" "V43_CAMERA_AFFINE_FORENSIC" "${PARAMS}")
mkdir -p "${OUT}"
cp -a "${A}/." "${OUT}/"
if [[ -f "${HOME}/jtzero_v43_camera_forensic.csv" ]]; then
  cp "${HOME}/jtzero_v43_camera_forensic.csv" "${OUT}/"
  V43_ROWS=$(awk 'END{print (NR>0?NR-1:0)}' "${HOME}/jtzero_v43_camera_forensic.csv")
  echo "[V43] forensic rows archived: ${V43_ROWS}"
else
  V43_ROWS=0
  echo "[V43] ERROR: forensic CSV missing"
fi
{
  echo "test=v43_camera_forensic"
  echo "physical_measured_passes=1"
  echo "truth_m=0.500"
  echo "range_source=/dev/ttyAMA2@115200"
  echo "camera_height_offset_m=${JTZERO_V41_CAMERA_HEIGHT_OFFSET_M}"
  echo "runner_exit_code=${RC}"
} > "${OUT}/V43_METADATA.txt"

echo
echo "======================================================================"
echo "V43 ЗАВЕРШЁН — 1 ИЗ 1."
echo "БОЛЬШЕ ФИЗИЧЕСКИХ ПРОХОДОВ НЕ НУЖНО."
echo "Archive: ${OUT}"
echo "Пришли полный терминальный вывод."
echo "======================================================================"
