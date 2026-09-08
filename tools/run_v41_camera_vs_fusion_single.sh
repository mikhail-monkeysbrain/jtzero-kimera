#!/usr/bin/env bash
set -euo pipefail

ROOT="${HOME}/jtzero-kimera-sync"
RUNS="${HOME}/jtzero_runs"
PARAMS="${ROOT}/params/JTZeroMonoFLU_TBS_Rm1d5_Pm5d5_ARW_003"
BIN="/tmp/live_mono_imu_500mm_v41_camera_vs_fusion"
STAMP=$(date +%Y%m%d_%H%M%S)
OUT="${RUNS}/${STAMP}_v41_CAMERA_VS_FUSION_SINGLE"

export JTZERO_V25_PARAMS="${PARAMS}"
export JTZERO_V26_TARGET_SEC=7.5
export JTZERO_V26_TOLERANCE_SEC=1.0
export JTZERO_GRAVITY_ALIGNED_IMU_INIT=1
export JTZERO_DIAG_IMU_INIT=1
export JTZERO_STAGED_ZUPT=1
export JTZERO_V41_CAMERA_HEIGHT_OFFSET_M="${JTZERO_V41_CAMERA_HEIGHT_OFFSET_M:--0.020}"
export JTZERO_V25_SOURCE="${ROOT}/tools/live_mono_imu_500mm_v41_camera_vs_fusion.cpp"
unset JTZERO_V25_EXTRA_CXXFLAGS

cd "${ROOT}"

echo "======================================================================"
echo "V41 — ОДИН ДИАГНОСТИЧЕСКИЙ ПРОХОД A→B / 500 мм"
echo "ОБЩИЙ ПРОГРЕСС: ШАГ 3 ИЗ 5"
echo "ФИЗИЧЕСКИХ ИЗМЕРЕНИЙ В ЭТОМ ШАГЕ: 1"
echo
echo "CAMERA-ONLY = независимая метрическая оценка по изображению + TF-Luna."
echo "BACKEND/FUSED = итоговая оценка Kimera после объединения камеры и IMU."
echo "PIM = внутреннее прединтегрированное IMU-измерение; НЕ трактуется как полный путь в мм."
echo
echo "СЦЕНАРИЙ:"
echo "1) Стенд на упоре A, полностью неподвижен."
echo "2) Дождаться окончания STARTUP/WARM-UP."
echo "3) SPACE на A -> плавно двигать к B."
echo "   Если SPACE не срабатывает из-за фокуса окна: ЛЕВЫЙ КЛИК по GUI = SPACE."
echo "4) Попасть к B примерно за 7.5 с."
echo "5) На физическом упоре B нажать SPACE и остановить стенд."
echo "   Резерв: ЛЕВЫЙ КЛИК по GUI = SPACE."
echo "6) Дождаться сообщения 1AB СОХРАНЁН."
echo "7) Q/ESC — закрыть GUI самостоятельно."
echo "ПОСЛЕ ЭТОГО НИКАКИХ ДОПОЛНИТЕЛЬНЫХ A→B НЕ БУДЕТ."
echo "======================================================================"

if [[ ! -x "${BIN}" || "${ROOT}/tools/live_mono_imu_500mm_v41_camera_vs_fusion.cpp" -nt "${BIN}" || "${ROOT}/tools/v25_parts/live_mono_imu_500mm_repeat_hud_v25_part01.inc" -nt "${BIN}" || "${ROOT}/tools/v18_parts/live_mono_imu_500mm_repeat_hud_v18_part07b.inc" -nt "${BIN}" || "${ROOT}/tools/v18_parts/live_mono_imu_500mm_repeat_hud_v18_part08a.inc" -nt "${BIN}" || "${ROOT}/tools/v25_parts/live_mono_imu_500mm_repeat_hud_v25_part08b.inc" -nt "${BIN}" ]]; then
  echo "[V41] Сборка диагностического бинарника (один раз после изменения исходников)..."
  bash "${ROOT}/tools/build_v25.sh" "${BIN}"
else
  echo "[V41] Использую уже собранный бинарник: ${BIN}"
fi

export JTZERO_V25_BIN="${BIN}"
rm -f "${HOME}"/jtzero_500mm_v26_[1-4]AB.csv
rm -f "${HOME}"/jtzero_500mm_v25_{legs,backend,frontend,events,camera,attitude,range}.csv
rm -f "${HOME}/jtzero_500mm_v25.csv"

read -r -p "ENTER — запустить единственный A→B проход: " _
set +e
bash "${ROOT}/tools/run_v25_auto_camera.sh" CONTROLLED_AB4
RC=$?
set -e

A=$(bash "${ROOT}/tools/archive_v25_run.sh" "V41_CAMERA_VS_FUSION_SINGLE" "${PARAMS}")
mkdir -p "${OUT}"
cp -a "${A}/." "${OUT}/"
{
  echo "test=v41_camera_vs_fusion_single"
  echo "physical_measured_passes=1"
  echo "direction=A_TO_B"
  echo "truth_m=0.500"
  echo "camera_height_offset_m=${JTZERO_V41_CAMERA_HEIGHT_OFFSET_M}"
  echo "params=${PARAMS}"
  echo "binary=${BIN}"
  echo "runner_exit_code=${RC}"
} > "${OUT}/V41_METADATA.txt"

echo
echo "======================================================================"
echo "V41 ФИЗИЧЕСКАЯ ЧАСТЬ ЗАВЕРШЕНА — 1 ИЗ 1."
echo "БОЛЬШЕ ПРОХОДОВ НЕ НУЖНО."
echo "Archive: ${OUT}"
echo "Пришли полный терминальный вывод; следующий шаг — анализ этого ОДНОГО прохода."
echo "======================================================================"
