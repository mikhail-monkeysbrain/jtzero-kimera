#!/usr/bin/env bash
set -u

ROOT="${HOME}/jtzero-kimera-sync"
RUNS="${HOME}/jtzero_runs"
PARAMS="${ROOT}/params/JTZeroMonoFLU_TBS_Rm1d5_Pm5d5_ARW_003"
BIN="/tmp/live_mono_imu_500mm_freshstart_v34"
STAMP=$(date +%Y%m%d_%H%M%S)
OUT="${RUNS}/${STAMP}_v34_FRESHSTART_4X"
mkdir -p "${OUT}"

echo "======================================================================"
echo "V34 FRESH-START 4x — SINGLE BUILD"
echo "Цель: проверить влияние переноса accelerometer bias."
echo
echo "Каждый подпроход:"
echo "  * новый процесс Kimera/VIO;"
echo "  * один измеряемый A->B;"
echo "  * GUI с видео + таймлайном 7.5 с;"
echo "  * SPACE = START / END;"
echo "  * после сохранения GUI остаётся открыт до Q/ESC;"
echo "  * возврат B->A выполняется только при закрытом процессе."
echo
echo "Бинарник собирается ОДИН РАЗ перед всей серией."
echo "======================================================================"
echo

cd "${ROOT}" || exit 1

export JTZERO_V26_TARGET_SEC=7.5
export JTZERO_V26_TOLERANCE_SEC=1.0
export JTZERO_V25_PARAMS="${PARAMS}"
export JTZERO_GRAVITY_ALIGNED_IMU_INIT=1
export JTZERO_DIAG_IMU_INIT="${JTZERO_DIAG_IMU_INIT:-1}"
export JTZERO_STAGED_ZUPT="${JTZERO_STAGED_ZUPT:-1}"

echo "[V34] Building single-pass GUI binary ONCE..."
export JTZERO_V25_EXTRA_CXXFLAGS="-DJTZERO_LEG_COUNT=1 -DJTZERO_SINGLE_LEG_MODE"
bash "${ROOT}/tools/build_v25.sh" "${BIN}"
unset JTZERO_V25_EXTRA_CXXFLAGS

if [[ ! -x "${BIN}" ]]; then
    echo "[V34] ERROR: binary was not created: ${BIN}"
    exit 2
fi

echo "[V34] build complete: ${BIN}"
echo

for N in 1 2 3 4; do
    echo
    echo "######################################################################"
    echo "FRESH START ${N}/4"
    echo "1) Стенд точно на A и полностью неподвижен."
    echo "2) Во время startup + 12 s warm-up НЕ ДВИГАТЬ."
    echo "3) Когда GUI разрешит старт: SPACE и сразу начинай A->B."
    echo "4) Цель — 7.5 с; на упоре B нажми SPACE."
    echo "5) Дождись сообщения 1AB СОХРАНЁН."
    echo "6) Закрой GUI САМ: Q или ESC."
    echo "######################################################################"
    echo
    read -r -p "ENTER — запустить fresh-start ${N}/4: " _

    rm -f "${HOME}"/jtzero_500mm_v26_[1-4]AB.csv
    rm -f "${HOME}"/jtzero_500mm_v25_{legs,backend,frontend,events,camera,attitude,range}.csv
    rm -f "${HOME}/jtzero_500mm_v25.csv"

    export JTZERO_V25_BIN="${BIN}"

    set +e
    bash "${ROOT}/tools/run_v25_auto_camera.sh" CONTROLLED_AB4
    RC=$?
    set -e 2>/dev/null || true

    LABEL="V34_FRESH_${N}_SINGLEPASS_7P5"
    ARCHIVE=$(bash "${ROOT}/tools/archive_v25_run.sh" "${LABEL}" "${PARAMS}")

    DEST="${OUT}/fresh_${N}"
    mkdir -p "${DEST}"
    cp -a "${ARCHIVE}/." "${DEST}/"

    {
        echo "fresh_index=${N}"
        echo "source_archive=${ARCHIVE}"
        echo "runner_exit_code=${RC}"
        echo "used_pass=1"
        echo "direction=A_TO_B"
        echo "target_duration_s=7.5"
        echo "binary=${BIN}"
        echo "binary_rebuilt_inside_loop=no"
        echo "process_restart_between_passes=yes"
    } > "${DEST}/V34_METADATA.txt"

    echo
    echo "[V34] fresh-start ${N} saved:"
    echo "      ${DEST}"

    if [[ "${N}" -lt 4 ]]; then
        echo
        echo "Процесс уже закрыт. Теперь вручную верни стенд B->A."
        read -r -p "ENTER после возврата на A и полной остановки: " _
    fi
done

echo
echo "======================================================================"
echo "V34 COLLECTION COMPLETE"
echo "Archive: ${OUT}"
echo "Binary was built once: ${BIN}"
echo "Use PASS 1 from fresh_1..fresh_4."
echo "======================================================================"
