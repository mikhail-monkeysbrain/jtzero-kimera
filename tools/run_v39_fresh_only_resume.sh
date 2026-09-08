#!/usr/bin/env bash
set -u

ROOT="${HOME}/jtzero-kimera-sync"
RUNS="${HOME}/jtzero_runs"
PARAMS="${ROOT}/params/JTZeroMonoFLU_TBS_Rm1d5_Pm5d5_ARW_003"
BIN="/tmp/live_mono_imu_500mm_v39_clean_ab"

if [[ $# -ge 1 ]]; then
    OUT="$1"
else
    OUT=$(find "${RUNS}" -maxdepth 1 -type d -name '*_v39_CLEAN_RESTART_CARRYOVER' -printf '%T@ %p\n' 2>/dev/null | sort -nr | awk 'NR==1{$1=""; sub(/^ /,""); print; exit}')
fi

if [[ -z "${OUT:-}" || ! -d "${OUT}/continuous" ]]; then
    echo "[V39-RESUME] ERROR: не найден существующий V39 archive с continuous/"
    echo "Использование:"
    echo "  bash tools/run_v39_fresh_only_resume.sh /home/vio/jtzero_runs/<V39_ARCHIVE>"
    exit 2
fi

if [[ ! -x "${BIN}" ]]; then
    echo "[V39-RESUME] ERROR: исходный бинарник V39 отсутствует: ${BIN}"
    echo "Чтобы эксперимент оставался чистым, этот resume-скрипт НЕ пересобирает бинарник."
    exit 3
fi

export JTZERO_V26_TARGET_SEC=7.5
export JTZERO_V26_TOLERANCE_SEC=1.0
export JTZERO_V25_PARAMS="${PARAMS}"
export JTZERO_GRAVITY_ALIGNED_IMU_INIT=1
export JTZERO_DIAG_IMU_INIT="${JTZERO_DIAG_IMU_INIT:-1}"
export JTZERO_STAGED_ZUPT="${JTZERO_STAGED_ZUPT:-1}"
export JTZERO_V25_EXTRA_CXXFLAGS="-DJTZERO_LEG_COUNT=4"
export JTZERO_V25_BIN="${BIN}"

cd "${ROOT}" || exit 1

clean_outputs() {
    rm -f "${HOME}"/jtzero_500mm_v26_[1-4]AB.csv
    rm -f "${HOME}"/jtzero_500mm_v25_{legs,backend,frontend,events,camera,attitude,range}.csv
    rm -f "${HOME}/jtzero_500mm_v25.csv"
}

echo "======================================================================"
echo "V39 RESUME — FRESH ONLY"
echo "CONTINUOUS уже сохранён и НЕ будет повторяться."
echo "Бинарник НЕ пересобирается: ${BIN}"
echo "Архив серии: ${OUT}"
echo
echo "FRESH = отдельный новый процесс VIO перед каждым измеряемым A->B."
echo "VIO = visual-inertial odometry (визуально-инерциальная одометрия)."
echo "SPACE используется только для START на A и END на B."
echo "После сохранения первого прохода закрыть GUI вручную Q/ESC."
echo "======================================================================"

for N in 1 2 3 4; do
    DEST="${OUT}/fresh_${N}"
    if [[ -f "${DEST}/V39_METADATA.txt" ]]; then
        echo
        echo "[V39-RESUME] FRESH ${N}/4 уже существует: ${DEST}"
        read -r -p "ENTER — пропустить его; R + ENTER — перезаписать: " ANSWER
        if [[ "${ANSWER}" != "R" && "${ANSWER}" != "r" ]]; then
            continue
        fi
        rm -rf "${DEST}"
    fi

    echo
    echo "######################################################################"
    echo "SERIES B / FRESH ${N}/4"
    echo "1) Стенд на A, полностью неподвижен."
    echo "2) ENTER запускает новый процесс VIO."
    echo "3) Дождаться startup/warm-up (стартовая инициализация и стабилизация)."
    echo "4) SPACE на A -> двигать A->B примерно за 7.5 s -> SPACE на B."
    echo "5) Дождаться строки LEG 1 ... и сохранения прохода."
    echo "6) Закрыть GUI ВРУЧНУЮ Q/ESC. Автоматического завершения нет."
    echo "7) После закрытия процесса вернуть стенд B->A без SPACE."
    echo "######################################################################"
    read -r -p "ENTER — запустить FRESH ${N}/4: " _

    clean_outputs
    set +e
    bash "${ROOT}/tools/run_v25_auto_camera.sh" CONTROLLED_AB4
    RC=$?
    set -e 2>/dev/null || true

    A=$(bash "${ROOT}/tools/archive_v25_run.sh" "V39_FRESH_${N}_A_TO_B" "${PARAMS}")
    mkdir -p "${DEST}"
    cp -a "${A}/." "${DEST}/"
    {
        echo "series=fresh"
        echo "fresh_index=${N}"
        echo "process_restart_before_measured_pass=yes"
        echo "used_pass=1"
        echo "measured_direction=A_TO_B"
        echo "target_duration_s=7.5"
        echo "params=${PARAMS}"
        echo "binary=${BIN}"
        echo "binary_rebuilt_for_this_pass=no"
        echo "resumed_existing_v39=yes"
        echo "runner_exit_code=${RC}"
    } > "${DEST}/V39_METADATA.txt"

    echo "[V39-RESUME] FRESH ${N} saved: ${DEST}"

    if [[ "${N}" -lt 4 ]]; then
        echo "Вернуть стенд B->A при закрытом процессе."
        read -r -p "ENTER после возврата на A и полной остановки: " _
    fi
done

echo
echo "======================================================================"
echo "V39 FRESH COMPLETE"
echo "Archive root: ${OUT}"
echo "Continuous preserved: ${OUT}/continuous"
echo "Fresh results: ${OUT}/fresh_1 ... fresh_4"
echo "Common binary (тот же бинарник): ${BIN}"
echo "======================================================================"
