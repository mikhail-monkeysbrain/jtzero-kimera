#!/usr/bin/env bash
set -u

ROOT="${HOME}/jtzero-kimera-sync"
RUNS="${HOME}/jtzero_runs"
PARAMS="${ROOT}/params/JTZeroMonoFLU_TBS_Rm1d5_Pm5d5_ARW_003"
BIN="/tmp/live_mono_imu_500mm_v39_clean_ab"
STAMP=$(date +%Y%m%d_%H%M%S)
OUT="${RUNS}/${STAMP}_v39_CLEAN_RESTART_CARRYOVER"
mkdir -p "${OUT}"

export JTZERO_V26_TARGET_SEC=7.5
export JTZERO_V26_TOLERANCE_SEC=1.0
export JTZERO_V25_PARAMS="${PARAMS}"
export JTZERO_GRAVITY_ALIGNED_IMU_INIT=1
export JTZERO_DIAG_IMU_INIT="${JTZERO_DIAG_IMU_INIT:-1}"
export JTZERO_STAGED_ZUPT="${JTZERO_STAGED_ZUPT:-1}"

cd "${ROOT}" || exit 1

# V39 must be invariant to caller shell state. Previous one-pass experiments may
# leave JTZERO_V25_EXTRA_CXXFLAGS=-DJTZERO_LEG_COUNT=1 in the environment.
# Force this test to compile exactly four measured passes. The same binary is
# then reused for CONTINUOUS and all FRESH runs; FRESH is closed manually after pass 1.
unset JTZERO_V25_EXTRA_CXXFLAGS
export JTZERO_V25_EXTRA_CXXFLAGS="-DJTZERO_LEG_COUNT=4"

echo "======================================================================"
echo "V39 CLEAN RESTART / CARRY-OVER TEST"
echo
echo "ОБЩИЙ ПЛАН: 2 ЭТАПА / 8 ИЗМЕРЯЕМЫХ A->B"
echo "ЭТАП 1 ИЗ 2: CONTINUOUS — 4 прохода A->B в одном процессе VIO."
echo "ЭТАП 2 ИЗ 2: FRESH — ещё 4 прохода A->B, каждый в новом процессе VIO."
echo "ВАЖНО: CONTINUOUS 4/4 = НЕ КОНЕЦ ТЕСТА."
echo "После него ОБЯЗАТЕЛЬНО будет ЭТАП 2 ИЗ 2 (FRESH 1/4..4/4)."
echo "ТЕСТ ЗАВЕРШЁН только после FRESH 4/4."
echo
echo "Один бинарник и один каталог параметров для всей серии."
echo "Все ИЗМЕРЯЕМЫЕ проходы: только A -> B."
echo "Возврат B -> A: БЕЗ SPACE, он не измеряется."
echo
echo "SERIES A — CONTINUOUS:"
echo "  4 измеряемых A -> B в одном процессе VIO."
echo "  Между ними вернуть стенд B -> A БЕЗ SPACE."
echo
echo "SERIES B — FRESH:"
echo "  4 измеряемых A -> B."
echo "  Перед каждым A -> B запускается новый процесс VIO."
echo "  После сохранения первого прохода закрыть GUI вручную Q/ESC."
echo
echo "SPACE используется ТОЛЬКО:"
echo "  1) на A для START;"
echo "  2) на B для END."
echo "======================================================================"

echo
echo "[V39] Building ONE common binary..."
echo "[V39] compile invariant: JTZERO_LEG_COUNT=4 (four measured passes)"
bash "${ROOT}/tools/build_v25.sh" "${BIN}"
if [[ ! -x "${BIN}" ]]; then
    echo "[V39] ERROR: binary not created: ${BIN}"
    exit 2
fi
export JTZERO_V25_BIN="${BIN}"

clean_outputs() {
    rm -f "${HOME}"/jtzero_500mm_v26_[1-4]AB.csv
    rm -f "${HOME}"/jtzero_500mm_v25_{legs,backend,frontend,events,camera,attitude,range}.csv
    rm -f "${HOME}/jtzero_500mm_v25.csv"
}

echo
echo "######################################################################"
echo "ЭТАП 1 ИЗ 2 — SERIES A / CONTINUOUS"
echo "СЕЙЧАС: 4 прохода. ПОТОМ: ЭТАП 2 ИЗ 2 — ещё 4 FRESH."
echo "НЕ ЗАКРЫВАЙ ТЕСТ ПОСЛЕ CONTINUOUS 4/4: это только половина эксперимента."
echo "1) Стенд на A, неподвижен."
echo "2) Дождаться завершения startup/warm-up."
echo "3) SPACE на A -> двигать A->B -> SPACE на B."
echo "4) После сохранения: вернуть B->A БЕЗ SPACE."
echo "5) На A снова SPACE. Повторить до четырёх измеряемых A->B."
echo "6) После 4-го сохранения закрыть GUI самостоятельно Q/ESC."
echo "######################################################################"
read -r -p "ENTER — запустить CONTINUOUS 4x A->B: " _

clean_outputs
set +e
bash "${ROOT}/tools/run_v25_auto_camera.sh" CONTROLLED_AB4
CONT_RC=$?
set -e 2>/dev/null || true

CONT_ARCHIVE=$(bash "${ROOT}/tools/archive_v25_run.sh" "V39_CONTINUOUS_4X_A_TO_B" "${PARAMS}")
mkdir -p "${OUT}/continuous"
cp -a "${CONT_ARCHIVE}/." "${OUT}/continuous/"
{
    echo "series=continuous"
    echo "process_restart_between_measured_passes=no"
    echo "measured_direction=A_TO_B"
    echo "unmeasured_return=B_TO_A"
    echo "target_duration_s=7.5"
    echo "params=${PARAMS}"
    echo "binary=${BIN}"
    echo "runner_exit_code=${CONT_RC}"
} > "${OUT}/continuous/V39_METADATA.txt"

echo
echo "[V39] CONTINUOUS saved: ${OUT}/continuous"
echo
echo "======================================================================"
echo "ЭТАП 1 ИЗ 2 ЗАВЕРШЁН. ЭКСПЕРИМЕНТ ЕЩЁ НЕ ЗАВЕРШЁН."
echo "ДАЛЬШЕ: ЭТАП 2 ИЗ 2 — FRESH 1/4, 2/4, 3/4, 4/4."
echo "Только после FRESH 4/4 будет полный конец V39."
echo "======================================================================"
echo

for N in 1 2 3 4; do
    echo
    echo "######################################################################"
    echo "ЭТАП 2 ИЗ 2 — SERIES B / FRESH ${N}/4"
    echo "ОБЩИЙ ПРОГРЕСС: выполнено $((4 + N - 1))/8, сейчас измерение $((4 + N))/8."
    if [[ "${N}" -eq 4 ]]; then
        echo "ПОСЛЕ ЭТОГО ПРОХОДА: ТЕСТ V39 ЗАВЕРШЁН."
    else
        echo "ПОСЛЕ ЭТОГО ПРОХОДА: будет FRESH $((N + 1))/4."
    fi
    echo "1) Стенд на A, неподвижен."
    echo "2) Новый процесс VIO запустится сейчас."
    echo "3) Дождаться завершения startup/warm-up."
    echo "4) SPACE на A -> двигать A->B -> SPACE на B."
    echo "5) Дождаться сообщения, что проход 1 сохранён."
    echo "6) Закрыть GUI САМОСТОЯТЕЛЬНО Q/ESC."
    echo "7) Возврат B->A выполнить после закрытия процесса."
    echo "######################################################################"
    read -r -p "ENTER — запустить FRESH ${N}/4: " _

    clean_outputs
    set +e
    bash "${ROOT}/tools/run_v25_auto_camera.sh" CONTROLLED_AB4
    RC=$?
    set -e 2>/dev/null || true

    A=$(bash "${ROOT}/tools/archive_v25_run.sh" "V39_FRESH_${N}_A_TO_B" "${PARAMS}")
    DEST="${OUT}/fresh_${N}"
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
        echo "runner_exit_code=${RC}"
    } > "${DEST}/V39_METADATA.txt"

    echo "[V39] FRESH ${N} saved: ${DEST}"

    if [[ "${N}" -lt 4 ]]; then
        echo "Теперь вернуть стенд B->A при закрытом процессе."
        read -r -p "ENTER после возврата на A и полной остановки: " _
    fi
done

echo
echo "======================================================================"
echo "V39 COMPLETE"
echo "Archive root: ${OUT}"
echo "Common binary: ${BIN}"
echo "Common params: ${PARAMS}"
echo "Next analysis must compare:"
echo "  continuous passes 1..4 (all A->B)"
echo "  fresh_1..fresh_4 pass 1 (all A->B)"
echo "======================================================================"
