#!/usr/bin/env bash
set -u

ROOT="${HOME}/jtzero-kimera-sync"
RUNS="${HOME}/jtzero_runs"
STAMP=$(date +%Y%m%d_%H%M%S)
OUT="${RUNS}/${STAMP}_v34_FRESHSTART_4X"
mkdir -p "${OUT}"

echo "======================================================================"
echo "V34 FRESH-START 4x"
echo "Цель: проверить влияние переноса оценки постоянной ошибки акселерометра."
echo
echo "ВАЖНО:"
echo "  * каждый подпроход — НОВЫЙ процесс VIO;"
echo "  * выполняй ТОЛЬКО первый A->B;"
echo "  * цель движения: 7.5 с;"
echo "  * после завершения PASS 1 дождись стабилизации;"
echo "  * затем ЗАКРОЙ GUI САМ (Q/ESC согласно текущему GUI);"
echo "  * программа НЕ должна закрываться автоматически;"
echo "  * возврат B->A делай только ПОСЛЕ закрытия текущего процесса."
echo "======================================================================"
echo

cd "${ROOT}" || exit 1

# Используем тот же runner и те же параметры, но каждый раз начинаем с полностью
# нового процесса Kimera. Нам нужен только PASS 1 каждого запуска.
export JTZERO_V26_TARGET_S=7.5
export JTZERO_V26_MIN_S=6.5
export JTZERO_V26_MAX_S=8.5

for N in 1 2 3 4; do
    echo
    echo "######################################################################"
    echo "FRESH START ${N}/4"
    echo "1) Стенд должен быть в точке A."
    echo "2) Не двигай стенд во время startup/warmup."
    echo "3) Выполни только PASS 1 A->B примерно за 7.5 с."
    echo "4) После фиксации результата PASS 1 закрой GUI САМ."
    echo "######################################################################"
    echo
    read -r -p "Нажми ENTER, когда готов начать fresh-start ${N}/4: " _

    BEFORE=$(find "${RUNS}" -maxdepth 1 -type d -name '*CONTROLLED_AB4*' -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -1 | cut -d' ' -f2-)

    set +e
    bash tools/run_v26_controlled_ab4_archived.sh
    RC=$?
    set -e 2>/dev/null || true

    AFTER=$(find "${RUNS}" -maxdepth 1 -type d -name '*CONTROLLED_AB4*' -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -1 | cut -d' ' -f2-)

    if [[ -z "${AFTER}" || "${AFTER}" == "${BEFORE}" ]]; then
        echo "[V34] Не найден новый архив после fresh-start ${N}."
        echo "[V34] Ничего не удаляется. Проверь вывод runner."
        exit 2
    fi

    DEST="${OUT}/fresh_${N}"
    mkdir -p "${DEST}"
    cp -a "${AFTER}/." "${DEST}/"

    {
        echo "fresh_index=${N}"
        echo "source_archive=${AFTER}"
        echo "runner_exit_code=${RC}"
        echo "expected_used_pass=1"
        echo "expected_direction=A_TO_B"
        echo "target_duration_s=7.5"
        echo "note=Only PASS 1 is valid for V34 comparison; later movement, if any, must be ignored."
    } > "${DEST}/V34_METADATA.txt"

    echo
    echo "[V34] fresh-start ${N} сохранён:"
    echo "      ${DEST}"
    echo
    if [[ "${N}" -lt 4 ]]; then
        echo "Теперь вручную верни стенд B->A при ЗАКРЫТОМ процессе."
        read -r -p "Нажми ENTER после возврата в A и полной остановки: " _
    fi
done

echo
echo "======================================================================"
echo "V34 COLLECTION COMPLETE"
echo "Архив: ${OUT}"
echo
echo "Для анализа использовать ТОЛЬКО PASS 1 из fresh_1..fresh_4."
echo "======================================================================"
