#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

COUNT="${1:-3}"
TARGET_MM="${JTZERO_FLOW_TARGET_MM:-175}"
if ! [[ "$COUNT" =~ ^[1-9][0-9]*$ ]]; then
  echo "ОШИБКА: число прогонов должно быть положительным целым"
  exit 2
fi

echo "======================================================================"
echo "JT-ZERO — REPEATABILITY $TARGET_MM мм"
echo "======================================================================"
echo "Будет выполнено $COUNT одинаковых guided-прогонов."
echo
echo "Перед началом:"
echo "  - пропеллеры сняты;"
echo "  - DISARM_DELAY=0;"
echo "  - FC ARMED;"
echo "  - аппарат всё время перемещается строго на $TARGET_MM мм без подъёма/наклона."
echo
echo "Каждый прогон сам покажет инструкцию."
echo "После каждого прогона верните аппарат в исходную точку вручную,"
echo "полностью остановите его и нажмите Enter для запуска следующего."
echo "======================================================================"
read -r -p "Готовы? [Enter] "

declare -a CSV=()
for ((i=1;i<=COUNT;i++)); do
  echo
  echo "######################################################################"
  echo "ПРОГОН $i / $COUNT"
  echo "######################################################################"
  before="$(find /home/vio/jtzero_runs -maxdepth 2 -type f -name optical_flow_mavlink.csv -printf '%T@ %p\n' 2>/dev/null | sort -nr | head -1 | cut -d' ' -f2- || true)"
  JTZERO_FLOW_GUIDED_MODE=armed-gate-open JTZERO_FLOW_TARGET_MM="$TARGET_MM" bash tools/run_optical_flow_mavlink_bench.sh
  after="$(find /home/vio/jtzero_runs -maxdepth 2 -type f -name optical_flow_mavlink.csv -printf '%T@ %p\n' | sort -nr | head -1 | cut -d' ' -f2-)"
  if [[ -z "$after" || "$after" == "$before" ]]; then
    echo "ОШИБКА: новый CSV после прогона $i не найден"
    exit 3
  fi
  CSV+=("$after")
  echo "CSV[$i]=$after"
  if (( i < COUNT )); then
    echo
    echo ">>> Верните аппарат в исходную точку. Это НЕ измеряемый участок."
    echo ">>> Полностью остановите аппарат."
    read -r -p ">>> Готовы к следующему прогону? [Enter] "
  fi
done

echo
echo "======================================================================"
echo "АВТОМАТИЧЕСКАЯ СВОДКА"
echo "======================================================================"
python3 tools/analyze_optical_flow_repeatability_175.py --target-mm "$TARGET_MM" "${CSV[@]}"
