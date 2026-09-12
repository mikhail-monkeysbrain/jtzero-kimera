#!/usr/bin/env bash
set -euo pipefail

ROOT="${HOME}/jtzero_runs"
REPO="${HOME}/jtzero-kimera-sync"
OUTROOT="${ROOT}/collected"

mkdir -p "${OUTROOT}"

latest_run="$(
  find "${ROOT}" -maxdepth 1 -mindepth 1 -type d \
    -name '*OPTICAL_FLOW*' -printf '%T@ %p\n' 2>/dev/null \
    | sort -nr | head -n1 | cut -d' ' -f2-
)"

if [[ -z "${latest_run:-}" || ! -d "${latest_run}" ]]; then
  echo "ОШИБКА: не найден каталог запуска *OPTICAL_FLOW* в ${ROOT}" >&2
  exit 1
fi

stamp="$(date +%Y%m%d_%H%M%S)"
bundle="${OUTROOT}/${stamp}_optical_flow_continuous_bundle"
mkdir -p "${bundle}"

echo "Найден последний запуск:"
echo "  ${latest_run}"
echo
echo "Собираю данные в:"
echo "  ${bundle}"

{
  echo "===== JT-ZERO CONTINUOUS OPTICAL FLOW LOG BUNDLE ====="
  echo "collected_at=$(date --iso-8601=seconds)"
  echo "hostname=$(hostname)"
  echo "user=$(whoami)"
  echo "run_dir=${latest_run}"
  echo

  echo "===== SYSTEM ====="
  uname -a || true
  echo

  echo "===== REPOSITORY ====="
  if [[ -d "${REPO}/.git" || -f "${REPO}/.git" ]]; then
    git -C "${REPO}" status --short --branch || true
    echo
    git -C "${REPO}" log -n 8 --oneline --decorate || true
    echo
    echo "HEAD=$(git -C "${REPO}" rev-parse HEAD 2>/dev/null || true)"
  else
    echo "Repository not found: ${REPO}"
  fi
  echo

  echo "===== IMPORTANT ENV / PARAMS ====="
  echo "Expected focal scale: JTZERO_FLOW_FOCAL_SCALE=1.1060"
  echo "Expected EK3_FLOW_DELAY=0"
  echo

  echo "===== RUN DIRECTORY TREE ====="
  find "${latest_run}" -maxdepth 3 -type f -printf '%P\t%s bytes\n' | sort || true
  echo

  echo "===== DISK USAGE ====="
  du -sh "${latest_run}" || true
} > "${bundle}/summary.txt" 2>&1

# Copy all small/medium textual diagnostics and metadata.
while IFS= read -r -d '' f; do
  rel="${f#"${latest_run}/"}"
  dest="${bundle}/run/${rel}"
  mkdir -p "$(dirname "${dest}")"
  cp -a "${f}" "${dest}"
done < <(
  find "${latest_run}" -type f \
    \( -iname '*.csv' -o -iname '*.json' -o -iname '*.txt' -o -iname '*.log' \
       -o -iname '*.yaml' -o -iname '*.yml' -o -iname '*.param' -o -iname '*.params' \
       -o -iname '*.md' -o -iname '*.bin' \) -print0
)

# Copy likely DataFlash/MAVLink/raw artifacts even when extension differs,
# but avoid huge video/image recordings that are not needed for EKF forensic.
while IFS= read -r -d '' f; do
  rel="${f#"${latest_run}/"}"
  dest="${bundle}/run/${rel}"
  [[ -e "${dest}" ]] && continue
  case "$(basename "${f}")" in
    *DataFlash*|*dataflash*|*mavlink*|*optical_flow*|*range*|*ekf*|*xkf*|*flow*)
      mkdir -p "$(dirname "${dest}")"
      cp -a "${f}" "${dest}"
      ;;
  esac
done < <(find "${latest_run}" -type f -print0)

# Copy the newest continuous-series SESSION JSON (stored in RUNS_ROOT, not inside the run dir).
latest_session="$(
  find "${ROOT}" -maxdepth 1 -type f -name '*_OPTICAL_FLOW_CONTINUOUS_SERIES.json' \
    -printf '%T@ %p\n' 2>/dev/null | sort -nr | head -n1 | cut -d' ' -f2-
)"
if [[ -n "${latest_session:-}" && -f "${latest_session}" ]]; then
  cp -a "${latest_session}" "${bundle}/run/"
  echo "session_json=${latest_session}" >> "${bundle}/summary.txt"
fi

# Save current source/scripts relevant to this test for exact reproducibility.
mkdir -p "${bundle}/source"
for f in \
  tools/optical_flow_mavlink_mvp_v2.cpp \
  tools/run_optical_flow_mavlink_bench.sh \
  tools/optical_flow_continuous_reciprocal_gui.py \
  tools/run_optical_flow_continuous_reciprocal_gui.sh
do
  if [[ -f "${REPO}/${f}" ]]; then
    mkdir -p "${bundle}/source/$(dirname "${f}")"
    cp -a "${REPO}/${f}" "${bundle}/source/${f}"
  fi
done

# Create a compact inventory with hashes.
(
  cd "${bundle}"
  find . -type f ! -name SHA256SUMS.txt -print0 \
    | sort -z \
    | xargs -0 sha256sum
) > "${bundle}/SHA256SUMS.txt"

archive="${bundle}.tar.gz"
tar -C "$(dirname "${bundle}")" -czf "${archive}" "$(basename "${bundle}")"

echo
echo "ГОТОВО"
echo "Каталог: ${bundle}"
echo "Архив:  ${archive}"
echo
ls -lh "${archive}"
echo
echo "Пришли мне этот файл:"
echo "${archive}"
