#!/usr/bin/env bash
set -euo pipefail

RUNS_ROOT="${JTZERO_RUNS_ROOT:-/home/vio/jtzero_runs}"
OUT_DIR="${JTZERO_OPTLOG_PACK_DIR:-$RUNS_ROOT}"

if [[ ! -d "$RUNS_ROOT" ]]; then
  echo "ОШИБКА: каталог прогонов не найден: $RUNS_ROOT" >&2
  exit 2
fi

mapfile -t runs < <(
  find "$RUNS_ROOT" -maxdepth 1 -mindepth 1 -type d     \( -name '*OPTICAL_FLOW_FLIGHT' -o -name '*OPTICAL_FLOW_MAVLINK_BENCH' \)     -printf '%T@ %p\n' 2>/dev/null   | sort -nr   | awk '{ $1=""; sub(/^ /,""); print }'   | while IFS= read -r run; do
      [[ -s "$run/optical_flow_mavlink.csv" ]] && printf '%s\n' "$run"
    done   | head -n 3
)

if (( ${#runs[@]} == 0 )); then
  echo "ОШИБКА: OpticalFlow run-каталоги в $RUNS_ROOT не найдены." >&2
  exit 2
fi

stamp="$(date +%Y%m%d_%H%M%S)"
out="$OUT_DIR/${stamp}_LATEST_OPTFLOW_LOGS.tar.gz"
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

manifest="$tmp/MANIFEST.txt"
{
  echo "JT-Zero latest OpticalFlow log bundle"
  echo "created: $(date --iso-8601=seconds)"
  echo "host: $(hostname)"
  echo "runs_root: $RUNS_ROOT"
  echo
  echo "Included runs:"
} > "$manifest"

for run in "${runs[@]}"; do
  base="$(basename "$run")"
  mkdir -p "$tmp/$base"
  echo "  $run" >> "$manifest"

  for f in     optical_flow_mavlink.csv     remote_ekf.bin     build.log     session.json     optical_flow_continuous_series.json     camera.csv     range.csv     attitude.csv
  do
    if [[ -f "$run/$f" ]]; then
      cp -a "$run/$f" "$tmp/$base/"
    fi
  done

  # Include small text/json/csv diagnostics produced in the run directory,
  # but avoid binaries/executables and large frame/media dumps.
  find "$run" -maxdepth 1 -type f     \( -name '*.txt' -o -name '*.json' -o -name '*.csv' -o -name '*.log' \)     -size -25M -print0 2>/dev/null   | while IFS= read -r -d '' f; do
      cp -an "$f" "$tmp/$base/" || true
    done

done

{
  echo
  echo "Git:"
  if git -C "$(dirname "${BASH_SOURCE[0]}")/.." rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    git -C "$(dirname "${BASH_SOURCE[0]}")/.." status --short
    git -C "$(dirname "${BASH_SOURCE[0]}")/.." log -3 --oneline
  fi
} >> "$manifest"

tar -C "$tmp" -czf "$out" .

echo "======================================================================"
echo "JT-ZERO — LATEST OPTICAL FLOW LOG BUNDLE"
echo "======================================================================"
echo "Packed latest ${#runs[@]} OpticalFlow run(s):"
printf '  %s\n' "${runs[@]}"
echo
echo "OUTPUT: $out"
echo "SIZE:   $(du -h "$out" | awk '{print $1}')"
echo
echo "Скопировать на ПК можно, например:"
echo "  scp vio@$(hostname):$out ."
echo "======================================================================"
