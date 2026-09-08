#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="/tmp/backup_fc_params_v45"
OUT="${1:-/home/vio/jtzero_fc_params_v45.tsv}"
export JTZERO_V25_SOURCE="$ROOT/tools/backup_fc_params_v45.cpp"
echo "============================================================"
echo "JT-ZERO: ПОЛНЫЙ BACKUP ПАРАМЕТРОВ FC v45"
echo "ТОЛЬКО ЧТЕНИЕ: FC НЕ изменяется."
echo "============================================================"
bash "$ROOT/tools/build_v25.sh" "$BIN"
LD_LIBRARY_PATH=/home/vio/Kimera-VIO/build:/usr/local/lib:${LD_LIBRARY_PATH:-} "$BIN" "$OUT"
echo
echo "SHA256:"
sha256sum "$OUT"
