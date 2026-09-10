#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_DIR="/home/vio/jtzero_runs"

if ! python3 - <<'PY' >/dev/null 2>&1
import tkinter
PY
then
  echo "ОШИБКА: Python tkinter не установлен."
  echo "Установите: sudo apt install python3-tk"
  exit 2
fi

before="$(ls -1t "$RUN_DIR"/*_GROUND_MOTION_MVP.csv 2>/dev/null | head -1 || true)"
python3 "$ROOT/tools/ground_motion_test_gui.py"
sleep 0.5
after="$(ls -1t "$RUN_DIR"/*_GROUND_MOTION_MVP.csv 2>/dev/null | head -1 || true)"

if [[ -z "$after" || "$after" == "$before" ]]; then
  echo "GUI закрыт. Новый CSV не найден."
  exit 0
fi

echo
echo "===== РЕЗУЛЬТАТ GROUND MOTION MVP ====="
echo "CSV: $after"
awk -F, '
NR==1 { for(i=1;i<=NF;i++) h[$i]=i; next }
{
  n++
  if($(h["valid"])==1){
    valid++
    dx=$(h["dx_m"]); dy=$(h["dy_m"])
    path += sqrt(dx*dx+dy*dy)
  }
  if($(h["mav_sent"])==1) sent++
  q += $(h["quality"])
  x=$(h["x_m"]); y=$(h["y_m"])
  hh=$(h["height_m"])
  if(hh>0){
    if(!have_h){hmin=hmax=hh;have_h=1}
    if(hh<hmin)hmin=hh
    if(hh>hmax)hmax=hh
  }
}
END {
  if(n<1){print "CSV пуст"; exit}
  net=sqrt(x*x+y*y)
  printf "rows=%d\n",n
  printf "valid=%d (%.1f%%)\n",valid,100*valid/n
  printf "mav_sent=%d\n",sent
  printf "mean_quality=%.3f\n",q/n
  printf "final_xy=(%.4f, %.4f) m\n",x,y
  printf "NET=%.1f mm\n",net*1000
  printf "PATH=%.1f mm\n",path*1000
  if(have_h) printf "height_range=%.1f .. %.1f mm\n",hmin*1000,hmax*1000
}' "$after"
echo "========================================"
