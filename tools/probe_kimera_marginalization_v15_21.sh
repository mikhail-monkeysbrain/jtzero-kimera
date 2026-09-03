#!/usr/bin/env bash
set -u -o pipefail
K=/home/vio/Kimera-VIO
OUT=/home/vio/jtzero_kimera_marginalization_probe_v15_21.txt

{
  echo '============================================================'
  echo 'JT-ZERO KIMERA MARGINALIZATION SOURCE PROBE v15.21'
  echo '============================================================'
  echo
  echo '=== REPOSITORY ==='
  cd "$K" || exit 2
  git rev-parse --show-toplevel 2>/dev/null || true
  git rev-parse HEAD 2>/dev/null || true
  git status --short 2>/dev/null || true
  git describe --always --dirty --tags 2>/dev/null || true
  echo
  echo '=== SMOOTHER TYPE ==='
  grep -RInE 'typedef gtsam::(IncrementalFixedLagSmoother|BatchFixedLagSmoother).*Smoother|using Smoother' "$K/include" 2>/dev/null || true
  echo
  echo '=== INCREMENTAL_SMOOTHER BUILD DEFINITION ==='
  grep -RIn 'INCREMENTAL_SMOOTHER' "$K/CMakeLists.txt" "$K/cmake" "$K/build" 2>/dev/null | head -80 || true
  echo
  echo '=== NR_STATES CONSTRUCTION ==='
  grep -RIn -C 5 'nr_states_' "$K/src/backend" "$K/include/kimera-vio/backend" 2>/dev/null | head -160 || true
  echo
  echo '=== SMOOTHER UPDATE CALLS ==='
  grep -RIn -C 12 'smoother_->update' "$K/src/backend" 2>/dev/null | head -240 || true
  echo
  echo '=== TIMESTAMPS PASSED TO SMOOTHER ==='
  grep -RIn -C 10 -E 'KeyTimestampMap|timestamps\[|timestamps\.insert|timestamps\.emplace|timestamp.*frame|frame.*timestamp' "$K/src/backend" "$K/include/kimera-vio/backend" 2>/dev/null | head -320 || true
  echo
  echo '=== FIXED-LAG / MARGINALIZATION REFERENCES ==='
  grep -RIn -C 6 -Ei 'marginali[sz]|fixed.?lag|erase.*factor|delete_slots|getFactors\(\)' "$K/src/backend" "$K/include/kimera-vio/backend" 2>/dev/null | head -420 || true
  echo
  echo '=== GTSAM FIXED-LAG HEADERS ==='
  for d in /usr/local/include /usr/include "$K/build"; do
    [[ -d "$d" ]] || continue
    grep -RIl 'class IncrementalFixedLagSmoother' "$d" 2>/dev/null | head -10 || true
  done
  echo
  echo 'RESULT: COMPLETE'
} | tee "$OUT"

echo "Saved: $OUT"
