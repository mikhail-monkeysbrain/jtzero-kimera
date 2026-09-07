#!/usr/bin/env bash
set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PARAMS="$ROOT/params/JTZeroMonoFLU_TBS_Rm1d5_Pm5d5_ARW_003"
BIN="/tmp/live_mono_imu_500mm_single_hud_v25"
SRC="$ROOT/tools/live_mono_imu_500mm_single_hud_v25.cpp"

export JTZERO_V25_PARAMS="$PARAMS"
export JTZERO_GRAVITY_ALIGNED_IMU_INIT=1
export JTZERO_DIAG_IMU_INIT="${JTZERO_DIAG_IMU_INIT:-1}"
export JTZERO_STAGED_ZUPT="${JTZERO_STAGED_ZUPT:-1}"
export JTZERO_V25_BIN="$BIN"
export JTZERO_V25_SOURCE="$SRC"
export JTZERO_V25_MODE_DESC="V25 isolated single-leg; fresh VIO process per 500mm leg; strict START; full startup static gate + 12s warm-up; FRD->FLU only; ZXY OFF; gravity feedback OFF"

echo "============================================================"
echo "V25 ISOLATED STATE-RESET DISCRIMINATOR"
echo "REQUIRED DRONE/STAND YAW: approximately -90 deg"
echo "Sequence: A->B, B->A, A->B, B->A"
echo "IMPORTANT: each leg launches a FRESH VIO process."
echo "Do not change yaw, height, scene, params, or A/B marks between legs."
echo "============================================================"
echo
echo "[ISOLATED] params=$PARAMS"
grep -nE 'accelerometer_random_walk|accelerometer_noise_density|gyroscope_random_walk' "$PARAMS/ImuParams.yaml"

echo "[ISOLATED] building dedicated single-leg binary..."
bash "$ROOT/tools/build_v25.sh" "$BIN"

directions=("A_TO_B" "B_TO_A" "A_TO_B" "B_TO_A")
labels=("RESET_L1_A_TO_B" "RESET_L2_B_TO_A" "RESET_L3_A_TO_B" "RESET_L4_B_TO_A")
overall_rc=0

for i in 0 1 2 3; do
  dir="${directions[$i]}"
  label="${labels[$i]}"
  echo
  echo "============================================================"
  echo "ISOLATED LEG $((i+1))/4"
  echo "DRONE/STAND YAW: approximately -90 deg"
  if [[ "$dir" == "A_TO_B" ]]; then
    echo "DIRECTION: A -> B"
    mode_arg=""
  else
    echo "DIRECTION: B -> A"
    mode_arg="B_FIRST"
  fi
  echo "Place the stand exactly at the required start mark."
  echo "The process will perform its own startup gate and 12s warm-up."
  echo "============================================================"

  set +e
  if [[ -n "$mode_arg" ]]; then
    bash "$ROOT/tools/run_v25_auto_camera.sh" "$mode_arg"
  else
    bash "$ROOT/tools/run_v25_auto_camera.sh"
  fi
  rc=$?
  set -e

  echo "[ISOLATED] leg $((i+1)) process exit code=$rc"
  archive_path="$(bash "$ROOT/tools/archive_v25_run.sh" "$label" "$PARAMS")"
  echo "[ISOLATED] leg $((i+1)) archive=$archive_path"

  if [[ $rc -ne 0 ]]; then
    overall_rc=$rc
    echo "[ISOLATED] NOTE: measurement/pipeline did not PASS for this leg, but archive was preserved."
  fi
done

echo
echo "============================================================"
echo "ISOLATED TEST COMPLETE"
echo "Four archives above are the causal dataset."
echo "Do NOT merge their estimator states; every archive is a fresh process."
echo "============================================================"

exit "$overall_rc"
