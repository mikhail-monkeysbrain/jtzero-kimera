#!/usr/bin/env bash
set -euo pipefail

ROOT="${HOME}/jtzero-kimera-sync"
KIMERA="${KIMERA_SOURCE:-/home/vio/Kimera-VIO}"
BIN="${JTZERO_V25_BIN:-/tmp/live_mono_imu_500mm_v43_camera_forensic}"
PARAMS="${JTZERO_V25_PARAMS:-${ROOT}/params/JTZeroMonoFLU_TBS_Rm1d5_Pm5d5_ARW_003}"

cd "${ROOT}"

echo "======================================================================"
echo "R1 1/6 — DETERMINISTIC RECORDER/REPLAY BASELINE"
echo "NO PHYSICAL MOVEMENT"
echo "======================================================================"

fail=0
check(){ if eval "$2"; then printf "%-42s PASS\n" "$1"; else printf "%-42s FAIL\n" "$1"; fail=1; fi; }

check "jtzero git repository" 'git rev-parse --is-inside-work-tree >/dev/null 2>&1'
check "V25 camera CSV schema" 'grep -q "sequence,raw_timestamp_ns,corrected_timestamp_ns,mapping_valid,selected,offset,bytes" tools/v18_parts/live_mono_imu_500mm_repeat_hud_v18_part07b.inc'
check "event real wall timestamp" 'grep -q "event_wall_ns,event" tools/v25_parts/live_mono_imu_500mm_repeat_hud_v25_part08b.inc'
check "IMU raw receive/source/mapped timestamps" 'grep -q "type,recv_ns,source_ns,mapped_ns" tools/v18_parts/live_mono_imu_500mm_repeat_hud_v18_part07b.inc'
check "attitude receive timestamp" 'grep -q "recv_ns,time_boot_ms" tools/v18_parts/live_mono_imu_500mm_repeat_hud_v18_part07b.inc'
check "range receive timestamp" 'grep -q "recv_ns,time_boot_ms.*signal_quality" tools/v18_parts/live_mono_imu_500mm_repeat_hud_v18_part07b.inc'

echo
echo "===== CURRENT IDENTITY ====="
echo "jtzero_head=$(git rev-parse HEAD)"
echo "jtzero_branch=$(git branch --show-current)"
echo "jtzero_dirty_count=$(git status --porcelain | wc -l)"
if [[ -d "${KIMERA}/.git" ]]; then
  echo "kimera_head=$(git -C "${KIMERA}" rev-parse HEAD)"
  echo "kimera_branch=$(git -C "${KIMERA}" branch --show-current)"
  echo "kimera_dirty_count=$(git -C "${KIMERA}" status --porcelain | wc -l)"
else
  echo "kimera_head=MISSING"
  fail=1
fi
if [[ -x "${BIN}" ]]; then
  echo "binary=${BIN}"
  echo "binary_sha256=$(sha256sum "${BIN}" | awk '{print $1}')"
else
  echo "binary=MISSING (${BIN})"
fi

echo
echo "===== BLOCKERS BEFORE DATASET RECORDING ====="
echo "RAW_FRAME_BYTES: BLOCKED — V25 intentionally does not persist MJPEG bytes."
echo "FLOW_PAIR_TIMESTAMPS: BLOCKED — V43 forensic rows have no frame0/frame1 IDs/timestamps."
echo "DIRECT_LUNA_SAMPLE_TIME: BLOCKED — V42 direct reader exposes only range_valid + vertical_range_m."
echo "DIRECT_LUNA_THREAD_SAFETY: BLOCKED — range payload is not a synchronized timestamped sample."
echo
if (( fail )); then
  echo "R1 STEP 1 RESULT: FAIL — baseline identity/schema prerequisites are incomplete."
  exit 1
fi
echo "R1 STEP 1 RESULT: PASS WITH 4 EXPLICIT R1 BLOCKERS."
echo "NEXT: R1 2/6 implements asynchronous raw-frame recording + exact frame-pair timestamps."
