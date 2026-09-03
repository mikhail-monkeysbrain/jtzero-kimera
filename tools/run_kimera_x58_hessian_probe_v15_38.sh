#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
K=/home/vio/Kimera-VIO
SRC="$K/src/backend/VioBackend.cpp"
BAK=/tmp/VioBackend.cpp.v15_38.bak
BIN=/tmp/replay_mono_imu_zxy_ab_v11_v15_38
OUT=/home/vio/jtzero_x58_hessian_v15_38
IMU="${1:-/home/vio/jtzero_yaw_only_v13.csv}"
CAM="${2:-/home/vio/jtzero_yaw_only_v13_camera.csv}"
MJPG="${3:-/home/vio/jtzero_yaw_only_v13.mjpg}"
mkdir -p "$OUT"
for f in "$IMU" "$CAM" "$MJPG"; do [[ -s "$f" ]] || { echo "FATAL missing $f" >&2; exit 2; }; done
if ! git -C "$K" diff --quiet -- src/backend/VioBackend.cpp; then echo 'FATAL dirty VioBackend.cpp' >&2; exit 3; fi
cp "$SRC" "$BAK"
CLEAN=0
cleanup(){ cp "$BAK" "$SRC" 2>/dev/null || true; if [[ "$CLEAN" -eq 0 ]]; then cmake --build "$K/build" -j2 --target kimera_vio || true; fi; }
trap cleanup EXIT
build_replay(){
 g++ -std=c++17 -O2 "$ROOT/tools/replay_mono_imu_zxy_ab_v11.cpp" -o "$BIN" \
  -I"$ROOT/tools" -I"$K/include" -I"$K/build" -I"$K/third_party/mavlink" -I/usr/include/eigen3 \
  $(pkg-config --cflags opencv4) -L"$K/build" -L/usr/local/lib -lkimera_vio -lgtsam -lgflags -lglog -lpthread $(pkg-config --libs opencv4)
}
P="$OUT/params_H28"; rm -rf "$P"; cp -a "$ROOT/params/JTZeroMonoFLU" "$P"; sed -i -E 's/^[[:space:]]*nr_states:[[:space:]]*.*/nr_states: 28/' "$P/BackendParams.yaml"
metric(){ awk -F': ' '/^final \|dP\| mm:/{print $2;exit}' "$1"; }
run_case(){
 local name="$1" mode="$2" log="$OUT/$name.txt"
 rm -f /home/vio/jtzero_zxy_replay_v11_CURRENT.csv
 set +e
 JT1538_MODE="$mode" LD_LIBRARY_PATH="$K/build:/usr/local/lib:${LD_LIBRARY_PATH:-}" "$BIN" "$P" CURRENT "$IMU" "$CAM" "$MJPG" >"$log" 2>&1
 local rc=$?; set -e
 local dp states; dp=$(metric "$log"); states=$(awk -F': ' '/^backend states:/{print $2;exit}' "$log")
 printf '%-20s rc=%d states=%s final_dP=%s\n' "$name" "$rc" "${states:-NA}" "${dp:-NA}"
 [[ $rc -eq 0 && -n "$dp" ]] || { tail -120 "$log"; exit 10; }
}

echo '============================================================'
echo 'JT-ZERO v15.38 POST-MARGINALIZATION HESSIAN CONDITION PROBE'
echo 'H28 yaw_only_v13. Probe runs are separate from zero-perturbation gate.'
echo '============================================================'
echo '[1/7] Build clean Kimera + replay'; cmake --build "$K/build" -j2 --target kimera_vio; build_replay
echo '[2/7] CLEAN H28'; run_case CLEAN_H28 OFF; CLEAN_DP=$(metric "$OUT/CLEAN_H28.txt")

echo '[3/7] Inject gate + separate post-update Hessian probe'
anchor='        smoother_->update(new_factors, new_values, timestamps, delete_slots);'
[[ $(grep -Foc "$anchor" "$SRC" || true) == 1 ]] || { echo 'FATAL update anchor not unique' >&2; exit 4; }
TMP=/tmp/VioBackend.cpp.v15_38.instrumented
awk -v anchor="$anchor" '
$0==anchor && !done {
 print "        ([&]() {";
 print "          std::map<Key,double> jt1538_ts = timestamps;";
 print "          const char* e=std::getenv(\"JT1538_MODE\"); const std::string m=e?e:\"CONTROL\";";
 print "          const bool delay=(m==\"DELAY\" || m==\"PROBE_DELAY\");";
 print "          const bool probe=(m==\"PROBE_CONTROL\" || m==\"PROBE_DELAY\");";
 print "          if (delay && curr_kf_id_==87) { const size_t i=58; const double t=59.0; jt1538_ts[gtsam::Symbol(\"x\"[0],i)]=t; jt1538_ts[gtsam::Symbol(\"v\"[0],i)]=t; jt1538_ts[gtsam::Symbol(\"b\"[0],i)]=t; }";
 print "          auto jt1538_result=smoother_->update(new_factors,new_values,jt1538_ts,delete_slots);";
 print "          if (probe && (curr_kf_id_==87 || curr_kf_id_==88)) {";
 print "            try {";
 print "              const auto& fg=smoother_->getFactors(); const auto& lp=smoother_->getLinearizationPoint();";
 print "              auto gfg=fg.linearize(lp); auto hb=gfg->hessian(); const gtsam::Matrix& H=hb.first;";
 print "              Eigen::SelfAdjointEigenSolver<gtsam::Matrix> es(H,Eigen::EigenvaluesOnly); const auto ev=es.eigenvalues();";
 print "              double maxe=ev.size()?ev.maxCoeff():0.0; double minpos=std::numeric_limits<double>::infinity(); size_t near0=0, neg=0;";
 print "              for(Eigen::Index i=0;i<ev.size();++i){ const double v=ev(i); if(v< -1e-9*std::max(1.0,maxe)) ++neg; if(std::abs(v)<=1e-9*std::max(1.0,maxe)) ++near0; if(v>1e-12*std::max(1.0,maxe) && v<minpos) minpos=v; }";
 print "              const double cond=(std::isfinite(minpos)&&minpos>0)?maxe/minpos:std::numeric_limits<double>::infinity();";
 print "              std::cerr<<\"[JT15.38] mode=\"<<m<<\" kf=\"<<curr_kf_id_<<\" factors=\"<<fg.size()<<\" values=\"<<lp.size()<<\" Hdim=\"<<H.rows()<<\" eig_min=\"<<(ev.size()?ev.minCoeff():0.0)<<\" eig_minpos=\"<<minpos<<\" eig_max=\"<<maxe<<\" cond=\"<<cond<<\" near0=\"<<near0<<\" neg=\"<<neg<<std::endl;";
 print "            } catch(const std::exception& ex) { std::cerr<<\"[JT15.38] PROBE_EXCEPTION kf=\"<<curr_kf_id_<<\" what=\"<<ex.what()<<std::endl; }";
 print "          }";
 print "          return jt1538_result;";
 print "        })();"; done=1; next }
{print}
END{if(!done)exit 7}' "$SRC" > "$TMP"; mv "$TMP" "$SRC"
grep -q '^#include <cstdlib>' "$SRC" || sed -i '/#include <map>/a #include <cstdlib>' "$SRC"
grep -q '^#include <limits>' "$SRC" || sed -i '/#include <map>/a #include <limits>' "$SRC"
grep -q 'Eigen/Eigenvalues' "$SRC" || sed -i '/#include <map>/a #include <Eigen/Eigenvalues>' "$SRC"
cmake --build "$K/build" -j2 --target kimera_vio

echo '[4/7] Zero-perturbation CONTROL gate'; run_case INSTR_CONTROL CONTROL; INSTR_DP=$(metric "$OUT/INSTR_CONTROL.txt")
python3 - "$CLEAN_DP" "$INSTR_DP" <<'PY'
import sys
c=float(sys.argv[1]); i=float(sys.argv[2]); d=abs(c-i)
print(f'GATE clean={c:.6f} instrumented={i:.6f} abs_delta={d:.6f} mm')
if d>0.01: print('RESULT: INVALID_INSTRUMENTATION'); raise SystemExit(21)
print('RESULT: CONTROL_GATE_PASS')
PY

echo '[5/7] Causal delay reference + isolated numerical probes'
run_case DELAY_X58 DELAY
run_case PROBE_CONTROL PROBE_CONTROL
run_case PROBE_DELAY PROBE_DELAY

echo '[6/7] Restore clean Kimera'; cp "$BAK" "$SRC"; cmake --build "$K/build" -j2 --target kimera_vio; CLEAN=1; trap - EXIT

echo '[7/7] Report'; echo
echo '================ V15.38 TRAJECTORY CHECK ================'
for n in CLEAN_H28 INSTR_CONTROL DELAY_X58 PROBE_CONTROL PROBE_DELAY; do printf '%-20s states=%-5s dP=%s\n' "$n" "$(awk -F': ' '/^backend states:/{print $2;exit}' "$OUT/$n.txt")" "$(metric "$OUT/$n.txt")"; done
echo
echo '================ HESSIAN PROBES ================'
grep '\[JT15.38\]' "$OUT/PROBE_CONTROL.txt" || true
grep '\[JT15.38\]' "$OUT/PROBE_DELAY.txt" || true
echo
echo 'Interpretation guard: PROBE trajectory may differ because dense linearization/eigendecomposition is diagnostic-only.'
echo 'Use Hessian lines at kf87/kf88 for conditioning comparison; use CLEAN/CONTROL/DELAY for causal trajectory metrics.'
echo "Full logs: $OUT"
echo 'RESULT: COMPLETE'
