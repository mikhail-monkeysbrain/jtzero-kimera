// JT-ZERO Stage 11 v15.44.
// Parameterized ZXY-scale replay is generated from the proven v11 replay at build time.
// This file intentionally keeps production jtzero_imu_correction.h untouched.
//
// Build usage is handled by run_zxy_scale_fc_reference_v15_44.sh.
// The runner copies replay_mono_imu_zxy_ab_v11.cpp to /tmp and performs three
// diagnostic-only substitutions there:
//   1) bool use_zxy -> double zxy_scale
//   2) applyZxy(gyro_flu) -> gyro_flu + scale*[Cx*gz,Cy*gz,0]
//   3) CURRENT/NO_ZXY CLI -> numeric scale CLI
//
// This translation unit exists as the tracked specification/guard for that
// diagnostic transformation. It is not linked directly because the complete
// replay implementation remains in replay_mono_imu_zxy_ab_v11.cpp.

#include <cstdlib>
#include <iostream>

int main(int argc, char** argv) {
  std::cerr
      << "JT-ZERO v15.44: do not run this helper directly.\n"
      << "Run: bash tools/run_zxy_scale_fc_reference_v15_44.sh\n";
  return argc > 1000 ? std::atoi(argv[1]) : 2;
}
