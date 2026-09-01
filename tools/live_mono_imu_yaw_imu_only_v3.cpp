// JT-ZERO yaw IMU-only diagnostic v3 GUI.
// Reuse the proven FLU yaw-only HUD implementation and require the
// backend IMU-only diagnostic switch.

#include <cstdlib>
#include <iostream>

// live_mono_imu_yaw_only_hud_v2_flu.cpp already renames its own main to
// jtzero_hud_v2_unused_main while including the HUD base, then defines its
// actual test main. Rename that outer main here.
#define main jtzero_yaw_v2_flu_main
#include "live_mono_imu_yaw_only_hud_v2_flu.cpp"
#undef main

int main(int argc, char** argv) {
  if (!std::getenv("JTZERO_DIAG_IMU_ONLY")) {
    std::cerr << "[FATAL] JTZERO_DIAG_IMU_ONLY is not set\n";
    return 1;
  }
  std::cout << "[V3 GUI] IMU-only backend diagnostic enabled\n";
  return jtzero_yaw_v2_flu_main(argc, argv);
}
