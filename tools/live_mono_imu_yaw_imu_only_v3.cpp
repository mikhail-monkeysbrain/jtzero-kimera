// JT-ZERO yaw IMU-only diagnostic v3 GUI.
// GUI version derived from the existing FLU yaw HUD.
#define main jtzero_yaw_v2_base_main
#include "live_mono_imu_yaw_only_hud_v2_flu.cpp"
#undef main

// This executable intentionally reuses the proven camera/MAVLink/GUI test path.
// Backend visual smart measurements are disabled by JTZERO_DIAG_IMU_ONLY in
// the patched Kimera MonoImuPipeline. The v2 HUD provides live camera, phase,
// FC roll/pitch/yaw, VIO yaw, false XY and Vxy guidance.
// Detailed BA/BG and raw FLU IMU instrumentation is added in the next isolated
// backend instrumentation step after this GUI path is verified on target.

int main(int argc, char** argv) {
  if (!std::getenv("JTZERO_DIAG_IMU_ONLY")) {
    std::cerr << "[FATAL] JTZERO_DIAG_IMU_ONLY is not set\n";
    return 1;
  }
  std::cout << "[V3 GUI] IMU-only backend diagnostic enabled\n";
  return jtzero_yaw_v2_base_main(argc, argv);
}
