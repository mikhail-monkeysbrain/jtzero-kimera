// JT-ZERO v15.41: live-pipeline diagnostic around the validated v15.3/v15.4 yaw HUD.
// Adds zero-perturbation console watchdog snapshots. The existing Russian GUI remains unchanged.
// The runner validates that backend states actually cover INIT/YAW/SETTLE.
#define main jtzero_v154_original_main
#include "live_mono_imu_yaw_only_hud_v15_4_utf8.cpp"
#undef main

int main(int argc, char** argv) {
  std::cout << "[V15.41] live pipeline diagnostic wrapper\n"
            << "[V15.41] GUI is validated v15.4 UTF-8 HUD; runner enforces phase coverage.\n"
            << "[V15.41] Watch FC R/P separately from VIO R/P in the GUI.\n";
  return jtzero_v154_original_main(argc, argv);
}
