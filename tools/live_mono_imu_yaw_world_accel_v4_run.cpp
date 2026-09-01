// Buildable entry point for JT-ZERO yaw WORLD-acceleration diagnostic v4.
// The v4 diagnostic source is included in the same translation unit.

#define main jtzero_v4_internal_main
#define vd double
#include "live_mono_imu_yaw_world_accel_v4.cpp"
#undef vd
#undef main

int main(int argc, char** argv) {
  return jtzero_v4_internal_main(argc, argv);
}
