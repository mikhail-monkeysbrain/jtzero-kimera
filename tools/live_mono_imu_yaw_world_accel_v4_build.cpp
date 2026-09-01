// Compile entry for JT-ZERO yaw WORLD-acceleration diagnostic v4.
// The v4 source includes v3 only to reuse shared helpers; rename both nested mains safely.
#define main jtzero_v3_unused_main
#include "live_mono_imu_yaw_world_accel_v4.cpp"
#undef main
