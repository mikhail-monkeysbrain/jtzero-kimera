// JT-ZERO 300 mm x6 v8.
// Keeps validated v7 fusion and replaces instantaneous START/END snapshots
// with stabilized backend endpoint capture.
// Reuse v7 implementation as the runtime entry point: rename its main to v8_main,
// then provide a single wrapper main below. This avoids duplicate main definitions.

#define main jtzero_v8_main
#include "live_mono_imu_300mm_repeat_hud_v7.cpp"
#undef main

int main(int argc, char** argv) {
  return jtzero_v8_main(argc, argv);
}
