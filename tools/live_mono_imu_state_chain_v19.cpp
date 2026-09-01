#include <gflags/gflags.h>
DECLARE_bool(no_incremental_pose);

#define main jtzero_state_chain_v18_main
#include "live_mono_imu_state_chain_v18.cpp"
#undef main

int main(int argc, char** argv) {
  FLAGS_no_incremental_pose = true;
  std::cout << "[v19] optimizer state output enabled\n";
  return jtzero_state_chain_v18_main(argc, argv);
}
