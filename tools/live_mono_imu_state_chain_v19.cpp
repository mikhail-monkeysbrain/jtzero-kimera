#include <gflags/gflags.h>
DECLARE_bool(no_incremental_pose);

#define JTZERO_V18_NO_MAIN
#include "live_mono_imu_state_chain_v18.cpp"
#undef JTZERO_V18_NO_MAIN

int main(int argc, char** argv) {
  FLAGS_no_incremental_pose = true;
  std::cout << "[v19] optimizer state output enabled\n";
  return runStateChainV18(argc, argv);
}
