// JT-ZERO diagnostic wrapper: feed ArduPilot HIGHRES_IMU (FRD) to Kimera in FLU.
//
// ArduPilot / body FRD: +X forward, +Y right, +Z down.
// Kimera diagnostic body FLU: +X forward, +Y left, +Z up.
// Transformation is R_FLU_FRD = diag(+1, -1, -1), i.e. 180 deg about X.
// The same transform must be used by camera T_BS in params/JTZeroMonoFLU.

#include "common/mavlink.h"

static inline void jtzero_mavlink_msg_highres_imu_decode_flu(
    const mavlink_message_t* msg,
    mavlink_highres_imu_t* imu) {
  mavlink_msg_highres_imu_decode(msg, imu);

  imu->yacc = -imu->yacc;
  imu->zacc = -imu->zacc;
  imu->ygyro = -imu->ygyro;
  imu->zgyro = -imu->zgyro;
}

// Intercept only HIGHRES_IMU decoding inside the existing live standstill tool.
// All camera, TIMESYNC, queueing, and backend logic stays byte-for-byte identical.
#define mavlink_msg_highres_imu_decode jtzero_mavlink_msg_highres_imu_decode_flu
#define main jtzero_live_mono_imu_standstill_frd_main
#include "live_mono_imu_standstill.cpp"
#undef main
#undef mavlink_msg_highres_imu_decode

int main(int argc, char** argv) {
  return jtzero_live_mono_imu_standstill_frd_main(argc, argv);
}
