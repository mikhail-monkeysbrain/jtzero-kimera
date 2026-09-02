// JT-ZERO 11.5 v10: raw Camera+IMU recorder for deterministic extrinsics A/B replay.
// Records one physical motion only. No fixed pivot and no exact angle reproduction required.

#define main jtzero_camera_imu_logger_unused_main
#include "camera_imu_extrinsics_logger.cpp"
#undef main

// The existing logger is intentionally reused unchanged for acquisition because it already
// records lossless per-frame MJPEG offsets, corrected camera timestamps, HIGHRES_IMU samples,
// TIMESYNC samples and the final FC->RPi clock mapping required by deterministic replay.
//
// v10 acquisition command is therefore the verified logger binary built from this translation
// unit. The replay side consumes these exact outputs:
//   /home/vio/camera_imu_extrinsics.csv
//   /dev/shm/camera_imu_extrinsics_camera.csv
//   /dev/shm/camera_imu_extrinsics.mjpg
//
// NOTE: This wrapper exists as a stable Stage-11.5 entry point. Acquisition semantics must not
// diverge from camera_imu_extrinsics_logger.cpp until the replay comparison is validated.

int main(int argc, char** argv) {
  return jtzero_camera_imu_logger_unused_main(argc, argv);
}
