#pragma once

#include <cstdint>

namespace jtzero::timesync {

// Measured on OV9281 USB 640x480 MJPEG @ 120 FPS + Matek H743 HIGHRES_IMU @ 200 Hz.
// Dedicated start/stop yaw test:
//   raw global offset: -10.55 ms
//   15 segment median: -10.15 ms
//   MAD: 0.80 ms
// Sign verification:
//   camera +10.5 ms -> residual -0.05 ms
//   camera -10.5 ms -> residual -21.05 ms
constexpr int64_t kCameraToImuCorrectionNs = 10'500'000LL;

constexpr int64_t correctCameraTimestampNs(const int64_t v4l2_timestamp_ns) noexcept {
    return v4l2_timestamp_ns + kCameraToImuCorrectionNs;
}

constexpr double cameraToImuCorrectionMs() noexcept {
    return static_cast<double>(kCameraToImuCorrectionNs) / 1.0e6;
}

}  // namespace jtzero::timesync
