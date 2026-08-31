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

// Runtime continuity policy for OV9281 USB @ 120 FPS.
// A frame pair is valid only when the V4L2 sequence is contiguous and the
// corrected camera timestamps advance by a physically plausible interval.
// The 20 ms ceiling is intentionally above the observed normal 8/12 ms UVC
// cadence, while still rejecting long gaps before they reach visual tracking.
constexpr int64_t kMaxCameraFrameDtNs = 20'000'000LL;

struct CameraFrameStamp {
    uint32_t sequence = 0;
    int64_t v4l2_timestamp_ns = 0;
};

enum class CameraContinuityStatus : uint8_t {
    kOk = 0,
    kSequenceGap,
    kNonMonotonicTimestamp,
    kTimestampGap,
};

struct CameraContinuityResult {
    CameraContinuityStatus status = CameraContinuityStatus::kOk;
    int64_t dt_ns = 0;
    uint32_t missing_frames = 0;

    constexpr bool ok() const noexcept {
        return status == CameraContinuityStatus::kOk;
    }
};

constexpr int64_t correctCameraTimestampNs(const int64_t v4l2_timestamp_ns) noexcept {
    return v4l2_timestamp_ns + kCameraToImuCorrectionNs;
}

constexpr double cameraToImuCorrectionMs() noexcept {
    return static_cast<double>(kCameraToImuCorrectionNs) / 1.0e6;
}

constexpr CameraContinuityResult checkCameraContinuity(
    const CameraFrameStamp& previous,
    const CameraFrameStamp& current) noexcept {

    const int64_t previous_corrected_ns =
        correctCameraTimestampNs(previous.v4l2_timestamp_ns);
    const int64_t current_corrected_ns =
        correctCameraTimestampNs(current.v4l2_timestamp_ns);
    const int64_t dt_ns = current_corrected_ns - previous_corrected_ns;

    // Sequence is checked before dt because a dropped UVC frame can still
    // produce an apparently normal timestamp interval. This was observed in
    // the runtime V4 test: one missing source frame with dt ~= 7.998 ms.
    const uint32_t expected_sequence = previous.sequence + 1U;
    if (current.sequence != expected_sequence) {
        const uint32_t missing = current.sequence > expected_sequence
            ? current.sequence - expected_sequence
            : 0U;
        return {
            CameraContinuityStatus::kSequenceGap,
            dt_ns,
            missing,
        };
    }

    if (dt_ns <= 0) {
        return {
            CameraContinuityStatus::kNonMonotonicTimestamp,
            dt_ns,
            0U,
        };
    }

    if (dt_ns > kMaxCameraFrameDtNs) {
        return {
            CameraContinuityStatus::kTimestampGap,
            dt_ns,
            0U,
        };
    }

    return {
        CameraContinuityStatus::kOk,
        dt_ns,
        0U,
    };
}

constexpr const char* cameraContinuityStatusName(
    const CameraContinuityStatus status) noexcept {
    switch (status) {
        case CameraContinuityStatus::kOk:
            return "ok";
        case CameraContinuityStatus::kSequenceGap:
            return "sequence_gap";
        case CameraContinuityStatus::kNonMonotonicTimestamp:
            return "non_monotonic_timestamp";
        case CameraContinuityStatus::kTimestampGap:
            return "timestamp_gap";
    }
    return "unknown";
}

}  // namespace jtzero::timesync
