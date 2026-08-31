#include "camera_imu_timestamp_policy.hpp"

#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

using jtzero::timesync::CameraContinuityResult;
using jtzero::timesync::CameraFrameStamp;
using jtzero::timesync::cameraContinuityStatusName;
using jtzero::timesync::checkCameraContinuity;
using jtzero::timesync::correctCameraTimestampNs;
using jtzero::timesync::kCameraToImuCorrectionNs;
using jtzero::timesync::kMaxCameraFrameDtNs;

std::vector<std::string> splitCsv(const std::string& line) {
    std::vector<std::string> out;
    std::string cell;
    std::stringstream ss(line);
    while (std::getline(ss, cell, ',')) {
        out.push_back(cell);
    }
    return out;
}

int findColumn(const std::vector<std::string>& header, const std::string& name) {
    for (size_t i = 0; i < header.size(); ++i) {
        if (header[i] == name) return static_cast<int>(i);
    }
    return -1;
}

bool selfTest() {
    bool ok = true;

    const CameraFrameStamp a{100, 1'000'000'000LL};

    struct Case {
        const char* name;
        CameraFrameStamp b;
        const char* expected;
    };

    const Case cases[] = {
        {"normal", {101, 1'008'000'000LL}, "ok"},
        {"sequence_gap", {103, 1'016'000'000LL}, "sequence_gap"},
        {"non_monotonic", {101, 999'000'000LL}, "non_monotonic_timestamp"},
        {"timestamp_gap", {101, 1'025'000'000LL}, "timestamp_gap"},
    };

    std::cout << "=== POLICY SELF-TEST ===\n";
    for (const auto& c : cases) {
        const auto r = checkCameraContinuity(a, c.b);
        const std::string got = cameraContinuityStatusName(r.status);
        const bool pass = got == c.expected;
        ok = ok && pass;
        std::cout << std::left << std::setw(20) << c.name
                  << " expected=" << std::setw(24) << c.expected
                  << " got=" << std::setw(24) << got
                  << (pass ? " PASS" : " FAIL") << '\n';
    }

    const int64_t corrected = correctCameraTimestampNs(1'000'000'000LL);
    const bool correction_ok = corrected == 1'000'000'000LL + kCameraToImuCorrectionNs;
    ok = ok && correction_ok;
    std::cout << "camera correction      expected=+10.500 ms"
              << " got=+" << std::fixed << std::setprecision(3)
              << static_cast<double>(corrected - 1'000'000'000LL) / 1.0e6
              << " ms " << (correction_ok ? "PASS" : "FAIL") << "\n\n";

    return ok;
}

bool checkCsv(const std::string& path) {
    std::ifstream in(path);
    if (!in) {
        std::cerr << "Cannot open: " << path << '\n';
        return false;
    }

    std::string line;
    if (!std::getline(in, line)) {
        std::cerr << "Empty CSV: " << path << '\n';
        return false;
    }

    const auto header = splitCsv(line);
    const int seq_col = findColumn(header, "sequence");
    const int ts_col = findColumn(header, "v4l2_timestamp_ns");
    const int corrected_col = findColumn(header, "camera_timestamp_corrected_ns");

    if (seq_col < 0 || ts_col < 0) {
        std::cerr << "Required columns not found in " << path << '\n';
        return false;
    }

    uint64_t frames = 0;
    uint64_t ok_pairs = 0;
    uint64_t rejected_pairs = 0;
    uint64_t sequence_gaps = 0;
    uint64_t timestamp_gaps = 0;
    uint64_t non_monotonic = 0;
    uint64_t missing_frames = 0;
    uint64_t corrected_mismatches = 0;
    int64_t max_rejected_dt_ns = 0;

    bool have_prev = false;
    CameraFrameStamp prev{};

    while (std::getline(in, line)) {
        if (line.empty()) continue;
        const auto cells = splitCsv(line);
        if (static_cast<int>(cells.size()) <= std::max(seq_col, ts_col)) continue;

        CameraFrameStamp cur{};
        try {
            cur.sequence = static_cast<uint32_t>(std::stoul(cells[seq_col]));
            cur.v4l2_timestamp_ns = std::stoll(cells[ts_col]);
        } catch (...) {
            continue;
        }

        ++frames;

        if (corrected_col >= 0 && static_cast<int>(cells.size()) > corrected_col) {
            try {
                const int64_t serialized = std::stoll(cells[corrected_col]);
                const int64_t expected = correctCameraTimestampNs(cur.v4l2_timestamp_ns);
                if (serialized != expected) ++corrected_mismatches;
            } catch (...) {
                ++corrected_mismatches;
            }
        }

        if (have_prev) {
            const CameraContinuityResult r = checkCameraContinuity(prev, cur);
            if (r.ok()) {
                ++ok_pairs;
            } else {
                ++rejected_pairs;
                missing_frames += r.missing_frames;
                if (r.dt_ns > max_rejected_dt_ns) max_rejected_dt_ns = r.dt_ns;
                const std::string status = cameraContinuityStatusName(r.status);
                if (status == "sequence_gap") ++sequence_gaps;
                else if (status == "timestamp_gap") ++timestamp_gaps;
                else if (status == "non_monotonic_timestamp") ++non_monotonic;
            }
        }

        prev = cur;
        have_prev = true;
    }

    std::cout << "=== CAMERA CSV RUNTIME POLICY CHECK ===\n";
    std::cout << "file:                    " << path << '\n';
    std::cout << "frames:                  " << frames << '\n';
    std::cout << "accepted pairs:          " << ok_pairs << '\n';
    std::cout << "rejected pairs:          " << rejected_pairs << '\n';
    std::cout << "sequence gaps:           " << sequence_gaps << '\n';
    std::cout << "timestamp gaps:          " << timestamp_gaps << '\n';
    std::cout << "non-monotonic:           " << non_monotonic << '\n';
    std::cout << "source frames missing:   " << missing_frames << '\n';
    std::cout << "corrected ts mismatches: " << corrected_mismatches << '\n';
    std::cout << "max rejected dt:         " << std::fixed << std::setprecision(3)
              << static_cast<double>(max_rejected_dt_ns) / 1.0e6 << " ms\n";
    std::cout << "max allowed dt:          "
              << static_cast<double>(kMaxCameraFrameDtNs) / 1.0e6 << " ms\n\n";

    const bool pass = frames >= 2 && corrected_mismatches == 0;
    std::cout << "RESULT: " << (pass ? "PASS" : "FAIL") << '\n';
    return pass;
}

}  // namespace

int main(int argc, char** argv) {
    const std::string path = argc > 1
        ? argv[1]
        : "/home/vio/camera_imu_yaw_camera.csv";

    const bool self_ok = selfTest();
    const bool csv_ok = checkCsv(path);

    return (self_ok && csv_ok) ? 0 : 1;
}
