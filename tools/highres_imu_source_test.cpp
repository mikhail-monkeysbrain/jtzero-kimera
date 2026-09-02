#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "common/mavlink.h"

namespace {

constexpr const char* SERIAL_DEVICE = "/dev/ttyAMA0";
constexpr double DEFAULT_DURATION_SEC = 30.0;
constexpr int HIGHRES_RATE_HZ = 200;
constexpr int SCALED_RATE_HZ = 100;
constexpr uint8_t COMPANION_SYSID = 255;
constexpr uint8_t COMPANION_COMPID = 190;
constexpr int64_t MATCH_WINDOW_NS = 12'000'000LL;
constexpr int64_t LAG_SCAN_MIN_NS = -20'000'000LL;
constexpr int64_t LAG_SCAN_MAX_NS = 20'000'000LL;
constexpr int64_t LAG_SCAN_STEP_NS = 500'000LL;

struct GyroSample {
    int64_t recv_ns = 0;
    int64_t fc_ns = 0;
    double gx = 0.0;
    double gy = 0.0;
    double gz = 0.0;
};

struct MatchStats {
    bool valid = false;
    int64_t best_lag_ns = 0;
    size_t matches = 0;
    double median_dt_ms = 0.0;
    double p95_dt_ms = 0.0;
    double rms_vec_rad_s = 0.0;
    double median_vec_rad_s = 0.0;
    double p95_vec_rad_s = 0.0;
    double corr_x = 0.0;
    double corr_y = 0.0;
    double corr_z = 0.0;
};

[[noreturn]] void fail(const std::string& text) {
    throw std::runtime_error(text + ": " + std::strerror(errno));
}

int64_t monotonicNs() {
    timespec ts{};
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) fail("clock_gettime");
    return static_cast<int64_t>(ts.tv_sec) * 1000000000LL +
           static_cast<int64_t>(ts.tv_nsec);
}

double nsToMs(int64_t ns) {
    return static_cast<double>(ns) / 1.0e6;
}

double percentile(std::vector<double> v, double q) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    const double p = q * static_cast<double>(v.size() - 1);
    const size_t lo = static_cast<size_t>(std::floor(p));
    const size_t hi = std::min(lo + 1, v.size() - 1);
    const double f = p - static_cast<double>(lo);
    return v[lo] * (1.0 - f) + v[hi] * f;
}

double correlation(const std::vector<double>& a, const std::vector<double>& b) {
    if (a.size() != b.size() || a.size() < 3) return 0.0;
    long double ma = 0.0L, mb = 0.0L;
    for (size_t i = 0; i < a.size(); ++i) {
        ma += a[i];
        mb += b[i];
    }
    ma /= static_cast<long double>(a.size());
    mb /= static_cast<long double>(b.size());
    long double sxx = 0.0L, syy = 0.0L, sxy = 0.0L;
    for (size_t i = 0; i < a.size(); ++i) {
        const long double da = static_cast<long double>(a[i]) - ma;
        const long double db = static_cast<long double>(b[i]) - mb;
        sxx += da * da;
        syy += db * db;
        sxy += da * db;
    }
    if (sxx <= 0.0L || syy <= 0.0L) return 0.0;
    return static_cast<double>(sxy / std::sqrt(sxx * syy));
}

int openSerial() {
    const int fd = open(SERIAL_DEVICE, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd == -1) fail("open serial");
    termios tty{};
    if (tcgetattr(fd, &tty) != 0) fail("tcgetattr");
    cfmakeraw(&tty);
    if (cfsetispeed(&tty, B460800) != 0 || cfsetospeed(&tty, B460800) != 0)
        fail("set baud");
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &tty) != 0) fail("tcsetattr");
    tcflush(fd, TCIFLUSH);
    std::cout << "[MAV] " << SERIAL_DEVICE << " @ 460800\n";
    return fd;
}

void serialWriteAll(int fd, const uint8_t* data, size_t size) {
    size_t done = 0;
    while (done < size) {
        const ssize_t rc = write(fd, data + done, size - done);
        if (rc > 0) {
            done += static_cast<size_t>(rc);
            continue;
        }
        if (rc == -1 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            pollfd pfd{};
            pfd.fd = fd;
            pfd.events = POLLOUT;
            poll(&pfd, 1, 10);
            continue;
        }
        if (rc == -1 && errno == EINTR) continue;
        fail("serial write");
    }
}

void sendMavlinkMessage(int fd, const mavlink_message_t& msg) {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    serialWriteAll(fd, buffer, len);
}

void requestMessageRate(int fd, uint8_t target_system, uint8_t target_component,
                        uint32_t message_id, int rate_hz) {
    const float interval_us = rate_hz > 0
        ? static_cast<float>(1000000.0 / rate_hz)
        : 0.0f;
    mavlink_message_t msg{};
    mavlink_msg_command_long_pack(
        COMPANION_SYSID, COMPANION_COMPID, &msg,
        target_system, target_component,
        MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        static_cast<float>(message_id), interval_us,
        0, 0, 0, 0, 0);
    sendMavlinkMessage(fd, msg);
}

bool waitHeartbeat(int fd, uint8_t& target_system, uint8_t& target_component) {
    mavlink_status_t status{};
    mavlink_message_t msg{};
    const int64_t deadline = monotonicNs() + 5'000'000'000LL;
    while (monotonicNs() < deadline) {
        pollfd pfd{};
        pfd.fd = fd;
        pfd.events = POLLIN;
        const int rc = poll(&pfd, 1, 100);
        if (rc < 0) {
            if (errno == EINTR) continue;
            fail("poll heartbeat");
        }
        if (rc == 0 || !(pfd.revents & POLLIN)) continue;
        uint8_t bytes[4096];
        const ssize_t n = read(fd, bytes, sizeof(bytes));
        if (n <= 0) continue;
        for (ssize_t i = 0; i < n; ++i) {
            if (!mavlink_parse_char(MAVLINK_COMM_0, bytes[i], &msg, &status)) continue;
            if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                target_system = msg.sysid;
                target_component = msg.compid;
                return true;
            }
        }
    }
    return false;
}

size_t nearestSampleIndex(const std::vector<GyroSample>& samples, int64_t target_fc_ns) {
    auto it = std::lower_bound(
        samples.begin(), samples.end(), target_fc_ns,
        [](const GyroSample& s, int64_t t) { return s.fc_ns < t; });
    if (it == samples.begin()) return 0;
    if (it == samples.end()) return samples.size() - 1;
    const size_t hi = static_cast<size_t>(it - samples.begin());
    const size_t lo = hi - 1;
    const int64_t dlo = std::llabs(samples[lo].fc_ns - target_fc_ns);
    const int64_t dhi = std::llabs(samples[hi].fc_ns - target_fc_ns);
    return (dlo <= dhi) ? lo : hi;
}

MatchStats evaluate(const std::vector<GyroSample>& highres,
                    const std::vector<GyroSample>& scaled,
                    int64_t lag_ns) {
    MatchStats out{};
    if (highres.empty() || scaled.empty()) return out;
    std::vector<double> dts_ms, vec_errors;
    std::vector<double> hx, hy, hz, sx, sy, sz;
    long double sum_sq = 0.0L;
    for (const auto& h : highres) {
        const int64_t target = h.fc_ns + lag_ns;
        const size_t idx = nearestSampleIndex(scaled, target);
        const auto& s = scaled[idx];
        const int64_t dt_ns = std::llabs(s.fc_ns - target);
        if (dt_ns > MATCH_WINDOW_NS) continue;
        const double dx = h.gx - s.gx;
        const double dy = h.gy - s.gy;
        const double dz = h.gz - s.gz;
        const double e = std::sqrt(dx * dx + dy * dy + dz * dz);
        dts_ms.push_back(nsToMs(dt_ns));
        vec_errors.push_back(e);
        sum_sq += static_cast<long double>(e) * e;
        hx.push_back(h.gx); hy.push_back(h.gy); hz.push_back(h.gz);
        sx.push_back(s.gx); sy.push_back(s.gy); sz.push_back(s.gz);
    }
    if (vec_errors.size() < 50) return out;
    out.valid = true;
    out.best_lag_ns = lag_ns;
    out.matches = vec_errors.size();
    out.median_dt_ms = percentile(dts_ms, 0.50);
    out.p95_dt_ms = percentile(dts_ms, 0.95);
    out.rms_vec_rad_s = std::sqrt(static_cast<double>(
        sum_sq / static_cast<long double>(vec_errors.size())));
    out.median_vec_rad_s = percentile(vec_errors, 0.50);
    out.p95_vec_rad_s = percentile(vec_errors, 0.95);
    out.corr_x = correlation(hx, sx);
    out.corr_y = correlation(hy, sy);
    out.corr_z = correlation(hz, sz);
    return out;
}

MatchStats findBestLag(const std::vector<GyroSample>& highres,
                       const std::vector<GyroSample>& scaled) {
    MatchStats best{};
    double best_score = std::numeric_limits<double>::infinity();
    for (int64_t lag = LAG_SCAN_MIN_NS; lag <= LAG_SCAN_MAX_NS; lag += LAG_SCAN_STEP_NS) {
        MatchStats s = evaluate(highres, scaled, lag);
        if (!s.valid) continue;
        if (s.rms_vec_rad_s < best_score) {
            best_score = s.rms_vec_rad_s;
            best = s;
        }
    }
    return best;
}

void printStats(const char* name, const MatchStats& s) {
    std::cout << "\n=== HIGHRES vs " << name << " ===\n";
    if (!s.valid) {
        std::cout << "RESULT: insufficient matched samples\n";
        return;
    }
    std::cout << std::fixed << std::setprecision(6)
              << "best lag:           " << nsToMs(s.best_lag_ns) << " ms\n"
              << "matched samples:    " << s.matches << "\n"
              << "median |dt|:        " << s.median_dt_ms << " ms\n"
              << "p95 |dt|:           " << s.p95_dt_ms << " ms\n"
              << "gyro vector RMS:    " << s.rms_vec_rad_s << " rad/s\n"
              << "gyro vector median: " << s.median_vec_rad_s << " rad/s\n"
              << "gyro vector p95:    " << s.p95_vec_rad_s << " rad/s\n"
              << "corr X:             " << s.corr_x << "\n"
              << "corr Y:             " << s.corr_y << "\n"
              << "corr Z:             " << s.corr_z << "\n";
}

double meanCorrelation(const MatchStats& s) {
    return (s.corr_x + s.corr_y + s.corr_z) / 3.0;
}

} // namespace

int main(int argc, char** argv) {
    int serial_fd = -1;
    try {
        double duration_sec = DEFAULT_DURATION_SEC;
        if (argc >= 2) duration_sec = std::stod(argv[1]);
        if (duration_sec <= 0.0) throw std::runtime_error("Duration must be > 0");

        serial_fd = openSerial();
        uint8_t target_system = 0, target_component = 0;
        std::cout << "[MAV] waiting for HEARTBEAT...\n";
        if (!waitHeartbeat(serial_fd, target_system, target_component))
            throw std::runtime_error("MAVLink HEARTBEAT not received");

        std::cout << "[MAV] connected system=" << static_cast<int>(target_system)
                  << " component=" << static_cast<int>(target_component) << "\n";

        requestMessageRate(serial_fd, target_system, target_component,
                           MAVLINK_MSG_ID_HIGHRES_IMU, HIGHRES_RATE_HZ);
        requestMessageRate(serial_fd, target_system, target_component,
                           MAVLINK_MSG_ID_SCALED_IMU, SCALED_RATE_HZ);
        requestMessageRate(serial_fd, target_system, target_component,
                           MAVLINK_MSG_ID_SCALED_IMU2, SCALED_RATE_HZ);

        std::cout << "[MAV] requested HIGHRES_IMU @ " << HIGHRES_RATE_HZ << " Hz\n"
                  << "[MAV] requested SCALED_IMU  @ " << SCALED_RATE_HZ << " Hz\n"
                  << "[MAV] requested SCALED_IMU2 @ " << SCALED_RATE_HZ << " Hz\n\n";

        tcflush(serial_fd, TCIFLUSH);
        mavlink_status_t mav_status{};
        mavlink_message_t mav_message{};
        std::vector<GyroSample> highres, imu0, imu1;
        highres.reserve(static_cast<size_t>(duration_sec * HIGHRES_RATE_HZ + 100));
        imu0.reserve(static_cast<size_t>(duration_sec * SCALED_RATE_HZ + 100));
        imu1.reserve(static_cast<size_t>(duration_sec * SCALED_RATE_HZ + 100));

        const int64_t start_ns = monotonicNs();
        const int64_t end_ns = start_ns + static_cast<int64_t>(duration_sec * 1.0e9);

        std::cout << "=== HIGHRES IMU SOURCE TEST ===\n"
                  << "duration: " << duration_sec << " s\n"
                  << "Move the vehicle by hand through ROLL, PITCH and YAW.\n"
                  << "Use both slow and moderately quick rotations.\n\n";

        while (monotonicNs() < end_ns) {
            pollfd pfd{};
            pfd.fd = serial_fd;
            pfd.events = POLLIN;
            const int rc = poll(&pfd, 1, 5);
            if (rc < 0) {
                if (errno == EINTR) continue;
                fail("poll serial");
            }
            if (rc == 0 || !(pfd.revents & POLLIN)) continue;

            uint8_t bytes[8192];
            while (true) {
                const ssize_t n = read(serial_fd, bytes, sizeof(bytes));
                if (n == -1) {
                    if (errno == EAGAIN || errno == EWOULDBLOCK) break;
                    if (errno == EINTR) continue;
                    fail("serial read");
                }
                if (n == 0) break;

                for (ssize_t i = 0; i < n; ++i) {
                    if (!mavlink_parse_char(MAVLINK_COMM_0, bytes[i],
                                            &mav_message, &mav_status)) continue;
                    const int64_t recv_ns = monotonicNs();

                    if (mav_message.msgid == MAVLINK_MSG_ID_HIGHRES_IMU) {
                        mavlink_highres_imu_t m{};
                        mavlink_msg_highres_imu_decode(&mav_message, &m);
                        highres.push_back({
                            recv_ns,
                            static_cast<int64_t>(m.time_usec) * 1000LL,
                            m.xgyro, m.ygyro, m.zgyro
                        });
                    } else if (mav_message.msgid == MAVLINK_MSG_ID_SCALED_IMU) {
                        mavlink_scaled_imu_t m{};
                        mavlink_msg_scaled_imu_decode(&mav_message, &m);
                        imu0.push_back({
                            recv_ns,
                            static_cast<int64_t>(m.time_boot_ms) * 1'000'000LL,
                            static_cast<double>(m.xgyro) * 0.001,
                            static_cast<double>(m.ygyro) * 0.001,
                            static_cast<double>(m.zgyro) * 0.001
                        });
                    } else if (mav_message.msgid == MAVLINK_MSG_ID_SCALED_IMU2) {
                        mavlink_scaled_imu2_t m{};
                        mavlink_msg_scaled_imu2_decode(&mav_message, &m);
                        imu1.push_back({
                            recv_ns,
                            static_cast<int64_t>(m.time_boot_ms) * 1'000'000LL,
                            static_cast<double>(m.xgyro) * 0.001,
                            static_cast<double>(m.ygyro) * 0.001,
                            static_cast<double>(m.zgyro) * 0.001
                        });
                    }
                }
            }
        }

        std::cout << "\nreceived:\n"
                  << "  HIGHRES_IMU: " << highres.size() << "\n"
                  << "  SCALED_IMU:  " << imu0.size() << "\n"
                  << "  SCALED_IMU2: " << imu1.size() << "\n";

        if (highres.size() < 100)
            throw std::runtime_error("Too few HIGHRES_IMU samples");
        if (imu0.size() < 50 && imu1.size() < 50)
            throw std::runtime_error("Neither SCALED_IMU stream was received");

        const MatchStats s0 = findBestLag(highres, imu0);
        const MatchStats s1 = findBestLag(highres, imu1);
        printStats("IMU0 / SCALED_IMU", s0);
        printStats("IMU1 / SCALED_IMU2", s1);

        std::cout << "\n=== FINAL RESULT ===\n";
        if (!s0.valid && !s1.valid) {
            std::cout << "HIGHRES gyro source = INCONCLUSIVE\n"
                      << "Reason: insufficient comparable data.\n";
        } else if (s0.valid && !s1.valid) {
            std::cout << "HIGHRES gyro source = IMU0\n";
        } else if (!s0.valid && s1.valid) {
            std::cout << "HIGHRES gyro source = IMU1\n";
        } else {
            const double corr0 = meanCorrelation(s0);
            const double corr1 = meanCorrelation(s1);
            const double err_ratio = s0.rms_vec_rad_s > 0.0
                ? s1.rms_vec_rad_s / s0.rms_vec_rad_s
                : std::numeric_limits<double>::infinity();
            const double inv_err_ratio = s1.rms_vec_rad_s > 0.0
                ? s0.rms_vec_rad_s / s1.rms_vec_rad_s
                : std::numeric_limits<double>::infinity();

            const bool imu0_clear =
                s0.rms_vec_rad_s < s1.rms_vec_rad_s &&
                err_ratio >= 1.5 && corr0 >= corr1 - 0.02;
            const bool imu1_clear =
                s1.rms_vec_rad_s < s0.rms_vec_rad_s &&
                inv_err_ratio >= 1.5 && corr1 >= corr0 - 0.02;

            if (imu0_clear) {
                std::cout << "HIGHRES gyro source = IMU0\n";
            } else if (imu1_clear) {
                std::cout << "HIGHRES gyro source = IMU1\n";
            } else {
                std::cout << "HIGHRES gyro source = INCONCLUSIVE\n"
                          << "Both streams are too similar or error separation is too small.\n";
            }

            std::cout << std::fixed << std::setprecision(3)
                      << "mean corr IMU0 = " << corr0 << "\n"
                      << "mean corr IMU1 = " << corr1 << "\n";
        }

        close(serial_fd);
        serial_fd = -1;
        return 0;
    } catch (const std::exception& e) {
        if (serial_fd >= 0) close(serial_fd);
        std::cerr << "\nERROR: " << e.what() << "\n";
        return 1;
    }
}
