// JT-ZERO FC-only yaw motion probe.
// No camera and no Kimera-VIO. Verifies that the operator motion is a real ~90 deg yaw
// and measures roll/pitch contamination before repeating the VIO test.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

#include <mavlink/common/mavlink.h>

namespace {

constexpr const char* kSerial = "/dev/ttyAMA0";
constexpr int kBaud = B921600;
constexpr int kAttitudeHz = 50;
constexpr double kInitSec = 5.0;
constexpr double kRunSec = 30.0;
constexpr double kTargetDeg = 90.0;
constexpr double kRpLimitDeg = 2.0;
constexpr double kPi = 3.14159265358979323846;

int64_t monoNs() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

double wrap180(double d) {
  while (d > 180.0) d -= 360.0;
  while (d < -180.0) d += 360.0;
  return d;
}

int openSerial() {
  const int fd = open(kSerial, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) throw std::runtime_error("cannot open /dev/ttyAMA0");
  termios tio{};
  if (tcgetattr(fd, &tio) != 0) throw std::runtime_error("tcgetattr failed");
  cfmakeraw(&tio);
  cfsetispeed(&tio, kBaud);
  cfsetospeed(&tio, kBaud);
  tio.c_cflag |= CLOCAL | CREAD;
  tio.c_cflag &= ~CSTOPB;
  tio.c_cflag &= ~CRTSCTS;
  if (tcsetattr(fd, TCSANOW, &tio) != 0) throw std::runtime_error("tcsetattr failed");
  tcflush(fd, TCIOFLUSH);
  return fd;
}

void requestRate(int fd, uint8_t sys, uint8_t comp, uint32_t msgid, int hz) {
  mavlink_message_t m{};
  const float interval_us = hz > 0 ? 1000000.0f / static_cast<float>(hz) : -1.0f;
  mavlink_msg_command_long_pack(
      255, 190, &m, sys, comp,
      MAV_CMD_SET_MESSAGE_INTERVAL, 0,
      static_cast<float>(msgid), interval_us, 0, 0, 0, 0, 0);
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  const uint16_t n = mavlink_msg_to_send_buffer(buf, &m);
  if (write(fd, buf, n) != n) throw std::runtime_error("MAVLink write failed");
}

struct Sample {
  int64_t t_ns = 0;
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
};

}  // namespace

int main() {
  int fd = -1;
  try {
    fd = openSerial();
    mavlink_status_t status{};
    mavlink_message_t msg{};
    uint8_t sys = 0, comp = 0;

    std::cout << "[FC] waiting for HEARTBEAT...\n";
    const int64_t hb_deadline = monoNs() + 10000000000LL;
    while (monoNs() < hb_deadline && !sys) {
      pollfd p{fd, POLLIN, 0};
      if (poll(&p, 1, 100) <= 0) continue;
      uint8_t b[2048];
      const ssize_t n = read(fd, b, sizeof(b));
      if (n <= 0) continue;
      for (ssize_t i = 0; i < n; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_0, b[i], &msg, &status) &&
            msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
          sys = msg.sysid;
          comp = msg.compid;
          break;
        }
      }
    }
    if (!sys) throw std::runtime_error("HEARTBEAT timeout");
    requestRate(fd, sys, comp, MAVLINK_MSG_ID_ATTITUDE, kAttitudeHz);

    std::cout << "\nFC YAW PROBE\n"
              << "5 s HOLD STILL, then rotate smoothly in one yaw direction toward 90 deg.\n"
              << "Keep dROLL/dPITCH inside +/-2 deg if possible.\n\n";

    bool have_ref = false;
    bool have_prev = false;
    Sample ref{}, prev{}, cur{};
    double yaw_accum = 0.0;
    double max_abs_roll = 0.0;
    double max_abs_pitch = 0.0;
    double max_abs_yaw_accum = 0.0;
    size_t attitude_count = 0;
    int64_t last_print = 0;

    const int64_t start = monoNs();
    const int64_t init_end = start + static_cast<int64_t>(kInitSec * 1e9);
    const int64_t end = init_end + static_cast<int64_t>(kRunSec * 1e9);

    while (monoNs() < end) {
      pollfd p{fd, POLLIN, 0};
      if (poll(&p, 1, 20) <= 0) continue;
      uint8_t b[4096];
      const ssize_t n = read(fd, b, sizeof(b));
      if (n <= 0) continue;
      for (ssize_t i = 0; i < n; ++i) {
        if (!mavlink_parse_char(MAVLINK_COMM_0, b[i], &msg, &status)) continue;
        if (msg.msgid != MAVLINK_MSG_ID_ATTITUDE) continue;

        mavlink_attitude_t a{};
        mavlink_msg_attitude_decode(&msg, &a);
        cur.t_ns = monoNs();
        cur.roll = a.roll * 180.0 / kPi;
        cur.pitch = a.pitch * 180.0 / kPi;
        cur.yaw = a.yaw * 180.0 / kPi;
        ++attitude_count;

        if (cur.t_ns < init_end) {
          ref = cur;
          prev = cur;
          have_prev = true;
          continue;
        }

        if (!have_ref) {
          ref = cur;
          prev = cur;
          have_prev = true;
          have_ref = true;
          yaw_accum = 0.0;
          std::cout << "[FC] reference captured: yaw=" << std::fixed << std::setprecision(2)
                    << ref.yaw << " roll=" << ref.roll << " pitch=" << ref.pitch << "\n";
        }

        if (have_prev) yaw_accum += wrap180(cur.yaw - prev.yaw);
        prev = cur;

        const double droll = wrap180(cur.roll - ref.roll);
        const double dpitch = wrap180(cur.pitch - ref.pitch);
        max_abs_roll = std::max(max_abs_roll, std::abs(droll));
        max_abs_pitch = std::max(max_abs_pitch, std::abs(dpitch));
        max_abs_yaw_accum = std::max(max_abs_yaw_accum, std::abs(yaw_accum));

        if (cur.t_ns - last_print >= 100000000LL) {
          last_print = cur.t_ns;
          const bool rp_bad = std::abs(droll) > kRpLimitDeg || std::abs(dpitch) > kRpLimitDeg;
          std::cout << "\rYAW_ACC=" << std::setw(7) << std::fixed << std::setprecision(2) << yaw_accum
                    << " deg  dROLL=" << std::setw(6) << droll
                    << "  dPITCH=" << std::setw(6) << dpitch
                    << "  " << (rp_bad ? "LEVEL!" : "OK    ")
                    << "  target left=" << std::max(0.0, kTargetDeg - std::abs(yaw_accum))
                    << "   " << std::flush;
        }
      }
    }

    requestRate(fd, sys, comp, MAVLINK_MSG_ID_ATTITUDE, 0);
    close(fd);
    fd = -1;

    const bool yaw_ok = max_abs_yaw_accum >= 85.0 && max_abs_yaw_accum <= 100.0;
    const bool rp_ok = max_abs_roll <= kRpLimitDeg && max_abs_pitch <= kRpLimitDeg;

    std::cout << "\n\n============================================================\n"
              << "JT-ZERO FC YAW PROBE RESULT\n"
              << "============================================================\n"
              << "ATTITUDE samples: " << attitude_count << "\n"
              << std::fixed << std::setprecision(3)
              << "final accumulated yaw: " << yaw_accum << " deg\n"
              << "max |accumulated yaw|: " << max_abs_yaw_accum << " deg\n"
              << "max |dRoll|: " << max_abs_roll << " deg\n"
              << "max |dPitch|: " << max_abs_pitch << " deg\n"
              << "yaw motion: " << (yaw_ok ? "OK" : "NOT 90 DEG") << "\n"
              << "roll/pitch contamination: " << (rp_ok ? "OK" : "TOO LARGE") << "\n";
    return (yaw_ok && rp_ok) ? 0 : 3;
  } catch (const std::exception& e) {
    if (fd >= 0) close(fd);
    std::cerr << "[FATAL] " << e.what() << "\n";
    return 1;
  }
}
