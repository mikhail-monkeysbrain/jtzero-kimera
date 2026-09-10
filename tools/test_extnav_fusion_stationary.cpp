// JT-Zero: диагностический тест fusion ExternalNav без физического движения.
// БПЛА должен оставаться неподвижным и DISARMED.
// Профиль VISION_SPEED_ESTIMATE (NED North):
//   0..3 c   :  0.00 m/s
//   3..7 c   : +0.15 m/s
//   7..11 c  : -0.15 m/s
//   11..14 c :  0.00 m/s
// Одновременно читается GLOBAL_POSITION_INT.vx/vy от FC.

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <poll.h>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

#include "common/mavlink.h"

namespace {

using Clock = std::chrono::steady_clock;

uint64_t monoUs() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(Clock::now().time_since_epoch()).count());
}

double elapsedSec(const Clock::time_point& t0) {
    return std::chrono::duration<double>(Clock::now() - t0).count();
}

bool writeMessage(int fd, const mavlink_message_t& msg) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t n = mavlink_msg_to_send_buffer(buf, &msg);
    size_t off = 0;
    while (off < n) {
        const ssize_t k = ::write(fd, buf + off, n - off);
        if (k > 0) {
            off += static_cast<size_t>(k);
            continue;
        }
        if (k < 0 && errno == EINTR) continue;
        if (k < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            pollfd p{fd, POLLOUT, 0};
            (void)::poll(&p, 1, 10);
            continue;
        }
        return false;
    }
    return true;
}

int openSerial(const std::string& dev) {
    const int fd = ::open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return -1;

    termios t{};
    if (tcgetattr(fd, &t) < 0) {
        ::close(fd);
        return -1;
    }
    cfmakeraw(&t);
    cfsetispeed(&t, B460800);
    cfsetospeed(&t, B460800);
    t.c_cflag |= CLOCAL | CREAD;
    t.c_cflag &= ~CRTSCTS;
    t.c_cflag &= ~PARENB;
    t.c_cflag &= ~CSTOPB;
    t.c_cflag &= ~CSIZE;
    t.c_cflag |= CS8;
    if (tcsetattr(fd, TCSANOW, &t) < 0) {
        ::close(fd);
        return -1;
    }
    tcflush(fd, TCIFLUSH);
    return fd;
}

void requestRate(int fd, uint8_t target_sys, uint8_t target_comp, uint32_t msgid, int hz) {
    constexpr uint8_t self_sys = 191;
    constexpr uint8_t self_comp = MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY;
    mavlink_message_t m{};
    mavlink_msg_command_long_pack(
        self_sys, self_comp, &m,
        target_sys, target_comp,
        MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        static_cast<float>(msgid), 1000000.0f / static_cast<float>(hz),
        0, 0, 0, 0, 0);
    (void)writeMessage(fd, m);
}

void sendVisionSpeed(int fd, float vn) {
    constexpr uint8_t self_sys = 191;
    constexpr uint8_t self_comp = MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY;
    constexpr float var_v = 0.01f; // sigma = 0.10 m/s, как production publisher
    float covariance[9] = {
        var_v, 0.0f, 0.0f,
        0.0f, var_v, 0.0f,
        0.0f, 0.0f, var_v
    };

    mavlink_message_t m{};
    mavlink_msg_vision_speed_estimate_pack(
        self_sys, self_comp, &m,
        monoUs(), vn, 0.0f, 0.0f,
        covariance, 0);
    (void)writeMessage(fd, m);
}

int phaseFor(double t) {
    if (t < 3.0) return 0;
    if (t < 7.0) return 1;
    if (t < 11.0) return 2;
    return 3;
}

float commandedVn(int phase) {
    if (phase == 1) return +0.15f;
    if (phase == 2) return -0.15f;
    return 0.0f;
}

const char* phaseName(int phase) {
    switch (phase) {
        case 0: return "ZERO_PRE";
        case 1: return "PLUS_015";
        case 2: return "MINUS_015";
        default: return "ZERO_POST";
    }
}

struct Stats {
    int n = 0;
    double sum_vn = 0.0;
    double sum_ve = 0.0;
    double max_abs_vn = 0.0;
};

} // namespace

int main(int argc, char** argv) {
    const std::string dev = (argc >= 2) ? argv[1] : "/dev/ttyAMA0";
    const int fd = openSerial(dev);
    if (fd < 0) {
        std::cerr << "ОШИБКА: не удалось открыть " << dev << ": " << std::strerror(errno) << "\n";
        return 2;
    }

    std::cerr << "JT-ZERO ExternalNav fusion stationary test\n";
    std::cerr << "FC=" << dev << "\n";
    std::cerr << "ВАЖНО: БПЛА НЕ ДВИГАТЬ И НЕ ARM.\n";

    mavlink_status_t st{};
    mavlink_message_t msg{};
    uint8_t buf[4096];
    uint8_t target_sys = 0, target_comp = 0;
    const auto hb_start = Clock::now();

    while (elapsedSec(hb_start) < 8.0 && target_sys == 0) {
        pollfd p{fd, POLLIN, 0};
        if (::poll(&p, 1, 100) <= 0) continue;
        const ssize_t n = ::read(fd, buf, sizeof(buf));
        if (n <= 0) continue;
        for (ssize_t i = 0; i < n; ++i) {
            if (!mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &st)) continue;
            if (msg.msgid != MAVLINK_MSG_ID_HEARTBEAT) continue;
            mavlink_heartbeat_t hb{};
            mavlink_msg_heartbeat_decode(&msg, &hb);
            if (hb.autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA) {
                target_sys = msg.sysid;
                target_comp = msg.compid;
                break;
            }
        }
    }

    if (target_sys == 0) {
        std::cerr << "ОШИБКА: heartbeat ArduPilot не найден\n";
        ::close(fd);
        return 3;
    }

    std::cerr << "FC heartbeat sys=" << static_cast<int>(target_sys)
              << " comp=" << static_cast<int>(target_comp) << "\n";
    requestRate(fd, target_sys, target_comp, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 20);

    Stats stats[4];
    const auto t0 = Clock::now();
    auto next_tx = t0;
    auto next_print = t0;
    int last_phase = -1;

    while (true) {
        const double t = elapsedSec(t0);
        if (t >= 14.0) break;
        const int phase = phaseFor(t);
        const float cmd = commandedVn(phase);

        if (phase != last_phase) {
            std::cerr << "\nPHASE " << phaseName(phase) << " cmd_vN=" << cmd << " m/s\n";
            last_phase = phase;
        }

        const auto now = Clock::now();
        if (now >= next_tx) {
            sendVisionSpeed(fd, cmd);
            next_tx += std::chrono::milliseconds(50); // 20 Hz
        }

        pollfd p{fd, POLLIN, 0};
        if (::poll(&p, 1, 5) > 0) {
            for (;;) {
                const ssize_t n = ::read(fd, buf, sizeof(buf));
                if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
                if (n <= 0) break;
                for (ssize_t i = 0; i < n; ++i) {
                    if (!mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &st)) continue;
                    if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT && msg.sysid == target_sys) {
                        mavlink_global_position_int_t gp{};
                        mavlink_msg_global_position_int_decode(&msg, &gp);
                        const double vn = static_cast<double>(gp.vx) * 0.01;
                        const double ve = static_cast<double>(gp.vy) * 0.01;
                        Stats& s = stats[phase];
                        ++s.n;
                        s.sum_vn += vn;
                        s.sum_ve += ve;
                        s.max_abs_vn = std::max(s.max_abs_vn, std::abs(vn));
                    }
                }
            }
        }

        if (now >= next_print) {
            std::cout << "t=" << t << " phase=" << phaseName(phase)
                      << " cmd_vN=" << cmd
                      << " samples=" << stats[phase].n;
            if (stats[phase].n > 0) {
                std::cout << " mean_fc_vN=" << stats[phase].sum_vn / stats[phase].n;
            }
            std::cout << "\n";
            next_print += std::chrono::seconds(1);
        }
    }

    // После теста несколько нулевых измерений, чтобы не оставлять ненулевую команду.
    for (int i = 0; i < 10; ++i) {
        sendVisionSpeed(fd, 0.0f);
        usleep(50000);
    }

    std::cout << "\n=== RESULT ===\n";
    for (int phase = 0; phase < 4; ++phase) {
        const Stats& s = stats[phase];
        std::cout << phaseName(phase) << " n=" << s.n;
        if (s.n > 0) {
            std::cout << " mean_fc_vN=" << (s.sum_vn / s.n)
                      << " mean_fc_vE=" << (s.sum_ve / s.n)
                      << " max|fc_vN|=" << s.max_abs_vn;
        }
        std::cout << "\n";
    }

    ::close(fd);
    return 0;
}
