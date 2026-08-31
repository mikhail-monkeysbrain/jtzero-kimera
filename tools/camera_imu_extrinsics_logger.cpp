#include <linux/videodev2.h>

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "common/mavlink.h"
#include "camera_imu_timestamp_policy.hpp"

namespace {

constexpr const char* CAMERA_DEVICE = "/dev/video0";
constexpr const char* SERIAL_DEVICE = "/dev/ttyAMA0";
constexpr const char* OUTPUT_CSV = "/home/vio/camera_imu_extrinsics.csv";
constexpr const char* OUTPUT_MJPEG = "/home/vio/camera_imu_extrinsics.mjpg";
constexpr const char* OUTPUT_CAMERA_INDEX = "/home/vio/camera_imu_extrinsics_camera.csv";

constexpr int CAMERA_WIDTH = 640;
constexpr int CAMERA_HEIGHT = 480;
constexpr int CAMERA_FPS = 120;
constexpr int CAMERA_BUFFER_COUNT = 6;
constexpr int CAMERA_WARMUP_FRAMES = 30;

constexpr int EXPOSURE_ABSOLUTE = 50;
constexpr int CAMERA_GAIN = 0;

constexpr int IMU_RATE_HZ = 200;
constexpr double TIMESYNC_RATE_HZ = 10.0;
constexpr int64_t TIMESYNC_PERIOD_NS =
    static_cast<int64_t>(1.0e9 / TIMESYNC_RATE_HZ);
constexpr double MAX_TIMESYNC_RTT_MS = 10.0;
constexpr double DEFAULT_DURATION_SEC = 30.0;

constexpr uint8_t COMPANION_SYSID = 255;
constexpr uint8_t COMPANION_COMPID = 190;

struct CameraBuffer {
    void* start = nullptr;
    size_t length = 0;
};

struct CameraSample {
    int64_t recv_ns = 0;
    int64_t v4l2_ns = 0;
    int64_t corrected_ns = 0;
    uint32_t sequence = 0;
    uint32_t flags = 0;
    uint32_t bytes_used = 0;
    uint64_t mjpeg_offset = 0;
};

struct ImuSample {
    int64_t recv_ns = 0;
    int64_t fc_ns = 0;
    float xacc = 0.0f;
    float yacc = 0.0f;
    float zacc = 0.0f;
    float xgyro = 0.0f;
    float ygyro = 0.0f;
    float zgyro = 0.0f;
    float temperature = 0.0f;
    uint16_t fields_updated = 0;
    uint8_t imu_id = 0;
};

struct TimeSyncSample {
    int64_t t0_rpi_ns = 0;
    int64_t t1_rpi_ns = 0;
    int64_t fc_ns = 0;
    int64_t rpi_mid_ns = 0;
    int64_t rtt_ns = 0;
    bool good = false;
};

struct ClockMapping {
    bool valid = false;
    long double a = 1.0L;
    int64_t fc_ref_ns = 0;
    long double rpi_ref_ns = 0.0L;
    double drift_ppm = 0.0;
    size_t samples = 0;

    int64_t map(int64_t fc_ns) const {
        const long double d =
            static_cast<long double>(fc_ns - fc_ref_ns);
        return static_cast<int64_t>(
            std::llround(rpi_ref_ns + a * d));
    }
};

[[noreturn]] void fail(const std::string& text) {
    throw std::runtime_error(text + ": " + std::strerror(errno));
}

int xioctl(int fd, unsigned long request, void* arg) {
    int rc;
    do {
        rc = ioctl(fd, request, arg);
    } while (rc == -1 && errno == EINTR);
    return rc;
}

int64_t monotonicNs() {
    timespec ts{};
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        fail("clock_gettime");
    }
    return static_cast<int64_t>(ts.tv_sec) * 1000000000LL +
           static_cast<int64_t>(ts.tv_nsec);
}

int64_t timevalToNs(const timeval& tv) {
    return static_cast<int64_t>(tv.tv_sec) * 1000000000LL +
           static_cast<int64_t>(tv.tv_usec) * 1000LL;
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

std::string cameraTimestampFlags(uint32_t flags) {
    std::string s =
        ((flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) ==
         V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC)
            ? "monotonic"
            : "other_clock";

    s += "|";

    s +=
        ((flags & V4L2_BUF_FLAG_TSTAMP_SRC_MASK) ==
         V4L2_BUF_FLAG_TSTAMP_SRC_SOE)
            ? "soe"
            : "eof";

    return s;
}

void setCameraControl(
    int fd, uint32_t id, int32_t value, const char* name) {

    v4l2_control ctrl{};
    ctrl.id = id;
    ctrl.value = value;

    if (xioctl(fd, VIDIOC_S_CTRL, &ctrl) == -1) {
        std::cerr << "[WARN] " << name << "=" << value
                  << " failed: " << std::strerror(errno) << '\n';
    } else {
        std::cout << "[CAM] " << name << "=" << value << '\n';
    }
}

void configureCamera(int fd) {
    v4l2_capability cap{};
    if (xioctl(fd, VIDIOC_QUERYCAP, &cap) == -1)
        fail("VIDIOC_QUERYCAP");

    std::cout << "[CAM] driver=" << cap.driver
              << " card=" << cap.card << '\n';

    v4l2_format fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = CAMERA_WIDTH;
    fmt.fmt.pix.height = CAMERA_HEIGHT;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;

    if (xioctl(fd, VIDIOC_S_FMT, &fmt) == -1)
        fail("VIDIOC_S_FMT");

    if (fmt.fmt.pix.width != CAMERA_WIDTH ||
        fmt.fmt.pix.height != CAMERA_HEIGHT ||
        fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_MJPEG) {
        throw std::runtime_error("Camera rejected 640x480 MJPEG");
    }

    v4l2_streamparm parm{};
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = CAMERA_FPS;

    if (xioctl(fd, VIDIOC_S_PARM, &parm) == -1)
        fail("VIDIOC_S_PARM");

    const auto& tpf = parm.parm.capture.timeperframe;
    const double actual_fps =
        tpf.numerator
            ? static_cast<double>(tpf.denominator) /
                  static_cast<double>(tpf.numerator)
            : 0.0;

    std::cout << "[CAM] 640x480 MJPEG requested=120 actual="
              << std::fixed << std::setprecision(3)
              << actual_fps << " FPS\n";

    setCameraControl(fd, V4L2_CID_EXPOSURE_AUTO,
                     V4L2_EXPOSURE_MANUAL, "auto_exposure");
    setCameraControl(fd, V4L2_CID_EXPOSURE_AUTO_PRIORITY,
                     0, "dynamic_framerate");
    setCameraControl(fd, V4L2_CID_EXPOSURE_ABSOLUTE,
                     EXPOSURE_ABSOLUTE, "exposure_absolute");
    setCameraControl(fd, V4L2_CID_GAIN,
                     CAMERA_GAIN, "gain");
    setCameraControl(fd, V4L2_CID_AUTO_WHITE_BALANCE,
                     0, "white_balance_automatic");
    setCameraControl(fd, V4L2_CID_POWER_LINE_FREQUENCY,
                     V4L2_CID_POWER_LINE_FREQUENCY_DISABLED,
                     "power_line_frequency");
    setCameraControl(fd, V4L2_CID_BACKLIGHT_COMPENSATION,
                     0, "backlight_compensation");
}

std::vector<CameraBuffer> initCameraBuffers(int fd) {
    v4l2_requestbuffers req{};
    req.count = CAMERA_BUFFER_COUNT;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (xioctl(fd, VIDIOC_REQBUFS, &req) == -1)
        fail("VIDIOC_REQBUFS");

    if (req.count < 4)
        throw std::runtime_error("Not enough V4L2 buffers");

    std::vector<CameraBuffer> buffers(req.count);

    for (uint32_t i = 0; i < req.count; ++i) {
        v4l2_buffer buf{};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (xioctl(fd, VIDIOC_QUERYBUF, &buf) == -1)
            fail("VIDIOC_QUERYBUF");

        buffers[i].length = buf.length;
        buffers[i].start = mmap(
            nullptr, buf.length,
            PROT_READ | PROT_WRITE,
            MAP_SHARED, fd, buf.m.offset);

        if (buffers[i].start == MAP_FAILED)
            fail("mmap");

        if (xioctl(fd, VIDIOC_QBUF, &buf) == -1)
            fail("VIDIOC_QBUF");
    }

    return buffers;
}

void discardCameraWarmup(int fd, int count) {
    std::cout << "[CAM] discarding " << count
              << " warmup frames...\n";

    int n = 0;
    while (n < count) {
        pollfd pfd{};
        pfd.fd = fd;
        pfd.events = POLLIN;

        const int rc = poll(&pfd, 1, 1000);
        if (rc < 0) {
            if (errno == EINTR) continue;
            fail("poll camera warmup");
        }
        if (rc == 0)
            throw std::runtime_error("Camera warmup timeout");
        if (!(pfd.revents & POLLIN))
            continue;

        v4l2_buffer buf{};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (xioctl(fd, VIDIOC_DQBUF, &buf) == -1) {
            if (errno == EAGAIN) continue;
            fail("VIDIOC_DQBUF warmup");
        }

        ++n;

        if (xioctl(fd, VIDIOC_QBUF, &buf) == -1)
            fail("VIDIOC_QBUF warmup");
    }
}

int openSerial() {
    const int fd =
        open(SERIAL_DEVICE, O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (fd == -1)
        fail("open serial");

    termios tty{};
    if (tcgetattr(fd, &tty) != 0)
        fail("tcgetattr");

    cfmakeraw(&tty);

    if (cfsetispeed(&tty, B460800) != 0 ||
        cfsetospeed(&tty, B460800) != 0)
        fail("set baud");

    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
        fail("tcsetattr");

    tcflush(fd, TCIFLUSH);

    std::cout << "[MAV] " << SERIAL_DEVICE
              << " @ 460800\n";

    return fd;
}

void serialWriteAll(int fd, const uint8_t* data, size_t size) {
    size_t done = 0;

    while (done < size) {
        const ssize_t rc =
            write(fd, data + done, size - done);

        if (rc > 0) {
            done += static_cast<size_t>(rc);
            continue;
        }

        if (rc == -1 &&
            (errno == EAGAIN || errno == EWOULDBLOCK)) {
            pollfd pfd{};
            pfd.fd = fd;
            pfd.events = POLLOUT;
            poll(&pfd, 1, 10);
            continue;
        }

        if (rc == -1 && errno == EINTR)
            continue;

        fail("serial write");
    }
}

void sendMavlinkMessage(int fd, const mavlink_message_t& msg) {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len =
        mavlink_msg_to_send_buffer(buffer, &msg);
    serialWriteAll(fd, buffer, len);
}

void requestHighresImuRate(
    int fd,
    uint8_t target_system,
    uint8_t target_component,
    int rate_hz) {

    const float interval_us =
        rate_hz > 0
            ? static_cast<float>(1000000.0 / rate_hz)
            : 0.0f;

    mavlink_message_t msg{};

    mavlink_msg_command_long_pack(
        COMPANION_SYSID,
        COMPANION_COMPID,
        &msg,
        target_system,
        target_component,
        MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        MAVLINK_MSG_ID_HIGHRES_IMU,
        interval_us,
        0, 0, 0, 0, 0);

    sendMavlinkMessage(fd, msg);
}

void sendTimeSyncRequest(
    int fd,
    int64_t t0_ns,
    uint8_t target_system,
    uint8_t target_component) {

    mavlink_message_t msg{};

    // Current c_library_v2 TIMESYNC signature:
    // tc1, ts1, target_system, target_component
    mavlink_msg_timesync_pack(
        COMPANION_SYSID,
        COMPANION_COMPID,
        &msg,
        0,
        t0_ns,
        target_system,
        target_component);

    sendMavlinkMessage(fd, msg);
}

ClockMapping estimateClockMapping(
    const std::vector<TimeSyncSample>& samples) {

    std::vector<const TimeSyncSample*> good;
    for (const auto& s : samples)
        if (s.good) good.push_back(&s);

    ClockMapping m{};
    if (good.size() < 10)
        return m;

    const int64_t fc0 = good.front()->fc_ns;
    const int64_t rpi0 = good.front()->rpi_mid_ns;

    long double mean_x = 0.0L;
    long double mean_y = 0.0L;

    for (const auto* s : good) {
        mean_x +=
            static_cast<long double>(s->fc_ns - fc0);
        mean_y +=
            static_cast<long double>(s->rpi_mid_ns - rpi0);
    }

    mean_x /= static_cast<long double>(good.size());
    mean_y /= static_cast<long double>(good.size());

    long double sxx = 0.0L;
    long double sxy = 0.0L;

    for (const auto* s : good) {
        const long double x =
            static_cast<long double>(s->fc_ns - fc0);
        const long double y =
            static_cast<long double>(s->rpi_mid_ns - rpi0);
        const long double dx = x - mean_x;
        const long double dy = y - mean_y;
        sxx += dx * dx;
        sxy += dx * dy;
    }

    if (sxx <= 0.0L)
        return m;

    m.a = sxy / sxx;
    const long double c = mean_y - m.a * mean_x;

    m.valid = true;
    m.fc_ref_ns = fc0;
    m.rpi_ref_ns = static_cast<long double>(rpi0) + c;
    m.drift_ppm =
        static_cast<double>((m.a - 1.0L) * 1.0e6L);
    m.samples = good.size();

    return m;
}

} // namespace

int main(int argc, char** argv) {
    int camera_fd = -1;
    int serial_fd = -1;
    bool camera_streaming = false;
    bool imu_rate_requested = false;

    uint8_t target_system = 0;
    uint8_t target_component = 0;

    std::vector<CameraBuffer> camera_buffers;

    try {
        double duration_sec = DEFAULT_DURATION_SEC;

        if (argc >= 2)
            duration_sec = std::stod(argv[1]);

        if (duration_sec <= 0.0)
            throw std::runtime_error("Duration must be > 0");

        serial_fd = openSerial();

        mavlink_status_t mav_status{};
        mavlink_message_t mav_message{};

        std::cout << "[MAV] waiting for HEARTBEAT...\n";

        const int64_t heartbeat_deadline =
            monotonicNs() + 5000000000LL;

        bool have_heartbeat = false;

        while (monotonicNs() < heartbeat_deadline) {
            pollfd pfd{};
            pfd.fd = serial_fd;
            pfd.events = POLLIN;

            const int rc = poll(&pfd, 1, 100);

            if (rc < 0) {
                if (errno == EINTR) continue;
                fail("poll heartbeat");
            }

            if (rc == 0 || !(pfd.revents & POLLIN))
                continue;

            uint8_t bytes[4096];
            const ssize_t n =
                read(serial_fd, bytes, sizeof(bytes));

            if (n <= 0)
                continue;

            for (ssize_t i = 0; i < n; ++i) {
                if (!mavlink_parse_char(
                        MAVLINK_COMM_0,
                        bytes[i],
                        &mav_message,
                        &mav_status))
                    continue;

                if (mav_message.msgid ==
                    MAVLINK_MSG_ID_HEARTBEAT) {
                    target_system = mav_message.sysid;
                    target_component = mav_message.compid;
                    have_heartbeat = true;
                    break;
                }
            }

            if (have_heartbeat)
                break;
        }

        if (!have_heartbeat)
            throw std::runtime_error(
                "MAVLink HEARTBEAT not received");

        std::cout << "[MAV] connected system="
                  << static_cast<int>(target_system)
                  << " component="
                  << static_cast<int>(target_component)
                  << '\n';

        requestHighresImuRate(
            serial_fd,
            target_system,
            target_component,
            IMU_RATE_HZ);

        imu_rate_requested = true;

        std::cout << "[MAV] HIGHRES_IMU requested @ "
                  << IMU_RATE_HZ << " Hz\n";

        camera_fd =
            open(CAMERA_DEVICE, O_RDWR | O_NONBLOCK);

        if (camera_fd == -1)
            fail("open camera");

        configureCamera(camera_fd);
        camera_buffers = initCameraBuffers(camera_fd);

        v4l2_buf_type camera_type =
            V4L2_BUF_TYPE_VIDEO_CAPTURE;

        if (xioctl(
                camera_fd,
                VIDIOC_STREAMON,
                &camera_type) == -1)
            fail("VIDIOC_STREAMON");

        camera_streaming = true;

        discardCameraWarmup(
            camera_fd,
            CAMERA_WARMUP_FRAMES);

        tcflush(serial_fd, TCIFLUSH);
        std::memset(&mav_status, 0, sizeof(mav_status));
        std::memset(&mav_message, 0, sizeof(mav_message));

        std::vector<CameraSample> camera_samples;
        std::vector<ImuSample> imu_samples;
        std::vector<TimeSyncSample> timesync_samples;

        std::ofstream mjpeg(
            OUTPUT_MJPEG,
            std::ios::binary | std::ios::trunc);

        if (!mjpeg) {
            throw std::runtime_error(
                "Cannot create MJPEG output");
        }

        camera_samples.reserve(
            static_cast<size_t>(
                duration_sec * CAMERA_FPS + 100));

        imu_samples.reserve(
            static_cast<size_t>(
                duration_sec * IMU_RATE_HZ + 100));

        timesync_samples.reserve(
            static_cast<size_t>(
                duration_sec * TIMESYNC_RATE_HZ + 100));

        uint64_t camera_source_drops = 0;
        bool have_camera_sequence = false;
        uint32_t previous_camera_sequence = 0;

        int64_t pending_timesync_t0 = 0;

        const int64_t start_ns = monotonicNs();
        const int64_t end_ns =
            start_ns +
            static_cast<int64_t>(duration_sec * 1.0e9);

        int64_t next_timesync_ns = start_ns;

        std::cout
            << "\n=== CAMERA + IMU + TIMESYNC LOGGER ===\n"
            << "duration:       " << duration_sec << " s\n"
            << "camera:         640x480 MJPEG @ 120 FPS\n"
            << "IMU:            HIGHRES_IMU @ 200 Hz\n"
            << "TIMESYNC:       10 Hz\n"
            << "clock:          CLOCK_MONOTONIC\n"
            << "camera offset:  +"
            << jtzero::timesync::cameraToImuCorrectionMs()
            << " ms\n"
            << "output:         " << OUTPUT_CSV << "\n\n";

        while (monotonicNs() < end_ns) {
            const int64_t now_ns = monotonicNs();

            if (now_ns >= next_timesync_ns &&
                pending_timesync_t0 == 0) {

                const int64_t t0 = monotonicNs();

                sendTimeSyncRequest(
                    serial_fd,
                    t0,
                    target_system,
                    target_component);

                pending_timesync_t0 = t0;
                next_timesync_ns =
                    t0 + TIMESYNC_PERIOD_NS;
            }

            pollfd pfds[2]{};
            pfds[0].fd = camera_fd;
            pfds[0].events = POLLIN;
            pfds[1].fd = serial_fd;
            pfds[1].events = POLLIN;

            const int rc = poll(pfds, 2, 2);

            if (rc < 0) {
                if (errno == EINTR) continue;
                fail("combined poll");
            }

            if (pfds[0].revents & POLLIN) {
                while (true) {
                    v4l2_buffer buf{};
                    buf.type =
                        V4L2_BUF_TYPE_VIDEO_CAPTURE;
                    buf.memory =
                        V4L2_MEMORY_MMAP;

                    if (xioctl(
                            camera_fd,
                            VIDIOC_DQBUF,
                            &buf) == -1) {
                        if (errno == EAGAIN)
                            break;
                        fail("VIDIOC_DQBUF");
                    }

                    const int64_t recv_ns =
                        monotonicNs();

                    const int64_t v4l2_ns =
                        timevalToNs(buf.timestamp);

                    const int64_t corrected_ns =
                        jtzero::timesync::correctCameraTimestampNs(
                            v4l2_ns);

                    if (have_camera_sequence) {
                        const uint32_t expected =
                            previous_camera_sequence + 1;

                        if (buf.sequence != expected) {
                            camera_source_drops +=
                                static_cast<uint32_t>(
                                    buf.sequence - expected);
                        }
                    }

                    previous_camera_sequence =
                        buf.sequence;

                    have_camera_sequence = true;

                    if (buf.index >= camera_buffers.size()) {
                        throw std::runtime_error(
                            "Invalid V4L2 buffer index");
                    }

                    if (buf.bytesused == 0 ||
                        buf.bytesused >
                            camera_buffers[buf.index].length) {
                        throw std::runtime_error(
                            "Invalid MJPEG buffer size");
                    }

                    const std::streampos file_pos =
                        mjpeg.tellp();

                    if (file_pos < 0) {
                        throw std::runtime_error(
                            "MJPEG tellp failed");
                    }

                    const uint64_t mjpeg_offset =
                        static_cast<uint64_t>(file_pos);

                    mjpeg.write(
                        static_cast<const char*>(
                            camera_buffers[buf.index].start),
                        static_cast<std::streamsize>(
                            buf.bytesused));

                    if (!mjpeg) {
                        throw std::runtime_error(
                            "MJPEG write failed");
                    }

                    camera_samples.push_back({
                        recv_ns,
                        v4l2_ns,
                        corrected_ns,
                        buf.sequence,
                        buf.flags,
                        buf.bytesused,
                        mjpeg_offset
                    });

                    if (xioctl(
                            camera_fd,
                            VIDIOC_QBUF,
                            &buf) == -1)
                        fail("VIDIOC_QBUF");
                }
            }

            if (pfds[1].revents & POLLIN) {
                uint8_t bytes[8192];

                while (true) {
                    const ssize_t n =
                        read(serial_fd, bytes, sizeof(bytes));

                    if (n == -1) {
                        if (errno == EAGAIN ||
                            errno == EWOULDBLOCK)
                            break;

                        if (errno == EINTR)
                            continue;

                        fail("serial read");
                    }

                    if (n == 0)
                        break;

                    for (ssize_t i = 0; i < n; ++i) {
                        if (!mavlink_parse_char(
                                MAVLINK_COMM_0,
                                bytes[i],
                                &mav_message,
                                &mav_status))
                            continue;

                        const int64_t recv_ns =
                            monotonicNs();

                        if (mav_message.msgid ==
                            MAVLINK_MSG_ID_HIGHRES_IMU) {

                            mavlink_highres_imu_t imu{};
                            mavlink_msg_highres_imu_decode(
                                &mav_message, &imu);

                            ImuSample s{};
                            s.recv_ns = recv_ns;
                            s.fc_ns =
                                static_cast<int64_t>(
                                    imu.time_usec) * 1000LL;
                            s.xacc = imu.xacc;
                            s.yacc = imu.yacc;
                            s.zacc = imu.zacc;
                            s.xgyro = imu.xgyro;
                            s.ygyro = imu.ygyro;
                            s.zgyro = imu.zgyro;
                            s.temperature = imu.temperature;
                            s.fields_updated =
                                imu.fields_updated;

#ifdef MAVLINK_MSG_HIGHRES_IMU_FIELD_ID_LEN
                            s.imu_id = imu.id;
#else
                            s.imu_id = 0;
#endif

                            imu_samples.push_back(s);
                        }
                        else if (mav_message.msgid ==
                                 MAVLINK_MSG_ID_TIMESYNC) {

                            mavlink_timesync_t ts{};
                            mavlink_msg_timesync_decode(
                                &mav_message, &ts);

                            // ArduPilot response in the currently tested
                            // MAVLink dialect:
                            //   ts1 = echoed RPi request timestamp
                            //   tc1 = FC timestamp
                            if (ts.tc1 != 0 &&
                                pending_timesync_t0 != 0 &&
                                ts.ts1 ==
                                    pending_timesync_t0) {

                                TimeSyncSample s{};
                                s.t0_rpi_ns =
                                    pending_timesync_t0;
                                s.t1_rpi_ns = recv_ns;
                                s.fc_ns = ts.tc1;
                                s.rtt_ns =
                                    s.t1_rpi_ns -
                                    s.t0_rpi_ns;
                                s.rpi_mid_ns =
                                    s.t0_rpi_ns +
                                    s.rtt_ns / 2;

                                s.good =
                                    s.rtt_ns > 0 &&
                                    nsToMs(s.rtt_ns) <=
                                        MAX_TIMESYNC_RTT_MS;

                                timesync_samples.push_back(s);
                                pending_timesync_t0 = 0;
                            }
                        }
                    }
                }
            }

            if (pending_timesync_t0 != 0 &&
                monotonicNs() - pending_timesync_t0 >
                    20000000LL) {
                pending_timesync_t0 = 0;
            }
        }

        mjpeg.flush();
        mjpeg.close();

        const ClockMapping mapping =
            estimateClockMapping(timesync_samples);

        if (!mapping.valid)
            throw std::runtime_error(
                "Not enough valid TIMESYNC samples");

        std::vector<double> camera_latency_ms;
        std::vector<double> camera_dt_ms;
        std::vector<double> imu_transport_ms;
        std::vector<double> timesync_rtt_ms;

        for (size_t i = 0;
             i < camera_samples.size();
             ++i) {

            camera_latency_ms.push_back(
                nsToMs(
                    camera_samples[i].recv_ns -
                    camera_samples[i].v4l2_ns));

            if (i > 0) {
                camera_dt_ms.push_back(
                    nsToMs(
                        camera_samples[i].v4l2_ns -
                        camera_samples[i - 1].v4l2_ns));
            }
        }

        for (const auto& s : imu_samples) {
            const int64_t mapped_ns =
                mapping.map(s.fc_ns);

            imu_transport_ms.push_back(
                nsToMs(s.recv_ns - mapped_ns));
        }

        size_t good_timesync = 0;

        for (const auto& s : timesync_samples) {
            if (s.good) {
                ++good_timesync;
                timesync_rtt_ms.push_back(
                    nsToMs(s.rtt_ns));
            }
        }

        std::ofstream camera_index(OUTPUT_CAMERA_INDEX);

        if (!camera_index)
            throw std::runtime_error(
                "Cannot create camera index CSV");

        camera_index
            << "sequence,v4l2_timestamp_ns,camera_timestamp_corrected_ns,"
            << "recv_rpi_ns,delivery_latency_ms,mjpeg_offset,bytes_used,flags\n";

        camera_index
            << std::fixed
            << std::setprecision(9);

        for (const auto& s : camera_samples) {
            camera_index
                << s.sequence << ','
                << s.v4l2_ns << ','
                << s.corrected_ns << ','
                << s.recv_ns << ','
                << nsToMs(s.recv_ns - s.v4l2_ns) << ','
                << s.mjpeg_offset << ','
                << s.bytes_used << ','
                << cameraTimestampFlags(s.flags)
                << '\n';
        }

        camera_index.close();

        std::ofstream csv(OUTPUT_CSV);

        if (!csv)
            throw std::runtime_error(
                "Cannot create output CSV");

        csv
            << "event,recv_rpi_ns,source_timestamp_ns,"
            << "mapped_rpi_ns,transport_latency_ms,"
            << "camera_sequence,camera_flags,camera_bytes,"
            << "xacc_m_s2,yacc_m_s2,zacc_m_s2,"
            << "xgyro_rad_s,ygyro_rad_s,zgyro_rad_s,"
            << "temperature_c,imu_id,"
            << "timesync_t0_rpi_ns,timesync_t1_rpi_ns,"
            << "timesync_fc_ns,timesync_mid_rpi_ns,"
            << "timesync_rtt_ms,timesync_good,"
            << "map_a,map_drift_ppm,"
            << "map_fc_ref_ns,map_rpi_ref_ns\n";

        csv << std::fixed << std::setprecision(9);

        for (const auto& s : camera_samples) {
            csv
                << "CAMERA,"
                << s.recv_ns << ','
                << s.v4l2_ns << ','
                << s.corrected_ns << ','
                << nsToMs(s.recv_ns - s.v4l2_ns) << ','
                << s.sequence << ','
                << cameraTimestampFlags(s.flags) << ','
                << s.bytes_used
                << ",,,,,,,,,,,,,,"
                << static_cast<double>(mapping.a) << ','
                << mapping.drift_ppm << ','
                << mapping.fc_ref_ns << ','
                << static_cast<int64_t>(
                       std::llround(mapping.rpi_ref_ns))
                << '\n';
        }

        for (const auto& s : imu_samples) {
            const int64_t mapped_ns =
                mapping.map(s.fc_ns);

            csv
                << "IMU,"
                << s.recv_ns << ','
                << s.fc_ns << ','
                << mapped_ns << ','
                << nsToMs(s.recv_ns - mapped_ns)
                << ",,,,"
                << s.xacc << ','
                << s.yacc << ','
                << s.zacc << ','
                << s.xgyro << ','
                << s.ygyro << ','
                << s.zgyro << ','
                << s.temperature << ','
                << static_cast<int>(s.imu_id)
                << ",,,,,,"
                << static_cast<double>(mapping.a) << ','
                << mapping.drift_ppm << ','
                << mapping.fc_ref_ns << ','
                << static_cast<int64_t>(
                       std::llround(mapping.rpi_ref_ns))
                << '\n';
        }

        for (const auto& s : timesync_samples) {
            csv
                << "TIMESYNC,"
                << s.t1_rpi_ns
                << ",,,,,,,,,,,,,,,"
                << s.t0_rpi_ns << ','
                << s.t1_rpi_ns << ','
                << s.fc_ns << ','
                << s.rpi_mid_ns << ','
                << nsToMs(s.rtt_ns) << ','
                << (s.good ? 1 : 0) << ','
                << static_cast<double>(mapping.a) << ','
                << mapping.drift_ppm << ','
                << mapping.fc_ref_ns << ','
                << static_cast<int64_t>(
                       std::llround(mapping.rpi_ref_ns))
                << '\n';
        }

        csv.close();

        std::cout
            << "\n============================================================\n"
            << "FINAL CLOCK MAPPING\n"
            << "============================================================\n"
            << std::setprecision(12)
            << "A (RPi/FC):           "
            << static_cast<double>(mapping.a) << '\n'
            << std::setprecision(3)
            << "drift:                "
            << mapping.drift_ppm << " ppm\n"
            << "TIMESYNC good:         "
            << good_timesync << "/"
            << timesync_samples.size() << '\n';

        if (!timesync_rtt_ms.empty()) {
            std::cout
                << "TIMESYNC RTT median:   "
                << percentile(timesync_rtt_ms, 0.50)
                << " ms\n"
                << "TIMESYNC RTT p95:      "
                << percentile(timesync_rtt_ms, 0.95)
                << " ms\n";
        }

        std::cout
            << "\n============================================================\n"
            << "CAMERA\n"
            << "============================================================\n"
            << "frames:                "
            << camera_samples.size() << '\n'
            << "source drops:          "
            << camera_source_drops << '\n';

        if (!camera_latency_ms.empty()) {
            std::cout
                << "delivery median:       "
                << percentile(camera_latency_ms, 0.50)
                << " ms\n"
                << "delivery p95:          "
                << percentile(camera_latency_ms, 0.95)
                << " ms\n"
                << "delivery p99:          "
                << percentile(camera_latency_ms, 0.99)
                << " ms\n"
                << "delivery max:          "
                << *std::max_element(
                       camera_latency_ms.begin(),
                       camera_latency_ms.end())
                << " ms\n";
        }

        if (!camera_dt_ms.empty()) {
            std::cout
                << "timestamp dt median:   "
                << percentile(camera_dt_ms, 0.50)
                << " ms\n"
                << "timestamp dt p95:      "
                << percentile(camera_dt_ms, 0.95)
                << " ms\n";
        }

        std::cout
            << "\n============================================================\n"
            << "IMU\n"
            << "============================================================\n"
            << "samples:               "
            << imu_samples.size() << '\n';

        if (!imu_transport_ms.empty()) {
            std::cout
                << "transport median:      "
                << percentile(imu_transport_ms, 0.50)
                << " ms\n"
                << "transport p95:         "
                << percentile(imu_transport_ms, 0.95)
                << " ms\n"
                << "transport p99:         "
                << percentile(imu_transport_ms, 0.99)
                << " ms\n"
                << "transport min:         "
                << *std::min_element(
                       imu_transport_ms.begin(),
                       imu_transport_ms.end())
                << " ms\n"
                << "transport max:         "
                << *std::max_element(
                       imu_transport_ms.begin(),
                       imu_transport_ms.end())
                << " ms\n";
        }

        std::cout
            << "\nCombined CSV:          "
            << OUTPUT_CSV << '\n'
            << "Camera index CSV:      "
            << OUTPUT_CAMERA_INDEX << '\n'
            << "Camera MJPEG:          "
            << OUTPUT_MJPEG << '\n';

        if (imu_rate_requested) {
            requestHighresImuRate(
                serial_fd,
                target_system,
                target_component,
                0);
            imu_rate_requested = false;
        }

        if (camera_streaming) {
            v4l2_buf_type type =
                V4L2_BUF_TYPE_VIDEO_CAPTURE;
            xioctl(
                camera_fd,
                VIDIOC_STREAMOFF,
                &type);
            camera_streaming = false;
        }

        for (auto& b : camera_buffers) {
            if (b.start && b.start != MAP_FAILED)
                munmap(b.start, b.length);
        }

        if (camera_fd != -1)
            close(camera_fd);

        if (serial_fd != -1)
            close(serial_fd);

        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "\n[FATAL] " << e.what() << '\n';

        if (serial_fd != -1 &&
            imu_rate_requested &&
            target_system != 0) {
            try {
                requestHighresImuRate(
                    serial_fd,
                    target_system,
                    target_component,
                    0);
            } catch (...) {}
        }

        if (camera_fd != -1 &&
            camera_streaming) {
            v4l2_buf_type type =
                V4L2_BUF_TYPE_VIDEO_CAPTURE;
            xioctl(
                camera_fd,
                VIDIOC_STREAMOFF,
                &type);
        }

        for (auto& b : camera_buffers) {
            if (b.start && b.start != MAP_FAILED)
                munmap(b.start, b.length);
        }

        if (camera_fd != -1)
            close(camera_fd);

        if (serial_fd != -1)
            close(serial_fd);

        return 1;
    }
}
