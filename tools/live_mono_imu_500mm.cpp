#include <linux/videodev2.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <termios.h>
#include <unistd.h>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <future>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

#include "common/mavlink.h"
#include "camera_imu_timestamp_policy.hpp"

#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/imu-frontend/ImuFrontend-definitions.h"
#include "kimera-vio/pipeline/MonoImuPipeline.h"
#include "kimera-vio/pipeline/Pipeline-definitions.h"

DECLARE_bool(visualize);
DECLARE_int32(viz_type);
DECLARE_bool(use_lcd);
DECLARE_bool(log_output);
DECLARE_bool(extract_planes_from_the_scene);

namespace {

constexpr const char* kCameraDevice = "/dev/video0";
constexpr const char* kSerialDevice = "/dev/ttyAMA0";
constexpr const char* kCsvPath = "/home/vio/jtzero_live_500mm.csv";
constexpr int kWidth = 640;
constexpr int kHeight = 480;
constexpr int kCameraFps = 120;
constexpr int kCameraBuffers = 6;
constexpr int kWarmupFrames = 30;
constexpr int kImuRateHz = 200;
constexpr int kExposureAbsolute = 50;
constexpr int kGain = 0;
constexpr uint8_t kCompanionSysId = 255;
constexpr uint8_t kCompanionCompId = 190;
constexpr int64_t kTimesyncPeriodNs = 100000000LL;
constexpr int64_t kMappingStaleNs = 2000000000LL;
constexpr double kMaxTimesyncRttMs = 10.0;
constexpr double kStartPhaseSec = 10.0;
constexpr double kMovePhaseSec = 15.0;
constexpr double kEndPhaseSec = 10.0;
constexpr double kAverageWindowSec = 5.0;
constexpr double kExpectedDistanceM = 0.500;
constexpr double kPi = 3.14159265358979323846;

struct CameraBuffer {
  void* start = nullptr;
  size_t length = 0;
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
  long double rpi_ref_ns = 0;
  double drift_ppm = 0.0;
  int64_t last_update_ns = 0;

  int64_t map(int64_t fc_ns) const {
    return static_cast<int64_t>(std::llround(
        rpi_ref_ns + a * static_cast<long double>(fc_ns - fc_ref_ns)));
  }
};

struct VioState {
  int64_t timestamp_ns = 0;
  int64_t keyframe = 0;
  double px = 0.0, py = 0.0, pz = 0.0;
  double vx = 0.0, vy = 0.0, vz = 0.0;
  double roll_deg = 0.0, pitch_deg = 0.0, yaw_deg = 0.0;
};

struct MeanState {
  bool valid = false;
  size_t count = 0;
  double px = 0.0, py = 0.0, pz = 0.0;
  double vx = 0.0, vy = 0.0, vz = 0.0;
  double roll_deg = 0.0, pitch_deg = 0.0, yaw_deg = 0.0;
};

[[noreturn]] void fail(const std::string& s) {
  throw std::runtime_error(s + ": " + std::strerror(errno));
}

int xioctl(int fd, unsigned long req, void* arg) {
  int r;
  do {
    r = ioctl(fd, req, arg);
  } while (r == -1 && errno == EINTR);
  return r;
}

int64_t monotonicNs() {
  timespec t{};
  if (clock_gettime(CLOCK_MONOTONIC, &t) != 0) fail("clock_gettime");
  return static_cast<int64_t>(t.tv_sec) * 1000000000LL + t.tv_nsec;
}

int64_t timevalToNs(const timeval& t) {
  return static_cast<int64_t>(t.tv_sec) * 1000000000LL +
         static_cast<int64_t>(t.tv_usec) * 1000LL;
}

double nsToMs(int64_t ns) { return static_cast<double>(ns) / 1e6; }

double wrapDeg(double d) {
  while (d > 180.0) d -= 360.0;
  while (d < -180.0) d += 360.0;
  return d;
}

void setCameraControl(int fd, uint32_t id, int32_t value) {
  v4l2_control c{};
  c.id = id;
  c.value = value;
  xioctl(fd, VIDIOC_S_CTRL, &c);
}

void configureCamera(int fd) {
  v4l2_format fmt{};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = kWidth;
  fmt.fmt.pix.height = kHeight;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
  fmt.fmt.pix.field = V4L2_FIELD_ANY;
  if (xioctl(fd, VIDIOC_S_FMT, &fmt) == -1) fail("VIDIOC_S_FMT");

  v4l2_streamparm p{};
  p.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  p.parm.capture.timeperframe.numerator = 1;
  p.parm.capture.timeperframe.denominator = kCameraFps;
  if (xioctl(fd, VIDIOC_S_PARM, &p) == -1) fail("VIDIOC_S_PARM");

  setCameraControl(fd, V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL);
  setCameraControl(fd, V4L2_CID_EXPOSURE_AUTO_PRIORITY, 0);
  setCameraControl(fd, V4L2_CID_EXPOSURE_ABSOLUTE, kExposureAbsolute);
  setCameraControl(fd, V4L2_CID_GAIN, kGain);
  setCameraControl(fd, V4L2_CID_AUTO_WHITE_BALANCE, 0);
  setCameraControl(fd, V4L2_CID_POWER_LINE_FREQUENCY,
                   V4L2_CID_POWER_LINE_FREQUENCY_DISABLED);
  setCameraControl(fd, V4L2_CID_BACKLIGHT_COMPENSATION, 0);
}

std::vector<CameraBuffer> initCameraBuffers(int fd) {
  v4l2_requestbuffers r{};
  r.count = kCameraBuffers;
  r.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  r.memory = V4L2_MEMORY_MMAP;
  if (xioctl(fd, VIDIOC_REQBUFS, &r) == -1) fail("VIDIOC_REQBUFS");

  std::vector<CameraBuffer> buffers(r.count);
  for (uint32_t i = 0; i < r.count; ++i) {
    v4l2_buffer b{};
    b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    b.memory = V4L2_MEMORY_MMAP;
    b.index = i;
    if (xioctl(fd, VIDIOC_QUERYBUF, &b) == -1) fail("VIDIOC_QUERYBUF");
    buffers[i].length = b.length;
    buffers[i].start = mmap(nullptr, b.length, PROT_READ | PROT_WRITE,
                            MAP_SHARED, fd, b.m.offset);
    if (buffers[i].start == MAP_FAILED) fail("mmap");
    if (xioctl(fd, VIDIOC_QBUF, &b) == -1) fail("VIDIOC_QBUF");
  }
  return buffers;
}

void discardWarmup(int fd) {
  for (int n = 0; n < kWarmupFrames;) {
    pollfd p{fd, POLLIN, 0};
    if (poll(&p, 1, 1000) <= 0) continue;
    v4l2_buffer b{};
    b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    b.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd, VIDIOC_DQBUF, &b) == -1) {
      if (errno == EAGAIN) continue;
      fail("DQBUF warmup");
    }
    ++n;
    if (xioctl(fd, VIDIOC_QBUF, &b) == -1) fail("QBUF warmup");
  }
}

int openSerial() {
  int fd = open(kSerialDevice, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd == -1) fail("open serial");
  termios t{};
  if (tcgetattr(fd, &t) != 0) fail("tcgetattr");
  cfmakeraw(&t);
  if (cfsetispeed(&t, B460800) || cfsetospeed(&t, B460800)) fail("baud");
  t.c_cflag |= CLOCAL | CREAD;
  t.c_cflag &= ~CRTSCTS;
  t.c_cflag &= ~PARENB;
  t.c_cflag &= ~CSTOPB;
  t.c_cflag &= ~CSIZE;
  t.c_cflag |= CS8;
  t.c_cc[VMIN] = 0;
  t.c_cc[VTIME] = 0;
  if (tcsetattr(fd, TCSANOW, &t) != 0) fail("tcsetattr");
  tcflush(fd, TCIFLUSH);
  return fd;
}

void serialWriteAll(int fd, const uint8_t* data, size_t size) {
  for (size_t pos = 0; pos < size;) {
    const ssize_t n = write(fd, data + pos, size - pos);
    if (n > 0) {
      pos += static_cast<size_t>(n);
      continue;
    }
    if (n == -1 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      pollfd q{fd, POLLOUT, 0};
      poll(&q, 1, 10);
      continue;
    }
    if (n == -1 && errno == EINTR) continue;
    fail("serial write");
  }
}

void sendMsg(int fd, const mavlink_message_t& m) {
  uint8_t bytes[MAVLINK_MAX_PACKET_LEN];
  const uint16_t n = mavlink_msg_to_send_buffer(bytes, &m);
  serialWriteAll(fd, bytes, n);
}

void requestRate(int fd, uint8_t sys, uint8_t comp, uint32_t msgid, int hz) {
  mavlink_message_t m{};
  const float interval_us = hz > 0 ? static_cast<float>(1000000.0 / hz) : 0.0f;
  mavlink_msg_command_long_pack(kCompanionSysId, kCompanionCompId, &m,
      sys, comp, MAV_CMD_SET_MESSAGE_INTERVAL, 0, msgid,
      interval_us, 0, 0, 0, 0, 0);
  sendMsg(fd, m);
}

void sendTimesync(int fd, int64_t t0, uint8_t sys, uint8_t comp) {
  mavlink_message_t m{};
  mavlink_msg_timesync_pack(kCompanionSysId, kCompanionCompId, &m,
                            0, t0, sys, comp);
  sendMsg(fd, m);
}

ClockMapping estimateClockMapping(const std::vector<TimeSyncSample>& samples) {
  std::vector<const TimeSyncSample*> good;
  for (const auto& s : samples) if (s.good) good.push_back(&s);

  ClockMapping m;
  if (good.size() < 20) return m;

  const int64_t fc0 = good.front()->fc_ns;
  const int64_t rpi0 = good.front()->rpi_mid_ns;
  long double mx = 0.0L, my = 0.0L;
  for (const auto* s : good) {
    mx += s->fc_ns - fc0;
    my += s->rpi_mid_ns - rpi0;
  }
  mx /= good.size();
  my /= good.size();

  long double sxx = 0.0L, sxy = 0.0L;
  for (const auto* s : good) {
    const long double dx = (s->fc_ns - fc0) - mx;
    const long double dy = (s->rpi_mid_ns - rpi0) - my;
    sxx += dx * dx;
    sxy += dx * dy;
  }
  if (sxx <= 0.0L) return m;

  m.a = sxy / sxx;
  m.fc_ref_ns = fc0;
  m.rpi_ref_ns = static_cast<long double>(rpi0) + (my - m.a * mx);
  m.drift_ppm = static_cast<double>((m.a - 1.0L) * 1e6L);
  m.valid = true;
  m.last_update_ns = good.back()->t1_rpi_ns;
  return m;
}

class LinearPipeline final : public VIO::MonoImuPipeline {
 public:
  explicit LinearPipeline(const VIO::VioParams& params)
      : VIO::MonoImuPipeline(params) {}

  void installBackendCallback() {
    registerBackendOutputCallback(
        [this](const std::shared_ptr<VIO::BackendOutput>& out) {
          if (!out) return;
          const auto& state = out->W_State_Blkf_;
          const auto p = state.pose_.translation();
          const auto rpy = state.pose_.rotation().rpy();
          const auto& v = state.velocity_;
          VioState s;
          s.timestamp_ns = state.timestamp_;
          s.keyframe = out->cur_kf_id_;
          s.px = p.x(); s.py = p.y(); s.pz = p.z();
          s.vx = v.x(); s.vy = v.y(); s.vz = v.z();
          s.roll_deg = rpy.x() * 180.0 / kPi;
          s.pitch_deg = rpy.y() * 180.0 / kPi;
          s.yaw_deg = rpy.z() * 180.0 / kPi;
          {
            std::lock_guard<std::mutex> lock(mutex_);
            states_.push_back(s);
          }
          if ((s.keyframe % 10) == 0) {
            std::cout << std::fixed << std::setprecision(4)
                      << "[VIO] kf=" << s.keyframe
                      << " P=[" << s.px << "," << s.py << "," << s.pz << "]"
                      << " V=[" << s.vx << "," << s.vy << "," << s.vz << "]"
                      << " RPYdeg=[" << s.roll_deg << "," << s.pitch_deg
                      << "," << s.yaw_deg << "]\n";
          }
        });
  }

  std::vector<VioState> states() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return states_;
  }

 private:
  mutable std::mutex mutex_;
  std::vector<VioState> states_;
};

MeanState meanWindow(const std::vector<VioState>& states,
                     int64_t begin_ns, int64_t end_ns) {
  MeanState m;
  double roll_ref = 0.0, pitch_ref = 0.0, yaw_ref = 0.0;
  bool have_ref = false;
  for (const auto& s : states) {
    if (s.timestamp_ns < begin_ns || s.timestamp_ns >= end_ns) continue;
    if (!have_ref) {
      roll_ref = s.roll_deg;
      pitch_ref = s.pitch_deg;
      yaw_ref = s.yaw_deg;
      have_ref = true;
    }
    m.px += s.px; m.py += s.py; m.pz += s.pz;
    m.vx += s.vx; m.vy += s.vy; m.vz += s.vz;
    m.roll_deg += roll_ref + wrapDeg(s.roll_deg - roll_ref);
    m.pitch_deg += pitch_ref + wrapDeg(s.pitch_deg - pitch_ref);
    m.yaw_deg += yaw_ref + wrapDeg(s.yaw_deg - yaw_ref);
    ++m.count;
  }
  if (m.count == 0) return m;
  const double n = static_cast<double>(m.count);
  m.px /= n; m.py /= n; m.pz /= n;
  m.vx /= n; m.vy /= n; m.vz /= n;
  m.roll_deg /= n; m.pitch_deg /= n; m.yaw_deg /= n;
  m.valid = true;
  return m;
}

const char* phaseName(int64_t ts, int64_t start_ns,
                      int64_t move_ns, int64_t end_ns, int64_t stop_ns) {
  if (ts < start_ns) return "PRE";
  if (ts < move_ns) return "START";
  if (ts < end_ns) return "MOVE";
  if (ts < stop_ns) return "END";
  return "POST";
}

void writeCsv(const std::vector<VioState>& states,
              int64_t start_ns, int64_t move_ns,
              int64_t end_ns, int64_t stop_ns) {
  std::ofstream f(kCsvPath, std::ios::trunc);
  if (!f) throw std::runtime_error("Cannot create 500 mm CSV");
  f << "phase,keyframe,timestamp_ns,px_m,py_m,pz_m,vx_m_s,vy_m_s,vz_m_s,roll_deg,pitch_deg,yaw_deg\n";
  f << std::fixed << std::setprecision(9);
  for (const auto& s : states) {
    f << phaseName(s.timestamp_ns, start_ns, move_ns, end_ns, stop_ns) << ','
      << s.keyframe << ',' << s.timestamp_ns << ','
      << s.px << ',' << s.py << ',' << s.pz << ','
      << s.vx << ',' << s.vy << ',' << s.vz << ','
      << s.roll_deg << ',' << s.pitch_deg << ',' << s.yaw_deg << '\n';
  }
}

void printMeasurement(const MeanState& a, const MeanState& b) {
  if (!a.valid || !b.valid) {
    std::cout << "MEASUREMENT RESULT:     FAIL (insufficient START/END states)\n";
    return;
  }

  const double dx = b.px - a.px;
  const double dy = b.py - a.py;
  const double dz = b.pz - a.pz;
  const double horizontal = std::sqrt(dx * dx + dy * dy);
  const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
  const double error_m = distance - kExpectedDistanceM;
  const double scale = distance / kExpectedDistanceM;
  const double percent = error_m / kExpectedDistanceM * 100.0;
  const double end_v = std::sqrt(b.vx*b.vx + b.vy*b.vy + b.vz*b.vz);

  std::cout << std::fixed << std::setprecision(6)
            << "START avg states:       " << a.count << "\n"
            << "END avg states:         " << b.count << "\n"
            << "START mean P:           [" << a.px << "," << a.py << "," << a.pz << "] m\n"
            << "END mean P:             [" << b.px << "," << b.py << "," << b.pz << "] m\n"
            << "measured dP:            [" << dx << "," << dy << "," << dz << "] m\n"
            << "horizontal distance:    " << horizontal * 1000.0 << " mm\n"
            << "3D measured distance:   " << distance * 1000.0 << " mm\n"
            << "expected distance:      " << kExpectedDistanceM * 1000.0 << " mm\n"
            << "absolute error:         " << error_m * 1000.0 << " mm\n"
            << "relative error:         " << percent << " %\n"
            << "scale measured/true:    " << scale << "\n"
            << "dRoll:                  " << wrapDeg(b.roll_deg-a.roll_deg) << " deg\n"
            << "dPitch:                 " << wrapDeg(b.pitch_deg-a.pitch_deg) << " deg\n"
            << "dYaw:                   " << wrapDeg(b.yaw_deg-a.yaw_deg) << " deg\n"
            << "END mean |V|:           " << end_v * 1000.0 << " mm/s\n"
            << "MEASUREMENT RESULT:     REPORT ONLY (scale thresholds not frozen yet)\n";
}

}  // namespace

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

  FLAGS_visualize = false;
  FLAGS_viz_type = 2;
  FLAGS_use_lcd = false;
  FLAGS_log_output = false;
  FLAGS_extract_planes_from_the_scene = false;

  const std::string params = argc > 1 ? argv[1] : "params/JTZeroMono";
  const double total_sec = kStartPhaseSec + kMovePhaseSec + kEndPhaseSec;

  int camera_fd = -1;
  int serial_fd = -1;
  bool streaming = false;
  bool imu_rate_requested = false;
  uint8_t target_system = 0;
  uint8_t target_component = 0;
  std::vector<CameraBuffer> buffers;
  std::shared_ptr<LinearPipeline> pipeline;

  try {
    VIO::VioParams vio_params(params);
    if (vio_params.camera_params_.empty())
      throw std::runtime_error("No camera params loaded");

    pipeline = std::make_shared<LinearPipeline>(vio_params);
    pipeline->installBackendCallback();
    auto pipeline_thread = std::async(std::launch::async,
        [pipeline]() { return pipeline->spin(); });

    serial_fd = openSerial();
    std::cout << "[MAV] waiting for HEARTBEAT...\n";
    mavlink_status_t mav_status{};
    mavlink_message_t mav_msg{};
    const int64_t hb_deadline = monotonicNs() + 10000000000LL;
    while (monotonicNs() < hb_deadline && target_system == 0) {
      pollfd p{serial_fd, POLLIN, 0};
      if (poll(&p, 1, 100) <= 0) continue;
      uint8_t bytes[2048];
      const ssize_t n = read(serial_fd, bytes, sizeof(bytes));
      if (n <= 0) continue;
      for (ssize_t i = 0; i < n; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_0, bytes[i], &mav_msg, &mav_status) &&
            mav_msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
          target_system = mav_msg.sysid;
          target_component = mav_msg.compid;
          break;
        }
      }
    }
    if (!target_system) throw std::runtime_error("HEARTBEAT timeout");

    requestRate(serial_fd, target_system, target_component,
                MAVLINK_MSG_ID_HIGHRES_IMU, kImuRateHz);
    imu_rate_requested = true;

    camera_fd = open(kCameraDevice, O_RDWR | O_NONBLOCK);
    if (camera_fd == -1) fail("open camera");
    configureCamera(camera_fd);
    buffers = initCameraBuffers(camera_fd);
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(camera_fd, VIDIOC_STREAMON, &type) == -1) fail("STREAMON");
    streaming = true;
    discardWarmup(camera_fd);

    std::cout << "\n============================================================\n"
              << "JT-ZERO GUIDED 500 MM TEST\n"
              << "============================================================\n"
              << "START: keep rig motionless for " << kStartPhaseSec << " s.\n"
              << "MOVE:  then move it exactly 500 mm in a straight horizontal line\n"
              << "       during the next " << kMovePhaseSec << " s and stop at the mark.\n"
              << "END:   keep it motionless for the final " << kEndPhaseSec << " s.\n"
              << "Do not return to the start in this run.\n"
              << "Visualizer disabled. CSV: " << kCsvPath << "\n"
              << "============================================================\n";

    std::vector<TimeSyncSample> timesync;
    timesync.reserve(500);
    ClockMapping mapping;
    int64_t pending_timesync = 0;
    int64_t next_timesync = monotonicNs();

    size_t raw_frames = 0;
    size_t rejected_raw_pairs = 0;
    size_t selected_frames = 0;
    size_t decoded_frames = 0;
    size_t imu_rx = 0;
    size_t imu_fed = 0;
    size_t imu_skipped_mapping = 0;
    uint32_t previous_seq = 0;
    int64_t previous_corrected_ns = 0;
    bool have_previous_raw = false;
    int64_t last_selected_ns = 0;
    VIO::FrameId frame_id = 0;

    const int64_t start_ns = monotonicNs();
    const int64_t move_ns = start_ns + static_cast<int64_t>(kStartPhaseSec * 1e9);
    const int64_t end_ns = move_ns + static_cast<int64_t>(kMovePhaseSec * 1e9);
    const int64_t stop_ns = end_ns + static_cast<int64_t>(kEndPhaseSec * 1e9);
    bool move_announced = false;
    bool end_announced = false;
    int last_countdown = -1;

    while (monotonicNs() < stop_ns) {
      const int64_t now = monotonicNs();

      if (!move_announced && now >= move_ns) {
        move_announced = true;
        std::cout << "\n>>>>>>>> MOVE NOW: exactly 500 mm, straight line. <<<<<<<<\n";
      }
      if (!end_announced && now >= end_ns) {
        end_announced = true;
        std::cout << "\n>>>>>>>> STOP. END PHASE: DO NOT MOVE. <<<<<<<<\n";
      }

      int countdown = 0;
      if (now < move_ns) countdown = static_cast<int>(std::ceil((move_ns-now)/1e9));
      else if (now < end_ns) countdown = static_cast<int>(std::ceil((end_ns-now)/1e9));
      else countdown = static_cast<int>(std::ceil((stop_ns-now)/1e9));
      if (countdown != last_countdown && countdown <= 5) {
        last_countdown = countdown;
        if (now < move_ns) std::cout << "[START] move begins in " << countdown << " s\n";
        else if (now < end_ns) std::cout << "[MOVE] stop at mark within " << countdown << " s\n";
        else std::cout << "[END] finish in " << countdown << " s\n";
      }

      if (now >= next_timesync && pending_timesync == 0) {
        pending_timesync = now;
        sendTimesync(serial_fd, pending_timesync, target_system, target_component);
        next_timesync = now + kTimesyncPeriodNs;
      }

      pollfd pf[2] = {{camera_fd, POLLIN, 0}, {serial_fd, POLLIN, 0}};
      const int rc = poll(pf, 2, 2);
      if (rc < 0) {
        if (errno == EINTR) continue;
        fail("poll");
      }

      if (pf[1].revents & POLLIN) {
        uint8_t bytes[8192];
        for (;;) {
          const ssize_t n = read(serial_fd, bytes, sizeof(bytes));
          if (n == -1 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
          if (n <= 0) break;
          for (ssize_t i = 0; i < n; ++i) {
            if (!mavlink_parse_char(MAVLINK_COMM_0, bytes[i], &mav_msg, &mav_status))
              continue;
            const int64_t recv_ns = monotonicNs();

            if (mav_msg.msgid == MAVLINK_MSG_ID_TIMESYNC) {
              mavlink_timesync_t ts{};
              mavlink_msg_timesync_decode(&mav_msg, &ts);
              if (ts.tc1 != 0 && pending_timesync != 0 && ts.ts1 == pending_timesync) {
                TimeSyncSample s;
                s.t0_rpi_ns = pending_timesync;
                s.t1_rpi_ns = recv_ns;
                s.fc_ns = ts.tc1;
                s.rtt_ns = recv_ns - pending_timesync;
                s.rpi_mid_ns = pending_timesync + s.rtt_ns / 2;
                s.good = s.rtt_ns > 0 && nsToMs(s.rtt_ns) <= kMaxTimesyncRttMs;
                timesync.push_back(s);
                pending_timesync = 0;
                mapping = estimateClockMapping(timesync);
              }
            } else if (mav_msg.msgid == MAVLINK_MSG_ID_HIGHRES_IMU) {
              ++imu_rx;
              mavlink_highres_imu_t imu{};
              mavlink_msg_highres_imu_decode(&mav_msg, &imu);
              if (!mapping.valid || recv_ns - mapping.last_update_ns > kMappingStaleNs) {
                ++imu_skipped_mapping;
                continue;
              }
              const int64_t fc_ns = static_cast<int64_t>(imu.time_usec) * 1000LL;
              const int64_t mapped_ns = mapping.map(fc_ns);
              VIO::ImuAccGyr data;
              data << imu.xacc, imu.yacc, imu.zacc,
                      imu.xgyro, imu.ygyro, imu.zgyro;
              pipeline->fillSingleImuQueue(VIO::ImuMeasurement(mapped_ns, data));
              ++imu_fed;
            }
          }
        }
      }

      if (pending_timesync != 0 && monotonicNs() - pending_timesync > 20000000LL)
        pending_timesync = 0;

      if (pf[0].revents & POLLIN) {
        for (;;) {
          v4l2_buffer b{};
          b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
          b.memory = V4L2_MEMORY_MMAP;
          if (xioctl(camera_fd, VIDIOC_DQBUF, &b) == -1) {
            if (errno == EAGAIN) break;
            fail("DQBUF");
          }

          ++raw_frames;
          const int64_t source_ns = timevalToNs(b.timestamp);
          const int64_t corrected_ns = jtzero::timesync::correctCameraTimestampNs(source_ns);
          bool raw_ok = true;
          if (have_previous_raw) {
            const int64_t dt = corrected_ns - previous_corrected_ns;
            raw_ok = b.sequence == previous_seq + 1U && dt > 0 && dt <= 20000000LL;
            if (!raw_ok) ++rejected_raw_pairs;
          }
          previous_seq = b.sequence;
          previous_corrected_ns = corrected_ns;
          have_previous_raw = true;

          const bool selector_due = last_selected_ns == 0 ||
                                    corrected_ns - last_selected_ns >= 30000000LL;
          if (raw_ok && selector_due && mapping.valid) {
            std::vector<unsigned char> jpeg(b.bytesused);
            std::memcpy(jpeg.data(), buffers[b.index].start, b.bytesused);
            cv::Mat gray = cv::imdecode(jpeg, cv::IMREAD_GRAYSCALE);
            if (!gray.empty()) {
              ++decoded_frames;
              pipeline->fillLeftFrameQueue(std::make_unique<VIO::Frame>(
                  frame_id++, corrected_ns, vio_params.camera_params_.at(0), gray.clone()));
              last_selected_ns = corrected_ns;
              ++selected_frames;
            }
          }

          if (xioctl(camera_fd, VIDIOC_QBUF, &b) == -1) fail("QBUF");
        }
      }
    }

    std::cout << "[500MM] stopping and draining...\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    pipeline->shutdown();
    pipeline_thread.get();

    const auto states = pipeline->states();
    writeCsv(states, start_ns, move_ns, end_ns, stop_ns);

    const int64_t start_avg_begin = move_ns - static_cast<int64_t>(kAverageWindowSec * 1e9);
    const int64_t start_avg_end = move_ns;
    const int64_t end_avg_begin = stop_ns - static_cast<int64_t>(kAverageWindowSec * 1e9);
    const int64_t end_avg_end = stop_ns;
    const MeanState start_mean = meanWindow(states, start_avg_begin, start_avg_end);
    const MeanState end_mean = meanWindow(states, end_avg_begin, end_avg_end);

    if (imu_rate_requested) {
      requestRate(serial_fd, target_system, target_component,
                  MAVLINK_MSG_ID_HIGHRES_IMU, 0);
      imu_rate_requested = false;
    }
    if (streaming) {
      v4l2_buf_type t = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      xioctl(camera_fd, VIDIOC_STREAMOFF, &t);
      streaming = false;
    }
    for (auto& b : buffers)
      if (b.start && b.start != MAP_FAILED) munmap(b.start, b.length);
    if (camera_fd != -1) close(camera_fd);
    if (serial_fd != -1) close(serial_fd);

    std::cout << "\n============================================================\n"
              << "JT-ZERO LIVE MONO+IMU 500 MM RESULT\n"
              << "============================================================\n"
              << "raw camera frames:     " << raw_frames << "\n"
              << "rejected raw pairs:    " << rejected_raw_pairs << "\n"
              << "selected frames:       " << selected_frames << "\n"
              << "decoded frames:        " << decoded_frames << "\n"
              << "IMU received:          " << imu_rx << "\n"
              << "IMU fed to Kimera:     " << imu_fed << "\n"
              << "IMU skipped mapping:   " << imu_skipped_mapping << "\n"
              << "TIMESYNC samples:      " << timesync.size() << "\n"
              << "mapping valid:         " << (mapping.valid ? "yes" : "no") << "\n"
              << "mapping drift ppm:     " << std::fixed << std::setprecision(3)
              << mapping.drift_ppm << "\n"
              << "backend states:        " << states.size() << "\n";
    printMeasurement(start_mean, end_mean);
    std::cout << "CSV:                    " << kCsvPath << "\n";

    const bool pipeline_pass = mapping.valid && selected_frames >= total_sec * 20.0 &&
                               imu_fed >= total_sec * 150.0 && states.size() >= 80 &&
                               start_mean.valid && end_mean.valid;
    std::cout << "PIPELINE RESULT:        " << (pipeline_pass ? "PASS" : "FAIL") << "\n";
    return pipeline_pass ? 0 : 1;

  } catch (const std::exception& e) {
    std::cerr << "[FATAL] " << e.what() << "\n";
    if (pipeline) pipeline->shutdown();
    if (serial_fd != -1 && imu_rate_requested && target_system) {
      try {
        requestRate(serial_fd, target_system, target_component,
                    MAVLINK_MSG_ID_HIGHRES_IMU, 0);
      } catch (...) {}
    }
    if (streaming && camera_fd != -1) {
      v4l2_buf_type t = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      xioctl(camera_fd, VIDIOC_STREAMOFF, &t);
    }
    for (auto& b : buffers)
      if (b.start && b.start != MAP_FAILED) munmap(b.start, b.length);
    if (camera_fd != -1) close(camera_fd);
    if (serial_fd != -1) close(serial_fd);
    return 1;
  }
}
