// JT-ZERO yaw IMU-only diagnostic v3.
// Purpose: isolate rotation->translation coupling in IMU/preintegration/backend.
// Protocol: 10 s INIT still -> rotate yaw until accumulated FC yaw reaches 88 deg
// -> automatic 10 s HOLD.
//
// IMPORTANT:
//   Run with JTZERO_DIAG_IMU_ONLY=1 after applying
//   tools/patch_kimera_mono_imu_only_diag.sh and rebuilding Kimera.
//
// ArduPilot HIGHRES_IMU FRD is converted to Kimera FLU before feed:
//   accel: [x, -y, -z]
//   gyro : [x, -y, -z]

#define main jtzero_standstill_unused_main
#include "live_mono_imu_standstill.cpp"
#undef main

#include <atomic>
#include <cmath>
#include <iomanip>
#include <mutex>

namespace {

constexpr int kAttitudeRateHz = 50;
constexpr double kInitSec = 10.0;
constexpr double kHoldSec = 10.0;
constexpr double kYawTriggerDeg = 88.0;
constexpr double kYawTimeoutSec = 35.0;
constexpr const char* kCsv = "/home/vio/jtzero_live_yaw_imu_only_v3.csv";

static double wrap180(double d) {
  while (d > 180.0) d -= 360.0;
  while (d < -180.0) d += 360.0;
  return d;
}

struct SharedTelemetry {
  std::mutex m;
  bool fc_valid = false;
  double fc_roll_deg = 0.0;
  double fc_pitch_deg = 0.0;
  double fc_yaw_deg = 0.0;
  double fc_accum_yaw_deg = 0.0;
  double fc_prev_yaw_deg = 0.0;
  bool fc_have_prev = false;

  bool raw_valid = false;
  double ax = 0.0, ay = 0.0, az = 0.0;
  double gx = 0.0, gy = 0.0, gz = 0.0;
};

struct State {
  int64_t wall_ns = 0;
  int64_t ts = 0;
  int64_t kf = 0;
  double px = 0, py = 0, pz = 0;
  double vx = 0, vy = 0, vz = 0;
  double roll = 0, pitch = 0, yaw = 0;
  double bax = 0, bay = 0, baz = 0;
  double bgx = 0, bgy = 0, bgz = 0;
  double fc_roll = 0, fc_pitch = 0, fc_yaw = 0, fc_accum_yaw = 0;
  double ax = 0, ay = 0, az = 0;
  double gx = 0, gy = 0, gz = 0;
};

class DiagPipeline final : public VIO::MonoImuPipeline {
 public:
  DiagPipeline(const VIO::VioParams& p, SharedTelemetry* telemetry)
      : VIO::MonoImuPipeline(p), telemetry_(telemetry) {}

  void install() {
    registerBackendOutputCallback(
        [this](const std::shared_ptr<VIO::BackendOutput>& out) {
          if (!out) return;
          const auto& s = out->W_State_Blkf_;
          const auto p = s.pose_.translation();
          const auto rpy = s.pose_.rotation().rpy();
          const auto& v = s.velocity_;
          const auto ba = s.imu_bias_.accelerometer();
          const auto bg = s.imu_bias_.gyroscope();

          State z;
          z.wall_ns = monotonicNs();
          z.ts = s.timestamp_;
          z.kf = out->cur_kf_id_;
          z.px = p.x(); z.py = p.y(); z.pz = p.z();
          z.vx = v.x(); z.vy = v.y(); z.vz = v.z();
          z.roll = rpy.x() * 180.0 / M_PI;
          z.pitch = rpy.y() * 180.0 / M_PI;
          z.yaw = rpy.z() * 180.0 / M_PI;
          z.bax = ba.x(); z.bay = ba.y(); z.baz = ba.z();
          z.bgx = bg.x(); z.bgy = bg.y(); z.bgz = bg.z();

          if (telemetry_) {
            std::lock_guard<std::mutex> l(telemetry_->m);
            z.fc_roll = telemetry_->fc_roll_deg;
            z.fc_pitch = telemetry_->fc_pitch_deg;
            z.fc_yaw = telemetry_->fc_yaw_deg;
            z.fc_accum_yaw = telemetry_->fc_accum_yaw_deg;
            z.ax = telemetry_->ax; z.ay = telemetry_->ay; z.az = telemetry_->az;
            z.gx = telemetry_->gx; z.gy = telemetry_->gy; z.gz = telemetry_->gz;
          }

          {
            std::lock_guard<std::mutex> l(m_);
            states_.push_back(z);
          }

          if ((z.kf % 5) == 0) {
            std::cout << std::fixed << std::setprecision(5)
                      << "[V3] kf=" << z.kf
                      << " FCaccYaw=" << z.fc_accum_yaw
                      << " P=[" << z.px << ',' << z.py << ',' << z.pz << ']'
                      << " V=[" << z.vx << ',' << z.vy << ',' << z.vz << ']'
                      << " BA=[" << z.bax << ',' << z.bay << ',' << z.baz << ']'
                      << " BG=[" << z.bgx << ',' << z.bgy << ',' << z.bgz << ']'
                      << " Aflu=[" << z.ax << ',' << z.ay << ',' << z.az << ']'
                      << " Gflu=[" << z.gx << ',' << z.gy << ',' << z.gz << "]\n";
          }
        });
  }

  std::vector<State> states() const {
    std::lock_guard<std::mutex> l(m_);
    return states_;
  }

 private:
  SharedTelemetry* telemetry_ = nullptr;
  mutable std::mutex m_;
  std::vector<State> states_;
};

static void saveCsv(const std::vector<State>& states,
                    int64_t init_end,
                    int64_t yaw_end) {
  std::ofstream f(kCsv, std::ios::trunc);
  f << "phase,wall_ns,keyframe,timestamp_ns,"
       "px,py,pz,vx,vy,vz,roll_deg,pitch_deg,yaw_deg,"
       "bax,bay,baz,bgx,bgy,bgz,"
       "fc_roll_deg,fc_pitch_deg,fc_yaw_deg,fc_accum_yaw_deg,"
       "ax_flu,ay_flu,az_flu,gx_flu,gy_flu,gz_flu\n";
  f << std::fixed << std::setprecision(9);
  for (const auto& z : states) {
    const char* phase = z.wall_ns < init_end ? "INIT" :
                        (z.wall_ns < yaw_end ? "YAW" : "HOLD");
    f << phase << ',' << z.wall_ns << ',' << z.kf << ',' << z.ts << ','
      << z.px << ',' << z.py << ',' << z.pz << ','
      << z.vx << ',' << z.vy << ',' << z.vz << ','
      << z.roll << ',' << z.pitch << ',' << z.yaw << ','
      << z.bax << ',' << z.bay << ',' << z.baz << ','
      << z.bgx << ',' << z.bgy << ',' << z.bgz << ','
      << z.fc_roll << ',' << z.fc_pitch << ',' << z.fc_yaw << ','
      << z.fc_accum_yaw << ','
      << z.ax << ',' << z.ay << ',' << z.az << ','
      << z.gx << ',' << z.gy << ',' << z.gz << '\n';
  }
}

}  // namespace

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_visualize = false;
  FLAGS_viz_type = 2;
  FLAGS_use_lcd = false;
  FLAGS_log_output = false;
  FLAGS_extract_planes_from_the_scene = false;

  const std::string params = argc > 1 ? argv[1] : "params/JTZeroMonoFLUZeroLever";

  int cfd = -1, sfd = -1;
  bool streaming = false, imu_req = false, att_req = false;
  uint8_t sys = 0, comp = 0;
  std::vector<CameraBuffer> bufs;
  SharedTelemetry telemetry;
  std::shared_ptr<DiagPipeline> pipe;
  std::future<bool> worker;

  try {
    if (!std::getenv("JTZERO_DIAG_IMU_ONLY")) {
      throw std::runtime_error(
          "JTZERO_DIAG_IMU_ONLY is not set; refusing to run ambiguous diagnostic");
    }

    VIO::VioParams vp(params);
    if (vp.camera_params_.empty()) throw std::runtime_error("No camera params loaded");
    pipe = std::make_shared<DiagPipeline>(vp, &telemetry);
    pipe->install();
    worker = std::async(std::launch::async, [pipe]() { return pipe->spin(); });

    sfd = openSerial();
    mavlink_status_t mst{};
    mavlink_message_t msg{};
    std::cout << "[MAV] waiting for HEARTBEAT...\n";
    const int64_t hb_deadline = monotonicNs() + 10000000000LL;
    while (monotonicNs() < hb_deadline && !sys) {
      pollfd p{sfd, POLLIN, 0};
      if (poll(&p, 1, 100) <= 0) continue;
      uint8_t b[2048];
      const ssize_t n = read(sfd, b, sizeof(b));
      if (n <= 0) continue;
      for (ssize_t i = 0; i < n; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_0, b[i], &msg, &mst) &&
            msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
          sys = msg.sysid;
          comp = msg.compid;
          break;
        }
      }
    }
    if (!sys) throw std::runtime_error("HEARTBEAT timeout");

    requestRate(sfd, sys, comp, MAVLINK_MSG_ID_HIGHRES_IMU, kImuRateHz);
    imu_req = true;
    requestRate(sfd, sys, comp, MAVLINK_MSG_ID_ATTITUDE, kAttitudeRateHz);
    att_req = true;

    cfd = open(kCameraDevice, O_RDWR | O_NONBLOCK);
    if (cfd == -1) fail("open camera");
    configureCamera(cfd);
    bufs = initCameraBuffers(cfd);
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(cfd, VIDIOC_STREAMON, &type) == -1) fail("STREAMON");
    streaming = true;
    discardWarmup(cfd);

    std::vector<TimeSyncSample> sync;
    ClockMapping mapping;
    int64_t pending = 0, next_sync = monotonicNs();
    size_t raw = 0, rejected = 0, selected = 0;
    size_t imu_rx = 0, imu_fed = 0, imu_skip = 0, att_rx = 0;
    uint32_t prev_seq = 0;
    int64_t prev_ts = 0, last_sel = 0;
    bool have_prev = false;
    VIO::FrameId fid = 0;

    const int64_t start = monotonicNs();
    const int64_t init_end = start + static_cast<int64_t>(kInitSec * 1e9);
    int64_t yaw_end = 0;
    int64_t hold_end = 0;
    bool reference_set = false;
    double fc_accum_ref = 0.0;
    double roll_ref = 0.0, pitch_ref = 0.0;
    double max_droll = 0.0, max_dpitch = 0.0;

    std::cout << "\nJT-ZERO YAW IMU-ONLY v3\n"
              << "10 s INIT still -> yaw ~90 deg -> 10 s HOLD\n"
              << "FC yaw uses incremental accumulation across +/-180 deg.\n\n";

    while (true) {
      const int64_t now = monotonicNs();

      if (!reference_set && now >= init_end) {
        std::lock_guard<std::mutex> l(telemetry.m);
        if (telemetry.fc_valid) {
          reference_set = true;
          fc_accum_ref = telemetry.fc_accum_yaw_deg;
          roll_ref = telemetry.fc_roll_deg;
          pitch_ref = telemetry.fc_pitch_deg;
          std::cout << "[YAW] reference captured\n";
        }
      }

      if (reference_set && yaw_end == 0) {
        double dyaw = 0.0, droll = 0.0, dpitch = 0.0;
        {
          std::lock_guard<std::mutex> l(telemetry.m);
          dyaw = telemetry.fc_accum_yaw_deg - fc_accum_ref;
          droll = wrap180(telemetry.fc_roll_deg - roll_ref);
          dpitch = wrap180(telemetry.fc_pitch_deg - pitch_ref);
        }
        max_droll = std::max(max_droll, std::abs(droll));
        max_dpitch = std::max(max_dpitch, std::abs(dpitch));
        if (std::abs(dyaw) >= kYawTriggerDeg) {
          yaw_end = now;
          hold_end = now + static_cast<int64_t>(kHoldSec * 1e9);
          std::cout << "[YAW] target reached at accumulated FC yaw="
                    << std::fixed << std::setprecision(3) << dyaw
                    << " deg -> HOLD\n";
        }
        if (now - init_end > static_cast<int64_t>(kYawTimeoutSec * 1e9)) {
          throw std::runtime_error("Yaw target timeout");
        }
      }

      if (yaw_end != 0 && now >= hold_end) break;

      if (now >= next_sync && pending == 0) {
        pending = now;
        sendTimesync(sfd, pending, sys, comp);
        next_sync = now + kTimesyncPeriodNs;
      }

      pollfd pf[2] = {{cfd, POLLIN, 0}, {sfd, POLLIN, 0}};
      const int rc = poll(pf, 2, 2);
      if (rc < 0) {
        if (errno == EINTR) continue;
        fail("poll");
      }

      if (pf[1].revents & POLLIN) {
        uint8_t b[8192];
        for (;;) {
          const ssize_t n = read(sfd, b, sizeof(b));
          if (n == -1 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
          if (n <= 0) break;
          for (ssize_t i = 0; i < n; ++i) {
            if (!mavlink_parse_char(MAVLINK_COMM_0, b[i], &msg, &mst)) continue;
            const int64_t rx = monotonicNs();

            if (msg.msgid == MAVLINK_MSG_ID_TIMESYNC) {
              mavlink_timesync_t ts{};
              mavlink_msg_timesync_decode(&msg, &ts);
              if (ts.tc1 != 0 && pending != 0 && ts.ts1 == pending) {
                TimeSyncSample s;
                s.t0_rpi_ns = pending;
                s.t1_rpi_ns = rx;
                s.fc_ns = ts.tc1;
                s.rtt_ns = rx - pending;
                s.rpi_mid_ns = pending + s.rtt_ns / 2;
                s.good = s.rtt_ns > 0 && nsToMs(s.rtt_ns) <= kMaxTimesyncRttMs;
                sync.push_back(s);
                pending = 0;
                mapping = estimateClockMapping(sync);
              }
            } else if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE) {
              ++att_rx;
              mavlink_attitude_t a{};
              mavlink_msg_attitude_decode(&msg, &a);
              const double roll = a.roll * 180.0 / M_PI;
              const double pitch = a.pitch * 180.0 / M_PI;
              const double yaw = a.yaw * 180.0 / M_PI;
              std::lock_guard<std::mutex> l(telemetry.m);
              if (telemetry.fc_have_prev) {
                telemetry.fc_accum_yaw_deg += wrap180(yaw - telemetry.fc_prev_yaw_deg);
              }
              telemetry.fc_prev_yaw_deg = yaw;
              telemetry.fc_have_prev = true;
              telemetry.fc_roll_deg = roll;
              telemetry.fc_pitch_deg = pitch;
              telemetry.fc_yaw_deg = yaw;
              telemetry.fc_valid = true;
            } else if (msg.msgid == MAVLINK_MSG_ID_HIGHRES_IMU) {
              ++imu_rx;
              mavlink_highres_imu_t h{};
              mavlink_msg_highres_imu_decode(&msg, &h);

              const double ax = h.xacc;
              const double ay = -h.yacc;
              const double az = -h.zacc;
              const double gx = h.xgyro;
              const double gy = -h.ygyro;
              const double gz = -h.zgyro;
              {
                std::lock_guard<std::mutex> l(telemetry.m);
                telemetry.raw_valid = true;
                telemetry.ax = ax; telemetry.ay = ay; telemetry.az = az;
                telemetry.gx = gx; telemetry.gy = gy; telemetry.gz = gz;
              }

              if (!mapping.valid || rx - mapping.last_update_ns > kMappingStaleNs) {
                ++imu_skip;
                continue;
              }
              VIO::ImuAccGyr d;
              d << ax, ay, az, gx, gy, gz;
              pipe->fillSingleImuQueue(VIO::ImuMeasurement(
                  mapping.map(static_cast<int64_t>(h.time_usec) * 1000LL), d));
              ++imu_fed;
            }
          }
        }
      }

      if (pending && monotonicNs() - pending > 20000000LL) pending = 0;

      if (pf[0].revents & POLLIN) {
        for (;;) {
          v4l2_buffer b{};
          b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
          b.memory = V4L2_MEMORY_MMAP;
          if (xioctl(cfd, VIDIOC_DQBUF, &b) == -1) {
            if (errno == EAGAIN) break;
            fail("DQBUF");
          }
          ++raw;
          const int64_t ts =
              jtzero::timesync::correctCameraTimestampNs(timevalToNs(b.timestamp));
          bool ok = true;
          if (have_prev) {
            const int64_t dt = ts - prev_ts;
            ok = b.sequence == prev_seq + 1U && dt > 0 && dt <= 20000000LL;
            if (!ok) ++rejected;
          }
          prev_seq = b.sequence;
          prev_ts = ts;
          have_prev = true;

          const bool due = last_sel == 0 || ts - last_sel >= 30000000LL;
          if (ok && due && mapping.valid) {
            std::vector<unsigned char> jpeg(b.bytesused);
            std::memcpy(jpeg.data(), bufs[b.index].start, b.bytesused);
            cv::Mat gray = cv::imdecode(jpeg, cv::IMREAD_GRAYSCALE);
            if (!gray.empty()) {
              pipe->fillLeftFrameQueue(std::make_unique<VIO::Frame>(
                  fid++, ts, vp.camera_params_.at(0), gray.clone()));
              last_sel = ts;
              ++selected;
            }
          }
          if (xioctl(cfd, VIDIOC_QBUF, &b) == -1) fail("QBUF");
        }
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    pipe->shutdown();
    worker.get();
    const auto states = pipe->states();
    saveCsv(states, init_end, yaw_end);

    if (imu_req) requestRate(sfd, sys, comp, MAVLINK_MSG_ID_HIGHRES_IMU, 0);
    if (att_req) requestRate(sfd, sys, comp, MAVLINK_MSG_ID_ATTITUDE, 0);
    if (streaming) {
      v4l2_buf_type t = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      xioctl(cfd, VIDIOC_STREAMOFF, &t);
      streaming = false;
    }
    for (auto& b : bufs)
      if (b.start && b.start != MAP_FAILED) munmap(b.start, b.length);
    if (cfd >= 0) close(cfd);
    if (sfd >= 0) close(sfd);

    double final_accum = 0.0;
    {
      std::lock_guard<std::mutex> l(telemetry.m);
      final_accum = telemetry.fc_accum_yaw_deg - fc_accum_ref;
    }

    std::cout << "\n============================================================\n"
              << "JT-ZERO YAW IMU-ONLY v3 RESULT\n"
              << "============================================================\n"
              << "raw camera frames: " << raw << '\n'
              << "rejected raw pairs: " << rejected << '\n'
              << "selected frames: " << selected << '\n'
              << "IMU received: " << imu_rx << '\n'
              << "IMU fed: " << imu_fed << '\n'
              << "IMU skipped mapping: " << imu_skip << '\n'
              << "ATTITUDE received: " << att_rx << '\n'
              << "TIMESYNC samples: " << sync.size() << '\n'
              << "mapping valid: " << (mapping.valid ? "yes" : "no") << '\n'
              << "mapping drift ppm: " << mapping.drift_ppm << '\n'
              << "backend states: " << states.size() << '\n'
              << "accumulated FC dYaw: " << final_accum << " deg\n"
              << "max FC |dRoll|: " << max_droll << " deg\n"
              << "max FC |dPitch|: " << max_dpitch << " deg\n"
              << "CSV: " << kCsv << '\n';

    if (!states.empty()) {
      const auto& a = states.front();
      const auto& b = states.back();
      double max_xy = 0.0, max_vxy = 0.0;
      for (const auto& z : states) {
        max_xy = std::max(max_xy, std::hypot(z.px - a.px, z.py - a.py));
        max_vxy = std::max(max_vxy, std::hypot(z.vx, z.vy));
      }
      std::cout << std::fixed << std::setprecision(6)
                << "FIRST BA=[" << a.bax << ',' << a.bay << ',' << a.baz << "] "
                << "BG=[" << a.bgx << ',' << a.bgy << ',' << a.bgz << "]\n"
                << "LAST  BA=[" << b.bax << ',' << b.bay << ',' << b.baz << "] "
                << "BG=[" << b.bgx << ',' << b.bgy << ',' << b.bgz << "]\n"
                << "final dP=[" << b.px-a.px << ',' << b.py-a.py << ',' << b.pz-a.pz << "] m\n"
                << "max false XY=" << max_xy * 1000.0 << " mm\n"
                << "max Vxy=" << max_vxy * 1000.0 << " mm/s\n";
    }
    return 0;
  } catch (const std::exception& e) {
    if (pipe) pipe->shutdown();
    if (sfd >= 0 && sys) {
      try {
        if (imu_req) requestRate(sfd, sys, comp, MAVLINK_MSG_ID_HIGHRES_IMU, 0);
        if (att_req) requestRate(sfd, sys, comp, MAVLINK_MSG_ID_ATTITUDE, 0);
      } catch (...) {}
    }
    if (streaming && cfd >= 0) {
      v4l2_buf_type t = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      xioctl(cfd, VIDIOC_STREAMOFF, &t);
    }
    for (auto& b : bufs)
      if (b.start && b.start != MAP_FAILED) munmap(b.start, b.length);
    if (cfd >= 0) close(cfd);
    if (sfd >= 0) close(sfd);
    std::cerr << "[FATAL] " << e.what() << '\n';
    return 1;
  }
}
