// JT-ZERO v37: sample-aligned GTSAM PIM/backend prediction vs custom SO(3) integration.
// Both branches use the same HIGHRES_IMU stream, same backend orientation/velocity and BA/BG at SPACE.
// TRUE IMU-only required. Russian GUI. Route: physical 0 -> arbitrary right yaw -> physical 0.

#define JTZERO_V10_NO_MAIN
#include "live_mono_imu_integrator_v10.cpp"
#undef JTZERO_V10_NO_MAIN

#include <gflags/gflags.h>
DECLARE_bool(no_incremental_pose);

namespace jtzero_v37 {
using namespace jtzero_v10;

constexpr const char* kCsv37 = "/home/vio/jtzero_live_pim_vs_custom_v37.csv";
constexpr const char* kWindow37 = "JT-ZERO: PIM ПРОТИВ SO(3) v37";
constexpr int kStable37 = 100;
constexpr double kGyroStill37 = 0.035;
constexpr double kAccTol37 = 0.45;

struct Backend37 {
  bool valid = false;
  int64_t kf = 0;
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d V = Eigen::Vector3d::Zero();
  Eigen::Vector3d BA = Eigen::Vector3d::Zero();
  Eigen::Vector3d BG = Eigen::Vector3d::Zero();
  double pim_vxy = 0.0;
};

class Pipeline37 final : public VIO::MonoImuPipeline {
 public:
  explicit Pipeline37(const VIO::VioParams& p) : VIO::MonoImuPipeline(p) {}

  void install() {
    registerBackendOutputCallback([this](const std::shared_ptr<VIO::BackendOutput>& out) {
      if (!out) return;
      Backend37 b;
      b.valid = true;
      b.kf = out->cur_kf_id_;
      const auto& st = out->W_State_Blkf_;
      b.R = st.pose_.rotation().matrix();
      b.V = st.velocity_;
      b.BA = st.imu_bias_.accelerometer();
      b.BG = st.imu_bias_.gyroscope();
      const auto& pv = out->debug_info_.navstate_k_.velocity();
      b.pim_vxy = std::hypot(pv.x(), pv.y());
      std::lock_guard<std::mutex> lock(m_);
      latest_ = b;
    });
  }

  bool latest(Backend37* out) const {
    std::lock_guard<std::mutex> lock(m_);
    if (!latest_.valid) return false;
    *out = latest_;
    return true;
  }

 private:
  mutable std::mutex m_;
  Backend37 latest_;
};

struct Row37 {
  uint64_t us = 0;
  double dt = 0.0;
  double yaw_rel = 0.0;
  int64_t kf = 0;
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyr = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_custom = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_backend = Eigen::Vector3d::Zero();
  double pim_vxy = 0.0;
  double custom_vxy = 0.0;
  double dv = 0.0;
  double rot_from_start = 0.0;
};

struct State37 {
  bool armed = false;
  uint64_t last_us = 0;
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d R0 = Eigen::Matrix3d::Identity();
  Eigen::Vector3d V = Eigen::Vector3d::Zero();
  Eigen::Vector3d BA = Eigen::Vector3d::Zero();
  Eigen::Vector3d BG = Eigen::Vector3d::Zero();
  std::vector<Row37> rows;
  double max_pim = 0.0;
  double max_custom = 0.0;
  double max_dv = 0.0;
};

static double clamp37(double x) {
  return std::max(-1.0, std::min(1.0, x));
}

static double rot37(const Eigen::Matrix3d& R) {
  return std::acos(clamp37((R.trace() - 1.0) * 0.5)) * 180.0 / kPi;
}

static void process37(State37& s,
                      const ImuSample10& im,
                      const Pipeline37& pipe,
                      double yaw_rel) {
  if (!s.armed) return;

  double dt = 0.0;
  if (s.last_us != 0 && im.us > s.last_us) {
    dt = static_cast<double>(im.us - s.last_us) * 1e-6;
  }
  s.last_us = im.us;
  if (dt <= 0.0 || dt > 0.03) return;

  Eigen::Vector3d acc(im.ax, im.ay, im.az);
  Eigen::Vector3d gyr(im.gx, im.gy, im.gz);
  const Eigen::Vector3d w = gyr - s.BG;

  const Eigen::Vector3d theta = w * dt;
  const double angle = theta.norm();
  if (angle > 1e-12) {
    s.R = s.R * Eigen::AngleAxisd(angle, theta / angle).toRotationMatrix();
  }

  const Eigen::Vector3d a_world =
      s.R * (acc - s.BA) + Eigen::Vector3d(0.0, 0.0, 9.81);
  s.V += a_world * dt;

  Backend37 b;
  if (!pipe.latest(&b)) return;

  Row37 row;
  row.us = im.us;
  row.dt = dt;
  row.yaw_rel = yaw_rel;
  row.kf = b.kf;
  row.acc = acc;
  row.gyr = w;
  row.v_custom = s.V;
  row.v_backend = b.V;
  row.pim_vxy = b.pim_vxy;
  row.custom_vxy = std::hypot(s.V.x(), s.V.y());
  row.dv = (s.V - b.V).norm();
  row.rot_from_start = rot37(s.R0.transpose() * s.R);

  s.rows.push_back(row);
  s.max_pim = std::max(s.max_pim, row.pim_vxy);
  s.max_custom = std::max(s.max_custom, row.custom_vxy);
  s.max_dv = std::max(s.max_dv, row.dv);
}

static void save37(const State37& s) {
  std::ofstream f(kCsv37, std::ios::trunc);
  f << std::fixed << std::setprecision(9);
  f << "imu_us,dt,yaw_rel_deg,kf,ax,ay,az,gx_corr,gy_corr,gz_corr,"
       "custom_vx,custom_vy,custom_vz,backend_vx,backend_vy,backend_vz,"
       "custom_vxy,pim_vxy,dv_norm,custom_rot_from_start_deg\n";

  for (const auto& row : s.rows) {
    f << row.us << ',' << row.dt << ',' << row.yaw_rel << ',' << row.kf << ','
      << row.acc.x() << ',' << row.acc.y() << ',' << row.acc.z() << ','
      << row.gyr.x() << ',' << row.gyr.y() << ',' << row.gyr.z() << ','
      << row.v_custom.x() << ',' << row.v_custom.y() << ',' << row.v_custom.z() << ','
      << row.v_backend.x() << ',' << row.v_backend.y() << ',' << row.v_backend.z() << ','
      << row.custom_vxy << ',' << row.pim_vxy << ',' << row.dv << ','
      << row.rot_from_start << '\n';
  }
}

static void hud37(const cv::Mat& gray,
                  const Telemetry& tel,
                  bool ready,
                  bool turned,
                  bool done,
                  int stable,
                  const State37& s,
                  double ref_yaw) {
  cv::Mat canvas(940, 1440, CV_8UC3, cv::Scalar(15, 15, 15));
  cv::Mat video;
  if (gray.empty()) {
    video = cv::Mat(kHeight, kWidth, CV_8UC3, cv::Scalar(0));
  } else {
    cv::cvtColor(gray, video, cv::COLOR_GRAY2BGR);
  }
  cv::resize(video, video, {820, 615}, 0, 0, cv::INTER_NEAREST);
  video.copyTo(canvas(cv::Rect(20, 80, 820, 615)));

  const cv::Scalar white(235, 235, 235);
  const cv::Scalar green(80, 220, 80);
  const cv::Scalar yellow(0, 220, 255);
  const cv::Scalar red(40, 40, 245);
  const cv::Scalar muted(150, 150, 150);

  uiText(canvas, "JT-ZERO: GTSAM PIM ПРОТИВ SO(3) v37", 20, 48, .70, white, 2);

  double yaw = 0.0;
  double gyro_norm = 0.0;
  double acc_norm = 0.0;
  {
    std::lock_guard<std::mutex> lock(tel.mutex);
    yaw = tel.fc_accum_yaw - ref_yaw;
    gyro_norm = std::sqrt(tel.gx * tel.gx + tel.gy * tel.gy + tel.gz * tel.gz);
    acc_norm = std::sqrt(tel.ax * tel.ax + tel.ay * tel.ay + tel.az * tel.az);
  }

  if (!ready) {
    uiText(canvas, "СТЕНД В ФИЗИЧЕСКИЙ НОЛЬ", 170, 285, .82, white, 2);
    uiText(canvas, "SPACE — ОБЩИЙ СТАРТ PIM И SO(3)", 150, 355, .65, yellow, 2);
  } else if (done) {
    uiText(canvas, "ТЕСТ ЗАВЕРШЁН", 250, 300, 1.05, green, 3);
    uiText(canvas, "SPACE — ВЫХОД", 300, 365, .66, white, 2);
  } else if (!turned) {
    uiText(canvas, "ПЛАВНО ПОВЕРНИТЕ ВПРАВО НА 60–100°", 100, 285, .68, yellow, 2);
    uiText(canvas, "ОСТАНОВИТЕСЬ И ДОЖДИТЕСЬ 100/100", 130, 350, .58, white, 2);
  } else {
    uiText(canvas, "ВЕРНИТЕ СТЕНД В ФИЗИЧЕСКИЙ НОЛЬ", 95, 315, .68, white, 2);
  }

  cv::Mat side = canvas(cv::Rect(860, 70, 560, 850));
  char buf[256];
  std::snprintf(buf, sizeof(buf), "FC yaw от старта: %+.2f°", yaw);
  uiText(side, buf, 18, 80, .50, white, 1);
  std::snprintf(buf, sizeof(buf), "|gyro| %.4f   |acc| %.3f", gyro_norm, acc_norm);
  uiText(side, buf, 18, 125, .44, white, 1);
  std::snprintf(buf, sizeof(buf), "Стабильность %d/%d", stable, kStable37);
  uiText(side, buf, 18, 170, .44, white, 1);

  if (!s.rows.empty()) {
    const auto& row = s.rows.back();
    std::snprintf(buf, sizeof(buf), "CUSTOM Vxy: %.3f м/с", row.custom_vxy);
    uiText(side, buf, 18, 260, .55, yellow, 2);
    std::snprintf(buf, sizeof(buf), "PIM Vxy: %.3f м/с", row.pim_vxy);
    uiText(side, buf, 18, 315, .55, red, 2);
    std::snprintf(buf, sizeof(buf), "|Vcustom-Vbackend|: %.3f", row.dv);
    uiText(side, buf, 18, 370, .48, white, 1);
    std::snprintf(buf, sizeof(buf), "CUSTOM ΔR: %.1f°", row.rot_from_start);
    uiText(side, buf, 18, 425, .48, white, 1);
  }

  uiText(side, "Одинаковые HIGHRES_IMU / BA / BG / R / V", 18, 535, .39, green, 1);
  uiText(side, "FC yaw используется только для индикации", 18, 575, .39, white, 1);
  uiText(canvas, "SPACE: старт/поворот/ноль/выход   ESC/Q: прервать", 25, 910, .40, muted, 1);
  uiText(canvas, std::string("CSV: ") + kCsv37, 760, 910, .38, muted, 1);
  cv::imshow(kWindow37, canvas);
}

}  // namespace jtzero_v37

int main(int argc, char** argv) {
  using namespace jtzero_v10;
  using namespace jtzero_v37;

  google::InitGoogleLogging(argv[0]);
  FLAGS_no_incremental_pose = true;
  FLAGS_visualize = false;
  FLAGS_viz_type = 2;
  FLAGS_use_lcd = false;
  FLAGS_log_output = false;
  FLAGS_extract_planes_from_the_scene = false;

  if (!std::getenv("JTZERO_DIAG_IMU_ONLY")) {
    std::cerr << "[FATAL] JTZERO_DIAG_IMU_ONLY is not set\n";
    return 1;
  }

  const std::string params =
      argc > 1 ? argv[1] : "params/JTZeroMonoFLUZeroLever";

  int cfd = -1;
  int sfd = -1;
  bool streaming = false;
  bool pipeline_started = false;
  bool imu_req = false;
  bool att_req = false;
  bool aborted = false;
  uint8_t sys = 0;
  uint8_t comp = 0;
  std::vector<CameraBuffer> bufs;
  std::shared_ptr<Pipeline37> pipe;
  std::thread pipe_thread;
  Telemetry tel;

  try {
    VIO::VioParams vp(params);
    pipe = std::make_shared<Pipeline37>(vp);
    pipe->install();
    pipe_thread = std::thread([pipe]() { pipe->spin(); });
    pipeline_started = true;

    sfd = openSerial();
    mavlink_status_t mav_status{};
    mavlink_message_t msg{};

    std::cout << "[MAV] waiting for HEARTBEAT...\n";
    const int64_t heartbeat_deadline = monotonicNs() + 10000000000LL;
    while (monotonicNs() < heartbeat_deadline && !sys) {
      pollfd p{sfd, POLLIN, 0};
      if (poll(&p, 1, 100) <= 0) continue;
      uint8_t data[2048];
      const ssize_t n = read(sfd, data, sizeof(data));
      if (n <= 0) continue;
      for (ssize_t i = 0; i < n; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_0, data[i], &msg, &mav_status) &&
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
    requestRate(sfd, sys, comp, MAVLINK_MSG_ID_ATTITUDE, kAttRateHz);
    att_req = true;

    cfd = open(kCameraDevice, O_RDWR | O_NONBLOCK);
    if (cfd == -1) fail("open camera");
    configureCamera(cfd);
    bufs = initCameraBuffers(cfd);
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(cfd, VIDIOC_STREAMON, &type) == -1) fail("STREAMON");
    streaming = true;
    discardWarmup(cfd);

    cv::setNumThreads(1);
    cv::namedWindow(kWindow37, cv::WINDOW_NORMAL);
    cv::resizeWindow(kWindow37, 1440, 940);

    std::vector<TimeSyncSample> sync;
    ClockMapping mapping;
    int64_t pending_sync = 0;
    int64_t next_sync = monotonicNs();
    int64_t next_hud = monotonicNs();
    uint32_t prev_seq = 0;
    int64_t prev_ts = 0;
    int64_t last_selected = 0;
    bool have_prev = false;
    VIO::FrameId frame_id = 0;
    cv::Mat last_gray(kHeight, kWidth, CV_8UC1, cv::Scalar(0));

    State37 state;
    bool ready = false;
    bool turned = false;
    bool done = false;
    int stable = 0;
    double ref_yaw = 0.0;

    std::cout
        << "\nJT-ZERO PIM vs CUSTOM v37\n"
        << "SPACE at physical zero. Then turn right, SPACE after stop, "
           "return to same physical zero, SPACE.\n";

    while (true) {
      const int64_t now = monotonicNs();

      if (now >= next_sync && pending_sync == 0) {
        pending_sync = now;
        sendTimesync(sfd, pending_sync, sys, comp);
        next_sync = now + kTimesyncPeriodNs;
      }

      pollfd pf[2] = {{cfd, POLLIN, 0}, {sfd, POLLIN, 0}};
      const int rc = poll(pf, 2, 2);
      if (rc < 0) {
        if (errno == EINTR) continue;
        fail("poll");
      }

      if (pf[1].revents & POLLIN) {
        uint8_t data[8192];
        for (;;) {
          const ssize_t n = read(sfd, data, sizeof(data));
          if (n == -1 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
          if (n <= 0) break;

          for (ssize_t i = 0; i < n; ++i) {
            if (!mavlink_parse_char(MAVLINK_COMM_0, data[i], &msg, &mav_status)) {
              continue;
            }

            const int64_t rx = monotonicNs();

            if (msg.msgid == MAVLINK_MSG_ID_TIMESYNC) {
              mavlink_timesync_t ts{};
              mavlink_msg_timesync_decode(&msg, &ts);
              if (ts.tc1 != 0 && pending_sync != 0 && ts.ts1 == pending_sync) {
                TimeSyncSample sample;
                sample.t0_rpi_ns = pending_sync;
                sample.t1_rpi_ns = rx;
                sample.fc_ns = ts.tc1;
                sample.rtt_ns = rx - pending_sync;
                sample.rpi_mid_ns = pending_sync + sample.rtt_ns / 2;
                sample.good = sample.rtt_ns > 0 &&
                              nsToMs(sample.rtt_ns) <= kMaxTimesyncRttMs;
                sync.push_back(sample);
                pending_sync = 0;
                mapping = estimateClockMapping(sync);
              }
            } else if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE) {
              mavlink_attitude_t att{};
              mavlink_msg_attitude_decode(&msg, &att);
              const double yaw_deg = att.yaw * 180.0 / kPi;
              std::lock_guard<std::mutex> lock(tel.mutex);
              if (tel.have_prev_yaw) {
                tel.fc_accum_yaw += wrapDeg(yaw_deg - tel.prev_yaw);
              }
              tel.prev_yaw = yaw_deg;
              tel.have_prev_yaw = true;
              tel.fc_roll = att.roll * 180.0 / kPi;
              tel.fc_pitch = att.pitch * 180.0 / kPi;
              tel.fc_yaw = yaw_deg;
              tel.fc_valid = true;
            } else if (msg.msgid == MAVLINK_MSG_ID_HIGHRES_IMU) {
              mavlink_highres_imu_t h{};
              mavlink_msg_highres_imu_decode(&msg, &h);

              ImuSample10 im{h.time_usec,
                             h.xacc,
                             -h.yacc,
                             -h.zacc,
                             h.xgyro,
                             -h.ygyro,
                             -h.zgyro};

              double yaw_rel = 0.0;
              {
                std::lock_guard<std::mutex> lock(tel.mutex);
                tel.ax = im.ax;
                tel.ay = im.ay;
                tel.az = im.az;
                tel.gx = im.gx;
                tel.gy = im.gy;
                tel.gz = im.gz;
                yaw_rel = tel.fc_accum_yaw - ref_yaw;
              }

              if (mapping.valid && rx - mapping.last_update_ns <= kMappingStaleNs) {
                VIO::ImuAccGyr imu_data;
                imu_data << im.ax, im.ay, im.az, im.gx, im.gy, im.gz;
                pipe->fillSingleImuQueue(VIO::ImuMeasurement(
                    mapping.map(static_cast<int64_t>(h.time_usec) * 1000LL),
                    imu_data));
              }

              process37(state, im, *pipe, yaw_rel);

              if (ready && !done) {
                const bool still =
                    std::sqrt(im.gx * im.gx + im.gy * im.gy + im.gz * im.gz) <=
                        kGyroStill37 &&
                    std::abs(std::sqrt(im.ax * im.ax + im.ay * im.ay +
                                       im.az * im.az) -
                             9.81) <= kAccTol37;
                if (still) {
                  stable = std::min(stable + 1, kStable37);
                } else {
                  stable = 0;
                }
              }
            }
          }
        }
      }

      if (pending_sync != 0 && monotonicNs() - pending_sync > 20000000LL) {
        pending_sync = 0;
      }

      if (pf[0].revents & POLLIN) {
        for (;;) {
          v4l2_buffer buf{};
          buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
          buf.memory = V4L2_MEMORY_MMAP;
          if (xioctl(cfd, VIDIOC_DQBUF, &buf) == -1) {
            if (errno == EAGAIN) break;
            fail("DQBUF");
          }

          const int64_t ts = jtzero::timesync::correctCameraTimestampNs(
              timevalToNs(buf.timestamp));
          bool continuity_ok = true;
          if (have_prev) {
            const int64_t dt = ts - prev_ts;
            continuity_ok = buf.sequence == prev_seq + 1U && dt > 0 &&
                            dt <= 20000000LL;
          }
          prev_seq = buf.sequence;
          prev_ts = ts;
          have_prev = true;

          const bool due =
              last_selected == 0 || ts - last_selected >= 30000000LL;
          if (continuity_ok && due && mapping.valid) {
            std::vector<unsigned char> jpeg(buf.bytesused);
            std::memcpy(jpeg.data(), bufs[buf.index].start, buf.bytesused);
            cv::Mat gray = cv::imdecode(jpeg, cv::IMREAD_GRAYSCALE);
            if (!gray.empty()) {
              last_gray = gray;
              pipe->fillLeftFrameQueue(std::make_unique<VIO::Frame>(
                  frame_id++, ts, vp.camera_params_.at(0), gray.clone()));
              last_selected = ts;
            }
          }

          if (xioctl(cfd, VIDIOC_QBUF, &buf) == -1) fail("QBUF");
        }
      }

      if (now >= next_hud) {
        hud37(last_gray, tel, ready, turned, done, stable, state, ref_yaw);
        next_hud = now + kHudPeriodNs;
      }

      const int key = cv::waitKey(1) & 0xff;
      if (key == 27 || key == 'q' || key == 'Q') {
        if (!done) aborted = true;
        break;
      }

      if (key == ' ' && !ready) {
        Backend37 b;
        if (pipe->latest(&b)) {
          {
            std::lock_guard<std::mutex> lock(tel.mutex);
            ref_yaw = tel.fc_accum_yaw;
          }
          state.R = b.R;
          state.R0 = b.R;
          state.V = b.V;
          state.BA = b.BA;
          state.BG = b.BG;
          state.last_us = 0;
          state.armed = true;
          ready = true;
          stable = 0;
          std::cout << "[ZERO] KF " << b.kf
                    << " R/V/BA/BG shared. V0=[" << b.V.transpose()
                    << "] BA=[" << b.BA.transpose() << "] BG=["
                    << b.BG.transpose() << "]\n";
        }
      } else if (key == ' ' && ready && !turned && stable >= kStable37) {
        turned = true;
        stable = 0;
        std::cout
            << "[STEP] right position confirmed. Return to physical zero.\n";
      } else if (key == ' ' && turned && !done && stable >= kStable37) {
        done = true;
        std::cout << "[TEST] physical zero confirmed.\n";
      } else if (key == ' ' && done) {
        break;
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    pipe->shutdown();
    if (pipe_thread.joinable()) pipe_thread.join();
    pipeline_started = false;

    save37(state);

    if (imu_req) {
      requestRate(sfd, sys, comp, MAVLINK_MSG_ID_HIGHRES_IMU, 0);
    }
    if (att_req) {
      requestRate(sfd, sys, comp, MAVLINK_MSG_ID_ATTITUDE, 0);
    }
    if (streaming) {
      v4l2_buf_type stop_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      xioctl(cfd, VIDIOC_STREAMOFF, &stop_type);
      streaming = false;
    }
    for (auto& b : bufs) {
      if (b.start && b.start != MAP_FAILED) munmap(b.start, b.length);
    }
    if (cfd >= 0) close(cfd);
    if (sfd >= 0) close(sfd);
    cv::destroyAllWindows();

    std::cout
        << "\n============================================================\n"
        << "JT-ZERO PIM vs CUSTOM v37 RESULT\n"
        << "============================================================\n"
        << "aborted: " << (aborted ? "yes" : "no") << '\n'
        << "completed: " << (done ? "yes" : "no") << '\n'
        << "rows: " << state.rows.size() << '\n'
        << "max PIM Vxy: " << state.max_pim << " m/s\n"
        << "max CUSTOM Vxy: " << state.max_custom << " m/s\n"
        << "max |Vcustom-Vbackend|: " << state.max_dv << " m/s\n";

    if (state.max_pim > 2.0 && state.max_custom > 0.6 * state.max_pim) {
      std::cout
          << "RESULT: custom integration reproduces most of PIM runaway; "
             "investigate IMU measurements/frame/calibration rather than GTSAM.\n";
    } else if (state.max_pim > 2.0 &&
               state.max_custom < 0.3 * state.max_pim) {
      std::cout
          << "RESULT: PIM and custom integration diverge strongly; inspect "
             "GTSAM preintegration configuration/initialization.\n";
    } else {
      std::cout
          << "RESULT: inspect CSV and time history; separation is not decisive.\n";
    }

    std::cout << "CSV: " << kCsv37 << "\nOpen CSV:\n  code " << kCsv37 << '\n';
    return aborted ? 2 : 0;
  } catch (const std::exception& e) {
    if (pipe) pipe->shutdown();
    if (pipeline_started && pipe_thread.joinable()) pipe_thread.join();
    if (streaming && cfd >= 0) {
      v4l2_buf_type stop_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      xioctl(cfd, VIDIOC_STREAMOFF, &stop_type);
    }
    for (auto& b : bufs) {
      if (b.start && b.start != MAP_FAILED) munmap(b.start, b.length);
    }
    if (cfd >= 0) close(cfd);
    if (sfd >= 0) close(sfd);
    cv::destroyAllWindows();
    std::cerr << "[FATAL] " << e.what() << '\n';
    return 1;
  }
}
