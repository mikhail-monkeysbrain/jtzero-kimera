// JT-ZERO v43: TRUE IMU-only control with fixed validated gyro cross-axis correction.
// Reuses v10 diagnostic infrastructure, but feeds corrected FLU gyro to both
// the independent gyro integrator and Kimera.

#define JTZERO_V10_NO_MAIN
#include "live_mono_imu_integrator_v10.cpp"
#undef JTZERO_V10_NO_MAIN

namespace {

constexpr const char* kCsv43 = "/home/vio/jtzero_live_imu_integrator_v43.csv";
constexpr double kGyroCx43 = 0.014570;
constexpr double kGyroCy43 = 0.082383;

void saveCsv43(const jtzero_v10::Integrator10& in) {
  std::ofstream f(kCsv43, std::ios::trunc);
  f << "imu_us,dt,ax,ay,az,gx,gy,gz,fcR_roll,fcR_pitch,fcR_yaw,gyroR_roll,gyroR_pitch,gyroR_yaw,a_fc_x,a_fc_y,a_fc_z,a_gyro_x,a_gyro_y,a_gyro_z,v_fc_x,v_fc_y,v_fc_z,v_gyro_x,v_gyro_y,v_gyro_z,backend_vxy,pim_vxy\n";
  f << std::fixed << std::setprecision(9);
  for (const auto& r : in.rows) {
    f << r.imu_us << ',' << r.dt << ',' << r.ax << ',' << r.ay << ',' << r.az << ','
      << r.gx << ',' << r.gy << ',' << r.gz << ','
      << r.fc_roll << ',' << r.fc_pitch << ',' << r.fc_yaw << ','
      << r.gyro_roll << ',' << r.gyro_pitch << ',' << r.gyro_yaw << ','
      << r.afx << ',' << r.afy << ',' << r.afz << ','
      << r.agx << ',' << r.agy << ',' << r.agz << ','
      << r.vfx << ',' << r.vfy << ',' << r.vfz << ','
      << r.vgx << ',' << r.vgy << ',' << r.vgz << ','
      << r.backend_vxy << ',' << r.pim_vxy << '\n';
  }
}

}  // namespace

int main(int argc, char** argv) {
  using namespace jtzero_v10;

  google::InitGoogleLogging(argv[0]);
  FLAGS_visualize = false;
  FLAGS_viz_type = 2;
  FLAGS_use_lcd = false;
  FLAGS_log_output = false;
  FLAGS_extract_planes_from_the_scene = false;

  if (!std::getenv("JTZERO_DIAG_IMU_ONLY")) {
    std::cerr << "[FATAL] JTZERO_DIAG_IMU_ONLY is not set\n";
    return 1;
  }

  const std::string params = argc > 1 ? argv[1] : "params/JTZeroMonoFLUZeroLever";
  int cfd = -1, sfd = -1;
  bool streaming = false, pipeline_started = false, aborted = false;
  bool imu_req = false, att_req = false;
  uint8_t sys = 0, comp = 0;
  std::vector<CameraBuffer> bufs;
  std::shared_ptr<Pipeline10> pipe;
  std::thread pipe_thread;
  Telemetry tel;

  try {
    VIO::VioParams vp(params);
    pipe = std::make_shared<Pipeline10>(vp);
    pipe->installCallback();
    pipe_thread = std::thread([pipe]() { pipe->spin(); });
    pipeline_started = true;

    sfd = openSerial();
    mavlink_status_t mst{};
    mavlink_message_t msg{};
    std::cout << "[MAV] waiting for HEARTBEAT...\n";
    int64_t dl = monotonicNs() + 10000000000LL;
    while (monotonicNs() < dl && !sys) {
      pollfd p{sfd, POLLIN, 0};
      if (poll(&p, 1, 100) <= 0) continue;
      uint8_t b[2048];
      ssize_t n = read(sfd, b, sizeof(b));
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
    cv::namedWindow(kWindow10, cv::WINDOW_NORMAL);
    cv::resizeWindow(kWindow10, 1440, 940);

    std::vector<TimeSyncSample> sync;
    ClockMapping mapping;
    int64_t pending_sync = 0;
    int64_t next_sync = monotonicNs();
    int64_t next_hud = monotonicNs();
    uint32_t prev_seq = 0;
    int64_t prev_ts = 0, last_sel = 0;
    bool have_prev = false;
    VIO::FrameId fid = 0;
    cv::Mat last_gray(kHeight, kWidth, CV_8UC1, cv::Scalar(0));

    std::deque<ImuSample10> iq;
    AttSample10 prev_att{}, cur_att{};
    bool have_att = false;
    Integrator10 integ;
    bool ready = false, returning = false, done = false;
    int stable = 0;
    double ref_roll = 0, ref_pitch = 0, ref_accum = 0;
    size_t raw = 0, sel = 0, imu_rx = 0, imu_fed = 0, att_rx = 0;

    std::cout << "\nJT-ZERO IMU INTEGRATOR v43 — FIXED GYRO CORRECTION\n"
              << "SPACE fixes zero, BA and BG. Test: YAW +80 then return to zero.\n"
              << "gyro correction: wx += " << kGyroCx43 << "*wz, wy += "
              << kGyroCy43 << "*wz\n";

    while (true) {
      const int64_t now = monotonicNs();
      if (now >= next_sync && pending_sync == 0) {
        pending_sync = now;
        sendTimesync(sfd, pending_sync, sys, comp);
        next_sync = now + kTimesyncPeriodNs;
      }

      pollfd pf[2] = {{cfd, POLLIN, 0}, {sfd, POLLIN, 0}};
      int rc = poll(pf, 2, 2);
      if (rc < 0) {
        if (errno == EINTR) continue;
        fail("poll");
      }

      if (pf[1].revents & POLLIN) {
        uint8_t b[8192];
        for (;;) {
          ssize_t n = read(sfd, b, sizeof(b));
          if (n == -1 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
          if (n <= 0) break;
          for (ssize_t i = 0; i < n; ++i) {
            if (!mavlink_parse_char(MAVLINK_COMM_0, b[i], &msg, &mst)) continue;
            const int64_t rx = monotonicNs();

            if (msg.msgid == MAVLINK_MSG_ID_TIMESYNC) {
              mavlink_timesync_t ts{};
              mavlink_msg_timesync_decode(&msg, &ts);
              if (ts.tc1 != 0 && pending_sync && ts.ts1 == pending_sync) {
                TimeSyncSample q;
                q.t0_rpi_ns = pending_sync;
                q.t1_rpi_ns = rx;
                q.fc_ns = ts.tc1;
                q.rtt_ns = rx - pending_sync;
                q.rpi_mid_ns = pending_sync + q.rtt_ns / 2;
                q.good = q.rtt_ns > 0 && nsToMs(q.rtt_ns) <= kMaxTimesyncRttMs;
                sync.push_back(q);
                pending_sync = 0;
                mapping = estimateClockMapping(sync);
              }
            } else if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE) {
              ++att_rx;
              mavlink_attitude_t a{};
              mavlink_msg_attitude_decode(&msg, &a);
              AttSample10 na{(uint64_t)a.time_boot_ms * 1000ULL,
                             a.roll * 180.0 / kPi,
                             a.pitch * 180.0 / kPi,
                             a.yaw * 180.0 / kPi};
              {
                std::lock_guard<std::mutex> l(tel.mutex);
                if (tel.have_prev_yaw) tel.fc_accum_yaw += wrapDeg(na.yaw - tel.prev_yaw);
                tel.prev_yaw = na.yaw;
                tel.have_prev_yaw = true;
                tel.fc_roll = na.roll;
                tel.fc_pitch = na.pitch;
                tel.fc_yaw = na.yaw;
                tel.fc_valid = true;
              }

              if (have_att) {
                prev_att = cur_att;
                cur_att = na;
                while (!iq.empty() && iq.front().us <= cur_att.us) {
                  auto im = iq.front();
                  if (im.us >= prev_att.us) {
                    processOne(integ, im, interpR(prev_att, cur_att, im.us), *pipe);
                    iq.pop_front();
                  } else {
                    iq.pop_front();
                  }
                }
              } else {
                cur_att = na;
                have_att = true;
              }

              if (ready && !done) {
                double rr, pp, yy, gn, an;
                {
                  std::lock_guard<std::mutex> l(tel.mutex);
                  rr = wrapDeg(tel.fc_roll - ref_roll);
                  pp = wrapDeg(tel.fc_pitch - ref_pitch);
                  yy = tel.fc_accum_yaw - ref_accum;
                  gn = std::sqrt(tel.gx * tel.gx + tel.gy * tel.gy + tel.gz * tel.gz);
                  an = std::sqrt(tel.ax * tel.ax + tel.ay * tel.ay + tel.az * tel.az);
                }
                double ty = returning ? 0 : 80;
                double tol = returning ? 3 : 2;
                bool ok = std::abs(yy - ty) <= tol && std::abs(pp) <= 3 &&
                          std::abs(rr) <= 3 && gn <= kGyroStill10 &&
                          std::abs(an - 9.81) <= kAccTol10;
                if (ok) ++stable;
                else stable = 0;
                if (stable >= kStable10) {
                  stable = 0;
                  if (!returning) {
                    returning = true;
                    std::cout << "[STEP] YAW +80 fixed. Return to zero. FCint Vxy="
                              << integ.vfc.head<2>().norm() << " GYROint Vxy="
                              << integ.vgyro.head<2>().norm() << "\n";
                  } else {
                    done = true;
                    std::cout << "[TEST] completed at zero.\n";
                  }
                }
              }
            } else if (msg.msgid == MAVLINK_MSG_ID_HIGHRES_IMU) {
              ++imu_rx;
              mavlink_highres_imu_t h{};
              mavlink_msg_highres_imu_decode(&msg, &h);

              // ArduPilot FRD -> Kimera FLU.
              const double gx = h.xgyro;
              const double gy = -h.ygyro;
              const double gz = -h.zgyro;

              // Fixed coefficients learned on v41 and independently validated by v42.
              const double gx_corr = gx + kGyroCx43 * gz;
              const double gy_corr = gy + kGyroCy43 * gz;

              ImuSample10 im{h.time_usec,
                             h.xacc, -h.yacc, -h.zacc,
                             gx_corr, gy_corr, gz};
              {
                std::lock_guard<std::mutex> l(tel.mutex);
                tel.ax = im.ax; tel.ay = im.ay; tel.az = im.az;
                tel.gx = im.gx; tel.gy = im.gy; tel.gz = im.gz;
              }
              iq.push_back(im);
              while (iq.size() > 200) iq.pop_front();

              if (mapping.valid && rx - mapping.last_update_ns <= kMappingStaleNs) {
                VIO::ImuAccGyr d;
                d << im.ax, im.ay, im.az, im.gx, im.gy, im.gz;
                pipe->fillSingleImuQueue(
                    VIO::ImuMeasurement(mapping.map((int64_t)h.time_usec * 1000LL), d));
                ++imu_fed;
              }
            }
          }
        }
      }

      if (pending_sync && monotonicNs() - pending_sync > 20000000LL) pending_sync = 0;

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
          const int64_t ts = jtzero::timesync::correctCameraTimestampNs(timevalToNs(b.timestamp));
          bool ok = true;
          if (have_prev) {
            int64_t dt = ts - prev_ts;
            ok = b.sequence == prev_seq + 1U && dt > 0 && dt <= 20000000LL;
          }
          prev_seq = b.sequence;
          prev_ts = ts;
          have_prev = true;
          bool due = last_sel == 0 || ts - last_sel >= 30000000LL;
          if (ok && due && mapping.valid) {
            std::vector<unsigned char> jpeg(b.bytesused);
            std::memcpy(jpeg.data(), bufs[b.index].start, b.bytesused);
            cv::Mat gray = cv::imdecode(jpeg, cv::IMREAD_GRAYSCALE);
            if (!gray.empty()) {
              last_gray = gray;
              pipe->fillLeftFrameQueue(
                  std::make_unique<VIO::Frame>(fid++, ts, vp.camera_params_.at(0), gray.clone()));
              last_sel = ts;
              ++sel;
            }
          }
          if (xioctl(cfd, VIDIOC_QBUF, &b) == -1) fail("QBUF");
        }
      }

      if (now >= next_hud) {
        drawHud(last_gray, tel, ready, returning, done, stable, integ,
                ref_roll, ref_pitch, ref_accum);
        next_hud = now + kHudPeriodNs;
      }

      int key = cv::waitKey(1) & 0xff;
      if (key == 27 || key == 'q' || key == 'Q') {
        if (!done) aborted = true;
        break;
      }
      if (key == ' ' && !ready) {
        Backend10 b;
        bool bv = pipe->latest(&b), fv = false;
        {
          std::lock_guard<std::mutex> l(tel.mutex);
          fv = tel.fc_valid;
          if (fv) {
            ref_roll = tel.fc_roll;
            ref_pitch = tel.fc_pitch;
            ref_accum = tel.fc_accum_yaw;
          }
        }
        if (bv && fv && have_att) {
          integ.fixed_ba = Eigen::Vector3d(b.bax, b.bay, b.baz);
          integ.fixed_bg = Eigen::Vector3d(b.bgx, b.bgy, b.bgz);
          integ.Rgyro = fcRnedFlu(cur_att.roll, cur_att.pitch, cur_att.yaw);
          integ.vfc.setZero();
          integ.vgyro.setZero();
          integ.last_us = 0;
          integ.armed = true;
          ready = true;
          std::cout << "[ZERO] fixed BA [" << b.bax << ", " << b.bay << ", " << b.baz
                    << "] BG [" << b.bgx << ", " << b.bgy << ", " << b.bgz << "]\n";
        }
      } else if (key == ' ' && done) {
        break;
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    pipe->shutdown();
    if (pipe_thread.joinable()) pipe_thread.join();
    pipeline_started = false;
    saveCsv43(integ);
    cv::destroyAllWindows();
    if (imu_req) requestRate(sfd, sys, comp, MAVLINK_MSG_ID_HIGHRES_IMU, 0);
    if (att_req) requestRate(sfd, sys, comp, MAVLINK_MSG_ID_ATTITUDE, 0);
    if (streaming) {
      v4l2_buf_type t = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      xioctl(cfd, VIDIOC_STREAMOFF, &t);
    }
    for (auto& b : bufs) if (b.start && b.start != MAP_FAILED) munmap(b.start, b.length);
    if (cfd >= 0) close(cfd);
    if (sfd >= 0) close(sfd);

    std::cout << "\n============================================================\n"
              << "JT-ZERO IMU INTEGRATOR v43 CORRECTED RESULT\n"
              << "============================================================\n"
              << "aborted: " << (aborted ? "yes" : "no") << "\n"
              << "completed: " << (done ? "yes" : "no") << "\n"
              << "rows: " << integ.rows.size() << "\n"
              << "max FC-integrator Vxy: " << integ.max_fc_vxy << " m/s\n"
              << "max GYRO-integrator Vxy: " << integ.max_gyro_vxy << " m/s\n"
              << "max backend Vxy: " << integ.max_backend_vxy << " m/s\n"
              << "max PIM Vxy: " << integ.max_pim_vxy << " m/s\n"
              << "CSV: " << kCsv43 << "\n"
              << "Open CSV:\n  code " << kCsv43 << "\n";
    return aborted ? 2 : 0;
  } catch (const std::exception& e) {
    if (pipe) pipe->shutdown();
    if (pipeline_started && pipe_thread.joinable()) pipe_thread.join();
    if (streaming && cfd >= 0) {
      v4l2_buf_type t = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      xioctl(cfd, VIDIOC_STREAMOFF, &t);
    }
    for (auto& b : bufs) if (b.start && b.start != MAP_FAILED) munmap(b.start, b.length);
    if (cfd >= 0) close(cfd);
    if (sfd >= 0) close(sfd);
    cv::destroyAllWindows();
    std::cerr << "[FATAL] " << e.what() << "\n";
    return 1;
  }
}
