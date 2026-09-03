// JT-ZERO P11 FINAL REGRESSION
// 3 x [YAW +90 -> HOLD -> HOME -> STILL]
// Records replay-compatible HIGHRES_IMU + camera,
// plus SCALED_IMU1/2, ATTITUDE, ATTITUDE_QUATERNION and VIBRATION.

#define main jtzero_camera_imu_logger_unused_main_p11
#include "camera_imu_extrinsics_logger.cpp"
#undef main

namespace {

constexpr const char* OUT_CSV   = "/home/vio/jtzero_p11_final.csv";
constexpr const char* OUT_CAM   = "/home/vio/jtzero_p11_final_camera.csv";
constexpr const char* OUT_MJPEG = "/home/vio/jtzero_p11_final.mjpg";
constexpr const char* OUT_ATT   = "/home/vio/jtzero_p11_final_attitude.csv";
constexpr const char* OUT_FC    = "/home/vio/jtzero_p11_final_fc.csv";

constexpr const char* WIN = "JT-Zero: P11 финальная регрессия";

constexpr double ZERO_SEC = 3.0;
constexpr double PRE_SEC = 15.0;
constexpr double ROT_SEC = 10.0;
constexpr double HOLD_SEC = 10.0;
constexpr double HOME_SEC = 10.0;
constexpr double HOME_HOLD_SEC = 10.0;
constexpr double FINAL_HOLD_SEC = 15.0;

constexpr double TOTAL_SEC =
    PRE_SEC +
    2.0 * (ROT_SEC + HOLD_SEC + HOME_SEC + HOME_HOLD_SEC) +
    (ROT_SEC + HOLD_SEC + HOME_SEC + FINAL_HOLD_SEC);

constexpr double RP_OK_DEG = 4.0;
constexpr double YAW_TARGET_DEG = 90.0;
constexpr double YAW_OK_DEG = 8.0;

constexpr double EXTRA_IMU_HZ = 50.0;
constexpr double EXTRA_ATT_HZ = 20.0;

struct AttP11 {
  int64_t recv_ns = 0;
  int64_t src_ns = 0;
  double r = 0, p = 0, y = 0;
  double rs = 0, ps = 0, ys = 0;
};

struct FcDiag {
  int64_t recv_ns = 0;
  int64_t src_ns = 0;
  std::string phase;
  std::string type;

  int imu_id = -1;

  double ax = 0, ay = 0, az = 0;
  double gx = 0, gy = 0, gz = 0;

  double roll = 0, pitch = 0, yaw = 0;

  double q1 = 0, q2 = 0, q3 = 0, q4 = 0;

  double vibration_x = 0;
  double vibration_y = 0;
  double vibration_z = 0;

  uint32_t clipping_0 = 0;
  uint32_t clipping_1 = 0;
  uint32_t clipping_2 = 0;

  double temperature = 0;
};

double relDegP11(double a, double z) {
  return wrap180(a - z);
}

struct PhaseP11 {
  std::string name;
  std::string instruction;
  double remain = 0;
  double target_yaw = 0;
  bool moving = false;
  int cycle = 0;
};

PhaseP11 phaseP11(double t) {
  if (t < PRE_SEC) {
    return {"PRE",
            "НЕ ДВИГАТЬ",
            PRE_SEC - t,
            0,
            false,
            0};
  }

  t -= PRE_SEC;

  for (int cycle = 1; cycle <= 3; ++cycle) {
    if (t < ROT_SEC) {
      return {
          "YAW" + std::to_string(cycle) + "_OUT",
          "ПЛАВНО ПОВЕРНИ YAW ДО +90°",
          ROT_SEC - t,
          90,
          true,
          cycle
      };
    }
    t -= ROT_SEC;

    if (t < HOLD_SEC) {
      return {
          "YAW" + std::to_string(cycle) + "_HOLD",
          "УДЕРЖИВАЙ +90°. НЕ НАКЛОНЯЙ",
          HOLD_SEC - t,
          90,
          false,
          cycle
      };
    }
    t -= HOLD_SEC;

    if (t < HOME_SEC) {
      return {
          "YAW" + std::to_string(cycle) + "_HOME",
          "ПЛАВНО ВЕРНИ YAW В 0°",
          HOME_SEC - t,
          0,
          true,
          cycle
      };
    }
    t -= HOME_SEC;

    const double still =
        (cycle == 3) ? FINAL_HOLD_SEC : HOME_HOLD_SEC;

    if (t < still) {
      return {
          cycle == 3
              ? "FINAL"
              : "HOME" + std::to_string(cycle),
          cycle == 3
              ? "ФИНАЛ: НЕ ДВИГАТЬ"
              : "НЕ ДВИГАТЬ. YAW ДОЛЖЕН БЫТЬ 0°",
          still - t,
          0,
          false,
          cycle
      };
    }

    t -= still;
  }

  return {"DONE", "ГОТОВО", 0, 0, false, 3};
}

void textP11(cv::Mat& im,
             const std::string& s,
             cv::Point p,
             cv::Scalar c = cv::Scalar(255,255,255),
             int px = 22) {
  cv::addText(
      im,
      s,
      p,
      "DejaVu Sans",
      px,
      c,
      cv::QT_FONT_NORMAL,
      cv::QT_STYLE_NORMAL,
      0);
}

int gaugeXP11(double v,
              double minv,
              double maxv,
              int x,
              int w) {
  double u = (v - minv) / (maxv - minv);
  u = std::clamp(u, 0.0, 1.0);
  return x + static_cast<int>(std::lround(u * w));
}

void drawGaugeP11(cv::Mat& im,
                  int x,
                  int y,
                  int w,
                  const std::string& name,
                  double value,
                  double target,
                  double tol,
                  double minv,
                  double maxv) {
  cv::rectangle(im, {x,y,w,28}, {70,70,70}, cv::FILLED);

  int g0 = gaugeXP11(target - tol, minv, maxv, x, w);
  int g1 = gaugeXP11(target + tol, minv, maxv, x, w);
  if (g1 < g0) std::swap(g0,g1);

  cv::rectangle(
      im,
      {g0,y,std::max(1,g1-g0),28},
      {0,140,0},
      cv::FILLED);

  int zero = gaugeXP11(0, minv, maxv, x, w);
  int val = gaugeXP11(value, minv, maxv, x, w);
  int tgt = gaugeXP11(target, minv, maxv, x, w);

  cv::line(im, {zero,y-5}, {zero,y+33}, {160,160,160}, 1);
  cv::line(im, {tgt,y-5}, {tgt,y+33}, {0,255,255}, 2);

  const bool ok =
      std::abs(wrap180(value - target)) <= tol;

  cv::line(
      im,
      {val,y-8},
      {val,y+36},
      ok ? cv::Scalar(0,255,0) : cv::Scalar(0,0,255),
      4,
      cv::LINE_AA);

  std::ostringstream ss;
  ss << name << ": "
     << std::fixed << std::setprecision(1)
     << value << "°";

  textP11(im, ss.str(), {x,y-12});
}

void drawGuiP11(cv::Mat& screen,
                const cv::Mat& gray,
                double elapsed,
                double rr,
                double rp,
                double ry,
                bool have_att,
                bool zero_ready) {
  screen.setTo(cv::Scalar(18,18,18));

  if (!gray.empty()) {
    cv::Mat bgr;
    cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
    cv::resize(bgr, bgr, {760,570});
    bgr.copyTo(screen(cv::Rect(20,120,760,570)));

    cv::rectangle(
        screen,
        {20,120,760,570},
        {180,180,180},
        1);

    cv::line(screen,{400,120},{400,690},{100,100,100},1);
    cv::line(screen,{20,405},{780,405},{100,100,100},1);
  }

  textP11(
      screen,
      "JT-ZERO — P11 ФИНАЛЬНАЯ РЕГРЕССИЯ",
      {35,45},
      {255,255,255},
      24);

  if (!zero_ready) {
    textP11(
        screen,
        "КАЛИБРОВКА НУЛЯ: держи БПЛА неподвижно",
        {35,82},
        {0,220,255});
  } else {
    const auto ph = phaseP11(elapsed);

    std::ostringstream title;
    if (ph.cycle > 0)
      title << "ЦИКЛ " << ph.cycle << "/3 — ";
    title << ph.instruction;

    textP11(
        screen,
        title.str(),
        {35,82},
        {0,220,255});

    std::ostringstream ts;
    ts << "Осталось: "
       << std::fixed << std::setprecision(1)
       << std::max(0.0,ph.remain)
       << " с";

    textP11(screen,ts.str(),{850,82});

    drawGaugeP11(
        screen,840,185,380,
        "ROLL",rr,0,RP_OK_DEG,-20,20);

    drawGaugeP11(
        screen,840,285,380,
        "PITCH",rp,0,RP_OK_DEG,-20,20);

    drawGaugeP11(
        screen,840,385,380,
        "YAW",ry,ph.target_yaw,YAW_OK_DEG,-120,120);

    const bool rp_ok =
        std::abs(rr) <= RP_OK_DEG &&
        std::abs(rp) <= RP_OK_DEG;

    textP11(
        screen,
        rp_ok
            ? "ROLL/PITCH В ЗЕЛЁНОЙ ЗОНЕ"
            : "ВНИМАНИЕ: СЛИШКОМ БОЛЬШОЙ НАКЛОН",
        {840,475},
        rp_ok
            ? cv::Scalar(0,255,0)
            : cv::Scalar(0,0,255));

    if (ph.moving) {
      textP11(
          screen,
          "Двигай только вокруг вертикальной оси",
          {840,525});
    } else {
      const bool yaw_ok =
          std::abs(wrap180(ry - ph.target_yaw))
          <= YAW_OK_DEG;

      textP11(
          screen,
          yaw_ok
              ? "YAW В ЦЕЛЕВОЙ ЗОНЕ"
              : "ДОВЕДИ YAW ДО ЗЕЛЁНОЙ ЗОНЫ",
          {840,525},
          yaw_ok
              ? cv::Scalar(0,255,0)
              : cv::Scalar(0,0,255));
    }
  }

  if (!have_att) {
    textP11(
        screen,
        "НЕТ ATTITUDE ОТ FC",
        {840,620},
        {0,0,255});
  }

  textP11(
      screen,
      "ESC — аварийно завершить тест",
      {840,665});
}

void stopRatesP11(int sfd,
                  uint8_t sys,
                  uint8_t comp) {
  if (sfd < 0 || !sys) return;

  try {
    requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);
    requestRate(sfd,sys,comp,MAVLINK_MSG_ID_SCALED_IMU,0);
    requestRate(sfd,sys,comp,MAVLINK_MSG_ID_SCALED_IMU2,0);
    requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);
    requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE_QUATERNION,0);
    requestRate(sfd,sys,comp,MAVLINK_MSG_ID_VIBRATION,0);
  } catch (...) {}
}

void cleanupP11(int sfd,
                int cfd,
                bool streaming,
                std::vector<CameraBuffer>& buf,
                uint8_t sys,
                uint8_t comp,
                bool rates) {
  if (rates)
    stopRatesP11(sfd,sys,comp);

  if (streaming && cfd >= 0) {
    v4l2_buf_type t = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(cfd,VIDIOC_STREAMOFF,&t);
  }

  for (auto& b : buf)
    if (b.start && b.length)
      munmap(b.start,b.length);

  if (cfd >= 0) close(cfd);
  if (sfd >= 0) close(sfd);

  cv::destroyAllWindows();
}

} // namespace


int main() {
  int sfd = -1;
  int cfd = -1;

  bool streaming = false;
  bool rates = false;

  std::vector<CameraBuffer> buf;

  uint8_t sys = 0;
  uint8_t comp = 0;

  try {
    sfd = openSerial();

    std::cout
        << "[MAV] waiting for HEARTBEAT...\n";

    mavlink_status_t ms{};
    mavlink_message_t mm{};

    int64_t dl =
        monotonicNs() + 10000000000LL;

    while (monotonicNs() < dl && !sys) {
      pollfd p{sfd,POLLIN,0};

      if (poll(&p,1,100) > 0) {
        uint8_t b[2048];
        ssize_t n = read(sfd,b,sizeof(b));

        for (ssize_t i=0; i<n; ++i) {
          if (mavlink_parse_char(
                  MAVLINK_COMM_0,b[i],&mm,&ms) &&
              mm.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
            sys = mm.sysid;
            comp = mm.compid;
            break;
          }
        }
      }
    }

    if (!sys)
      throw std::runtime_error("HEARTBEAT timeout");

    requestRate(
        sfd,sys,comp,
        MAVLINK_MSG_ID_HIGHRES_IMU,
        IMU_RATE_HZ);

    requestRate(
        sfd,sys,comp,
        MAVLINK_MSG_ID_SCALED_IMU,
        EXTRA_IMU_HZ);

    requestRate(
        sfd,sys,comp,
        MAVLINK_MSG_ID_SCALED_IMU2,
        EXTRA_IMU_HZ);

    requestRate(
        sfd,sys,comp,
        MAVLINK_MSG_ID_ATTITUDE,
        EXTRA_ATT_HZ);

    requestRate(
        sfd,sys,comp,
        MAVLINK_MSG_ID_ATTITUDE_QUATERNION,
        EXTRA_ATT_HZ);

    requestRate(
        sfd,sys,comp,
        MAVLINK_MSG_ID_VIBRATION,
        EXTRA_ATT_HZ);

    rates = true;

    cfd = open(
        CAMERA_DEVICE,
        O_RDWR | O_NONBLOCK);

    if (cfd < 0)
      fail("open camera");

    configureCamera(cfd);
    buf = initCameraBuffers(cfd);

    v4l2_buf_type typ =
        V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (xioctl(cfd,VIDIOC_STREAMON,&typ) == -1)
      fail("STREAMON");

    streaming = true;

    discardWarmup(
        cfd,
        CAMERA_WARMUP_FRAMES);

    cv::namedWindow(
        WIN,
        cv::WINDOW_NORMAL);

    cv::setWindowProperty(
        WIN,
        cv::WND_PROP_FULLSCREEN,
        cv::WINDOW_FULLSCREEN);

    cv::Mat screen(
        720,1280,CV_8UC3);

    cv::Mat latest_gray;

    std::vector<CameraSample> cams;
    std::vector<ImuSample> imus;
    std::vector<TimeSyncSample> syncs;
    std::vector<AttP11> atts;
    std::vector<FcDiag> fcdiag;

    cams.reserve(6000);
    imus.reserve(40000);
    syncs.reserve(2000);
    atts.reserve(5000);
    fcdiag.reserve(50000);

    std::ofstream mj(
        OUT_MJPEG,
        std::ios::binary | std::ios::trunc);

    if (!mj)
      throw std::runtime_error(
          "Cannot create MJPEG");

    bool have_seq = false;
    uint32_t prev_seq = 0;
    uint64_t drops = 0;

    int64_t pending = 0;
    int64_t next_ts = monotonicNs();

    bool have_att = false;
    bool zero_ready = false;

    double ar = 0;
    double ap = 0;
    double ay = 0;

    double sum_r = 0;
    double sum_p = 0;
    double sum_sy = 0;
    double sum_cy = 0;

    size_t zn = 0;

    int64_t zero_start = 0;
    int64_t test_start = 0;
    int64_t last_gui = 0;

    uint32_t preview_div = 0;

    tcflush(sfd,TCIFLUSH);

    std::memset(&ms,0,sizeof(ms));
    std::memset(&mm,0,sizeof(mm));

    std::cout
        << "[GUI] P11 final regression started.\n"
        << "[GUI] ZERO 3 s -> PRE 15 s -> "
        << "3 yaw out/home cycles.\n"
        << "[GUI] Total test after zero: "
        << TOTAL_SEC << " s\n";

    bool abort = false;

    while (!abort) {
      int64_t now = monotonicNs();

      double elapsed =
          zero_ready
              ? (now-test_start)*1e-9
              : 0.0;

      if (zero_ready &&
          elapsed >= TOTAL_SEC)
        break;

      if (now >= next_ts &&
          pending == 0) {
        pending = now;
        sendTimesync(
            sfd,pending,sys,comp);
        next_ts =
            now + TIMESYNC_PERIOD_NS;
      }

      pollfd pf[2] = {
          {cfd,POLLIN,0},
          {sfd,POLLIN,0}
      };

      int rc = poll(pf,2,2);

      if (rc < 0) {
        if (errno == EINTR)
          continue;
        fail("poll");
      }

      if (pf[0].revents & POLLIN) {
        for (;;) {
          v4l2_buffer b{};
          b.type =
              V4L2_BUF_TYPE_VIDEO_CAPTURE;
          b.memory =
              V4L2_MEMORY_MMAP;

          if (xioctl(
                  cfd,
                  VIDIOC_DQBUF,
                  &b) == -1) {
            if (errno == EAGAIN)
              break;
            fail("DQBUF");
          }

          int64_t recv =
              monotonicNs();

          int64_t v4 =
              timevalToNs(b.timestamp);

          int64_t corr =
              jtzero::timesync::
              correctCameraTimestampNs(v4);

          if (have_seq) {
            uint32_t ex =
                prev_seq + 1;

            if (b.sequence != ex)
              drops +=
                  uint32_t(
                      b.sequence-ex);
          }

          prev_seq = b.sequence;
          have_seq = true;

          uint64_t off =
              static_cast<uint64_t>(
                  mj.tellp());

          mj.write(
              reinterpret_cast<const char*>(
                  buf[b.index].start),
              b.bytesused);

          cams.push_back({
              recv,
              v4,
              corr,
              b.sequence,
              b.flags,
              b.bytesused,
              off
          });

          if ((preview_div++ % 4) == 0) {
            std::vector<unsigned char> j(
                static_cast<unsigned char*>(
                    buf[b.index].start),
                static_cast<unsigned char*>(
                    buf[b.index].start)
                    + b.bytesused);

            latest_gray =
                cv::imdecode(
                    j,
                    cv::IMREAD_GRAYSCALE);
          }

          if (xioctl(
                  cfd,
                  VIDIOC_QBUF,
                  &b) == -1)
            fail("QBUF");
        }
      }

      if (pf[1].revents & POLLIN) {
        uint8_t by[8192];

        for (;;) {
          ssize_t n =
              read(sfd,by,sizeof(by));

          if (n == -1 &&
              (errno == EAGAIN ||
               errno == EWOULDBLOCK))
            break;

          if (n <= 0)
            break;

          for (ssize_t i=0; i<n; ++i) {
            if (!mavlink_parse_char(
                    MAVLINK_COMM_0,
                    by[i],
                    &mm,
                    &ms))
              continue;

            int64_t recv =
                monotonicNs();

            const double trel =
                zero_ready
                    ? (recv-test_start)*1e-9
                    : -1.0;

            const std::string ph =
                zero_ready
                    ? phaseP11(trel).name
                    : "ZERO";

            if (mm.msgid ==
                MAVLINK_MSG_ID_HIGHRES_IMU) {
              mavlink_highres_imu_t a{};
              mavlink_msg_highres_imu_decode(
                  &mm,&a);

              ImuSample s;
              s.recv_ns = recv;
              s.fc_ns =
                  static_cast<int64_t>(
                      a.time_usec) * 1000;
              s.xacc = a.xacc;
              s.yacc = a.yacc;
              s.zacc = a.zacc;
              s.xgyro = a.xgyro;
              s.ygyro = a.ygyro;
              s.zgyro = a.zgyro;
              s.temperature = a.temperature;
              s.fields_updated =
                  a.fields_updated;

#ifdef MAVLINK_MSG_HIGHRES_IMU_FIELD_ID_LEN
              s.imu_id = a.id;
#endif

              imus.push_back(s);

              FcDiag d;
              d.recv_ns = recv;
              d.src_ns = s.fc_ns;
              d.phase = ph;
              d.type = "HIGHRES_IMU";
              d.ax = a.xacc;
              d.ay = a.yacc;
              d.az = a.zacc;
              d.gx = a.xgyro;
              d.gy = a.ygyro;
              d.gz = a.zgyro;
              d.temperature = a.temperature;

#ifdef MAVLINK_MSG_HIGHRES_IMU_FIELD_ID_LEN
              d.imu_id = a.id;
#endif
              fcdiag.push_back(d);
            }
            else if (mm.msgid ==
                     MAVLINK_MSG_ID_SCALED_IMU) {
              mavlink_scaled_imu_t a{};
              mavlink_msg_scaled_imu_decode(
                  &mm,&a);

              FcDiag d;
              d.recv_ns = recv;
              d.src_ns =
                  static_cast<int64_t>(
                      a.time_boot_ms)
                  * 1000000LL;
              d.phase = ph;
              d.type = "SCALED_IMU";

              d.ax = a.xacc * 9.80665e-3;
              d.ay = a.yacc * 9.80665e-3;
              d.az = a.zacc * 9.80665e-3;

              d.gx = a.xgyro * 1e-3;
              d.gy = a.ygyro * 1e-3;
              d.gz = a.zgyro * 1e-3;

              fcdiag.push_back(d);
            }
            else if (mm.msgid ==
                     MAVLINK_MSG_ID_SCALED_IMU2) {
              mavlink_scaled_imu2_t a{};
              mavlink_msg_scaled_imu2_decode(
                  &mm,&a);

              FcDiag d;
              d.recv_ns = recv;
              d.src_ns =
                  static_cast<int64_t>(
                      a.time_boot_ms)
                  * 1000000LL;
              d.phase = ph;
              d.type = "SCALED_IMU2";

              d.ax = a.xacc * 9.80665e-3;
              d.ay = a.yacc * 9.80665e-3;
              d.az = a.zacc * 9.80665e-3;

              d.gx = a.xgyro * 1e-3;
              d.gy = a.ygyro * 1e-3;
              d.gz = a.zgyro * 1e-3;

              fcdiag.push_back(d);
            }
            else if (mm.msgid ==
                     MAVLINK_MSG_ID_ATTITUDE) {
              mavlink_attitude_t a{};
              mavlink_msg_attitude_decode(
                  &mm,&a);

              double r = rad2deg(a.roll);
              double p = rad2deg(a.pitch);
              double y = rad2deg(a.yaw);

              atts.push_back({
                  recv,
                  static_cast<int64_t>(
                      a.time_boot_ms)
                      * 1000000LL,
                  r,p,y,
                  a.rollspeed,
                  a.pitchspeed,
                  a.yawspeed
              });

              FcDiag d;
              d.recv_ns = recv;
              d.src_ns =
                  static_cast<int64_t>(
                      a.time_boot_ms)
                  * 1000000LL;
              d.phase = ph;
              d.type = "ATTITUDE";
              d.roll = r;
              d.pitch = p;
              d.yaw = y;
              d.gx = a.rollspeed;
              d.gy = a.pitchspeed;
              d.gz = a.yawspeed;
              fcdiag.push_back(d);

              have_att = true;

              if (!zero_ready) {
                if (zero_start == 0)
                  zero_start = recv;

                sum_r += r;
                sum_p += p;
                sum_sy += std::sin(a.yaw);
                sum_cy += std::cos(a.yaw);
                ++zn;

                if ((recv-zero_start)*1e-9
                        >= ZERO_SEC &&
                    zn > 20) {
                  ar = sum_r / zn;
                  ap = sum_p / zn;
                  ay = rad2deg(
                      std::atan2(
                          sum_sy,
                          sum_cy));

                  zero_ready = true;
                  test_start = recv;

                  std::cout
                      << "[ZERO] roll="
                      << ar
                      << " pitch="
                      << ap
                      << " yaw="
                      << ay
                      << "\n";
                }
              }
            }
            else if (mm.msgid ==
                     MAVLINK_MSG_ID_ATTITUDE_QUATERNION) {
              mavlink_attitude_quaternion_t a{};
              mavlink_msg_attitude_quaternion_decode(
                  &mm,&a);

              FcDiag d;
              d.recv_ns = recv;
              d.src_ns =
                  static_cast<int64_t>(
                      a.time_boot_ms)
                  * 1000000LL;
              d.phase = ph;
              d.type =
                  "ATTITUDE_QUATERNION";

              d.q1 = a.q1;
              d.q2 = a.q2;
              d.q3 = a.q3;
              d.q4 = a.q4;

              d.gx = a.rollspeed;
              d.gy = a.pitchspeed;
              d.gz = a.yawspeed;

              fcdiag.push_back(d);
            }
            else if (mm.msgid ==
                     MAVLINK_MSG_ID_VIBRATION) {
              mavlink_vibration_t a{};
              mavlink_msg_vibration_decode(
                  &mm,&a);

              FcDiag d;
              d.recv_ns = recv;
              d.src_ns =
                  static_cast<int64_t>(
                      a.time_usec)
                  * 1000LL;
              d.phase = ph;
              d.type = "VIBRATION";

              d.vibration_x = a.vibration_x;
              d.vibration_y = a.vibration_y;
              d.vibration_z = a.vibration_z;

              d.clipping_0 = a.clipping_0;
              d.clipping_1 = a.clipping_1;
              d.clipping_2 = a.clipping_2;

              fcdiag.push_back(d);
            }
            else if (mm.msgid ==
                     MAVLINK_MSG_ID_TIMESYNC) {
              mavlink_timesync_t a{};
              mavlink_msg_timesync_decode(
                  &mm,&a);

              if (a.tc1 != 0 &&
                  pending != 0 &&
                  a.ts1 == pending) {
                TimeSyncSample s;

                s.t0_rpi_ns = pending;
                s.t1_rpi_ns = recv;
                s.fc_ns = a.tc1;
                s.rtt_ns = recv-pending;
                s.rpi_mid_ns =
                    pending + s.rtt_ns/2;

                s.good =
                    s.rtt_ns > 0 &&
                    nsToMs(s.rtt_ns)
                        <= MAX_TIMESYNC_RTT_MS;

                syncs.push_back(s);
                pending = 0;
              }
            }
          }
        }
      }

      if (pending &&
          monotonicNs()-pending
              > 20000000LL)
        pending = 0;

      now = monotonicNs();

      if (now-last_gui >
          33000000LL) {
        last_gui = now;

        double rr = 0;
        double rp = 0;
        double ry = 0;

        if (have_att &&
            !atts.empty() &&
            zero_ready) {
          rr =
              relDegP11(
                  atts.back().r,ar);

          rp =
              relDegP11(
                  atts.back().p,ap);

          ry =
              relDegP11(
                  atts.back().y,ay);
        }

        drawGuiP11(
            screen,
            latest_gray,
            zero_ready
                ? (now-test_start)*1e-9
                : 0.0,
            rr,rp,ry,
            have_att,
            zero_ready);

        cv::imshow(WIN,screen);

        int k = cv::waitKeyEx(1);

        if (k == 27)
          abort = true;
      }
    }

    mj.flush();
    mj.close();

    ClockMapping map =
        estimateClockMapping(syncs);

    if (!map.valid)
      throw std::runtime_error(
          "Not enough valid TIMESYNC samples");

    {
      std::ofstream ci(OUT_CAM);

      ci
          << "sequence,v4l2_timestamp_ns,"
          << "camera_timestamp_corrected_ns,"
          << "recv_rpi_ns,"
          << "delivery_latency_ms,"
          << "mjpeg_offset,"
          << "bytes_used,"
          << "flags\n"
          << std::fixed
          << std::setprecision(9);

      for (auto& s : cams) {
        ci
            << s.sequence << ','
            << s.v4l2_ns << ','
            << s.corrected_ns << ','
            << s.recv_ns << ','
            << nsToMs(
                   s.recv_ns-s.v4l2_ns)
            << ','
            << s.mjpeg_offset << ','
            << s.bytes_used << ','
            << cameraTimestampFlags(
                   s.flags)
            << '\n';
      }
    }

    {
      std::ofstream csv(OUT_CSV);

      csv
          << "event,recv_rpi_ns,"
          << "source_timestamp_ns,"
          << "mapped_rpi_ns,"
          << "transport_latency_ms,"
          << "c5,c6,c7,"
          << "xacc_m_s2,"
          << "yacc_m_s2,"
          << "zacc_m_s2,"
          << "xgyro_rad_s,"
          << "ygyro_rad_s,"
          << "zgyro_rad_s\n"
          << std::fixed
          << std::setprecision(9);

      for (auto& s : imus) {
        int64_t mn =
            map.map(s.fc_ns);

        csv
            << "IMU,"
            << s.recv_ns << ','
            << s.fc_ns << ','
            << mn << ','
            << nsToMs(
                   s.recv_ns-mn)
            << ",,,,"
            << s.xacc << ','
            << s.yacc << ','
            << s.zacc << ','
            << s.xgyro << ','
            << s.ygyro << ','
            << s.zgyro
            << '\n';
      }
    }

    {
      std::ofstream af(OUT_ATT);

      af
          << "recv_rpi_ns,"
          << "source_timestamp_ns,"
          << "mapped_rpi_ns,"
          << "roll_deg,"
          << "pitch_deg,"
          << "yaw_deg,"
          << "rel_roll_deg,"
          << "rel_pitch_deg,"
          << "rel_yaw_deg,"
          << "rollspeed,"
          << "pitchspeed,"
          << "yawspeed\n"
          << std::fixed
          << std::setprecision(9);

      for (auto& s : atts) {
        int64_t mn =
            map.map(s.src_ns);

        af
            << s.recv_ns << ','
            << s.src_ns << ','
            << mn << ','
            << s.r << ','
            << s.p << ','
            << s.y << ','
            << relDegP11(s.r,ar) << ','
            << relDegP11(s.p,ap) << ','
            << relDegP11(s.y,ay) << ','
            << s.rs << ','
            << s.ps << ','
            << s.ys
            << '\n';
      }
    }

    {
      std::ofstream ff(OUT_FC);

      ff
          << "recv_rpi_ns,"
          << "source_timestamp_ns,"
          << "mapped_rpi_ns,"
          << "transport_latency_ms,"
          << "phase,"
          << "type,"
          << "imu_id,"
          << "ax_m_s2,"
          << "ay_m_s2,"
          << "az_m_s2,"
          << "gx_rad_s,"
          << "gy_rad_s,"
          << "gz_rad_s,"
          << "roll_deg,"
          << "pitch_deg,"
          << "yaw_deg,"
          << "q1,q2,q3,q4,"
          << "vibration_x,"
          << "vibration_y,"
          << "vibration_z,"
          << "clipping_0,"
          << "clipping_1,"
          << "clipping_2,"
          << "temperature\n"
          << std::fixed
          << std::setprecision(9);

      for (auto& d : fcdiag) {
        int64_t mn =
            map.map(d.src_ns);

        ff
            << d.recv_ns << ','
            << d.src_ns << ','
            << mn << ','
            << nsToMs(d.recv_ns-mn)
            << ','
            << d.phase << ','
            << d.type << ','
            << d.imu_id << ','
            << d.ax << ','
            << d.ay << ','
            << d.az << ','
            << d.gx << ','
            << d.gy << ','
            << d.gz << ','
            << d.roll << ','
            << d.pitch << ','
            << d.yaw << ','
            << d.q1 << ','
            << d.q2 << ','
            << d.q3 << ','
            << d.q4 << ','
            << d.vibration_x << ','
            << d.vibration_y << ','
            << d.vibration_z << ','
            << d.clipping_0 << ','
            << d.clipping_1 << ','
            << d.clipping_2 << ','
            << d.temperature
            << '\n';
      }
    }

    size_t good = 0;
    for (auto& s : syncs)
      if (s.good) ++good;

    size_t n_hi = 0;
    size_t n_i1 = 0;
    size_t n_i2 = 0;
    size_t n_att = 0;
    size_t n_q = 0;
    size_t n_vib = 0;

    uint64_t clipping_total = 0;

    for (const auto& d : fcdiag) {
      if (d.type == "HIGHRES_IMU") ++n_hi;
      else if (d.type == "SCALED_IMU") ++n_i1;
      else if (d.type == "SCALED_IMU2") ++n_i2;
      else if (d.type == "ATTITUDE") ++n_att;
      else if (d.type == "ATTITUDE_QUATERNION") ++n_q;
      else if (d.type == "VIBRATION") {
        ++n_vib;
        clipping_total +=
            d.clipping_0 +
            d.clipping_1 +
            d.clipping_2;
      }
    }

    std::cout
        << "\n"
        << "================ P11 FINAL CAPTURE ================\n"
        << "camera frames: " << cams.size() << "\n"
        << "source drops: " << drops << "\n"
        << "HIGHRES_IMU: " << n_hi << "\n"
        << "SCALED_IMU: " << n_i1 << "\n"
        << "SCALED_IMU2: " << n_i2 << "\n"
        << "ATTITUDE: " << n_att << "\n"
        << "ATTITUDE_QUATERNION: " << n_q << "\n"
        << "VIBRATION: " << n_vib << "\n"
        << "VIBRATION clipping total: "
        << clipping_total << "\n"
        << "TIMESYNC good: "
        << good << "/" << syncs.size() << "\n"
        << "IMU: " << OUT_CSV << "\n"
        << "CAM: " << OUT_CAM << "\n"
        << "MJPEG: " << OUT_MJPEG << "\n"
        << "ATTITUDE: " << OUT_ATT << "\n"
        << "FC DIAG: " << OUT_FC << "\n"
        << "RESULT: "
        << (abort ? "ABORTED" : "CAPTURE_COMPLETE")
        << "\n";

    cleanupP11(
        sfd,cfd,streaming,
        buf,sys,comp,rates);

    return abort ? 2 : 0;
  }
  catch (const std::exception& e) {
    std::cerr
        << "[FATAL] "
        << e.what()
        << "\n";

    cleanupP11(
        sfd,cfd,streaming,
        buf,sys,comp,rates);

    return 1;
  }
}
