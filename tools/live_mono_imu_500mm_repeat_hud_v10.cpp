// JT-ZERO 500 mm x6 v10.
// Keeps v8 stabilized endpoint capture and adds a startup static gate.
// Camera/IMU are not fed to Kimera until a stable 3 s IMU window passes.

#define JTZERO_V7_EMBEDDED
#include "live_mono_imu_300mm_repeat_hud_v7.cpp"
#undef JTZERO_V7_EMBEDDED
#undef main

namespace {

constexpr const char* kV8CsvPath = "/home/vio/jtzero_live_500mm_repeat_v10.csv";
constexpr const char* kV8WindowName = "JT-ZERO 500 мм x6 v10 — startup-gated";
constexpr double kTruth500M = 0.500;
constexpr int kStableStatesNeeded = 4;
constexpr double kStableMaxSpeedMps = 0.020;
constexpr double kStableMaxSpanM = 0.008;
constexpr double kV8MaxLegErrorM = 0.030;
constexpr double kV8MaxStdM = 0.020;
constexpr double kV8MaxAbsDzM = 0.030;
constexpr double kV8MinPairAngleDeg = 160.0;

constexpr double kStartupStaticSec = 3.0;
constexpr size_t kStartupMinSamples = 450;
constexpr double kStartupMaxMeanGyroRadS = 0.010;
constexpr double kStartupMaxGyroStdRadS = 0.005;
constexpr double kStartupMinAccelNorm = 9.60;
constexpr double kStartupMaxAccelNorm = 10.00;
constexpr double kStartupMaxAccelNormStd = 0.080;

struct StartupStaticGate {
  bool passed=false;
  uint64_t first_us=0,last_us=0;
  size_t n=0;
  Eigen::Vector3d sum_g=Eigen::Vector3d::Zero();
  Eigen::Vector3d sum_g2=Eigen::Vector3d::Zero();
  Eigen::Vector3d sum_a=Eigen::Vector3d::Zero();
  double sum_an=0.0,sum_an2=0.0;

  void reset(){
    first_us=last_us=0;n=0;
    sum_g.setZero();sum_g2.setZero();sum_a.setZero();
    sum_an=sum_an2=0.0;
  }

  bool add(uint64_t us,const Eigen::Vector3d&a,const Eigen::Vector3d&g){
    if(passed)return true;
    if(first_us==0)first_us=us;
    last_us=us;++n;
    sum_g+=g;sum_g2+=g.cwiseProduct(g);sum_a+=a;
    const double an=a.norm();sum_an+=an;sum_an2+=an*an;

    if(n<kStartupMinSamples)return false;
    const double elapsed=(last_us>first_us)?(last_us-first_us)*1e-6:0.0;
    if(elapsed<kStartupStaticSec)return false;

    const double dn=static_cast<double>(n);
    const Eigen::Vector3d mg=sum_g/dn;
    const Eigen::Vector3d vg=(sum_g2/dn-mg.cwiseProduct(mg)).cwiseMax(0.0);
    const Eigen::Vector3d sg=vg.cwiseSqrt();
    const Eigen::Vector3d ma=sum_a/dn;
    const double man=sum_an/dn;
    const double san=std::sqrt(std::max(0.0,sum_an2/dn-man*man));

    const bool ok=
      mg.norm()<=kStartupMaxMeanGyroRadS &&
      sg.maxCoeff()<=kStartupMaxGyroStdRadS &&
      man>=kStartupMinAccelNorm && man<=kStartupMaxAccelNorm &&
      san<=kStartupMaxAccelNormStd;

    if(ok){
      passed=true;
      std::cout<<std::fixed<<std::setprecision(6)
               <<"[STARTUP] PASS samples="<<n<<" sec="<<elapsed
               <<" mean_acc=["<<ma.transpose()<<"] |acc|="<<man
               <<" std|acc|="<<san
               <<" mean_gyro=["<<mg.transpose()<<"] gyro_std=["<<sg.transpose()<<"]\n";
      return true;
    }

    std::cout<<std::fixed<<std::setprecision(6)
             <<"[STARTUP] RETRY samples="<<n<<" sec="<<elapsed
             <<" |mean_gyro|="<<mg.norm()
             <<" max_gyro_std="<<sg.maxCoeff()
             <<" |acc|="<<man<<" std|acc|="<<san<<"\n";
    reset();
    return false;
  }
};

enum class CapturePhase { WAIT_START, SETTLE_START, MOVING, SETTLE_END };

struct StableCapture {
  std::vector<VioState> samples;
  int64_t last_kf = -1;

  void reset() {
    samples.clear();
    last_kf = -1;
  }

  static double speed(const VioState& s) {
    return std::sqrt(s.vx*s.vx + s.vy*s.vy + s.vz*s.vz);
  }

  bool update(const VioState& s, Mark* out) {
    if (s.keyframe == last_kf) return false;
    last_kf = s.keyframe;

    if (speed(s) > kStableMaxSpeedMps) {
      samples.clear();
      return false;
    }

    if (!samples.empty()) {
      const VioState& first = samples.front();
      const double dx = s.px - first.px;
      const double dy = s.py - first.py;
      const double dz = s.pz - first.pz;
      if (std::sqrt(dx*dx + dy*dy + dz*dz) > kStableMaxSpanM) {
        samples.clear();
      }
    }

    samples.push_back(s);
    if (static_cast<int>(samples.size()) < kStableStatesNeeded) return false;

    VioState avg = samples.back();
    avg.px = avg.py = avg.pz = 0.0;
    avg.vx = avg.vy = avg.vz = 0.0;
    for (const auto& q : samples) {
      avg.px += q.px; avg.py += q.py; avg.pz += q.pz;
      avg.vx += q.vx; avg.vy += q.vy; avg.vz += q.vz;
    }
    const double n = static_cast<double>(samples.size());
    avg.px /= n; avg.py /= n; avg.pz /= n;
    avg.vx /= n; avg.vy /= n; avg.vz /= n;
    out->s = avg;
    out->valid = true;
    reset();
    return true;
  }

  int count() const { return static_cast<int>(samples.size()); }
};

struct V8Leg {
  Leg measured;
  Mark start_press;
  Mark end_press;
  double start_settle_shift_m = 0.0;
  double end_settle_shift_m = 0.0;
};

double markDistance(const Mark& a, const Mark& b) {
  if (!a.valid || !b.valid) return 0.0;
  const double dx = b.s.px-a.s.px;
  const double dy = b.s.py-a.s.py;
  const double dz = b.s.pz-a.s.pz;
  return std::sqrt(dx*dx+dy*dy+dz*dz);
}

Leg makeLeg500(int idx,const Mark&a,const Mark&b){
  Leg r;
  r.index=idx;
  r.direction=(idx%2==0)?"A->B":"B->A";
  r.a=a; r.b=b;
  r.dx=b.s.px-a.s.px;
  r.dy=b.s.py-a.s.py;
  r.dz=b.s.pz-a.s.pz;
  r.horizontal=std::sqrt(r.dx*r.dx+r.dy*r.dy);
  r.d3=std::sqrt(r.horizontal*r.horizontal+r.dz*r.dz);
  r.error=r.horizontal-kTruth500M;
  r.scale=r.horizontal/kTruth500M;
  return r;
}

bool v8MeasurementPass(const std::vector<V8Leg>& rows, double* sd_out=nullptr) {
  if (rows.size() != kLegCount) return false;
  double sum=0.0, sum2=0.0;
  for (const auto& row : rows) {
    const Leg& r = row.measured;
    const double error500 = r.horizontal - kTruth500M;
    if (std::abs(error500) > kV8MaxLegErrorM || std::abs(r.dz) > kV8MaxAbsDzM) return false;
    sum += r.horizontal;
    sum2 += r.horizontal*r.horizontal;
  }
  const double mean = sum / rows.size();
  const double sd = std::sqrt(std::max(0.0, sum2/rows.size()-mean*mean));
  if (sd_out) *sd_out = sd;
  if (sd > kV8MaxStdM) return false;
  for (size_t i=0; i+1<rows.size(); i+=2) {
    if (pairAngleDeg(rows[i].measured, rows[i+1].measured) < kV8MinPairAngleDeg) return false;
  }
  return true;
}

void saveCsvV8(const std::vector<V8Leg>& rows) {
  std::ofstream f(kV8CsvPath, std::ios::trunc);
  if (!f) return;
  f << "leg,direction,start_press_kf,start_settled_kf,end_press_kf,end_settled_kf,"
       "start_press_px,start_press_py,start_press_pz,start_px,start_py,start_pz,"
       "end_press_px,end_press_py,end_press_pz,end_px,end_py,end_pz,"
       "start_settle_shift_m,end_settle_shift_m,dx_m,dy_m,dz_m,horizontal_m,"
       "distance3d_m,error_horizontal_m,scale_horizontal,pair_angle_deg\n";
  f << std::fixed << std::setprecision(9);
  for (size_t i=0;i<rows.size();++i) {
    const auto& row=rows[i];
    const auto& r=row.measured;
    const double angle=(i%2)==1?pairAngleDeg(rows[i-1].measured,r):0.0;
    f << r.index+1 << ',' << r.direction << ','
      << row.start_press.s.keyframe << ',' << r.a.s.keyframe << ','
      << row.end_press.s.keyframe << ',' << r.b.s.keyframe << ','
      << row.start_press.s.px << ',' << row.start_press.s.py << ',' << row.start_press.s.pz << ','
      << r.a.s.px << ',' << r.a.s.py << ',' << r.a.s.pz << ','
      << row.end_press.s.px << ',' << row.end_press.s.py << ',' << row.end_press.s.pz << ','
      << r.b.s.px << ',' << r.b.s.py << ',' << r.b.s.pz << ','
      << row.start_settle_shift_m << ',' << row.end_settle_shift_m << ','
      << r.dx << ',' << r.dy << ',' << r.dz << ',' << r.horizontal << ','
      << r.d3 << ',' << r.error << ',' << r.scale << ',' << angle << '\n';
  }
}

void saveBackendTraceV8(const std::vector<VioState>& states,
                        const std::vector<V8Leg>& rows) {
  const char* path = "/home/vio/jtzero_live_500mm_repeat_v10_backend.csv";
  std::ofstream f(path, std::ios::trunc);
  if (!f) return;

  f << "leg,phase,keyframe,timestamp_ns,"
       "px_m,py_m,pz_m,vx_m_s,vy_m_s,vz_m_s,"
       "speed_m_s,roll_deg,pitch_deg,yaw_deg\n";

  f << std::fixed << std::setprecision(9);

  for (const auto& s : states) {
    int leg = 0;
    const char* phase = "OUTSIDE";

    for (size_t i = 0; i < rows.size(); ++i) {
      const auto& r = rows[i];

      const int64_t start_press = r.start_press.s.keyframe;
      const int64_t start_settled = r.measured.a.s.keyframe;
      const int64_t end_press = r.end_press.s.keyframe;
      const int64_t end_settled = r.measured.b.s.keyframe;

      if (s.keyframe >= start_press && s.keyframe <= end_settled) {
        leg = static_cast<int>(i) + 1;

        if (s.keyframe < start_settled)
          phase = "SETTLE_START";
        else if (s.keyframe <= end_press)
          phase = "MOVE";
        else
          phase = "SETTLE_END";

        break;
      }
    }

    const double speed =
        std::sqrt(s.vx*s.vx + s.vy*s.vy + s.vz*s.vz);

    f << leg << ','
      << phase << ','
      << s.keyframe << ','
      << s.timestamp_ns << ','
      << s.px << ',' << s.py << ',' << s.pz << ','
      << s.vx << ',' << s.vy << ',' << s.vz << ','
      << speed << ','
      << s.roll_deg << ','
      << s.pitch_deg << ','
      << s.yaw_deg << '\n';
  }

  std::cout << "BACKEND TRACE CSV: " << path << '\n';
}

void printSummaryV8(const std::vector<V8Leg>& rows,size_t raw,size_t rejected,
                    size_t selected,size_t decoded,size_t imu_rx,size_t imu_fed,
                    size_t att_rx,size_t sync_count,const ClockMapping& mapping,
                    size_t states) {
  std::cout << "\n================ 500 MM x6 V10 RESULT ================\n"
            << "fusion: v7 static-only gravity feedback\n"
            << "endpoint capture: " << kStableStatesNeeded << " stable backend states, |V| <= "
            << kStableMaxSpeedMps*1000.0 << " mm/s, span <= " << kStableMaxSpanM*1000.0 << " mm\n"
            << "raw camera frames: " << raw << "\n"
            << "rejected raw pairs: " << rejected << "\n"
            << "selected frames: " << selected << "\n"
            << "decoded frames: " << decoded << "\n"
            << "IMU received: " << imu_rx << "\n"
            << "IMU fed: " << imu_fed << "\n"
            << "ATTITUDE received: " << att_rx << "\n"
            << "TIMESYNC samples: " << sync_count << "\n"
            << "mapping valid: " << (mapping.valid?"yes":"no") << "\n"
            << "backend states: " << states << "\n";

  double sum=0.0,sum2=0.0,ab=0.0,ba=0.0;
  int nab=0,nba=0;
  for (const auto& row : rows) {
    const auto& r=row.measured;
    std::cout << "LEG " << r.index+1 << ' ' << r.direction << ": "
              << std::fixed << std::setprecision(2) << r.horizontal*1000.0
              << " mm  error " << r.error*1000.0
              << " mm  dz " << r.dz*1000.0
              << " mm  settle START/END " << row.start_settle_shift_m*1000.0
              << "/" << row.end_settle_shift_m*1000.0 << " mm\n";
    sum+=r.horizontal; sum2+=r.horizontal*r.horizontal;
    if(r.index%2==0){ab+=r.horizontal;++nab;}else{ba+=r.horizontal;++nba;}
  }
  for(size_t i=0;i+1<rows.size();i+=2){
    std::cout << "PAIR " << (i/2)+1 << " reversal angle: "
              << std::fixed << std::setprecision(2)
              << pairAngleDeg(rows[i].measured,rows[i+1].measured) << " deg\n";
  }
  if(!rows.empty()){
    const double mean=sum/rows.size();
    const double sd=std::sqrt(std::max(0.0,sum2/rows.size()-mean*mean));
    std::cout << "MEAN HORIZONTAL: " << mean*1000.0 << " mm\n"
              << "STD: " << sd*1000.0 << " mm\n"
              << "A->B mean: " << (nab?ab/nab*1000.0:0.0) << " mm\n"
              << "B->A mean: " << (nba?ba/nba*1000.0:0.0) << " mm\n"
              << "MEAN SCALE measured/true: " << std::setprecision(6) << mean/kTruth500M << "\n";
  }
  std::cout << "CSV: " << kV8CsvPath << "\n";
}

void drawHudV8(const cv::Mat& gray,const HudPipeline& pipeline,const FcAttitude& fc,
               int leg,CapturePhase phase,const StableCapture& stable,
               const std::vector<V8Leg>& rows,const std::string& notice) {
  cv::Mat bgr,video;
  cv::cvtColor(gray,bgr,cv::COLOR_GRAY2BGR);
  cv::resize(bgr,video,{900,675},0,0,cv::INTER_NEAREST);
  cv::Mat panel(900,380,CV_8UC3,cv::Scalar(24,24,24));
  cv::Mat canvas(900,1280,CV_8UC3,cv::Scalar(8,8,8));
  video.copyTo(canvas(cv::Rect(0,105,900,675)));

  ru(canvas,"JT-ZERO — 300 мм × 6 — v9",{28,52},27,{245,245,245},cv::QT_FONT_BOLD);
  ru(canvas,"Startup gate + fusion v7 + stabilized endpoints",{28,84},17,{190,190,190});
  char b[256];
  std::snprintf(b,sizeof(b),"ПРОХОД %d / %d",std::min(leg+1,kLegCount),kLegCount);
  ru(panel,b,{18,55},25,{245,245,245},cv::QT_FONT_BOLD);
  ru(panel,(leg%2==0)?"A  →  B":"B  →  A",{18,105},30,{0,230,255},cv::QT_FONT_BOLD);

  VioState s; const bool have_state=pipeline.latest(&s);
  ru(panel,have_state?"КООРДИНАТЫ: ЕСТЬ":"КООРДИНАТЫ: НЕТ",{18,160},18,
     have_state?cv::Scalar(90,220,90):cv::Scalar(40,40,245),cv::QT_FONT_BOLD);
  if(have_state){
    std::snprintf(b,sizeof(b),"P [%.3f  %.3f  %.3f] м",s.px,s.py,s.pz);ru(panel,b,{18,198},15,{220,220,220});
    std::snprintf(b,sizeof(b),"|V| %.1f мм/с   KF %lld",StableCapture::speed(s)*1000.0,(long long)s.keyframe);ru(panel,b,{18,230},14,{190,190,190});
  }
  if(fc.valid){std::snprintf(b,sizeof(b),"FC: R %.2f  P %.2f  Y %.2f°",fc.roll_deg,fc.pitch_deg,fc.yaw_deg);ru(panel,b,{18,268},14,{190,190,190});}

  std::string status,action;
  cv::Scalar status_color(220,220,220);
  if(phase==CapturePhase::WAIT_START){status="ГОТОВО К СТАРТУ";action="ПРОБЕЛ / ENTER: СТАРТ";status_color=cv::Scalar(90,220,90);}
  else if(phase==CapturePhase::SETTLE_START){status="СТАБИЛИЗАЦИЯ СТАРТА";action="НЕ ДВИГАТЬ";status_color=cv::Scalar(0,210,255);}
  else if(phase==CapturePhase::MOVING){status="ДВИЖЕНИЕ — ACC GRAVITY OFF";action="ПРОБЕЛ / ENTER: КОНЕЦ";status_color=cv::Scalar(0,210,255);}
  else {status="СТАБИЛИЗАЦИЯ КОНЦА";action="НЕ ДВИГАТЬ";status_color=cv::Scalar(0,210,255);}
  ru(panel,status,{18,320},15,status_color,cv::QT_FONT_BOLD);
  if(phase==CapturePhase::SETTLE_START||phase==CapturePhase::SETTLE_END){
    std::snprintf(b,sizeof(b),"Стабильных состояний: %d / %d",stable.count(),kStableStatesNeeded);
    ru(panel,b,{18,355},14,{220,220,220});
  }else{
    ru(panel,phase==CapturePhase::WAIT_START?"Установите камеру точно на метку":"Переместите камеру ровно на 500 мм",{18,355},14,{220,220,220});
  }
  cv::rectangle(panel,{18,395,344,92},cv::Scalar(55,145,55),-1);
  cv::rectangle(panel,{18,395,344,92},cv::Scalar(220,220,220),2);
  ru(panel,action,{35,452},18,{255,255,255},cv::QT_FONT_BOLD);

  if(!rows.empty()){
    const auto&r=rows.back().measured;
    std::snprintf(b,sizeof(b),"Последний: %.1f мм",r.horizontal*1000.0);ru(panel,b,{18,555},20,{245,245,245},cv::QT_FONT_BOLD);
    std::snprintf(b,sizeof(b),"Ошибка: %+.1f мм",r.error*1000.0);ru(panel,b,{18,595},16,{220,220,220});
    std::snprintf(b,sizeof(b),"ΔZ: %+.1f мм",r.dz*1000.0);ru(panel,b,{18,630},15,{190,190,190});
  }
  ru(panel,"PASS: каждый ±30 мм, σ≤20 мм, ΔZ≤30 мм",{18,700},12,{185,185,185});
  ru(panel,"Разворот каждой пары ≥160°",{18,725},12,{185,185,185});
  ru(panel,"Q / ESC — выход",{18,755},14,{215,215,215});
  if(!notice.empty()){cv::rectangle(panel,{10,790,360,70},cv::Scalar(35,35,125),-1);ru(panel,notice,{20,835},13,{255,255,255},cv::QT_FONT_BOLD);}
  panel.copyTo(canvas(cv::Rect(900,0,380,900)));
  cv::imshow(kV8WindowName,canvas);
}

} // namespace

int main(int argc,char**argv){
  google::InitGoogleLogging(argv[0]);
  FLAGS_visualize=false;FLAGS_viz_type=2;FLAGS_use_lcd=false;FLAGS_log_output=false;FLAGS_extract_planes_from_the_scene=false;
  const std::string params=argc>1?argv[1]:"params/JTZeroMonoFLU";
  const std::string camera_device=argc>2?argv[2]:kCameraDevice;
  std::cout<<"[CAM] device="<<camera_device<<"\n";

  int camera_fd=-1,serial_fd=-1;bool streaming=false,imu_req=false,att_req=false,pipeline_started=false,aborted=false;
  uint8_t sys=0,comp=0;std::vector<CameraBuffer>buffers;std::shared_ptr<HudPipeline>pipeline;std::thread pipeline_thread;

  try{
    VIO::VioParams vp(params);if(vp.camera_params_.empty())throw std::runtime_error("No camera params loaded");
    pipeline=std::make_shared<HudPipeline>(vp);pipeline->installBackendCallback();pipeline_thread=std::thread([pipeline](){pipeline->spin();});pipeline_started=true;
    serial_fd=openSerial();mavlink_status_t mst{};mavlink_message_t msg{};std::cout<<"[MAV] waiting for HEARTBEAT...\n";
    int64_t dl=monotonicNs()+10000000000LL;
    while(monotonicNs()<dl&&!sys){pollfd p{serial_fd,POLLIN,0};if(poll(&p,1,100)<=0)continue;uint8_t b[2048];ssize_t n=read(serial_fd,b,sizeof(b));if(n<=0)continue;for(ssize_t i=0;i<n;++i)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst)&&msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=msg.sysid;comp=msg.compid;break;}}
    if(!sys)throw std::runtime_error("HEARTBEAT timeout");
    requestRate(serial_fd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,kImuRateHz);imu_req=true;
    requestRate(serial_fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,kFcAttitudeRateHz);att_req=true;

    camera_fd=open(camera_device.c_str(),O_RDWR|O_NONBLOCK);if(camera_fd==-1)fail("open camera");configureCamera(camera_fd);buffers=initCameraBuffers(camera_fd);
    v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;if(xioctl(camera_fd,VIDIOC_STREAMON,&type)==-1)fail("STREAMON");streaming=true;discardWarmup(camera_fd);
    cv::setNumThreads(1);cv::namedWindow(kV8WindowName,cv::WINDOW_NORMAL);cv::resizeWindow(kV8WindowName,1280,900);

    std::vector<TimeSyncSample>sync;sync.reserve(500);ClockMapping mapping;int64_t pending=0,next_sync=monotonicNs();
    size_t raw=0,rejected=0,selected=0,decoded=0,imu_rx=0,imu_fed=0,imu_skip=0,att_rx=0;
    uint32_t prev_seq=0;int64_t prev_ts=0,last_sel=0;bool have_prev=false;VIO::FrameId fid=0;FcAttitude fc;
    cv::Mat gray(kHeight,kWidth,CV_8UC1,cv::Scalar(0));jtzero::ImuCorrection imu_correction;StartupStaticGate startup_gate;bool vio_feed_enabled=false;
    int leg=0;CapturePhase phase=CapturePhase::WAIT_START;StableCapture stable;Mark start_press,start_mark,end_press;
    std::vector<V8Leg>rows;std::string notice;int64_t notice_until=0,next_hud=0;

    while(leg<kLegCount){
      const int64_t now=monotonicNs();
      if(now>=next_sync&&pending==0){pending=now;sendTimesync(serial_fd,pending,sys,comp);next_sync=now+kTimesyncPeriodNs;}
      pollfd pf[2]={{camera_fd,POLLIN,0},{serial_fd,POLLIN,0}};int rc=poll(pf,2,2);if(rc<0){if(errno==EINTR)continue;fail("poll");}

      if(pf[1].revents&POLLIN){uint8_t b[8192];for(;;){ssize_t n=read(serial_fd,b,sizeof(b));if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;for(ssize_t i=0;i<n;++i){
        if(!mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst))continue;const int64_t recv=monotonicNs();
        if(msg.msgid==MAVLINK_MSG_ID_TIMESYNC){mavlink_timesync_t ts{};mavlink_msg_timesync_decode(&msg,&ts);if(ts.tc1!=0&&pending&&ts.ts1==pending){TimeSyncSample s;s.t0_rpi_ns=pending;s.t1_rpi_ns=recv;s.fc_ns=ts.tc1;s.rtt_ns=recv-pending;s.rpi_mid_ns=pending+s.rtt_ns/2;s.good=s.rtt_ns>0&&nsToMs(s.rtt_ns)<=kMaxTimesyncRttMs;sync.push_back(s);pending=0;mapping=estimateClockMapping(sync);}}
        else if(msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){++imu_rx;mavlink_highres_imu_t h{};mavlink_msg_highres_imu_decode(&msg,&h);if(!mapping.valid||recv-mapping.last_update_ns>kMappingStaleNs){++imu_skip;continue;}const Eigen::Vector3d acc=jtzero::ImuCorrection::accelFrdToFlu(h.xacc,h.yacc,h.zacc);const Eigen::Vector3d gyro=jtzero::ImuCorrection::gyroFrdToFlu(h.xgyro,h.ygyro,h.zgyro);if(!vio_feed_enabled){if(startup_gate.add(h.time_usec,acc,gyro)){imu_correction.reset();vio_feed_enabled=true;std::cout<<"[STARTUP] VIO FEED ENABLED — fresh IMU/camera only from this point\n";}else{++imu_skip;continue;}}const bool moving=(phase==CapturePhase::MOVING);const Eigen::Vector3d w=imu_correction.correctGyro(h.time_usec,acc,gyro,!moving);VIO::ImuAccGyr data;data<<acc.x(),acc.y(),acc.z(),w.x(),w.y(),w.z();pipeline->fillSingleImuQueue(VIO::ImuMeasurement(mapping.map((int64_t)h.time_usec*1000LL),data));++imu_fed;}
        else if(msg.msgid==MAVLINK_MSG_ID_ATTITUDE){mavlink_attitude_t a{};mavlink_msg_attitude_decode(&msg,&a);++att_rx;fc.valid=true;fc.wall_ns=recv;fc.roll_deg=a.roll*180.0/kPi;fc.pitch_deg=a.pitch*180.0/kPi;fc.yaw_deg=a.yaw*180.0/kPi;}
      }}}
      if(pending&&monotonicNs()-pending>20000000LL)pending=0;

      if(pf[0].revents&POLLIN){for(;;){v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;if(xioctl(camera_fd,VIDIOC_DQBUF,&b)==-1){if(errno==EAGAIN)break;fail("DQBUF");}++raw;const int64_t corrected=jtzero::timesync::correctCameraTimestampNs(timevalToNs(b.timestamp));bool ok=true;if(have_prev){const int64_t dt=corrected-prev_ts;ok=b.sequence==prev_seq+1U&&dt>0&&dt<=20000000LL;if(!ok)++rejected;}prev_seq=b.sequence;prev_ts=corrected;have_prev=true;const bool due=last_sel==0||corrected-last_sel>=30000000LL;if(ok&&due&&mapping.valid&&vio_feed_enabled){std::vector<unsigned char>jpg(b.bytesused);std::memcpy(jpg.data(),buffers[b.index].start,b.bytesused);cv::Mat g=cv::imdecode(jpg,cv::IMREAD_GRAYSCALE);if(!g.empty()){++decoded;gray=g;pipeline->fillLeftFrameQueue(std::make_unique<VIO::Frame>(fid++,corrected,vp.camera_params_.at(0),g.clone()));last_sel=corrected;++selected;}}if(xioctl(camera_fd,VIDIOC_QBUF,&b)==-1)fail("QBUF");}}

      if(phase==CapturePhase::SETTLE_START||phase==CapturePhase::SETTLE_END){
        VioState s;if(pipeline->latest(&s)){Mark settled;if(stable.update(s,&settled)){
          if(phase==CapturePhase::SETTLE_START){start_mark=settled;phase=CapturePhase::MOVING;notice="СТАРТ СТАБИЛИЗИРОВАН — ДВИГАЙТЕ 500 ММ";notice_until=monotonicNs()+1500000000LL;std::cout<<"LEG "<<leg+1<<" START press KF="<<start_press.s.keyframe<<" settled KF="<<start_mark.s.keyframe<<" shift="<<markDistance(start_press,start_mark)*1000.0<<" mm [gravity feedback OFF]\n";}
          else {V8Leg row;row.start_press=start_press;row.end_press=end_press;row.measured=makeLeg500(leg,start_mark,settled);row.start_settle_shift_m=markDistance(start_press,start_mark);row.end_settle_shift_m=markDistance(end_press,settled);rows.push_back(row);std::cout<<"LEG "<<leg+1<<' '<<row.measured.direction<<" horizontal="<<std::fixed<<std::setprecision(2)<<row.measured.horizontal*1000.0<<" mm error="<<row.measured.error*1000.0<<" mm dz="<<row.measured.dz*1000.0<<" mm end_settle_shift="<<row.end_settle_shift_m*1000.0<<" mm\n";++leg;phase=CapturePhase::WAIT_START;notice="КОНЕЦ СТАБИЛИЗИРОВАН — СЛЕДУЮЩИЙ ПРОХОД";notice_until=monotonicNs()+1500000000LL;}
        }}
      }

      if(now>=next_hud){if(monotonicNs()>notice_until)notice.clear();drawHudV8(gray,*pipeline,fc,leg,phase,stable,rows,notice);next_hud=now+kHudPeriodNs;}
      const int key=cv::waitKey(1)&0xff;if(key==27||key=='q'||key=='Q'){aborted=true;break;}
      if(key==32||key==13||key==10){VioState s;if(!pipeline->latest(&s)){notice="КООРДИНАТ ПОКА НЕТ";notice_until=monotonicNs()+1500000000LL;}
        else if(phase==CapturePhase::WAIT_START){start_press.s=s;start_press.valid=true;stable.reset();phase=CapturePhase::SETTLE_START;notice="НЕ ДВИГАТЬ — ФИКСИРУЮ СТАРТ";notice_until=monotonicNs()+1500000000LL;std::cout<<"LEG "<<leg+1<<" START key accepted KF="<<s.keyframe<<" [settling]\n";}
        else if(phase==CapturePhase::MOVING){end_press.s=s;end_press.valid=true;stable.reset();phase=CapturePhase::SETTLE_END;notice="НЕ ДВИГАТЬ — ФИКСИРУЮ КОНЕЦ";notice_until=monotonicNs()+1500000000LL;std::cout<<"LEG "<<leg+1<<" END key accepted KF="<<s.keyframe<<" [gravity feedback re-armed, settling]\n";}
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));pipeline->shutdown();if(pipeline_thread.joinable())pipeline_thread.join();pipeline_started=false;cv::destroyAllWindows();const auto states=pipeline->states();
    saveCsvV8(rows);
    saveBackendTraceV8(states,rows);
    if(imu_req){requestRate(serial_fd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);imu_req=false;}if(att_req){requestRate(serial_fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);att_req=false;}if(streaming){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(camera_fd,VIDIOC_STREAMOFF,&t);streaming=false;}for(auto&bf:buffers)if(bf.start&&bf.start!=MAP_FAILED)munmap(bf.start,bf.length);if(camera_fd!=-1)close(camera_fd);if(serial_fd!=-1)close(serial_fd);
    printSummaryV8(rows,raw,rejected,selected,decoded,imu_rx,imu_fed,att_rx,sync.size(),mapping,states.size());double sd=0.0;const bool metric_pass=v8MeasurementPass(rows,&sd);const bool pipeline_pass=!aborted&&rows.size()==kLegCount&&mapping.valid&&states.size()>0;std::cout<<"PIPELINE RESULT: "<<(pipeline_pass?"PASS":"FAIL")<<"\nMEASUREMENT RESULT: "<<(metric_pass?"PASS":"FAIL")<<"\n";const bool pass=pipeline_pass&&metric_pass;std::cout<<"RESULT: "<<(pass?"PASS":"FAIL")<<"\n";return pass?0:1;
  }catch(const std::exception&e){std::cerr<<"[FATAL] "<<e.what()<<"\n";if(pipeline)pipeline->shutdown();if(pipeline_started&&pipeline_thread.joinable())pipeline_thread.join();try{cv::destroyAllWindows();}catch(...){}if(serial_fd!=-1&&imu_req&&sys)try{requestRate(serial_fd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);}catch(...){}if(serial_fd!=-1&&att_req&&sys)try{requestRate(serial_fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);}catch(...){}if(streaming&&camera_fd!=-1){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(camera_fd,VIDIOC_STREAMOFF,&t);}for(auto&bf:buffers)if(bf.start&&bf.start!=MAP_FAILED)munmap(bf.start,bf.length);if(camera_fd!=-1)close(camera_fd);if(serial_fd!=-1)close(serial_fd);return 1;}
}
