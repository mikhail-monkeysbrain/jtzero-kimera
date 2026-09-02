// JT-ZERO v48: comprehensive IMU master validation with operator-first Russian GUI.
// Designed for hand-held testing: approximate angles are enough; exact position return is NOT required.

#define JTZERO_V10_NO_MAIN
#include "live_mono_imu_integrator_v10.cpp"
#undef JTZERO_V10_NO_MAIN

#include <deque>
#include <fstream>
#include <iomanip>
#include <string>

namespace jtzero_v48 {
using namespace jtzero_v10;

constexpr const char* kCsv48 = "/home/vio/jtzero_imu_master_v48.csv";
constexpr const char* kWindow48 = "JT-ZERO: MASTER VALIDATION IMU v48";
constexpr double kG48 = 9.81;
constexpr double kCx48 = 0.014570;
constexpr double kCy48 = 0.082383;
constexpr double kBgTau48 = 2.0;
constexpr double kFcStaticRate48 = 0.010;
constexpr double kAccNormTol48 = 0.35;
constexpr double kAccStdMax48 = 0.16;
constexpr double kStaticWindowSec48 = 0.50;

struct Fc48 {
  bool valid=false, have_prev=false;
  double roll=0, pitch=0, yaw=0, accum_yaw=0, prev_yaw=0;
  double rs=0, ps=0, ys=0;
};

struct Cal48 {
  bool ready=false;
  size_t n=0;
  Eigen::Vector3d sa=Eigen::Vector3d::Zero();
  Eigen::Vector3d sg=Eigen::Vector3d::Zero();
  Eigen::Vector3d ba=Eigen::Vector3d::Zero();
  Eigen::Vector3d bg=Eigen::Vector3d::Zero();
  Eigen::Matrix3d R0=Eigen::Matrix3d::Identity();
};

struct AccWin48 { uint64_t us=0; double norm=0; };

struct Branch48 {
  Eigen::Matrix3d R=Eigen::Matrix3d::Identity();
  Eigen::Vector3d v=Eigen::Vector3d::Zero();
  Eigen::Vector3d aw=Eigen::Vector3d::Zero();
};

struct State48 {
  bool ready=false;
  uint64_t prev_us=0;
  Branch48 raw, zxy, adapt;
  double bx=0;
  std::deque<AccWin48> awin;
  double asum=0, asum2=0;
  double max_raw=0, max_zxy=0, max_adapt=0;
};

enum class Stage48:int {
  CALIB=0, STATIC_A,
  YAW_SLOW_R, YAW_RETURN_1, YAW_FAST_L, YAW_RETURN_2,
  ROLL_POS, ROLL_ZERO_1, ROLL_NEG, ROLL_ZERO_2,
  PITCH_POS, PITCH_ZERO_1, PITCH_NEG, PITCH_ZERO_2,
  RP_PP, RP_ZERO_1, RP_NP, RP_ZERO_2, RP_PN, RP_ZERO_3, RP_NN, RP_ZERO_4,
  YAW90_STATIC, YAW90_ROLL, YAW90_ZERO,
  REPEAT_ROLL_POS, REPEAT_ZERO_1, REPEAT_PITCH_NEG, REPEAT_ZERO_2,
  FINAL_STATIC, DONE
};

constexpr int kStageCount48 = static_cast<int>(Stage48::DONE);

const char* stageName48(Stage48 s) {
  static const char* n[] = {
    "CALIB","STATIC_A","YAW_SLOW_R","YAW_RETURN_1","YAW_FAST_L","YAW_RETURN_2",
    "ROLL_POS","ROLL_ZERO_1","ROLL_NEG","ROLL_ZERO_2",
    "PITCH_POS","PITCH_ZERO_1","PITCH_NEG","PITCH_ZERO_2",
    "RP_PP","RP_ZERO_1","RP_NP","RP_ZERO_2","RP_PN","RP_ZERO_3","RP_NN","RP_ZERO_4",
    "YAW90_STATIC","YAW90_ROLL","YAW90_ZERO",
    "REPEAT_ROLL_POS","REPEAT_ZERO_1","REPEAT_PITCH_NEG","REPEAT_ZERO_2",
    "FINAL_STATIC","DONE"
  };
  return n[static_cast<int>(s)];
}

const char* actionRu48(Stage48 s) {
  switch (s) {
    case Stage48::CALIB: return "НЕ ДВИГАЙТЕ АППАРАТ";
    case Stage48::STATIC_A: return "ОСТАВЬТЕ АППАРАТ В ИСХОДНОМ ПОЛОЖЕНИИ";
    case Stage48::YAW_SLOW_R: return "МЕДЛЕННО ПОВЕРНИТЕ YAW ВПРАВО ПРИМЕРНО НА 60–100°";
    case Stage48::YAW_RETURN_1: return "МЕДЛЕННО ВЕРНИТЕ YAW ПРИМЕРНО В ИСХОДНОЕ";
    case Stage48::YAW_FAST_L: return "БЫСТРЕЕ, НО ПЛАВНО ПОВЕРНИТЕ YAW ВЛЕВО НА 60–100°";
    case Stage48::YAW_RETURN_2: return "БЫСТРЕЕ, НО ПЛАВНО ВЕРНИТЕ YAW ПРИМЕРНО НАЗАД";
    case Stage48::ROLL_POS: return "НАКЛОНИТЕ ROLL В ОДНУ СТОРОНУ ПРИМЕРНО НА 20–35°";
    case Stage48::ROLL_ZERO_1: return "ВЕРНИТЕ ROLL ПРИМЕРНО В ИСХОДНОЕ";
    case Stage48::ROLL_NEG: return "НАКЛОНИТЕ ROLL В ДРУГУЮ СТОРОНУ ПРИМЕРНО НА 20–35°";
    case Stage48::ROLL_ZERO_2: return "СНОВА ВЕРНИТЕ ROLL ПРИМЕРНО В ИСХОДНОЕ";
    case Stage48::PITCH_POS: return "НАКЛОНИТЕ PITCH В ОДНУ СТОРОНУ ПРИМЕРНО НА 20–35°";
    case Stage48::PITCH_ZERO_1: return "ВЕРНИТЕ PITCH ПРИМЕРНО В ИСХОДНОЕ";
    case Stage48::PITCH_NEG: return "НАКЛОНИТЕ PITCH В ДРУГУЮ СТОРОНУ ПРИМЕРНО НА 20–35°";
    case Stage48::PITCH_ZERO_2: return "СНОВА ВЕРНИТЕ PITCH ПРИМЕРНО В ИСХОДНОЕ";
    case Stage48::RP_PP: return "СОВМЕСТИТЕ ROLL И PITCH: ОБА ПРИМЕРНО ПО 20°";
    case Stage48::RP_ZERO_1: return "ВЕРНИТЕ ОБА НАКЛОНА ПРИМЕРНО В ИСХОДНОЕ";
    case Stage48::RP_NP: return "ROLL В ДРУГУЮ СТОРОНУ, PITCH КАК РАНЬШЕ: ПРИМЕРНО ПО 20°";
    case Stage48::RP_ZERO_2: return "ВЕРНИТЕ ОБА НАКЛОНА ПРИМЕРНО В ИСХОДНОЕ";
    case Stage48::RP_PN: return "ROLL КАК В ПЕРВОЙ КОМБИНАЦИИ, PITCH В ДРУГУЮ СТОРОНУ";
    case Stage48::RP_ZERO_3: return "ВЕРНИТЕ ОБА НАКЛОНА ПРИМЕРНО В ИСХОДНОЕ";
    case Stage48::RP_NN: return "ROLL И PITCH В ПРОТИВОПОЛОЖНЫЕ СТОРОНЫ ОТ ПЕРВОЙ КОМБИНАЦИИ";
    case Stage48::RP_ZERO_4: return "ВЕРНИТЕ ОБА НАКЛОНА ПРИМЕРНО В ИСХОДНОЕ";
    case Stage48::YAW90_STATIC: return "ПОВЕРНИТЕ YAW ПРИМЕРНО НА 90°, НАКЛОН СПЕЦИАЛЬНО НЕ ДОБАВЛЯЙТЕ";
    case Stage48::YAW90_ROLL: return "СОХРАНЯЯ YAW ОКОЛО 90°, ДОБАВЬТЕ ROLL ПРИМЕРНО 20–30°";
    case Stage48::YAW90_ZERO: return "ВЕРНИТЕ YAW И НАКЛОН ПРИМЕРНО В ИСХОДНОЕ";
    case Stage48::REPEAT_ROLL_POS: return "ПОВТОРИТЕ ПЕРВЫЙ НАКЛОН ROLL ПРИМЕРНО 20–35°";
    case Stage48::REPEAT_ZERO_1: return "ВЕРНИТЕСЬ ПРИМЕРНО В ИСХОДНУЮ ОРИЕНТАЦИЮ";
    case Stage48::REPEAT_PITCH_NEG: return "ПОВТОРИТЕ ВТОРОЙ НАКЛОН PITCH ПРИМЕРНО 20–35°";
    case Stage48::REPEAT_ZERO_2: return "ВЕРНИТЕСЬ ПРИМЕРНО В ИСХОДНУЮ ОРИЕНТАЦИЮ";
    case Stage48::FINAL_STATIC: return "ФИНАЛЬНАЯ СТАТИКА: ПРОСТО НЕ ДВИГАЙТЕ АППАРАТ";
    case Stage48::DONE: return "ТЕСТ ЗАВЕРШЁН";
  }
  return "";
}

Stage48 nextStage48(Stage48 s) {
  int v = static_cast<int>(s);
  return v >= static_cast<int>(Stage48::DONE) ? Stage48::DONE : static_cast<Stage48>(v + 1);
}

Eigen::Matrix3d expSO348(const Eigen::Vector3d& q) {
  const double a = q.norm();
  return a < 1e-12 ? Eigen::Matrix3d::Identity() : Eigen::AngleAxisd(a, q / a).toRotationMatrix();
}

Eigen::Vector3d rpy48(const Eigen::Matrix3d& R) {
  return R.eulerAngles(0,1,2) * 180.0 / kPi;
}

void reticle48(cv::Mat& im) {
  const int x=im.cols/2, y=im.rows/2;
  const cv::Scalar c(0,255,255);
  cv::line(im,{x-45,y},{x-8,y},c,2,cv::LINE_AA);
  cv::line(im,{x+8,y},{x+45,y},c,2,cv::LINE_AA);
  cv::line(im,{x,y-45},{x,y-8},c,2,cv::LINE_AA);
  cv::line(im,{x,y+8},{x,y+45},c,2,cv::LINE_AA);
  cv::circle(im,{x,y},18,c,2,cv::LINE_AA);
}

void drawHud48(const cv::Mat& gray, Stage48 st, const Fc48& fc, const Cal48& cal,
               const State48& q, const Eigen::Vector3d& acc, bool guarded, double astd) {
  const cv::Scalar bg(15,15,15), white(235,235,235), green(80,220,80), red(40,40,245),
                   yellow(0,220,255), muted(155,155,155), cyan(255,220,80);
  cv::Mat canvas(980,1500,CV_8UC3,bg), video;
  if (gray.empty()) video=cv::Mat(kHeight,kWidth,CV_8UC3,cv::Scalar(0));
  else cv::cvtColor(gray,video,cv::COLOR_GRAY2BGR);
  cv::resize(video,video,{780,585},0,0,cv::INTER_NEAREST);
  reticle48(video);
  video.copyTo(canvas(cv::Rect(20,90,780,585)));

  uiText(canvas,"JT-ZERO: MASTER-ТЕСТ IMU v48",20,52,.72,white,2);
  uiText(canvas,"ТЕСТ ДЛЯ РУЧНОГО ПЕРЕМЕЩЕНИЯ — ТОЧНЫЙ СТЕНД НЕ НУЖЕН",25,720,.48,cyan,2);
  uiText(canvas,"Не пытайтесь точно вернуть положение в пространстве или точку камеры.",25,755,.43,muted,1);
  uiText(canvas,"Нужно только примерно выполнить указанную ориентацию и затем полностью остановиться.",25,788,.43,muted,1);

  cv::Mat side=canvas(cv::Rect(825,75,650,845));
  cv::rectangle(side,{0,0},{649,844},cv::Scalar(50,50,50),1);
  char b[256];
  const int idx=static_cast<int>(st);
  std::snprintf(b,sizeof(b),"ЭТАП %d / %d",std::min(idx+1,kStageCount48),kStageCount48);
  uiText(side,b,18,38,.48,muted,1);
  uiText(side,"СЕЙЧАС СДЕЛАЙТЕ:",18,78,.48,yellow,2);
  uiText(side,actionRu48(st),18,118,.42,white,2);

  uiText(side,"ПОСЛЕ ДВИЖЕНИЯ:",18,180,.45,yellow,2);
  uiText(side,"1. Полностью остановите аппарат",18,216,.40,white,1);
  uiText(side,"2. Подождите 1–2 секунды",18,248,.40,white,1);
  uiText(side,"3. Точный угол и точная точка НЕ нужны",18,280,.40,white,1);
  uiText(side,"4. Когда готовы — нажмите SPACE",18,312,.40,white,1);

  std::snprintf(b,sizeof(b),"СТАТИКА: %s",guarded?"ДА":"НЕТ");
  uiText(side,b,18,365,.55,guarded?green:red,2);
  uiText(side,guarded?"Можно нажимать SPACE":"Остановите аппарат и немного подождите",200,365,.38,guarded?green:yellow,1);

  std::snprintf(b,sizeof(b),"ROLL %.1f°   PITCH %.1f°",fc.roll,fc.pitch); uiText(side,b,18,420,.40,white,1);
  std::snprintf(b,sizeof(b),"YAW накопл. %.1f°",fc.accum_yaw); uiText(side,b,18,452,.40,white,1);
  std::snprintf(b,sizeof(b),"|acc| %.3f   std %.4f",acc.norm(),astd); uiText(side,b,18,484,.38,white,1);
  std::snprintf(b,sizeof(b),"adaptive BG_X %.7f",q.bx); uiText(side,b,18,516,.38,white,1);

  std::snprintf(b,sizeof(b),"RAW Vxy %.3f",q.raw.v.head<2>().norm()); uiText(side,b,18,565,.42,red,2);
  std::snprintf(b,sizeof(b),"ZXY Vxy %.3f",q.zxy.v.head<2>().norm()); uiText(side,b,18,598,.42,white,1);
  std::snprintf(b,sizeof(b),"ZXY+BGX Vxy %.3f",q.adapt.v.head<2>().norm()); uiText(side,b,18,631,.42,green,2);

  if (st != Stage48::DONE) {
    uiText(side,"СЛЕДУЮЩИЙ ЭТАП:",18,686,.40,muted,1);
    uiText(side,actionRu48(nextStage48(st)),18,718,.36,muted,1);
  }

  uiText(side,"SPACE — подтвердить текущий этап",18,775,.38,yellow,1);
  uiText(side,"ESC / Q — сохранить CSV и выйти",18,808,.36,muted,1);
  uiText(canvas,std::string("CSV: ")+kCsv48,25,945,.38,muted,1);
  cv::imshow(kWindow48,canvas);
}

} // namespace jtzero_v48

int main(int argc, char** argv) {
  using namespace jtzero_v10;
  using namespace jtzero_v48;

  google::InitGoogleLogging(argv[0]);
  FLAGS_visualize=false; FLAGS_use_lcd=false; FLAGS_log_output=false;
  FLAGS_extract_planes_from_the_scene=false;

  int cfd=-1, sfd=-1;
  bool streaming=false, imu_req=false, att_req=false, aborted=false;
  uint8_t sys=0, comp=0;
  std::vector<CameraBuffer> bufs;
  std::ofstream csv;
  State48 q;

  try {
    sfd=openSerial();
    mavlink_status_t mst{}; mavlink_message_t msg{};
    std::cout << "[MAV] waiting HEARTBEAT...\n";
    int64_t dl=monotonicNs()+10000000000LL;
    while(monotonicNs()<dl && !sys) {
      pollfd p{sfd,POLLIN,0};
      if(poll(&p,1,100)<=0) continue;
      uint8_t b[2048]; ssize_t n=read(sfd,b,sizeof(b)); if(n<=0) continue;
      for(ssize_t i=0;i<n;++i) {
        if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst) && msg.msgid==MAVLINK_MSG_ID_HEARTBEAT) {
          sys=msg.sysid; comp=msg.compid; break;
        }
      }
    }
    if(!sys) throw std::runtime_error("HEARTBEAT timeout");

    requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,kImuRateHz); imu_req=true;
    requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,kAttRateHz); att_req=true;

    cfd=open(kCameraDevice,O_RDWR|O_NONBLOCK); if(cfd==-1) fail("open camera");
    configureCamera(cfd); bufs=initCameraBuffers(cfd);
    v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(xioctl(cfd,VIDIOC_STREAMON,&type)==-1) fail("STREAMON");
    streaming=true; discardWarmup(cfd);

    csv.open(kCsv48,std::ios::trunc);
    if(!csv) throw std::runtime_error("cannot open CSV");
    csv << "stage_id,stage,imu_us,rx_mono_ns,dt,fields_updated,temp_c,"
           "frd_ax,frd_ay,frd_az,frd_gx,frd_gy,frd_gz,"
           "flu_ax,flu_ay,flu_az,raw_gx,raw_gy,raw_gz,"
           "zxy_gx,zxy_gy,zxy_gz,adapt_gx,adapt_gy,adapt_gz,adaptive_bgx,static_guard,acc_window_std,"
           "fc_roll,fc_pitch,fc_yaw,fc_accum_yaw,fc_rollspeed,fc_pitchspeed,fc_yawspeed,"
           "raw_rr,raw_rp,raw_ry,zxy_rr,zxy_rp,zxy_ry,adapt_rr,adapt_rp,adapt_ry,"
           "raw_awx,raw_awy,raw_awz,zxy_awx,zxy_awy,zxy_awz,adapt_awx,adapt_awy,adapt_awz,"
           "raw_vx,raw_vy,raw_vz,zxy_vx,zxy_vy,zxy_vz,adapt_vx,adapt_vy,adapt_vz\n"
        << std::fixed << std::setprecision(9);

    cv::setNumThreads(1);
    cv::namedWindow(kWindow48,cv::WINDOW_NORMAL);
    cv::resizeWindow(kWindow48,1500,980);
    cv::Mat gray(kHeight,kWidth,CV_8UC1,cv::Scalar(0));
    Fc48 fc; Cal48 cal; Stage48 st=Stage48::CALIB;
    Eigen::Vector3d last_acc=Eigen::Vector3d::Zero();
    double last_std=999; bool last_guard=false;
    int64_t next_hud=monotonicNs();

    std::cout << "JT-ZERO v48: ручной master-тест. Точные углы/позиции не требуются.\n";

    while(true) {
      pollfd pf[2]={{cfd,POLLIN,0},{sfd,POLLIN,0}};
      int rc=poll(pf,2,2);
      if(rc<0) { if(errno==EINTR) continue; fail("poll"); }

      if(pf[1].revents&POLLIN) {
        uint8_t b[8192];
        for(;;) {
          ssize_t n=read(sfd,b,sizeof(b));
          if(n==-1 && (errno==EAGAIN||errno==EWOULDBLOCK)) break;
          if(n<=0) break;
          for(ssize_t i=0;i<n;++i) {
            if(!mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst)) continue;
            const int64_t rx=monotonicNs();

            if(msg.msgid==MAVLINK_MSG_ID_ATTITUDE) {
              mavlink_attitude_t a{}; mavlink_msg_attitude_decode(&msg,&a);
              const double yd=a.yaw*180.0/kPi;
              if(fc.have_prev) fc.accum_yaw += wrapDeg(yd-fc.prev_yaw);
              fc.prev_yaw=yd; fc.have_prev=true; fc.valid=true;
              fc.roll=a.roll*180.0/kPi; fc.pitch=a.pitch*180.0/kPi; fc.yaw=yd;
              fc.rs=a.rollspeed; fc.ps=a.pitchspeed; fc.ys=a.yawspeed;
            } else if(msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU) {
              mavlink_highres_imu_t h{}; mavlink_msg_highres_imu_decode(&msg,&h);
              const Eigen::Vector3d af(h.xacc,h.yacc,h.zacc), gf(h.xgyro,h.ygyro,h.zgyro);
              const Eigen::Vector3d acc(h.xacc,-h.yacc,-h.zacc), gyr(h.xgyro,-h.ygyro,-h.zgyro);
              last_acc=acc;

              double dt=0;
              if(q.prev_us && h.time_usec>q.prev_us) dt=(h.time_usec-q.prev_us)*1e-6;
              q.prev_us=h.time_usec;
              if(dt<0 || dt>0.03) dt=0;

              const double an=acc.norm();
              q.awin.push_back({h.time_usec,an}); q.asum+=an; q.asum2+=an*an;
              while(!q.awin.empty() && (h.time_usec-q.awin.front().us)>uint64_t(kStaticWindowSec48*1e6)) {
                q.asum-=q.awin.front().norm; q.asum2-=q.awin.front().norm*q.awin.front().norm; q.awin.pop_front();
              }
              const double mean=q.awin.empty()?0:q.asum/q.awin.size();
              const double var=q.awin.empty()?999:std::max(0.0,q.asum2/q.awin.size()-mean*mean);
              last_std=std::sqrt(var);
              const double fcr=std::sqrt(fc.rs*fc.rs+fc.ps*fc.ps+fc.ys*fc.ys);
              last_guard=fc.valid && q.awin.size()>50 && fcr<kFcStaticRate48 && last_std<kAccStdMax48 && std::abs(mean-kG48)<kAccNormTol48;

              const bool initial_still=gyr.norm()<0.08 && std::abs(an-kG48)<kAccNormTol48;
              if(st==Stage48::CALIB && !cal.ready && initial_still) { cal.sa+=acc; cal.sg+=gyr; ++cal.n; }

              Eigen::Vector3d wr=gyr-cal.bg, wz=wr;
              wz.x() += kCx48*wr.z(); wz.y() += kCy48*wr.z();
              if(q.ready && dt>0 && last_guard) {
                const double alpha=1.0-std::exp(-dt/kBgTau48);
                q.bx += alpha*(wz.x()-q.bx);
              }
              Eigen::Vector3d wa=wz; wa.x()-=q.bx;

              if(q.ready && dt>0) {
                auto step=[&](Branch48& z,const Eigen::Vector3d& w) {
                  z.R=z.R*expSO348(w*dt);
                  z.aw=z.R*(acc-cal.ba)+Eigen::Vector3d(0,0,kG48);
                  z.v+=z.aw*dt;
                };
                step(q.raw,wr); step(q.zxy,wz); step(q.adapt,wa);
                q.max_raw=std::max(q.max_raw,q.raw.v.head<2>().norm());
                q.max_zxy=std::max(q.max_zxy,q.zxy.v.head<2>().norm());
                q.max_adapt=std::max(q.max_adapt,q.adapt.v.head<2>().norm());
              }

              const auto rr=rpy48(q.raw.R), rz=rpy48(q.zxy.R), ra=rpy48(q.adapt.R);
              csv << static_cast<int>(st) << ',' << stageName48(st) << ',' << h.time_usec << ',' << rx << ',' << dt << ','
                  << h.fields_updated << ',' << h.temperature << ','
                  << af.x()<<','<<af.y()<<','<<af.z()<<','<<gf.x()<<','<<gf.y()<<','<<gf.z()<<','
                  << acc.x()<<','<<acc.y()<<','<<acc.z()<<','
                  << wr.x()<<','<<wr.y()<<','<<wr.z()<<','<<wz.x()<<','<<wz.y()<<','<<wz.z()<<','
                  << wa.x()<<','<<wa.y()<<','<<wa.z()<<','<<q.bx<<','<<(last_guard?1:0)<<','<<last_std<<','
                  << fc.roll<<','<<fc.pitch<<','<<fc.yaw<<','<<fc.accum_yaw<<','<<fc.rs<<','<<fc.ps<<','<<fc.ys<<','
                  << rr.x()<<','<<rr.y()<<','<<rr.z()<<','<<rz.x()<<','<<rz.y()<<','<<rz.z()<<','<<ra.x()<<','<<ra.y()<<','<<ra.z()<<','
                  << q.raw.aw.x()<<','<<q.raw.aw.y()<<','<<q.raw.aw.z()<<','<<q.zxy.aw.x()<<','<<q.zxy.aw.y()<<','<<q.zxy.aw.z()<<','
                  << q.adapt.aw.x()<<','<<q.adapt.aw.y()<<','<<q.adapt.aw.z()<<','
                  << q.raw.v.x()<<','<<q.raw.v.y()<<','<<q.raw.v.z()<<','<<q.zxy.v.x()<<','<<q.zxy.v.y()<<','<<q.zxy.v.z()<<','
                  << q.adapt.v.x()<<','<<q.adapt.v.y()<<','<<q.adapt.v.z()<<'\n';
            }
          }
        }
      }

      if(pf[0].revents&POLLIN) {
        for(;;) {
          v4l2_buffer z{}; z.type=V4L2_BUF_TYPE_VIDEO_CAPTURE; z.memory=V4L2_MEMORY_MMAP;
          if(xioctl(cfd,VIDIOC_DQBUF,&z)==-1) { if(errno==EAGAIN) break; fail("DQBUF"); }
          std::vector<unsigned char> j(z.bytesused); std::memcpy(j.data(),bufs[z.index].start,z.bytesused);
          cv::Mat g=cv::imdecode(j,cv::IMREAD_GRAYSCALE); if(!g.empty()) gray=g;
          if(xioctl(cfd,VIDIOC_QBUF,&z)==-1) fail("QBUF");
        }
      }

      const int64_t now=monotonicNs();
      if(now>=next_hud) { drawHud48(gray,st,fc,cal,q,last_acc,last_guard,last_std); next_hud=now+kHudPeriodNs; }

      const int key=cv::waitKey(1)&0xff;
      if(key==27 || key=='q' || key=='Q') { aborted=st!=Stage48::DONE; break; }
      if(key==' ') {
        if(st==Stage48::CALIB) {
          if(cal.n<100) { std::cout << "[WAIT] calibration samples=" << cal.n << "\n"; continue; }
          const Eigen::Vector3d ma=cal.sa/static_cast<double>(cal.n), mg=cal.sg/static_cast<double>(cal.n);
          cal.R0=fcRnedFlu(fc.roll,fc.pitch,fc.yaw);
          cal.ba=ma-cal.R0.transpose()*Eigen::Vector3d(0,0,-kG48);
          cal.bg=mg; cal.ready=true; q.ready=true;
          q.raw.R=q.zxy.R=q.adapt.R=cal.R0;
          q.raw.v.setZero(); q.zxy.v.setZero(); q.adapt.v.setZero(); q.bx=0;
          st=nextStage48(st);
          std::cout << "[CAL] BA=" << cal.ba.transpose() << " BG=" << cal.bg.transpose() << "\n";
          std::cout << "[NEXT] " << stageName48(st) << "\n";
        } else if(st!=Stage48::DONE) {
          std::cout << "[STEP] " << stageName48(st)
                    << " RAW=" << q.raw.v.head<2>().norm()
                    << " ZXY=" << q.zxy.v.head<2>().norm()
                    << " ADAPT=" << q.adapt.v.head<2>().norm()
                    << " BGX=" << q.bx << "\n";
          st=nextStage48(st);
          if(st==Stage48::DONE) break;
        } else break;
      }
    }

    csv.flush(); csv.close();
    if(imu_req) requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);
    if(att_req) requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);
    if(streaming) { v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE; ioctl(cfd,VIDIOC_STREAMOFF,&t); }
    for(auto& b:bufs) if(b.start && b.start!=MAP_FAILED) munmap(b.start,b.length);
    if(cfd>=0) close(cfd); if(sfd>=0) close(sfd); cv::destroyAllWindows();

    std::cout << "\nCSV: " << kCsv48
              << "\nMAX Vxy RAW=" << q.max_raw
              << " ZXY=" << q.max_zxy
              << " ZXY+BGX=" << q.max_adapt << "\n"
              << (aborted?"ABORTED\n":"DONE\n");
    return aborted?2:0;
  } catch(const std::exception& e) {
    std::cerr << "[FATAL] " << e.what() << "\n";
    return 1;
  }
}
