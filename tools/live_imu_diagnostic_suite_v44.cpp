// JT-ZERO v44: combined IMU diagnostic suite.
// One physical run, one master CSV, multiple offline analyses.
// Russian event-driven GUI with live camera video and center reticle.

#define JTZERO_V10_NO_MAIN
#include "live_mono_imu_integrator_v10.cpp"
#undef JTZERO_V10_NO_MAIN

#include <array>
#include <deque>
#include <fstream>
#include <iomanip>
#include <string>

namespace jtzero_v44 {
using namespace jtzero_v10;

constexpr const char* kCsv44 = "/home/vio/jtzero_imu_master_v44.csv";
constexpr const char* kWindow44 = "JT-ZERO: IMU DIAGNOSTIC SUITE v44";
constexpr double kGyroCx44 = 0.014570;
constexpr double kGyroCy44 = 0.082383;
constexpr double kG44 = 9.81;
constexpr double kStillGyro44 = 0.08;
constexpr double kStillAccTol44 = 0.35;

struct Fc44 {
  bool valid=false;
  uint64_t us=0;
  double roll=0,pitch=0,yaw=0;
  double rollspeed=0,pitchspeed=0,yawspeed=0;
  double accum_yaw=0,prev_yaw=0;
  bool have_prev_yaw=false;
};

struct Cal44 {
  bool ready=false;
  size_t n=0;
  Eigen::Vector3d sum_acc=Eigen::Vector3d::Zero();
  Eigen::Vector3d sum_gyro=Eigen::Vector3d::Zero();
  Eigen::Vector3d ba=Eigen::Vector3d::Zero();
  Eigen::Vector3d bg=Eigen::Vector3d::Zero();
  Eigen::Matrix3d R0=Eigen::Matrix3d::Identity();
};

struct Int44 {
  bool ready=false;
  uint64_t last_us=0;
  double prev_wz=0;
  Eigen::Matrix3d Rraw=Eigen::Matrix3d::Identity();
  Eigen::Matrix3d Rfix=Eigen::Matrix3d::Identity();
  Eigen::Vector3d vraw=Eigen::Vector3d::Zero();
  Eigen::Vector3d vfix=Eigen::Vector3d::Zero();
  Eigen::Vector3d araw=Eigen::Vector3d::Zero();
  Eigen::Vector3d afix=Eigen::Vector3d::Zero();
  double dwz_dt=0;
  double max_vraw=0,max_vfix=0;
};

enum class Stage44 : int {
  CALIB_STATIC=0,
  YAW_SLOW_R,
  YAW_RETURN_SR,
  YAW_FAST_R,
  YAW_RETURN_FR,
  YAW_SLOW_L,
  YAW_RETURN_SL,
  ROLL_POS,
  ROLL_RETURN,
  PITCH_POS,
  PITCH_RETURN,
  FINAL_STATIC,
  DONE
};

const char* stageName44(Stage44 s){
  switch(s){
    case Stage44::CALIB_STATIC:return "CALIB_STATIC";
    case Stage44::YAW_SLOW_R:return "YAW_SLOW_R";
    case Stage44::YAW_RETURN_SR:return "YAW_RETURN_SR";
    case Stage44::YAW_FAST_R:return "YAW_FAST_R";
    case Stage44::YAW_RETURN_FR:return "YAW_RETURN_FR";
    case Stage44::YAW_SLOW_L:return "YAW_SLOW_L";
    case Stage44::YAW_RETURN_SL:return "YAW_RETURN_SL";
    case Stage44::ROLL_POS:return "ROLL_POS";
    case Stage44::ROLL_RETURN:return "ROLL_RETURN";
    case Stage44::PITCH_POS:return "PITCH_POS";
    case Stage44::PITCH_RETURN:return "PITCH_RETURN";
    case Stage44::FINAL_STATIC:return "FINAL_STATIC";
    case Stage44::DONE:return "DONE";
  }
  return "UNKNOWN";
}

const char* stageRu44(Stage44 s){
  switch(s){
    case Stage44::CALIB_STATIC:return "НЕ ДВИГАТЬ. КАЛИБРОВКА В НУЛЕ";
    case Stage44::YAW_SLOW_R:return "МЕДЛЕННО ПОВЕРНИТЕ YAW ВПРАВО НА 60–90°";
    case Stage44::YAW_RETURN_SR:return "МЕДЛЕННО ВЕРНИТЕ YAW В НОЛЬ";
    case Stage44::YAW_FAST_R:return "БЫСТРО, НО ПЛАВНО ПОВЕРНИТЕ YAW ВПРАВО НА 60–90°";
    case Stage44::YAW_RETURN_FR:return "БЫСТРО, НО ПЛАВНО ВЕРНИТЕ YAW В НОЛЬ";
    case Stage44::YAW_SLOW_L:return "МЕДЛЕННО ПОВЕРНИТЕ YAW ВЛЕВО НА 60–90°";
    case Stage44::YAW_RETURN_SL:return "МЕДЛЕННО ВЕРНИТЕ YAW В НОЛЬ";
    case Stage44::ROLL_POS:return "НАКЛОНИТЕ АППАРАТ ПО ROLL НА 15–30°";
    case Stage44::ROLL_RETURN:return "ВЕРНИТЕ ROLL В НОЛЬ";
    case Stage44::PITCH_POS:return "НАКЛОНИТЕ АППАРАТ ПО PITCH НА 15–30°";
    case Stage44::PITCH_RETURN:return "ВЕРНИТЕ PITCH В НОЛЬ";
    case Stage44::FINAL_STATIC:return "ФИНАЛЬНАЯ СТАТИКА. НЕ ДВИГАТЬ";
    case Stage44::DONE:return "ТЕСТ ЗАВЕРШЁН";
  }
  return "";
}

Stage44 nextStage44(Stage44 s){
  int v=static_cast<int>(s);
  if(v>=static_cast<int>(Stage44::DONE))return Stage44::DONE;
  return static_cast<Stage44>(v+1);
}

Eigen::Matrix3d expSO344(const Eigen::Vector3d& wdt){
  const double a=wdt.norm();
  if(a<1e-12)return Eigen::Matrix3d::Identity();
  return Eigen::AngleAxisd(a,wdt/a).toRotationMatrix();
}

Eigen::Vector3d rpy44(const Eigen::Matrix3d& R){
  return R.eulerAngles(0,1,2)*180.0/kPi;
}

void drawReticle44(cv::Mat& img){
  const int cx=img.cols/2,cy=img.rows/2;
  const cv::Scalar col(0,255,255);
  cv::line(img,{cx-45,cy},{cx-10,cy},col,2,cv::LINE_AA);
  cv::line(img,{cx+10,cy},{cx+45,cy},col,2,cv::LINE_AA);
  cv::line(img,{cx,cy-45},{cx,cy-10},col,2,cv::LINE_AA);
  cv::line(img,{cx,cy+10},{cx,cy+45},col,2,cv::LINE_AA);
  cv::circle(img,{cx,cy},20,col,2,cv::LINE_AA);
  cv::circle(img,{cx,cy},60,col,1,cv::LINE_AA);
  cv::circle(img,{cx,cy},3,col,-1,cv::LINE_AA);
}

void drawHud44(const cv::Mat& gray,Stage44 stage,const Fc44& fc,const Cal44& cal,const Int44& in,
               const Eigen::Vector3d& last_acc,const Eigen::Vector3d& last_gyr){
  const cv::Scalar bg(15,15,15),white(235,235,235),green(80,220,80),red(40,40,245),
                   yellow(0,220,255),muted(155,155,155);
  cv::Mat canvas(940,1440,CV_8UC3,bg),video;
  if(gray.empty())video=cv::Mat(kHeight,kWidth,CV_8UC3,cv::Scalar(0));
  else cv::cvtColor(gray,video,cv::COLOR_GRAY2BGR);
  cv::resize(video,video,{820,615},0,0,cv::INTER_NEAREST);
  drawReticle44(video);
  video.copyTo(canvas(cv::Rect(20,80,820,615)));

  uiText(canvas,"JT-ZERO: КОМБИНИРОВАННАЯ ДИАГНОСТИКА IMU v44",20,48,.68,white,2);
  uiText(canvas,"ДЕРЖИТЕ ВЫБРАННУЮ ТОЧКУ В ЦЕНТРЕ ПРИЦЕЛА",95,735,.52,yellow,2);

  cv::Mat side=canvas(cv::Rect(860,70,560,790));
  cv::rectangle(side,{0,0},{559,789},cv::Scalar(45,45,45),1);
  uiText(side,"ТЕКУЩИЙ ЭТАП",18,42,.48,muted,1);
  uiText(side,stageRu44(stage),18,82,.44,white,2);

  char b[256];
  std::snprintf(b,sizeof(b),"YAW: %.1f°",fc.accum_yaw);uiText(side,b,18,145,.48,white,1);
  std::snprintf(b,sizeof(b),"ROLL: %.1f°",fc.roll);uiText(side,b,18,180,.48,white,1);
  std::snprintf(b,sizeof(b),"PITCH: %.1f°",fc.pitch);uiText(side,b,18,215,.48,white,1);
  std::snprintf(b,sizeof(b),"FC yawspeed: %.3f rad/s",fc.yawspeed);uiText(side,b,18,250,.44,white,1);

  const double gn=last_gyr.norm(),an=last_acc.norm();
  std::snprintf(b,sizeof(b),"|gyro|: %.4f rad/s",gn);uiText(side,b,18,310,.45,gn<kStillGyro44?green:white,1);
  std::snprintf(b,sizeof(b),"|acc|: %.4f m/s²",an);uiText(side,b,18,345,.45,std::abs(an-kG44)<kStillAccTol44?green:white,1);
  std::snprintf(b,sizeof(b),"Калибровочных samples: %zu",cal.n);uiText(side,b,18,380,.43,cal.ready?green:yellow,1);

  std::snprintf(b,sizeof(b),"RAW Vxy: %.3f m/s",in.vraw.head<2>().norm());uiText(side,b,18,445,.50,red,2);
  std::snprintf(b,sizeof(b),"FIXED Vxy: %.3f m/s",in.vfix.head<2>().norm());uiText(side,b,18,485,.50,green,2);
  std::snprintf(b,sizeof(b),"RAW axy: %.3f m/s²",in.araw.head<2>().norm());uiText(side,b,18,535,.44,white,1);
  std::snprintf(b,sizeof(b),"FIXED axy: %.3f m/s²",in.afix.head<2>().norm());uiText(side,b,18,570,.44,white,1);
  std::snprintf(b,sizeof(b),"dWz/dt: %.3f rad/s²",in.dwz_dt);uiText(side,b,18,605,.44,white,1);

  uiText(side,"SPACE — подтвердить достигнутую позицию",18,675,.39,yellow,1);
  uiText(side,"После движения остановитесь, затем SPACE",18,708,.36,muted,1);
  uiText(side,"ESC / Q — прервать и сохранить CSV",18,742,.36,muted,1);

  uiText(canvas,std::string("CSV: ")+kCsv44,25,910,.40,muted,1);
  cv::imshow(kWindow44,canvas);
}

} // namespace jtzero_v44

int main(int argc,char** argv){
  using namespace jtzero_v10;
  using namespace jtzero_v44;

  google::InitGoogleLogging(argv[0]);
  FLAGS_visualize=false;FLAGS_viz_type=2;FLAGS_use_lcd=false;FLAGS_log_output=false;
  FLAGS_extract_planes_from_the_scene=false;

  int cfd=-1,sfd=-1;bool streaming=false,aborted=false,imu_req=false,att_req=false;
  uint8_t sys=0,comp=0;std::vector<CameraBuffer> bufs;
  std::ofstream csv;

  try{
    sfd=openSerial();
    mavlink_status_t mst{};mavlink_message_t msg{};
    std::cout<<"[MAV] waiting for HEARTBEAT...\n";
    int64_t dl=monotonicNs()+10000000000LL;
    while(monotonicNs()<dl&&!sys){
      pollfd p{sfd,POLLIN,0};if(poll(&p,1,100)<=0)continue;
      uint8_t b[2048];ssize_t n=read(sfd,b,sizeof(b));if(n<=0)continue;
      for(ssize_t i=0;i<n;++i)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst)&&msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=msg.sysid;comp=msg.compid;break;}
    }
    if(!sys)throw std::runtime_error("HEARTBEAT timeout");

    requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,kImuRateHz);imu_req=true;
    requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,kAttRateHz);att_req=true;

    cfd=open(kCameraDevice,O_RDWR|O_NONBLOCK);if(cfd==-1)fail("open camera");
    configureCamera(cfd);bufs=initCameraBuffers(cfd);v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(xioctl(cfd,VIDIOC_STREAMON,&type)==-1)fail("STREAMON");streaming=true;discardWarmup(cfd);

    csv.open(kCsv44,std::ios::trunc);if(!csv)throw std::runtime_error("cannot open CSV");
    csv<<"stage_id,stage,imu_us,rx_mono_ns,dt,"
          "frd_ax,frd_ay,frd_az,frd_gx,frd_gy,frd_gz,"
          "flu_ax,flu_ay,flu_az,raw_gx,raw_gy,raw_gz,fixed_gx,fixed_gy,fixed_gz,"
          "fc_roll,fc_pitch,fc_yaw,fc_accum_yaw,fc_rollspeed,fc_pitchspeed,fc_yawspeed,"
          "acc_norm,gyro_norm,wz2,wz_abs_wz,dwz_dt,"
          "raw_R_roll,raw_R_pitch,raw_R_yaw,fixed_R_roll,fixed_R_pitch,fixed_R_yaw,"
          "raw_aw_x,raw_aw_y,raw_aw_z,fixed_aw_x,fixed_aw_y,fixed_aw_z,"
          "raw_vx,raw_vy,raw_vz,fixed_vx,fixed_vy,fixed_vz\n";
    csv<<std::fixed<<std::setprecision(9);

    cv::setNumThreads(1);cv::namedWindow(kWindow44,cv::WINDOW_NORMAL);cv::resizeWindow(kWindow44,1440,940);
    cv::Mat last_gray(kHeight,kWidth,CV_8UC1,cv::Scalar(0));
    Fc44 fc;Cal44 cal;Int44 integ;Stage44 stage=Stage44::CALIB_STATIC;
    Eigen::Vector3d last_acc=Eigen::Vector3d::Zero(),last_gyr=Eigen::Vector3d::Zero();
    uint64_t prev_imu_us=0;int64_t next_hud=monotonicNs();

    std::cout<<"\nJT-ZERO IMU DIAGNOSTIC SUITE v44\n"
             <<"Один запуск собирает master CSV для gyro/accel/lever-arm/3x3 анализа.\n"
             <<"Видео содержит центральный прицел. После каждого движения остановитесь и нажмите SPACE.\n"
             <<"FIXED gyro: wx += "<<kGyroCx44<<"*wz, wy += "<<kGyroCy44<<"*wz\n";

    while(true){
      pollfd pf[2]={{cfd,POLLIN,0},{sfd,POLLIN,0}};int rc=poll(pf,2,2);
      if(rc<0){if(errno==EINTR)continue;fail("poll");}

      if(pf[1].revents&POLLIN){
        uint8_t b[8192];for(;;){
          ssize_t n=read(sfd,b,sizeof(b));if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;
          for(ssize_t i=0;i<n;++i){
            if(!mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst))continue;
            const int64_t rx=monotonicNs();
            if(msg.msgid==MAVLINK_MSG_ID_ATTITUDE){
              mavlink_attitude_t a{};mavlink_msg_attitude_decode(&msg,&a);
              const double yaw=a.yaw*180.0/kPi;
              if(fc.have_prev_yaw)fc.accum_yaw+=wrapDeg(yaw-fc.prev_yaw);
              fc.prev_yaw=yaw;fc.have_prev_yaw=true;fc.valid=true;fc.us=(uint64_t)a.time_boot_ms*1000ULL;
              fc.roll=a.roll*180.0/kPi;fc.pitch=a.pitch*180.0/kPi;fc.yaw=yaw;
              fc.rollspeed=a.rollspeed;fc.pitchspeed=a.pitchspeed;fc.yawspeed=a.yawspeed;
            }else if(msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){
              mavlink_highres_imu_t h{};mavlink_msg_highres_imu_decode(&msg,&h);
              const Eigen::Vector3d acc_frd(h.xacc,h.yacc,h.zacc),gyr_frd(h.xgyro,h.ygyro,h.zgyro);
              const Eigen::Vector3d acc(h.xacc,-h.yacc,-h.zacc),gyr(h.xgyro,-h.ygyro,-h.zgyro);
              last_acc=acc;last_gyr=gyr;
              double dt=0;if(prev_imu_us&&h.time_usec>prev_imu_us)dt=(h.time_usec-prev_imu_us)*1e-6;prev_imu_us=h.time_usec;
              if(dt<0||dt>0.03)dt=0;

              const bool still=gyr.norm()<kStillGyro44&&std::abs(acc.norm()-kG44)<kStillAccTol44;
              if(stage==Stage44::CALIB_STATIC&&!cal.ready&&still){cal.sum_acc+=acc;cal.sum_gyro+=gyr;++cal.n;}

              Eigen::Vector3d wraw=gyr-cal.bg,wfix=wraw;
              wfix.x()+=kGyroCx44*wraw.z();wfix.y()+=kGyroCy44*wraw.z();
              if(integ.ready&&dt>0){
                integ.dwz_dt=(wraw.z()-integ.prev_wz)/dt;integ.prev_wz=wraw.z();
                integ.Rraw=integ.Rraw*expSO344(wraw*dt);
                integ.Rfix=integ.Rfix*expSO344(wfix*dt);
                const Eigen::Vector3d ac=acc-cal.ba,gN(0,0,kG44);
                integ.araw=integ.Rraw*ac+gN;integ.afix=integ.Rfix*ac+gN;
                integ.vraw+=integ.araw*dt;integ.vfix+=integ.afix*dt;
                integ.max_vraw=std::max(integ.max_vraw,integ.vraw.head<2>().norm());
                integ.max_vfix=std::max(integ.max_vfix,integ.vfix.head<2>().norm());
              }

              const Eigen::Vector3d er=rpy44(integ.Rraw),ef=rpy44(integ.Rfix);
              csv<<(int)stage<<','<<stageName44(stage)<<','<<h.time_usec<<','<<rx<<','<<dt<<','
                 <<acc_frd.x()<<','<<acc_frd.y()<<','<<acc_frd.z()<<','<<gyr_frd.x()<<','<<gyr_frd.y()<<','<<gyr_frd.z()<<','
                 <<acc.x()<<','<<acc.y()<<','<<acc.z()<<','<<wraw.x()<<','<<wraw.y()<<','<<wraw.z()<<','
                 <<wfix.x()<<','<<wfix.y()<<','<<wfix.z()<<','
                 <<fc.roll<<','<<fc.pitch<<','<<fc.yaw<<','<<fc.accum_yaw<<','<<fc.rollspeed<<','<<fc.pitchspeed<<','<<fc.yawspeed<<','
                 <<acc.norm()<<','<<gyr.norm()<<','<<wraw.z()*wraw.z()<<','<<wraw.z()*std::abs(wraw.z())<<','<<integ.dwz_dt<<','
                 <<er.x()<<','<<er.y()<<','<<er.z()<<','<<ef.x()<<','<<ef.y()<<','<<ef.z()<<','
                 <<integ.araw.x()<<','<<integ.araw.y()<<','<<integ.araw.z()<<','<<integ.afix.x()<<','<<integ.afix.y()<<','<<integ.afix.z()<<','
                 <<integ.vraw.x()<<','<<integ.vraw.y()<<','<<integ.vraw.z()<<','<<integ.vfix.x()<<','<<integ.vfix.y()<<','<<integ.vfix.z()<<'\n';
            }
          }
        }
      }

      if(pf[0].revents&POLLIN){
        for(;;){v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;
          if(xioctl(cfd,VIDIOC_DQBUF,&b)==-1){if(errno==EAGAIN)break;fail("DQBUF");}
          std::vector<unsigned char> jpeg(b.bytesused);std::memcpy(jpeg.data(),bufs[b.index].start,b.bytesused);
          cv::Mat gray=cv::imdecode(jpeg,cv::IMREAD_GRAYSCALE);if(!gray.empty())last_gray=gray;
          if(xioctl(cfd,VIDIOC_QBUF,&b)==-1)fail("QBUF");
        }
      }

      const int64_t now=monotonicNs();if(now>=next_hud){drawHud44(last_gray,stage,fc,cal,integ,last_acc,last_gyr);next_hud=now+kHudPeriodNs;}
      int key=cv::waitKey(1)&0xff;
      if(key==27||key=='q'||key=='Q'){aborted=stage!=Stage44::DONE;break;}
      if(key==' '){
        if(stage==Stage44::CALIB_STATIC){
          if(cal.n<100){std::cout<<"[WAIT] Недостаточно стабильных IMU samples для калибровки: "<<cal.n<<"\n";continue;}
          const Eigen::Vector3d mean_acc=cal.sum_acc/(double)cal.n,mean_gyr=cal.sum_gyro/(double)cal.n;
          cal.R0=fcRnedFlu(fc.roll,fc.pitch,fc.yaw);
          const Eigen::Vector3d expected=cal.R0.transpose()*Eigen::Vector3d(0,0,-kG44);
          cal.ba=mean_acc-expected;cal.bg=mean_gyr;cal.ready=true;
          integ.ready=true;integ.Rraw=cal.R0;integ.Rfix=cal.R0;integ.vraw.setZero();integ.vfix.setZero();integ.last_us=0;integ.prev_wz=0;
          stage=nextStage44(stage);
          std::cout<<"[CAL] BA="<<cal.ba.transpose()<<" BG="<<cal.bg.transpose()<<"\n";
          std::cout<<"[STEP] "<<stageName44(stage)<<"\n";
        }else if(stage!=Stage44::DONE){
          stage=nextStage44(stage);
          std::cout<<"[STEP] "<<stageName44(stage)<<" RAW Vxy="<<integ.vraw.head<2>().norm()
                   <<" FIXED Vxy="<<integ.vfix.head<2>().norm()<<"\n";
          if(stage==Stage44::DONE)break;
        }else break;
      }
    }

    csv.flush();csv.close();
    if(imu_req)requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);
    if(att_req)requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);
    if(streaming){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);}
    for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);
    if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);cv::destroyAllWindows();

    std::cout<<"\n============================================================\n"
             <<"JT-ZERO IMU DIAGNOSTIC SUITE v44 RESULT\n"
             <<"============================================================\n"
             <<"aborted: "<<(aborted?"yes":"no")<<"\n"
             <<"calibration samples: "<<cal.n<<"\n"
             <<"BA: "<<cal.ba.transpose()<<"\n"
             <<"BG: "<<cal.bg.transpose()<<"\n"
             <<"max RAW Vxy: "<<integ.max_vraw<<" m/s\n"
             <<"max FIXED Vxy: "<<integ.max_vfix<<" m/s\n"
             <<"final RAW Vxy: "<<integ.vraw.head<2>().norm()<<" m/s\n"
             <<"final FIXED Vxy: "<<integ.vfix.head<2>().norm()<<" m/s\n"
             <<"CSV: "<<kCsv44<<"\nOpen CSV:\n  code "<<kCsv44<<"\n";
    return aborted?2:0;
  }catch(const std::exception& e){
    if(csv.is_open()){csv.flush();csv.close();}
    if(streaming&&cfd>=0){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);}
    for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);
    if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);cv::destroyAllWindows();
    std::cerr<<"[FATAL] "<<e.what()<<"\n";return 1;
  }
}
