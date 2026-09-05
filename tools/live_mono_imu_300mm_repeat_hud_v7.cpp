// JT-ZERO 300 mm x6 v7.
// Built on the verified v4/v6 runtime path.
// Fusion change: accelerometer gravity feedback is permitted only while the
// operator is NOT inside a measured leg, and only after the shared correction
// class confirms a static interval. During A<->B motion the gyro path is
// FRD->FLU + validated ZXY only.

#define main jtzero_hud_v2_unused_main
#define kExpectedDistanceM jtzero_v2_kExpectedDistanceM
#define kCsvPath jtzero_v2_kCsvPath
#define kWindowName jtzero_v2_kWindowName
#define writeCsv jtzero_v2_writeCsv
#define printMeasurement jtzero_v2_printMeasurement
#define renderHud jtzero_v2_renderHud
#include "live_mono_imu_500mm_hud_v2.cpp"
#undef renderHud
#undef printMeasurement
#undef writeCsv
#undef kWindowName
#undef kCsvPath
#undef kExpectedDistanceM
#undef main
#ifdef JTZERO_V7_EMBEDDED
#define main jtzero_v7_unused_main
#endif

#include "jtzero_imu_correction.h"

namespace {
constexpr double kTruthM = 0.300;
#ifndef JTZERO_LEG_COUNT
constexpr int kLegCount = 6;
#else
constexpr int kLegCount = JTZERO_LEG_COUNT;
#endif
constexpr int kFcAttitudeRateHz = 50;
constexpr double kMaxLegErrorM = 0.060;
constexpr double kMaxStdM = 0.040;
constexpr double kMaxAbsDzM = 0.060;
constexpr double kMaxPairDot = -0.70;  // >134 deg reversal required.
constexpr const char* kCsvPath = "/home/vio/jtzero_live_300mm_repeat_v7.csv";
constexpr const char* kWindowName = "JT-ZERO 300 мм x6 v7 — static-only fusion";

struct FcAttitude {
  bool valid=false;
  int64_t wall_ns=0;
  double roll_deg=0,pitch_deg=0,yaw_deg=0;
};
struct Mark { VioState s{}; bool valid=false; };
struct Leg {
  int index=0;
  std::string direction;
  Mark a,b;
  double dx=0,dy=0,dz=0,horizontal=0,d3=0,error=0,scale=0;
};

void ru(cv::Mat&img,const std::string&s,cv::Point p,int size,cv::Scalar c,
        int weight=cv::QT_FONT_NORMAL){
  cv::addText(img,s,p,"DejaVu Sans",size,c,weight,cv::QT_STYLE_NORMAL,0);
}

Leg makeLeg(int idx,const Mark&a,const Mark&b){
  Leg r;
  r.index=idx;
  r.direction=(idx%2==0)?"A->B":"B->A";
  r.a=a;
  r.b=b;
  r.dx=b.s.px-a.s.px;
  r.dy=b.s.py-a.s.py;
  r.dz=b.s.pz-a.s.pz;
  r.horizontal=std::sqrt(r.dx*r.dx+r.dy*r.dy);
  r.d3=std::sqrt(r.horizontal*r.horizontal+r.dz*r.dz);
  r.error=r.horizontal-kTruthM;
  r.scale=r.horizontal/kTruthM;
  return r;
}

double pairDot(const Leg&a,const Leg&b){
  if(a.horizontal<1e-9||b.horizontal<1e-9)return 1.0;
  return (a.dx*b.dx+a.dy*b.dy)/(a.horizontal*b.horizontal);
}

double pairAngleDeg(const Leg&a,const Leg&b){
  const double d=std::max(-1.0,std::min(1.0,pairDot(a,b)));
  return std::acos(d)*180.0/kPi;
}

bool measurementPass(const std::vector<Leg>&legs,double*std_out=nullptr){
  if(legs.size()!=kLegCount)return false;
  double sum=0.0,sum2=0.0;
  bool legs_ok=true;
  for(const auto&r:legs){
    sum+=r.horizontal;
    sum2+=r.horizontal*r.horizontal;
    if(std::abs(r.error)>kMaxLegErrorM||std::abs(r.dz)>kMaxAbsDzM)legs_ok=false;
  }
  const double mean=sum/legs.size();
  const double sd=std::sqrt(std::max(0.0,sum2/legs.size()-mean*mean));
  if(std_out)*std_out=sd;
  bool direction_ok=true;
  for(size_t i=0;i+1<legs.size();i+=2){
    if(pairDot(legs[i],legs[i+1])>kMaxPairDot)direction_ok=false;
  }
  return legs_ok&&sd<=kMaxStdM&&direction_ok;
}

void saveCsv(const std::vector<Leg>&legs){
  std::ofstream f(kCsvPath,std::ios::trunc);
  if(!f)return;
  f<<"leg,direction,start_kf,end_kf,dx_m,dy_m,dz_m,horizontal_m,distance3d_m,error_horizontal_m,scale_horizontal,pair_angle_deg\n";
  f<<std::fixed<<std::setprecision(9);
  for(size_t i=0;i<legs.size();++i){
    const auto&r=legs[i];
    double angle=0.0;
    if((i%2)==1)angle=pairAngleDeg(legs[i-1],legs[i]);
    f<<r.index+1<<','<<r.direction<<','<<r.a.s.keyframe<<','<<r.b.s.keyframe<<','
     <<r.dx<<','<<r.dy<<','<<r.dz<<','<<r.horizontal<<','<<r.d3<<','<<r.error<<','
     <<r.scale<<','<<angle<<'\n';
  }
}

void printSummary(const std::vector<Leg>&legs,size_t raw,size_t rejected,
                  size_t selected,size_t decoded,size_t imu_rx,size_t imu_fed,
                  size_t att_rx,size_t sync_count,const ClockMapping&mapping,
                  size_t states){
  std::cout<<"\n================ 300 MM x6 V7 RESULT ================\n";
  std::cout<<"fusion: static-only gravity feedback\n"
           <<"raw camera frames: "<<raw<<"\n"
           <<"rejected raw pairs: "<<rejected<<"\n"
           <<"selected frames: "<<selected<<"\n"
           <<"decoded frames: "<<decoded<<"\n"
           <<"IMU received: "<<imu_rx<<"\n"
           <<"IMU fed: "<<imu_fed<<"\n"
           <<"ATTITUDE received: "<<att_rx<<"\n"
           <<"TIMESYNC samples: "<<sync_count<<"\n"
           <<"mapping valid: "<<(mapping.valid?"yes":"no")<<"\n"
           <<"backend states: "<<states<<"\n";
  if(legs.empty()){std::cout<<"NO COMPLETED LEGS\n";return;}

  double sum=0,sum2=0,ab=0,ba=0;
  int nab=0,nba=0;
  for(const auto&r:legs){
    std::cout<<"LEG "<<r.index+1<<' '<<r.direction<<": "
             <<std::fixed<<std::setprecision(2)<<r.horizontal*1000.0
             <<" mm  error "<<r.error*1000.0
             <<" mm  dz "<<r.dz*1000.0
             <<" mm  scale "<<std::setprecision(5)<<r.scale<<"\n";
    sum+=r.horizontal;
    sum2+=r.horizontal*r.horizontal;
    if(r.index%2==0){ab+=r.horizontal;++nab;}else{ba+=r.horizontal;++nba;}
  }
  for(size_t i=0;i+1<legs.size();i+=2){
    std::cout<<"PAIR "<<(i/2)+1<<" reversal angle: "
             <<std::fixed<<std::setprecision(2)<<pairAngleDeg(legs[i],legs[i+1])
             <<" deg\n";
  }
  const double mean=sum/legs.size();
  const double sd=std::sqrt(std::max(0.0,sum2/legs.size()-mean*mean));
  const double scale=mean/kTruthM;
  std::cout<<std::setprecision(2)
           <<"MEAN HORIZONTAL: "<<mean*1000.0<<" mm\n"
           <<"STD: "<<sd*1000.0<<" mm\n"
           <<"A->B mean: "<<(nab?ab/nab*1000.0:0)<<" mm\n"
           <<"B->A mean: "<<(nba?ba/nba*1000.0:0)<<" mm\n"
           <<std::setprecision(6)
           <<"MEAN SCALE measured/true: "<<scale<<"\n"
           <<"CORRECTION true/measured: "<<(scale>1e-9?1.0/scale:0.0)<<"\n"
           <<"CSV: "<<kCsvPath<<"\n";
}

void drawHud(const cv::Mat&gray,const HudPipeline&pipeline,const FcAttitude&fc,
             int leg,bool have_start,const std::vector<Leg>&legs,
             const std::string&notice){
  cv::Mat bgr,video;
  cv::cvtColor(gray,bgr,cv::COLOR_GRAY2BGR);
  cv::resize(bgr,video,{900,675},0,0,cv::INTER_NEAREST);
  cv::Mat panel(900,380,CV_8UC3,cv::Scalar(24,24,24));
  cv::Mat canvas(900,1280,CV_8UC3,cv::Scalar(8,8,8));
  video.copyTo(canvas(cv::Rect(0,105,900,675)));

  ru(canvas,"JT-ZERO — 300 мм × 6 — fusion v7",{28,52},27,{245,245,245},cv::QT_FONT_BOLD);
  ru(canvas,"Во время движения: только FLU + ZXY gyro",{28,84},17,{190,190,190});
  char b[192];
  std::snprintf(b,sizeof(b),"ПРОХОД %d / %d",std::min(leg+1,kLegCount),kLegCount);
  ru(panel,b,{18,55},25,{245,245,245},cv::QT_FONT_BOLD);
  ru(panel,(leg%2==0)?"A  →  B":"B  →  A",{18,105},30,{0,230,255},cv::QT_FONT_BOLD);

  VioState s;
  const bool have_state=pipeline.latest(&s);
  ru(panel,have_state?"КООРДИНАТЫ: ЕСТЬ":"КООРДИНАТЫ: НЕТ",{18,165},19,
     have_state?cv::Scalar(90,220,90):cv::Scalar(40,40,245),cv::QT_FONT_BOLD);
  if(have_state){
    std::snprintf(b,sizeof(b),"P [%.3f  %.3f  %.3f] м",s.px,s.py,s.pz);
    ru(panel,b,{18,205},16,{220,220,220});
    std::snprintf(b,sizeof(b),"KF %lld",(long long)s.keyframe);
    ru(panel,b,{18,238},15,{190,190,190});
  }
  if(fc.valid){
    std::snprintf(b,sizeof(b),"FC: R %.2f  P %.2f  Y %.2f°",fc.roll_deg,fc.pitch_deg,fc.yaw_deg);
    ru(panel,b,{18,280},14,{190,190,190});
  }

  ru(panel,have_start?"FUSION: ДВИЖЕНИЕ — ACC GRAVITY OFF":"FUSION: СТАТИКА — ACC GRAVITY ARMED",
     {18,320},14,have_start?cv::Scalar(0,210,255):cv::Scalar(90,220,90),cv::QT_FONT_BOLD);
  ru(panel,!have_start?"Поставьте камеру на метку и нажмите СТАРТ":"Переместите ровно на 300 мм и нажмите КОНЕЦ",
     {18,355},14,{220,220,220});
  cv::rectangle(panel,{18,395,344,92},cv::Scalar(55,145,55),-1);
  cv::rectangle(panel,{18,395,344,92},cv::Scalar(220,220,220),2);
  ru(panel,have_start?"ПРОБЕЛ / ENTER: КОНЕЦ":"ПРОБЕЛ / ENTER: СТАРТ",
     {35,452},18,{255,255,255},cv::QT_FONT_BOLD);

  if(!legs.empty()){
    const Leg&r=legs.back();
    std::snprintf(b,sizeof(b),"Последний: %.1f мм",r.horizontal*1000.0);
    ru(panel,b,{18,555},20,{245,245,245},cv::QT_FONT_BOLD);
    std::snprintf(b,sizeof(b),"Ошибка: %+.1f мм (%+.2f%%)",r.error*1000.0,r.error/kTruthM*100.0);
    ru(panel,b,{18,595},16,{220,220,220});
    std::snprintf(b,sizeof(b),"ΔZ: %+.1f мм",r.dz*1000.0);
    ru(panel,b,{18,630},15,{190,190,190});
  }
  ru(panel,"PASS: ±60 мм, σ≤40 мм, разворот >134°",{18,700},13,{185,185,185});
  ru(panel,"Q / ESC — выход",{18,735},15,{215,215,215});
  if(!notice.empty()){
    cv::rectangle(panel,{10,790,360,70},cv::Scalar(35,35,125),-1);
    ru(panel,notice,{20,835},14,{255,255,255},cv::QT_FONT_BOLD);
  }
  panel.copyTo(canvas(cv::Rect(900,0,380,900)));
  cv::imshow(kWindowName,canvas);
}
} // namespace

int main(int argc,char**argv){
  google::InitGoogleLogging(argv[0]);
  FLAGS_visualize=false;
  FLAGS_viz_type=2;
  FLAGS_use_lcd=false;
  FLAGS_log_output=false;
  FLAGS_extract_planes_from_the_scene=false;
  const std::string params=argc>1?argv[1]:"params/JTZeroMonoFLU";

  int camera_fd=-1,serial_fd=-1;
  bool streaming=false,imu_req=false,att_req=false,pipeline_started=false,aborted=false;
  uint8_t sys=0,comp=0;
  std::vector<CameraBuffer>buffers;
  std::shared_ptr<HudPipeline>pipeline;
  std::thread pipeline_thread;

  try{
    VIO::VioParams vp(params);
    if(vp.camera_params_.empty())throw std::runtime_error("No camera params loaded");
    pipeline=std::make_shared<HudPipeline>(vp);
    pipeline->installBackendCallback();
    pipeline_thread=std::thread([pipeline](){pipeline->spin();});
    pipeline_started=true;

    serial_fd=openSerial();
    mavlink_status_t mst{};
    mavlink_message_t msg{};
    std::cout<<"[MAV] waiting for HEARTBEAT...\n";
    int64_t dl=monotonicNs()+10000000000LL;
    while(monotonicNs()<dl&&!sys){
      pollfd p{serial_fd,POLLIN,0};
      if(poll(&p,1,100)<=0)continue;
      uint8_t b[2048];
      ssize_t n=read(serial_fd,b,sizeof(b));
      if(n<=0)continue;
      for(ssize_t i=0;i<n;++i){
        if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst)&&msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){
          sys=msg.sysid;comp=msg.compid;break;
        }
      }
    }
    if(!sys)throw std::runtime_error("HEARTBEAT timeout");
    requestRate(serial_fd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,kImuRateHz);imu_req=true;
    requestRate(serial_fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,kFcAttitudeRateHz);att_req=true;

    camera_fd=open(kCameraDevice,O_RDWR|O_NONBLOCK);
    if(camera_fd==-1)fail("open camera");
    configureCamera(camera_fd);
    buffers=initCameraBuffers(camera_fd);
    v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(xioctl(camera_fd,VIDIOC_STREAMON,&type)==-1)fail("STREAMON");
    streaming=true;
    discardWarmup(camera_fd);
    cv::setNumThreads(1);
    cv::namedWindow(kWindowName,cv::WINDOW_NORMAL);
    cv::resizeWindow(kWindowName,1280,900);

    std::vector<TimeSyncSample>sync;
    sync.reserve(500);
    ClockMapping mapping;
    int64_t pending=0,next_sync=monotonicNs();
    size_t raw=0,rejected=0,selected=0,decoded=0,imu_rx=0,imu_fed=0,imu_skip=0,att_rx=0;
    uint32_t prev_seq=0;
    int64_t prev_ts=0,last_sel=0;
    bool have_prev=false;
    VIO::FrameId fid=0;
    FcAttitude fc;
    cv::Mat gray(kHeight,kWidth,CV_8UC1,cv::Scalar(0));
    jtzero::ImuCorrection imu_correction;
    int leg=0;
    bool have_start=false;
    Mark start_mark;
    std::vector<Leg>legs;
    std::string notice;
    int64_t notice_until=0;
    int64_t next_hud=0;

    while(leg<kLegCount){
      const int64_t now=monotonicNs();
      if(now>=next_sync&&pending==0){
        pending=now;
        sendTimesync(serial_fd,pending,sys,comp);
        next_sync=now+kTimesyncPeriodNs;
      }
      pollfd pf[2]={{camera_fd,POLLIN,0},{serial_fd,POLLIN,0}};
      int rc=poll(pf,2,2);
      if(rc<0){if(errno==EINTR)continue;fail("poll");}

      if(pf[1].revents&POLLIN){
        uint8_t b[8192];
        for(;;){
          ssize_t n=read(serial_fd,b,sizeof(b));
          if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK))break;
          if(n<=0)break;
          for(ssize_t i=0;i<n;++i){
            if(!mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst))continue;
            const int64_t recv=monotonicNs();
            if(msg.msgid==MAVLINK_MSG_ID_TIMESYNC){
              mavlink_timesync_t ts{};
              mavlink_msg_timesync_decode(&msg,&ts);
              if(ts.tc1!=0&&pending&&ts.ts1==pending){
                TimeSyncSample s;
                s.t0_rpi_ns=pending;s.t1_rpi_ns=recv;s.fc_ns=ts.tc1;
                s.rtt_ns=recv-pending;s.rpi_mid_ns=pending+s.rtt_ns/2;
                s.good=s.rtt_ns>0&&nsToMs(s.rtt_ns)<=kMaxTimesyncRttMs;
                sync.push_back(s);pending=0;mapping=estimateClockMapping(sync);
              }
            }else if(msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){
              ++imu_rx;
              mavlink_highres_imu_t h{};
              mavlink_msg_highres_imu_decode(&msg,&h);
              if(!mapping.valid||recv-mapping.last_update_ns>kMappingStaleNs){++imu_skip;continue;}
              const Eigen::Vector3d acc=jtzero::ImuCorrection::accelFrdToFlu(h.xacc,h.yacc,h.zacc);
              const Eigen::Vector3d gyro=jtzero::ImuCorrection::gyroFrdToFlu(h.xgyro,h.ygyro,h.zgyro);
              const bool allow_gravity_feedback=!have_start;
              const Eigen::Vector3d w=imu_correction.correctGyro(h.time_usec,acc,gyro,allow_gravity_feedback);
              VIO::ImuAccGyr data;
              data<<acc.x(),acc.y(),acc.z(),w.x(),w.y(),w.z();
              pipeline->fillSingleImuQueue(VIO::ImuMeasurement(mapping.map((int64_t)h.time_usec*1000LL),data));
              ++imu_fed;
            }else if(msg.msgid==MAVLINK_MSG_ID_ATTITUDE){
              mavlink_attitude_t a{};
              mavlink_msg_attitude_decode(&msg,&a);
              ++att_rx;
              fc.valid=true;fc.wall_ns=recv;
              fc.roll_deg=a.roll*180.0/kPi;fc.pitch_deg=a.pitch*180.0/kPi;fc.yaw_deg=a.yaw*180.0/kPi;
            }
          }
        }
      }

      if(pending&&monotonicNs()-pending>20000000LL)pending=0;

      if(pf[0].revents&POLLIN){
        for(;;){
          v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;
          if(xioctl(camera_fd,VIDIOC_DQBUF,&b)==-1){if(errno==EAGAIN)break;fail("DQBUF");}
          ++raw;
          const int64_t corrected=jtzero::timesync::correctCameraTimestampNs(timevalToNs(b.timestamp));
          bool ok=true;
          if(have_prev){
            const int64_t dt=corrected-prev_ts;
            ok=b.sequence==prev_seq+1U&&dt>0&&dt<=20000000LL;
            if(!ok)++rejected;
          }
          prev_seq=b.sequence;prev_ts=corrected;have_prev=true;
          const bool due=last_sel==0||corrected-last_sel>=30000000LL;
          if(ok&&due&&mapping.valid){
            std::vector<unsigned char>jpg(b.bytesused);
            std::memcpy(jpg.data(),buffers[b.index].start,b.bytesused);
            cv::Mat g=cv::imdecode(jpg,cv::IMREAD_GRAYSCALE);
            if(!g.empty()){
              ++decoded;gray=g;
              pipeline->fillLeftFrameQueue(std::make_unique<VIO::Frame>(fid++,corrected,vp.camera_params_.at(0),g.clone()));
              last_sel=corrected;++selected;
            }
          }
          if(xioctl(camera_fd,VIDIOC_QBUF,&b)==-1)fail("QBUF");
        }
      }

      if(now>=next_hud){
        if(monotonicNs()>notice_until)notice.clear();
        drawHud(gray,*pipeline,fc,leg,have_start,legs,notice);
        next_hud=now+kHudPeriodNs;
      }

      const int key=cv::waitKey(1)&0xff;
      if(key==27||key=='q'||key=='Q'){aborted=true;break;}
      if(key==32||key==13||key==10){
        VioState s;
        if(!pipeline->latest(&s)){
          notice="КНОПКА ПРИНЯТА, НО КООРДИНАТ ПОКА НЕТ";
          notice_until=monotonicNs()+2000000000LL;
          std::cout<<"[HUD] capture key accepted, no backend state yet; raw="<<raw
                   <<" selected="<<selected<<" imu_fed="<<imu_fed
                   <<" sync="<<sync.size()<<" mapping="<<(mapping.valid?"yes":"no")<<"\n";
        }else if(!have_start){
          start_mark.s=s;start_mark.valid=true;have_start=true;
          notice="СТАРТ ЗАФИКСИРОВАН — GRAVITY FEEDBACK OFF";
          notice_until=monotonicNs()+1200000000LL;
          std::cout<<"LEG "<<leg+1<<" START KF="<<s.keyframe<<" [gravity feedback OFF]\n";
        }else{
          Mark end;end.s=s;end.valid=true;
          Leg r=makeLeg(leg,start_mark,end);
          legs.push_back(r);
          std::cout<<"LEG "<<leg+1<<' '<<r.direction
                   <<" horizontal="<<std::fixed<<std::setprecision(2)<<r.horizontal*1000.0
                   <<" mm error="<<r.error*1000.0<<" mm dz="<<r.dz*1000.0
                   <<" mm [gravity feedback re-armed]\n";
          ++leg;have_start=false;
          notice="КОНЕЦ ЗАФИКСИРОВАН — СТАТИЧЕСКАЯ КОРРЕКЦИЯ ВКЛЮЧЕНА";
          notice_until=monotonicNs()+1200000000LL;
        }
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    pipeline->shutdown();
    if(pipeline_thread.joinable())pipeline_thread.join();
    pipeline_started=false;
    cv::destroyAllWindows();
    const auto states=pipeline->states();
    saveCsv(legs);

    if(imu_req){requestRate(serial_fd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);imu_req=false;}
    if(att_req){requestRate(serial_fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);att_req=false;}
    if(streaming){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(camera_fd,VIDIOC_STREAMOFF,&t);streaming=false;}
    for(auto&bf:buffers)if(bf.start&&bf.start!=MAP_FAILED)munmap(bf.start,bf.length);
    if(camera_fd!=-1)close(camera_fd);
    if(serial_fd!=-1)close(serial_fd);

    printSummary(legs,raw,rejected,selected,decoded,imu_rx,imu_fed,att_rx,sync.size(),mapping,states.size());
    double sd=0.0;
    const bool metric_pass=measurementPass(legs,&sd);
    const bool pipeline_pass=!aborted&&legs.size()==kLegCount&&mapping.valid&&states.size()>0;
    std::cout<<"PIPELINE RESULT: "<<(pipeline_pass?"PASS":"FAIL")<<"\n";
    std::cout<<"MEASUREMENT RESULT: "<<(metric_pass?"PASS":"FAIL")<<"\n";
    const bool pass=pipeline_pass&&metric_pass;
    std::cout<<"RESULT: "<<(pass?"PASS":"FAIL")<<"\n";
    return pass?0:1;
  }catch(const std::exception&e){
    std::cerr<<"[FATAL] "<<e.what()<<"\n";
    if(pipeline)pipeline->shutdown();
    if(pipeline_started&&pipeline_thread.joinable())pipeline_thread.join();
    try{cv::destroyAllWindows();}catch(...){}
    if(serial_fd!=-1&&imu_req&&sys)try{requestRate(serial_fd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);}catch(...){}
    if(serial_fd!=-1&&att_req&&sys)try{requestRate(serial_fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);}catch(...){}
    if(streaming&&camera_fd!=-1){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(camera_fd,VIDIOC_STREAMOFF,&t);}
    for(auto&bf:buffers)if(bf.start&&bf.start!=MAP_FAILED)munmap(bf.start,bf.length);
    if(camera_fd!=-1)close(camera_fd);
    if(serial_fd!=-1)close(serial_fd);
    return 1;
  }
}
