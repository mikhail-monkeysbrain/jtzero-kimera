// JT-Zero — отдельный bench MVP для штатного ArduPilot OpticalFlow.
//
// Цель: проверить архитектуру "OV9281 -> raw optical-flow rate -> MAVLink
// OPTICAL_FLOW -> ArduPilot EKF3" без собственного интегратора absolute X/Y.
// Production ground_motion_mvp.cpp этот файл не изменяет и не использует.
//
// Вход camera_yaml тот же, что у Ground Motion: intrinsics + T_BS в FLU.
// Выход flow_rate_x/y заранее поворачивается в body FRD, поэтому для этого
// диагностического publisher ожидается FLOW_ORIENT_YAW=0 и FLOW_OPTIONS=0.

#define main jtzero_optflow_base_unused_main
#include "ground_motion_live_v2_v3_ab.cpp"
#undef main
#include "ground_motion_mavlink.hpp"

#include <deque>
#include <sstream>

namespace {

struct FlowFcLocal {
  float x=0,y=0,z=0,vx=0,vy=0,vz=0;
  int64_t recv_ns=0;
  bool valid=false;
};

struct FlowFc {
  int fd=-1;
  std::thread th;
  std::mutex mu;
  FlowFcLocal local{};
  uint64_t local_count=0;
  uint8_t target_sys=0,target_comp=0;

  static constexpr uint8_t self_sys=191;
  static constexpr uint8_t self_comp=MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY;

  ~FlowFc(){ stop(); }

  static void writeAll(int fd,const uint8_t* p,size_t n){
    size_t o=0;
    while(o<n){
      const ssize_t k=::write(fd,p+o,n-o);
      if(k>0){o+=static_cast<size_t>(k);continue;}
      if(k<0&&(errno==EAGAIN||errno==EWOULDBLOCK)){
        pollfd q{fd,POLLOUT,0}; poll(&q,1,10); continue;
      }
      if(k<0&&errno==EINTR)continue;
      fail("FC write");
    }
  }

  static void requestRate(int fd,uint8_t sys,uint8_t comp,uint32_t msgid,int hz){
    mavlink_message_t m{};
    mavlink_msg_command_long_pack(self_sys,self_comp,&m,sys,comp,
      MAV_CMD_SET_MESSAGE_INTERVAL,0,msgid,1000000.0f/hz,0,0,0,0,0);
    uint8_t b[MAVLINK_MAX_PACKET_LEN];
    const auto n=mavlink_msg_to_send_buffer(b,&m);
    writeAll(fd,b,n);
  }

  void start(const std::string& dev){
    fd=::open(dev.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK);
    if(fd<0)fail("open FC");
    termios t{};
    if(tcgetattr(fd,&t)<0)fail("FC tcgetattr");
    cfmakeraw(&t);
    cfsetispeed(&t,B460800); cfsetospeed(&t,B460800);
    t.c_cflag|=CLOCAL|CREAD;
    t.c_cflag&=~CRTSCTS; t.c_cflag&=~PARENB; t.c_cflag&=~CSTOPB;
    t.c_cflag&=~CSIZE; t.c_cflag|=CS8;
    if(tcsetattr(fd,TCSANOW,&t)<0)fail("FC tcsetattr");
    tcflush(fd,TCIFLUSH);

    th=std::thread([this]{
      mavlink_status_t st{}; mavlink_message_t m{}; uint8_t buf[4096];
      uint8_t sys=0,comp=0;
      const int64_t deadline=monoNs()+10000000000LL;

      while(g_running&&!sys&&monoNs()<deadline){
        pollfd p{fd,POLLIN,0};
        if(poll(&p,1,100)<=0)continue;
        const ssize_t n=read(fd,buf,sizeof(buf));
        if(n<=0)continue;
        for(ssize_t i=0;i<n;i++){
          if(!mavlink_parse_char(MAVLINK_COMM_0,buf[i],&m,&st))continue;
          if(m.msgid!=MAVLINK_MSG_ID_HEARTBEAT)continue;
          mavlink_heartbeat_t hb{}; mavlink_msg_heartbeat_decode(&m,&hb);
          if(hb.autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA){sys=m.sysid;comp=m.compid;break;}
        }
      }

      if(!sys){std::cerr<<"FC: ArduPilot HEARTBEAT timeout\n";g_running=false;return;}
      target_sys=sys; target_comp=comp;
      std::cerr<<"FC: ArduPilot heartbeat sys="<<(int)sys<<" comp="<<(int)comp<<"\n";
      requestRate(fd,sys,comp,MAVLINK_MSG_ID_LOCAL_POSITION_NED,20);

      while(g_running){
        pollfd p{fd,POLLIN,0};
        if(poll(&p,1,50)<=0)continue;
        for(;;){
          const ssize_t n=read(fd,buf,sizeof(buf));
          if(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK))break;
          if(n<=0)break;
          for(ssize_t i=0;i<n;i++){
            if(!mavlink_parse_char(MAVLINK_COMM_0,buf[i],&m,&st))continue;
            if(m.msgid==MAVLINK_MSG_ID_LOCAL_POSITION_NED && m.sysid==sys){
              mavlink_local_position_ned_t q{}; mavlink_msg_local_position_ned_decode(&m,&q);
              std::lock_guard<std::mutex> l(mu);
              local.x=q.x; local.y=q.y; local.z=q.z;
              local.vx=q.vx; local.vy=q.vy; local.vz=q.vz;
              local.recv_ns=monoNs(); local.valid=true; ++local_count;
            }
          }
        }
      }
    });
  }

  bool latestLocal(FlowFcLocal* out,double* age_ms,uint64_t* count=nullptr){
    std::lock_guard<std::mutex> l(mu);
    if(count)*count=local_count;
    if(!local.valid)return false;
    *out=local;
    if(age_ms)*age_ms=(monoNs()-local.recv_ns)*1e-6;
    return true;
  }

  void stop(){
    if(th.joinable())th.join();
    if(fd>=0){::close(fd);fd=-1;}
  }
};

bool sendOpticalFlow(int fd,uint64_t time_usec,float rate_x,float rate_y,uint8_t quality){
  if(fd<0 || !std::isfinite(rate_x) || !std::isfinite(rate_y))return false;
  mavlink_message_t msg{};
  mavlink_msg_optical_flow_pack(
    FlowFc::self_sys,FlowFc::self_comp,&msg,time_usec,
    0,                 // sensor_id
    0,0,               // legacy integer flow_x/y intentionally unused
    0.0f,0.0f,         // flow_comp_m_x/y unused by ArduPilot MAV backend
    quality,
    -1.0f,              // range comes independently through DISTANCE_SENSOR
    rate_x,rate_y);
  return GroundMotionMavlinkPublisher::writeMessage(fd,msg);
}

struct FlowStep {
  bool valid=false;
  int features=0,tracked=0,inliers=0;
  double inlier_ratio=0;
  double du_norm=0,dv_norm=0;
  double du_px=0,dv_px=0;
  double flow_cam_x=0,flow_cam_y=0;
  double flow_body_x=0,flow_body_y=0;
};

FlowStep estimateRawFlow(const cv::Mat& prev,const cv::Mat& curr,double dt,const CameraCalib& calib){
  FlowStep o;
  if(prev.empty()||curr.empty()||!(dt>0&&dt<0.2))return o;

  cv::Mat feature_mask(prev.size(),CV_8UC1,cv::Scalar(0));
  const int x0=prev.cols/5, y0=prev.rows/5;
  const int rw=prev.cols*3/5, rh=prev.rows*3/5;
  feature_mask(cv::Rect(x0,y0,rw,rh)).setTo(255);

  std::vector<cv::Point2f> p0,p1;
  cv::goodFeaturesToTrack(prev,p0,500,0.01,7,feature_mask);
  o.features=(int)p0.size();
  if(p0.size()<30)return o;

  std::vector<uchar> st; std::vector<float> err;
  cv::calcOpticalFlowPyrLK(prev,curr,p0,p1,st,err,{21,21},3);
  std::vector<cv::Point2f> a,b;
  for(size_t i=0;i<p0.size();++i){
    if(st[i]){a.push_back(p0[i]);b.push_back(p1[i]);}
  }
  o.tracked=(int)a.size();
  if(a.size()<20)return o;

  cv::Mat mask;
  cv::findHomography(a,b,cv::RANSAC,2.0,mask);
  if(mask.empty())return o;

  std::vector<cv::Point2f> ai,bi;
  for(size_t i=0;i<a.size();++i){
    if(mask.at<uchar>((int)i)){ai.push_back(a[i]);bi.push_back(b[i]);}
  }
  o.inliers=(int)ai.size();
  o.inlier_ratio=a.empty()?0.0:(double)ai.size()/a.size();
  if(ai.size()<20)return o;

  std::vector<cv::Point2f> au,bu;
  cv::undistortPoints(ai,au,calib.K,calib.D);
  cv::undistortPoints(bi,bu,calib.K,calib.D);

  std::vector<double> dun,dvn,dup,dvp;
  dun.reserve(ai.size()); dvn.reserve(ai.size()); dup.reserve(ai.size()); dvp.reserve(ai.size());
  for(size_t i=0;i<ai.size();++i){
    dun.push_back((double)bu[i].x-au[i].x);
    dvn.push_back((double)bu[i].y-au[i].y);
    dup.push_back((double)bi[i].x-ai[i].x);
    dvp.push_back((double)bi[i].y-ai[i].y);
  }
  o.du_norm=median(dun); o.dv_norm=median(dvn);
  o.du_px=median(dup); o.dv_px=median(dvp);

  // OpenCV camera: +X image-right, +Y image-down, +Z optical-forward.
  // Near principal point a positive RH camera rotation about Cx produces +dv,
  // and a positive rotation about Cy produces -du.  Это именно raw sensor-flow
  // convention, который ожидает AP_OpticalFlow_MAV до внутренней смены знака.
  o.flow_cam_x=o.dv_norm/dt;
  o.flow_cam_y=-o.du_norm/dt;

  // camera -> body FLU берём из T_BS, затем FLU -> ArduPilot body FRD.
  const cv::Matx33d FLU_TO_FRD(1,0,0, 0,-1,0, 0,0,-1);
  const cv::Matx33d FRD_R_C=FLU_TO_FRD*calib.B_R_C;
  const cv::Vec3d fb=FRD_R_C*cv::Vec3d(o.flow_cam_x,o.flow_cam_y,0.0);
  o.flow_body_x=fb[0]; o.flow_body_y=fb[1];

  const double mag=std::hypot(o.flow_body_x,o.flow_body_y);
  o.valid=std::isfinite(mag) && mag<4.0;
  return o;
}

} // namespace

int main(int argc,char** argv){
  if(argc<7){
    std::cerr<<"Использование: "<<argv[0]
             <<" <camera> <luna> <fc> <csv> <camera_yaml> <focal_scale>\n";
    return 2;
  }

  const std::string camdev=argv[1], lunadev=argv[2], fcdev=argv[3];
  const std::string csvpath=argv[4], yaml=argv[5];
  const double focal_scale=std::stod(argv[6]);
  if(!(focal_scale>0.5&&focal_scale<2.0)){
    std::cerr<<"ОШИБКА: focal_scale вне разумного диапазона 0.5..2.0\n";
    return 2;
  }

  try{
    CameraCalib calib=loadCameraCalib(yaml);
    calib.fx*=focal_scale; calib.fy*=focal_scale;
    calib.K=(cv::Mat_<double>(3,3)<<calib.fx,0,calib.cx,0,calib.fy,calib.cy,0,0,1);

    Camera cam; cam.openDev(camdev);
    LunaReader luna; luna.start(lunadev);
    FlowFc fc; fc.start(fcdev);
    GroundMotionMavlinkPublisher range_pub;
    range_pub.system_id=FlowFc::self_sys;
    range_pub.component_id=FlowFc::self_comp;

    std::ofstream csv(csvpath,std::ios::trunc);
    csv<<"mono_ns,frame,valid,dt_s,features,tracked,inliers,inlier_ratio,du_px,dv_px,du_norm,dv_norm,flow_cam_x,flow_cam_y,flow_body_x,flow_body_y,quality,luna_m,luna_age_ms,flow_sent,range_sent,ekf_local_valid,ekf_x_ned,ekf_y_ned,ekf_z_ned,ekf_vx_ned,ekf_vy_ned,ekf_vz_ned,ekf_age_ms,ekf_count\n";

    cv::setNumThreads(1);
    std::signal(SIGINT,onSignal); std::signal(SIGTERM,onSignal);

    cv::Mat prev; int64_t prev_ts=0; uint64_t frame=0;
    uint64_t flow_sent_total=0,flow_invalid_total=0,range_sent_total=0;
    int64_t last_range_send_ns=0;

    std::cerr<<"JT-ZERO OPTICAL FLOW MAVLINK MVP\n"
             <<"camera="<<camdev<<"\n"
             <<"fx/fy effective="<<calib.fx<<" / "<<calib.fy
             <<" (focal_scale="<<focal_scale<<")\n"
             <<"ВАЖНО: publisher выдаёт body-FRD flow; ожидается FLOW_ORIENT_YAW=0, FLOW_OPTIONS=0\n";

    // Переводим AP_OpticalFlow_MAV в high-precision flow_rate mode даже если
    // первые реальные кадры почти неподвижны. quality=0 => это не валидное aiding measurement.
    sendOpticalFlow(fc.fd,(uint64_t)(monoNs()/1000),1.0e-6f,0.0f,0);

    while(g_running){
      pollfd p{cam.fd,POLLIN,0};
      const int pr=poll(&p,1,20);
      if(pr<0){if(errno==EINTR)continue;fail("camera poll");}
      if(pr<=0)continue;

      while(g_running){
        v4l2_buffer b{}; b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory=V4L2_MEMORY_MMAP;
        if(xioctl(cam.fd,VIDIOC_DQBUF,&b)<0){if(errno==EAGAIN)break;fail("VIDIOC_DQBUF");}
        const int64_t now=monoNs();
        const int64_t ts=(int64_t)b.timestamp.tv_sec*1000000000LL+(int64_t)b.timestamp.tv_usec*1000LL;
        cv::Mat raw(1,(int)b.bytesused,CV_8UC1,cam.bufs[b.index].p);
        cv::Mat gray=cv::imdecode(raw,cv::IMREAD_GRAYSCALE);
        if(xioctl(cam.fd,VIDIOC_QBUF,&b)<0)fail("VIDIOC_QBUF");
        if(gray.empty())continue; // decode dropout: старый prev сохраняется
        ++frame;

        double lm=0; int strength=0; int64_t lns=0;
        const bool hl=luna.latest(&lm,&strength,&lns);
        const double lage=hl?(now-lns)*1e-6:1e9;
        bool range_sent=false;
        if(hl&&lage<200&&(last_range_send_ns==0||now-last_range_send_ns>=50000000LL)){
          range_sent=range_pub.sendDistanceSensor(fc.fd,(uint32_t)(now/1000000LL),lm);
          last_range_send_ns=now;
          if(range_sent)++range_sent_total;
        }

        const double dt=prev_ts?(ts-prev_ts)*1e-9:0.0;
        FlowStep s;
        if(!prev.empty())s=estimateRawFlow(prev,gray,dt,calib);

        bool flow_sent=false; uint8_t quality=0;
        if(s.valid){
          // Диагностический MVP использует stringent KLT+RANSAC gate, а после
          // успешного gate — max quality, как BlueOS. Динамическую quality
          // калибруем отдельно после доказательства осей/масштаба.
          quality=255;
          flow_sent=sendOpticalFlow(fc.fd,(uint64_t)(now/1000),
            (float)s.flow_body_x,(float)s.flow_body_y,quality);
          if(flow_sent)++flow_sent_total;
        } else if(!prev.empty()){
          ++flow_invalid_total;
        }

        FlowFcLocal ep{}; double eage=1e9; uint64_t ec=0;
        const bool eok=fc.latestLocal(&ep,&eage,&ec);
        const bool efresh=eok&&eage<500.0;

        csv<<now<<','<<frame<<','<<(s.valid?1:0)<<','<<dt<<','
           <<s.features<<','<<s.tracked<<','<<s.inliers<<','<<s.inlier_ratio<<','
           <<s.du_px<<','<<s.dv_px<<','<<s.du_norm<<','<<s.dv_norm<<','
           <<s.flow_cam_x<<','<<s.flow_cam_y<<','<<s.flow_body_x<<','<<s.flow_body_y<<','
           <<(int)quality<<','<<lm<<','<<lage<<','<<(flow_sent?1:0)<<','<<(range_sent?1:0)<<','
           <<(efresh?1:0)<<','<<ep.x<<','<<ep.y<<','<<ep.z<<','<<ep.vx<<','<<ep.vy<<','<<ep.vz<<','<<eage<<','<<ec<<'\n';

        if(frame%100==0){
          std::cerr<<"OF frame="<<frame
                   <<" valid="<<(s.valid?1:0)
                   <<" rateFRD=("<<s.flow_body_x<<","<<s.flow_body_y<<") rad/s"
                   <<" inliers="<<s.inliers<<"/"<<s.tracked
                   <<" sent="<<flow_sent_total<<" invalid="<<flow_invalid_total
                   <<" range="<<range_sent_total;
          if(efresh)std::cerr<<" EKF pN/E=("<<ep.x<<","<<ep.y<<") vN/E=("<<ep.vx<<","<<ep.vy<<")";
          std::cerr<<"\r"<<std::flush;
        }

        // Как в BlueOS: любой успешно декодированный кадр становится новым prev.
        // Здесь это не портит собственный absolute X/Y, потому что такого интегратора нет.
        prev=gray.clone(); prev_ts=ts;
      }
    }

    g_running=false; fc.stop(); luna.stop();
    std::cerr<<"\nОстановлено. CSV: "<<csvpath
             <<" flow_sent="<<flow_sent_total
             <<" invalid="<<flow_invalid_total
             <<" range_sent="<<range_sent_total<<"\n";
    return 0;
  } catch(const std::exception& e){
    g_running=false;
    std::cerr<<"ОШИБКА: "<<e.what()<<"\n";
    return 1;
  }
}
