// JT-Zero — OpticalFlow MAVLink bench v2 с прямой диагностикой EKF_STATUS_REPORT.
//
// Цель: проверить цепочку OV9281 -> raw optical-flow rate -> MAVLink OPTICAL_FLOW
// -> ArduPilot EKF3 без собственного интегратора absolute X/Y и одновременно
// видеть, какие EKF status flags реально подняты.
//
// ВАЖНО: сначала подключаем ardupilotmega dialect. Включаемый ниже legacy/base
// файл сам включает common/mavlink.h; после этого include guard уже не даст
// переопределить dialect, поэтому порядок здесь принципиален.
#include "ardupilotmega/mavlink.h"

#define main jtzero_optflow_base_unused_main
#include "ground_motion_live_v2_v3_ab.cpp"
#undef main
#include "ground_motion_mavlink.hpp"

#include <deque>
#include <sstream>
#include <atomic>
#include <array>
#include <map>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>

#if __has_include(<opencv2/freetype.hpp>)
#include <opencv2/freetype.hpp>
#define JTZERO_GUI_FREETYPE 1
#else
#define JTZERO_GUI_FREETYPE 0
#endif

namespace {

#if JTZERO_GUI_FREETYPE
cv::Ptr<cv::freetype::FreeType2> g_gui_font;
#endif

bool initGuiFont(){
#if JTZERO_GUI_FREETYPE
  const std::array<const char*,6> candidates{{
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSansCondensed.ttf",
    "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf",
    "/usr/share/fonts/truetype/freefont/FreeSans.ttf",
    "/usr/share/fonts/opentype/dejavu/DejaVuSans.ttf",
    "/usr/share/fonts/dejavu/DejaVuSans.ttf"
  }};
  for(const char* p:candidates){
    std::ifstream fh(p,std::ios::binary);
    if(!fh.good()) continue;
    try{
      g_gui_font=cv::freetype::createFreeType2();
      g_gui_font->loadFontData(p,0);
      std::cerr<<"GUI: русский UTF-8 шрифт: "<<p<<"\n";
      return true;
    }catch(const cv::Exception&){}
  }
#endif
  std::cerr<<"ПРЕДУПРЕЖДЕНИЕ: UTF-8 шрифт GUI не найден; кириллица может отображаться некорректно.\n";
  return false;
}

void putGuiText(cv::Mat& img,const std::string& text,cv::Point org,
                double scale,cv::Scalar color,int thickness=1){
#if JTZERO_GUI_FREETYPE
  if(g_gui_font){
    const int h=std::max(12,(int)std::lround(31.0*scale));
    g_gui_font->putText(img,text,org,h,color,thickness,cv::LINE_AA,true);
    return;
  }
#endif
  cv::putText(img,text,org,cv::FONT_HERSHEY_SIMPLEX,scale,color,thickness,cv::LINE_AA);
}

struct FlowFcLocal {
  float x=0,y=0,z=0,vx=0,vy=0,vz=0;
  int64_t recv_ns=0;
  bool valid=false;
};

struct FlowEkfStatus {
  uint16_t flags=0;
  float velocity_variance=0;
  float pos_horiz_variance=0;
  float pos_vert_variance=0;
  float compass_variance=0;
  float terrain_alt_variance=0;
  int64_t recv_ns=0;
  bool valid=false;
};

struct FlowFcGyro {
  double roll=0,pitch=0,yaw=0; // ATTITUDE angles, rad
  double x=0,y=0,z=0;          // body FRD roll/pitch/yaw rates, rad/s
  int64_t recv_ns=0;
  bool valid=false;
};

struct FlowFc {
  int fd=-1;
  std::thread th;
  std::mutex mu;
  FlowFcLocal local{};
  FlowEkfStatus ekf{};
  FlowFcGyro gyro{};
  uint64_t local_count=0;
  uint64_t ekf_count=0;
  uint64_t gyro_count=0;
  double gyro_sum_x=0,gyro_sum_y=0,gyro_sum_z=0;
  uint64_t gyro_sum_count=0;
  bool armed=false;
  bool heartbeat_valid=false;
  int64_t heartbeat_recv_ns=0;
  uint8_t target_sys=0,target_comp=0;

  static constexpr size_t remote_block_size=MAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_FIELD_DATA_LEN;
  std::ofstream remote_ofs;
  std::map<uint32_t,std::array<uint8_t,remote_block_size>> remote_pending;
  uint32_t remote_expected=0;
  uint64_t remote_blocks_rx=0,remote_blocks_written=0,remote_duplicates=0;
  bool remote_active=false;
  std::string remote_path;

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

  static void sendRemoteStatus(int fd,uint8_t sys,uint8_t comp,uint32_t seq,uint8_t status){
    mavlink_message_t m{};
    mavlink_msg_remote_log_block_status_pack(self_sys,self_comp,&m,sys,comp,seq,status);
    uint8_t b[MAVLINK_MAX_PACKET_LEN];
    const auto n=mavlink_msg_to_send_buffer(b,&m);
    writeAll(fd,b,n);
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
          if(hb.autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA){
            sys=m.sysid; comp=m.compid;
            {
              std::lock_guard<std::mutex> l(mu);
              armed=(hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)!=0;
              heartbeat_valid=true;
              heartbeat_recv_ns=monoNs();
            }
            break;
          }
        }
      }

      if(!sys){std::cerr<<"FC: ArduPilot HEARTBEAT timeout\n";g_running=false;return;}
      target_sys=sys; target_comp=comp;
      std::cerr<<"FC: ArduPilot heartbeat sys="<<(int)sys<<" comp="<<(int)comp<<"\n";
      requestRate(fd,sys,comp,MAVLINK_MSG_ID_LOCAL_POSITION_NED,20);
      requestRate(fd,sys,comp,MAVLINK_MSG_ID_EKF_STATUS_REPORT,5);
      requestRate(fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,100);

      while(g_running){
        pollfd p{fd,POLLIN,0};
        if(poll(&p,1,50)<=0)continue;
        for(;;){
          const ssize_t n=read(fd,buf,sizeof(buf));
          if(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK))break;
          if(n<=0)break;
          for(ssize_t i=0;i<n;i++){
            if(!mavlink_parse_char(MAVLINK_COMM_0,buf[i],&m,&st))continue;
            if(m.sysid!=sys)continue;
            if(m.msgid==MAVLINK_MSG_ID_REMOTE_LOG_DATA_BLOCK){
              mavlink_remote_log_data_block_t q{}; mavlink_msg_remote_log_data_block_decode(&m,&q);
              std::lock_guard<std::mutex> l(mu);
              if(remote_active && q.target_system==self_sys && q.target_component==self_comp){
                ++remote_blocks_rx;
                if(q.seqno<remote_expected || remote_pending.count(q.seqno)){
                  ++remote_duplicates;
                } else {
                  std::array<uint8_t,remote_block_size> a{};
                  std::memcpy(a.data(),q.data,remote_block_size);
                  remote_pending.emplace(q.seqno,a);
                }
                sendRemoteStatus(fd,sys,comp,q.seqno,MAV_REMOTE_LOG_DATA_BLOCK_ACK);
                for(;;){
                  auto it=remote_pending.find(remote_expected);
                  if(it==remote_pending.end())break;
                  remote_ofs.write(reinterpret_cast<const char*>(it->second.data()),remote_block_size);
                  remote_pending.erase(it);
                  ++remote_expected; ++remote_blocks_written;
                }
              }
            } else if(m.msgid==MAVLINK_MSG_ID_HEARTBEAT){
              mavlink_heartbeat_t hb{}; mavlink_msg_heartbeat_decode(&m,&hb);
              if(hb.autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA){
                std::lock_guard<std::mutex> l(mu);
                armed=(hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)!=0;
                heartbeat_valid=true;
                heartbeat_recv_ns=monoNs();
              }
            } else if(m.msgid==MAVLINK_MSG_ID_ATTITUDE){
              mavlink_attitude_t q{}; mavlink_msg_attitude_decode(&m,&q);
              std::lock_guard<std::mutex> l(mu);
              gyro.roll=q.roll; gyro.pitch=q.pitch; gyro.yaw=q.yaw;
              gyro.x=q.rollspeed; gyro.y=q.pitchspeed; gyro.z=q.yawspeed;
              gyro.recv_ns=monoNs(); gyro.valid=true; ++gyro_count;
              gyro_sum_x+=q.rollspeed; gyro_sum_y+=q.pitchspeed; gyro_sum_z+=q.yawspeed;
              ++gyro_sum_count;
            } else if(m.msgid==MAVLINK_MSG_ID_LOCAL_POSITION_NED){
              mavlink_local_position_ned_t q{}; mavlink_msg_local_position_ned_decode(&m,&q);
              std::lock_guard<std::mutex> l(mu);
              local.x=q.x; local.y=q.y; local.z=q.z;
              local.vx=q.vx; local.vy=q.vy; local.vz=q.vz;
              local.recv_ns=monoNs(); local.valid=true; ++local_count;
            } else if(m.msgid==MAVLINK_MSG_ID_EKF_STATUS_REPORT){
              mavlink_ekf_status_report_t q{}; mavlink_msg_ekf_status_report_decode(&m,&q);
              std::lock_guard<std::mutex> l(mu);
              ekf.flags=q.flags;
              ekf.velocity_variance=q.velocity_variance;
              ekf.pos_horiz_variance=q.pos_horiz_variance;
              ekf.pos_vert_variance=q.pos_vert_variance;
              ekf.compass_variance=q.compass_variance;
              ekf.terrain_alt_variance=q.terrain_alt_variance;
              ekf.recv_ns=monoNs(); ekf.valid=true; ++ekf_count;
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

  bool startRemoteLog(const std::string& path,double timeout_s=5.0){
    const int64_t deadline=monoNs()+(int64_t)(timeout_s*1e9);
    while(g_running && monoNs()<deadline){
      {
        std::lock_guard<std::mutex> l(mu);
        if(target_sys!=0)break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    uint8_t sys=0,comp=0;
    {
      std::lock_guard<std::mutex> l(mu);
      sys=target_sys; comp=target_comp;
      if(!sys)return false;
      remote_ofs.open(path,std::ios::binary|std::ios::trunc);
      if(!remote_ofs)return false;
      remote_path=path;
      remote_pending.clear(); remote_expected=0;
      remote_blocks_rx=remote_blocks_written=remote_duplicates=0;
      remote_active=true;
    }
    sendRemoteStatus(fd,sys,comp,MAV_REMOTE_LOG_DATA_BLOCK_START,MAV_REMOTE_LOG_DATA_BLOCK_ACK);
    return true;
  }

  void stopRemoteLog(){
    uint8_t sys=0,comp=0;
    {
      std::lock_guard<std::mutex> l(mu);
      if(!remote_active)return;
      sys=target_sys; comp=target_comp;
      remote_active=false;
    }
    if(sys)sendRemoteStatus(fd,sys,comp,MAV_REMOTE_LOG_DATA_BLOCK_STOP,MAV_REMOTE_LOG_DATA_BLOCK_ACK);
    std::lock_guard<std::mutex> l(mu);
    remote_ofs.flush();
    remote_ofs.close();
  }

  void remoteStats(uint64_t* rx,uint64_t* written,uint64_t* dup,size_t* pending){
    std::lock_guard<std::mutex> l(mu);
    if(rx)*rx=remote_blocks_rx;
    if(written)*written=remote_blocks_written;
    if(dup)*dup=remote_duplicates;
    if(pending)*pending=remote_pending.size();
  }

  bool latestArm(bool* out,double* age_ms=nullptr){
    std::lock_guard<std::mutex> l(mu);
    if(!heartbeat_valid)return false;
    *out=armed;
    if(age_ms)*age_ms=(monoNs()-heartbeat_recv_ns)*1e-6;
    return true;
  }

  bool consumeGyroAverage(FlowFcGyro* out,double* age_ms,uint64_t* sample_count=nullptr){
    std::lock_guard<std::mutex> l(mu);
    if(!gyro.valid)return false;
    *out=gyro;
    if(gyro_sum_count>0){
      out->x=gyro_sum_x/gyro_sum_count;
      out->y=gyro_sum_y/gyro_sum_count;
      out->z=gyro_sum_z/gyro_sum_count;
      if(sample_count)*sample_count=gyro_sum_count;
      gyro_sum_x=gyro_sum_y=gyro_sum_z=0.0;
      gyro_sum_count=0;
    } else {
      if(sample_count)*sample_count=0;
    }
    if(age_ms)*age_ms=(monoNs()-gyro.recv_ns)*1e-6;
    return true;
  }

  bool latestEkf(FlowEkfStatus* out,double* age_ms,uint64_t* count=nullptr){
    std::lock_guard<std::mutex> l(mu);
    if(count)*count=ekf_count;
    if(!ekf.valid)return false;
    *out=ekf;
    if(age_ms)*age_ms=(monoNs()-ekf.recv_ns)*1e-6;
    return true;
  }

  void stop(){
    if(remote_active)stopRemoteLog();
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
    -1.0f,              // range independently through DISTANCE_SENSOR
    rate_x,rate_y);
  return GroundMotionMavlinkPublisher::writeMessage(fd,msg);
}

struct FeatureRoi {
  double x0=0.20;
  double y0=0.20;
  double x1=0.80;
  double y1=0.80;
};

FeatureRoi g_feature_roi{};

struct FlowStep {
  bool valid=false;
  int features=0,tracked=0,inliers=0;
  double inlier_ratio=0;
  double du_norm=0,dv_norm=0;
  double du_px=0,dv_px=0;
  double yaw_rate_cam_z=0; // fitted optical-axis rotation, rad/s, removed before MAVLink
  double flow_cam_x=0,flow_cam_y=0;
  double flow_body_x=0,flow_body_y=0;

  // 3x3 spatial diagnostics inside the configured feature ROI.
  // Each cell stores median inlier flow transformed to body FRD.
  std::array<int,9> cell_n{};
  std::array<double,9> cell_body_x{};
  std::array<double,9> cell_body_y{};
};

FlowStep estimateRawFlow(const cv::Mat& prev,const cv::Mat& curr,double dt,const CameraCalib& calib){
  FlowStep o;
  if(prev.empty()||curr.empty()||!(dt>0&&dt<0.2))return o;

  cv::Mat feature_mask(prev.size(),CV_8UC1,cv::Scalar(0));
  const int x0=std::clamp((int)std::lround(g_feature_roi.x0*prev.cols),0,prev.cols-1);
  const int y0=std::clamp((int)std::lround(g_feature_roi.y0*prev.rows),0,prev.rows-1);
  const int x1=std::clamp((int)std::lround(g_feature_roi.x1*prev.cols),x0+1,prev.cols);
  const int y1=std::clamp((int)std::lround(g_feature_roi.y1*prev.rows),y0+1,prev.rows);
  feature_mask(cv::Rect(x0,y0,x1-x0,y1-y0)).setTo(255);

  std::vector<cv::Point2f> p0,p1;
  cv::goodFeaturesToTrack(prev,p0,500,0.01,7,feature_mask);
  o.features=(int)p0.size();
  if(p0.size()<30)return o;

  std::vector<uchar> st; std::vector<float> err;
  cv::calcOpticalFlowPyrLK(prev,curr,p0,p1,st,err,{21,21},3);
  std::vector<cv::Point2f> a,b;
  for(size_t i=0;i<p0.size();++i){if(st[i]){a.push_back(p0[i]);b.push_back(p1[i]);}}
  o.tracked=(int)a.size();
  if(a.size()<20)return o;

  cv::Mat mask;
  cv::findHomography(a,b,cv::RANSAC,2.0,mask);
  if(mask.empty())return o;

  std::vector<cv::Point2f> ai,bi;
  for(size_t i=0;i<a.size();++i){if(mask.at<uchar>((int)i)){ai.push_back(a[i]);bi.push_back(b[i]);}}
  o.inliers=(int)ai.size();
  o.inlier_ratio=a.empty()?0.0:(double)ai.size()/a.size();
  if(ai.size()<20)return o;

  std::vector<cv::Point2f> au,bu;
  cv::undistortPoints(ai,au,calib.K,calib.D);
  cv::undistortPoints(bi,bu,calib.K,calib.D);

  std::vector<double> dun,dvn,dup,dvp;
  dun.reserve(ai.size()); dvn.reserve(ai.size()); dup.reserve(ai.size()); dvp.reserve(ai.size());

  std::array<std::vector<double>,9> cell_du,cell_dv;

  for(size_t i=0;i<ai.size();++i){
    const double du=(double)bu[i].x-au[i].x;
    const double dv=(double)bu[i].y-au[i].y;
    dun.push_back(du);
    dvn.push_back(dv);
    dup.push_back((double)bi[i].x-ai[i].x);
    dvp.push_back((double)bi[i].y-ai[i].y);

    const double nx=(ai[i].x/(double)prev.cols-g_feature_roi.x0)/
                    (g_feature_roi.x1-g_feature_roi.x0);
    const double ny=(ai[i].y/(double)prev.rows-g_feature_roi.y0)/
                    (g_feature_roi.y1-g_feature_roi.y0);
    const int cx=std::clamp((int)std::floor(nx*3.0),0,2);
    const int cy=std::clamp((int)std::floor(ny*3.0),0,2);
    const int ci=cy*3+cx;
    cell_du[ci].push_back(du);
    cell_dv[ci].push_back(dv);
  }
  // Separate optical-axis rotation (camera yaw) from image translation.
  //
  // ArduPilot's MAV OpticalFlow backend compensates bodyRate X/Y internally,
  // but bodyRate Z is not part of OpticalFlow_state. Therefore a plain median
  // of du/dv leaks yaw image rotation into horizontal velocity whenever the
  // feature distribution / ROI is not perfectly centro-symmetric.
  //
  // On undistorted normalized coordinates fit:
  //   du = tx - wz*y
  //   dv = ty + wz*x
  // Unknowns: tx, ty, wz*dt.  tx/ty preserve the constant image motion used
  // for roll/pitch + translation; only the optical-axis rotational component
  // is removed before publishing to ArduPilot.
  cv::Mat A((int)ai.size()*2,3,CV_64F);
  cv::Mat bb((int)ai.size()*2,1,CV_64F);
  for(size_t k=0;k<ai.size();++k){
    const double x=(double)au[k].x;
    const double y=(double)au[k].y;
    const double du=(double)bu[k].x-au[k].x;
    const double dv=(double)bu[k].y-au[k].y;
    A.at<double>((int)(2*k),0)=1.0;
    A.at<double>((int)(2*k),1)=0.0;
    A.at<double>((int)(2*k),2)=-y;
    bb.at<double>((int)(2*k),0)=du;
    A.at<double>((int)(2*k+1),0)=0.0;
    A.at<double>((int)(2*k+1),1)=1.0;
    A.at<double>((int)(2*k+1),2)=x;
    bb.at<double>((int)(2*k+1),0)=dv;
  }
  cv::Mat sol;
  const bool fit_ok=cv::solve(A,bb,sol,cv::DECOMP_SVD);
  if(fit_ok && sol.rows==3){
    o.du_norm=sol.at<double>(0,0);
    o.dv_norm=sol.at<double>(1,0);
    o.yaw_rate_cam_z=sol.at<double>(2,0)/dt;
  } else {
    o.du_norm=median(dun);
    o.dv_norm=median(dvn);
    o.yaw_rate_cam_z=0.0;
  }
  o.du_px=median(dup); o.dv_px=median(dvp);

  // OpenCV camera: +X image-right, +Y image-down, +Z optical-forward.
  // Positive RH camera rotation about Cx -> +dv, about Cy -> -du.
  o.flow_cam_x=o.dv_norm/dt;
  o.flow_cam_y=-o.du_norm/dt;

  // camera -> body FLU из T_BS, затем FLU -> ArduPilot body FRD.
  const cv::Matx33d FLU_TO_FRD(1,0,0, 0,-1,0, 0,0,-1);
  const cv::Matx33d FRD_R_C=FLU_TO_FRD*calib.B_R_C;
  const cv::Vec3d fb=FRD_R_C*cv::Vec3d(o.flow_cam_x,o.flow_cam_y,0.0);
  o.flow_body_x=fb[0]; o.flow_body_y=fb[1];

  for(int ci=0;ci<9;ci++){
    o.cell_n[ci]=(int)cell_du[ci].size();
    if(o.cell_n[ci]>=3){
      // Cell diagnostic uses the same globally fitted yaw removal.
      std::vector<double> cdu_clean,cdv_clean;
      cdu_clean.reserve(cell_du[ci].size());
      cdv_clean.reserve(cell_dv[ci].size());
      for(size_t k=0;k<ai.size();++k){
        const double nx=(ai[k].x/(double)prev.cols-g_feature_roi.x0)/
                        (g_feature_roi.x1-g_feature_roi.x0);
        const double ny=(ai[k].y/(double)prev.rows-g_feature_roi.y0)/
                        (g_feature_roi.y1-g_feature_roi.y0);
        const int cx=std::clamp((int)std::floor(nx*3.0),0,2);
        const int cy=std::clamp((int)std::floor(ny*3.0),0,2);
        if(cy*3+cx!=ci) continue;
        const double x=(double)au[k].x, y=(double)au[k].y;
        const double wzdt=o.yaw_rate_cam_z*dt;
        cdu_clean.push_back(((double)bu[k].x-au[k].x) + wzdt*y);
        cdv_clean.push_back(((double)bu[k].y-au[k].y) - wzdt*x);
      }
      const double cdu=cdu_clean.empty()?median(cell_du[ci]):median(cdu_clean);
      const double cdv=cdv_clean.empty()?median(cell_dv[ci]):median(cdv_clean);
      const double cfx=cdv/dt;
      const double cfy=-cdu/dt;
      const cv::Vec3d cfb=FRD_R_C*cv::Vec3d(cfx,cfy,0.0);
      o.cell_body_x[ci]=cfb[0];
      o.cell_body_y[ci]=cfb[1];
    }
  }

  const double mag=std::hypot(o.flow_body_x,o.flow_body_y);
  o.valid=std::isfinite(mag) && mag<4.0;
  return o;
}

std::string ekfFlagsText(uint16_t f){
  std::ostringstream s;
  s<<"att="<<((f&1)?1:0)
   <<" velH="<<((f&2)?1:0)
   <<" velV="<<((f&4)?1:0)
   <<" posRel="<<((f&8)?1:0)
   <<" posAbs="<<((f&16)?1:0)
   <<" posVAbs="<<((f&32)?1:0)
   <<" posVAGL="<<((f&64)?1:0)
   <<" constPos="<<((f&128)?1:0)
   <<" predRel="<<((f&256)?1:0)
   <<" predAbs="<<((f&512)?1:0)
   <<" uninit="<<((f&1024)?1:0);
  return s.str();
}

} // namespace

#ifndef JTZERO_OPTFLOW_LIBRARY
int main(int argc,char** argv){
  if(argc<7){
    std::cerr<<"Использование: "<<argv[0]
             <<" <camera> <luna> <fc> <csv> <camera_yaml> <focal_scale>\n";
    return 2;
  }

  const std::string camdev=argv[1], lunadev=argv[2], fcdev=argv[3];
  const std::string csvpath=argv[4], yaml=argv[5];
  const double focal_scale=std::stod(argv[6]);
  bool guided=false;
  bool continuous_guided=false;
  int continuous_legs=1;
  double guided_target_mm=175.0;
  bool require_armed=false;
  bool nominal_target_only=false;
  bool return_gui=false;
  bool rotation_gui=false;
  bool return_manual_target=false;
  double diag_camera_z_m=std::numeric_limits<double>::quiet_NaN();
  double diag_range_z_m=std::numeric_limits<double>::quiet_NaN();
  double bench_height_override=0.0;
  double bench_true_camera_height=0.0;
  double pre_static_sec=5.0;
  double post_static_sec=5.0;
  std::string remote_log_path;
  for(int i=7;i<argc;i++){
    const std::string a=argv[i];
    if(a=="--guided-175"){ guided=true; guided_target_mm=175.0; }
    else if(a=="--guided-mm" && i+1<argc){ guided=true; guided_target_mm=std::stod(argv[++i]); }
    else if(a=="--continuous-legs" && i+1<argc){
      guided=true; continuous_guided=true; continuous_legs=std::stoi(argv[++i]);
    }
    else if(a=="--require-armed") require_armed=true;
    else if(a=="--nominal-target") nominal_target_only=true;
    else if(a=="--return-gui") return_gui=true;
    else if(a=="--rotation-gui") rotation_gui=true;
    else if(a=="--return-manual-target") return_manual_target=true;
    else if(a=="--diag-camera-z-m" && i+1<argc) diag_camera_z_m=std::stod(argv[++i]);
    else if(a=="--diag-range-z-m" && i+1<argc) diag_range_z_m=std::stod(argv[++i]);
    else if(a=="--bench-height" && i+1<argc) bench_height_override=std::stod(argv[++i]);
    else if(a=="--bench-true-camera-height" && i+1<argc) bench_true_camera_height=std::stod(argv[++i]);
    else if(a=="--remote-log" && i+1<argc) remote_log_path=argv[++i];
    else if(a=="--pre-static-sec" && i+1<argc) pre_static_sec=std::stod(argv[++i]);
    else if(a=="--post-static-sec" && i+1<argc) post_static_sec=std::stod(argv[++i]);
    else if(a=="--feature-roi" && i+4<argc){
      g_feature_roi.x0=std::stod(argv[++i]);
      g_feature_roi.y0=std::stod(argv[++i]);
      g_feature_roi.x1=std::stod(argv[++i]);
      g_feature_roi.y1=std::stod(argv[++i]);
    }
  }
  if(continuous_guided && (continuous_legs<2 || continuous_legs>30)){
    std::cerr<<"ОШИБКА: --continuous-legs разрешён только 2..30\n";
    return 2;
  }
  if(guided && !(guided_target_mm>=50.0 && guided_target_mm<=1000.0)){
    std::cerr<<"ОШИБКА: --guided-mm разрешён только 50..1000 мм для стенда\n";
    return 2;
  }
  if(bench_height_override!=0.0 && !(bench_height_override>=0.20 && bench_height_override<=2.0)){
    std::cerr<<"ОШИБКА: --bench-height разрешён только 0.20..2.0 м для bench-диагностики\n";
    return 2;
  }
  if(bench_true_camera_height!=0.0 && !(bench_true_camera_height>=0.05 && bench_true_camera_height<=2.0)){
    std::cerr<<"ОШИБКА: --bench-true-camera-height разрешён только 0.05..2.0 м\n";
    return 2;
  }
  if(bench_true_camera_height>0.0 && bench_height_override<=0.0){
    std::cerr<<"ОШИБКА: --bench-true-camera-height требует --bench-height\n";
    return 2;
  }
  if(!(pre_static_sec>=1.0&&pre_static_sec<=30.0) || !(post_static_sec>=1.0&&post_static_sec<=30.0)){
    std::cerr<<"ОШИБКА: --pre-static-sec/--post-static-sec разрешены 1..30 с\n";
    return 2;
  }
  if(!(focal_scale>0.5&&focal_scale<2.0)){
    std::cerr<<"ОШИБКА: focal_scale вне разумного диапазона 0.5..2.0\n";
    return 2;
  }
  if(!(g_feature_roi.x0>=0.0 && g_feature_roi.y0>=0.0 &&
       g_feature_roi.x1<=1.0 && g_feature_roi.y1<=1.0 &&
       g_feature_roi.x1-g_feature_roi.x0>=0.20 &&
       g_feature_roi.y1-g_feature_roi.y0>=0.20)){
    std::cerr<<"ОШИБКА: --feature-roi должен быть x0 y0 x1 y1 в 0..1 и иметь размер >=0.20\n";
    return 2;
  }

  try{
    CameraCalib calib=loadCameraCalib(yaml);
    calib.fx*=focal_scale; calib.fy*=focal_scale;
    calib.K=(cv::Mat_<double>(3,3)<<calib.fx,0,calib.cx,0,calib.fy,calib.cy,0,0,1);

    Camera cam; cam.openDev(camdev);
    LunaReader luna; luna.start(lunadev);
    FlowFc fc; fc.start(fcdev);
    if(!remote_log_path.empty()){
      if(fc.startRemoteLog(remote_log_path)){
        std::cerr<<"REMOTE DATAFLASH: запись запущена -> "<<remote_log_path<<"\n";
      } else {
        std::cerr<<"ПРЕДУПРЕЖДЕНИЕ: не удалось запустить REMOTE DATAFLASH logging.\n"
                 <<"Проверь LOG_BACKEND_TYPE=2 и reboot FC. Тест продолжится без BIN.\n";
      }
    }
    GroundMotionMavlinkPublisher range_pub;
    range_pub.system_id=FlowFc::self_sys;
    range_pub.component_id=FlowFc::self_comp;

    std::ofstream csv(csvpath,std::ios::trunc);
    csv<<"mono_ns,camera_ts_ns,flow_send_ns,frame_pipeline_latency_ms,camera_queue_dropped,camera_queue_dropped_total,frame,guide_leg,guide_stage,valid,dt_s,features,tracked,inliers,inlier_ratio,du_px,dv_px,du_norm,dv_norm,yaw_rate_cam_z,flow_cam_x,flow_cam_y,flow_body_x,flow_body_y,quality,luna_m,luna_age_ms,range_to_fc_m,flow_send_x,flow_send_y,flow_sent,range_sent,fc_armed,ekf_local_valid,ekf_x_ned,ekf_y_ned,ekf_z_ned,ekf_vx_ned,ekf_vy_ned,ekf_vz_ned,ekf_age_ms,ekf_count,ekf_status_valid,ekf_flags,ekf_status_age_ms,ekf_status_count,ekf_vel_var,ekf_pos_h_var,ekf_pos_v_var,ekf_compass_var,ekf_terrain_var,return_event,fc_roll,fc_pitch,fc_yaw,fc_gyro_x,fc_gyro_y,fc_gyro_z,fc_gyro_age_ms,fc_gyro_samples,c0_n,c0_bx,c0_by,c1_n,c1_bx,c1_by,c2_n,c2_bx,c2_by,c3_n,c3_bx,c3_by,c4_n,c4_bx,c4_by,c5_n,c5_bx,c5_by,c6_n,c6_bx,c6_by,c7_n,c7_bx,c7_by,c8_n,c8_bx,c8_by\n";

    cv::setNumThreads(1);
    std::signal(SIGINT,onSignal); std::signal(SIGTERM,onSignal);

    cv::Mat prev; int64_t prev_ts=0; uint64_t frame=0;
    uint64_t flow_sent_total=0,flow_invalid_total=0,range_sent_total=0;
    uint64_t camera_queue_dropped_total=0, stale_flow_rejected_total=0;
    int64_t last_range_send_ns=0;
    constexpr double kMaxFlowPipelineAgeMs=80.0;

    // Flight-only readiness gate. It does not arm or inhibit ArduPilot; it is an
    // explicit operator indication that the same signals used by the EKF are healthy.
    const bool flight_ready_gate=!guided;
    bool flight_ready=false;
    int64_t flight_ready_since_ns=0;
    int64_t flight_gate_begin_ns=monoNs();
    int64_t last_not_ready_print_ns=0;
    constexpr double kReadyStableSec=3.0;
    constexpr double kReadyTimeoutSec=20.0;
    constexpr double kReadyMinRangeM=0.10;
    constexpr double kReadyMaxRangeM=10.0;
    constexpr double kReadyMaxSpeedMps=0.03;

    bool return_target_set=false;
    double return_target_n=0.0,return_target_e=0.0;
    double return_view_halfspan_m=0.50;
    std::deque<cv::Point2d> return_trail;

    // Return-to-target forensic state. RAW is accumulated in native body-flow
    // measurement coordinates using the camera height above the observed plane.
    // It is intentionally kept independent from EKF position.
    // Native LOS integral is kept for continuity with earlier diagnostics.
    double return_raw_x=0.0,return_raw_y=0.0;
    // AP-model translational displacement, first in body FRD, then rotated to NED.
    double return_body_dx=0.0,return_body_dy=0.0;
    double return_ned_n=0.0,return_ned_e=0.0;
    double return_yaw0=0.0;
    bool return_yaw0_set=false;
    bool return_b_marked=false;
    bool return_home_marked=false;
    double return_b_n=0.0,return_b_e=0.0;
    double return_b_raw_x=0.0,return_b_raw_y=0.0,return_b_yaw=0.0;
    double return_b_body_dx=0.0,return_b_body_dy=0.0;
    double return_b_ned_n=0.0,return_b_ned_e=0.0;
    int pending_return_event=0; // 1=A/target, 2=B/turn, 3=H/physical-home mark
    const std::string return_window_name="JT-Zero — Возврат в исходную точку";
    const std::string rotation_window_name="JT-Zero — Диагностика вращения";
    if(return_gui || rotation_gui) initGuiFont();
    if(return_gui){
      cv::namedWindow(return_window_name,cv::WINDOW_NORMAL);
      cv::resizeWindow(return_window_name,1500,900);
      std::cerr<<"GUI ВОЗВРАТА: "<<(return_manual_target?"точка A задаётся вручную после подъёма":"точка A задаётся автоматически после готовности")<<".\n"
               <<"Клавиши: SPACE=A/домой, B=дальняя точка, H=физический возврат, C=очистить хвост, Q/ESC=выход.\n";
    }
    if(rotation_gui){
      cv::namedWindow(rotation_window_name,cv::WINDOW_NORMAL);
      cv::resizeWindow(rotation_window_name,1500,900);
      std::cerr<<"GUI ВРАЩЕНИЯ: отдельная диагностика roll/pitch/yaw без A/B/H.\n"
               <<"Клавиши: Q/ESC=выход.\n";
    }

    std::atomic<int> guide_stage{0}; // 0=pre-static, 1=move, 2=post-static, 3=wait-next, 4=done
    std::atomic<int> guide_leg{0};
    std::atomic<bool> arm_lost{false};
    FlowFcLocal guide_start{}, guide_end{};
    std::thread guide_thread;
    if(guided){
      guide_thread=std::thread([&]{
        std::this_thread::sleep_for(std::chrono::milliseconds(750));
        bool arm=false; double arm_age=1e9;
        const bool have_arm=fc.latestArm(&arm,&arm_age) && arm_age<2500.0;

        std::cerr<<"\n======================================================================\n"
                 <<(continuous_guided?"CONTINUOUS RECIPROCAL TEST":"GUIDED TEST")
                 <<" — НОМИНАЛЬНЫЙ СДВИГ "<<guided_target_mm<<" мм\n"
                 <<"======================================================================\n"
                 <<"Проходов: "<<(continuous_guided?continuous_legs:1)<<"\n"
                 <<"Один процесс камеры/MAVLink/DataFlash на всю серию.\n"
                 <<"Фактическое расстояние измеряется после каждого прохода.\n"
                 <<"======================================================================\n"
                 <<"ARM STATE: "<<(have_arm?(arm?"ARMED":"DISARMED"):"NO_DATA")<<"\n";
        if(require_armed && (!have_arm || !arm)){
          std::cerr<<"ОШИБКА: этот тест требует ARMED.\n";
          g_running=false; return;
        }

        auto wait_height=[&](double stable_sec)->bool{
          if(bench_height_override<=0.0) return true;
          constexpr double kHgtTolM=0.035;
          constexpr double kTimeoutSec=20.0;
          std::cerr<<"\n>>> СИНХРОНИЗАЦИЯ ВЫСОТЫ. НЕ ДВИГАТЬ.\n"
                   <<">>> Ждём LOCAL Z около -"<<bench_height_override
                   <<" м (±"<<kHgtTolM<<" м) непрерывно "<<stable_sec<<" с.\n";
          const int64_t sync_begin=monoNs();
          int64_t stable_begin=0;
          double last_z=0.0,last_age=1e9;
          while(g_running){
            FlowFcLocal q{}; double qage=1e9; uint64_t qcount=0;
            const bool qok=fc.latestLocal(&q,&qage,&qcount) && qage<500.0;
            if(qok){
              last_z=q.z; last_age=qage;
              const bool in_band=std::abs((-double(q.z))-bench_height_override)<=kHgtTolM;
              if(in_band){
                if(stable_begin==0) stable_begin=monoNs();
                if((monoNs()-stable_begin)*1e-9>=stable_sec){
                  std::cerr<<">>> ВЫСОТА СТАБИЛЬНА: LOCAL Z="<<q.z
                           <<" м, inferred HAGL="<<(-q.z)<<" м.\n";
                  return true;
                }
              } else {
                stable_begin=0;
              }
            }
            if((monoNs()-sync_begin)*1e-9>=kTimeoutSec){
              std::cerr<<"\nОШИБКА: EKF height не сошёлся за "<<kTimeoutSec
                       <<" с. Последний LOCAL Z="<<last_z<<" м age="<<last_age<<" ms.\n";
              return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
          }
          return false;
        };

        const int legs=continuous_guided?continuous_legs:1;
        for(int leg=1; leg<=legs && g_running; ++leg){
          guide_leg=leg;
          guide_stage=0;
          if(!wait_height(leg==1?2.0:1.0)){ g_running=false; return; }

          std::cerr<<"\n======================================================================\n"
                   <<"LEG "<<leg<<" / "<<legs<<"\n"
                   <<"======================================================================\n"
                   <<"СТАТИКА "<<pre_static_sec<<" секунд. НЕ ДВИГАТЬ.\n";
          std::this_thread::sleep_for(std::chrono::milliseconds((int)std::llround(pre_static_sec*1000.0)));

          double age=0; uint64_t count=0;
          if(!fc.latestLocal(&guide_start,&age,&count) || age>500){
            std::cerr<<"ОШИБКА GUIDE: нет свежего LOCAL_POSITION_NED перед движением.\n";
            g_running=false; return;
          }

          guide_stage=1;
          std::cerr<<"\n>>> LEG "<<leg<<" ДВИГАЙТЕ\n"
                   <<">>> Сдвиньте аппарат строго по столу. После полной остановки нажмите Enter.\n";
          std::string line; std::getline(std::cin,line);

          guide_stage=2;
          std::cerr<<"\n>>> LEG "<<leg<<" СТОП. НЕ ТРОГАТЬ аппарат "<<post_static_sec<<" секунд.\n";
          std::this_thread::sleep_for(std::chrono::milliseconds((int)std::llround(post_static_sec*1000.0)));

          if(!fc.latestLocal(&guide_end,&age,&count) || age>500){
            std::cerr<<"ОШИБКА GUIDE: нет свежего LOCAL_POSITION_NED после движения.\n";
            g_running=false; return;
          }
          if(require_armed && arm_lost.load()){
            std::cerr<<"ARMed-test прерван из-за DISARM.\n";
            g_running=false; return;
          }

          const double dn=guide_end.x-guide_start.x, de=guide_end.y-guide_start.y;
          const double dist=std::hypot(dn,de);
          std::cerr<<"\n======================================================================\n"
                   <<"CONTINUOUS LEG "<<leg<<" RESULT\n"
                   <<"START N/E = ("<<guide_start.x<<", "<<guide_start.y<<") m\n"
                   <<"END   N/E = ("<<guide_end.x<<", "<<guide_end.y<<") m\n"
                   <<"DELTA N/E = ("<<dn<<", "<<de<<") m\n"
                   <<"EKF horizontal displacement = "<<dist*1000.0<<" mm\n"
                   <<"Nominal guided target = "<<guided_target_mm
                   <<" mm (ТОЛЬКО ИНСТРУКЦИЯ; физический эталон вводится в GUI)\n"
                   <<"======================================================================\n";

          guide_stage=3;
          std::cerr<<"\n>>> LEG "<<leg<<" COMPLETE. Введите физическое расстояние в GUI.\n";
          if(leg<legs){
            std::cerr<<">>> После сохранения GUI продолжит следующий проход.\n";
            std::getline(std::cin,line);
          }
        }

        guide_stage=4;
        guide_leg=legs;
        std::cerr<<"\n>>> CONTINUOUS SERIES COMPLETE\n";
        g_running=false;
      });
    }

    std::cerr<<"JT-ZERO OPTICAL FLOW MAVLINK MVP v2\n"
             <<"camera="<<camdev<<"\n"
             <<"fx/fy effective="<<calib.fx<<" / "<<calib.fy
             <<" (focal_scale="<<focal_scale<<")\n"
             <<"ВАЖНО: publisher выдаёт body-FRD flow; ожидается FLOW_ORIENT_YAW=0, FLOW_OPTIONS=0\n"
             <<"DIAG: запрошен EKF_STATUS_REPORT 5 Hz; LOCAL_POSITION_NED 20 Hz; ATTITUDE 100 Hz\n";
    if(return_gui && std::isfinite(diag_camera_z_m) && std::isfinite(diag_range_z_m)){
      std::cerr<<"RETURN GUI geometry: camera_z="<<diag_camera_z_m
               <<" m range_z="<<diag_range_z_m
               <<" m, camera-range dz="<<(diag_camera_z_m-diag_range_z_m)<<" m\n";
    }
    if(bench_height_override>0.0){
      std::cerr<<"BENCH HEIGHT OVERRIDE: FC получает "<<bench_height_override
               <<" м вместо реального TF-Luna.\n";
      if(bench_true_camera_height>0.0){
        std::cerr<<"FIXED TRUE CAMERA HEIGHT: "<<bench_true_camera_height
                 <<" м; TF-Luna НЕ используется для метрического масштаба flow.\n";
      } else {
        std::cerr<<"Flow-rate масштабируется real/fake по TF-Luna.\n";
      }
      std::cerr<<"ЭТО ТОЛЬКО СТЕНДОВАЯ ДИАГНОСТИКА, НЕ FLIGHT-РЕЖИМ.\n";
    }

    // Переводим AP_OpticalFlow_MAV в high-precision flow_rate mode.
    // quality=0: это не валидное aiding measurement.
    sendOpticalFlow(fc.fd,(uint64_t)(monoNs()/1000),1.0e-6f,0.0f,0);

    while(g_running){
      pollfd p{cam.fd,POLLIN,0};
      const int pr=poll(&p,1,20);
      if(pr<0){if(errno==EINTR)continue;fail("camera poll");}
      if(pr<=0)continue;

      // КРИТИЧЕСКИ: обработка KLT медленнее capture-rate камеры. Если
      // обрабатывать каждый queued MJPEG кадр, возникает постоянный backlog
      // (~0.4-0.8 с в плохом прогоне), а ArduPilot компенсирует такой старый
      // flow СВЕЖИМ gyro. Поэтому всегда выкидываем промежуточные queued
      // кадры и обрабатываем только самый свежий доступный кадр.
      std::vector<uint8_t> latest_jpeg;
      int64_t ts=0;
      uint64_t camera_queue_dropped=0;
      while(g_running){
        v4l2_buffer b{}; b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory=V4L2_MEMORY_MMAP;
        if(xioctl(cam.fd,VIDIOC_DQBUF,&b)<0){
          if(errno==EAGAIN)break;
          fail("VIDIOC_DQBUF");
        }
        const int64_t bts=(int64_t)b.timestamp.tv_sec*1000000000LL+(int64_t)b.timestamp.tv_usec*1000LL;
        if(!latest_jpeg.empty()) ++camera_queue_dropped;
        const uint8_t* pjpeg=reinterpret_cast<const uint8_t*>(cam.bufs[b.index].p);
        latest_jpeg.assign(pjpeg,pjpeg+b.bytesused);
        ts=bts;
        if(xioctl(cam.fd,VIDIOC_QBUF,&b)<0)fail("VIDIOC_QBUF");
      }
      if(latest_jpeg.empty()) continue;
      camera_queue_dropped_total += camera_queue_dropped;

      const int64_t now=monoNs();
      cv::Mat raw(1,(int)latest_jpeg.size(),CV_8UC1,latest_jpeg.data());
      cv::Mat gray=cv::imdecode(raw,cv::IMREAD_GRAYSCALE);
      if(gray.empty()) continue;
      ++frame;

        double lm=0; int strength=0; int64_t lns=0;
        const bool hl=luna.latest(&lm,&strength,&lns);
        const double lage=hl?(now-lns)*1e-6:1e9;
        bool range_sent=false;
        const double range_to_fc=(bench_height_override>0.0)?bench_height_override:lm;
        if(hl&&lage<200&&(last_range_send_ns==0||now-last_range_send_ns>=50000000LL)){
          range_sent=range_pub.sendDistanceSensor(fc.fd,(uint32_t)(now/1000000LL),range_to_fc);
          last_range_send_ns=now;
          if(range_sent)++range_sent_total;
        }

        const double dt=prev_ts?(ts-prev_ts)*1e-9:0.0;
        FlowStep s;
        if(!prev.empty())s=estimateRawFlow(prev,gray,dt,calib);

        bool flow_sent=false; uint8_t quality=0;
        double flow_send_x=s.flow_body_x, flow_send_y=s.flow_body_y;
        if(s.valid && bench_height_override>0.0){
          if(bench_true_camera_height>0.0){
            // Bench-only fixed-height mode. Do not trust TF-Luna when it is below
            // its reliable minimum range. ArduPilot receives a synthetic range,
            // while angular flow is scaled so metric horizontal velocity remains
            // equal to raw_flow * measured true camera height.
            double fake_camera_height=bench_height_override;
            if(std::isfinite(diag_camera_z_m) && std::isfinite(diag_range_z_m)){
              fake_camera_height=bench_height_override-(diag_camera_z_m-diag_range_z_m);
            }
            if(fake_camera_height>0.02){
              const double k=bench_true_camera_height/fake_camera_height;
              flow_send_x*=k;
              flow_send_y*=k;
            }
          } else if(hl && lm>0.05){
            const double k=lm/bench_height_override;
            flow_send_x*=k;
            flow_send_y*=k;
          }
        }
        int64_t flow_send_ns=monoNs();
        const double frame_pipeline_latency_ms =
          (ts>0) ? (flow_send_ns-ts)*1e-6 : -1.0;
        const bool flow_fresh = frame_pipeline_latency_ms>=0.0 &&
                                frame_pipeline_latency_ms<=kMaxFlowPipelineAgeMs;
        if(s.valid && flow_fresh){
          quality=255;
          // AP_OpticalFlow_MAV currently timestamps measurement by RECEIVE time,
          // not packet.time_usec, so low pipeline latency is mandatory.
          flow_sent=sendOpticalFlow(fc.fd,(uint64_t)(flow_send_ns/1000),
            (float)flow_send_x,(float)flow_send_y,quality);
          if(flow_sent)++flow_sent_total;
        } else {
          if(!prev.empty() && !s.valid) ++flow_invalid_total;
          if(s.valid && !flow_fresh) ++stale_flow_rejected_total;
        }

        FlowFcLocal ep{}; double eage=1e9; uint64_t ec=0;
        const bool eok=fc.latestLocal(&ep,&eage,&ec);
        const bool efresh=eok&&eage<500.0;
        FlowEkfStatus es{}; double esage=1e9; uint64_t esc=0;
        const bool esok=fc.latestEkf(&es,&esage,&esc);
        const bool esfresh=esok&&esage<1000.0;

        FlowFcGyro fg{}; double fg_age=1e9; uint64_t fg_samples=0;
        const bool fg_ok=fc.consumeGyroAverage(&fg,&fg_age,&fg_samples);

        bool arm_now=false; double arm_age_now=1e9;
        const bool arm_ok=fc.latestArm(&arm_now,&arm_age_now) && arm_age_now<2500.0;
        if(require_armed && arm_ok && !arm_now && guide_stage.load()<3){
          if(!arm_lost.exchange(true)){
            std::cerr<<"\nОШИБКА: FC ПЕРЕШЁЛ В DISARMED ВО ВРЕМЯ ARMED-ТЕСТА.\n"
                     <<"Тест остановлен; результат движения недействителен.\n";
          }
          g_running=false;
        }

        csv<<now<<','<<ts<<','<<flow_send_ns<<','<<frame_pipeline_latency_ms<<','
           <<camera_queue_dropped<<','<<camera_queue_dropped_total<<','
           <<frame<<','<<guide_leg.load()<<','<<guide_stage.load()<<','<<(s.valid?1:0)<<','<<dt<<','
           <<s.features<<','<<s.tracked<<','<<s.inliers<<','<<s.inlier_ratio<<','
           <<s.du_px<<','<<s.dv_px<<','<<s.du_norm<<','<<s.dv_norm<<','<<s.yaw_rate_cam_z<<','
           <<s.flow_cam_x<<','<<s.flow_cam_y<<','<<s.flow_body_x<<','<<s.flow_body_y<<','
           <<(int)quality<<','<<lm<<','<<lage<<','<<range_to_fc<<','<<flow_send_x<<','<<flow_send_y<<','<<(flow_sent?1:0)<<','<<(range_sent?1:0)<<','
           <<(arm_ok?(arm_now?1:0):-1)<<','
           <<(efresh?1:0)<<','<<ep.x<<','<<ep.y<<','<<ep.z<<','<<ep.vx<<','<<ep.vy<<','<<ep.vz<<','<<eage<<','<<ec<<','
           <<(esfresh?1:0)<<','<<es.flags<<','<<esage<<','<<esc<<','
           <<es.velocity_variance<<','<<es.pos_horiz_variance<<','<<es.pos_vert_variance<<','<<es.compass_variance<<','<<es.terrain_alt_variance<<','
           <<pending_return_event<<','
           <<(fg_ok?fg.roll:0.0)<<','<<(fg_ok?fg.pitch:0.0)<<','<<(fg_ok?fg.yaw:0.0)<<','
           <<(fg_ok?fg.x:0.0)<<','<<(fg_ok?fg.y:0.0)<<','<<(fg_ok?fg.z:0.0)<<','<<(fg_ok?fg_age:-1.0)<<','<<fg_samples;
        for(int ci=0;ci<9;ci++){
          csv<<','<<s.cell_n[ci]<<','<<s.cell_body_x[ci]<<','<<s.cell_body_y[ci];
        }
        csv<<'\n';
        pending_return_event=0;

        if(flight_ready_gate && !flight_ready){
          const double speed_h=efresh?std::hypot((double)ep.vx,(double)ep.vy):1e9;
          const double ready_range=(bench_height_override>0.0)?bench_height_override:lm;
          const bool luna_ok=(bench_true_camera_height>0.0 && bench_height_override>0.0)
            ? (ready_range>=kReadyMinRangeM && ready_range<=kReadyMaxRangeM)
            : (hl && lage>=-2.0 && lage<100.0 &&
               ready_range>=kReadyMinRangeM && ready_range<=kReadyMaxRangeM);
          const bool flow_ok=s.valid && flow_sent && s.inliers>=30;
          const bool ekf_ok=esfresh &&
                            (es.flags & EKF_ATTITUDE) &&
                            (es.flags & EKF_VELOCITY_HORIZ) &&
                            (es.flags & EKF_POS_HORIZ_REL) &&
                            !(es.flags & EKF_UNINITIALIZED);
          const bool local_ok=efresh && speed_h<=kReadyMaxSpeedMps;
          const bool ready_now=luna_ok && flow_ok && ekf_ok && local_ok;
          if(ready_now){
            if(flight_ready_since_ns==0) flight_ready_since_ns=now;
            if((now-flight_ready_since_ns)*1e-9>=kReadyStableSec){
              flight_ready=true;
              std::cerr<<"\n======================================================================\n"
                       <<"СИСТЕМА ГОТОВА\n"
                       <<"range="<<((bench_height_override>0.0)?bench_height_override:lm)
                       <<" m, flow valid, EKF velH/posRel valid, |vH|="
                       <<speed_h<<" m/s\n"
                       <<"Состояние было непрерывно стабильным "<<kReadyStableSec<<" с.\n"
                       <<"======================================================================\n";
            }
          }else{
            flight_ready_since_ns=0;
            if(last_not_ready_print_ns==0 || now-last_not_ready_print_ns>1000000000LL){
              std::cerr<<"\nНЕ ГОТОВО:"
                       <<" luna="<<(luna_ok?"OK":"NO")
                       <<" flow="<<(flow_ok?"OK":"NO")
                       <<" ekf="<<(ekf_ok?"OK":"NO")
                       <<" local="<<(local_ok?"OK":"NO")
                       <<" range="<<((bench_height_override>0.0)?bench_height_override:(hl?lm:-1.0))
                       <<" vH="<<(efresh?speed_h:-1.0)<<"\n";
              last_not_ready_print_ns=now;
            }
          }
          if((now-flight_gate_begin_ns)*1e-9>kReadyTimeoutSec && !flight_ready){
            std::cerr<<"\nПРЕДУПРЕЖДЕНИЕ: СИСТЕМА ГОТОВА не достигнут за "
                     <<kReadyTimeoutSec<<" с. Publisher продолжает работать; взлёт не выполнять.\n";
            flight_gate_begin_ns=now;
          }
        }

        if(return_gui && return_target_set && s.valid && flow_sent && dt>0.0 && dt<0.2){
          double hcam=0.0;
          if(bench_true_camera_height>0.0){
            hcam=bench_true_camera_height;
          } else if(hl){
            hcam=lm;
            if(std::isfinite(diag_camera_z_m) && std::isfinite(diag_range_z_m)){
              hcam=lm-(diag_camera_z_m-diag_range_z_m);
            }
          }
          if(hcam>0.02){
            double native_fx=flow_send_x, native_fy=flow_send_y;
            if(bench_true_camera_height>0.0 && bench_height_override>0.0){
              double fake_camera_height=bench_height_override;
              if(std::isfinite(diag_camera_z_m) && std::isfinite(diag_range_z_m)){
                fake_camera_height=bench_height_override-(diag_camera_z_m-diag_range_z_m);
              }
              const double undo=(bench_true_camera_height>0.0)?fake_camera_height/bench_true_camera_height:1.0;
              native_fx*=undo; native_fy*=undo;
            }

            // Legacy/native LOS integral.
            return_raw_x += native_fx*hcam*dt;
            return_raw_y += native_fy*hcam*dt;

            // Mirror ArduPilot EKF3 optical-flow conventions:
            //   internal flowRadXY = -rawFlowRates
            //   flowRadXYcomp = flowRadXY + bodyRateXY
            //   losPred.x = v_body_y/range
            //   losPred.y = -v_body_x/range
            // Therefore:
            //   v_body_x = -flowComp.y * range
            //   v_body_y =  flowComp.x * range
            // Use FC ATTITUDE/gyro from the same camera interval average.
            if(fg_ok){
              const double comp_x=-native_fx + fg.x;
              const double comp_y=-native_fy + fg.y;
              const double dbx=(-comp_y)*hcam*dt;
              const double dby=( comp_x)*hcam*dt;
              return_body_dx += dbx;
              return_body_dy += dby;

              // Full 3-2-1 body-FRD -> NED rotation, planar body displacement z=0.
              const double cr=std::cos(fg.roll),  sr=std::sin(fg.roll);
              const double cp=std::cos(fg.pitch), sp=std::sin(fg.pitch);
              const double cy=std::cos(fg.yaw),   sy=std::sin(fg.yaw);
              const double r00=cy*cp;
              const double r01=cy*sp*sr-sy*cr;
              const double r10=sy*cp;
              const double r11=sy*sp*sr+cy*cr;
              return_ned_n += r00*dbx + r01*dby;
              return_ned_e += r10*dbx + r11*dby;
            }
          }
        }

        if(rotation_gui){
          cv::Mat hud(900,1500,CV_8UC3,cv::Scalar(18,18,18));

          const double roll_deg=fg_ok?fg.roll*180.0/M_PI:0.0;
          const double pitch_deg=fg_ok?fg.pitch*180.0/M_PI:0.0;
          const double yaw_deg=fg_ok?fg.yaw*180.0/M_PI:0.0;
          const double gx=fg_ok?fg.x:0.0, gy=fg_ok?fg.y:0.0;
          const double flowx=s.valid?s.flow_body_x:0.0, flowy=s.valid?s.flow_body_y:0.0;
          // ArduPilot MAV optical-flow backend subtracts body rotation internally.
          // For a rotation-only diagnostic, this is the instantaneous uncompensated residual.
          const double rx=flowx-gx;
          const double ry=flowy-gy;
          const double rmag=std::hypot(rx,ry);
          double hcam=hl?lm:0.0;
          if(hl && std::isfinite(diag_camera_z_m) && std::isfinite(diag_range_z_m)){
            hcam=lm-(diag_camera_z_m-diag_range_z_m);
          }
          const double false_speed=(hcam>0.02)?rmag*hcam:0.0;
          const double vh=efresh?std::hypot((double)ep.vx,(double)ep.vy):0.0;

          putGuiText(hud,"JT-ZERO — ДИАГНОСТИКА ВРАЩЕНИЯ",{35,45},0.95,cv::Scalar(240,240,240),2);
          std::string status = flight_ready ? "СИСТЕМА ГОТОВА" : "ЖДИТЕ ГОТОВНОСТИ";
          putGuiText(hud,status,{35,85},0.72,flight_ready?cv::Scalar(0,220,0):cv::Scalar(0,200,255),2);

          cv::rectangle(hud,cv::Rect(30,110,820,160),cv::Scalar(30,30,30),cv::FILLED);
          cv::rectangle(hud,cv::Rect(30,110,820,160),cv::Scalar(100,100,100),1);
          putGuiText(hud,"ПРОТОКОЛ",{50,140},0.68,cv::Scalar(0,255,255),2);
          putGuiText(hud,"1. Дождитесь «СИСТЕМА ГОТОВА».",{50,172},0.52,cv::Scalar(230,230,230),1);
          putGuiText(hud,"2. Коротко наклоните аппарат по КРЕНУ примерно на ±5° и верните.",{50,202},0.52,cv::Scalar(230,230,230),1);
          putGuiText(hud,"3. Затем по ТАНГАЖУ примерно на ±5° и верните.",{50,232},0.52,cv::Scalar(230,230,230),1);
          putGuiText(hud,"4. Не переносите аппарат по XY. После теста нажмите Q.",{50,262},0.52,cv::Scalar(230,230,230),1);

          std::ostringstream a1,a2,a3,a4,a5,a6;
          a1<<std::fixed<<std::setprecision(1)<<"КРЕН / ТАНГАЖ / КУРС: "<<roll_deg<<" / "<<pitch_deg<<" / "<<yaw_deg<<" °";
          a2<<std::fixed<<std::setprecision(3)<<"GYRO X/Y: "<<gx<<" / "<<gy<<" рад/с";
          a3<<std::fixed<<std::setprecision(3)<<"FLOW X/Y: "<<flowx<<" / "<<flowy<<" рад/с";
          a4<<std::fixed<<std::setprecision(3)<<"ОСТАТОК FLOW-GYRO: "<<rx<<" / "<<ry<<" рад/с   |.|="<<rmag;
          a5<<std::fixed<<std::setprecision(3)<<"ЭКВИВАЛЕНТНАЯ ЛОЖНАЯ СКОРОСТЬ: "<<false_speed<<" м/с";
          a6<<std::fixed<<std::setprecision(3)<<"EKF |Vxy|: "<<vh<<" м/с   TF-Luna: "<<(hl?lm:-1.0)<<" м   задержка кадра: "<<frame_pipeline_latency_ms<<" мс";

          putGuiText(hud,a1.str(),{45,330},0.66,cv::Scalar(230,230,230),2);
          putGuiText(hud,a2.str(),{45,375},0.66,cv::Scalar(230,230,230),2);
          putGuiText(hud,a3.str(),{45,420},0.66,cv::Scalar(230,230,230),2);
          putGuiText(hud,a4.str(),{45,465},0.66,rmag<0.08?cv::Scalar(0,220,0):cv::Scalar(0,120,255),2);
          putGuiText(hud,a5.str(),{45,510},0.66,false_speed<0.03?cv::Scalar(0,220,0):cv::Scalar(0,120,255),2);
          putGuiText(hud,a6.str(),{45,555},0.56,cv::Scalar(190,190,190),1);

          // Visual bars for gyro, flow and residual magnitude.
          const double bar_scale=350.0;
          auto draw_bar=[&](int y,double v,cv::Scalar col,const std::string& name){
            putGuiText(hud,name,{45,y-8},0.50,cv::Scalar(190,190,190),1);
            cv::line(hud,{300,y},{760,y},cv::Scalar(80,80,80),2);
            const int x2=std::clamp(530+(int)std::lround(v*bar_scale),300,760);
            cv::line(hud,{530,y},{x2,y},col,8,cv::LINE_AA);
          };
          draw_bar(620,std::hypot(gx,gy),cv::Scalar(255,180,0),"|GYRO XY|");
          draw_bar(675,std::hypot(flowx,flowy),cv::Scalar(0,255,255),"|FLOW|");
          draw_bar(730,rmag,cv::Scalar(0,100,255),"|ОСТАТОК|");

          // Live camera panel.
          cv::Mat cam_bgr,cam_view;
          cv::cvtColor(gray,cam_bgr,cv::COLOR_GRAY2BGR);
          const int cam_w=560;
          const int cam_h=(int)std::lround((double)cam_bgr.rows*cam_w/cam_bgr.cols);
          cv::resize(cam_bgr,cam_view,cv::Size(cam_w,cam_h),0,0,cv::INTER_AREA);
          const int cam_x=910, cam_y=120;
          if(cam_y+cam_h<=hud.rows && cam_x+cam_w<=hud.cols){
            cam_view.copyTo(hud(cv::Rect(cam_x,cam_y,cam_w,cam_h)));
            const int rx0=cam_x+(int)std::lround(g_feature_roi.x0*cam_w);
            const int ry0=cam_y+(int)std::lround(g_feature_roi.y0*cam_h);
            const int rx1=cam_x+(int)std::lround(g_feature_roi.x1*cam_w);
            const int ry1=cam_y+(int)std::lround(g_feature_roi.y1*cam_h);
            cv::rectangle(hud,cv::Point(rx0,ry0),cv::Point(rx1,ry1),cv::Scalar(0,255,255),2,cv::LINE_AA);
            putGuiText(hud,"OV9281 — ЖИВОЕ ВИДЕО",{cam_x,85},0.76,cv::Scalar(240,240,240),2);
          }
          putGuiText(hud,"Q / ESC — ЗАВЕРШИТЬ ТЕСТ",{45,855},0.58,cv::Scalar(180,180,180),1);

          cv::imshow(rotation_window_name,hud);
          const int rkey=cv::waitKey(1)&0xff;
          if(rkey=='q'||rkey=='Q'||rkey==27) g_running=false;
        }

        if(return_gui){
          if(flight_ready && efresh && !return_target_set && !return_manual_target){
            return_target_n=ep.x;
            return_target_e=ep.y;
            return_target_set=true;
            return_trail.clear();
            return_raw_x=return_raw_y=0.0;
            return_body_dx=return_body_dy=0.0;
            return_ned_n=return_ned_e=0.0;
            return_b_marked=false;
            return_home_marked=false;
            if(fg_ok){ return_yaw0=fg.yaw; return_yaw0_set=true; }
            pending_return_event=1;
            std::cerr<<"RETURN GUI TARGET SET: N="<<return_target_n<<" E="<<return_target_e
                     <<" yaw_deg="<<(return_yaw0_set?return_yaw0*180.0/M_PI:0.0)<<"\n";
          }

          // Screen-recording HUD: trajectory stays on the left; the live OV9281
          // image is shown on the right with the exact feature ROI used by KLT.
          cv::Mat hud(900,1500,CV_8UC3,cv::Scalar(20,20,20));
          const cv::Point center(450,450);

          // Крупная русская инструкция оператору — её видно на записи экрана.
          std::string step_title, step_line1, step_line2;
          cv::Scalar step_color(220,220,220);
          if(!flight_ready){
            step_title="ШАГ 1 / 5 — ЖДИТЕ ГОТОВНОСТИ";
            step_line1="Держите аппарат неподвижно. Тест пока не начинайте.";
            step_line2="После ГОТОВНОСТИ поднимите аппарат на удобную высоту.";
            step_color=cv::Scalar(0,200,255);
          } else if(!return_target_set){
            step_title="ШАГ 2 / 5 — ЗАДАЙТЕ ФИЗИЧЕСКУЮ ТОЧКУ A";
            step_line1="Поднимите аппарат и удерживайте его неподвижно 2–3 секунды.";
            step_line2="Нажмите SPACE. Эта позиция станет точкой A / ДОМОЙ.";
            step_color=cv::Scalar(0,255,255);
          } else if(!return_b_marked){
            step_title="ШАГ 3 / 5 — ПЕРЕНЕСИТЕ A → B";
            step_line1="Естественно перенесите аппарат примерно на 300–500 мм.";
            step_line2="В точке B остановитесь на 2–3 секунды и нажмите B.";
            step_color=cv::Scalar(0,255,0);
          } else if(!return_home_marked){
            step_title="ШАГ 4 / 5 — ФИЗИЧЕСКИ ВЕРНИТЕСЬ В A";
            step_line1="Вернитесь в реальную исходную точку, НЕ по метке EKF.";
            step_line2="Полностью остановитесь на 2–3 секунды и нажмите H.";
            step_color=cv::Scalar(0,180,255);
          } else {
            step_title="ШАГ 5 / 5 — ТЕСТ ЗАВЕРШЁН";
            step_line1="Ещё 2–3 секунды держите аппарат неподвижно.";
            step_line2="Результат уже записан. Нажмите Q или ESC для выхода.";
            step_color=cv::Scalar(255,255,0);
          }
          cv::rectangle(hud,cv::Rect(25,255,840,105),cv::Scalar(30,30,30),cv::FILLED);
          cv::rectangle(hud,cv::Rect(25,255,840,105),step_color,2);
          putGuiText(hud,step_title,{45,285},0.72,step_color,2);
          putGuiText(hud,step_line1,{45,318},0.48,cv::Scalar(230,230,230),1);
          putGuiText(hud,step_line2,{45,345},0.48,cv::Scalar(230,230,230),1);
          cv::line(hud,{450,45},{450,855},cv::Scalar(70,70,70),1);
          cv::line(hud,{45,450},{855,450},cv::Scalar(70,70,70),1);
          cv::circle(hud,center,16,cv::Scalar(0,220,0),2);
          cv::line(hud,{435,450},{465,450},cv::Scalar(0,220,0),2);
          cv::line(hud,{450,435},{450,465},cv::Scalar(0,220,0),2);

          double dn=0.0,de=0.0,dist=0.0,vh=0.0;
          if(return_target_set && efresh){
            dn=(double)ep.x-return_target_n;
            de=(double)ep.y-return_target_e;
            dist=std::hypot(dn,de);
            vh=std::hypot((double)ep.vx,(double)ep.vy);

            return_view_halfspan_m=std::max(0.30,std::max(return_view_halfspan_m*0.999,
                                      1.20*std::max(std::abs(dn),std::abs(de))));
            return_view_halfspan_m=std::min(return_view_halfspan_m,5.0);
            const double px_per_m=360.0/return_view_halfspan_m;
            const cv::Point cur(
              std::clamp((int)std::lround(center.x+de*px_per_m),50,850),
              std::clamp((int)std::lround(center.y-dn*px_per_m),50,850));

            return_trail.emplace_back(de,dn);
            while(return_trail.size()>1200)return_trail.pop_front();
            for(size_t ti=1;ti<return_trail.size();++ti){
              const cv::Point a(
                std::clamp((int)std::lround(center.x+return_trail[ti-1].x*px_per_m),50,850),
                std::clamp((int)std::lround(center.y-return_trail[ti-1].y*px_per_m),50,850));
              const cv::Point bpt(
                std::clamp((int)std::lround(center.x+return_trail[ti].x*px_per_m),50,850),
                std::clamp((int)std::lround(center.y-return_trail[ti].y*px_per_m),50,850));
              cv::line(hud,a,bpt,cv::Scalar(110,110,110),1);
            }
            cv::circle(hud,cur,10,cv::Scalar(0,180,255),-1);
            cv::arrowedLine(hud,cur,center,cv::Scalar(0,220,255),3,cv::LINE_AA,0,0.08);

            // Cyan RAW-NED point: independent optical-flow+gyro+attitude integration.
            const cv::Point raw_cur(
              std::clamp((int)std::lround(center.x+return_ned_e*px_per_m),50,850),
              std::clamp((int)std::lround(center.y-return_ned_n*px_per_m),50,850));
            cv::circle(hud,raw_cur,8,cv::Scalar(255,255,0),2,cv::LINE_AA);

            if(dist<=0.025){
              cv::circle(hud,center,34,cv::Scalar(0,255,0),3);
              putGuiText(hud,"ТОЧКА ДОСТИГНУТА",{285,95},1.0,cv::Scalar(0,255,0),3);
            }
          }

          const double raw_closure_mm=1000.0*std::hypot(return_raw_x,return_raw_y);
          const double raw_ned_closure_mm=1000.0*std::hypot(return_ned_n,return_ned_e);
          const double raw_body_closure_mm=1000.0*std::hypot(return_body_dx,return_body_dy);
          const double roll_deg=fg_ok?fg.roll*180.0/M_PI:0.0;
          const double pitch_deg=fg_ok?fg.pitch*180.0/M_PI:0.0;
          const double yaw_deg=fg_ok?fg.yaw*180.0/M_PI:0.0;
          const double dyaw_deg=(fg_ok&&return_yaw0_set)?std::remainder(fg.yaw-return_yaw0,2.0*M_PI)*180.0/M_PI:0.0;
          std::ostringstream l1,l2,l3,l4,l5,l6,l7,l8,l9,l10;
          l1<<std::fixed<<std::setprecision(0)<<"ДО ЦЕЛИ: "<<dist*1000.0<<" мм";
          l2<<std::fixed<<std::setprecision(1)<<"ОШИБКА N: "<<dn*1000.0<<" мм";
          l3<<std::fixed<<std::setprecision(1)<<"ОШИБКА E: "<<de*1000.0<<" мм";
          l4<<std::fixed<<std::setprecision(3)<<"СКОРОСТЬ XY: "<<vh<<" м/с   ДАЛЬНОМЕР: "<<(hl?lm:-1.0)<<" м";
          l5<<std::fixed<<std::setprecision(2)<<"МАСШТАБ: ±"<<return_view_halfspan_m<<" м";
          l6<<std::fixed<<std::setprecision(1)<<"RAW LOS, старый: "<<raw_closure_mm<<" мм";
          l7<<std::fixed<<std::setprecision(1)<<"КУРС: "<<yaw_deg<<"°   ΔКУРС(A): "<<dyaw_deg<<"°";
          l8<<std::fixed<<std::setprecision(1)<<"RAW NED, замыкание: "<<raw_ned_closure_mm
            <<" мм   ΔN/E "<<return_ned_n*1000.0<<"/"<<return_ned_e*1000.0;
          l9<<std::fixed<<std::setprecision(1)<<"RAW BODY: "<<raw_body_closure_mm
            <<" мм   ΔX/Y "<<return_body_dx*1000.0<<"/"<<return_body_dy*1000.0;
          l10<<std::fixed<<std::setprecision(1)<<"КРЕН/ТАНГАЖ/КУРС: "<<roll_deg<<"/"<<pitch_deg<<"/"<<yaw_deg
             <<"°   TF-Luna: "<<(hl?lm:-1.0)<<" м";
          putGuiText(hud,return_target_set?l1.str():"ОЖИДАНИЕ ГОТОВНОСТИ / ТОЧКИ A",{35,40},0.85,cv::Scalar(240,240,240),2);
          putGuiText(hud,l2.str(),{35,75},0.65,cv::Scalar(220,220,220),2);
          putGuiText(hud,l3.str(),{35,105},0.65,cv::Scalar(220,220,220),2);
          putGuiText(hud,l6.str(),{35,140},0.58,cv::Scalar(190,190,190),1);
          putGuiText(hud,l8.str(),{35,172},0.62,cv::Scalar(0,220,255),2);
          putGuiText(hud,l9.str(),{35,204},0.55,cv::Scalar(190,190,190),1);
          putGuiText(hud,l7.str(),{35,236},0.58,cv::Scalar(190,190,190),1);
          putGuiText(hud,l10.str(),{35,382},0.52,
                      (hl&&lm<0.20)?cv::Scalar(0,80,255):cv::Scalar(190,190,190),1);
          if(hl&&lm<0.20){
            putGuiText(hud,"TF-LUNA < 0,20 м: показания могут быть ненадёжны",{35,410},
                        0.50,cv::Scalar(0,80,255),1);
          }
          putGuiText(hud,l4.str(),{35,850},0.58,cv::Scalar(200,200,200),1);
          putGuiText(hud,l5.str(),{650,850},0.52,cv::Scalar(180,180,180),1);
          cv::putText(hud,"N",{458,65},cv::FONT_HERSHEY_SIMPLEX,0.65,cv::Scalar(160,160,160),2,cv::LINE_AA);
          cv::putText(hud,"E",{825,440},cv::FONT_HERSHEY_SIMPLEX,0.65,cv::Scalar(160,160,160),2,cv::LINE_AA);
          putGuiText(hud,"SPACE: ТОЧКА A   B: ДАЛЬНЯЯ   H: ВОЗВРАТ   C: ОЧИСТИТЬ   Q/ESC: ВЫХОД",{35,885},
                      0.50,cv::Scalar(160,160,160),1);

          // Live camera panel. Use the already decoded frame so this does not
          // open a second V4L2 stream or alter the optical-flow pipeline.
          cv::Mat cam_bgr,cam_view;
          cv::cvtColor(gray,cam_bgr,cv::COLOR_GRAY2BGR);
          const int cam_w=560;
          const int cam_h=(int)std::lround((double)cam_bgr.rows*cam_w/cam_bgr.cols);
          cv::resize(cam_bgr,cam_view,cv::Size(cam_w,cam_h),0,0,cv::INTER_AREA);
          const int cam_x=920;
          const int cam_y=105;
          if(cam_y+cam_h<=hud.rows && cam_x+cam_w<=hud.cols){
            cam_view.copyTo(hud(cv::Rect(cam_x,cam_y,cam_w,cam_h)));
            const int rx0=cam_x+(int)std::lround(g_feature_roi.x0*cam_w);
            const int ry0=cam_y+(int)std::lround(g_feature_roi.y0*cam_h);
            const int rx1=cam_x+(int)std::lround(g_feature_roi.x1*cam_w);
            const int ry1=cam_y+(int)std::lround(g_feature_roi.y1*cam_h);
            cv::rectangle(hud,cv::Point(rx0,ry0),cv::Point(rx1,ry1),
                          cv::Scalar(0,255,255),2,cv::LINE_AA);
            putGuiText(hud,"OV9281 — ЖИВОЕ ВИДЕО",{cam_x,70},0.80,
                        cv::Scalar(240,240,240),2);
            putGuiText(hud,"ЖЁЛТАЯ РАМКА — ОБЛАСТЬ KLT",{cam_x,cam_y+cam_h+32},
                        0.55,cv::Scalar(0,255,255),1);
            std::ostringstream cam_diag;
            cam_diag<<"КАДР "<<frame<<"   ВАЛИДЕН "<<(s.valid?1:0)
                    <<"   ИНЛАЙЕРЫ "<<s.inliers<<"/"<<s.tracked;
            putGuiText(hud,cam_diag.str(),{cam_x,cam_y+cam_h+62},
                        0.52,cv::Scalar(210,210,210),1);
          }

          cv::imshow(return_window_name,hud);
          const int key=cv::waitKey(1)&0xff;
          if(key==' ' && efresh){
            return_target_n=ep.x; return_target_e=ep.y;
            return_target_set=true; return_trail.clear();
            return_view_halfspan_m=0.50;
            return_raw_x=return_raw_y=0.0;
            return_body_dx=return_body_dy=0.0;
            return_ned_n=return_ned_e=0.0;
            return_b_marked=false;
            return_home_marked=false;
            if(fg_ok){ return_yaw0=fg.yaw; return_yaw0_set=true; }
            pending_return_event=1;
            std::cerr<<"RETURN GUI TARGET RESET: N="<<return_target_n<<" E="<<return_target_e
                     <<" yaw_deg="<<(return_yaw0_set?return_yaw0*180.0/M_PI:0.0)<<"\n";
          } else if((key=='b'||key=='B') && efresh){
            return_b_marked=true;
            return_b_n=ep.x; return_b_e=ep.y;
            return_b_raw_x=return_raw_x; return_b_raw_y=return_raw_y;
            return_b_body_dx=return_body_dx; return_b_body_dy=return_body_dy;
            return_b_ned_n=return_ned_n; return_b_ned_e=return_ned_e;
            return_b_yaw=fg_ok?fg.yaw:0.0;
            pending_return_event=2;
            std::cerr<<"RETURN GUI B MARK: EKF_from_A="<<1000.0*std::hypot(ep.x-return_target_n,ep.y-return_target_e)
                     <<" mm RAW_NED_from_A="<<1000.0*std::hypot(return_ned_n,return_ned_e)
                     <<" mm RAW_BODY_from_A="<<1000.0*std::hypot(return_body_dx,return_body_dy)
                     <<" mm RAW_LOS_legacy="<<1000.0*std::hypot(return_raw_x,return_raw_y)
                     <<" mm dYaw="<<(fg_ok&&return_yaw0_set?std::remainder(fg.yaw-return_yaw0,2.0*M_PI)*180.0/M_PI:0.0)<<" deg\n";
          } else if((key=='h'||key=='H') && efresh){
            pending_return_event=3;
            return_home_marked=true;
            const double ekf_close=1000.0*std::hypot(ep.x-return_target_n,ep.y-return_target_e);
            const double raw_close=1000.0*std::hypot(return_raw_x,return_raw_y);
            const double raw_body_close=1000.0*std::hypot(return_body_dx,return_body_dy);
            const double raw_ned_close=1000.0*std::hypot(return_ned_n,return_ned_e);
            std::cerr<<"\n======================================================================\n"
                     <<"RETURN CLOSURE MARK (PHYSICAL HOME)\n"
                     <<"EKF closure = "<<ekf_close<<" mm\n"
                     <<"RAW NED closure = "<<raw_ned_close<<" mm"
                     <<"  dN/E="<<return_ned_n*1000.0<<"/"<<return_ned_e*1000.0<<" mm\n"
                     <<"RAW BODY metric closure = "<<raw_body_close<<" mm\n"
                     <<"RAW LOS legacy closure = "<<raw_close<<" mm\n";
            if(return_b_marked){
              std::cerr<<"A->B EKF = "<<1000.0*std::hypot(return_b_n-return_target_n,return_b_e-return_target_e)<<" mm\n"
                       <<"B->H EKF = "<<1000.0*std::hypot(ep.x-return_b_n,ep.y-return_b_e)<<" mm\n"
                       <<"A->B RAW NED = "<<1000.0*std::hypot(return_b_ned_n,return_b_ned_e)<<" mm\n"
                       <<"B->H RAW NED = "<<1000.0*std::hypot(return_ned_n-return_b_ned_n,return_ned_e-return_b_ned_e)<<" mm\n"
                       <<"A->B RAW BODY = "<<1000.0*std::hypot(return_b_body_dx,return_b_body_dy)<<" mm\n"
                       <<"B->H RAW BODY = "<<1000.0*std::hypot(return_body_dx-return_b_body_dx,return_body_dy-return_b_body_dy)<<" mm\n"
                       <<"A->B RAW LOS legacy = "<<1000.0*std::hypot(return_b_raw_x,return_b_raw_y)<<" mm\n"
                       <<"B->H RAW LOS legacy = "<<1000.0*std::hypot(return_raw_x-return_b_raw_x,return_raw_y-return_b_raw_y)<<" mm\n";
            }
            std::cerr<<"dYaw(A->H) = "<<(fg_ok&&return_yaw0_set?std::remainder(fg.yaw-return_yaw0,2.0*M_PI)*180.0/M_PI:0.0)<<" deg\n"
                     <<"======================================================================\n";
          } else if(key=='c'||key=='C'){
            return_trail.clear();
          } else if(key=='q'||key=='Q'||key==27){
            g_running=false;
          }
        }

        // В guided-режиме подробная телеметрия остаётся в CSV, но не засоряет терминал.
        if(!guided && frame%100==0){
          std::cerr<<"OF frame="<<frame
                   <<" valid="<<(s.valid?1:0)
                   <<" rateFRD=("<<s.flow_body_x<<","<<s.flow_body_y<<") rad/s"
                   <<" inliers="<<s.inliers<<"/"<<s.tracked
                   <<" sent="<<flow_sent_total<<" invalid="<<flow_invalid_total
                   <<" stale_reject="<<stale_flow_rejected_total
                   <<" cam_drop="<<camera_queue_dropped_total
                   <<" latency="<<frame_pipeline_latency_ms<<"ms"
                   <<" range="<<range_sent_total
                   <<" luna="<<(hl?lm:-1.0)<<"m age="<<(hl?lage:-1.0)<<"ms";
          if(esfresh){
            std::cerr<<" EKFSTAT flags=0x"<<std::hex<<es.flags<<std::dec
                     <<" ["<<ekfFlagsText(es.flags)<<"]"
                     <<" varV="<<es.velocity_variance
                     <<" varPH="<<es.pos_horiz_variance;
          } else {
            std::cerr<<" EKFSTAT=NO_DATA";
          }
          if(efresh)std::cerr<<" LOCAL pN/E=("<<ep.x<<","<<ep.y<<") vN/E=("<<ep.vx<<","<<ep.vy<<")";
          else std::cerr<<" LOCAL=NO_DATA";
          std::cerr<<"\r"<<std::flush;
        }

        // Как в BlueOS: любой успешно декодированный кадр становится новым prev.
        prev=gray.clone(); prev_ts=ts;
    }

    g_running=false;
    if(guide_thread.joinable()) guide_thread.join();
    if(!remote_log_path.empty()){
      uint64_t rrx=0,rwr=0,rdup=0; size_t rpend=0;
      fc.remoteStats(&rrx,&rwr,&rdup,&rpend);
      fc.stopRemoteLog();
      std::cerr<<"REMOTE DATAFLASH: blocks_rx="<<rrx<<" written="<<rwr
               <<" duplicates="<<rdup<<" pending="<<rpend
               <<" BIN="<<remote_log_path<<"\n";
    }
    fc.stop(); luna.stop();
    std::cerr<<"\nОстановлено. CSV: "<<csvpath
             <<" flow_sent="<<flow_sent_total
             <<" invalid="<<flow_invalid_total
             <<" stale_reject="<<stale_flow_rejected_total
             <<" camera_queue_dropped="<<camera_queue_dropped_total
             <<" range_sent="<<range_sent_total<<"\n";
    return 0;
  } catch(const std::exception& e){
    g_running=false;
    std::cerr<<"ОШИБКА: "<<e.what()<<"\n";
    return 1;
  }
}
#endif // JTZERO_OPTFLOW_LIBRARY
