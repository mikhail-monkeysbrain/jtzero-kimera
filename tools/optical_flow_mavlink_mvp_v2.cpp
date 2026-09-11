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

namespace {

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

struct FlowFc {
  int fd=-1;
  std::thread th;
  std::mutex mu;
  FlowFcLocal local{};
  FlowEkfStatus ekf{};
  uint64_t local_count=0;
  uint64_t ekf_count=0;
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
  for(size_t i=0;i<ai.size();++i){
    dun.push_back((double)bu[i].x-au[i].x);
    dvn.push_back((double)bu[i].y-au[i].y);
    dup.push_back((double)bi[i].x-ai[i].x);
    dvp.push_back((double)bi[i].y-ai[i].y);
  }
  o.du_norm=median(dun); o.dv_norm=median(dvn);
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

int main(int argc,char** argv){
  if(argc<7){
    std::cerr<<"Использование: "<<argv[0]
             <<" <camera> <luna> <fc> <csv> <camera_yaml> <focal_scale>\n";
    return 2;
  }

  const std::string camdev=argv[1], lunadev=argv[2], fcdev=argv[3];
  const std::string csvpath=argv[4], yaml=argv[5];
  const double focal_scale=std::stod(argv[6]);
  bool guided175=false;
  bool require_armed=false;
  double bench_height_override=0.0;
  std::string remote_log_path;
  for(int i=7;i<argc;i++){
    const std::string a=argv[i];
    if(a=="--guided-175") guided175=true;
    else if(a=="--require-armed") require_armed=true;
    else if(a=="--bench-height" && i+1<argc) bench_height_override=std::stod(argv[++i]);
    else if(a=="--remote-log" && i+1<argc) remote_log_path=argv[++i];
  }
  if(bench_height_override!=0.0 && !(bench_height_override>=0.55 && bench_height_override<=2.0)){
    std::cerr<<"ОШИБКА: --bench-height разрешён только 0.55..2.0 м для bench-диагностики\n";
    return 2;
  }
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
    csv<<"mono_ns,frame,valid,dt_s,features,tracked,inliers,inlier_ratio,du_px,dv_px,du_norm,dv_norm,flow_cam_x,flow_cam_y,flow_body_x,flow_body_y,quality,luna_m,luna_age_ms,range_to_fc_m,flow_send_x,flow_send_y,flow_sent,range_sent,fc_armed,ekf_local_valid,ekf_x_ned,ekf_y_ned,ekf_z_ned,ekf_vx_ned,ekf_vy_ned,ekf_vz_ned,ekf_age_ms,ekf_count,ekf_status_valid,ekf_flags,ekf_status_age_ms,ekf_status_count,ekf_vel_var,ekf_pos_h_var,ekf_pos_v_var,ekf_compass_var,ekf_terrain_var\n";

    cv::setNumThreads(1);
    std::signal(SIGINT,onSignal); std::signal(SIGTERM,onSignal);

    cv::Mat prev; int64_t prev_ts=0; uint64_t frame=0;
    uint64_t flow_sent_total=0,flow_invalid_total=0,range_sent_total=0;
    int64_t last_range_send_ns=0;

    std::atomic<int> guide_stage{0}; // 0=pre-static, 1=move, 2=post-static, 3=done
    std::atomic<bool> arm_lost{false};
    FlowFcLocal guide_start{}, guide_end{};
    std::thread guide_thread;
    if(guided175){
      guide_thread=std::thread([&]{
        // Даём стартовым строкам camera/FC напечататься до пошаговой инструкции.
        std::this_thread::sleep_for(std::chrono::milliseconds(750));
        bool arm=false; double arm_age=1e9;
        const bool have_arm=fc.latestArm(&arm,&arm_age) && arm_age<2500.0;
        std::cerr<<"\n======================================================================\n"
                 <<"GUIDED TEST — ФИЗИЧЕСКИЙ СДВИГ 175 мм\n"
                 <<"======================================================================\n"
                 <<"1. НЕ ДВИГАЙТЕ аппарат. Сейчас автоматически собирается 5 с статики.\n"
                 <<"2. После команды ДВИГАЙТЕ сдвиньте ВЕСЬ аппарат строго по столу на 175 мм.\n"
                 <<"3. НЕ вращайте, не наклоняйте и не приподнимайте аппарат.\n"
                 <<"4. После сдвига полностью остановите аппарат.\n"
                 <<"5. Только после полной остановки нажмите Enter.\n"
                 <<"6. Затем аппарат снова НЕ ТРОГАТЬ 5 секунд — тест завершится сам.\n"
                 <<"======================================================================\n"
                 <<"ARM STATE: "<<(have_arm?(arm?"ARMED":"DISARMED"):"NO_DATA")<<"\n";
        if(require_armed && (!have_arm || !arm)){
          std::cerr<<"ОШИБКА: этот A/B-прогон требует ARMED.\n"
                   <<"Программа сама НЕ армит FC. Сначала безопасно подготовьте аппарат,\n"
                   <<"уберите пропеллеры/исключите тягу, армируйте штатным способом и запустите тест снова.\n";
          g_running=false; return;
        }
        std::cerr<<"СТАТИКА 5 секунд. НЕ ДВИГАТЬ.\n";
        std::this_thread::sleep_for(std::chrono::seconds(5));
        double age=0; uint64_t count=0;
        if(!fc.latestLocal(&guide_start,&age,&count) || age>500){
          std::cerr<<"\nОШИБКА GUIDE: нет свежего LOCAL_POSITION_NED перед движением.\n";
          g_running=false; return;
        }
        guide_stage=1;
        std::cerr<<"\n>>> ДВИГАЙТЕ: сдвиньте аппарат на 175 мм строго по столу.\n"
                 <<">>> После полной остановки нажмите Enter.\n";
        std::string line; std::getline(std::cin,line);
        guide_stage=2;
        std::cerr<<"\n>>> СТОП. НЕ ТРОГАТЬ аппарат 5 секунд. Идёт финальная статика...\n";
        std::this_thread::sleep_for(std::chrono::seconds(5));
        if(!fc.latestLocal(&guide_end,&age,&count) || age>500){
          std::cerr<<"\nОШИБКА GUIDE: нет свежего LOCAL_POSITION_NED после движения.\n";
          g_running=false; return;
        }
        if(require_armed && arm_lost.load()){
          std::cerr<<"\nARMed-test прерван из-за DISARM. Итог 175 мм не вычисляется.\n";
          g_running=false; return;
        }
        guide_stage=3;
        const double dn=guide_end.x-guide_start.x, de=guide_end.y-guide_start.y;
        const double dist=std::hypot(dn,de);
        std::cerr<<"\n======================================================================\n"
                 <<"GUIDED 175 мм — РЕЗУЛЬТАТ\n"
                 <<"======================================================================\n"
                 <<"START N/E = ("<<guide_start.x<<", "<<guide_start.y<<") m\n"
                 <<"END   N/E = ("<<guide_end.x<<", "<<guide_end.y<<") m\n"
                 <<"DELTA N/E = ("<<dn<<", "<<de<<") m\n"
                 <<"EKF horizontal displacement = "<<dist*1000.0<<" mm\n"
                 <<"Target = 175.0 mm\n"
                 <<"Error  = "<<(dist*1000.0-175.0)<<" mm ("<<((dist/0.175)-1.0)*100.0<<" %)\n"
                 <<"======================================================================\n";
        g_running=false;
      });
    }

    std::cerr<<"JT-ZERO OPTICAL FLOW MAVLINK MVP v2\n"
             <<"camera="<<camdev<<"\n"
             <<"fx/fy effective="<<calib.fx<<" / "<<calib.fy
             <<" (focal_scale="<<focal_scale<<")\n"
             <<"ВАЖНО: publisher выдаёт body-FRD flow; ожидается FLOW_ORIENT_YAW=0, FLOW_OPTIONS=0\n"
             <<"DIAG: запрошен EKF_STATUS_REPORT 5 Hz; LOCAL_POSITION_NED 20 Hz\n";
    if(bench_height_override>0.0){
      std::cerr<<"BENCH HEIGHT OVERRIDE: FC получает "<<bench_height_override
               <<" м вместо реального TF-Luna. Flow-rate масштабируется real/fake,\n"
               <<"чтобы метрическая скорость оставалась соответствующей реальной высоте.\n"
               <<"ЭТО ТОЛЬКО СТЕНДОВАЯ ДИАГНОСТИКА, НЕ FLIGHT-РЕЖИМ.\n";
    }

    // Переводим AP_OpticalFlow_MAV в high-precision flow_rate mode.
    // quality=0: это не валидное aiding measurement.
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
        if(gray.empty())continue;
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
        if(s.valid && bench_height_override>0.0 && hl && lm>0.05){
          const double k=lm/bench_height_override;
          flow_send_x*=k;
          flow_send_y*=k;
        }
        if(s.valid){
          quality=255;
          flow_sent=sendOpticalFlow(fc.fd,(uint64_t)(now/1000),
            (float)flow_send_x,(float)flow_send_y,quality);
          if(flow_sent)++flow_sent_total;
        } else if(!prev.empty()){
          ++flow_invalid_total;
        }

        FlowFcLocal ep{}; double eage=1e9; uint64_t ec=0;
        const bool eok=fc.latestLocal(&ep,&eage,&ec);
        const bool efresh=eok&&eage<500.0;
        FlowEkfStatus es{}; double esage=1e9; uint64_t esc=0;
        const bool esok=fc.latestEkf(&es,&esage,&esc);
        const bool esfresh=esok&&esage<1000.0;

        bool arm_now=false; double arm_age_now=1e9;
        const bool arm_ok=fc.latestArm(&arm_now,&arm_age_now) && arm_age_now<2500.0;
        if(require_armed && arm_ok && !arm_now && guide_stage.load()<3){
          if(!arm_lost.exchange(true)){
            std::cerr<<"\nОШИБКА: FC ПЕРЕШЁЛ В DISARMED ВО ВРЕМЯ ARMED-ТЕСТА.\n"
                     <<"Тест остановлен; результат движения недействителен.\n";
          }
          g_running=false;
        }

        csv<<now<<','<<frame<<','<<(s.valid?1:0)<<','<<dt<<','
           <<s.features<<','<<s.tracked<<','<<s.inliers<<','<<s.inlier_ratio<<','
           <<s.du_px<<','<<s.dv_px<<','<<s.du_norm<<','<<s.dv_norm<<','
           <<s.flow_cam_x<<','<<s.flow_cam_y<<','<<s.flow_body_x<<','<<s.flow_body_y<<','
           <<(int)quality<<','<<lm<<','<<lage<<','<<range_to_fc<<','<<flow_send_x<<','<<flow_send_y<<','<<(flow_sent?1:0)<<','<<(range_sent?1:0)<<','
           <<(arm_ok?(arm_now?1:0):-1)<<','
           <<(efresh?1:0)<<','<<ep.x<<','<<ep.y<<','<<ep.z<<','<<ep.vx<<','<<ep.vy<<','<<ep.vz<<','<<eage<<','<<ec<<','
           <<(esfresh?1:0)<<','<<es.flags<<','<<esage<<','<<esc<<','
           <<es.velocity_variance<<','<<es.pos_horiz_variance<<','<<es.pos_vert_variance<<','<<es.compass_variance<<','<<es.terrain_alt_variance<<'\n';

        // В guided-режиме подробная телеметрия остаётся в CSV, но не засоряет терминал.
        if(!guided175 && frame%100==0){
          std::cerr<<"OF frame="<<frame
                   <<" valid="<<(s.valid?1:0)
                   <<" rateFRD=("<<s.flow_body_x<<","<<s.flow_body_y<<") rad/s"
                   <<" inliers="<<s.inliers<<"/"<<s.tracked
                   <<" sent="<<flow_sent_total<<" invalid="<<flow_invalid_total
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
             <<" range_sent="<<range_sent_total<<"\n";
    return 0;
  } catch(const std::exception& e){
    g_running=false;
    std::cerr<<"ОШИБКА: "<<e.what()<<"\n";
    return 1;
  }
}
