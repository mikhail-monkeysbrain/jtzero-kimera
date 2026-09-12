// JT-Zero — handheld geometry identification recorder.
// Записывает OV9281 MJPG + FC ATTITUDE/HIGHRES_IMU/RAW_IMU + TF-Luna
// в одном monotonic clock domain. Ничего не публикует в FC и не меняет параметры.
#define main jtzero_handheld_geometry_base_unused_main
#include "ground_motion_live_v2_v3_ab.cpp"
#undef main

#include "ardupilotmega/mavlink.h"

#include <atomic>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <thread>

namespace {

struct FcSample {
  double roll=0,pitch=0,yaw=0;
  double xacc=0,yacc=0,zacc=0;
  double xgyro=0,ygyro=0,zgyro=0;
  int64_t att_recv_ns=0, imu_recv_ns=0;
  bool att_valid=false, imu_valid=false;
  uint64_t att_count=0, imu_count=0;
};

struct FcReader {
  int fd=-1;
  std::thread th;
  std::mutex mu;
  FcSample s{};
  uint8_t target_sys=0,target_comp=0;

  static constexpr uint8_t self_sys=191;
  static constexpr uint8_t self_comp=198;

  ~FcReader(){stop();}

  static void writeAll(int fd,const uint8_t* p,size_t n){
    size_t o=0;
    while(o<n){
      ssize_t k=::write(fd,p+o,n-o);
      if(k>0){o+=(size_t)k;continue;}
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
    auto n=mavlink_msg_to_send_buffer(b,&m);
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
        ssize_t n=read(fd,buf,sizeof(buf));
        if(n<=0)continue;
        for(ssize_t i=0;i<n;i++){
          if(!mavlink_parse_char(MAVLINK_COMM_0,buf[i],&m,&st))continue;
          if(m.msgid!=MAVLINK_MSG_ID_HEARTBEAT)continue;
          mavlink_heartbeat_t hb{}; mavlink_msg_heartbeat_decode(&m,&hb);
          if(hb.autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA){sys=m.sysid;comp=m.compid;break;}
        }
      }
      if(!sys){std::cerr<<"FC heartbeat timeout\n";g_running=false;return;}
      target_sys=sys; target_comp=comp;
      std::cerr<<"FC heartbeat sys="<<(int)sys<<" comp="<<(int)comp<<"\n";
      requestRate(fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,100);
      requestRate(fd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,100);
      requestRate(fd,sys,comp,MAVLINK_MSG_ID_RAW_IMU,100);

      while(g_running){
        pollfd p{fd,POLLIN,0};
        if(poll(&p,1,50)<=0)continue;
        for(;;){
          ssize_t n=read(fd,buf,sizeof(buf));
          if(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK))break;
          if(n<=0)break;
          for(ssize_t i=0;i<n;i++){
            if(!mavlink_parse_char(MAVLINK_COMM_0,buf[i],&m,&st))continue;
            if(m.sysid!=sys)continue;
            const int64_t now=monoNs();
            if(m.msgid==MAVLINK_MSG_ID_ATTITUDE){
              mavlink_attitude_t q{}; mavlink_msg_attitude_decode(&m,&q);
              std::lock_guard<std::mutex> l(mu);
              s.roll=q.roll;s.pitch=q.pitch;s.yaw=q.yaw;
              s.att_recv_ns=now;s.att_valid=true;++s.att_count;
            } else if(m.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){
              mavlink_highres_imu_t q{}; mavlink_msg_highres_imu_decode(&m,&q);
              std::lock_guard<std::mutex> l(mu);
              s.xacc=q.xacc;s.yacc=q.yacc;s.zacc=q.zacc;
              s.xgyro=q.xgyro;s.ygyro=q.ygyro;s.zgyro=q.zgyro;
              s.imu_recv_ns=now;s.imu_valid=true;++s.imu_count;
            } else if(m.msgid==MAVLINK_MSG_ID_RAW_IMU && !s.imu_valid){
              mavlink_raw_imu_t q{}; mavlink_msg_raw_imu_decode(&m,&q);
              std::lock_guard<std::mutex> l(mu);
              // RAW_IMU accel is milli-g and gyro is millirad/s in ArduPilot MAVLink output.
              s.xacc=q.xacc*9.80665e-3;s.yacc=q.yacc*9.80665e-3;s.zacc=q.zacc*9.80665e-3;
              s.xgyro=q.xgyro*1e-3;s.ygyro=q.ygyro*1e-3;s.zgyro=q.zgyro*1e-3;
              s.imu_recv_ns=now;s.imu_valid=true;++s.imu_count;
            }
          }
        }
      }
    });
  }

  FcSample latest(){
    std::lock_guard<std::mutex> l(mu);
    return s;
  }

  void stop(){
    if(th.joinable())th.join();
    if(fd>=0){::close(fd);fd=-1;}
  }
};

void writeU32(std::ofstream& f,uint32_t v){
  f.write(reinterpret_cast<const char*>(&v),sizeof(v));
}

} // namespace

int main(int argc,char** argv){
  if(argc<5){
    std::cerr<<"Использование: "<<argv[0]<<" <camera> <luna> <fc> <run_dir> [save_every_n]\n";
    return 2;
  }
  const std::string camdev=argv[1],lunadev=argv[2],fcdev=argv[3],run_dir=argv[4];
  const int save_every=(argc>5)?std::max(1,std::stoi(argv[5])):5;

  std::signal(SIGINT,onSignal); std::signal(SIGTERM,onSignal);

  try{
    Camera cam; cam.openDev(camdev);
    LunaReader luna; luna.start(lunadev);
    FcReader fc; fc.start(fcdev);

    std::filesystem::create_directories(run_dir);
    std::ofstream frames(run_dir+"/frames.csv",std::ios::trunc);
    std::ofstream bin(run_dir+"/frames.mjpgbin",std::ios::binary|std::ios::trunc);
    if(!frames||!bin)fail("open output");

    frames<<"frame,camera_ts_ns,recv_ns,jpeg_offset,jpeg_size,luna_m,luna_age_ms,"
             "roll,pitch,yaw,att_age_ms,xacc,yacc,zacc,xgyro,ygyro,zgyro,imu_age_ms,att_count,imu_count\n";

    uint64_t frame=0,saved=0; uint64_t offset=0;
    std::cerr<<"HANDHELD GEOMETRY RECORDER\n"
             <<"camera="<<camdev<<" luna="<<lunadev<<" fc="<<fcdev<<"\n"
             <<"run_dir="<<run_dir<<" save_every="<<save_every<<"\n"
             <<"Ctrl+C для остановки.\n";

    while(g_running){
      pollfd p{cam.fd,POLLIN,0};
      int pr=poll(&p,1,20);
      if(pr<0){if(errno==EINTR)continue;fail("camera poll");}
      if(pr<=0)continue;

      while(g_running){
        v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;
        if(xioctl(cam.fd,VIDIOC_DQBUF,&b)<0){if(errno==EAGAIN)break;fail("VIDIOC_DQBUF");}
        const int64_t now=monoNs();
        const int64_t ts=(int64_t)b.timestamp.tv_sec*1000000000LL+(int64_t)b.timestamp.tv_usec*1000LL;
        ++frame;

        if(frame%(uint64_t)save_every==0){
          double lm=0;int strength=0;int64_t lns=0;
          bool hl=luna.latest(&lm,&strength,&lns);
          auto q=fc.latest();

          const uint32_t sz=(uint32_t)b.bytesused;
          const uint64_t payload_offset=offset+sizeof(uint32_t);
          writeU32(bin,sz);
          bin.write(reinterpret_cast<const char*>(cam.bufs[b.index].p),sz);
          offset+=sizeof(uint32_t)+sz;
          ++saved;

          const double lage=hl?(now-lns)*1e-6:1e9;
          const double aage=q.att_valid?(now-q.att_recv_ns)*1e-6:1e9;
          const double iage=q.imu_valid?(now-q.imu_recv_ns)*1e-6:1e9;
          frames<<frame<<','<<ts<<','<<now<<','<<payload_offset<<','<<sz<<','
                <<(hl?lm:-1.0)<<','<<lage<<','
                <<q.roll<<','<<q.pitch<<','<<q.yaw<<','<<aage<<','
                <<q.xacc<<','<<q.yacc<<','<<q.zacc<<','
                <<q.xgyro<<','<<q.ygyro<<','<<q.zgyro<<','<<iage<<','
                <<q.att_count<<','<<q.imu_count<<'\n';
        }

        if(xioctl(cam.fd,VIDIOC_QBUF,&b)<0)fail("VIDIOC_QBUF");
        if(frame%500==0){
          auto q=fc.latest();
          std::cerr<<"frame="<<frame<<" saved="<<saved
                   <<" ATT="<<q.att_count<<" IMU="<<q.imu_count<<"\r"<<std::flush;
        }
      }
    }

    g_running=false;
    fc.stop(); luna.stop();
    std::cerr<<"\nГОТОВО: "<<run_dir<<" frames_saved="<<saved<<"\n";
    return 0;
  }catch(const std::exception& e){
    g_running=false;
    std::cerr<<"ОШИБКА: "<<e.what()<<"\n";
    return 1;
  }
}
