// JT-Zero Ground Motion — диагностическая запись для детерминированного replay.
// Production-код не меняет. Сохраняет исходные MJPG-байты каждого V4L2 кадра
// и синхронизированные ATTITUDE/TF-Luna/height метаданные.
#define main jtzero_ground_motion_capture_base_unused_main
#include "ground_motion_live_v2_v3_ab.cpp"
#undef main

#include <deque>
#include <filesystem>

namespace {
int64_t frameTvToNsReplayRecord(const timeval& tv){return (int64_t)tv.tv_sec*1000000000LL+(int64_t)tv.tv_usec*1000LL;}
double wrapReplayRecord(double a){while(a>kPi)a-=2*kPi;while(a<-kPi)a+=2*kPi;return a;}

struct CaptureFcHistory {
  int fd=-1; std::thread th; std::mutex mu; std::deque<Attitude> hist;
  static constexpr uint8_t self_sys=191;
  static constexpr uint8_t self_comp=MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY;
  ~CaptureFcHistory(){g_running=false;stop();}

  static void writeAll(int fd,const uint8_t*p,size_t n){
    size_t o=0; while(o<n){ssize_t k=write(fd,p+o,n-o);if(k>0){o+=(size_t)k;continue;}
      if(k<0&&(errno==EAGAIN||errno==EWOULDBLOCK)){pollfd q{fd,POLLOUT,0};poll(&q,1,10);continue;}
      if(k<0&&errno==EINTR)continue;fail("FC write");}
  }
  static void requestRate(int fd,uint8_t sys,uint8_t comp,uint32_t msgid,int hz){
    mavlink_message_t m{};mavlink_msg_command_long_pack(self_sys,self_comp,&m,sys,comp,MAV_CMD_SET_MESSAGE_INTERVAL,0,msgid,1000000.0f/hz,0,0,0,0,0);
    uint8_t b[MAVLINK_MAX_PACKET_LEN];auto n=mavlink_msg_to_send_buffer(b,&m);writeAll(fd,b,n);
  }
  void start(const std::string&dev){
    fd=::open(dev.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK);if(fd<0)fail("open FC");
    termios t{};if(tcgetattr(fd,&t)<0)fail("FC tcgetattr");cfmakeraw(&t);cfsetispeed(&t,B460800);cfsetospeed(&t,B460800);
    t.c_cflag|=CLOCAL|CREAD;t.c_cflag&=~CRTSCTS;t.c_cflag&=~PARENB;t.c_cflag&=~CSTOPB;t.c_cflag&=~CSIZE;t.c_cflag|=CS8;
    if(tcsetattr(fd,TCSANOW,&t)<0)fail("FC tcsetattr");tcflush(fd,TCIFLUSH);
    th=std::thread([this]{
      mavlink_status_t st{};mavlink_message_t m{};uint8_t sys=0,comp=0,buf[4096];int64_t deadline=monoNs()+10000000000LL;
      while(g_running&&!sys&&monoNs()<deadline){pollfd p{fd,POLLIN,0};if(poll(&p,1,100)<=0)continue;ssize_t n=read(fd,buf,sizeof(buf));if(n<=0)continue;
        for(ssize_t i=0;i<n;i++){if(!mavlink_parse_char(MAVLINK_COMM_0,buf[i],&m,&st)||m.msgid!=MAVLINK_MSG_ID_HEARTBEAT)continue;
          mavlink_heartbeat_t hb{};mavlink_msg_heartbeat_decode(&m,&hb);if(hb.autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA){sys=m.sysid;comp=m.compid;break;}}}
      if(!sys){std::cerr<<"FC: ArduPilot HEARTBEAT timeout\n";g_running=false;return;}
      std::cerr<<"FC: ArduPilot heartbeat sys="<<(int)sys<<" comp="<<(int)comp<<"\n";requestRate(fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,50);
      while(g_running){pollfd p{fd,POLLIN,0};if(poll(&p,1,50)<=0)continue;for(;;){ssize_t n=read(fd,buf,sizeof(buf));if(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;
          for(ssize_t i=0;i<n;i++){if(!mavlink_parse_char(MAVLINK_COMM_0,buf[i],&m,&st)||m.msgid!=MAVLINK_MSG_ID_ATTITUDE||m.sysid!=sys)continue;
            mavlink_attitude_t a{};mavlink_msg_attitude_decode(&m,&a);Attitude x{a.roll,a.pitch,a.yaw,monoNs(),true};std::lock_guard<std::mutex>l(mu);hist.push_back(x);
            int64_t keep=x.recv_ns-3000000000LL;while(hist.size()>2&&hist.front().recv_ns<keep)hist.pop_front();}}}
    });
  }
  bool at(int64_t target,Attitude*out,double*nearest_ms){
    std::lock_guard<std::mutex>l(mu);if(hist.size()<2||target<hist.front().recv_ns||target>hist.back().recv_ns)return false;
    size_t hi=1;while(hi<hist.size()&&hist[hi].recv_ns<target)++hi;if(hi>=hist.size())return false;const auto&a=hist[hi-1];const auto&b=hist[hi];
    double u=double(target-a.recv_ns)/double(b.recv_ns-a.recv_ns);out->roll=a.roll+u*(b.roll-a.roll);out->pitch=a.pitch+u*(b.pitch-a.pitch);
    out->yaw=wrapReplayRecord(a.yaw+u*wrapReplayRecord(b.yaw-a.yaw));out->recv_ns=target;out->valid=true;
    if(nearest_ms)*nearest_ms=std::min(std::llabs(target-a.recv_ns),std::llabs(b.recv_ns-target))*1e-6;return true;
  }
  void stop(){if(th.joinable())th.join();if(fd>=0){::close(fd);fd=-1;}}
};

struct PendingMeta {
  uint64_t frame=0,offset=0,size=0;
  int64_t mono_ns=0,ts_ns=0;
  bool decode_ok=false,luna_valid=false;
  double luna_m=0,luna_age_ms=1e9;
};
}

int main(int argc,char**argv){
  if(argc<7){std::cerr<<"Использование: "<<argv[0]<<" <camera> <luna> <fc> <out_dir> <camera_yaml> <camera_offset_mm>\n";return 2;}
  const std::string camdev=argv[1],lunadev=argv[2],fcdev=argv[3];const std::filesystem::path outdir=argv[4];
  const double offset=std::stod(argv[6])/1000.0;const cv::Vec3d lever_b(0.04916,0.00022,0.0);
  try{
    std::filesystem::create_directories(outdir);Camera cam;cam.openDev(camdev);LunaReader luna;luna.start(lunadev);CaptureFcHistory fc;fc.start(fcdev);
    std::ofstream payload(outdir/"frames.mjpgbin",std::ios::binary|std::ios::trunc),meta(outdir/"frames.csv",std::ios::trunc);
    if(!payload||!meta)throw std::runtime_error("не удалось создать файлы dataset");
    meta<<"frame,mono_ns,ts_ns,offset_bytes,size_bytes,decode_ok,luna_valid,luna_m,luna_age_ms,att_valid,att_age_ms,roll,pitch,yaw,height_m,sensors_valid,sync_wait_ms,sync_timeout\n";
    cv::setNumThreads(1);std::signal(SIGINT,onSignal);std::signal(SIGTERM,onSignal);
    uint64_t frame=0,att_ok_count=0,sensors_ok_count=0,sync_timeout_count=0;std::deque<PendingMeta>pending;
    constexpr int64_t kSyncGraceNs=80000000LL; // 80 ms: > 1 ATTITUDE period at 50 Hz, без блокировки capture.

    auto flushPending=[&](bool force){
      while(!pending.empty()){
        const PendingMeta&p=pending.front();Attitude att{};double att_age=1e9;bool ha=fc.at(p.ts_ns,&att,&att_age);
        const int64_t cur=monoNs();const bool timeout=(cur-p.mono_ns)>=kSyncGraceNs;
        if(!ha&&!force&&!timeout)break;
        if(ha)++att_ok_count;else if(timeout||force)++sync_timeout_count;
        double h=0,down=0;if(p.luna_valid&&ha){auto R=attitudeFluToNwu(att);down=-(R*cv::Vec3d(0,0,-1))[2];h=p.luna_m*down-offset+(R*lever_b)[2];}
        const bool sensors=p.decode_ok&&p.luna_valid&&ha&&down>0.20&&h>0.05&&p.luna_age_ms<200&&att_age<30;
        if(sensors)++sensors_ok_count;
        const double wait_ms=(cur-p.mono_ns)*1e-6;
        meta<<p.frame<<','<<p.mono_ns<<','<<p.ts_ns<<','<<p.offset<<','<<p.size<<','<<(p.decode_ok?1:0)<<','<<(p.luna_valid?1:0)<<','<<p.luna_m<<','<<p.luna_age_ms<<','<<(ha?1:0)<<','<<att_age<<','
            <<att.roll<<','<<att.pitch<<','<<att.yaw<<','<<h<<','<<(sensors?1:0)<<','<<wait_ms<<','<<((!ha&&(timeout||force))?1:0)<<'\n';
        pending.pop_front();
      }
    };

    while(g_running){
      pollfd pfd{cam.fd,POLLIN,0};int pr=poll(&pfd,1,20);if(pr<0){if(errno==EINTR)continue;fail("camera poll");}if(pr<=0){flushPending(false);continue;}
      while(g_running){
        v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;if(xioctl(cam.fd,VIDIOC_DQBUF,&b)<0){if(errno==EAGAIN)break;fail("VIDIOC_DQBUF");}
        const int64_t now=monoNs(),ts=frameTvToNsReplayRecord(b.timestamp);std::vector<uint8_t>raw((uint8_t*)cam.bufs[b.index].p,(uint8_t*)cam.bufs[b.index].p+b.bytesused);
        if(xioctl(cam.fd,VIDIOC_QBUF,&b)<0)fail("VIDIOC_QBUF");++frame;const uint64_t off=(uint64_t)payload.tellp();
        payload.write(reinterpret_cast<const char*>(raw.data()),(std::streamsize)raw.size());if(!payload)throw std::runtime_error("ошибка записи frames.mjpgbin");
        cv::Mat one(1,(int)raw.size(),CV_8UC1,raw.data());const bool decode_ok=!cv::imdecode(one,cv::IMREAD_GRAYSCALE).empty();
        double lm=0;int strength=0;int64_t lns=0;const bool hl=luna.latest(&lm,&strength,&lns);const double lage=hl?(now-lns)*1e-6:1e9;
        pending.push_back(PendingMeta{frame,off,(uint64_t)raw.size(),now,ts,decode_ok,hl,lm,lage});
        flushPending(false);
        if(frame%100==0){meta.flush();payload.flush();std::cerr<<"REC frame="<<frame<<" bytes="<<off+raw.size()<<" pending="<<pending.size()<<" att_ok="<<att_ok_count<<" sensors="<<sensors_ok_count<<" timeout="<<sync_timeout_count<<"\r"<<std::flush;}
      }
      flushPending(false);
    }
    flushPending(true);g_running=false;fc.stop();luna.stop();meta.flush();payload.flush();
    std::cerr<<"\nЗапись завершена: "<<outdir<<" frames="<<frame<<" att_ok="<<att_ok_count<<" sensors="<<sensors_ok_count<<" sync_timeout="<<sync_timeout_count<<"\n";return 0;
  }catch(const std::exception&e){g_running=false;std::cerr<<"ОШИБКА: "<<e.what()<<"\n";return 1;}
}
