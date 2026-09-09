// JT-Zero R1 standalone deterministic recorder.
// Deliberately independent of V25/V43/Kimera diagnostic sources.
#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <termios.h>
#include <unistd.h>

#include "common/mavlink.h"

namespace {
std::atomic<bool> running{true};
uint64_t mono_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
void sig_handler(int) { running=false; }
int xioctl(int fd, unsigned long req, void* arg) {
  int r; do { r=::ioctl(fd,req,arg); } while(r<0 && errno==EINTR); return r;
}
void fail(const std::string& s) { throw std::runtime_error(s+": "+std::strerror(errno)); }

struct MapBuf { void* p=nullptr; size_t n=0; };
struct Camera {
  int fd=-1; std::vector<MapBuf> bufs;
  ~Camera(){ close(); }
  void open_dev(const std::string& dev) {
    fd=::open(dev.c_str(),O_RDWR|O_NONBLOCK);
    if(fd<0) fail("open camera");
    v4l2_format f{}; f.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    f.fmt.pix.width=640; f.fmt.pix.height=480;
    f.fmt.pix.pixelformat=V4L2_PIX_FMT_MJPEG; f.fmt.pix.field=V4L2_FIELD_ANY;
    if(xioctl(fd,VIDIOC_S_FMT,&f)<0) fail("VIDIOC_S_FMT");
    if(f.fmt.pix.pixelformat!=V4L2_PIX_FMT_MJPEG) throw std::runtime_error("camera did not accept MJPEG");
    v4l2_streamparm sp{}; sp.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    sp.parm.capture.timeperframe.numerator=1; sp.parm.capture.timeperframe.denominator=100;
    xioctl(fd,VIDIOC_S_PARM,&sp);
    v4l2_requestbuffers rb{}; rb.count=8; rb.type=V4L2_BUF_TYPE_VIDEO_CAPTURE; rb.memory=V4L2_MEMORY_MMAP;
    if(xioctl(fd,VIDIOC_REQBUFS,&rb)<0 || rb.count<2) fail("VIDIOC_REQBUFS");
    bufs.resize(rb.count);
    for(unsigned i=0;i<rb.count;i++){
      v4l2_buffer b{}; b.type=rb.type; b.memory=rb.memory; b.index=i;
      if(xioctl(fd,VIDIOC_QUERYBUF,&b)<0) fail("VIDIOC_QUERYBUF");
      bufs[i].n=b.length; bufs[i].p=::mmap(nullptr,b.length,PROT_READ|PROT_WRITE,MAP_SHARED,fd,b.m.offset);
      if(bufs[i].p==MAP_FAILED) fail("mmap");
      if(xioctl(fd,VIDIOC_QBUF,&b)<0) fail("VIDIOC_QBUF");
    }
    v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(xioctl(fd,VIDIOC_STREAMON,&t)<0) fail("VIDIOC_STREAMON");
  }
  void close(){
    if(fd<0) return;
    v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE; xioctl(fd,VIDIOC_STREAMOFF,&t);
    for(auto& b:bufs) if(b.p && b.p!=MAP_FAILED) ::munmap(b.p,b.n);
    ::close(fd); fd=-1;
  }
};

struct FcMavlink {
  int fd=-1; std::thread th; std::ofstream imu_csv, att_csv;
  std::atomic<uint64_t> imu_samples{0}, att_samples{0};
  ~FcMavlink(){ stop(); }
  static void write_all(int fd,const uint8_t* p,size_t n){
    for(size_t o=0;o<n;){ ssize_t k=::write(fd,p+o,n-o);
      if(k>0){o+=static_cast<size_t>(k);continue;}
      if(k<0&&(errno==EAGAIN||errno==EWOULDBLOCK)){pollfd q{fd,POLLOUT,0};::poll(&q,1,10);continue;}
      if(k<0&&errno==EINTR)continue; fail("FC serial write");
    }
  }
  static void send_msg(int fd,const mavlink_message_t& m){
    uint8_t b[MAVLINK_MAX_PACKET_LEN]; const uint16_t n=mavlink_msg_to_send_buffer(b,&m); write_all(fd,b,n);
  }
  static void request_rate(int fd,uint8_t sys,uint8_t comp,uint32_t msgid,int hz){
    mavlink_message_t m{};
    mavlink_msg_command_long_pack(255,190,&m,sys,comp,MAV_CMD_SET_MESSAGE_INTERVAL,0,
      static_cast<float>(msgid),1000000.0f/static_cast<float>(hz),0,0,0,0,0);
    send_msg(fd,m);
  }
  void start(const std::string& dev,const std::string& out){
    fd=::open(dev.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK); if(fd<0) fail("open FC MAVLink");
    termios t{}; if(tcgetattr(fd,&t)<0) fail("FC tcgetattr"); cfmakeraw(&t);
    if(cfsetispeed(&t,B460800)||cfsetospeed(&t,B460800)) fail("FC baud");
    t.c_cflag|=CLOCAL|CREAD; t.c_cflag&=~CRTSCTS; t.c_cflag&=~PARENB; t.c_cflag&=~CSTOPB;
    t.c_cflag&=~CSIZE; t.c_cflag|=CS8; t.c_cc[VMIN]=0; t.c_cc[VTIME]=0;
    if(tcsetattr(fd,TCSANOW,&t)<0) fail("FC tcsetattr"); tcflush(fd,TCIFLUSH);
    imu_csv.open(out+"/imu.csv",std::ios::trunc); att_csv.open(out+"/attitude.csv",std::ios::trunc);
    if(!imu_csv||!att_csv) throw std::runtime_error("open FC csv");
    imu_csv<<"sample_id,recv_mono_ns,time_usec,xacc,yacc,zacc,xgyro,ygyro,zgyro,xmag,ymag,zmag,abs_pressure,diff_pressure,pressure_alt,temperature,fields_updated\n";
    att_csv<<"sample_id,recv_mono_ns,time_boot_ms,roll,pitch,yaw,rollspeed,pitchspeed,yawspeed\n";
    th=std::thread([this]{
      mavlink_status_t st{}; mavlink_message_t m{}; uint8_t target_sys=0,target_comp=0; uint8_t buf[8192];
      const uint64_t deadline=mono_ns()+10000000000ULL;
      while(running && !target_sys && mono_ns()<deadline){
        pollfd p{fd,POLLIN,0}; if(::poll(&p,1,100)<=0) continue; ssize_t n=::read(fd,buf,sizeof(buf)); if(n<=0) continue;
        for(ssize_t i=0;i<n;i++) if(mavlink_parse_char(MAVLINK_COMM_0,buf[i],&m,&st) && m.msgid==MAVLINK_MSG_ID_HEARTBEAT){
          target_sys=m.sysid; target_comp=m.compid; break;
        }
      }
      if(!target_sys){ std::cerr<<"R1 FC FAIL: HEARTBEAT timeout\n"; running=false; return; }
      request_rate(fd,target_sys,target_comp,MAVLINK_MSG_ID_HIGHRES_IMU,200);
      request_rate(fd,target_sys,target_comp,MAVLINK_MSG_ID_ATTITUDE,50);
      while(running){
        pollfd p{fd,POLLIN,0}; int pr=::poll(&p,1,50); if(pr<=0) continue;
        for(;;){ ssize_t n=::read(fd,buf,sizeof(buf)); if(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK)) break; if(n<=0) break;
          for(ssize_t i=0;i<n;i++) if(mavlink_parse_char(MAVLINK_COMM_0,buf[i],&m,&st)){
            const uint64_t recv=mono_ns();
            if(m.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){
              mavlink_highres_imu_t x{}; mavlink_msg_highres_imu_decode(&m,&x); auto id=++imu_samples;
              imu_csv<<id<<','<<recv<<','<<x.time_usec<<','<<x.xacc<<','<<x.yacc<<','<<x.zacc<<','<<x.xgyro<<','<<x.ygyro<<','<<x.zgyro<<','<<x.xmag<<','<<x.ymag<<','<<x.zmag<<','<<x.abs_pressure<<','<<x.diff_pressure<<','<<x.pressure_alt<<','<<x.temperature<<','<<x.fields_updated<<'\n';
            } else if(m.msgid==MAVLINK_MSG_ID_ATTITUDE){
              mavlink_attitude_t x{}; mavlink_msg_attitude_decode(&m,&x); auto id=++att_samples;
              att_csv<<id<<','<<recv<<','<<x.time_boot_ms<<','<<x.roll<<','<<x.pitch<<','<<x.yaw<<','<<x.rollspeed<<','<<x.pitchspeed<<','<<x.yawspeed<<'\n';
            }
          }
        }
      }
    });
  }
  void stop(){ if(th.joinable()) th.join(); if(fd>=0){::close(fd);fd=-1;} if(imu_csv.is_open())imu_csv.close(); if(att_csv.is_open())att_csv.close(); }
};

struct Luna {
  int fd=-1; std::thread th; std::ofstream csv; std::atomic<uint64_t> samples{0};
  ~Luna(){ stop(); }
  void start(const std::string& dev,const std::string& path){
    fd=::open(dev.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK);
    if(fd<0) fail("open TF-Luna");
    termios tty{}; if(tcgetattr(fd,&tty)<0) fail("tcgetattr");
    cfmakeraw(&tty); cfsetispeed(&tty,B115200); cfsetospeed(&tty,B115200);
    tty.c_cflag|=CLOCAL|CREAD; tty.c_cflag&=~CSTOPB; tty.c_cflag&=~CRTSCTS;
    if(tcsetattr(fd,TCSANOW,&tty)<0) fail("tcsetattr");
    csv.open(path,std::ios::trunc); if(!csv) throw std::runtime_error("open range csv");
    csv<<"sample_id,recv_mono_ns,distance_cm,strength,temp_raw,valid\n";
    th=std::thread([this]{
      std::vector<uint8_t> q; q.reserve(256); uint8_t tmp[128];
      while(running){
        pollfd p{fd,POLLIN,0}; int pr=::poll(&p,1,50);
        if(pr<=0) continue;
        ssize_t n=::read(fd,tmp,sizeof(tmp)); if(n<=0) continue;
        q.insert(q.end(),tmp,tmp+n);
        while(q.size()>=9){
          size_t s=0; while(s+1<q.size() && !(q[s]==0x59 && q[s+1]==0x59)) ++s;
          if(s){q.erase(q.begin(),q.begin()+s); if(q.size()<9) break;}
          unsigned sum=0; for(int i=0;i<8;i++) sum+=q[i];
          bool ok=((sum&0xff)==q[8]);
          uint16_t d=q[2]|(uint16_t(q[3])<<8), st=q[4]|(uint16_t(q[5])<<8), tr=q[6]|(uint16_t(q[7])<<8);
          auto id=++samples; csv<<id<<','<<mono_ns()<<','<<d<<','<<st<<','<<tr<<','<<(ok?1:0)<<'\n';
          q.erase(q.begin(),q.begin()+9);
        }
      }
    });
  }
  void stop(){
    if(th.joinable()) th.join();
    if(fd>=0){::close(fd);fd=-1;}
    if(csv.is_open()) csv.close();
  }
};
}

int main(int argc,char** argv){
  const std::string cam=argc>1?argv[1]:"/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC762-video-index0";
  const std::string luna=argc>2?argv[2]:"/dev/ttyAMA2";
  const std::string out=argc>3?argv[3]:"/home/vio/r1_dataset";
  const std::string fc=argc>4?argv[4]:"/dev/ttyAMA0";
  std::signal(SIGINT,sig_handler); std::signal(SIGTERM,sig_handler);
  try{
    std::filesystem::create_directories(out);
    std::ofstream mjpg(out+"/frames.mjpg",std::ios::binary|std::ios::trunc);
    std::ofstream frames(out+"/frames.csv",std::ios::trunc);
    std::ofstream events(out+"/events.csv",std::ios::trunc);
    if(!mjpg||!frames||!events) throw std::runtime_error("cannot create recorder outputs");
    frames<<"frame_id,sequence,v4l2_sec,v4l2_usec,recv_mono_ns,offset,bytes\n";
    events<<"event_id,recv_mono_ns,event\n";

    Camera c; c.open_dev(cam);
    Luna l; l.start(luna,out+"/range.csv");
    FcMavlink f; f.start(fc,out);
    std::cout<<"R1 STANDALONE RECORDER\nCAM="<<cam<<"\nLUNA="<<luna<<"\nFC="<<fc<<"\nOUT="<<out
             <<"\nENTER #1 = MOVE_START, ENTER #2 = MOVE_END, Ctrl-C = stop\n";

    uint64_t id=0,off=0,event_id=0;
    int move_marks=0;
    while(running){
      pollfd pfds[2]={{c.fd,POLLIN,0},{STDIN_FILENO,POLLIN,0}};
      int pr=::poll(pfds,2,100);
      if(pr<0){if(errno==EINTR) continue; fail("poll recorder");}
      if(pfds[1].revents&POLLIN){
        char ibuf[64]; ssize_t n=::read(STDIN_FILENO,ibuf,sizeof(ibuf));
        if(n>0){
          for(ssize_t k=0;k<n;k++) if(ibuf[k]=='\n'){
            const uint64_t t=mono_ns();
            if(move_marks==0){ events<<++event_id<<','<<t<<",MOVE_START\n"; events.flush(); move_marks=1; std::cout<<"[R1 EVENT] MOVE_START "<<t<<"\n"; }
            else if(move_marks==1){ events<<++event_id<<','<<t<<",MOVE_END\n"; events.flush(); move_marks=2; std::cout<<"[R1 EVENT] MOVE_END "<<t<<"\n"; }
            else { std::cout<<"[R1 EVENT] extra ENTER ignored\n"; }
          }
        }
      }
      if(!(pfds[0].revents&POLLIN)) continue;
      for(;;){
        v4l2_buffer b{}; b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory=V4L2_MEMORY_MMAP;
        if(xioctl(c.fd,VIDIOC_DQBUF,&b)<0){
          if(errno==EAGAIN) break; fail("VIDIOC_DQBUF");
        }
        const uint64_t recv=mono_ns();
        mjpg.write(static_cast<const char*>(c.bufs[b.index].p),b.bytesused);
        if(!mjpg) throw std::runtime_error("MJPEG write failed");
        ++id;
        frames<<id<<','<<b.sequence<<','<<b.timestamp.tv_sec<<','<<b.timestamp.tv_usec<<','<<recv<<','<<off<<','<<b.bytesused<<'\n';
        off+=b.bytesused;
        if(xioctl(c.fd,VIDIOC_QBUF,&b)<0) fail("VIDIOC_QBUF");
      }
    }
    f.stop(); l.stop(); c.close(); mjpg.close(); frames.close(); events.close();
    std::cout<<"R1 RECORDER PASS frames="<<id<<" mjpeg_bytes="<<off<<" luna_samples="<<l.samples.load()
             <<" imu_samples="<<f.imu_samples.load()<<" attitude_samples="<<f.att_samples.load()<<" move_marks="<<move_marks<<"\n";
    return (id>0 && off>0 && l.samples.load()>0 && f.imu_samples.load()>0 && f.att_samples.load()>0)?0:5;
  }catch(const std::exception& e){
    running=false;
    std::cerr<<"R1 RECORDER FAIL: "<<e.what()<<"\n";
    return 1;
  }
}
