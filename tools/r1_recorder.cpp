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
  std::signal(SIGINT,sig_handler); std::signal(SIGTERM,sig_handler);
  try{
    std::filesystem::create_directories(out);
    std::ofstream mjpg(out+"/frames.mjpg",std::ios::binary|std::ios::trunc);
    std::ofstream frames(out+"/frames.csv",std::ios::trunc);
    if(!mjpg||!frames) throw std::runtime_error("cannot create recorder outputs");
    frames<<"frame_id,sequence,v4l2_sec,v4l2_usec,recv_mono_ns,offset,bytes\n";

    Camera c; c.open_dev(cam);
    Luna l; l.start(luna,out+"/range.csv");
    std::cout<<"R1 STANDALONE RECORDER\nCAM="<<cam<<"\nLUNA="<<luna<<"\nOUT="<<out<<"\nCtrl-C to stop\n";

    uint64_t id=0,off=0;
    while(running){
      pollfd p{c.fd,POLLIN,0}; int pr=::poll(&p,1,100);
      if(pr<0){if(errno==EINTR) continue; fail("poll camera");}
      if(pr==0) continue;
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
    l.stop(); c.close(); mjpg.close(); frames.close();
    std::cout<<"R1 RECORDER PASS frames="<<id<<" mjpeg_bytes="<<off<<" luna_samples="<<l.samples.load()<<"\n";
    return (id>0 && off>0 && l.samples.load()>0)?0:5;
  }catch(const std::exception& e){
    running=false;
    std::cerr<<"R1 RECORDER FAIL: "<<e.what()<<"\n";
    return 1;
  }
}
