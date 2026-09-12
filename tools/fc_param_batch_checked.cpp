// JT-Zero — robust MAVLink PARAM read/set utility for MatekH743 on /dev/ttyAMA0.
// Intentionally does NOT wait for HEARTBEAT. The FC sysid/compid are explicit
// (JT-Zero bench/flight uses ArduPilot sys=1 comp=1) and PARAM_VALUE itself is
// the acknowledgement. Unrelated telemetry is simply ignored while preserving
// MAVLink parser state across all reads.
#include "ardupilotmega/mavlink.h"
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

static mavlink_status_t g_status{};

[[noreturn]] static void die(const std::string& s){
  std::cerr<<"ОШИБКА: "<<s<<"\n";
  std::exit(2);
}

static int open_serial(const std::string& dev,int baud){
  if(baud!=460800) die("эта утилита сейчас поддерживает baud=460800");
  int fd=::open(dev.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK);
  if(fd<0) die("open "+dev+": "+std::strerror(errno));

  termios t{};
  if(tcgetattr(fd,&t)<0) die("tcgetattr: "+std::string(std::strerror(errno)));
  cfmakeraw(&t);
  cfsetispeed(&t,B460800);
  cfsetospeed(&t,B460800);
  t.c_cflag|=CLOCAL|CREAD;
  t.c_cflag&=~CRTSCTS;
  t.c_cflag&=~PARENB;
  t.c_cflag&=~CSTOPB;
  t.c_cflag&=~CSIZE;
  t.c_cflag|=CS8;
  if(tcsetattr(fd,TCSANOW,&t)<0) die("tcsetattr: "+std::string(std::strerror(errno)));
  tcflush(fd,TCIFLUSH);
  return fd;
}

static void write_all(int fd,const uint8_t* p,size_t n){
  size_t off=0;
  while(off<n){
    ssize_t k=::write(fd,p+off,n-off);
    if(k>0){ off+=(size_t)k; continue; }
    if(k<0 && (errno==EAGAIN || errno==EWOULDBLOCK)){
      pollfd q{fd,POLLOUT,0};
      poll(&q,1,20);
      continue;
    }
    if(k<0 && errno==EINTR) continue;
    die("serial write: "+std::string(std::strerror(errno)));
  }
}

static std::string param_id_string(const mavlink_param_value_t& q){
  char id[17]{};
  std::memcpy(id,q.param_id,16);
  return std::string(id);
}

static bool wait_param_value(
    int fd,uint8_t target_sys,const std::string& wanted,
    mavlink_param_value_t* out,int timeout_ms)
{
  const auto mono_us=[]() -> int64_t {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC,&ts);
    return (int64_t)ts.tv_sec*1000000LL + ts.tv_nsec/1000;
  };
  const int64_t end_us = mono_us() + (int64_t)timeout_ms*1000LL;

  uint8_t buf[2048];
  while(true){
    const int64_t now_us=mono_us();
    if(now_us>=end_us) return false;

    int remain_ms=(int)std::max<int64_t>(1,(end_us-now_us)/1000);
    pollfd p{fd,POLLIN,0};
    int pr=poll(&p,1,std::min(remain_ms,50));
    if(pr<0){
      if(errno==EINTR) continue;
      die("poll: "+std::string(std::strerror(errno)));
    }
    if(pr==0) continue;

    for(;;){
      ssize_t n=::read(fd,buf,sizeof(buf));
      if(n<0 && (errno==EAGAIN || errno==EWOULDBLOCK)) break;
      if(n<0 && errno==EINTR) continue;
      if(n<=0) break;

      for(ssize_t i=0;i<n;i++){
        mavlink_message_t m{};
        if(!mavlink_parse_char(MAVLINK_COMM_0,buf[i],&m,&g_status)) continue;
        if(m.sysid!=target_sys) continue;
        if(m.msgid!=MAVLINK_MSG_ID_PARAM_VALUE) continue;

        mavlink_param_value_t q{};
        mavlink_msg_param_value_decode(&m,&q);
        if(param_id_string(q)==wanted){
          *out=q;
          return true;
        }
      }
    }
  }
}

static void send_param_request(
    int fd,uint8_t target_sys,uint8_t target_comp,const std::string& name)
{
  mavlink_message_t m{};
  mavlink_msg_param_request_read_pack(
      191,199,&m,target_sys,target_comp,name.c_str(),-1);
  uint8_t b[MAVLINK_MAX_PACKET_LEN];
  auto n=mavlink_msg_to_send_buffer(b,&m);
  write_all(fd,b,n);
}

static bool read_param(
    int fd,uint8_t target_sys,uint8_t target_comp,const std::string& name,
    mavlink_param_value_t* out)
{
  // Retry the request. This is deliberate: serial telemetry may be busy and
  // PARAM_REQUEST_READ is not a reliable transport by itself.
  for(int attempt=0;attempt<5;attempt++){
    send_param_request(fd,target_sys,target_comp,name);
    if(wait_param_value(fd,target_sys,name,out,900)) return true;
  }
  return false;
}

static bool set_param_checked(
    int fd,uint8_t target_sys,uint8_t target_comp,
    const std::string& name,double value)
{
  mavlink_param_value_t cur{};
  if(!read_param(fd,target_sys,target_comp,name,&cur)){
    std::cerr<<"ОШИБКА: параметр не прочитан перед записью: "<<name<<"\n";
    return false;
  }

  mavlink_message_t m{};
  mavlink_msg_param_set_pack(
      191,199,&m,target_sys,target_comp,name.c_str(),
      (float)value,cur.param_type);
  uint8_t b[MAVLINK_MAX_PACKET_LEN];
  auto n=mavlink_msg_to_send_buffer(b,&m);
  write_all(fd,b,n);

  // ArduPilot normally answers PARAM_SET with PARAM_VALUE. If that packet is
  // lost, explicit read-back below still verifies persistent value.
  mavlink_param_value_t got{};
  bool ok=false;
  for(int attempt=0;attempt<8;attempt++){
    if(attempt>0) send_param_request(fd,target_sys,target_comp,name);
    if(!wait_param_value(fd,target_sys,name,&got,700)) continue;
    const double tol=std::max(1e-6,std::abs(value)*1e-5);
    if(std::abs((double)got.param_value-value)<=tol){ ok=true; break; }
  }
  if(!ok){
    std::cerr<<"ОШИБКА: read-back "<<name
             <<" не подтвердил "<<value<<"\n";
    return false;
  }

  std::cout<<name<<": "<<cur.param_value<<" -> "<<got.param_value
           <<" VERIFIED\n";
  return true;
}

int main(int argc,char** argv){
  if(argc<6){
    std::cerr
      <<"Использование:\n"
      <<"  "<<argv[0]<<" <device> <baud> <sysid> <compid> read NAME [NAME...]\n"
      <<"  "<<argv[0]<<" <device> <baud> <sysid> <compid> apply-current-mount "
      <<"CAM_X CAM_Y CAM_Z LUNA_X LUNA_Y LUNA_Z\n";
    return 2;
  }

  const std::string dev=argv[1];
  const int baud=std::stoi(argv[2]);
  const uint8_t sys=(uint8_t)std::stoi(argv[3]);
  const uint8_t comp=(uint8_t)std::stoi(argv[4]);
  const std::string mode=argv[5];

  int fd=open_serial(dev,baud);
  std::cout<<"TARGET FC sys="<<(int)sys<<" comp="<<(int)comp<<"\n";

  if(mode=="read"){
    if(argc<7) die("read требует хотя бы одно имя параметра");
    for(int i=6;i<argc;i++){
      mavlink_param_value_t p{};
      if(!read_param(fd,sys,comp,argv[i],&p))
        die(std::string("параметр не прочитан: ")+argv[i]);
      std::cout<<argv[i]<<"="<<p.param_value<<"\n";
    }
    ::close(fd);
    return 0;
  }

  if(mode=="apply-current-mount"){
    if(argc!=12) die("apply-current-mount требует 6 чисел");

    std::cout<<"Проверка INS_POS1_*:\n";
    for(const char* name: {"INS_POS1_X","INS_POS1_Y","INS_POS1_Z"}){
      mavlink_param_value_t p{};
      if(!read_param(fd,sys,comp,name,&p))
        die(std::string("параметр не прочитан: ")+name);
      std::cout<<"  "<<name<<"="<<p.param_value<<"\n";
      if(std::abs((double)p.param_value)>0.005)
        die(std::string(name)+" не около нуля; сначала нужно определить body/CG reference");
    }

    const char* names[]={
      "FLOW_POS_X","FLOW_POS_Y","FLOW_POS_Z",
      "RNGFND1_POS_X","RNGFND1_POS_Y","RNGFND1_POS_Z"
    };
    for(int i=0;i<6;i++){
      double value=std::stod(argv[6+i]);
      if(!set_param_checked(fd,sys,comp,names[i],value)){
        ::close(fd);
        return 3;
      }
    }

    std::cout<<"\nOFFSETS APPLIED AND VERIFIED\n";
    ::close(fd);
    return 0;
  }

  die("неизвестный mode: "+mode);
}
