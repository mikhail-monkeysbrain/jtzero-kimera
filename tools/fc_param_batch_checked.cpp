// JT-Zero — batch MAVLink parameter read/set with read-back, no pymavlink.
// One serial session avoids Python/pymavlink instance-handling failures.
#include "ardupilotmega/mavlink.h"
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <iostream>
#include <map>
#include <string>
#include <vector>

static void die(const std::string& s){ std::cerr<<"ОШИБКА: "<<s<<"\n"; std::exit(2); }

static int open_serial(const std::string& dev,int baud){
  int fd=::open(dev.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK);
  if(fd<0) die("open "+dev+": "+std::strerror(errno));
  termios t{};
  if(tcgetattr(fd,&t)<0) die("tcgetattr");
  cfmakeraw(&t);
  speed_t sp=B460800;
  if(baud!=460800) die("поддерживается только baud=460800");
  cfsetispeed(&t,sp); cfsetospeed(&t,sp);
  t.c_cflag|=CLOCAL|CREAD;
  t.c_cflag&=~CRTSCTS; t.c_cflag&=~PARENB; t.c_cflag&=~CSTOPB;
  t.c_cflag&=~CSIZE; t.c_cflag|=CS8;
  if(tcsetattr(fd,TCSANOW,&t)<0) die("tcsetattr");
  tcflush(fd,TCIFLUSH);
  return fd;
}

static void write_all(int fd,const uint8_t* p,size_t n){
  size_t o=0;
  while(o<n){
    ssize_t k=::write(fd,p+o,n-o);
    if(k>0){o+=(size_t)k;continue;}
    if(k<0&&(errno==EAGAIN||errno==EWOULDBLOCK)){
      pollfd q{fd,POLLOUT,0}; poll(&q,1,20); continue;
    }
    if(k<0&&errno==EINTR)continue;
    die("serial write");
  }
}

static mavlink_status_t g_mav_status{};

static bool recv_msg(int fd,mavlink_message_t* out,double timeout_s){
  uint8_t buf[4096];
  const int loops=std::max(1,(int)std::ceil(timeout_s*20.0));
  for(int k=0;k<loops;k++){
    pollfd p{fd,POLLIN,0};
    if(poll(&p,1,50)<=0) continue;
    for(;;){
      ssize_t n=::read(fd,buf,sizeof(buf));
      if(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK))break;
      if(n<=0)break;
      for(ssize_t i=0;i<n;i++){
        if(mavlink_parse_char(MAVLINK_COMM_0,buf[i],out,&g_mav_status)) return true;
      }
    }
  }
  return false;
}

static bool wait_fc(int fd,uint8_t* sys,uint8_t* comp){
  const int64_t attempts=200;
  for(int i=0;i<attempts;i++){
    mavlink_message_t m{};
    if(!recv_msg(fd,&m,0.1))continue;
    if(m.msgid!=MAVLINK_MSG_ID_HEARTBEAT)continue;
    mavlink_heartbeat_t hb{}; mavlink_msg_heartbeat_decode(&m,&hb);
    if(hb.autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA){
      *sys=m.sysid; *comp=m.compid; return true;
    }
  }
  return false;
}

struct Param { float value=0; uint8_t type=0; };

static bool read_param(int fd,uint8_t sys,uint8_t comp,const std::string& name,Param* out,double timeout_s=3.0){
  mavlink_message_t req{};
  mavlink_msg_param_request_read_pack(191,199,&req,sys,comp,name.c_str(),-1);
  uint8_t b[MAVLINK_MAX_PACKET_LEN];
  auto n=mavlink_msg_to_send_buffer(b,&req); write_all(fd,b,n);

  const int loops=std::max(1,(int)std::ceil(timeout_s*20.0));
  for(int i=0;i<loops;i++){
    mavlink_message_t m{};
    if(!recv_msg(fd,&m,0.05))continue;
    if(m.sysid!=sys || m.msgid!=MAVLINK_MSG_ID_PARAM_VALUE)continue;
    mavlink_param_value_t q{}; mavlink_msg_param_value_decode(&m,&q);
    char id[17]{}; std::memcpy(id,q.param_id,16);
    if(name==std::string(id)){
      out->value=q.param_value; out->type=q.param_type; return true;
    }
  }
  return false;
}

static bool set_param_checked(int fd,uint8_t sys,uint8_t comp,const std::string& name,double value){
  Param cur{};
  if(!read_param(fd,sys,comp,name,&cur)) return false;
  mavlink_message_t msg{};
  mavlink_msg_param_set_pack(191,199,&msg,sys,comp,name.c_str(),(float)value,cur.type);
  uint8_t b[MAVLINK_MAX_PACKET_LEN];
  auto n=mavlink_msg_to_send_buffer(b,&msg); write_all(fd,b,n);

  for(int tries=0;tries<6;tries++){
    usleep(120000);
    Param got{};
    if(!read_param(fd,sys,comp,name,&got,0.8))continue;
    const double tol=std::max(1e-6,std::abs(value)*1e-5);
    if(std::abs((double)got.value-value)<=tol){
      std::cout<<name<<": "<<cur.value<<" -> "<<got.value<<" VERIFIED\n";
      return true;
    }
  }
  return false;
}

int main(int argc,char** argv){
  if(argc<4){
    std::cerr<<"Использование: "<<argv[0]<<" <device> <baud> <mode> ...\n"
             <<"  read NAME [NAME...]\n"
             <<"  apply-current-mount CAM_X CAM_Y CAM_Z LUNA_X LUNA_Y LUNA_Z\n";
    return 2;
  }
  std::string dev=argv[1]; int baud=std::stoi(argv[2]); std::string mode=argv[3];
  int fd=open_serial(dev,baud);
  uint8_t sys=0,comp=0;
  if(!wait_fc(fd,&sys,&comp)) die("HEARTBEAT ArduPilot не найден");
  std::cout<<"FC sys="<<(int)sys<<" comp="<<(int)comp<<"\n";

  if(mode=="read"){
    for(int i=4;i<argc;i++){
      Param p{};
      if(!read_param(fd,sys,comp,argv[i],&p)) die(std::string("параметр не прочитан: ")+argv[i]);
      std::cout<<argv[i]<<"="<<p.value<<"\n";
    }
    ::close(fd); return 0;
  }

  if(mode=="apply-current-mount"){
    if(argc!=10) die("apply-current-mount требует 6 чисел");
    const char* guard[]={"INS_POS1_X","INS_POS1_Y","INS_POS1_Z"};
    std::cout<<"Проверка INS_POS1_*:\n";
    for(const char* name:guard){
      Param p{};
      if(!read_param(fd,sys,comp,name,&p)) die(std::string("параметр не прочитан: ")+name);
      std::cout<<"  "<<name<<"="<<p.value<<"\n";
      if(std::abs((double)p.value)>0.005)
        die(std::string(name)+" не около нуля; sensor offsets нельзя трактовать напрямую относительно IMU");
    }
    const char* names[]={"FLOW_POS_X","FLOW_POS_Y","FLOW_POS_Z",
                         "RNGFND1_POS_X","RNGFND1_POS_Y","RNGFND1_POS_Z"};
    for(int i=0;i<6;i++){
      double v=std::stod(argv[4+i]);
      if(!set_param_checked(fd,sys,comp,names[i],v))
        die(std::string("не удалось записать/проверить ")+names[i]);
    }
    std::cout<<"\nOFFSETS APPLIED AND VERIFIED\n";
    ::close(fd); return 0;
  }

  die("неизвестный mode");
  return 2;
}
