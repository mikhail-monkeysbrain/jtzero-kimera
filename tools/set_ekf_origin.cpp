// JT-Zero: установка локального EKF origin через MAVLink SET_GPS_GLOBAL_ORIGIN.
// Нужна для GPS-denied position aiding. Координаты задаются явно аргументами.

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <poll.h>
#include <string>
#include <termios.h>
#include <unistd.h>
#include "common/mavlink.h"

namespace {
using Clock=std::chrono::steady_clock;
constexpr uint8_t kSelfSys=191;
constexpr uint8_t kSelfComp=MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY;

uint64_t monoUs(){return (uint64_t)std::chrono::duration_cast<std::chrono::microseconds>(Clock::now().time_since_epoch()).count();}
double elapsed(const Clock::time_point&t0){return std::chrono::duration<double>(Clock::now()-t0).count();}

int openSerial(const std::string&dev){
  int fd=::open(dev.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK);if(fd<0)return -1;
  termios t{};if(tcgetattr(fd,&t)<0){::close(fd);return -1;}cfmakeraw(&t);
  cfsetispeed(&t,B460800);cfsetospeed(&t,B460800);t.c_cflag|=CLOCAL|CREAD;
  t.c_cflag&=~CRTSCTS;t.c_cflag&=~PARENB;t.c_cflag&=~CSTOPB;t.c_cflag&=~CSIZE;t.c_cflag|=CS8;
  if(tcsetattr(fd,TCSANOW,&t)<0){::close(fd);return -1;}tcflush(fd,TCIFLUSH);return fd;
}

bool writeMsg(int fd,const mavlink_message_t&m){
  uint8_t b[MAVLINK_MAX_PACKET_LEN];const uint16_t n=mavlink_msg_to_send_buffer(b,&m);size_t o=0;
  while(o<n){ssize_t k=::write(fd,b+o,n-o);if(k>0){o+=(size_t)k;continue;}if(k<0&&errno==EINTR)continue;
    if(k<0&&(errno==EAGAIN||errno==EWOULDBLOCK)){pollfd p{fd,POLLOUT,0};(void)::poll(&p,1,10);continue;}return false;}
  return true;
}

void sendOrigin(int fd,uint8_t target,double lat_deg,double lon_deg,double alt_m){
  const int32_t lat=(int32_t)std::llround(lat_deg*1.0e7);
  const int32_t lon=(int32_t)std::llround(lon_deg*1.0e7);
  const int32_t alt=(int32_t)std::llround(alt_m*1000.0);
  mavlink_message_t m{};
  mavlink_msg_set_gps_global_origin_pack(kSelfSys,kSelfComp,&m,target,lat,lon,alt,monoUs());
  (void)writeMsg(fd,m);
}
}

int main(int argc,char**argv){
  if(argc<5){
    std::cerr<<"Использование: "<<argv[0]<<" <fc_dev> <lat_deg> <lon_deg> <alt_m>\n";
    return 2;
  }
  const std::string dev=argv[1];
  const double lat=std::stod(argv[2]),lon=std::stod(argv[3]),alt=std::stod(argv[4]);
  if(!std::isfinite(lat)||!std::isfinite(lon)||!std::isfinite(alt)||lat<=-90||lat>=90||lon<=-180||lon>=180||(lat==0&&lon==0)){
    std::cerr<<"ОШИБКА: некорректный origin\n";return 2;
  }
  int fd=openSerial(dev);if(fd<0){std::cerr<<"ОШИБКА: open "<<dev<<": "<<std::strerror(errno)<<"\n";return 3;}

  mavlink_status_t st{};mavlink_message_t msg{};uint8_t buf[4096];uint8_t sys=0,comp=0;auto t0=Clock::now();
  while(elapsed(t0)<8.0&&!sys){pollfd p{fd,POLLIN,0};if(::poll(&p,1,100)<=0)continue;ssize_t n=::read(fd,buf,sizeof(buf));if(n<=0)continue;
    for(ssize_t i=0;i<n;i++){if(!mavlink_parse_char(MAVLINK_COMM_0,buf[i],&msg,&st))continue;if(msg.msgid!=MAVLINK_MSG_ID_HEARTBEAT)continue;
      mavlink_heartbeat_t hb{};mavlink_msg_heartbeat_decode(&msg,&hb);if(hb.autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA){sys=msg.sysid;comp=msg.compid;break;}}}
  if(!sys){std::cerr<<"ОШИБКА: heartbeat ArduPilot не найден\n";::close(fd);return 4;}
  std::cerr<<"FC heartbeat sys="<<(int)sys<<" comp="<<(int)comp<<"\n";

  // Несколько отправок: EKF может ещё завершать bootstrap после загрузки.
  auto start=Clock::now();auto next_tx=start;bool confirmed=false;int32_t got_lat=0,got_lon=0,got_alt=0;
  while(elapsed(start)<6.0){
    auto now=Clock::now();if(now>=next_tx){sendOrigin(fd,sys,lat,lon,alt);next_tx+=std::chrono::milliseconds(500);}
    pollfd p{fd,POLLIN,0};if(::poll(&p,1,50)<=0)continue;ssize_t n=::read(fd,buf,sizeof(buf));if(n<=0)continue;
    for(ssize_t i=0;i<n;i++){if(!mavlink_parse_char(MAVLINK_COMM_0,buf[i],&msg,&st))continue;if(msg.msgid==MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN&&msg.sysid==sys){
      mavlink_gps_global_origin_t o{};mavlink_msg_gps_global_origin_decode(&msg,&o);got_lat=o.latitude;got_lon=o.longitude;got_alt=o.altitude;confirmed=true;}}
    if(confirmed)break;
  }

  if(confirmed){
    std::cout<<"EKF ORIGIN CONFIRMED lat="<<(got_lat/1.0e7)<<" lon="<<(got_lon/1.0e7)<<" alt="<<(got_alt/1000.0)<<" m\n";
    ::close(fd);return 0;
  }
  std::cerr<<"ОШИБКА: FC не подтвердил GPS_GLOBAL_ORIGIN\n";::close(fd);return 5;
}
