// JT-ZERO v44: чтение AUTOPILOT_VERSION через MAVLink.
// ТОЛЬКО ЧТЕНИЕ. Параметры и состояние FC не изменяются.

#define main jtzero_unused_main_v44
#include "camera_imu_extrinsics_logger.cpp"
#undef main

#include <iomanip>
#include <sstream>

namespace {
std::string hex_bytes44(const uint8_t* p, size_t n) {
  std::ostringstream s;
  s << std::hex << std::setfill('0');
  for (size_t i=0;i<n;++i) s << std::setw(2) << (unsigned)p[i];
  return s.str();
}
std::string printable44(const uint8_t* p, size_t n) {
  std::string s;
  for (size_t i=0;i<n && p[i];++i) {
    const unsigned char c=p[i];
    s += (c>=32 && c<=126) ? (char)c : '.';
  }
  return s;
}
void request44(int fd,uint8_t sys,uint8_t comp) {
  mavlink_message_t m{}; uint8_t b[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_command_long_pack(
      COMPANION_SYSID, COMPANION_COMPID, &m,
      sys, comp, MAV_CMD_REQUEST_MESSAGE, 0,
      (float)MAVLINK_MSG_ID_AUTOPILOT_VERSION, 0,0,0,0,0,0);
  const uint16_t n=mavlink_msg_to_send_buffer(b,&m);
  if(write(fd,b,n)!=(ssize_t)n) fail("MAV_CMD_REQUEST_MESSAGE write");
}
}

int main() {
  int fd=-1;
  try {
    fd=openSerial();
    std::cout<<"[MAV] ожидание HEARTBEAT...\n";
    mavlink_status_t st{}; mavlink_message_t msg{};
    uint8_t sys=0,comp=0;
    int64_t deadline=monotonicNs()+10000000000LL;
    while(monotonicNs()<deadline && !sys) {
      pollfd p{fd,POLLIN,0};
      if(poll(&p,1,100)<=0) continue;
      uint8_t buf[4096]; const ssize_t n=read(fd,buf,sizeof(buf));
      for(ssize_t i=0;i<n;++i) {
        if(mavlink_parse_char(MAVLINK_COMM_0,buf[i],&msg,&st) &&
           msg.msgid==MAVLINK_MSG_ID_HEARTBEAT) {
          sys=msg.sysid; comp=msg.compid; break;
        }
      }
    }
    if(!sys) throw std::runtime_error("HEARTBEAT timeout");
    std::cout<<"[MAV] FC sysid="<<(int)sys<<" compid="<<(int)comp<<"\n";

    request44(fd,sys,comp);
    deadline=monotonicNs()+5000000000LL;
    std::memset(&st,0,sizeof(st));

    while(monotonicNs()<deadline) {
      pollfd p{fd,POLLIN,0};
      if(poll(&p,1,200)<=0) continue;
      uint8_t buf[4096]; const ssize_t n=read(fd,buf,sizeof(buf));
      for(ssize_t i=0;i<n;++i) {
        if(!mavlink_parse_char(MAVLINK_COMM_0,buf[i],&msg,&st)) continue;
        if(msg.msgid==MAVLINK_MSG_ID_COMMAND_ACK) {
          mavlink_command_ack_t a{}; mavlink_msg_command_ack_decode(&msg,&a);
          if(a.command==MAV_CMD_REQUEST_MESSAGE)
            std::cout<<"[ACK] REQUEST_MESSAGE result="<<(int)a.result<<"\n";
        }
        if(msg.msgid!=MAVLINK_MSG_ID_AUTOPILOT_VERSION) continue;

        mavlink_autopilot_version_t v{};
        mavlink_msg_autopilot_version_decode(&msg,&v);
        const unsigned major=(v.flight_sw_version>>24)&0xff;
        const unsigned minor=(v.flight_sw_version>>16)&0xff;
        const unsigned patch=(v.flight_sw_version>>8)&0xff;
        const unsigned type=v.flight_sw_version&0xff;

        std::cout<<"\n================ ВЕРСИЯ FC v44 ================\n";
        std::cout<<"ArduPilot version        = "<<major<<"."<<minor<<"."<<patch
                 <<"  type="<<type<<"\n";
        std::cout<<"flight_sw_version raw    = "<<v.flight_sw_version<<"\n";
        std::cout<<"flight_custom_version    = "<<printable44(v.flight_custom_version,8)
                 <<"  hex="<<hex_bytes44(v.flight_custom_version,8)<<"\n";
        std::cout<<"middleware_sw_version    = "<<v.middleware_sw_version<<"\n";
        std::cout<<"os_sw_version            = "<<v.os_sw_version<<"\n";
        std::cout<<"board_version            = "<<v.board_version<<"\n";
        std::cout<<"vendor_id / product_id   = "<<v.vendor_id<<" / "<<v.product_id<<"\n";
        std::cout<<"capabilities             = "<<v.capabilities<<"\n";
        std::cout<<"uid                      = "<<v.uid<<"\n";
        std::cout<<"uid2 hex                 = "<<hex_bytes44(v.uid2,sizeof(v.uid2))<<"\n";
        std::cout<<"\nREAD-ONLY: FC не изменялся.\n";
        close(fd); return 0;
      }
    }
    throw std::runtime_error("AUTOPILOT_VERSION timeout");
  } catch(const std::exception&e) {
    std::cerr<<"[FATAL] "<<e.what()<<"\n";
    if(fd>=0) close(fd);
    return 1;
  }
}
