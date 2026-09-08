// JT-ZERO v43: чтение параметров MAVLink logging backend.
// ТОЛЬКО ЧТЕНИЕ. Никакие параметры FC не изменяются.

#define main jtzero_unused_main_v43
#include "camera_imu_extrinsics_logger.cpp"
#undef main

namespace {
std::string pname43(const mavlink_param_value_t& p) {
  char b[17]{};
  std::memcpy(b,p.param_id,16);
  return std::string(b,strnlen(b,16));
}
bool wanted43(const std::string& n) {
  static const std::vector<std::string> exact = {
    "LOG_BACKEND_TYPE","LOG_DISARMED","LOG_MAV_BUFSIZE","LOG_MAV_RATEMAX",
    "LOG_BITMASK","INS_LOG_BAT_MASK","INS_LOG_BAT_OPT","INS_RAW_LOG_OPT",
    "SERIAL0_BAUD","SERIAL1_BAUD","SERIAL2_BAUD","SERIAL3_BAUD",
    "SERIAL4_BAUD","SERIAL5_BAUD","SERIAL6_BAUD","SERIAL7_BAUD",
    "SERIAL0_PROTOCOL","SERIAL1_PROTOCOL","SERIAL2_PROTOCOL","SERIAL3_PROTOCOL",
    "SERIAL4_PROTOCOL","SERIAL5_PROTOCOL","SERIAL6_PROTOCOL","SERIAL7_PROTOCOL"
  };
  return std::find(exact.begin(),exact.end(),n)!=exact.end();
}
void request43(int fd,uint8_t sys,uint8_t comp) {
  mavlink_message_t m{}; uint8_t b[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_param_request_list_pack(COMPANION_SYSID,COMPANION_COMPID,&m,sys,comp);
  auto n=mavlink_msg_to_send_buffer(b,&m);
  if(write(fd,b,n)!=(ssize_t)n) fail("PARAM_REQUEST_LIST write");
}
}

int main(){
  int fd=-1;
  try{
    fd=openSerial();
    std::cout<<"[MAV] ожидание HEARTBEAT...\n";
    mavlink_status_t st{}; mavlink_message_t msg{}; uint8_t sys=0,comp=0;
    int64_t deadline=monotonicNs()+10000000000LL;
    while(monotonicNs()<deadline && !sys){
      pollfd p{fd,POLLIN,0};
      if(poll(&p,1,100)>0){
        uint8_t buf[4096]; ssize_t n=read(fd,buf,sizeof(buf));
        for(ssize_t i=0;i<n;++i){
          if(mavlink_parse_char(MAVLINK_COMM_0,buf[i],&msg,&st) &&
             msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=msg.sysid; comp=msg.compid; break;}
        }
      }
    }
    if(!sys) throw std::runtime_error("HEARTBEAT timeout");
    std::cout<<"[MAV] FC sysid="<<(int)sys<<" compid="<<(int)comp<<"\n";
    request43(fd,sys,comp);

    std::map<std::string,double> vals;
    int expected=-1; int64_t last_rx=monotonicNs();
    deadline=last_rx+15000000000LL;
    std::memset(&st,0,sizeof(st));
    while(monotonicNs()<deadline){
      pollfd p{fd,POLLIN,0}; int rc=poll(&p,1,200);
      if(rc<=0){ if(monotonicNs()-last_rx>2000000000LL) break; continue; }
      uint8_t buf[8192]; ssize_t n=read(fd,buf,sizeof(buf));
      if(n<=0) continue;
      for(ssize_t i=0;i<n;++i){
        if(!mavlink_parse_char(MAVLINK_COMM_0,buf[i],&msg,&st)) continue;
        if(msg.msgid!=MAVLINK_MSG_ID_PARAM_VALUE) continue;
        mavlink_param_value_t pv{}; mavlink_msg_param_value_decode(&msg,&pv);
        last_rx=monotonicNs(); expected=pv.param_count;
        auto name=pname43(pv);
        if(wanted43(name)) vals[name]=pv.param_value;
      }
    }
    std::cout<<"\n================ MAVLINK LOGGING BACKEND v43 ================\n";
    std::cout<<"Всего параметров FC: "<<expected<<"\n";
    for(const auto& kv:vals)
      std::cout<<std::left<<std::setw(24)<<kv.first<<" = "<<std::setprecision(10)<<kv.second<<"\n";
    std::cout<<"\nREAD-ONLY: параметры FC не изменялись.\n";
    close(fd); return 0;
  }catch(const std::exception&e){
    std::cerr<<"[FATAL] "<<e.what()<<"\n";
    if(fd>=0) close(fd);
    return 1;
  }
}
