// JT-ZERO Stage 11.9 v15.1: read-only FC estimator/compass/IMU parameter snapshot.
// No parameters are modified.

#define main jtzero_camera_imu_logger_unused_main_v151
#include "camera_imu_extrinsics_logger.cpp"
#undef main

namespace {

bool wanted151(const std::string& n) {
  static const std::vector<std::string> exact = {
    "AHRS_EKF_TYPE","AHRS_ORIENTATION",
    "EK3_ENABLE","EK3_PRIMARY","EK3_IMU_MASK",
    "EK3_SRC1_POSXY","EK3_SRC1_POSZ","EK3_SRC1_VELXY","EK3_SRC1_VELZ","EK3_SRC1_YAW",
    "EK3_SRC2_POSXY","EK3_SRC2_POSZ","EK3_SRC2_VELXY","EK3_SRC2_VELZ","EK3_SRC2_YAW",
    "EK3_SRC3_POSXY","EK3_SRC3_POSZ","EK3_SRC3_VELXY","EK3_SRC3_VELZ","EK3_SRC3_YAW",
    "COMPASS_ENABLE","COMPASS_USE","COMPASS_USE2","COMPASS_USE3",
    "COMPASS_PRIMARY","COMPASS_AUTO_ROT","COMPASS_ORIENT","COMPASS_ORIENT2","COMPASS_ORIENT3",
    "COMPASS_EXTERN2","COMPASS_EXTERN3",
    "INS_USE","INS_USE2","INS_USE3",
    "INS_GYR_ID","INS_GYR2_ID","INS_GYR3_ID",
    "INS_ACC_ID","INS_ACC2_ID","INS_ACC3_ID"
  };
  if (std::find(exact.begin(), exact.end(), n) != exact.end()) return true;
  if (n.rfind("EK3_MAG_",0)==0) return true;
  if (n.rfind("EK3_YAW_",0)==0) return true;
  if (n.rfind("COMPASS_DIA",0)==0 || n.rfind("COMPASS_ODI",0)==0 || n.rfind("COMPASS_OFS",0)==0) return true;
  return false;
}

void sendParamList151(int fd,uint8_t target_sys,uint8_t target_comp){
  mavlink_message_t m{};uint8_t b[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_param_request_list_pack(COMPANION_SYSID,COMPANION_COMPID,&m,target_sys,target_comp);
  uint16_t n=mavlink_msg_to_send_buffer(b,&m);
  if(write(fd,b,n)!=(ssize_t)n) fail("write PARAM_REQUEST_LIST");
}

std::string pname151(const mavlink_param_value_t& p){
  char b[17]{};std::memcpy(b,p.param_id,16);return std::string(b,strnlen(b,16));
}

} // namespace

int main(){
  int fd=-1;
  try{
    fd=openSerial();
    std::cout<<"[MAV] waiting for HEARTBEAT...\n";
    mavlink_status_t st{};mavlink_message_t msg{};uint8_t sys=0,comp=0;
    int64_t deadline=monotonicNs()+10000000000LL;
    while(monotonicNs()<deadline&&!sys){
      pollfd p{fd,POLLIN,0};
      if(poll(&p,1,100)>0){
        uint8_t buf[4096];ssize_t n=read(fd,buf,sizeof(buf));
        for(ssize_t i=0;i<n;++i) if(mavlink_parse_char(MAVLINK_COMM_0,buf[i],&msg,&st) && msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=msg.sysid;comp=msg.compid;break;}
      }
    }
    if(!sys) throw std::runtime_error("HEARTBEAT timeout");
    std::cout<<"[MAV] FC sysid="<<(int)sys<<" compid="<<(int)comp<<"\n";

    sendParamList151(fd,sys,comp);
    std::map<std::string,double> vals;
    int expected=-1;int64_t last_rx=monotonicNs();deadline=last_rx+15000000000LL;
    std::memset(&st,0,sizeof(st));
    while(monotonicNs()<deadline){
      pollfd p{fd,POLLIN,0};int rc=poll(&p,1,200);
      if(rc<=0){if(monotonicNs()-last_rx>2000000000LL)break;continue;}
      uint8_t buf[8192];ssize_t n=read(fd,buf,sizeof(buf));if(n<=0)continue;
      for(ssize_t i=0;i<n;++i){
        if(!mavlink_parse_char(MAVLINK_COMM_0,buf[i],&msg,&st))continue;
        if(msg.msgid!=MAVLINK_MSG_ID_PARAM_VALUE)continue;
        mavlink_param_value_t pv{};mavlink_msg_param_value_decode(&msg,&pv);last_rx=monotonicNs();expected=pv.param_count;
        std::string name=pname151(pv);if(wanted151(name))vals[name]=pv.param_value;
      }
    }

    std::cout<<"\n================ FC ESTIMATOR PARAMS V15.1 ================\n";
    std::cout<<"reported parameter count: "<<expected<<"\n";
    std::cout<<"selected parameters received: "<<vals.size()<<"\n\n";
    for(const auto& kv:vals)std::cout<<std::left<<std::setw(18)<<kv.first<<" = "<<std::setprecision(9)<<kv.second<<"\n";
    std::cout<<"\nREAD-ONLY RESULT: PASS\n";
    close(fd);return 0;
  }catch(const std::exception&e){std::cerr<<"[FATAL] "<<e.what()<<"\n";if(fd>=0)close(fd);return 1;}
}
