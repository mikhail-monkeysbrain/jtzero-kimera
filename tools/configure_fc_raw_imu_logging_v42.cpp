// JT-ZERO v42: безопасное включение/восстановление batch raw-IMU logging.
// Меняет ТОЛЬКО INS_LOG_BAT_MASK и INS_LOG_BAT_OPT.
// Перед изменением сохраняет исходные значения в ~/jtzero_rawlog_v42_original.txt.

#define main jtzero_unused_main_v42
#include "camera_imu_extrinsics_logger.cpp"
#undef main

#include <fstream>
#include <map>

namespace {

std::string pname(const mavlink_param_value_t& p) {
  char b[17]{};
  std::memcpy(b, p.param_id, 16);
  return std::string(b, strnlen(b, 16));
}

void send_param_request_list(int fd, uint8_t sys, uint8_t comp) {
  mavlink_message_t m{}; uint8_t b[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_param_request_list_pack(COMPANION_SYSID, COMPANION_COMPID, &m, sys, comp);
  const uint16_t n = mavlink_msg_to_send_buffer(b, &m);
  if (write(fd, b, n) != (ssize_t)n) fail("PARAM_REQUEST_LIST write");
}

void send_param_set(int fd, uint8_t sys, uint8_t comp, const std::string& name, float value) {
  mavlink_message_t m{}; uint8_t b[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_param_set_pack(COMPANION_SYSID, COMPANION_COMPID, &m,
                             sys, comp, name.c_str(), value, MAV_PARAM_TYPE_REAL32);
  const uint16_t n = mavlink_msg_to_send_buffer(b, &m);
  if (write(fd, b, n) != (ssize_t)n) fail("PARAM_SET write");
}

std::map<std::string,double> read_params(int fd, uint8_t sys, uint8_t comp) {
  send_param_request_list(fd, sys, comp);
  std::map<std::string,double> vals;
  mavlink_status_t st{}; mavlink_message_t msg{};
  int64_t last_rx = monotonicNs();
  const int64_t deadline = last_rx + 15000000000LL;
  while (monotonicNs() < deadline) {
    pollfd p{fd, POLLIN, 0};
    int rc = poll(&p, 1, 200);
    if (rc <= 0) {
      if (monotonicNs() - last_rx > 2000000000LL) break;
      continue;
    }
    uint8_t buf[8192]; ssize_t n = read(fd, buf, sizeof(buf));
    if (n <= 0) continue;
    for (ssize_t i=0;i<n;++i) {
      if (!mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &st)) continue;
      if (msg.msgid != MAVLINK_MSG_ID_PARAM_VALUE) continue;
      mavlink_param_value_t pv{};
      mavlink_msg_param_value_decode(&msg, &pv);
      last_rx = monotonicNs();
      auto name = pname(pv);
      if (name=="INS_LOG_BAT_MASK" || name=="INS_LOG_BAT_OPT") vals[name]=pv.param_value;
    }
  }
  return vals;
}

std::pair<uint8_t,uint8_t> heartbeat(int fd) {
  mavlink_status_t st{}; mavlink_message_t msg{};
  const int64_t deadline = monotonicNs()+10000000000LL;
  while (monotonicNs()<deadline) {
    pollfd p{fd,POLLIN,0};
    if (poll(&p,1,100)<=0) continue;
    uint8_t buf[4096]; ssize_t n=read(fd,buf,sizeof(buf));
    for(ssize_t i=0;i<n;++i) {
      if (mavlink_parse_char(MAVLINK_COMM_0,buf[i],&msg,&st) &&
          msg.msgid==MAVLINK_MSG_ID_HEARTBEAT) return {msg.sysid,msg.compid};
    }
  }
  throw std::runtime_error("HEARTBEAT timeout");
}

std::string save_path() {
  const char* h = getenv("HOME");
  return std::string(h ? h : "/home/vio") + "/jtzero_rawlog_v42_original.txt";
}

void save_original(const std::map<std::string,double>& v) {
  std::ofstream f(save_path());
  if (!f) throw std::runtime_error("cannot write backup file");
  for (auto& kv:v) f << kv.first << " " << std::setprecision(12) << kv.second << "\n";
}

std::map<std::string,double> load_original() {
  std::ifstream f(save_path());
  if (!f) throw std::runtime_error("backup file not found: "+save_path());
  std::map<std::string,double> v; std::string n; double x;
  while (f>>n>>x) v[n]=x;
  return v;
}

void wait_echo(int fd, const std::string& target, double expected) {
  mavlink_status_t st{}; mavlink_message_t msg{};
  const int64_t deadline=monotonicNs()+4000000000LL;
  while(monotonicNs()<deadline) {
    pollfd p{fd,POLLIN,0};
    if(poll(&p,1,200)<=0) continue;
    uint8_t buf[4096]; ssize_t n=read(fd,buf,sizeof(buf));
    for(ssize_t i=0;i<n;++i) {
      if(!mavlink_parse_char(MAVLINK_COMM_0,buf[i],&msg,&st)) continue;
      if(msg.msgid!=MAVLINK_MSG_ID_PARAM_VALUE) continue;
      mavlink_param_value_t pv{}; mavlink_msg_param_value_decode(&msg,&pv);
      if(pname(pv)==target) {
        std::cout<<"[VERIFY] "<<target<<" = "<<pv.param_value<<"\n";
        if (std::fabs((double)pv.param_value-expected)>0.01)
          throw std::runtime_error("parameter verification failed for "+target);
        return;
      }
    }
  }
  throw std::runtime_error("no PARAM_VALUE echo for "+target);
}

}

int main(int argc,char**argv) {
  if(argc!=2 || (std::string(argv[1])!="--enable" && std::string(argv[1])!="--restore")) {
    std::cerr<<"Использование: "<<argv[0]<<" --enable | --restore\n";
    return 2;
  }
  int fd=-1;
  try {
    fd=openSerial();
    std::cout<<"[MAV] ожидание HEARTBEAT...\n";
    auto [sys,comp]=heartbeat(fd);
    std::cout<<"[MAV] FC sysid="<<(int)sys<<" compid="<<(int)comp<<"\n";

    const std::string mode=argv[1];
    if(mode=="--enable") {
      auto cur=read_params(fd,sys,comp);
      if(cur.size()!=2) throw std::runtime_error("required logging parameters not found");
      std::cout<<"Исходные значения:\n";
      for(auto& kv:cur) std::cout<<"  "<<kv.first<<" = "<<kv.second<<"\n";
      save_original(cur);
      std::cout<<"Backup: "<<save_path()<<"\n";

      // BAT_MASK bit0+bit1 = IMU1 + IMU2.
      // BAT_OPT bit0 = sensor-rate logging, i.e. pre-calibration sensor-rate path.
      send_param_set(fd,sys,comp,"INS_LOG_BAT_MASK",3.0f);
      wait_echo(fd,"INS_LOG_BAT_MASK",3.0);
      send_param_set(fd,sys,comp,"INS_LOG_BAT_OPT",1.0f);
      wait_echo(fd,"INS_LOG_BAT_OPT",1.0);

      std::cout<<"\nRAW batch logging ВКЛЮЧЕН для IMU1+IMU2.\n";
      std::cout<<"INS_LOG_BAT_MASK требует перезагрузки FC.\n";
      std::cout<<"После перезагрузки повторно проверь параметры v41 перед тестом.\n";
    } else {
      auto old=load_original();
      if(old.count("INS_LOG_BAT_MASK")!=1 || old.count("INS_LOG_BAT_OPT")!=1)
        throw std::runtime_error("backup incomplete");
      send_param_set(fd,sys,comp,"INS_LOG_BAT_MASK",(float)old["INS_LOG_BAT_MASK"]);
      wait_echo(fd,"INS_LOG_BAT_MASK",old["INS_LOG_BAT_MASK"]);
      send_param_set(fd,sys,comp,"INS_LOG_BAT_OPT",(float)old["INS_LOG_BAT_OPT"]);
      wait_echo(fd,"INS_LOG_BAT_OPT",old["INS_LOG_BAT_OPT"]);
      std::cout<<"\nИсходные параметры восстановлены. Для BAT_MASK нужна перезагрузка FC.\n";
    }

    close(fd);
    return 0;
  } catch(const std::exception& e) {
    std::cerr<<"[FATAL] "<<e.what()<<"\n";
    if(fd>=0) close(fd);
    return 1;
  }
}
