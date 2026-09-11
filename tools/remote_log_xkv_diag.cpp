// JT-Zero — захват DataFlash EKF-лога по MAVLink без SD-карты.
//
// Требования на FC перед запуском:
//   LOG_BACKEND_TYPE = 2   (MAVLink backend)
//   LOG_DISARMED     = 1   (логировать на DISARMED)
//   EK3_LOG_LEVEL    = 0   (ALL, чтобы писать XKV1/XKV2/XKT)
//   EK3_SRC1_YAW     = 0   (только для текущего gyro-bias gate теста)
// После изменения LOG_BACKEND_TYPE нужен reboot FC, потому что backend создаётся при init().
//
// Программа:
//   - ждёт FC HEARTBEAT;
//   - читает ключевые параметры и отказывается стартовать при неверной конфигурации;
//   - runtime выбирает SRC1;
//   - запускает AP_Logger_MAVLink через REMOTE_LOG_BLOCK_STATUS(START);
//   - держит synthetic OPTICAL_FLOW свежим 50 Гц;
//   - принимает REMOTE_LOG_DATA_BLOCK, ACK/NACK-ит блоки и пишет непрерывный .BIN на RPi;
//   - слушает EKF_STATUS_REPORT;
//   - по завершении отправляет STOP.

#include "ardupilotmega/mavlink.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <poll.h>
#include <string>
#include <termios.h>
#include <unistd.h>

namespace {
using Clock = std::chrono::steady_clock;
constexpr uint8_t kSelfSys = 191;
constexpr uint8_t kSelfComp = 198;
constexpr size_t kBlockSize = MAVLINK_MSG_REMOTE_LOG_DATA_BLOCK_FIELD_DATA_LEN;

uint64_t monoUs(){
    return (uint64_t)std::chrono::duration_cast<std::chrono::microseconds>(Clock::now().time_since_epoch()).count();
}

double elapsed(const Clock::time_point& t0){
    return std::chrono::duration<double>(Clock::now()-t0).count();
}

int openSerial(const std::string& dev){
    int fd = ::open(dev.c_str(), O_RDWR|O_NOCTTY|O_NONBLOCK);
    if(fd < 0) return -1;
    termios t{};
    if(tcgetattr(fd,&t)<0){::close(fd);return -1;}
    cfmakeraw(&t);
    cfsetispeed(&t,B460800); cfsetospeed(&t,B460800);
    t.c_cflag |= CLOCAL|CREAD;
    t.c_cflag &= ~CRTSCTS; t.c_cflag &= ~PARENB; t.c_cflag &= ~CSTOPB;
    t.c_cflag &= ~CSIZE; t.c_cflag |= CS8;
    if(tcsetattr(fd,TCSANOW,&t)<0){::close(fd);return -1;}
    tcflush(fd,TCIFLUSH);
    return fd;
}

bool writeMsg(int fd,const mavlink_message_t& m){
    uint8_t b[MAVLINK_MAX_PACKET_LEN];
    const uint16_t n=mavlink_msg_to_send_buffer(b,&m);
    size_t off=0;
    while(off<n){
        const ssize_t k=::write(fd,b+off,n-off);
        if(k>0){off+=(size_t)k;continue;}
        if(k<0&&errno==EINTR)continue;
        if(k<0&&(errno==EAGAIN||errno==EWOULDBLOCK)){
            pollfd p{fd,POLLOUT,0}; (void)::poll(&p,1,10); continue;
        }
        return false;
    }
    return true;
}

void sendHeartbeat(int fd){
    mavlink_message_t m{};
    mavlink_msg_heartbeat_pack(kSelfSys,kSelfComp,&m,
        MAV_TYPE_ONBOARD_CONTROLLER,MAV_AUTOPILOT_INVALID,0,0,MAV_STATE_ACTIVE);
    (void)writeMsg(fd,m);
}

void requestRate(int fd,uint8_t sys,uint8_t comp,uint32_t msgid,int hz){
    mavlink_message_t m{};
    mavlink_msg_command_long_pack(kSelfSys,kSelfComp,&m,sys,comp,
        MAV_CMD_SET_MESSAGE_INTERVAL,0,(float)msgid,1000000.0f/(float)hz,0,0,0,0,0);
    (void)writeMsg(fd,m);
}

void requestParam(int fd,uint8_t sys,uint8_t comp,const std::string& name){
    char id[16]{};
    std::memcpy(id,name.c_str(),std::min<size_t>(name.size(),sizeof(id)));
    mavlink_message_t m{};
    mavlink_msg_param_request_read_pack(kSelfSys,kSelfComp,&m,sys,comp,id,-1);
    (void)writeMsg(fd,m);
}

std::string paramName(const mavlink_param_value_t& p){
    size_t n=0; while(n<sizeof(p.param_id)&&p.param_id[n])++n;
    return std::string(p.param_id,p.param_id+n);
}

void selectSource1(int fd,uint8_t sys,uint8_t comp){
    mavlink_message_t m{};
    mavlink_msg_command_long_pack(kSelfSys,kSelfComp,&m,sys,comp,
        MAV_CMD_SET_EKF_SOURCE_SET,0,1,0,0,0,0,0,0);
    (void)writeMsg(fd,m);
}

void sendFlow(int fd,float rx,float ry,uint8_t q){
    mavlink_message_t m{};
    mavlink_msg_optical_flow_pack(kSelfSys,kSelfComp,&m,monoUs(),
        0,0,0,0.0f,0.0f,q,-1.0f,rx,ry);
    (void)writeMsg(fd,m);
}

void sendRemoteStatus(int fd,uint8_t sys,uint8_t comp,uint32_t seq,uint8_t status){
    mavlink_message_t m{};
    mavlink_msg_remote_log_block_status_pack(kSelfSys,kSelfComp,&m,
        sys,comp,seq,status);
    (void)writeMsg(fd,m);
}

std::string flagsText(uint16_t f){
    std::string s;
    auto add=[&](const char* n,bool v){if(!s.empty())s+=' ';s+=n;s+='=';s+=(v?'1':'0');};
    add("att",f&1); add("velH",f&2); add("velV",f&4); add("posRel",f&8);
    add("posAbs",f&16); add("posVAbs",f&32); add("posVAGL",f&64);
    add("constPos",f&128); add("predRel",f&256); add("predAbs",f&512); add("uninit",f&1024);
    return s;
}
}

int main(int argc,char** argv){
    const std::string dev=(argc>1)?argv[1]:"/dev/ttyAMA0";
    const std::string out=(argc>2)?argv[2]:"remote_ekf.bin";
    const double duration=(argc>3)?std::stod(argv[3]):45.0;

    int fd=openSerial(dev);
    if(fd<0){std::cerr<<"ОШИБКА: не удалось открыть "<<dev<<": "<<std::strerror(errno)<<"\n";return 2;}

    mavlink_status_t st{}; mavlink_message_t msg{}; uint8_t buf[4096];
    uint8_t sys=0,comp=0;
    const auto wait0=Clock::now();
    while(elapsed(wait0)<8.0&&!sys){
        pollfd p{fd,POLLIN,0}; if(::poll(&p,1,100)<=0)continue;
        const ssize_t n=::read(fd,buf,sizeof(buf)); if(n<=0)continue;
        for(ssize_t i=0;i<n;i++){
            if(!mavlink_parse_char(MAVLINK_COMM_0,buf[i],&msg,&st))continue;
            if(msg.msgid!=MAVLINK_MSG_ID_HEARTBEAT)continue;
            mavlink_heartbeat_t hb{};mavlink_msg_heartbeat_decode(&msg,&hb);
            if(hb.autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA){sys=msg.sysid;comp=msg.compid;break;}
        }
    }
    if(!sys){std::cerr<<"ОШИБКА: ArduPilot HEARTBEAT не найден\n";::close(fd);return 3;}
    std::cout<<"FC heartbeat sys="<<(int)sys<<" comp="<<(int)comp<<"\n";

    const char* params[]={"LOG_BACKEND_TYPE","LOG_DISARMED","LOG_MAV_BUFSIZE","LOG_MAV_RATEMAX","EK3_LOG_LEVEL","EK3_FLOW_USE","EK3_SRC1_VELXY","EK3_SRC1_YAW"};
    std::map<std::string,float> pv;
    for(const char* n:params)requestParam(fd,sys,comp,n);
    const auto pt0=Clock::now();
    while(elapsed(pt0)<4.0&&pv.size()<8){
        pollfd p{fd,POLLIN,0}; if(::poll(&p,1,100)<=0)continue;
        const ssize_t n=::read(fd,buf,sizeof(buf)); if(n<=0)continue;
        for(ssize_t i=0;i<n;i++){
            if(!mavlink_parse_char(MAVLINK_COMM_0,buf[i],&msg,&st))continue;
            if(msg.sysid!=sys||msg.msgid!=MAVLINK_MSG_ID_PARAM_VALUE)continue;
            mavlink_param_value_t q{};mavlink_msg_param_value_decode(&msg,&q);
            const std::string name=paramName(q);
            for(const char* wanted:params)if(name==wanted)pv[name]=q.param_value;
        }
    }

    std::cout<<"\n===== PARAMS =====\n"<<std::fixed<<std::setprecision(6);
    for(const char* n:params){auto it=pv.find(n);if(it==pv.end())std::cout<<n<<" = NO_RESPONSE\n";else std::cout<<n<<" = "<<it->second<<"\n";}

    auto get=[&](const char* n)->double{auto it=pv.find(n);return it==pv.end()?NAN:it->second;};
    bool config_ok=true;
    if(!std::isfinite(get("LOG_BACKEND_TYPE")) || ((int)std::lround(get("LOG_BACKEND_TYPE")) & 2)==0){
        std::cerr<<"ОШИБКА: MAVLink logger backend не активен. Нужен LOG_BACKEND_TYPE=2 и reboot FC.\n";config_ok=false;
    }
    if(!std::isfinite(get("LOG_DISARMED")) || (int)std::lround(get("LOG_DISARMED"))!=1){
        std::cerr<<"ОШИБКА: нужен LOG_DISARMED=1 для DISARMED bench-лога.\n";config_ok=false;
    }
    if(!std::isfinite(get("EK3_LOG_LEVEL")) || (int)std::lround(get("EK3_LOG_LEVEL"))!=0){
        std::cerr<<"ОШИБКА: нужен EK3_LOG_LEVEL=0 (ALL), иначе XKV1/XKV2 не гарантированы.\n";config_ok=false;
    }
    if(!std::isfinite(get("EK3_FLOW_USE")) || (int)std::lround(get("EK3_FLOW_USE"))!=1){std::cerr<<"ОШИБКА: EK3_FLOW_USE должен быть 1.\n";config_ok=false;}
    if(!std::isfinite(get("EK3_SRC1_VELXY")) || (int)std::lround(get("EK3_SRC1_VELXY"))!=5){std::cerr<<"ОШИБКА: EK3_SRC1_VELXY должен быть 5.\n";config_ok=false;}
    if(!std::isfinite(get("EK3_SRC1_YAW")) || (int)std::lround(get("EK3_SRC1_YAW"))!=0){std::cerr<<"ОШИБКА: для текущего gate-теста ожидается EK3_SRC1_YAW=0.\n";config_ok=false;}
    if(!config_ok){::close(fd);return 4;}

    std::ofstream ofs(out,std::ios::binary|std::ios::trunc);
    if(!ofs){std::cerr<<"ОШИБКА: не удалось создать "<<out<<"\n";::close(fd);return 5;}

    requestRate(fd,sys,comp,MAVLINK_MSG_ID_EKF_STATUS_REPORT,5);
    requestRate(fd,sys,comp,MAVLINK_MSG_ID_OPTICAL_FLOW,10);
    selectSource1(fd,sys,comp);
    sendFlow(fd,1.0e-6f,0.0f,0); // включить high-precision rad/s mode, но не дать valid sample
    sendHeartbeat(fd);
    sendRemoteStatus(fd,sys,comp,MAV_REMOTE_LOG_DATA_BLOCK_START,MAV_REMOTE_LOG_DATA_BLOCK_ACK);

    std::map<uint32_t,std::array<uint8_t,kBlockSize>> pending;
    uint32_t expected=0;
    uint64_t blocks_rx=0,blocks_written=0,dup=0,ekf_n=0,fc_of=0;
    uint16_t flags=0;
    bool relative=false;
    auto last_data=Clock::now();
    auto last_nack=Clock::now();
    auto next_flow=Clock::now();
    auto next_hb=Clock::now()+std::chrono::seconds(1);
    auto next_print=Clock::now()+std::chrono::seconds(5);
    const auto t0=Clock::now();

    while(elapsed(t0)<duration){
        const auto now=Clock::now();
        if(now>=next_flow){sendFlow(fd,0.0f,0.0f,255);next_flow+=std::chrono::milliseconds(20);}
        if(now>=next_hb){sendHeartbeat(fd);next_hb+=std::chrono::seconds(1);}

        pollfd p{fd,POLLIN,0};
        if(::poll(&p,1,5)>0){
            for(;;){
                const ssize_t n=::read(fd,buf,sizeof(buf));
                if(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK))break;
                if(n<=0)break;
                for(ssize_t i=0;i<n;i++){
                    if(!mavlink_parse_char(MAVLINK_COMM_0,buf[i],&msg,&st))continue;
                    if(msg.sysid!=sys)continue;
                    if(msg.msgid==MAVLINK_MSG_ID_REMOTE_LOG_DATA_BLOCK){
                        mavlink_remote_log_data_block_t q{};mavlink_msg_remote_log_data_block_decode(&msg,&q);
                        if(q.target_system!=kSelfSys||q.target_component!=kSelfComp)continue;
                        ++blocks_rx; last_data=Clock::now();
                        if(q.seqno<expected || pending.count(q.seqno)){++dup;}
                        else {std::array<uint8_t,kBlockSize> a{};std::memcpy(a.data(),q.data,kBlockSize);pending.emplace(q.seqno,a);}
                        sendRemoteStatus(fd,sys,comp,q.seqno,MAV_REMOTE_LOG_DATA_BLOCK_ACK);
                        for(;;){auto it=pending.find(expected);if(it==pending.end())break;ofs.write((const char*)it->second.data(),kBlockSize);pending.erase(it);++expected;++blocks_written;}
                    } else if(msg.msgid==MAVLINK_MSG_ID_EKF_STATUS_REPORT){
                        mavlink_ekf_status_report_t q{};mavlink_msg_ekf_status_report_decode(&msg,&q);flags=q.flags;++ekf_n;if((flags&8)&&!(flags&128))relative=true;
                    } else if(msg.msgid==MAVLINK_MSG_ID_OPTICAL_FLOW){++fc_of;}
                }
            }
        }

        if(!pending.empty() && pending.begin()->first>expected && std::chrono::duration<double>(now-last_nack).count()>0.2){
            sendRemoteStatus(fd,sys,comp,expected,MAV_REMOTE_LOG_DATA_BLOCK_NACK);last_nack=now;
        }
        if(blocks_rx==0 && elapsed(t0)>2.0 && std::chrono::duration<double>(now-last_data).count()>1.0){
            sendRemoteStatus(fd,sys,comp,MAV_REMOTE_LOG_DATA_BLOCK_START,MAV_REMOTE_LOG_DATA_BLOCK_ACK);last_data=now;
        }
        if(now>=next_print){
            std::cout<<std::fixed<<std::setprecision(1)
                     <<"t="<<elapsed(t0)<<"s blocks_rx="<<blocks_rx<<" written="<<blocks_written<<" pending="<<pending.size()
                     <<" bytes="<<(blocks_written*kBlockSize)
                     <<" EKF=0x"<<std::hex<<flags<<std::dec<<" ["<<flagsText(flags)<<"] FC_OF="<<fc_of<<"\n";
            next_print+=std::chrono::seconds(5);
        }
    }

    sendRemoteStatus(fd,sys,comp,MAV_REMOTE_LOG_DATA_BLOCK_STOP,MAV_REMOTE_LOG_DATA_BLOCK_ACK);
    ofs.flush(); ofs.close();
    ::close(fd);

    std::cout<<"\n===== VERDICT =====\n";
    std::cout<<"REMOTE_LOG_BLOCKS_RX="<<blocks_rx<<"\n";
    std::cout<<"REMOTE_LOG_BLOCKS_WRITTEN="<<blocks_written<<"\n";
    std::cout<<"REMOTE_LOG_DUPLICATES="<<dup<<"\n";
    std::cout<<"REMOTE_LOG_PENDING_GAPS="<<pending.size()<<"\n";
    std::cout<<"REMOTE_LOG_BYTES="<<(blocks_written*kBlockSize)<<"\n";
    std::cout<<"AID_RELATIVE_OBSERVED="<<(relative?"YES":"NO")<<"\n";
    std::cout<<"FINAL_EKF flags=0x"<<std::hex<<flags<<std::dec<<" ["<<flagsText(flags)<<"]\n";
    std::cout<<"BIN="<<out<<"\n";
    if(blocks_written==0)std::cout<<"INTERPRETATION: FC не начал MAVLink remote logging; проверить backend после reboot.\n";
    else if(!pending.empty())std::cout<<"INTERPRETATION: лог получен, но остались пропуски блоков; файл может быть неполным.\n";
    else std::cout<<"INTERPRETATION: непрерывный DataFlash BIN сохранён на RPi без SD-карты FC.\n";
    return blocks_written?0:6;
}
