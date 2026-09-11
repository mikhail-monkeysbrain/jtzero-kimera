// JT-Zero — диагностика последнего gate перед AID_RELATIVE для OpticalFlow.
//
// Камера и TF-Luna не используются. Тест держит synthetic OPTICAL_FLOW свежим,
// принудительно выбирает SRC1, читает EKF status и одновременно собирает статистику
// стационарного gyro по SCALED_IMU/2/3 и параметры, влияющие на gyro-bias convergence.
//
// ВАЖНО: внутренний bool delAngBiasLearned напрямую через MAVLink не экспортируется.
// Если FLOW_USE/SRC/fresh-flow/tilt подтверждены, а AID_RELATIVE не начинается,
// этот bool остаётся последним условием readyToUseOptFlow().

#include "ardupilotmega/mavlink.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
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
constexpr uint8_t kSelfComp = MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY;

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
    const uint16_t n = mavlink_msg_to_send_buffer(b,&m);
    size_t off=0;
    while(off<n){
        const ssize_t k=::write(fd,b+off,n-off);
        if(k>0){off+=(size_t)k;continue;}
        if(k<0 && errno==EINTR) continue;
        if(k<0 && (errno==EAGAIN||errno==EWOULDBLOCK)){
            pollfd p{fd,POLLOUT,0}; (void)::poll(&p,1,10); continue;
        }
        return false;
    }
    return true;
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

void selectSource1(int fd,uint8_t sys,uint8_t comp){
    mavlink_message_t m{};
    mavlink_msg_command_long_pack(kSelfSys,kSelfComp,&m,sys,comp,
        MAV_CMD_SET_EKF_SOURCE_SET,0,1,0,0,0,0,0,0);
    (void)writeMsg(fd,m);
}

void sendFlow(int fd,float rx,float ry,uint8_t quality){
    mavlink_message_t m{};
    mavlink_msg_optical_flow_pack(kSelfSys,kSelfComp,&m,monoUs(),
        0,0,0,0.0f,0.0f,quality,-1.0f,rx,ry);
    (void)writeMsg(fd,m);
}

std::string paramName(const mavlink_param_value_t& p){
    size_t n=0; while(n<sizeof(p.param_id)&&p.param_id[n]) ++n;
    return std::string(p.param_id,p.param_id+n);
}

std::string flagsText(uint16_t f){
    std::string s;
    auto add=[&](const char* n,bool v){if(!s.empty())s+=' ';s+=n;s+='=';s+=(v?'1':'0');};
    add("att",f&1); add("velH",f&2); add("velV",f&4); add("posRel",f&8);
    add("posAbs",f&16); add("posVAbs",f&32); add("posVAGL",f&64);
    add("constPos",f&128); add("predRel",f&256); add("predAbs",f&512); add("uninit",f&1024);
    return s;
}

struct AxisStats {
    uint64_t n=0;
    std::array<double,3> mean{0,0,0};
    std::array<double,3> m2{0,0,0};
    void add(double x,double y,double z){
        const std::array<double,3> v{x,y,z};
        ++n;
        for(int i=0;i<3;i++){
            const double d=v[i]-mean[i];
            mean[i]+=d/(double)n;
            const double d2=v[i]-mean[i];
            m2[i]+=d*d2;
        }
    }
    double sd(int i) const { return n>1?std::sqrt(m2[i]/(double)(n-1)):0.0; }
};

void printImuStats(const char* name,const AxisStats& s){
    constexpr double kRadToDeg=57.29577951308232;
    std::cout<<name<<" samples="<<s.n;
    if(!s.n){std::cout<<" NO_DATA\n";return;}
    std::cout<<std::fixed<<std::setprecision(6)
             <<" mean_rad_s=("<<s.mean[0]<<','<<s.mean[1]<<','<<s.mean[2]<<')'
             <<" sd_rad_s=("<<s.sd(0)<<','<<s.sd(1)<<','<<s.sd(2)<<')'
             <<" sd_deg_s=("<<s.sd(0)*kRadToDeg<<','<<s.sd(1)*kRadToDeg<<','<<s.sd(2)*kRadToDeg<<")\n";
}
}

int main(int argc,char** argv){
    const std::string dev=(argc>1)?argv[1]:"/dev/ttyAMA0";
    const double duration=(argc>2)?std::stod(argv[2]):30.0;
    int fd=openSerial(dev);
    if(fd<0){std::cerr<<"ОШИБКА: не удалось открыть "<<dev<<": "<<std::strerror(errno)<<"\n";return 2;}

    mavlink_status_t st{}; mavlink_message_t msg{}; uint8_t buf[4096];
    uint8_t sys=0,comp=0;
    const auto wait0=Clock::now();
    while(elapsed(wait0)<8.0 && !sys){
        pollfd p{fd,POLLIN,0}; if(::poll(&p,1,100)<=0) continue;
        const ssize_t n=::read(fd,buf,sizeof(buf)); if(n<=0) continue;
        for(ssize_t i=0;i<n;i++){
            if(!mavlink_parse_char(MAVLINK_COMM_0,buf[i],&msg,&st)) continue;
            if(msg.msgid!=MAVLINK_MSG_ID_HEARTBEAT) continue;
            mavlink_heartbeat_t hb{}; mavlink_msg_heartbeat_decode(&msg,&hb);
            if(hb.autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA){sys=msg.sysid;comp=msg.compid;break;}
        }
    }
    if(!sys){std::cerr<<"ОШИБКА: ArduPilot HEARTBEAT не найден\n";::close(fd);return 3;}
    std::cout<<"FC heartbeat sys="<<(int)sys<<" comp="<<(int)comp<<"\n";

    requestRate(fd,sys,comp,MAVLINK_MSG_ID_OPTICAL_FLOW,10);
    requestRate(fd,sys,comp,MAVLINK_MSG_ID_EKF_STATUS_REPORT,5);
    requestRate(fd,sys,comp,MAVLINK_MSG_ID_LOCAL_POSITION_NED,10);
    requestRate(fd,sys,comp,MAVLINK_MSG_ID_SCALED_IMU,50);
    requestRate(fd,sys,comp,MAVLINK_MSG_ID_SCALED_IMU2,50);
#ifdef MAVLINK_MSG_ID_SCALED_IMU3
    requestRate(fd,sys,comp,MAVLINK_MSG_ID_SCALED_IMU3,50);
#endif

    const char* params[]={
        "EK3_FLOW_USE","EK3_SRC_OPTIONS","EK3_SRC1_VELXY","EK3_SRC1_YAW",
        "EK3_GBIAS_P_NSE","EK3_GYRO_P_NSE","EK3_IMU_MASK","EK3_PRIMARY",
        "INS_GYR_CAL","INS_STILL_THRESH","INS_GYRO_FILTER","INS_USE","INS_USE2","INS_USE3"
    };
    for(const char* n:params) requestParam(fd,sys,comp,n);

    selectSource1(fd,sys,comp);
    // force MAV backend into high-precision flow_rate_x/y mode without giving EKF a valid aiding sample
    sendFlow(fd,1.0e-6f,0.0f,0);

    uint64_t tx=0,fc_of=0,ekf_n=0,local_n=0;
    bool source_ack=false,relative_seen=false;
    uint16_t last_flags=0;
    std::map<std::string,float> pv;
    AxisStats imu1,imu2,imu3;

    const auto t0=Clock::now();
    auto next_tx=Clock::now();
    auto next_print=Clock::now()+std::chrono::seconds(5);
    while(elapsed(t0)<duration){
        const auto now=Clock::now();
        if(now>=next_tx){
            sendFlow(fd,0.0f,0.0f,255); ++tx;
            next_tx += std::chrono::milliseconds(20);
        }

        pollfd p{fd,POLLIN,0};
        if(::poll(&p,1,5)>0){
            for(;;){
                const ssize_t n=::read(fd,buf,sizeof(buf));
                if(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK)) break;
                if(n<=0) break;
                for(ssize_t i=0;i<n;i++){
                    if(!mavlink_parse_char(MAVLINK_COMM_0,buf[i],&msg,&st)) continue;
                    if(msg.sysid!=sys) continue;
                    if(msg.msgid==MAVLINK_MSG_ID_OPTICAL_FLOW){
                        ++fc_of;
                    } else if(msg.msgid==MAVLINK_MSG_ID_EKF_STATUS_REPORT){
                        mavlink_ekf_status_report_t q{}; mavlink_msg_ekf_status_report_decode(&msg,&q);
                        last_flags=q.flags; ++ekf_n;
                        if((q.flags&8) && !(q.flags&128)) relative_seen=true;
                    } else if(msg.msgid==MAVLINK_MSG_ID_LOCAL_POSITION_NED){
                        ++local_n;
                    } else if(msg.msgid==MAVLINK_MSG_ID_PARAM_VALUE){
                        mavlink_param_value_t q{}; mavlink_msg_param_value_decode(&msg,&q);
                        const std::string n=paramName(q);
                        for(const char* wanted:params) if(n==wanted) pv[n]=q.param_value;
                    } else if(msg.msgid==MAVLINK_MSG_ID_COMMAND_ACK){
                        mavlink_command_ack_t q{}; mavlink_msg_command_ack_decode(&msg,&q);
                        if(q.command==MAV_CMD_SET_EKF_SOURCE_SET && q.result==MAV_RESULT_ACCEPTED) source_ack=true;
                    } else if(msg.msgid==MAVLINK_MSG_ID_SCALED_IMU){
                        mavlink_scaled_imu_t q{}; mavlink_msg_scaled_imu_decode(&msg,&q);
                        imu1.add(q.xgyro*0.001,q.ygyro*0.001,q.zgyro*0.001);
                    } else if(msg.msgid==MAVLINK_MSG_ID_SCALED_IMU2){
                        mavlink_scaled_imu2_t q{}; mavlink_msg_scaled_imu2_decode(&msg,&q);
                        imu2.add(q.xgyro*0.001,q.ygyro*0.001,q.zgyro*0.001);
#ifdef MAVLINK_MSG_ID_SCALED_IMU3
                    } else if(msg.msgid==MAVLINK_MSG_ID_SCALED_IMU3){
                        mavlink_scaled_imu3_t q{}; mavlink_msg_scaled_imu3_decode(&msg,&q);
                        imu3.add(q.xgyro*0.001,q.ygyro*0.001,q.zgyro*0.001);
#endif
                    }
                }
            }
        }

        if(now>=next_print){
            std::cout<<std::fixed<<std::setprecision(1)
                     <<"t="<<elapsed(t0)<<"s tx="<<tx<<" FC_OF="<<fc_of<<" EKF="<<ekf_n
                     <<" flags=0x"<<std::hex<<last_flags<<std::dec<<" ["<<flagsText(last_flags)<<"]"
                     <<" LOCAL="<<local_n
                     <<" IMU_samples="<<imu1.n<<'/'<<imu2.n<<'/'<<imu3.n<<"\n";
            next_print += std::chrono::seconds(5);
        }
    }

    std::cout<<"\n===== PARAMS =====\n";
    for(const char* n:params){
        const auto it=pv.find(n);
        if(it==pv.end()) std::cout<<n<<" = NO_RESPONSE\n";
        else std::cout<<std::setprecision(9)<<n<<" = "<<it->second<<"\n";
    }

    std::cout<<"\n===== STATIONARY GYRO =====\n";
    printImuStats("SCALED_IMU1",imu1);
    printImuStats("SCALED_IMU2",imu2);
    printImuStats("SCALED_IMU3",imu3);

    std::cout<<"\n===== VERDICT =====\n";
    std::cout<<"SOURCE_SET_PRIMARY_ACK="<<(source_ack?"YES":"NO")<<"\n";
    std::cout<<"FC_OPTICAL_FLOW_ECHO="<<(fc_of?"YES":"NO")<<"\n";
    std::cout<<"AID_RELATIVE_OBSERVED="<<(relative_seen?"YES":"NO")<<"\n";
    std::cout<<"FINAL_EKF flags=0x"<<std::hex<<last_flags<<std::dec<<" ["<<flagsText(last_flags)<<"]\n";
    if(!relative_seen){
        std::cout<<"NOTE: при подтверждённых FLOW_USE=1, SRC1_VELXY=5, fresh FC_OF и att=1\n"
                 <<"последним условием readyToUseOptFlow() остаётся delAngBiasLearned.\n"
                 <<"Этот внутренний bool напрямую MAVLink не экспортирует; статистика gyro ниже нужна для поиска причины его невыполнения.\n";
    }
    std::cout<<"tx_flow="<<tx<<" fc_of="<<fc_of<<" ekf="<<ekf_n<<" local="<<local_n<<"\n";
    ::close(fd);
    return 0;
}
