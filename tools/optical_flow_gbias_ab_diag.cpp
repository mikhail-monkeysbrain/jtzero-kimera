// JT-Zero — обратимый A/B-тест EK3_GBIAS_P_NSE для OpticalFlow AID_RELATIVE.
//
// Камера и TF-Luna не используются. Программа:
//  1) читает текущий EK3_GBIAS_P_NSE и ключевые source-параметры;
//  2) подтверждает baseline при штатном значении;
//  3) временно уменьшает EK3_GBIAS_P_NSE до тестового значения;
//  4) держит synthetic OPTICAL_FLOW свежим и следит за переходом в AID_RELATIVE;
//  5) ВСЕГДА пытается вернуть исходный EK3_GBIAS_P_NSE перед штатным выходом/CTRL-C.
//
// Это диагностический bench-тест. Тестовое значение не является рекомендацией для flight-конфигурации.

#include "ardupilotmega/mavlink.h"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
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
using Clock=std::chrono::steady_clock;
constexpr uint8_t kSelfSys=191;
constexpr uint8_t kSelfComp=MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY;
volatile std::sig_atomic_t gStop=0;
void onSignal(int){ gStop=1; }

uint64_t monoUs(){return (uint64_t)std::chrono::duration_cast<std::chrono::microseconds>(Clock::now().time_since_epoch()).count();}
double elapsed(const Clock::time_point&t0){return std::chrono::duration<double>(Clock::now()-t0).count();}

int openSerial(const std::string&dev){
    int fd=::open(dev.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK); if(fd<0)return -1;
    termios t{}; if(tcgetattr(fd,&t)<0){::close(fd);return -1;} cfmakeraw(&t);
    cfsetispeed(&t,B460800); cfsetospeed(&t,B460800); t.c_cflag|=CLOCAL|CREAD;
    t.c_cflag&=~CRTSCTS; t.c_cflag&=~PARENB; t.c_cflag&=~CSTOPB; t.c_cflag&=~CSIZE; t.c_cflag|=CS8;
    if(tcsetattr(fd,TCSANOW,&t)<0){::close(fd);return -1;} tcflush(fd,TCIFLUSH); return fd;
}

bool writeMsg(int fd,const mavlink_message_t&m){
    uint8_t b[MAVLINK_MAX_PACKET_LEN]; const uint16_t n=mavlink_msg_to_send_buffer(b,&m); size_t off=0;
    while(off<n){
        const ssize_t k=::write(fd,b+off,n-off);
        if(k>0){off+=(size_t)k;continue;} if(k<0&&errno==EINTR)continue;
        if(k<0&&(errno==EAGAIN||errno==EWOULDBLOCK)){pollfd p{fd,POLLOUT,0};(void)::poll(&p,1,10);continue;}
        return false;
    }
    return true;
}

void requestRate(int fd,uint8_t sys,uint8_t comp,uint32_t msgid,int hz){
    mavlink_message_t m{}; mavlink_msg_command_long_pack(kSelfSys,kSelfComp,&m,sys,comp,MAV_CMD_SET_MESSAGE_INTERVAL,0,(float)msgid,1000000.0f/(float)hz,0,0,0,0,0); (void)writeMsg(fd,m);
}
void requestParam(int fd,uint8_t sys,uint8_t comp,const std::string&name){
    char id[16]{}; std::memcpy(id,name.c_str(),std::min<size_t>(name.size(),sizeof(id)));
    mavlink_message_t m{}; mavlink_msg_param_request_read_pack(kSelfSys,kSelfComp,&m,sys,comp,id,-1); (void)writeMsg(fd,m);
}
void setParam(int fd,uint8_t sys,uint8_t comp,const std::string&name,float value){
    char id[16]{}; std::memcpy(id,name.c_str(),std::min<size_t>(name.size(),sizeof(id)));
    mavlink_message_t m{}; mavlink_msg_param_set_pack(kSelfSys,kSelfComp,&m,sys,comp,id,value,MAV_PARAM_TYPE_REAL32); (void)writeMsg(fd,m);
}
void selectSource1(int fd,uint8_t sys,uint8_t comp){
    mavlink_message_t m{}; mavlink_msg_command_long_pack(kSelfSys,kSelfComp,&m,sys,comp,MAV_CMD_SET_EKF_SOURCE_SET,0,1,0,0,0,0,0,0); (void)writeMsg(fd,m);
}
void sendFlow(int fd,float rx,float ry,uint8_t q){
    mavlink_message_t m{}; mavlink_msg_optical_flow_pack(kSelfSys,kSelfComp,&m,monoUs(),0,0,0,0.0f,0.0f,q,-1.0f,rx,ry); (void)writeMsg(fd,m);
}
std::string paramName(const mavlink_param_value_t&p){size_t n=0;while(n<sizeof(p.param_id)&&p.param_id[n])++n;return std::string(p.param_id,p.param_id+n);}
std::string flagsText(uint16_t f){
    std::string s; auto add=[&](const char*n,bool v){if(!s.empty())s+=' ';s+=n;s+='=';s+=(v?'1':'0');};
    add("att",f&1);add("velH",f&2);add("velV",f&4);add("posRel",f&8);add("posAbs",f&16);add("posVAbs",f&32);add("posVAGL",f&64);add("constPos",f&128);add("predRel",f&256);add("predAbs",f&512);add("uninit",f&1024);return s;
}

struct RxState{
    uint16_t flags=0; uint64_t ekf=0,fc_of=0,local=0; bool source_ack=false,relative=false; std::map<std::string,float> params;
};

void consume(int fd,uint8_t sys,mavlink_status_t&st,mavlink_message_t&msg,RxState&rs){
    uint8_t buf[4096];
    for(;;){
        const ssize_t n=::read(fd,buf,sizeof(buf));
        if(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK))break; if(n<=0)break;
        for(ssize_t i=0;i<n;i++){
            if(!mavlink_parse_char(MAVLINK_COMM_0,buf[i],&msg,&st))continue; if(msg.sysid!=sys)continue;
            if(msg.msgid==MAVLINK_MSG_ID_OPTICAL_FLOW){++rs.fc_of;}
            else if(msg.msgid==MAVLINK_MSG_ID_EKF_STATUS_REPORT){mavlink_ekf_status_report_t q{};mavlink_msg_ekf_status_report_decode(&msg,&q);rs.flags=q.flags;++rs.ekf;if((q.flags&8)&&!(q.flags&128))rs.relative=true;}
            else if(msg.msgid==MAVLINK_MSG_ID_LOCAL_POSITION_NED){++rs.local;}
            else if(msg.msgid==MAVLINK_MSG_ID_PARAM_VALUE){mavlink_param_value_t q{};mavlink_msg_param_value_decode(&msg,&q);rs.params[paramName(q)]=q.param_value;}
            else if(msg.msgid==MAVLINK_MSG_ID_COMMAND_ACK){mavlink_command_ack_t q{};mavlink_msg_command_ack_decode(&msg,&q);if(q.command==MAV_CMD_SET_EKF_SOURCE_SET&&q.result==MAV_RESULT_ACCEPTED)rs.source_ack=true;}
        }
    }
}

bool waitParam(int fd,uint8_t sys,uint8_t comp,mavlink_status_t&st,mavlink_message_t&msg,RxState&rs,const std::string&name,float expected,double timeout){
    const auto t0=Clock::now(); auto nextReq=t0;
    while(elapsed(t0)<timeout&&!gStop){
        const auto now=Clock::now(); if(now>=nextReq){requestParam(fd,sys,comp,name);nextReq=now+std::chrono::milliseconds(300);}
        pollfd p{fd,POLLIN,0}; if(::poll(&p,1,50)>0)consume(fd,sys,st,msg,rs);
        auto it=rs.params.find(name); if(it!=rs.params.end()&&std::fabs(it->second-expected)<=std::max(1e-8f,std::fabs(expected)*0.02f))return true;
    }
    return false;
}

void runFlowPhase(int fd,uint8_t sys,mavlink_status_t&st,mavlink_message_t&msg,RxState&rs,double seconds,const std::string&label){
    const auto t0=Clock::now(); auto nextTx=t0; auto nextPrint=t0+std::chrono::seconds(5); bool printedChange=false;
    std::cout<<"\n===== "<<label<<" =====\n";
    while(elapsed(t0)<seconds&&!gStop){
        const auto now=Clock::now(); if(now>=nextTx){sendFlow(fd,0,0,255);nextTx+=std::chrono::milliseconds(20);}
        pollfd p{fd,POLLIN,0}; if(::poll(&p,1,5)>0)consume(fd,sys,st,msg,rs);
        if(rs.relative&&!printedChange){
            std::cout<<"AID_RELATIVE_OBSERVED at t="<<std::fixed<<std::setprecision(3)<<elapsed(t0)<<"s flags=0x"<<std::hex<<rs.flags<<std::dec<<" ["<<flagsText(rs.flags)<<"]\n";
            printedChange=true;
        }
        if(now>=nextPrint){
            std::cout<<std::fixed<<std::setprecision(1)<<"t="<<elapsed(t0)<<"s FC_OF="<<rs.fc_of<<" EKF="<<rs.ekf<<" flags=0x"<<std::hex<<rs.flags<<std::dec<<" ["<<flagsText(rs.flags)<<"] LOCAL="<<rs.local<<"\n";
            nextPrint+=std::chrono::seconds(5);
        }
        if(printedChange&&elapsed(t0)>3.0)break;
    }
}
}

int main(int argc,char**argv){
    std::signal(SIGINT,onSignal); std::signal(SIGTERM,onSignal);
    const std::string dev=(argc>1)?argv[1]:"/dev/ttyAMA0";
    const float testValue=(argc>2)?std::stof(argv[2]):0.0001f;
    const double testSeconds=(argc>3)?std::stod(argv[3]):90.0;
    int fd=openSerial(dev); if(fd<0){std::cerr<<"ОШИБКА: не удалось открыть "<<dev<<": "<<std::strerror(errno)<<"\n";return 2;}

    mavlink_status_t st{};mavlink_message_t msg{};uint8_t buf[4096];uint8_t sys=0,comp=0;
    const auto wh=Clock::now();
    while(elapsed(wh)<8&&!sys){pollfd p{fd,POLLIN,0};if(::poll(&p,1,100)<=0)continue;const ssize_t n=::read(fd,buf,sizeof(buf));if(n<=0)continue;for(ssize_t i=0;i<n;i++){if(!mavlink_parse_char(MAVLINK_COMM_0,buf[i],&msg,&st))continue;if(msg.msgid!=MAVLINK_MSG_ID_HEARTBEAT)continue;mavlink_heartbeat_t hb{};mavlink_msg_heartbeat_decode(&msg,&hb);if(hb.autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA){sys=msg.sysid;comp=msg.compid;break;}}}
    if(!sys){std::cerr<<"ОШИБКА: ArduPilot HEARTBEAT не найден\n";::close(fd);return 3;}
    std::cout<<"FC heartbeat sys="<<(int)sys<<" comp="<<(int)comp<<"\n";

    requestRate(fd,sys,comp,MAVLINK_MSG_ID_OPTICAL_FLOW,10);requestRate(fd,sys,comp,MAVLINK_MSG_ID_EKF_STATUS_REPORT,5);requestRate(fd,sys,comp,MAVLINK_MSG_ID_LOCAL_POSITION_NED,10);
    RxState rs; const char* names[]={"EK3_GBIAS_P_NSE","EK3_FLOW_USE","EK3_SRC1_VELXY","EK3_SRC1_YAW","EK3_IMU_MASK"}; for(const char*n:names)requestParam(fd,sys,comp,n);
    selectSource1(fd,sys,comp);sendFlow(fd,1e-6f,0,0);

    const auto pr=Clock::now();while(elapsed(pr)<3&&!gStop){pollfd p{fd,POLLIN,0};if(::poll(&p,1,100)>0)consume(fd,sys,st,msg,rs);if(rs.params.size()>=5&&rs.source_ack)break;}
    std::cout<<std::setprecision(9);
    for(const char*n:names){auto it=rs.params.find(n);std::cout<<n<<" = "<<(it==rs.params.end()?std::string("NO_RESPONSE"):std::to_string(it->second))<<"\n";}
    auto itOrig=rs.params.find("EK3_GBIAS_P_NSE");if(itOrig==rs.params.end()){std::cerr<<"ОШИБКА: не прочитан EK3_GBIAS_P_NSE; тест отменён\n";::close(fd);return 4;}
    const float original=itOrig->second;
    if(rs.params.count("EK3_SRC1_YAW")&&std::fabs(rs.params["EK3_SRC1_YAW"])>0.1f){std::cerr<<"ОШИБКА: для этого A/B-теста ожидается EK3_SRC1_YAW=0; параметр не менялся\n";::close(fd);return 5;}

    runFlowPhase(fd,sys,st,msg,rs,10.0,"BASELINE — исходный GBIAS_P_NSE");
    if(rs.relative){std::cout<<"\nBaseline уже вошёл в AID_RELATIVE; менять GBIAS не требуется.\n";::close(fd);return 0;}

    std::cout<<"\nTEMP PARAM: EK3_GBIAS_P_NSE "<<original<<" -> "<<testValue<<"\n";
    setParam(fd,sys,comp,"EK3_GBIAS_P_NSE",testValue);
    rs.params.erase("EK3_GBIAS_P_NSE");
    const bool setOk=waitParam(fd,sys,comp,st,msg,rs,"EK3_GBIAS_P_NSE",testValue,4.0);
    if(!setOk){std::cerr<<"ОШИБКА: тестовое значение не подтверждено; пытаюсь восстановить исходное\n";setParam(fd,sys,comp,"EK3_GBIAS_P_NSE",original);(void)waitParam(fd,sys,comp,st,msg,rs,"EK3_GBIAS_P_NSE",original,4.0);::close(fd);return 6;}

    rs.relative=false;
    runFlowPhase(fd,sys,st,msg,rs,testSeconds,"TEST — уменьшенный GBIAS_P_NSE");
    const bool relativeWithTest=rs.relative;

    std::cout<<"\nRESTORE: EK3_GBIAS_P_NSE -> "<<original<<"\n";
    setParam(fd,sys,comp,"EK3_GBIAS_P_NSE",original);rs.params.erase("EK3_GBIAS_P_NSE");
    const bool restoreOk=waitParam(fd,sys,comp,st,msg,rs,"EK3_GBIAS_P_NSE",original,5.0);

    std::cout<<"\n===== VERDICT =====\n";
    std::cout<<"TEST_VALUE="<<testValue<<"\n";
    std::cout<<"AID_RELATIVE_WITH_TEST_VALUE="<<(relativeWithTest?"YES":"NO")<<"\n";
    std::cout<<"ORIGINAL_GBIAS_RESTORED="<<(restoreOk?"YES":"NO")<<" value="<<original<<"\n";
    std::cout<<"FINAL_EKF flags=0x"<<std::hex<<rs.flags<<std::dec<<" ["<<flagsText(rs.flags)<<"]\n";
    if(relativeWithTest)std::cout<<"INTERPRETATION: снижение gyro-bias process noise изменило прохождение gate. Это диагностический результат, НЕ готовая flight-настройка.\n";
    else std::cout<<"INTERPRETATION: десятикратное снижение GBIAS_P_NSE не открыло AID_RELATIVE за окно теста; гипотеза о process-noise floor не подтверждена этим экспериментом.\n";
    if(!restoreOk)std::cout<<"ВНИМАНИЕ: автоматическое восстановление не подтверждено. Перед дальнейшими тестами вручную вернуть EK3_GBIAS_P_NSE="<<original<<".\n";
    ::close(fd);return restoreOk?0:7;
}
