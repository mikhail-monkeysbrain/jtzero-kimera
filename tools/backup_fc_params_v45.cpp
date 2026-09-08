// JT-ZERO v45: полный резервный снимок PARAM_VALUE FC.
// READ-ONLY: не отправляет PARAM_SET и не изменяет FC.
// Сохраняет имя, значение, MAV_PARAM_TYPE и индекс каждого параметра.

#define main jtzero_unused_main_v45
#include "camera_imu_extrinsics_logger.cpp"
#undef main
#include <fstream>
#include <map>
#include <iomanip>

namespace {
std::string param_name45(const mavlink_param_value_t& p) {
    char b[17]{};
    std::memcpy(b, p.param_id, 16);
    return std::string(b, strnlen(b, 16));
}
void request_all45(int fd, uint8_t sys, uint8_t comp) {
    mavlink_message_t m{}; uint8_t b[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_param_request_list_pack(COMPANION_SYSID, COMPANION_COMPID, &m, sys, comp);
    const uint16_t n=mavlink_msg_to_send_buffer(b,&m);
    if(write(fd,b,n)!=(ssize_t)n) fail("PARAM_REQUEST_LIST write");
}
struct P45 { float value{}; uint8_t type{}; uint16_t index{}; uint16_t count{}; };
}

int main(int argc,char**argv) {
    const std::string out = argc>1 ? argv[1] : "/home/vio/jtzero_fc_params_v45.tsv";
    int fd=-1;
    try {
        fd=openSerial();
        std::cout<<"[MAV] ожидание HEARTBEAT...\n";
        mavlink_status_t st{}; mavlink_message_t msg{}; uint8_t sys=0,comp=0;
        int64_t deadline=monotonicNs()+10000000000LL;
        while(monotonicNs()<deadline && !sys) {
            pollfd p{fd,POLLIN,0}; if(poll(&p,1,100)<=0) continue;
            uint8_t buf[4096]; ssize_t n=read(fd,buf,sizeof(buf));
            for(ssize_t i=0;i<n;++i) if(mavlink_parse_char(MAVLINK_COMM_0,buf[i],&msg,&st) &&
                msg.msgid==MAVLINK_MSG_ID_HEARTBEAT) {sys=msg.sysid;comp=msg.compid;break;}
        }
        if(!sys) throw std::runtime_error("HEARTBEAT timeout");
        std::cout<<"[MAV] FC sysid="<<(int)sys<<" compid="<<(int)comp<<"\n";
        request_all45(fd,sys,comp);

        std::map<std::string,P45> vals; int expected=-1; int64_t last=monotonicNs();
        deadline=last+30000000000LL; std::memset(&st,0,sizeof(st));
        while(monotonicNs()<deadline) {
            pollfd p{fd,POLLIN,0}; int rc=poll(&p,1,200);
            if(rc<=0) { if(expected>0 && (int)vals.size()>=expected) break; if(monotonicNs()-last>3000000000LL) break; continue; }
            uint8_t buf[8192]; ssize_t n=read(fd,buf,sizeof(buf)); if(n<=0) continue;
            for(ssize_t i=0;i<n;++i) {
                if(!mavlink_parse_char(MAVLINK_COMM_0,buf[i],&msg,&st) || msg.msgid!=MAVLINK_MSG_ID_PARAM_VALUE) continue;
                mavlink_param_value_t pv{}; mavlink_msg_param_value_decode(&msg,&pv);
                last=monotonicNs(); expected=pv.param_count;
                vals[param_name45(pv)]={pv.param_value,pv.param_type,pv.param_index,pv.param_count};
            }
            if(expected>0 && (int)vals.size()>=expected) break;
        }
        if(expected<=0) throw std::runtime_error("PARAM_VALUE не получены");

        std::ofstream f(out);
        if(!f) throw std::runtime_error("не удалось открыть output");
        f<<"# JT-ZERO FC parameter backup v45\n";
        f<<"# READ-ONLY capture; sysid="<<(int)sys<<" compid="<<(int)comp<<" expected="<<expected<<" received="<<vals.size()<<"\n";
        f<<"name\tvalue\ttype\tindex\tcount\n";
        f<<std::setprecision(10);
        for(const auto& kv:vals) f<<kv.first<<"\t"<<kv.second.value<<"\t"<<(int)kv.second.type<<"\t"<<kv.second.index<<"\t"<<kv.second.count<<"\n";
        f.close();

        std::cout<<"\n================ BACKUP ПАРАМЕТРОВ FC v45 ================\n";
        std::cout<<"Ожидалось параметров : "<<expected<<"\n";
        std::cout<<"Получено уникальных  : "<<vals.size()<<"\n";
        std::cout<<"Файл                 : "<<out<<"\n";
        if((int)vals.size()!=expected)
            std::cout<<"ВНИМАНИЕ: backup НЕПОЛНЫЙ. Ничего не прошивать.\n";
        else
            std::cout<<"BACKUP COMPLETE: количество совпало.\n";
        std::cout<<"READ-ONLY: параметры FC не изменялись.\n";
        close(fd);
        return (int)vals.size()==expected ? 0 : 2;
    } catch(const std::exception&e) {
        std::cerr<<"[FATAL] "<<e.what()<<"\n"; if(fd>=0) close(fd); return 1;
    }
}
