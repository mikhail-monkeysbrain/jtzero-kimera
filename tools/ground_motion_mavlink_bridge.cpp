// JT-Zero Ground Motion MVP: безопасный MAVLink bridge.
// Читает production CSV, публикует только valid=1 как ODOMETRY.
// Никаких параметров FC/EKF не изменяет.
#include "ground_motion_mavlink.hpp"
#include <fcntl.h>
#include <termios.h>
#include <poll.h>
#include <unistd.h>
#include <cerrno>
#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

static int openSerial(const std::string& dev) {
    int fd = ::open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return -1;
    termios t{};
    if (tcgetattr(fd, &t) < 0) { ::close(fd); return -1; }
    cfmakeraw(&t);
    cfsetispeed(&t, B460800); cfsetospeed(&t, B460800);
    t.c_cflag |= CLOCAL | CREAD; t.c_cflag &= ~CRTSCTS;
    if (tcsetattr(fd, TCSANOW, &t) < 0) { ::close(fd); return -1; }
    return fd;
}
static std::vector<std::string> split(const std::string&s){std::vector<std::string>v;std::stringstream ss(s);std::string x;while(std::getline(ss,x,','))v.push_back(x);return v;}
static uint64_t monoUs(){return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}

int main(int argc,char**argv){
    if(argc<3){std::cerr<<"Использование: "<<argv[0]<<" <mvp.csv> <fc_serial>\n";return 2;}
    const std::string csvpath=argv[1], fcdev=argv[2];
    int fd=openSerial(fcdev); if(fd<0){std::cerr<<"ОШИБКА: не удалось открыть "<<fcdev<<"\n";return 1;}
    std::ifstream f(csvpath); if(!f){std::cerr<<"ОШИБКА: не удалось открыть "<<csvpath<<"\n";return 1;}
    std::string line; std::getline(f,line); // header
    GroundMotionMavlinkPublisher pub;
    uint64_t sent=0, rejected=0; std::streampos pos=f.tellg();
    std::cerr<<"MAVLink bridge: ODOMETRY -> "<<fcdev<<" (EKF параметры НЕ меняются)\n";
    for(;;){
        if(std::getline(f,line)){
            auto c=split(line); if(c.size()<19){++rejected;continue;}
            try{
                bool valid=std::stoi(c[2])!=0; double q=std::stod(c[3]);
                double vx=std::stod(c[6]), vy=std::stod(c[7]);
                double x=std::stod(c[8]), y=std::stod(c[9]);
                if(pub.send(fd,monoUs(),valid,q,x,y,vx,vy)) ++sent; else ++rejected;
                if((sent+rejected)%100==0) std::cerr<<"ODOMETRY sent="<<sent<<" skipped="<<rejected<<"\r"<<std::flush;
            }catch(...){++rejected;}
            pos=f.tellg();
        }else{
            f.clear(); f.seekg(pos); std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}
