#include <linux/videodev2.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <termios.h>
#include <unistd.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "common/mavlink.h"
#include "camera_imu_timestamp_policy.hpp"

namespace {
constexpr const char* CAMERA_DEVICE="/dev/video0";
constexpr const char* SERIAL_DEVICE="/dev/ttyAMA0";
constexpr const char* OUTPUT_CSV="/home/vio/camera_imu_extrinsics.csv";
constexpr const char* OUTPUT_MJPEG="/dev/shm/camera_imu_extrinsics.mjpg";
constexpr const char* OUTPUT_CAMERA_INDEX="/dev/shm/camera_imu_extrinsics_camera.csv";
constexpr const char* PREVIEW_WINDOW="JT-Zero Guided Camera + IMU Calibration";
constexpr int CAMERA_WIDTH=640,CAMERA_HEIGHT=480,CAMERA_FPS=120,CAMERA_BUFFER_COUNT=6,CAMERA_WARMUP_FRAMES=30,PREVIEW_DIVIDER=4;
constexpr int IMU_RATE_HZ=200,ATTITUDE_RATE_HZ=50,EXPOSURE_ABSOLUTE=50,CAMERA_GAIN=0;
constexpr double MAX_TIMESYNC_RTT_MS=10.0,PI=3.14159265358979323846;
constexpr int64_t TIMESYNC_PERIOD_NS=100000000LL;
constexpr uint8_t COMPANION_SYSID=255,COMPANION_COMPID=190;
constexpr double ROLL_PITCH_TOL_DEG=5.0,YAW_TOL_DEG=7.0;
const std::string DEFAULT_PLAN="roll +/-30 hold 2; pitch +/-30 hold 2; yaw +/-60 hold 2; mixed 25 hold 2";

struct CameraBuffer{void*start=nullptr;size_t length=0;};
struct CameraSample{int64_t recv_ns=0,v4l2_ns=0,corrected_ns=0;uint32_t sequence=0,flags=0,bytes_used=0;uint64_t mjpeg_offset=0;};
struct ImuSample{int64_t recv_ns=0,fc_ns=0;float xacc=0,yacc=0,zacc=0,xgyro=0,ygyro=0,zgyro=0,temperature=0;uint16_t fields_updated=0;uint8_t imu_id=0;};
struct TimeSyncSample{int64_t t0_rpi_ns=0,t1_rpi_ns=0,fc_ns=0,rpi_mid_ns=0,rtt_ns=0;bool good=false;};
struct ClockMapping{bool valid=false;long double a=1.0L;int64_t fc_ref_ns=0;long double rpi_ref_ns=0;double drift_ppm=0;int64_t map(int64_t fc)const{return(int64_t)std::llround(rpi_ref_ns+a*(long double)(fc-fc_ref_ns));}};
struct Attitude{double roll_deg=0,pitch_deg=0,yaw_deg=0;bool valid=false;};
struct Step{double roll=0,pitch=0,yaw=0,hold_sec=2;std::string instruction;};
struct PreviewPacket{std::vector<unsigned char>jpeg;uint32_t sequence=0;};
struct PreviewState{
    std::mutex mutex;std::condition_variable cv;PreviewPacket latest;uint64_t generation=0;bool stopping=false;
    std::atomic<bool>stop_requested{false},plan_ready{false},attitude_valid{false},test_complete{false};
    std::atomic<double>roll_deg{0},pitch_deg{0},yaw_deg{0},yaw_zero_deg{0};
    std::atomic<int>step_index{0};std::atomic<double>step_collected_sec{0};
    std::string plan_text=DEFAULT_PLAN;std::vector<Step>steps;
};

[[noreturn]]void fail(const std::string&s){throw std::runtime_error(s+": "+std::strerror(errno));}
int xioctl(int fd,unsigned long req,void*arg){int r;do{r=ioctl(fd,req,arg);}while(r==-1&&errno==EINTR);return r;}
int64_t monotonicNs(){timespec t{};if(clock_gettime(CLOCK_MONOTONIC,&t)!=0)fail("clock_gettime");return(int64_t)t.tv_sec*1000000000LL+t.tv_nsec;}
int64_t timevalToNs(const timeval&t){return(int64_t)t.tv_sec*1000000000LL+(int64_t)t.tv_usec*1000LL;}
double nsToMs(int64_t n){return(double)n/1e6;} double rad2deg(double r){return r*180.0/PI;} double wrap180(double d){while(d>180)d-=360;while(d<-180)d+=360;return d;}
double percentile(std::vector<double>v,double q){if(v.empty())return 0;std::sort(v.begin(),v.end());double p=q*(v.size()-1);size_t lo=(size_t)floor(p),hi=std::min(lo+1,v.size()-1);double f=p-lo;return v[lo]*(1-f)+v[hi]*f;}
std::string lowerAscii(std::string s){for(char&c:s)if(c>='A'&&c<='Z')c=(char)(c-'A'+'a');return s;}

std::vector<Step> parsePlan(const std::string& text){
    std::vector<Step> out; std::stringstream ss(text); std::string part;
    while(std::getline(ss,part,';')){
        std::string d=lowerAscii(part); if(d.empty())continue;
        bool r=d.find("roll")!=std::string::npos, p=d.find("pitch")!=std::string::npos, y=d.find("yaw")!=std::string::npos;
        bool mixed=d.find("mixed")!=std::string::npos || ((int)r+(int)p+(int)y)>=2;
        double hold=2.0,mag=0.0;
        std::smatch hm;if(std::regex_search(d,hm,std::regex("hold\\s*=?\\s*([0-9]+(?:\\.[0-9]+)?)")))hold=std::stod(hm[1]);
        std::regex nr("([0-9]+(?:\\.[0-9]+)?)");for(std::sregex_iterator i(d.begin(),d.end(),nr),e;i!=e;++i){double v=std::stod((*i)[1]);if(v!=hold&&v<=90)mag=std::max(mag,v);}
        if(mag<=0)mag=y?60:30; hold=std::clamp(hold,0.5,10.0);
        auto add=[&](double rr,double pp,double yy,const std::string& name){out.push_back({rr,pp,yy,hold,name});};
        if(mixed){double m=mag;add(m*.65,-m*.55,std::max(35.0,m),"COMBINED RIGHT / NOSE DOWN / YAW RIGHT");add(0,0,0,"RETURN CENTER");add(-m*.65,m*.55,-std::max(35.0,m),"COMBINED LEFT / NOSE UP / YAW LEFT");add(0,0,0,"RETURN CENTER");}
        else if(r){add(-mag,0,0,"ROLL LEFT "+std::to_string((int)mag)+" deg");add(0,0,0,"RETURN CENTER");add(mag,0,0,"ROLL RIGHT "+std::to_string((int)mag)+" deg");add(0,0,0,"RETURN CENTER");}
        else if(p){add(0,-mag,0,"NOSE DOWN "+std::to_string((int)mag)+" deg");add(0,0,0,"RETURN CENTER");add(0,mag,0,"NOSE UP "+std::to_string((int)mag)+" deg");add(0,0,0,"RETURN CENTER");}
        else if(y){add(0,0,-mag,"YAW LEFT "+std::to_string((int)mag)+" deg");add(0,0,0,"RETURN CENTER");add(0,0,mag,"YAW RIGHT "+std::to_string((int)mag)+" deg");add(0,0,0,"RETURN CENTER");}
    }
    if(out.empty())return parsePlan(DEFAULT_PLAN); return out;
}

bool attitudeInZone(double r,double p,double y,const Step&s){return std::abs(wrap180(r-s.roll))<=ROLL_PITCH_TOL_DEG&&std::abs(wrap180(p-s.pitch))<=ROLL_PITCH_TOL_DEG&&std::abs(wrap180(y-s.yaw))<=YAW_TOL_DEG;}
void putOutlined(cv::Mat&i,const std::string&s,cv::Point p,double sc,int th=1){cv::putText(i,s,p,cv::FONT_HERSHEY_SIMPLEX,sc,{0,0,0},th+3,cv::LINE_AA);cv::putText(i,s,p,cv::FONT_HERSHEY_SIMPLEX,sc,{255,255,255},th,cv::LINE_AA);}
void drawReticle(cv::Mat&i,cv::Point c){for(int th:{6,2}){cv::Scalar col=th==6?cv::Scalar(0,0,0):cv::Scalar(255,255,255);cv::line(i,{c.x-65,c.y},{c.x-10,c.y},col,th,cv::LINE_AA);cv::line(i,{c.x+10,c.y},{c.x+65,c.y},col,th,cv::LINE_AA);cv::line(i,{c.x,c.y-65},{c.x,c.y-10},col,th,cv::LINE_AA);cv::line(i,{c.x,c.y+10},{c.x,c.y+65},col,th,cv::LINE_AA);cv::circle(i,c,24,col,th,cv::LINE_AA);}cv::circle(i,c,3,{0,0,0},8);cv::circle(i,c,3,{255,255,255},3);}
void drawGauge(cv::Mat&c,int x,int y,int w,const std::string&name,double val,double target,double range,double tol){cv::rectangle(c,{x,y,w,28},{0,0,180},cv::FILLED);auto px=[&](double v){return x+(int)(std::clamp((v+range)/(2*range),0.0,1.0)*w);};int g0=px(target-tol),g1=px(target+tol);cv::rectangle(c,{g0,y,std::max(1,g1-g0),28},{0,150,0},cv::FILLED);int z=px(0),v=px(val);cv::line(c,{z,y-4},{z,y+32},{150,150,150},1);cv::line(c,{v,y-8},{v,y+36},{255,255,255},4,cv::LINE_AA);bool ok=std::abs(wrap180(val-target))<=tol;cv::Scalar col=ok?cv::Scalar(0,255,0):cv::Scalar(0,0,255);std::ostringstream q;q<<name<<" "<<std::fixed<<std::setprecision(1)<<val<<" deg  target "<<target<<" +/-"<<tol;cv::putText(c,q.str(),{x,y-12},cv::FONT_HERSHEY_SIMPLEX,.56,col,2,cv::LINE_AA);}
std::vector<std::string>wrapText(const std::string&s,size_t n){std::vector<std::string>o;for(size_t p=0;p<s.size();p+=n)o.push_back(s.substr(p,n));if(o.empty())o.push_back("");return o;}
void drawSetup(cv::Mat&c,const std::string&p){c.setTo(cv::Scalar(20,20,20));cv::putText(c,"JT-ZERO STATE-DRIVEN CAMERA + IMU CALIBRATION",{50,70},cv::FONT_HERSHEY_SIMPLEX,.9,{255,255,255},2);cv::putText(c,"TEST PLAN INPUT",{60,130},cv::FONT_HERSHEY_SIMPLEX,.8,{0,220,255},2);cv::rectangle(c,{50,155,1180,250},{70,70,70},2);auto l=wrapText(p,100);int y=195;for(auto&s:l){cv::putText(c,s,{70,y},cv::FONT_HERSHEY_SIMPLEX,.56,{255,255,255},1);y+=32;if(y>380)break;}cv::putText(c,"ENTER=start  BACKSPACE=delete  F2=default  ESC=exit",{60,455},cv::FONT_HERSHEY_SIMPLEX,.65,{220,220,220},1);cv::putText(c,"Format: roll +/-30 hold 2; pitch +/-30 hold 2; yaw +/-60 hold 2; mixed 25 hold 2",{60,505},cv::FONT_HERSHEY_SIMPLEX,.58,{180,220,255},1);cv::putText(c,"Timer advances ONLY while attitude is inside the green target zones.",{60,560},cv::FONT_HERSHEY_SIMPLEX,.58,{180,255,180},1);}

void previewThreadMain(PreviewState*st){
    try{
        cv::namedWindow(PREVIEW_WINDOW,cv::WINDOW_NORMAL);cv::setWindowProperty(PREVIEW_WINDOW,cv::WND_PROP_FULLSCREEN,cv::WINDOW_FULLSCREEN);
        std::string edit=st->plan_text;uint64_t seen=0;double lastWall=0;
        while(true){
            PreviewPacket p;{std::unique_lock<std::mutex>lk(st->mutex);st->cv.wait_for(lk,std::chrono::milliseconds(25),[&]{return st->stopping||st->generation!=seen;});if(st->stopping)break;if(st->generation!=seen){p=st->latest;seen=st->generation;}}
            cv::Mat c(720,1280,CV_8UC3,cv::Scalar(20,20,20));
            if(!st->plan_ready.load()){
                drawSetup(c,edit);cv::imshow(PREVIEW_WINDOW,c);int k=cv::waitKeyEx(1);if(k==27){st->stop_requested.store(true);break;}if(k==13||k==10){{std::lock_guard<std::mutex>lk(st->mutex);st->plan_text=edit;st->steps=parsePlan(edit);}st->yaw_zero_deg.store(st->yaw_deg.load());st->plan_ready.store(true);lastWall=monotonicNs()/1e9;}else if(k==65471||k==0x710000)edit=DEFAULT_PLAN;else if(k==8||k==127){if(!edit.empty())edit.pop_back();}else if(k>=32&&k<=126)edit.push_back((char)k);continue;
            }
            cv::Mat gray;if(!p.jpeg.empty())gray=cv::imdecode(p.jpeg,cv::IMREAD_GRAYSCALE);
            if(!gray.empty()){cv::Mat b;cv::cvtColor(gray,b,cv::COLOR_GRAY2BGR);cv::resize(b,b,{760,570});b.copyTo(c(cv::Rect(20,110,760,570)));drawReticle(c,{400,395});}
            double now=monotonicNs()/1e9,dt=lastWall>0?std::min(.1,now-lastWall):0;lastWall=now;
            int idx=st->step_index.load();std::vector<Step>steps;{std::lock_guard<std::mutex>lk(st->mutex);steps=st->steps;}
            if(idx>=(int)steps.size()){st->test_complete.store(true);putOutlined(c,"CALIBRATION SEQUENCE COMPLETE",{25,52},.9,2);}
            else{
                Step s=steps[idx];double r=st->roll_deg.load(),pi=st->pitch_deg.load(),y=wrap180(st->yaw_deg.load()-st->yaw_zero_deg.load());bool anglesGood=st->attitude_valid.load()&&attitudeInZone(r,pi,y,s);double collected=st->step_collected_sec.load();if(anglesGood)collected+=dt;st->step_collected_sec.store(collected);
                if(collected>=s.hold_sec){st->step_index.store(idx+1);st->step_collected_sec.store(0);}
                std::ostringstream top;top<<"STEP "<<(idx+1)<<" / "<<steps.size()<<"   DATA "<<std::fixed<<std::setprecision(1)<<std::min(collected,s.hold_sec)<<" / "<<s.hold_sec<<" s";putOutlined(c,top.str(),{25,52},.8,2);putOutlined(c,s.instruction,{25,92},.72,2);
                cv::putText(c,"ATTITUDE / TARGET",{820,130},cv::FONT_HERSHEY_SIMPLEX,.72,{255,255,255},2);drawGauge(c,820,220,410,"ROLL",r,s.roll,60,ROLL_PITCH_TOL_DEG);drawGauge(c,820,320,410,"PITCH",pi,s.pitch,60,ROLL_PITCH_TOL_DEG);drawGauge(c,820,420,410,"YAW d",y,s.yaw,90,YAW_TOL_DEG);
                cv::putText(c,anglesGood?"COLLECTING DATA":"MOVE INTO GREEN ZONES - TIMER PAUSED",{820,545},cv::FONT_HERSHEY_SIMPLEX,.5,anglesGood?cv::Scalar(0,255,0):cv::Scalar(0,180,255),2,cv::LINE_AA);
            }
            cv::putText(c,"Q / ESC = stop",{820,620},cv::FONT_HERSHEY_SIMPLEX,.6,{255,255,255},1);cv::imshow(PREVIEW_WINDOW,c);int k=cv::waitKeyEx(1);if(k==27||k=='q'||k=='Q')st->stop_requested.store(true);
        }
        cv::destroyAllWindows();
    }catch(const cv::Exception&e){std::cerr<<"[PREVIEW] "<<e.what()<<'\n';st->stop_requested.store(true);}
}
void submitPreview(PreviewState*st,const void*d,size_t n,uint32_t seq){PreviewPacket p;auto*b=(const unsigned char*)d;p.jpeg.assign(b,b+n);p.sequence=seq;{std::lock_guard<std::mutex>lk(st->mutex);st->latest=std::move(p);++st->generation;}st->cv.notify_one();}
void stopPreview(PreviewState*st,std::thread*th){{std::lock_guard<std::mutex>lk(st->mutex);st->stopping=true;}st->cv.notify_all();if(th&&th->joinable())th->join();}
std::string cameraTimestampFlags(uint32_t f){std::string s=((f&V4L2_BUF_FLAG_TIMESTAMP_MASK)==V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC)?"monotonic":"other_clock";s+="|";s+=((f&V4L2_BUF_FLAG_TSTAMP_SRC_MASK)==V4L2_BUF_FLAG_TSTAMP_SRC_SOE)?"soe":"eof";return s;}
void setCameraControl(int fd,uint32_t id,int32_t val,const char*name){v4l2_control c{};c.id=id;c.value=val;if(xioctl(fd,VIDIOC_S_CTRL,&c)==-1)std::cerr<<"[WARN] "<<name<<" failed: "<<std::strerror(errno)<<'\n';else std::cout<<"[CAM] "<<name<<"="<<val<<'\n';}
void configureCamera(int fd){v4l2_capability cap{};if(xioctl(fd,VIDIOC_QUERYCAP,&cap)==-1)fail("VIDIOC_QUERYCAP");std::cout<<"[CAM] driver="<<cap.driver<<" card="<<cap.card<<'\n';v4l2_format fmt{};fmt.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;fmt.fmt.pix.width=CAMERA_WIDTH;fmt.fmt.pix.height=CAMERA_HEIGHT;fmt.fmt.pix.pixelformat=V4L2_PIX_FMT_MJPEG;fmt.fmt.pix.field=V4L2_FIELD_ANY;if(xioctl(fd,VIDIOC_S_FMT,&fmt)==-1)fail("VIDIOC_S_FMT");v4l2_streamparm p{};p.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;p.parm.capture.timeperframe.numerator=1;p.parm.capture.timeperframe.denominator=CAMERA_FPS;if(xioctl(fd,VIDIOC_S_PARM,&p)==-1)fail("VIDIOC_S_PARM");double fps=p.parm.capture.timeperframe.numerator?double(p.parm.capture.timeperframe.denominator)/p.parm.capture.timeperframe.numerator:0;std::cout<<"[CAM] 640x480 MJPEG requested=120 actual="<<std::fixed<<std::setprecision(3)<<fps<<" FPS\n";setCameraControl(fd,V4L2_CID_EXPOSURE_AUTO,V4L2_EXPOSURE_MANUAL,"auto_exposure");setCameraControl(fd,V4L2_CID_EXPOSURE_AUTO_PRIORITY,0,"dynamic_framerate");setCameraControl(fd,V4L2_CID_EXPOSURE_ABSOLUTE,EXPOSURE_ABSOLUTE,"exposure_absolute");setCameraControl(fd,V4L2_CID_GAIN,CAMERA_GAIN,"gain");setCameraControl(fd,V4L2_CID_AUTO_WHITE_BALANCE,0,"white_balance_automatic");setCameraControl(fd,V4L2_CID_POWER_LINE_FREQUENCY,V4L2_CID_POWER_LINE_FREQUENCY_DISABLED,"power_line_frequency");setCameraControl(fd,V4L2_CID_BACKLIGHT_COMPENSATION,0,"backlight_compensation");}
std::vector<CameraBuffer>initCameraBuffers(int fd){v4l2_requestbuffers r{};r.count=CAMERA_BUFFER_COUNT;r.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;r.memory=V4L2_MEMORY_MMAP;if(xioctl(fd,VIDIOC_REQBUFS,&r)==-1)fail("VIDIOC_REQBUFS");std::vector<CameraBuffer>b(r.count);for(uint32_t i=0;i<r.count;++i){v4l2_buffer q{};q.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;q.memory=V4L2_MEMORY_MMAP;q.index=i;if(xioctl(fd,VIDIOC_QUERYBUF,&q)==-1)fail("VIDIOC_QUERYBUF");b[i].length=q.length;b[i].start=mmap(nullptr,q.length,PROT_READ|PROT_WRITE,MAP_SHARED,fd,q.m.offset);if(b[i].start==MAP_FAILED)fail("mmap");if(xioctl(fd,VIDIOC_QBUF,&q)==-1)fail("VIDIOC_QBUF");}return b;}
void discardWarmup(int fd,int count){std::cout<<"[CAM] discarding "<<count<<" warmup frames...\n";for(int n=0;n<count;){pollfd p{fd,POLLIN,0};if(poll(&p,1,1000)<=0)continue;v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;if(xioctl(fd,VIDIOC_DQBUF,&b)==-1){if(errno==EAGAIN)continue;fail("DQBUF warmup");}++n;if(xioctl(fd,VIDIOC_QBUF,&b)==-1)fail("QBUF warmup");}}
int openSerial(){int fd=open(SERIAL_DEVICE,O_RDWR|O_NOCTTY|O_NONBLOCK);if(fd==-1)fail("open serial");termios t{};if(tcgetattr(fd,&t)!=0)fail("tcgetattr");cfmakeraw(&t);if(cfsetispeed(&t,B460800)||cfsetospeed(&t,B460800))fail("baud");t.c_cflag|=CLOCAL|CREAD;t.c_cflag&=~CRTSCTS;t.c_cflag&=~PARENB;t.c_cflag&=~CSTOPB;t.c_cflag&=~CSIZE;t.c_cflag|=CS8;t.c_cc[VMIN]=0;t.c_cc[VTIME]=0;if(tcsetattr(fd,TCSANOW,&t)!=0)fail("tcsetattr");tcflush(fd,TCIFLUSH);std::cout<<"[MAV] "<<SERIAL_DEVICE<<" @ 460800\n";return fd;}
void serialWriteAll(int fd,const uint8_t*d,size_t n){for(size_t p=0;p<n;){ssize_t r=write(fd,d+p,n-p);if(r>0){p+=size_t(r);continue;}if(r==-1&&(errno==EAGAIN||errno==EWOULDBLOCK)){pollfd q{fd,POLLOUT,0};poll(&q,1,10);continue;}if(r==-1&&errno==EINTR)continue;fail("serial write");}}
void sendMsg(int fd,const mavlink_message_t&m){uint8_t b[MAVLINK_MAX_PACKET_LEN];uint16_t n=mavlink_msg_to_send_buffer(b,&m);serialWriteAll(fd,b,n);}
void requestRate(int fd,uint8_t sys,uint8_t comp,uint32_t msgid,int hz){mavlink_message_t m{};float us=hz>0?float(1000000.0/hz):0;mavlink_msg_command_long_pack(COMPANION_SYSID,COMPANION_COMPID,&m,sys,comp,MAV_CMD_SET_MESSAGE_INTERVAL,0,msgid,us,0,0,0,0,0);sendMsg(fd,m);}
void sendTimesync(int fd,int64_t t0,uint8_t sys,uint8_t comp){mavlink_message_t m{};mavlink_msg_timesync_pack(COMPANION_SYSID,COMPANION_COMPID,&m,0,t0,sys,comp);sendMsg(fd,m);}
ClockMapping estimateClockMapping(const std::vector<TimeSyncSample>&s){std::vector<const TimeSyncSample*>g;for(auto&x:s)if(x.good)g.push_back(&x);ClockMapping m;if(g.size()<10)return m;int64_t f0=g.front()->fc_ns,r0=g.front()->rpi_mid_ns;long double mx=0,my=0;for(auto*x:g){mx+=x->fc_ns-f0;my+=x->rpi_mid_ns-r0;}mx/=g.size();my/=g.size();long double sxx=0,sxy=0;for(auto*x:g){long double dx=(x->fc_ns-f0)-mx,dy=(x->rpi_mid_ns-r0)-my;sxx+=dx*dx;sxy+=dx*dy;}if(sxx<=0)return m;m.a=sxy/sxx;m.fc_ref_ns=f0;m.rpi_ref_ns=(long double)r0+(my-m.a*mx);m.drift_ppm=(double)((m.a-1)*1e6L);m.valid=true;return m;}

} // namespace

int main(){
    int serial_fd=-1,camera_fd=-1;bool streaming=false,rates=false,preview_started=false;std::vector<CameraBuffer>buffers;PreviewState ps;std::thread preview;uint8_t target_system=0,target_component=0;
    try{
        serial_fd=openSerial();std::cout<<"[MAV] waiting for HEARTBEAT...\n";mavlink_status_t ms{};mavlink_message_t mm{};int64_t deadline=monotonicNs()+10000000000LL;
        while(monotonicNs()<deadline&&target_system==0){pollfd p{serial_fd,POLLIN,0};if(poll(&p,1,100)>0){uint8_t b[2048];ssize_t n=read(serial_fd,b,sizeof(b));for(ssize_t i=0;i<n;++i)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&mm,&ms)&&mm.msgid==MAVLINK_MSG_ID_HEARTBEAT){target_system=mm.sysid;target_component=mm.compid;break;}}}if(!target_system)throw std::runtime_error("HEARTBEAT timeout");std::cout<<"[MAV] connected system="<<int(target_system)<<" component="<<int(target_component)<<'\n';requestRate(serial_fd,target_system,target_component,MAVLINK_MSG_ID_HIGHRES_IMU,IMU_RATE_HZ);requestRate(serial_fd,target_system,target_component,MAVLINK_MSG_ID_ATTITUDE,ATTITUDE_RATE_HZ);rates=true;
        camera_fd=open(CAMERA_DEVICE,O_RDWR|O_NONBLOCK);if(camera_fd==-1)fail("open camera");configureCamera(camera_fd);buffers=initCameraBuffers(camera_fd);v4l2_buf_type typ=V4L2_BUF_TYPE_VIDEO_CAPTURE;if(xioctl(camera_fd,VIDIOC_STREAMON,&typ)==-1)fail("STREAMON");streaming=true;discardWarmup(camera_fd,CAMERA_WARMUP_FRAMES);
        preview=std::thread(previewThreadMain,&ps);preview_started=true;std::cout<<"[UI] Full-screen setup opened. Edit plan and press ENTER.\n";uint32_t pc=0;Attitude latest;
        while(!ps.plan_ready.load()&&!ps.stop_requested.load()){pollfd pf[2]={{camera_fd,POLLIN,0},{serial_fd,POLLIN,0}};poll(pf,2,5);if(pf[0].revents&POLLIN){for(;;){v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;if(xioctl(camera_fd,VIDIOC_DQBUF,&b)==-1){if(errno==EAGAIN)break;fail("DQBUF setup");}if((pc++%PREVIEW_DIVIDER)==0)submitPreview(&ps,buffers[b.index].start,b.bytesused,b.sequence);if(xioctl(camera_fd,VIDIOC_QBUF,&b)==-1)fail("QBUF setup");}}if(pf[1].revents&POLLIN){uint8_t by[4096];ssize_t n=read(serial_fd,by,sizeof(by));for(ssize_t i=0;i<n;++i)if(mavlink_parse_char(MAVLINK_COMM_0,by[i],&mm,&ms)&&mm.msgid==MAVLINK_MSG_ID_ATTITUDE){mavlink_attitude_t a{};mavlink_msg_attitude_decode(&mm,&a);latest={rad2deg(a.roll),rad2deg(a.pitch),rad2deg(a.yaw),true};ps.roll_deg.store(latest.roll_deg);ps.pitch_deg.store(latest.pitch_deg);ps.yaw_deg.store(latest.yaw_deg);ps.attitude_valid.store(true);}}}
        if(ps.stop_requested.load())throw std::runtime_error("Cancelled in setup");tcflush(serial_fd,TCIFLUSH);std::memset(&ms,0,sizeof(ms));std::memset(&mm,0,sizeof(mm));
        std::vector<CameraSample>camera;std::vector<ImuSample>imu;std::vector<TimeSyncSample>ts;camera.reserve(20000);imu.reserve(30000);ts.reserve(1500);std::ofstream mjpeg(OUTPUT_MJPEG,std::ios::binary|std::ios::trunc);if(!mjpeg)throw std::runtime_error("Cannot create MJPEG output");uint64_t drops=0;bool have_seq=false;uint32_t prev_seq=0;int64_t pending=0,next_ts=monotonicNs();pc=0;
        std::cout<<"\n=== STATE-DRIVEN CAMERA + IMU + TIMESYNC LOGGER ===\n"<<"camera:         640x480 MJPEG @ 120 FPS\npreview:        fullscreen, attitude-driven hold timer\nIMU:            HIGHRES_IMU @ 200 Hz\nATTITUDE:       @ 50 Hz for UI\nTIMESYNC:       10 Hz\ncamera offset:  +"<<jtzero::timesync::cameraToImuCorrectionMs()<<" ms\n\n";
        while(!ps.stop_requested.load()&&!ps.test_complete.load()){
            int64_t now=monotonicNs();if(now>=next_ts&&pending==0){pending=now;sendTimesync(serial_fd,pending,target_system,target_component);next_ts=now+TIMESYNC_PERIOD_NS;}
            pollfd pf[2]={{camera_fd,POLLIN,0},{serial_fd,POLLIN,0}};int rc=poll(pf,2,2);if(rc<0){if(errno==EINTR)continue;fail("poll");}
            if(pf[0].revents&POLLIN){for(;;){v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;if(xioctl(camera_fd,VIDIOC_DQBUF,&b)==-1){if(errno==EAGAIN)break;fail("DQBUF");}int64_t recv=monotonicNs(),v4=timevalToNs(b.timestamp),corr=jtzero::timesync::correctCameraTimestampNs(v4);if(have_seq){uint32_t ex=prev_seq+1;if(b.sequence!=ex)drops+=uint32_t(b.sequence-ex);}prev_seq=b.sequence;have_seq=true;std::streampos pos=mjpeg.tellp();uint64_t off=uint64_t(pos);mjpeg.write((const char*)buffers[b.index].start,b.bytesused);camera.push_back({recv,v4,corr,b.sequence,b.flags,b.bytesused,off});if((pc++%PREVIEW_DIVIDER)==0)submitPreview(&ps,buffers[b.index].start,b.bytesused,b.sequence);if(xioctl(camera_fd,VIDIOC_QBUF,&b)==-1)fail("QBUF");}}
            if(pf[1].revents&POLLIN){uint8_t by[8192];for(;;){ssize_t n=read(serial_fd,by,sizeof(by));if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;for(ssize_t i=0;i<n;++i){if(!mavlink_parse_char(MAVLINK_COMM_0,by[i],&mm,&ms))continue;int64_t recv=monotonicNs();if(mm.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){mavlink_highres_imu_t a{};mavlink_msg_highres_imu_decode(&mm,&a);ImuSample s;s.recv_ns=recv;s.fc_ns=int64_t(a.time_usec)*1000;s.xacc=a.xacc;s.yacc=a.yacc;s.zacc=a.zacc;s.xgyro=a.xgyro;s.ygyro=a.ygyro;s.zgyro=a.zgyro;s.temperature=a.temperature;s.fields_updated=a.fields_updated;
#ifdef MAVLINK_MSG_HIGHRES_IMU_FIELD_ID_LEN
s.imu_id=a.id;
#endif
imu.push_back(s);}else if(mm.msgid==MAVLINK_MSG_ID_ATTITUDE){mavlink_attitude_t a{};mavlink_msg_attitude_decode(&mm,&a);ps.roll_deg.store(rad2deg(a.roll));ps.pitch_deg.store(rad2deg(a.pitch));ps.yaw_deg.store(rad2deg(a.yaw));ps.attitude_valid.store(true);}else if(mm.msgid==MAVLINK_MSG_ID_TIMESYNC){mavlink_timesync_t a{};mavlink_msg_timesync_decode(&mm,&a);if(a.tc1!=0&&pending!=0&&a.ts1==pending){TimeSyncSample s;s.t0_rpi_ns=pending;s.t1_rpi_ns=recv;s.fc_ns=a.tc1;s.rtt_ns=recv-pending;s.rpi_mid_ns=pending+s.rtt_ns/2;s.good=s.rtt_ns>0&&nsToMs(s.rtt_ns)<=MAX_TIMESYNC_RTT_MS;ts.push_back(s);pending=0;}}}}}if(pending&&monotonicNs()-pending>20000000LL)pending=0;
        }
        mjpeg.flush();mjpeg.close();ClockMapping map=estimateClockMapping(ts);if(!map.valid)throw std::runtime_error("Not enough valid TIMESYNC samples");std::vector<double>camlat,camdt,imut,trtt;for(size_t i=0;i<camera.size();++i){camlat.push_back(nsToMs(camera[i].recv_ns-camera[i].v4l2_ns));if(i)camdt.push_back(nsToMs(camera[i].v4l2_ns-camera[i-1].v4l2_ns));}for(auto&s:imu)imut.push_back(nsToMs(s.recv_ns-map.map(s.fc_ns)));size_t good=0;for(auto&s:ts)if(s.good){++good;trtt.push_back(nsToMs(s.rtt_ns));}
        std::ofstream ci(OUTPUT_CAMERA_INDEX);ci<<"sequence,v4l2_timestamp_ns,camera_timestamp_corrected_ns,recv_rpi_ns,delivery_latency_ms,mjpeg_offset,bytes_used,flags\n"<<std::fixed<<std::setprecision(9);for(auto&s:camera)ci<<s.sequence<<','<<s.v4l2_ns<<','<<s.corrected_ns<<','<<s.recv_ns<<','<<nsToMs(s.recv_ns-s.v4l2_ns)<<','<<s.mjpeg_offset<<','<<s.bytes_used<<','<<cameraTimestampFlags(s.flags)<<'\n';ci.close();
        std::ofstream csv(OUTPUT_CSV);csv<<"event,recv_rpi_ns,source_timestamp_ns,mapped_rpi_ns,transport_latency_ms,camera_sequence,camera_flags,camera_bytes,xacc_m_s2,yacc_m_s2,zacc_m_s2,xgyro_rad_s,ygyro_rad_s,zgyro_rad_s,temperature_c,imu_id,timesync_t0_rpi_ns,timesync_t1_rpi_ns,timesync_fc_ns,timesync_mid_rpi_ns,timesync_rtt_ms,timesync_good,map_a,map_drift_ppm,map_fc_ref_ns,map_rpi_ref_ns\n"<<std::fixed<<std::setprecision(9);for(auto&s:camera)csv<<"CAMERA,"<<s.recv_ns<<','<<s.v4l2_ns<<','<<s.corrected_ns<<','<<nsToMs(s.recv_ns-s.v4l2_ns)<<','<<s.sequence<<','<<cameraTimestampFlags(s.flags)<<','<<s.bytes_used<<",,,,,,,,,,,,,,,"<<double(map.a)<<','<<map.drift_ppm<<','<<map.fc_ref_ns<<','<<int64_t(std::llround(map.rpi_ref_ns))<<'\n';for(auto&s:imu){int64_t mn=map.map(s.fc_ns);csv<<"IMU,"<<s.recv_ns<<','<<s.fc_ns<<','<<mn<<','<<nsToMs(s.recv_ns-mn)<<",,,,"<<s.xacc<<','<<s.yacc<<','<<s.zacc<<','<<s.xgyro<<','<<s.ygyro<<','<<s.zgyro<<','<<s.temperature<<','<<int(s.imu_id)<<",,,,,,"<<double(map.a)<<','<<map.drift_ppm<<','<<map.fc_ref_ns<<','<<int64_t(std::llround(map.rpi_ref_ns))<<'\n';}for(auto&s:ts)csv<<"TIMESYNC,"<<s.t1_rpi_ns<<",,,,,,,,,,,,,,,,"<<s.t0_rpi_ns<<','<<s.t1_rpi_ns<<','<<s.fc_ns<<','<<s.rpi_mid_ns<<','<<nsToMs(s.rtt_ns)<<','<<(s.good?1:0)<<','<<double(map.a)<<','<<map.drift_ppm<<','<<map.fc_ref_ns<<','<<int64_t(std::llround(map.rpi_ref_ns))<<'\n';csv.close();
        stopPreview(&ps,&preview);preview_started=false;std::cout<<"\n============================================================\nFINAL CLOCK MAPPING\n============================================================\n"<<std::setprecision(12)<<"A (RPi/FC):           "<<double(map.a)<<'\n'<<std::setprecision(3)<<"drift:                "<<map.drift_ppm<<" ppm\nTIMESYNC good:         "<<good<<'/'<<ts.size()<<'\n';if(!trtt.empty())std::cout<<"TIMESYNC RTT median:   "<<percentile(trtt,.5)<<" ms\nTIMESYNC RTT p95:      "<<percentile(trtt,.95)<<" ms\n";std::cout<<"\n============================================================\nCAMERA\n============================================================\nframes:                "<<camera.size()<<"\nsource drops:          "<<drops<<'\n';if(!camlat.empty())std::cout<<"delivery median:       "<<percentile(camlat,.5)<<" ms\ndelivery p95:          "<<percentile(camlat,.95)<<" ms\ndelivery p99:          "<<percentile(camlat,.99)<<" ms\ndelivery max:          "<<*std::max_element(camlat.begin(),camlat.end())<<" ms\n";if(!camdt.empty())std::cout<<"timestamp dt median:   "<<percentile(camdt,.5)<<" ms\ntimestamp dt p95:      "<<percentile(camdt,.95)<<" ms\n";std::cout<<"\n============================================================\nIMU\n============================================================\nsamples:               "<<imu.size()<<'\n';if(!imut.empty())std::cout<<"transport median:      "<<percentile(imut,.5)<<" ms\ntransport p95:         "<<percentile(imut,.95)<<" ms\ntransport p99:         "<<percentile(imut,.99)<<" ms\ntransport min:         "<<*std::min_element(imut.begin(),imut.end())<<" ms\ntransport max:         "<<*std::max_element(imut.begin(),imut.end())<<" ms\n";std::cout<<"\nCombined CSV:          "<<OUTPUT_CSV<<"\nCamera index CSV:      "<<OUTPUT_CAMERA_INDEX<<"\nCamera MJPEG:          "<<OUTPUT_MJPEG<<'\n';
        if(rates){requestRate(serial_fd,target_system,target_component,MAVLINK_MSG_ID_HIGHRES_IMU,0);requestRate(serial_fd,target_system,target_component,MAVLINK_MSG_ID_ATTITUDE,0);rates=false;}if(streaming){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(camera_fd,VIDIOC_STREAMOFF,&t);streaming=false;}for(auto&b:buffers)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(camera_fd!=-1)close(camera_fd);if(serial_fd!=-1)close(serial_fd);return 0;
    }catch(const std::exception&e){std::cerr<<"\n[FATAL] "<<e.what()<<'\n';if(preview_started)stopPreview(&ps,&preview);if(serial_fd!=-1&&rates&&target_system){try{requestRate(serial_fd,target_system,target_component,MAVLINK_MSG_ID_HIGHRES_IMU,0);requestRate(serial_fd,target_system,target_component,MAVLINK_MSG_ID_ATTITUDE,0);}catch(...){}}if(camera_fd!=-1&&streaming){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(camera_fd,VIDIOC_STREAMOFF,&t);}for(auto&b:buffers)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(camera_fd!=-1)close(camera_fd);if(serial_fd!=-1)close(serial_fd);return 1;}
}