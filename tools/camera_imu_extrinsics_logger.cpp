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
#include <limits>
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

constexpr const char* CAMERA_DEVICE = "/dev/video0";
constexpr const char* SERIAL_DEVICE = "/dev/ttyAMA0";
constexpr const char* OUTPUT_CSV = "/home/vio/camera_imu_extrinsics.csv";
constexpr const char* OUTPUT_MJPEG = "/dev/shm/camera_imu_extrinsics.mjpg";
constexpr const char* OUTPUT_CAMERA_INDEX = "/dev/shm/camera_imu_extrinsics_camera.csv";
constexpr const char* PREVIEW_WINDOW = "JT-Zero Guided Camera + IMU Calibration";

constexpr int CAMERA_WIDTH = 640;
constexpr int CAMERA_HEIGHT = 480;
constexpr int CAMERA_FPS = 120;
constexpr int CAMERA_BUFFER_COUNT = 6;
constexpr int CAMERA_WARMUP_FRAMES = 30;
constexpr int PREVIEW_DIVIDER = 4;
constexpr int IMU_RATE_HZ = 200;
constexpr int ATTITUDE_RATE_HZ = 50;
constexpr double TIMESYNC_RATE_HZ = 10.0;
constexpr int64_t TIMESYNC_PERIOD_NS = 100000000LL;
constexpr double MAX_TIMESYNC_RTT_MS = 10.0;
constexpr int EXPOSURE_ABSOLUTE = 50;
constexpr int CAMERA_GAIN = 0;
constexpr uint8_t COMPANION_SYSID = 255;
constexpr uint8_t COMPANION_COMPID = 190;
constexpr double PI = 3.14159265358979323846;

const std::string DEFAULT_PLAN =
    "0-10 roll +/-20-30; 10-20 pitch +/-20-30; 20-30 yaw +/-40-60; "
    "30-50 roll+pitch+yaw; 50-60 roll+pitch+yaw";

struct CameraBuffer { void* start=nullptr; size_t length=0; };
struct CameraSample {
    int64_t recv_ns=0, v4l2_ns=0, corrected_ns=0;
    uint32_t sequence=0, flags=0, bytes_used=0;
    uint64_t mjpeg_offset=0;
};
struct ImuSample {
    int64_t recv_ns=0, fc_ns=0;
    float xacc=0,yacc=0,zacc=0,xgyro=0,ygyro=0,zgyro=0,temperature=0;
    uint16_t fields_updated=0;
    uint8_t imu_id=0;
};
struct TimeSyncSample {
    int64_t t0_rpi_ns=0,t1_rpi_ns=0,fc_ns=0,rpi_mid_ns=0,rtt_ns=0;
    bool good=false;
};
struct ClockMapping {
    bool valid=false;
    long double a=1.0L;
    int64_t fc_ref_ns=0;
    long double rpi_ref_ns=0.0L;
    double drift_ppm=0;
    int64_t map(int64_t fc_ns) const {
        return static_cast<int64_t>(std::llround(rpi_ref_ns + a*static_cast<long double>(fc_ns-fc_ref_ns)));
    }
};
struct Attitude {
    double roll_deg=0,pitch_deg=0,yaw_deg=0;
    bool valid=false;
};
struct Phase {
    double start=0,end=0,magnitude=30;
    enum Type { ROLL, PITCH, YAW, MIXED } type=MIXED;
    std::string source;
};
struct Targets {
    double roll=0,pitch=0,yaw=0;
    bool roll_active=true,pitch_active=true,yaw_active=true;
    std::string instruction="CENTER / READY";
};
struct PreviewPacket {
    std::vector<unsigned char> jpeg;
    double elapsed_sec=0,duration_sec=0;
    uint32_t sequence=0;
    bool running=false;
};
struct PreviewState {
    std::mutex mutex;
    std::condition_variable cv;
    PreviewPacket latest;
    uint64_t generation=0;
    bool stopping=false;
    std::atomic<bool> stop_requested{false};
    std::atomic<bool> plan_ready{false};
    std::atomic<bool> attitude_valid{false};
    std::atomic<double> roll_deg{0},pitch_deg{0},yaw_deg{0};
    std::atomic<double> yaw_zero_deg{0};
    std::string plan_text=DEFAULT_PLAN;
    std::vector<Phase> phases;
    double plan_duration=60;
};

[[noreturn]] void fail(const std::string& s){ throw std::runtime_error(s+": "+std::strerror(errno)); }
int xioctl(int fd,unsigned long req,void* arg){ int r; do{r=ioctl(fd,req,arg);}while(r==-1&&errno==EINTR); return r; }
int64_t monotonicNs(){ timespec ts{}; if(clock_gettime(CLOCK_MONOTONIC,&ts)!=0) fail("clock_gettime"); return int64_t(ts.tv_sec)*1000000000LL+ts.tv_nsec; }
int64_t timevalToNs(const timeval& tv){ return int64_t(tv.tv_sec)*1000000000LL+int64_t(tv.tv_usec)*1000LL; }
double nsToMs(int64_t ns){ return double(ns)/1e6; }
double rad2deg(double r){ return r*180.0/PI; }
double wrap180(double d){ while(d>180)d-=360; while(d<-180)d+=360; return d; }

double percentile(std::vector<double> v,double q){
    if(v.empty()) return 0; std::sort(v.begin(),v.end());
    double p=q*double(v.size()-1); size_t lo=size_t(std::floor(p)),hi=std::min(lo+1,v.size()-1); double f=p-lo;
    return v[lo]*(1-f)+v[hi]*f;
}
std::string lowerAscii(std::string s){ for(char& c:s) if(c>='A'&&c<='Z') c=char(c-'A'+'a'); return s; }

std::vector<Phase> parsePlan(const std::string& text,double* duration){
    std::vector<Phase> out;
    std::regex tr("([0-9]+(?:\\.[0-9]+)?)\\s*[-]\\s*([0-9]+(?:\\.[0-9]+)?)");
    std::sregex_iterator it(text.begin(),text.end(),tr),end;
    std::vector<std::smatch> matches; for(;it!=end;++it) matches.push_back(*it);
    for(size_t i=0;i<matches.size();++i){
        Phase p; p.start=std::stod(matches[i][1]); p.end=std::stod(matches[i][2]);
        size_t a=size_t(matches[i].position()+matches[i].length());
        size_t b=(i+1<matches.size())?size_t(matches[i+1].position()):text.size();
        std::string desc=lowerAscii(text.substr(a,b-a)); p.source=desc;
        bool has_roll=desc.find("roll")!=std::string::npos;
        bool has_pitch=desc.find("pitch")!=std::string::npos;
        bool has_yaw=desc.find("yaw")!=std::string::npos;
        if((has_roll&&has_pitch)||(has_roll&&has_yaw)||(has_pitch&&has_yaw)) p.type=Phase::MIXED;
        else if(has_roll) p.type=Phase::ROLL;
        else if(has_pitch) p.type=Phase::PITCH;
        else if(has_yaw) p.type=Phase::YAW;
        else p.type=Phase::MIXED;
        std::regex nr("([0-9]+(?:\\.[0-9]+)?)");
        double mag=0; for(std::sregex_iterator ni(desc.begin(),desc.end(),nr);ni!=end;++ni) mag=std::max(mag,std::stod((*ni)[1]));
        if(mag>0&&mag<=90) p.magnitude=mag; else p.magnitude=(p.type==Phase::YAW?60:30);
        if(p.end>p.start) out.push_back(p);
    }
    if(out.empty()){
        out={{0,10,30,Phase::ROLL,""},{10,20,30,Phase::PITCH,""},{20,30,60,Phase::YAW,""},{30,50,30,Phase::MIXED,""},{50,60,30,Phase::MIXED,""}};
    }
    *duration=0; for(const auto& p:out) *duration=std::max(*duration,p.end); return out;
}

Targets targetsFor(const std::vector<Phase>& phases,double t){
    Targets z; const Phase* ph=nullptr; for(const auto& p:phases) if(t>=p.start&&t<p.end){ph=&p;break;}
    if(!ph){ z.instruction="CENTER / WAIT"; return z; }
    double u=(t-ph->start)/(ph->end-ph->start); double m=ph->magnitude;
    auto signedCycle=[&](double uu,double* target,std::string neg,std::string pos){
        if(uu<0.18){*target=-m; z.instruction=neg+"  "+std::to_string(int(m))+" deg";}
        else if(uu<0.32){*target=-m; z.instruction="HOLD  -"+std::to_string(int(m))+" deg";}
        else if(uu<0.46){*target=0; z.instruction="RETURN CENTER";}
        else if(uu<0.64){*target=m; z.instruction=pos+"  "+std::to_string(int(m))+" deg";}
        else if(uu<0.78){*target=m; z.instruction="HOLD  +"+std::to_string(int(m))+" deg";}
        else {*target=0; z.instruction="RETURN CENTER";}
    };
    if(ph->type==Phase::ROLL){ signedCycle(u,&z.roll,"ROLL LEFT","ROLL RIGHT"); z.pitch=0; z.yaw=0; }
    else if(ph->type==Phase::PITCH){ signedCycle(u,&z.pitch,"NOSE DOWN","NOSE UP"); z.roll=0; z.yaw=0; }
    else if(ph->type==Phase::YAW){ signedCycle(u,&z.yaw,"YAW LEFT","YAW RIGHT"); z.roll=0; z.pitch=0; }
    else {
        double a=(u<0.33)?1.0:(u<0.66?-1.0:0.0);
        z.roll=a*m*0.65; z.pitch=-a*m*0.55; z.yaw=a*std::max(35.0,m);
        z.instruction=(a>0)?"COMBINED: RIGHT + NOSE DOWN + YAW RIGHT":(a<0?"COMBINED: LEFT + NOSE UP + YAW LEFT":"CENTER / RESET");
    }
    return z;
}

void putOutlined(cv::Mat& img,const std::string& s,cv::Point p,double scale,double thick=1){
    cv::putText(img,s,p,cv::FONT_HERSHEY_SIMPLEX,scale,{0,0,0},int(thick+3),cv::LINE_AA);
    cv::putText(img,s,p,cv::FONT_HERSHEY_SIMPLEX,scale,{255,255,255},int(thick),cv::LINE_AA);
}
void drawReticle(cv::Mat& img,cv::Point c){
    for(int th:{6,2}){ cv::Scalar col=th==6?cv::Scalar(0,0,0):cv::Scalar(255,255,255);
        cv::line(img,{c.x-65,c.y},{c.x-10,c.y},col,th,cv::LINE_AA); cv::line(img,{c.x+10,c.y},{c.x+65,c.y},col,th,cv::LINE_AA);
        cv::line(img,{c.x,c.y-65},{c.x,c.y-10},col,th,cv::LINE_AA); cv::line(img,{c.x,c.y+10},{c.x,c.y+65},col,th,cv::LINE_AA); cv::circle(img,c,24,col,th,cv::LINE_AA);
    }
    cv::circle(img,c,3,{0,0,0},8,cv::LINE_AA); cv::circle(img,c,3,{255,255,255},3,cv::LINE_AA);
}
void drawGauge(cv::Mat& canvas,int x,int y,int w,const std::string& name,double value,double target,double range,double tol){
    cv::Rect bar(x,y,w,28); cv::rectangle(canvas,bar,{0,0,180},cv::FILLED);
    auto px=[&](double v){ return x+int(std::clamp((v+range)/(2*range),0.0,1.0)*w); };
    int g0=px(target-tol),g1=px(target+tol); cv::rectangle(canvas,{g0,y,std::max(1,g1-g0),28},{0,150,0},cv::FILLED);
    int z=px(0); cv::line(canvas,{z,y-4},{z,y+32},{150,150,150},1);
    int v=px(value); cv::line(canvas,{v,y-8},{v,y+36},{255,255,255},4,cv::LINE_AA);
    bool ok=std::abs(wrap180(value-target))<=tol; cv::Scalar c=ok?cv::Scalar(0,255,0):cv::Scalar(0,0,255);
    std::ostringstream ss; ss<<name<<"  "<<std::fixed<<std::setprecision(1)<<value<<" deg   target "<<target<<" +/-"<<tol;
    cv::putText(canvas,ss.str(),{x,y-12},cv::FONT_HERSHEY_SIMPLEX,0.56,c,2,cv::LINE_AA);
}
std::vector<std::string> wrapText(const std::string& s,size_t n){ std::vector<std::string> out; for(size_t p=0;p<s.size();p+=n) out.push_back(s.substr(p,n)); if(out.empty())out.push_back(""); return out; }

void drawSetup(cv::Mat& canvas,const std::string& plan){
    canvas.setTo(cv::Scalar(20,20,20));
    cv::putText(canvas,"JT-ZERO GUIDED CAMERA + IMU CALIBRATION",{60,70},cv::FONT_HERSHEY_SIMPLEX,1.0,{255,255,255},2,cv::LINE_AA);
    cv::putText(canvas,"TEST PLAN INPUT",{60,130},cv::FONT_HERSHEY_SIMPLEX,0.8,{0,220,255},2,cv::LINE_AA);
    cv::rectangle(canvas,{50,155,1180,250},{70,70,70},2);
    auto lines=wrapText(plan,100); int y=195; for(const auto& l:lines){ cv::putText(canvas,l,{70,y},cv::FONT_HERSHEY_SIMPLEX,0.56,{255,255,255},1,cv::LINE_AA); y+=32; if(y>380)break; }
    cv::putText(canvas,"Edit with keyboard (ASCII). ENTER = start   BACKSPACE = delete   F2 = default   ESC = exit",{60,455},cv::FONT_HERSHEY_SIMPLEX,0.6,{220,220,220},1,cv::LINE_AA);
    cv::putText(canvas,"Format: 0-10 roll +/-20-30; 10-20 pitch +/-20-30; 20-30 yaw +/-40-60; ...",{60,500},cv::FONT_HERSHEY_SIMPLEX,0.58,{180,220,255},1,cv::LINE_AA);
    cv::putText(canvas,"During test: center board on reticle and follow green target zones.",{60,560},cv::FONT_HERSHEY_SIMPLEX,0.62,{180,255,180},1,cv::LINE_AA);
}

void previewThreadMain(PreviewState* st){
    try{
        cv::namedWindow(PREVIEW_WINDOW,cv::WINDOW_NORMAL);
        cv::setWindowProperty(PREVIEW_WINDOW,cv::WND_PROP_FULLSCREEN,cv::WINDOW_FULLSCREEN);
        std::string edit=st->plan_text; uint64_t seen=0;
        while(true){
            PreviewPacket packet;
            { std::unique_lock<std::mutex> lk(st->mutex); st->cv.wait_for(lk,std::chrono::milliseconds(25),[&]{return st->stopping||st->generation!=seen;}); if(st->stopping)break; if(st->generation!=seen){packet=st->latest;seen=st->generation;} }
            cv::Mat canvas(720,1280,CV_8UC3,cv::Scalar(20,20,20));
            if(!st->plan_ready.load()){
                drawSetup(canvas,edit); cv::imshow(PREVIEW_WINDOW,canvas); int k=cv::waitKeyEx(1);
                if(k==27){st->stop_requested.store(true);break;}
                if(k==13||k==10){ double dur=0; auto phases=parsePlan(edit,&dur); {std::lock_guard<std::mutex> lk(st->mutex);st->plan_text=edit;st->phases=phases;st->plan_duration=dur;} st->plan_ready.store(true); }
                else if(k==0x710000||k==65471){ edit=DEFAULT_PLAN; }
                else if(k==8||k==127){ if(!edit.empty())edit.pop_back(); }
                else if(k>=32&&k<=126){ edit.push_back(char(k)); }
                continue;
            }
            cv::Mat gray; if(!packet.jpeg.empty()) gray=cv::imdecode(packet.jpeg,cv::IMREAD_GRAYSCALE);
            if(!gray.empty()){
                cv::Mat bgr; cv::cvtColor(gray,bgr,cv::COLOR_GRAY2BGR); cv::resize(bgr,bgr,{760,570}); bgr.copyTo(canvas(cv::Rect(20,110,760,570))); drawReticle(canvas,{400,395});
            }
            double roll=st->roll_deg.load(),pitch=st->pitch_deg.load(); double yaw=wrap180(st->yaw_deg.load()-st->yaw_zero_deg.load());
            Targets tg; std::vector<Phase> phases; {std::lock_guard<std::mutex> lk(st->mutex); phases=st->phases;} tg=targetsFor(phases,packet.elapsed_sec);
            std::ostringstream timer; timer<<std::fixed<<std::setprecision(1)<<"TIME "<<packet.elapsed_sec<<" / "<<packet.duration_sec<<" s   LEFT "<<std::max(0.0,packet.duration_sec-packet.elapsed_sec)<<" s";
            putOutlined(canvas,timer.str(),{25,52},0.82,2); putOutlined(canvas,tg.instruction,{25,92},0.72,2);
            cv::putText(canvas,"ATTITUDE / TARGET",{820,130},cv::FONT_HERSHEY_SIMPLEX,0.72,{255,255,255},2,cv::LINE_AA);
            if(!st->attitude_valid.load()) cv::putText(canvas,"WAITING FOR MAVLink ATTITUDE",{820,170},cv::FONT_HERSHEY_SIMPLEX,0.5,{0,0,255},2);
            drawGauge(canvas,820,220,410,"ROLL ",roll,tg.roll,60,5);
            drawGauge(canvas,820,320,410,"PITCH",pitch,tg.pitch,60,5);
            drawGauge(canvas,820,420,410,"YAW d",yaw,tg.yaw,90,7);
            cv::putText(canvas,"GREEN = inside target zone",{820,500},cv::FONT_HERSHEY_SIMPLEX,0.55,{0,255,0},1,cv::LINE_AA);
            cv::putText(canvas,"RED = correct attitude needed",{820,535},cv::FONT_HERSHEY_SIMPLEX,0.55,{0,0,255},1,cv::LINE_AA);
            cv::putText(canvas,"Q / ESC = stop test",{820,600},cv::FONT_HERSHEY_SIMPLEX,0.6,{255,255,255},1,cv::LINE_AA);
            cv::imshow(PREVIEW_WINDOW,canvas); int k=cv::waitKeyEx(1); if(k==27||k=='q'||k=='Q') st->stop_requested.store(true);
        }
        cv::destroyAllWindows();
    }catch(const cv::Exception& e){ std::cerr<<"[PREVIEW] OpenCV error: "<<e.what()<<'\n'; st->stop_requested.store(true); }
}

void submitPreview(PreviewState* st,const void* data,size_t size,double elapsed,double duration,uint32_t seq,bool running){
    PreviewPacket p; const auto* b=static_cast<const unsigned char*>(data); p.jpeg.assign(b,b+size); p.elapsed_sec=elapsed;p.duration_sec=duration;p.sequence=seq;p.running=running;
    {std::lock_guard<std::mutex> lk(st->mutex);st->latest=std::move(p);++st->generation;} st->cv.notify_one();
}
void stopPreview(PreviewState* st,std::thread* th){ {std::lock_guard<std::mutex> lk(st->mutex);st->stopping=true;}st->cv.notify_all();if(th&&th->joinable())th->join(); }

std::string cameraTimestampFlags(uint32_t flags){ std::string s=((flags&V4L2_BUF_FLAG_TIMESTAMP_MASK)==V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC)?"monotonic":"other_clock"; s+="|"; s+=((flags&V4L2_BUF_FLAG_TSTAMP_SRC_MASK)==V4L2_BUF_FLAG_TSTAMP_SRC_SOE)?"soe":"eof"; return s; }
void setCameraControl(int fd,uint32_t id,int32_t val,const char* name){ v4l2_control c{};c.id=id;c.value=val;if(xioctl(fd,VIDIOC_S_CTRL,&c)==-1)std::cerr<<"[WARN] "<<name<<" failed: "<<std::strerror(errno)<<'\n';else std::cout<<"[CAM] "<<name<<"="<<val<<'\n'; }
void configureCamera(int fd){
    v4l2_capability cap{};if(xioctl(fd,VIDIOC_QUERYCAP,&cap)==-1)fail("VIDIOC_QUERYCAP");std::cout<<"[CAM] driver="<<cap.driver<<" card="<<cap.card<<'\n';
    v4l2_format fmt{};fmt.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;fmt.fmt.pix.width=CAMERA_WIDTH;fmt.fmt.pix.height=CAMERA_HEIGHT;fmt.fmt.pix.pixelformat=V4L2_PIX_FMT_MJPEG;fmt.fmt.pix.field=V4L2_FIELD_ANY;if(xioctl(fd,VIDIOC_S_FMT,&fmt)==-1)fail("VIDIOC_S_FMT");
    v4l2_streamparm p{};p.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;p.parm.capture.timeperframe.numerator=1;p.parm.capture.timeperframe.denominator=CAMERA_FPS;if(xioctl(fd,VIDIOC_S_PARM,&p)==-1)fail("VIDIOC_S_PARM");double fps=p.parm.capture.timeperframe.numerator?double(p.parm.capture.timeperframe.denominator)/p.parm.capture.timeperframe.numerator:0;std::cout<<"[CAM] 640x480 MJPEG requested=120 actual="<<std::fixed<<std::setprecision(3)<<fps<<" FPS\n";
    setCameraControl(fd,V4L2_CID_EXPOSURE_AUTO,V4L2_EXPOSURE_MANUAL,"auto_exposure");setCameraControl(fd,V4L2_CID_EXPOSURE_AUTO_PRIORITY,0,"dynamic_framerate");setCameraControl(fd,V4L2_CID_EXPOSURE_ABSOLUTE,EXPOSURE_ABSOLUTE,"exposure_absolute");setCameraControl(fd,V4L2_CID_GAIN,CAMERA_GAIN,"gain");setCameraControl(fd,V4L2_CID_AUTO_WHITE_BALANCE,0,"white_balance_automatic");setCameraControl(fd,V4L2_CID_POWER_LINE_FREQUENCY,V4L2_CID_POWER_LINE_FREQUENCY_DISABLED,"power_line_frequency");setCameraControl(fd,V4L2_CID_BACKLIGHT_COMPENSATION,0,"backlight_compensation");
}
std::vector<CameraBuffer> initCameraBuffers(int fd){ v4l2_requestbuffers r{};r.count=CAMERA_BUFFER_COUNT;r.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;r.memory=V4L2_MEMORY_MMAP;if(xioctl(fd,VIDIOC_REQBUFS,&r)==-1)fail("VIDIOC_REQBUFS");std::vector<CameraBuffer>b(r.count);for(uint32_t i=0;i<r.count;++i){v4l2_buffer q{};q.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;q.memory=V4L2_MEMORY_MMAP;q.index=i;if(xioctl(fd,VIDIOC_QUERYBUF,&q)==-1)fail("VIDIOC_QUERYBUF");b[i].length=q.length;b[i].start=mmap(nullptr,q.length,PROT_READ|PROT_WRITE,MAP_SHARED,fd,q.m.offset);if(b[i].start==MAP_FAILED)fail("mmap");if(xioctl(fd,VIDIOC_QBUF,&q)==-1)fail("VIDIOC_QBUF");}return b; }
void discardWarmup(int fd,int count){std::cout<<"[CAM] discarding "<<count<<" warmup frames...\n";for(int n=0;n<count;){pollfd p{fd,POLLIN,0};if(poll(&p,1,1000)<=0)continue;v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;if(xioctl(fd,VIDIOC_DQBUF,&b)==-1){if(errno==EAGAIN)continue;fail("DQBUF warmup");}++n;if(xioctl(fd,VIDIOC_QBUF,&b)==-1)fail("QBUF warmup");}}
int openSerial(){int fd=open(SERIAL_DEVICE,O_RDWR|O_NOCTTY|O_NONBLOCK);if(fd==-1)fail("open serial");termios t{};if(tcgetattr(fd,&t)!=0)fail("tcgetattr");cfmakeraw(&t);if(cfsetispeed(&t,B460800)||cfsetospeed(&t,B460800))fail("baud");t.c_cflag|=CLOCAL|CREAD;t.c_cflag&=~CRTSCTS;t.c_cflag&=~PARENB;t.c_cflag&=~CSTOPB;t.c_cflag&=~CSIZE;t.c_cflag|=CS8;t.c_cc[VMIN]=0;t.c_cc[VTIME]=0;if(tcsetattr(fd,TCSANOW,&t)!=0)fail("tcsetattr");tcflush(fd,TCIFLUSH);std::cout<<"[MAV] "<<SERIAL_DEVICE<<" @ 460800\n";return fd;}
void serialWriteAll(int fd,const uint8_t* d,size_t n){for(size_t p=0;p<n;){ssize_t r=write(fd,d+p,n-p);if(r>0){p+=size_t(r);continue;}if(r==-1&&(errno==EAGAIN||errno==EWOULDBLOCK)){pollfd q{fd,POLLOUT,0};poll(&q,1,10);continue;}if(r==-1&&errno==EINTR)continue;fail("serial write");}}
void sendMsg(int fd,const mavlink_message_t& m){uint8_t b[MAVLINK_MAX_PACKET_LEN];uint16_t n=mavlink_msg_to_send_buffer(b,&m);serialWriteAll(fd,b,n);}
void requestRate(int fd,uint8_t sys,uint8_t comp,uint32_t msgid,int hz){mavlink_message_t m{};float us=hz>0?float(1000000.0/hz):0;mavlink_msg_command_long_pack(COMPANION_SYSID,COMPANION_COMPID,&m,sys,comp,MAV_CMD_SET_MESSAGE_INTERVAL,0,msgid,us,0,0,0,0,0);sendMsg(fd,m);}
void sendTimesync(int fd,int64_t t0,uint8_t sys,uint8_t comp){mavlink_message_t m{};mavlink_msg_timesync_pack(COMPANION_SYSID,COMPANION_COMPID,&m,0,t0,sys,comp);sendMsg(fd,m);}
ClockMapping estimateClockMapping(const std::vector<TimeSyncSample>& s){std::vector<const TimeSyncSample*>g;for(auto&x:s)if(x.good)g.push_back(&x);ClockMapping m;if(g.size()<10)return m;int64_t f0=g.front()->fc_ns,r0=g.front()->rpi_mid_ns;long double mx=0,my=0;for(auto*x:g){mx+=x->fc_ns-f0;my+=x->rpi_mid_ns-r0;}mx/=g.size();my/=g.size();long double sxx=0,sxy=0;for(auto*x:g){long double dx=(x->fc_ns-f0)-mx,dy=(x->rpi_mid_ns-r0)-my;sxx+=dx*dx;sxy+=dx*dy;}if(sxx<=0)return m;m.a=sxy/sxx;long double c=my-m.a*mx;m.valid=true;m.fc_ref_ns=f0;m.rpi_ref_ns=r0+c;m.drift_ppm=double((m.a-1)*1e6L);return m;}

} // namespace

int main(){
    int camera_fd=-1,serial_fd=-1;bool streaming=false,rates=false,preview_started=false;uint8_t target_system=0,target_component=0;std::vector<CameraBuffer> buffers;PreviewState ps;std::thread preview;
    try{
        serial_fd=openSerial();mavlink_status_t ms{};mavlink_message_t mm{};std::cout<<"[MAV] waiting for HEARTBEAT...\n";int64_t dl=monotonicNs()+5000000000LL;bool hb=false;
        while(monotonicNs()<dl&&!hb){pollfd p{serial_fd,POLLIN,0};if(poll(&p,1,100)<=0)continue;uint8_t by[4096];ssize_t n=read(serial_fd,by,sizeof(by));for(ssize_t i=0;i<n;++i)if(mavlink_parse_char(MAVLINK_COMM_0,by[i],&mm,&ms)&&mm.msgid==MAVLINK_MSG_ID_HEARTBEAT){target_system=mm.sysid;target_component=mm.compid;hb=true;break;}}
        if(!hb)throw std::runtime_error("MAVLink HEARTBEAT not received");std::cout<<"[MAV] connected system="<<int(target_system)<<" component="<<int(target_component)<<'\n';requestRate(serial_fd,target_system,target_component,MAVLINK_MSG_ID_HIGHRES_IMU,IMU_RATE_HZ);requestRate(serial_fd,target_system,target_component,MAVLINK_MSG_ID_ATTITUDE,ATTITUDE_RATE_HZ);rates=true;
        camera_fd=open(CAMERA_DEVICE,O_RDWR|O_NONBLOCK);if(camera_fd==-1)fail("open camera");configureCamera(camera_fd);buffers=initCameraBuffers(camera_fd);v4l2_buf_type typ=V4L2_BUF_TYPE_VIDEO_CAPTURE;if(xioctl(camera_fd,VIDIOC_STREAMON,&typ)==-1)fail("STREAMON");streaming=true;discardWarmup(camera_fd,CAMERA_WARMUP_FRAMES);
        preview=std::thread(previewThreadMain,&ps);preview_started=true;
        std::cout<<"[UI] Full-screen setup opened. Edit plan and press ENTER.\n";
        uint32_t preview_counter=0; Attitude latest_att;
        while(!ps.plan_ready.load()&&!ps.stop_requested.load()){
            pollfd pf[2]={{camera_fd,POLLIN,0},{serial_fd,POLLIN,0}};poll(pf,2,5);
            if(pf[0].revents&POLLIN){for(;;){v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;if(xioctl(camera_fd,VIDIOC_DQBUF,&b)==-1){if(errno==EAGAIN)break;fail("DQBUF setup");}if((preview_counter++%PREVIEW_DIVIDER)==0)submitPreview(&ps,buffers[b.index].start,b.bytesused,0,ps.plan_duration,b.sequence,false);if(xioctl(camera_fd,VIDIOC_QBUF,&b)==-1)fail("QBUF setup");}}
            if(pf[1].revents&POLLIN){uint8_t by[4096];ssize_t n=read(serial_fd,by,sizeof(by));for(ssize_t i=0;i<n;++i)if(mavlink_parse_char(MAVLINK_COMM_0,by[i],&mm,&ms)&&mm.msgid==MAVLINK_MSG_ID_ATTITUDE){mavlink_attitude_t a{};mavlink_msg_attitude_decode(&mm,&a);latest_att={rad2deg(a.roll),rad2deg(a.pitch),rad2deg(a.yaw),true};ps.roll_deg.store(latest_att.roll_deg);ps.pitch_deg.store(latest_att.pitch_deg);ps.yaw_deg.store(latest_att.yaw_deg);ps.attitude_valid.store(true);}}
        }
        if(ps.stop_requested.load())throw std::runtime_error("Cancelled in setup");
        double duration;std::vector<Phase> phases;{std::lock_guard<std::mutex>lk(ps.mutex);duration=ps.plan_duration;phases=ps.phases;}ps.yaw_zero_deg.store(latest_att.valid?latest_att.yaw_deg:ps.yaw_deg.load());
        tcflush(serial_fd,TCIFLUSH);std::memset(&ms,0,sizeof(ms));std::memset(&mm,0,sizeof(mm));
        std::vector<CameraSample> camera;std::vector<ImuSample> imu;std::vector<TimeSyncSample> ts;camera.reserve(size_t(duration*CAMERA_FPS+100));imu.reserve(size_t(duration*IMU_RATE_HZ+100));ts.reserve(size_t(duration*TIMESYNC_RATE_HZ+100));
        std::ofstream mjpeg(OUTPUT_MJPEG,std::ios::binary|std::ios::trunc);if(!mjpeg)throw std::runtime_error("Cannot create MJPEG output");uint64_t drops=0;bool have_seq=false;uint32_t prev_seq=0;int64_t pending=0;int64_t start=monotonicNs(),end=start+int64_t(duration*1e9),next_ts=start;preview_counter=0;
        std::cout<<"\n=== GUIDED CAMERA + IMU + TIMESYNC LOGGER ===\n"<<"duration:       "<<duration<<" s\n"<<"camera:         640x480 MJPEG @ 120 FPS\n"<<"preview:        fullscreen guided UI\n"<<"IMU:            HIGHRES_IMU @ 200 Hz\n"<<"ATTITUDE:       @ 50 Hz for UI\n"<<"TIMESYNC:       10 Hz\n"<<"camera offset:  +"<<jtzero::timesync::cameraToImuCorrectionMs()<<" ms\n\n";
        while(monotonicNs()<end&&!ps.stop_requested.load()){
            int64_t now=monotonicNs();if(now>=next_ts&&pending==0){pending=monotonicNs();sendTimesync(serial_fd,pending,target_system,target_component);next_ts=pending+TIMESYNC_PERIOD_NS;}
            pollfd pf[2]={{camera_fd,POLLIN,0},{serial_fd,POLLIN,0}};int rc=poll(pf,2,2);if(rc<0){if(errno==EINTR)continue;fail("poll");}
            if(pf[0].revents&POLLIN){for(;;){v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;if(xioctl(camera_fd,VIDIOC_DQBUF,&b)==-1){if(errno==EAGAIN)break;fail("DQBUF");}int64_t recv=monotonicNs(),v4=timevalToNs(b.timestamp),corr=jtzero::timesync::correctCameraTimestampNs(v4);if(have_seq){uint32_t ex=prev_seq+1;if(b.sequence!=ex)drops+=uint32_t(b.sequence-ex);}prev_seq=b.sequence;have_seq=true;std::streampos pos=mjpeg.tellp();uint64_t off=uint64_t(pos);mjpeg.write(static_cast<const char*>(buffers[b.index].start),b.bytesused);camera.push_back({recv,v4,corr,b.sequence,b.flags,b.bytesused,off});double elapsed=double(recv-start)/1e9;if((preview_counter++%PREVIEW_DIVIDER)==0)submitPreview(&ps,buffers[b.index].start,b.bytesused,elapsed,duration,b.sequence,true);if(xioctl(camera_fd,VIDIOC_QBUF,&b)==-1)fail("QBUF");}}
            if(pf[1].revents&POLLIN){uint8_t by[8192];for(;;){ssize_t n=read(serial_fd,by,sizeof(by));if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;for(ssize_t i=0;i<n;++i){if(!mavlink_parse_char(MAVLINK_COMM_0,by[i],&mm,&ms))continue;int64_t recv=monotonicNs();if(mm.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){mavlink_highres_imu_t a{};mavlink_msg_highres_imu_decode(&mm,&a);ImuSample s;s.recv_ns=recv;s.fc_ns=int64_t(a.time_usec)*1000;s.xacc=a.xacc;s.yacc=a.yacc;s.zacc=a.zacc;s.xgyro=a.xgyro;s.ygyro=a.ygyro;s.zgyro=a.zgyro;s.temperature=a.temperature;s.fields_updated=a.fields_updated;#ifdef MAVLINK_MSG_HIGHRES_IMU_FIELD_ID_LEN
s.imu_id=a.id;
#endif
imu.push_back(s);}else if(mm.msgid==MAVLINK_MSG_ID_ATTITUDE){mavlink_attitude_t a{};mavlink_msg_attitude_decode(&mm,&a);ps.roll_deg.store(rad2deg(a.roll));ps.pitch_deg.store(rad2deg(a.pitch));ps.yaw_deg.store(rad2deg(a.yaw));ps.attitude_valid.store(true);}else if(mm.msgid==MAVLINK_MSG_ID_TIMESYNC){mavlink_timesync_t a{};mavlink_msg_timesync_decode(&mm,&a);if(a.tc1!=0&&pending!=0&&a.ts1==pending){TimeSyncSample s;s.t0_rpi_ns=pending;s.t1_rpi_ns=recv;s.fc_ns=a.tc1;s.rtt_ns=recv-pending;s.rpi_mid_ns=pending+s.rtt_ns/2;s.good=s.rtt_ns>0&&nsToMs(s.rtt_ns)<=MAX_TIMESYNC_RTT_MS;ts.push_back(s);pending=0;}}}}}
            if(pending&&monotonicNs()-pending>20000000LL)pending=0;
        }
        mjpeg.flush();mjpeg.close();ClockMapping map=estimateClockMapping(ts);if(!map.valid)throw std::runtime_error("Not enough valid TIMESYNC samples");
        std::vector<double> camlat,camdt,imut,trtt;for(size_t i=0;i<camera.size();++i){camlat.push_back(nsToMs(camera[i].recv_ns-camera[i].v4l2_ns));if(i)camdt.push_back(nsToMs(camera[i].v4l2_ns-camera[i-1].v4l2_ns));}for(auto&s:imu)imut.push_back(nsToMs(s.recv_ns-map.map(s.fc_ns)));size_t good=0;for(auto&s:ts)if(s.good){++good;trtt.push_back(nsToMs(s.rtt_ns));}
        std::ofstream ci(OUTPUT_CAMERA_INDEX);ci<<"sequence,v4l2_timestamp_ns,camera_timestamp_corrected_ns,recv_rpi_ns,delivery_latency_ms,mjpeg_offset,bytes_used,flags\n"<<std::fixed<<std::setprecision(9);for(auto&s:camera)ci<<s.sequence<<','<<s.v4l2_ns<<','<<s.corrected_ns<<','<<s.recv_ns<<','<<nsToMs(s.recv_ns-s.v4l2_ns)<<','<<s.mjpeg_offset<<','<<s.bytes_used<<','<<cameraTimestampFlags(s.flags)<<'\n';ci.close();
        std::ofstream csv(OUTPUT_CSV);csv<<"event,recv_rpi_ns,source_timestamp_ns,mapped_rpi_ns,transport_latency_ms,camera_sequence,camera_flags,camera_bytes,xacc_m_s2,yacc_m_s2,zacc_m_s2,xgyro_rad_s,ygyro_rad_s,zgyro_rad_s,temperature_c,imu_id,timesync_t0_rpi_ns,timesync_t1_rpi_ns,timesync_fc_ns,timesync_mid_rpi_ns,timesync_rtt_ms,timesync_good,map_a,map_drift_ppm,map_fc_ref_ns,map_rpi_ref_ns\n"<<std::fixed<<std::setprecision(9);for(auto&s:camera)csv<<"CAMERA,"<<s.recv_ns<<','<<s.v4l2_ns<<','<<s.corrected_ns<<','<<nsToMs(s.recv_ns-s.v4l2_ns)<<','<<s.sequence<<','<<cameraTimestampFlags(s.flags)<<','<<s.bytes_used<<",,,,,,,,,,,,,,,"<<double(map.a)<<','<<map.drift_ppm<<','<<map.fc_ref_ns<<','<<int64_t(std::llround(map.rpi_ref_ns))<<'\n';for(auto&s:imu){int64_t mn=map.map(s.fc_ns);csv<<"IMU,"<<s.recv_ns<<','<<s.fc_ns<<','<<mn<<','<<nsToMs(s.recv_ns-mn)<<",,,,"<<s.xacc<<','<<s.yacc<<','<<s.zacc<<','<<s.xgyro<<','<<s.ygyro<<','<<s.zgyro<<','<<s.temperature<<','<<int(s.imu_id)<<",,,,,,"<<double(map.a)<<','<<map.drift_ppm<<','<<map.fc_ref_ns<<','<<int64_t(std::llround(map.rpi_ref_ns))<<'\n';}for(auto&s:ts)csv<<"TIMESYNC,"<<s.t1_rpi_ns<<",,,,,,,,,,,,,,,,"<<s.t0_rpi_ns<<','<<s.t1_rpi_ns<<','<<s.fc_ns<<','<<s.rpi_mid_ns<<','<<nsToMs(s.rtt_ns)<<','<<(s.good?1:0)<<','<<double(map.a)<<','<<map.drift_ppm<<','<<map.fc_ref_ns<<','<<int64_t(std::llround(map.rpi_ref_ns))<<'\n';csv.close();
        stopPreview(&ps,&preview);preview_started=false;
        std::cout<<"\n============================================================\nFINAL CLOCK MAPPING\n============================================================\n"<<std::setprecision(12)<<"A (RPi/FC):           "<<double(map.a)<<'\n'<<std::setprecision(3)<<"drift:                "<<map.drift_ppm<<" ppm\nTIMESYNC good:         "<<good<<'/'<<ts.size()<<'\n';if(!trtt.empty())std::cout<<"TIMESYNC RTT median:   "<<percentile(trtt,.5)<<" ms\nTIMESYNC RTT p95:      "<<percentile(trtt,.95)<<" ms\n";
        std::cout<<"\n============================================================\nCAMERA\n============================================================\nframes:                "<<camera.size()<<"\nsource drops:          "<<drops<<'\n';if(!camlat.empty())std::cout<<"delivery median:       "<<percentile(camlat,.5)<<" ms\ndelivery p95:          "<<percentile(camlat,.95)<<" ms\ndelivery p99:          "<<percentile(camlat,.99)<<" ms\ndelivery max:          "<<*std::max_element(camlat.begin(),camlat.end())<<" ms\n";if(!camdt.empty())std::cout<<"timestamp dt median:   "<<percentile(camdt,.5)<<" ms\ntimestamp dt p95:      "<<percentile(camdt,.95)<<" ms\n";
        std::cout<<"\n============================================================\nIMU\n============================================================\nsamples:               "<<imu.size()<<'\n';if(!imut.empty())std::cout<<"transport median:      "<<percentile(imut,.5)<<" ms\ntransport p95:         "<<percentile(imut,.95)<<" ms\ntransport p99:         "<<percentile(imut,.99)<<" ms\ntransport min:         "<<*std::min_element(imut.begin(),imut.end())<<" ms\ntransport max:         "<<*std::max_element(imut.begin(),imut.end())<<" ms\n";
        std::cout<<"\nCombined CSV:          "<<OUTPUT_CSV<<"\nCamera index CSV:      "<<OUTPUT_CAMERA_INDEX<<"\nCamera MJPEG:          "<<OUTPUT_MJPEG<<'\n';
        if(rates){requestRate(serial_fd,target_system,target_component,MAVLINK_MSG_ID_HIGHRES_IMU,0);requestRate(serial_fd,target_system,target_component,MAVLINK_MSG_ID_ATTITUDE,0);rates=false;}if(streaming){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(camera_fd,VIDIOC_STREAMOFF,&t);streaming=false;}for(auto&b:buffers)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(camera_fd!=-1)close(camera_fd);if(serial_fd!=-1)close(serial_fd);return 0;
    }catch(const std::exception& e){std::cerr<<"\n[FATAL] "<<e.what()<<'\n';if(preview_started)stopPreview(&ps,&preview);if(serial_fd!=-1&&rates&&target_system){try{requestRate(serial_fd,target_system,target_component,MAVLINK_MSG_ID_HIGHRES_IMU,0);requestRate(serial_fd,target_system,target_component,MAVLINK_MSG_ID_ATTITUDE,0);}catch(...){}}if(camera_fd!=-1&&streaming){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(camera_fd,VIDIOC_STREAMOFF,&t);}for(auto&b:buffers)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(camera_fd!=-1)close(camera_fd);if(serial_fd!=-1)close(serial_fd);return 1;}
}
