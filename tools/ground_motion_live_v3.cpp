// JT-Zero Ground Motion V3
// Правильная геометрия pitch/roll: луч каждого feature пересекается с горизонтальной
// плоскостью земли с учетом attitude FC, T_BS камеры и текущей высоты TF-Luna.
// Kimera metric state не используется.

#include <linux/videodev2.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <termios.h>
#include <unistd.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "common/mavlink.h"

namespace {
constexpr int kWidth=640, kHeight=480, kCameraFps=120;
constexpr int kExposureAbsolute=50, kGain=0;
constexpr double kPi=3.14159265358979323846;
constexpr const char* kWindow="JT-ZERO — GROUND MOTION V3";
std::atomic<bool> g_running{true};
void onSignal(int){g_running=false;}

int64_t monoNs(){timespec t{};if(clock_gettime(CLOCK_MONOTONIC,&t)!=0)throw std::runtime_error("clock_gettime");return (int64_t)t.tv_sec*1000000000LL+t.tv_nsec;}
void fail(const std::string&s){throw std::runtime_error(s+": "+std::strerror(errno));}
int xioctl(int fd,unsigned long req,void*arg){int r;do{r=ioctl(fd,req,arg);}while(r<0&&errno==EINTR);return r;}
void ru(cv::Mat&img,const std::string&s,cv::Point p,int size,cv::Scalar c,int weight=cv::QT_FONT_NORMAL){cv::addText(img,s,p,"DejaVu Sans",size,c,weight,cv::QT_STYLE_NORMAL,0);}

struct CameraBuffer{void*p=nullptr;size_t n=0;};
struct Camera{
  int fd=-1;std::vector<CameraBuffer>bufs;~Camera(){close();}
  void openDev(const std::string&dev){
    fd=::open(dev.c_str(),O_RDWR|O_NONBLOCK);if(fd<0)fail("open camera");
    v4l2_format f{};f.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;f.fmt.pix.width=kWidth;f.fmt.pix.height=kHeight;f.fmt.pix.pixelformat=V4L2_PIX_FMT_MJPEG;f.fmt.pix.field=V4L2_FIELD_ANY;
    if(xioctl(fd,VIDIOC_S_FMT,&f)<0)fail("VIDIOC_S_FMT");
    v4l2_streamparm sp{};sp.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;sp.parm.capture.timeperframe.numerator=1;sp.parm.capture.timeperframe.denominator=kCameraFps;if(xioctl(fd,VIDIOC_S_PARM,&sp)<0)fail("VIDIOC_S_PARM");
    auto setc=[&](uint32_t id,int32_t v){v4l2_control c{};c.id=id;c.value=v;xioctl(fd,VIDIOC_S_CTRL,&c);};
    setc(V4L2_CID_EXPOSURE_AUTO,V4L2_EXPOSURE_MANUAL);setc(V4L2_CID_EXPOSURE_AUTO_PRIORITY,0);setc(V4L2_CID_EXPOSURE_ABSOLUTE,kExposureAbsolute);setc(V4L2_CID_GAIN,kGain);
    v4l2_requestbuffers rb{};rb.count=8;rb.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;rb.memory=V4L2_MEMORY_MMAP;if(xioctl(fd,VIDIOC_REQBUFS,&rb)<0||rb.count<2)fail("VIDIOC_REQBUFS");
    bufs.resize(rb.count);
    for(unsigned i=0;i<rb.count;i++){v4l2_buffer b{};b.type=rb.type;b.memory=rb.memory;b.index=i;if(xioctl(fd,VIDIOC_QUERYBUF,&b)<0)fail("VIDIOC_QUERYBUF");bufs[i].n=b.length;bufs[i].p=mmap(nullptr,b.length,PROT_READ|PROT_WRITE,MAP_SHARED,fd,b.m.offset);if(bufs[i].p==MAP_FAILED)fail("mmap");if(xioctl(fd,VIDIOC_QBUF,&b)<0)fail("VIDIOC_QBUF");}
    v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;if(xioctl(fd,VIDIOC_STREAMON,&t)<0)fail("VIDIOC_STREAMON");
  }
  void close(){if(fd<0)return;v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(fd,VIDIOC_STREAMOFF,&t);for(auto&b:bufs)if(b.p&&b.p!=MAP_FAILED)munmap(b.p,b.n);::close(fd);fd=-1;}
};

struct LunaState{std::mutex mu;double distance_m=0;int strength=0;int64_t recv_ns=0;bool valid=false;};
struct LunaReader{
  int fd=-1;std::thread th;LunaState state;~LunaReader(){stop();}
  void start(const std::string&dev){
    fd=::open(dev.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK);if(fd<0)fail("open TF-Luna");termios t{};if(tcgetattr(fd,&t)<0)fail("TF-Luna tcgetattr");cfmakeraw(&t);cfsetispeed(&t,B115200);cfsetospeed(&t,B115200);t.c_cflag|=CLOCAL|CREAD;t.c_cflag&=~CSTOPB;t.c_cflag&=~CRTSCTS;if(tcsetattr(fd,TCSANOW,&t)<0)fail("TF-Luna tcsetattr");tcflush(fd,TCIFLUSH);
    th=std::thread([this]{std::vector<uint8_t>q;uint8_t tmp[128];while(g_running){pollfd p{fd,POLLIN,0};if(poll(&p,1,50)<=0)continue;ssize_t n=read(fd,tmp,sizeof(tmp));if(n<=0)continue;q.insert(q.end(),tmp,tmp+n);while(q.size()>=9){size_t s=0;while(s+1<q.size()&&!(q[s]==0x59&&q[s+1]==0x59))++s;if(s){q.erase(q.begin(),q.begin()+s);if(q.size()<9)break;}unsigned sum=0;for(int i=0;i<8;i++)sum+=q[i];bool ok=((sum&0xff)==q[8]);uint16_t d=q[2]|(uint16_t(q[3])<<8),st=q[4]|(uint16_t(q[5])<<8);if(ok&&d>0){std::lock_guard<std::mutex>l(state.mu);state.distance_m=d/100.0;state.strength=st;state.recv_ns=monoNs();state.valid=true;}q.erase(q.begin(),q.begin()+9);}}});
  }
  bool latest(double*d,int*s,int64_t*t){std::lock_guard<std::mutex>l(state.mu);if(!state.valid)return false;*d=state.distance_m;*s=state.strength;*t=state.recv_ns;return true;}
  void stop(){if(th.joinable())th.join();if(fd>=0){::close(fd);fd=-1;}}
};

struct Attitude{double roll=0,pitch=0,yaw=0;int64_t recv_ns=0;bool valid=false;};
struct FcReader{
  int fd=-1;std::thread th;std::mutex mu;Attitude att;~FcReader(){stop();}
  static void writeAll(int fd,const uint8_t*p,size_t n){size_t o=0;while(o<n){ssize_t k=write(fd,p+o,n-o);if(k>0){o+=k;continue;}if(k<0&&(errno==EAGAIN||errno==EWOULDBLOCK)){pollfd q{fd,POLLOUT,0};poll(&q,1,10);continue;}if(k<0&&errno==EINTR)continue;fail("FC write");}}
  static void requestRate(int fd,uint8_t sys,uint8_t comp,uint32_t msgid,int hz){mavlink_message_t m{};mavlink_msg_command_long_pack(255,190,&m,sys,comp,MAV_CMD_SET_MESSAGE_INTERVAL,0,msgid,1000000.0f/hz,0,0,0,0,0);uint8_t b[MAVLINK_MAX_PACKET_LEN];auto n=mavlink_msg_to_send_buffer(b,&m);writeAll(fd,b,n);}
  void start(const std::string&dev){
    fd=::open(dev.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK);if(fd<0)fail("open FC");termios t{};if(tcgetattr(fd,&t)<0)fail("FC tcgetattr");cfmakeraw(&t);cfsetispeed(&t,B460800);cfsetospeed(&t,B460800);t.c_cflag|=CLOCAL|CREAD;t.c_cflag&=~CRTSCTS;t.c_cflag&=~PARENB;t.c_cflag&=~CSTOPB;t.c_cflag&=~CSIZE;t.c_cflag|=CS8;if(tcsetattr(fd,TCSANOW,&t)<0)fail("FC tcsetattr");tcflush(fd,TCIFLUSH);
    th=std::thread([this]{mavlink_status_t st{};mavlink_message_t m{};uint8_t sys=0,comp=0,buf[4096];int64_t deadline=monoNs()+10000000000LL;while(g_running&&!sys&&monoNs()<deadline){pollfd p{fd,POLLIN,0};if(poll(&p,1,100)<=0)continue;ssize_t n=read(fd,buf,sizeof(buf));if(n<=0)continue;for(ssize_t i=0;i<n;i++)if(mavlink_parse_char(MAVLINK_COMM_0,buf[i],&m,&st)&&m.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=m.sysid;comp=m.compid;break;}}if(!sys){std::cerr<<"FC: HEARTBEAT timeout\n";g_running=false;return;}requestRate(fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,50);while(g_running){pollfd p{fd,POLLIN,0};if(poll(&p,1,50)<=0)continue;for(;;){ssize_t n=read(fd,buf,sizeof(buf));if(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;for(ssize_t i=0;i<n;i++)if(mavlink_parse_char(MAVLINK_COMM_0,buf[i],&m,&st)&&m.msgid==MAVLINK_MSG_ID_ATTITUDE){mavlink_attitude_t a{};mavlink_msg_attitude_decode(&m,&a);std::lock_guard<std::mutex>l(mu);att={a.roll,a.pitch,a.yaw,monoNs(),true};}}}});
  }
  bool latest(Attitude*a){std::lock_guard<std::mutex>l(mu);if(!att.valid)return false;*a=att;return true;}
  void stop(){if(th.joinable())th.join();if(fd>=0){::close(fd);fd=-1;}}
};

cv::Matx33d RzRyRx(double r,double p,double y){double cr=cos(r),sr=sin(r),cp=cos(p),sp=sin(p),cy=cos(y),sy=sin(y);return {cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr,sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr,-sp,cp*sr,cp*cr};}
cv::Matx33d attitudeFluToNwu(const Attitude&a){const cv::Matx33d S(1,0,0,0,-1,0,0,0,-1);return S*RzRyRx(a.roll,a.pitch,a.yaw)*S;}

struct CameraCalib{double fx=0,fy=0,cx=0,cy=0;cv::Matx33d B_R_C=cv::Matx33d::eye();cv::Mat K,D;};
CameraCalib loadCameraCalib(const std::string&path){
  cv::FileStorage fs(path,cv::FileStorage::READ);if(!fs.isOpened())throw std::runtime_error("не удалось открыть camera yaml");std::vector<double> intr,dist,data;fs["intrinsics"]>>intr;fs["distortion_coefficients"]>>dist;cv::FileNode tbs=fs["T_BS"];tbs["data"]>>data;if(intr.size()<4||data.size()!=16)throw std::runtime_error("camera yaml: неверные intrinsics/T_BS");
  CameraCalib c;c.fx=intr[0];c.fy=intr[1];c.cx=intr[2];c.cy=intr[3];for(int r=0;r<3;r++)for(int k=0;k<3;k++)c.B_R_C(r,k)=data[r*4+k];c.K=(cv::Mat_<double>(3,3)<<c.fx,0,c.cx,0,c.fy,c.cy,0,0,1);c.D=cv::Mat(dist).clone().reshape(1,1);return c;
}

bool footprint(const cv::Point2f&normalized,const cv::Matx33d&W_R_C,double height,cv::Vec2d*out){
  cv::Vec3d ray_c(normalized.x,normalized.y,1.0);cv::Vec3d ray_w=W_R_C*ray_c;if(ray_w[2]>=-1e-5)return false;double s=-height/ray_w[2];if(!(s>0)&&std::isfinite(s))return false;cv::Vec2d q(ray_w[0]*s,ray_w[1]*s);if(!std::isfinite(q[0])||!std::isfinite(q[1]))return false;*out=q;return true;
}

double median(std::vector<double>v){if(v.empty())return 0;size_t n=v.size()/2;std::nth_element(v.begin(),v.begin()+n,v.end());double m=v[n];if(v.size()%2==0){std::nth_element(v.begin(),v.begin()+n-1,v.end());m=0.5*(m+v[n-1]);}return m;}

bool robustDelta(const std::vector<cv::Vec2d>&d,cv::Vec2d*out,int*used,double*scatter){
  if(d.size()<12)return false;std::vector<double>xs,ys;xs.reserve(d.size());ys.reserve(d.size());for(auto&q:d){xs.push_back(q[0]);ys.push_back(q[1]);}cv::Vec2d m(median(xs),median(ys));std::vector<double>res;res.reserve(d.size());for(auto&q:d)res.push_back(cv::norm(q-m));double mr=median(res);double gate=std::max(0.0025,3.5*mr+0.0010);cv::Vec2d sum(0,0);int n=0;for(size_t i=0;i<d.size();++i)if(res[i]<=gate){sum+=d[i];++n;}if(n<10)return false;*out=sum*(1.0/n);*used=n;*scatter=mr;return true;
}

struct Estimate{double x=0,y=0,vx=0,vy=0,path=0,height=0;int inliers=0;double scatter=0;uint64_t frames=0;};
}

int main(int argc,char**argv){
  if(argc<7){std::cerr<<"Использование: "<<argv[0]<<" <camera> <luna> <fc> <csv> <camera_yaml> <camera_offset_mm>\n";return 2;}
  const std::string camdev=argv[1],lunadev=argv[2],fcdev=argv[3],csvpath=argv[4],yaml=argv[5];const double offset_m=std::stod(argv[6])/1000.0;
  try{
    CameraCalib calib=loadCameraCalib(yaml);Camera cam;cam.openDev(camdev);LunaReader luna;luna.start(lunadev);FcReader fc;fc.start(fcdev);std::ofstream csv(csvpath,std::ios::trunc);csv<<"mono_ns,frame,luna_slant_m,height_m,x_m,y_m,vx_mps,vy_mps,path_m,inliers,scatter_m,roll,pitch,yaw\n";
    cv::setNumThreads(1);std::signal(SIGINT,onSignal);std::signal(SIGTERM,onSignal);cv::namedWindow(kWindow,cv::WINDOW_NORMAL);cv::resizeWindow(kWindow,1280,720);cv::moveWindow(kWindow,0,0);
    enum class State{READY,MOVING,DONE};State state=State::READY;cv::Mat prev;Attitude prev_att{};double prev_h=0;int64_t prev_ns=0;Estimate est;uint64_t frame_id=0;double result_x=0,result_y=0;
    while(g_running){
      pollfd p{cam.fd,POLLIN,0};int pr=poll(&p,1,20);if(pr<0){if(errno==EINTR)continue;fail("camera poll");}if(pr<=0)continue;
      // Важно: цикл dequeue тоже обязан видеть запрос выхода. Иначе при
      // постоянном backlog камеры Q/ESC ставит g_running=false, но цикл
      // продолжает бесконечно получать кадры и окно визуально не закрывается.
      while(g_running){
        v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;if(xioctl(cam.fd,VIDIOC_DQBUF,&b)<0){if(errno==EAGAIN)break;fail("VIDIOC_DQBUF");}int64_t now=monoNs();cv::Mat raw(1,(int)b.bytesused,CV_8UC1,cam.bufs[b.index].p);cv::Mat gray=cv::imdecode(raw,cv::IMREAD_GRAYSCALE);if(xioctl(cam.fd,VIDIOC_QBUF,&b)<0)fail("VIDIOC_QBUF");if(gray.empty())continue;++frame_id;

        double luna_m=0;int strength=0;int64_t luna_ns=0;Attitude att{};bool have_luna=luna.latest(&luna_m,&strength,&luna_ns),have_att=fc.latest(&att);cv::Matx33d W_R_B=have_att?attitudeFluToNwu(att):cv::Matx33d::eye();cv::Vec3d beam_w=W_R_B*cv::Vec3d(0,0,-1);double beam_down=-beam_w[2];double h=(have_luna&&have_att)?luna_m*beam_down-offset_m:0;bool sensors_ok=have_luna&&have_att&&beam_down>0.20&&h>0.05&&(now-luna_ns)<200000000LL&&(now-att.recv_ns)<200000000LL;

        if(state==State::MOVING&&sensors_ok&&!prev.empty()&&prev_att.valid&&prev_h>0){
          std::vector<cv::Point2f>p0,p1;cv::goodFeaturesToTrack(prev,p0,700,0.01,7);if(p0.size()>=30){std::vector<uchar>st;std::vector<float>err;cv::calcOpticalFlowPyrLK(prev,gray,p0,p1,st,err,{21,21},3);std::vector<cv::Point2f>a,bp;for(size_t i=0;i<p0.size();++i)if(st[i]){a.push_back(p0[i]);bp.push_back(p1[i]);}if(a.size()>=20){cv::Mat mask;cv::findHomography(a,bp,cv::RANSAC,2.0,mask);if(!mask.empty()){
            std::vector<cv::Point2f>ai,bi;for(size_t i=0;i<a.size();++i)if(mask.at<uchar>((int)i)){ai.push_back(a[i]);bi.push_back(bp[i]);}
            if(ai.size()>=15){std::vector<cv::Point2f>au,bu;cv::undistortPoints(ai,au,calib.K,calib.D);cv::undistortPoints(bi,bu,calib.K,calib.D);cv::Matx33d W_R_C0=attitudeFluToNwu(prev_att)*calib.B_R_C;cv::Matx33d W_R_C1=W_R_B*calib.B_R_C;std::vector<cv::Vec2d>deltas;deltas.reserve(au.size());for(size_t i=0;i<au.size();++i){cv::Vec2d g0,g1;if(footprint(au[i],W_R_C0,prev_h,&g0)&&footprint(bu[i],W_R_C1,h,&g1))deltas.push_back(g0-g1);}cv::Vec2d dxy;int used=0;double scatter=0;double dt=prev_ns?((now-prev_ns)*1e-9):0;if(dt>0&&dt<0.2&&robustDelta(deltas,&dxy,&used,&scatter)&&cv::norm(dxy)<0.20){est.x+=dxy[0];est.y+=dxy[1];est.path+=cv::norm(dxy);est.vx=dxy[0]/dt;est.vy=dxy[1]/dt;est.height=h;est.inliers=used;est.scatter=scatter;++est.frames;}}}
          }}
        }

        if(sensors_ok){prev=gray.clone();prev_att=att;prev_h=h;prev_ns=now;}else{prev.release();prev_att.valid=false;prev_h=0;prev_ns=0;}

        cv::Mat bgr,video;cv::cvtColor(gray,bgr,cv::COLOR_GRAY2BGR);cv::resize(bgr,video,{900,675});cv::Mat canvas(720,1280,CV_8UC3,cv::Scalar(12,12,12));video.copyTo(canvas(cv::Rect(0,45,900,675)));ru(canvas,"JT-ZERO — GROUND MOTION V3 — ПЛОСКОСТЬ ЗЕМЛИ",{22,31},18,{245,245,245},cv::QT_FONT_BOLD);cv::Mat panel=canvas(cv::Rect(900,0,380,720));
        ru(panel,sensors_ok?"СИСТЕМА ГОТОВА":"ЖДИТЕ ДАТЧИКИ",{18,48},16,sensors_ok?cv::Scalar(90,220,90):cv::Scalar(0,210,255),cv::QT_FONT_BOLD);const char*now_text=state==State::READY?"СЕЙЧАС: ТОЧКА A — НЕ ДВИГАТЬ":state==State::MOVING?"СЕЙЧАС: ДВИЖЕНИЕ A -> B":"СЕЙЧАС: РЕЗУЛЬТАТ ЗАФИКСИРОВАН";const char*next_text=state==State::READY?"ПРОБЕЛ — СТАРТ":state==State::MOVING?"В ТОЧКЕ B: ПРОБЕЛ — СТОП":"Q / ESC — ВЫХОД";ru(panel,now_text,{18,105},12,{255,255,255},cv::QT_FONT_BOLD);ru(panel,next_text,{18,145},12,{235,235,235},cv::QT_FONT_BOLD);
        char z[160];snprintf(z,sizeof(z),"TF-Luna луч: %.1f см",luna_m*100);ru(panel,z,{18,205},11,{220,220,220});snprintf(z,sizeof(z),"Вертикальная H: %.1f мм",h*1000);ru(panel,z,{18,240},11,{220,220,220});snprintf(z,sizeof(z),"X мира: %+.1f мм",est.x*1000);ru(panel,z,{18,305},14,{245,245,245},cv::QT_FONT_BOLD);snprintf(z,sizeof(z),"Y мира: %+.1f мм",est.y*1000);ru(panel,z,{18,345},14,{245,245,245},cv::QT_FONT_BOLD);snprintf(z,sizeof(z),"Путь: %.1f мм",est.path*1000);ru(panel,z,{18,385},13,{245,245,245},cv::QT_FONT_BOLD);snprintf(z,sizeof(z),"Vx/Vy: %+.3f / %+.3f",est.vx,est.vy);ru(panel,z,{18,440},11,{220,220,220});snprintf(z,sizeof(z),"Точек: %d",est.inliers);ru(panel,z,{18,485},11,{220,220,220});snprintf(z,sizeof(z),"Разброс: %.2f мм",est.scatter*1000);ru(panel,z,{18,520},11,{220,220,220});snprintf(z,sizeof(z),"Крен/тангаж: %+.2f / %+.2f°",att.roll*180/kPi,att.pitch*180/kPi);ru(panel,z,{18,565},11,{220,220,220});if(state==State::DONE){snprintf(z,sizeof(z),"ИТОГ NET: %.1f мм",std::hypot(result_x,result_y)*1000);ru(panel,z,{18,625},13,{245,245,245},cv::QT_FONT_BOLD);}ru(panel,"Q / ESC — ВЫХОД",{18,685},11,{210,210,210});cv::imshow(kWindow,canvas);

        // waitKeyEx даёт полный код клавиши; low byte покрывает Q/q/ESC и
        // не зависит от особенностей Qt backend. 10 мс даёт Qt время
        // обработать события клавиатуры и закрытия окна даже под VNC.
        const int raw_key=cv::waitKeyEx(10);
        const int key=(raw_key<0)?-1:(raw_key&0xff);
        if(cv::getWindowProperty(kWindow,cv::WND_PROP_VISIBLE)<1){g_running=false;break;}
        if(key==' '&&sensors_ok){if(state==State::READY){est={};prev.release();prev_att.valid=false;prev_h=0;prev_ns=0;state=State::MOVING;}else if(state==State::MOVING){result_x=est.x;result_y=est.y;est.vx=est.vy=0;state=State::DONE;}}
        if(key=='q'||key=='Q'||key==27){g_running=false;break;}

        if(csv&&sensors_ok){csv<<now<<','<<frame_id<<','<<std::fixed<<std::setprecision(6)<<luna_m<<','<<h<<','<<est.x<<','<<est.y<<','<<est.vx<<','<<est.vy<<','<<est.path<<','<<est.inliers<<','<<est.scatter<<','<<att.roll<<','<<att.pitch<<','<<att.yaw<<'\n';if((frame_id&3u)==0u)csv.flush();}
      }
    }
    cv::destroyAllWindows();return 0;
  }catch(const std::exception&e){std::cerr<<"GROUND MOTION V3 FAIL: "<<e.what()<<"\n";return 1;}
}