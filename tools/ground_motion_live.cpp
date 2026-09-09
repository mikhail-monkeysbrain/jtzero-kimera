// JT-Zero Ground Motion Live v1.
// Independent of Kimera metric state. Uses OV9281 + direct TF-Luna + FC attitude.
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
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d.hpp>
#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
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
constexpr const char* kWindow="JT-ZERO — ОЦЕНКА ДВИЖЕНИЯ ПО ЗЕМЛЕ";
constexpr double kPi=3.14159265358979323846;
std::atomic<bool> g_running{true};

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
    fd=::open(dev.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK);if(fd<0)fail("open TF-Luna");
    termios t{};if(tcgetattr(fd,&t)<0)fail("TF-Luna tcgetattr");cfmakeraw(&t);cfsetispeed(&t,B115200);cfsetospeed(&t,B115200);t.c_cflag|=CLOCAL|CREAD;t.c_cflag&=~CSTOPB;t.c_cflag&=~CRTSCTS;if(tcsetattr(fd,TCSANOW,&t)<0)fail("TF-Luna tcsetattr");tcflush(fd,TCIFLUSH);
    th=std::thread([this]{std::vector<uint8_t>q;q.reserve(256);uint8_t tmp[128];while(g_running){pollfd p{fd,POLLIN,0};if(poll(&p,1,50)<=0)continue;ssize_t n=read(fd,tmp,sizeof(tmp));if(n<=0)continue;q.insert(q.end(),tmp,tmp+n);while(q.size()>=9){size_t s=0;while(s+1<q.size()&&!(q[s]==0x59&&q[s+1]==0x59))++s;if(s){q.erase(q.begin(),q.begin()+s);if(q.size()<9)break;}unsigned sum=0;for(int i=0;i<8;i++)sum+=q[i];bool ok=((sum&0xff)==q[8]);uint16_t d=q[2]|(uint16_t(q[3])<<8),st=q[4]|(uint16_t(q[5])<<8);if(ok&&d>0){std::lock_guard<std::mutex>l(state.mu);state.distance_m=d/100.0;state.strength=st;state.recv_ns=monoNs();state.valid=true;}q.erase(q.begin(),q.begin()+9);}}});
  }
  bool latest(double*d,int*strength,int64_t*recv){std::lock_guard<std::mutex>l(state.mu);if(!state.valid)return false;*d=state.distance_m;*strength=state.strength;*recv=state.recv_ns;return true;}
  void stop(){if(th.joinable())th.join();if(fd>=0){::close(fd);fd=-1;}}
};

struct Attitude{double roll=0,pitch=0,yaw=0;int64_t recv_ns=0;bool valid=false;};
struct FcReader{
  int fd=-1;std::thread th;std::mutex mu;Attitude att;~FcReader(){stop();}
  static void writeAll(int fd,const uint8_t*p,size_t n){size_t o=0;while(o<n){ssize_t k=write(fd,p+o,n-o);if(k>0){o+=k;continue;}if(k<0&&(errno==EAGAIN||errno==EWOULDBLOCK)){pollfd q{fd,POLLOUT,0};poll(&q,1,10);continue;}if(k<0&&errno==EINTR)continue;fail("FC write");}}
  static void sendMsg(int fd,const mavlink_message_t&m){uint8_t b[MAVLINK_MAX_PACKET_LEN];auto n=mavlink_msg_to_send_buffer(b,&m);writeAll(fd,b,n);}
  static void requestRate(int fd,uint8_t sys,uint8_t comp,uint32_t msgid,int hz){mavlink_message_t m{};mavlink_msg_command_long_pack(255,190,&m,sys,comp,MAV_CMD_SET_MESSAGE_INTERVAL,0,msgid,1000000.0f/hz,0,0,0,0,0);sendMsg(fd,m);}
  void start(const std::string&dev){
    fd=::open(dev.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK);if(fd<0)fail("open FC");termios t{};if(tcgetattr(fd,&t)<0)fail("FC tcgetattr");cfmakeraw(&t);cfsetispeed(&t,B460800);cfsetospeed(&t,B460800);t.c_cflag|=CLOCAL|CREAD;t.c_cflag&=~CRTSCTS;t.c_cflag&=~PARENB;t.c_cflag&=~CSTOPB;t.c_cflag&=~CSIZE;t.c_cflag|=CS8;if(tcsetattr(fd,TCSANOW,&t)<0)fail("FC tcsetattr");tcflush(fd,TCIFLUSH);
    th=std::thread([this]{mavlink_status_t st{};mavlink_message_t m{};uint8_t sys=0,comp=0,buf[4096];int64_t deadline=monoNs()+10000000000LL;while(g_running&&!sys&&monoNs()<deadline){pollfd p{fd,POLLIN,0};if(poll(&p,1,100)<=0)continue;ssize_t n=read(fd,buf,sizeof(buf));if(n<=0)continue;for(ssize_t i=0;i<n;i++)if(mavlink_parse_char(MAVLINK_COMM_0,buf[i],&m,&st)&&m.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=m.sysid;comp=m.compid;break;}}if(!sys){std::cerr<<"FC: HEARTBEAT timeout\n";g_running=false;return;}requestRate(fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,50);while(g_running){pollfd p{fd,POLLIN,0};if(poll(&p,1,50)<=0)continue;for(;;){ssize_t n=read(fd,buf,sizeof(buf));if(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;for(ssize_t i=0;i<n;i++)if(mavlink_parse_char(MAVLINK_COMM_0,buf[i],&m,&st)&&m.msgid==MAVLINK_MSG_ID_ATTITUDE){mavlink_attitude_t a{};mavlink_msg_attitude_decode(&m,&a);std::lock_guard<std::mutex>l(mu);att={a.roll,a.pitch,a.yaw,monoNs(),true};}}}});
  }
  bool latest(Attitude*a){std::lock_guard<std::mutex>l(mu);if(!att.valid)return false;*a=att;return true;}
  void stop(){if(th.joinable())th.join();if(fd>=0){::close(fd);fd=-1;}}
};

cv::Matx33d RzRyRx(double r,double p,double y){double cr=cos(r),sr=sin(r),cp=cos(p),sp=sin(p),cy=cos(y),sy=sin(y);return {cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr,sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr,-sp,cp*sr,cp*cr};}
struct CameraCalib{
  double fx=0,fy=0,cx=0,cy=0;
  cv::Matx33d B_R_C=cv::Matx33d::eye();
};
CameraCalib loadCameraCalib(const std::string& path){
  cv::FileStorage fs(path,cv::FileStorage::READ);
  if(!fs.isOpened())throw std::runtime_error("не удалось открыть camera yaml");
  std::vector<double> intr;fs["intrinsics"]>>intr;
  if(intr.size()<4)throw std::runtime_error("camera yaml: intrinsics");
  cv::Mat T;fs["T_BS"]>>T;
  if(T.rows!=4||T.cols!=4)throw std::runtime_error("camera yaml: T_BS");
  T.convertTo(T,CV_64F);
  CameraCalib k;k.fx=intr[0];k.fy=intr[1];k.cx=intr[2];k.cy=intr[3];
  for(int r=0;r<3;r++)for(int col=0;col<3;col++)k.B_R_C(r,col)=T.at<double>(r,col);
  return k;
}
struct Estimate{double x=0,y=0,vx=0,vy=0,path=0,height=0;int inliers=0;uint64_t frames=0;};
}

int main(int argc,char**argv){
  if(argc<8){std::cerr<<"Использование: "<<argv[0]<<" <camera> <luna_serial> <fc_serial> <out_csv> <fx> <fy> <camera_offset_mm>\n";return 2;}
  const std::string camdev=argv[1],lunadev=argv[2],fcdev=argv[3],csvpath=argv[4];const double fx=std::stod(argv[5]),fy=std::stod(argv[6]),offset_m=std::stod(argv[7])/1000.0;
  try{
    Camera cam;cam.openDev(camdev);LunaReader luna;luna.start(lunadev);FcReader fc;fc.start(fcdev);std::ofstream csv(csvpath,std::ios::trunc);csv<<"mono_ns,frame,height_m,x_m,y_m,vx_mps,vy_mps,path_m,inliers,roll,pitch,yaw\n";
    cv::setNumThreads(1);cv::namedWindow(kWindow,cv::WINDOW_NORMAL);cv::setWindowProperty(kWindow,cv::WND_PROP_FULLSCREEN,cv::WINDOW_FULLSCREEN);
    cv::Mat prev;Attitude prev_att{};int64_t prev_ns=0;Estimate est;bool reset_pending=true;bool armed=false;uint64_t frame_id=0;
    while(g_running){
      pollfd p{cam.fd,POLLIN,0};int pr=poll(&p,1,20);if(pr<0){if(errno==EINTR)continue;fail("camera poll");}if(pr<=0)continue;
      for(;;){
        v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;if(xioctl(cam.fd,VIDIOC_DQBUF,&b)<0){if(errno==EAGAIN)break;fail("VIDIOC_DQBUF");}
        const int64_t now=monoNs();cv::Mat raw(1,(int)b.bytesused,CV_8UC1,cam.bufs[b.index].p);cv::Mat gray=cv::imdecode(raw,cv::IMREAD_GRAYSCALE);if(xioctl(cam.fd,VIDIOC_QBUF,&b)<0)fail("VIDIOC_QBUF");if(gray.empty())continue;frame_id++;
        double luna_m=0;int strength=0;int64_t luna_ns=0;Attitude att{};bool have_luna=luna.latest(&luna_m,&strength,&luna_ns),have_att=fc.latest(&att);double h=have_luna?luna_m-offset_m:0;bool sensors_ok=have_luna&&have_att&&h>0.05&&(now-luna_ns)<200000000LL&&(now-att.recv_ns)<200000000LL;
        if(reset_pending){prev.release();est={};reset_pending=false;}
        if(armed&&sensors_ok&&!prev.empty()&&prev_att.valid){
          std::vector<cv::Point2f>p0,p1;cv::goodFeaturesToTrack(prev,p0,700,0.01,7);
          if(p0.size()>=30){
            std::vector<uchar>st;std::vector<float>er;cv::calcOpticalFlowPyrLK(prev,gray,p0,p1,st,er,{21,21},3);
            std::vector<cv::Point2f>a,bp;for(size_t i=0;i<p0.size();i++)if(st[i]){a.push_back(p0[i]);bp.push_back(p1[i]);}
            if(a.size()>=20){
              cv::Mat mask;cv::Mat H=cv::findHomography(a,bp,cv::RANSAC,2.0,mask);int nin=mask.empty()?0:cv::countNonZero(mask);
              if(!H.empty()&&nin>=15){
                cv::Matx33d K(calib.fx,0,calib.cx,0,calib.fy,calib.cy,0,0,1),Ki=K.inv();
                cv::Matx33d W_R_C0=RzRyRx(prev_att.roll,prev_att.pitch,prev_att.yaw)*calib.B_R_C;
                cv::Matx33d W_R_C1=RzRyRx(att.roll,att.pitch,att.yaw)*calib.B_R_C;
                cv::Matx33d Hr=K*(W_R_C1.t()*W_R_C0)*Ki;
                cv::Mat Hrd(3,3,CV_64F);for(int r=0;r<3;r++)for(int c=0;c<3;c++)Hrd.at<double>(r,c)=Hr(r,c);cv::Mat Hd=Hrd.inv()*H;Hd/=Hd.at<double>(2,2);
                cv::Matx31d c0(calib.cx,calib.cy,1),q;for(int r=0;r<3;r++)q(r)=Hd.at<double>(r,0)*c0(0)+Hd.at<double>(r,1)*c0(1)+Hd.at<double>(r,2);
                double dx=q(0)/q(2)-calib.cx,dy=q(1)/q(2)-calib.cy,mx=-dx*h/calib.fx,my=-dy*h/calib.fy,dt=prev_ns?((now-prev_ns)*1e-9):0;
                if(dt>0&&dt<0.2){est.x+=mx;est.y+=my;est.path+=std::hypot(mx,my);est.vx=mx/dt;est.vy=my/dt;est.height=h;est.inliers=nin;est.frames++;}
              }
            }
          }
        }
        prev=gray.clone();prev_att=att;prev_ns=now;
        cv::Mat bgr,video;cv::cvtColor(gray,bgr,cv::COLOR_GRAY2BGR);cv::resize(bgr,video,{900,675});cv::Mat canvas(720,1280,CV_8UC3,cv::Scalar(12,12,12));video.copyTo(canvas(cv::Rect(0,45,900,675)));ru(canvas,"JT-ZERO — ОЦЕНКА ДВИЖЕНИЯ ПО ЗЕМЛЕ",{22,31},20,{245,245,245},cv::QT_FONT_BOLD);cv::Mat panel=canvas(cv::Rect(900,0,380,720));
        ru(panel,sensors_ok?"СИСТЕМА ГОТОВА":"ЖДИТЕ ДАТЧИКИ",{18,48},16,sensors_ok?cv::Scalar(90,220,90):cv::Scalar(0,210,255),cv::QT_FONT_BOLD);
        ru(panel,armed?"СЕЙЧАС: ДВИГАЙТЕ СТЕНД":"СЕЙЧАС: НЕ ДВИГАТЬ",{18,105},14,{255,255,255},cv::QT_FONT_BOLD);
        ru(panel,armed?"ДАЛЬШЕ: Q / ESC — ЗАВЕРШИТЬ":"ДАЛЬШЕ: ПРОБЕЛ — ОБНУЛИТЬ",{18,145},12,{235,235,235},cv::QT_FONT_BOLD);
        char z[160];snprintf(z,sizeof(z),"TF-Luna: %.1f см",luna_m*100);ru(panel,z,{18,210},12,{220,220,220});snprintf(z,sizeof(z),"Высота камеры: %.1f мм",h*1000);ru(panel,z,{18,245},12,{220,220,220});snprintf(z,sizeof(z),"X: %+.1f мм",est.x*1000);ru(panel,z,{18,315},15,{245,245,245},cv::QT_FONT_BOLD);snprintf(z,sizeof(z),"Y: %+.1f мм",est.y*1000);ru(panel,z,{18,355},15,{245,245,245},cv::QT_FONT_BOLD);snprintf(z,sizeof(z),"Путь: %.1f мм",est.path*1000);ru(panel,z,{18,395},14,{245,245,245},cv::QT_FONT_BOLD);snprintf(z,sizeof(z),"Vx: %+.3f м/с",est.vx);ru(panel,z,{18,455},12,{220,220,220});snprintf(z,sizeof(z),"Vy: %+.3f м/с",est.vy);ru(panel,z,{18,490},12,{220,220,220});snprintf(z,sizeof(z),"Геом. точек: %d",est.inliers);ru(panel,z,{18,550},12,{220,220,220});snprintf(z,sizeof(z),"Крен/тангаж: %+.2f / %+.2f°",att.roll*180/kPi,att.pitch*180/kPi);ru(panel,z,{18,585},11,{220,220,220});ru(panel,"Q / ESC — ВЫХОД",{18,685},11,{210,210,210});cv::imshow(kWindow,canvas);int k=cv::waitKey(1);if(k==' '&&sensors_ok){reset_pending=true;armed=true;}if(k=='q'||k=='Q'||k==27)g_running=false;
        if(csv&&sensors_ok)csv<<now<<','<<frame_id<<','<<std::fixed<<std::setprecision(6)<<h<<','<<est.x<<','<<est.y<<','<<est.vx<<','<<est.vy<<','<<est.path<<','<<est.inliers<<','<<att.roll<<','<<att.pitch<<','<<att.yaw<<'\n';
      }
    }
    return 0;
  }catch(const std::exception&e){std::cerr<<"GROUND MOTION LIVE FAIL: "<<e.what()<<"\n";return 1;}
}
