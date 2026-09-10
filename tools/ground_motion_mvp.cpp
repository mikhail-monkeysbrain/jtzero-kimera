// JT-Zero Ground Motion MVP
// Production-контур: один estimator, без диагностических A/B веток.
// Использует проверенный V3 ray-to-ground-plane, SYNC attitude и CAD lever Luna->camera.
#define main jtzero_v2_v3_base_unused_main
#include "ground_motion_live_v2_v3_ab.cpp"
#undef main
#include "ground_motion_mavlink.hpp"
#include <deque>
#include <cstdlib>

namespace {
int64_t frameTvToNsMvp(const timeval& tv){return (int64_t)tv.tv_sec*1000000000LL+(int64_t)tv.tv_usec*1000LL;}
double wrapMvp(double a){while(a>kPi)a-=2*kPi;while(a<-kPi)a+=2*kPi;return a;}

struct FcHistory {
  int fd=-1; std::thread th; std::mutex mu; std::deque<Attitude> hist;
  ~FcHistory(){stop();}
  static void writeAll(int fd,const uint8_t*p,size_t n){size_t o=0;while(o<n){ssize_t k=write(fd,p+o,n-o);if(k>0){o+=k;continue;}if(k<0&&(errno==EAGAIN||errno==EWOULDBLOCK)){pollfd q{fd,POLLOUT,0};poll(&q,1,10);continue;}if(k<0&&errno==EINTR)continue;fail("FC write");}}
  static void requestRate(int fd,uint8_t sys,uint8_t comp,uint32_t msgid,int hz){mavlink_message_t m{};mavlink_msg_command_long_pack(255,190,&m,sys,comp,MAV_CMD_SET_MESSAGE_INTERVAL,0,msgid,1000000.0f/hz,0,0,0,0,0);uint8_t b[MAVLINK_MAX_PACKET_LEN];auto n=mavlink_msg_to_send_buffer(b,&m);writeAll(fd,b,n);}
  void start(const std::string&dev){fd=::open(dev.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK);if(fd<0)fail("open FC");termios t{};if(tcgetattr(fd,&t)<0)fail("FC tcgetattr");cfmakeraw(&t);cfsetispeed(&t,B460800);cfsetospeed(&t,B460800);t.c_cflag|=CLOCAL|CREAD;t.c_cflag&=~CRTSCTS;t.c_cflag&=~PARENB;t.c_cflag&=~CSTOPB;t.c_cflag&=~CSIZE;t.c_cflag|=CS8;if(tcsetattr(fd,TCSANOW,&t)<0)fail("FC tcsetattr");tcflush(fd,TCIFLUSH);th=std::thread([this]{mavlink_status_t st{};mavlink_message_t m{};uint8_t sys=0,comp=0,buf[4096];int64_t deadline=monoNs()+10000000000LL;while(g_running&&!sys&&monoNs()<deadline){pollfd p{fd,POLLIN,0};if(poll(&p,1,100)<=0)continue;ssize_t n=read(fd,buf,sizeof(buf));if(n<=0)continue;for(ssize_t i=0;i<n;i++)if(mavlink_parse_char(MAVLINK_COMM_0,buf[i],&m,&st)&&m.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=m.sysid;comp=m.compid;break;}}if(!sys){std::cerr<<"FC: HEARTBEAT timeout\n";g_running=false;return;}requestRate(fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,50);while(g_running){pollfd p{fd,POLLIN,0};if(poll(&p,1,50)<=0)continue;for(;;){ssize_t n=read(fd,buf,sizeof(buf));if(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;for(ssize_t i=0;i<n;i++)if(mavlink_parse_char(MAVLINK_COMM_0,buf[i],&m,&st)&&m.msgid==MAVLINK_MSG_ID_ATTITUDE){mavlink_attitude_t a{};mavlink_msg_attitude_decode(&m,&a);Attitude x{a.roll,a.pitch,a.yaw,monoNs(),true};std::lock_guard<std::mutex>l(mu);hist.push_back(x);int64_t keep=x.recv_ns-3000000000LL;while(hist.size()>2&&hist.front().recv_ns<keep)hist.pop_front();}}}});}
  bool at(int64_t target,Attitude*out,double*nearest_ms){std::lock_guard<std::mutex>l(mu);if(hist.size()<2||target<hist.front().recv_ns||target>hist.back().recv_ns)return false;size_t hi=1;while(hi<hist.size()&&hist[hi].recv_ns<target)++hi;if(hi>=hist.size())return false;const auto&a=hist[hi-1];const auto&b=hist[hi];double u=double(target-a.recv_ns)/double(b.recv_ns-a.recv_ns);out->roll=a.roll+u*(b.roll-a.roll);out->pitch=a.pitch+u*(b.pitch-a.pitch);out->yaw=wrapMvp(a.yaw+u*wrapMvp(b.yaw-a.yaw));out->recv_ns=target;out->valid=true;if(nearest_ms)*nearest_ms=std::min(std::llabs(target-a.recv_ns),std::llabs(b.recv_ns-target))*1e-6;return true;}
  void stop(){if(th.joinable())th.join();if(fd>=0){::close(fd);fd=-1;}}
};

bool v3StepMvp(const std::vector<cv::Point2f>&ai,const std::vector<cv::Point2f>&bi,const CameraCalib&c,const Attitude&a0,double h0,const Attitude&a1,double h1,cv::Vec2d*out,int*used,double*scatter){std::vector<cv::Point2f>au,bu;cv::undistortPoints(ai,au,c.K,c.D);cv::undistortPoints(bi,bu,c.K,c.D);auto R0=attitudeFluToNwu(a0)*c.B_R_C;auto R1=attitudeFluToNwu(a1)*c.B_R_C;std::vector<cv::Vec2d>d;d.reserve(au.size());for(size_t i=0;i<au.size();++i){cv::Vec2d g0,g1;if(footprint(au[i],R0,h0,&g0)&&footprint(bu[i],R1,h1,&g1))d.push_back(g0-g1);}return robustDelta(d,out,used,scatter);}
}

int main(int argc,char**argv){
 if(argc<7){std::cerr<<"Использование: "<<argv[0]<<" <camera> <luna> <fc> <csv> <camera_yaml> <camera_offset_mm>\n";return 2;}
 const std::string camdev=argv[1],lunadev=argv[2],fcdev=argv[3],csvpath=argv[4],yaml=argv[5];const double offset=std::stod(argv[6])/1000.0;const cv::Vec3d lever_b(0.04916,0.00022,0.0);
 try{CameraCalib calib=loadCameraCalib(yaml);Camera cam;cam.openDev(camdev);LunaReader luna;luna.start(lunadev);FcHistory fc;fc.start(fcdev);GroundMotionMavlinkPublisher mavpub;uint64_t mav_sent=0,mav_skipped=0;std::ofstream csv(csvpath,std::ios::trunc);csv<<"mono_ns,frame,valid,quality,dx_m,dy_m,vx_mps,vy_mps,x_m,y_m,height_m,luna_m,inliers,scatter_m,att_age_ms,luna_age_ms,roll,pitch,yaw,mav_sent\n";cv::setNumThreads(1);std::signal(SIGINT,onSignal);std::signal(SIGTERM,onSignal);
 cv::Mat prev;Attitude prev_att{};double prev_h=0;int64_t prev_ts=0;double x=0,y=0;uint64_t frame=0;
 while(g_running){pollfd p{cam.fd,POLLIN,0};int pr=poll(&p,1,20);if(pr<0){if(errno==EINTR)continue;fail("camera poll");}if(pr<=0)continue;while(g_running){v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;if(xioctl(cam.fd,VIDIOC_DQBUF,&b)<0){if(errno==EAGAIN)break;fail("VIDIOC_DQBUF");}int64_t now=monoNs(),ts=frameTvToNsMvp(b.timestamp);cv::Mat raw(1,(int)b.bytesused,CV_8UC1,cam.bufs[b.index].p);cv::Mat gray=cv::imdecode(raw,cv::IMREAD_GRAYSCALE);if(xioctl(cam.fd,VIDIOC_QBUF,&b)<0)fail("VIDIOC_QBUF");if(gray.empty())continue;++frame;
 double lm=0;int strength=0;int64_t lns=0;Attitude att{};double att_age=1e9;bool hl=luna.latest(&lm,&strength,&lns),ha=fc.at(ts,&att,&att_age);double lage=hl?(now-lns)*1e-6:1e9,h=0,down=0;if(hl&&ha){auto R=attitudeFluToNwu(att);down=-(R*cv::Vec3d(0,0,-1))[2];h=lm*down-offset+(R*lever_b)[2];}bool sensors=hl&&ha&&down>0.20&&h>0.05&&lage<200&&att_age<30;
 bool valid=false;double dx=0,dy=0,vx=0,vy=0,scatter=0,quality=0;int inliers=0;double dt=prev_ts?(ts-prev_ts)*1e-9:0;
 if(sensors&&!prev.empty()&&prev_att.valid&&prev_h>0&&dt>0&&dt<0.2){std::vector<cv::Point2f>p0,p1;cv::goodFeaturesToTrack(prev,p0,700,0.01,7);if(p0.size()>=30){std::vector<uchar>st;std::vector<float>err;cv::calcOpticalFlowPyrLK(prev,gray,p0,p1,st,err,{21,21},3);std::vector<cv::Point2f>a,bp;for(size_t i=0;i<p0.size();++i)if(st[i]){a.push_back(p0[i]);bp.push_back(p1[i]);}if(a.size()>=20){cv::Mat mask;cv::findHomography(a,bp,cv::RANSAC,2.0,mask);if(!mask.empty()){std::vector<cv::Point2f>ai,bi;for(size_t i=0;i<a.size();++i)if(mask.at<uchar>((int)i)){ai.push_back(a[i]);bi.push_back(bp[i]);}cv::Vec2d d;if(ai.size()>=15&&v3StepMvp(ai,bi,calib,prev_att,prev_h,att,h,&d,&inliers,&scatter)&&cv::norm(d)<0.20){dx=d[0];dy=d[1];vx=dx/dt;vy=dy/dt;quality=std::clamp((inliers/150.0)*std::exp(-scatter/0.003),0.0,1.0);valid=inliers>=20&&scatter<0.004&&quality>=0.15;if(valid){x+=dx;y+=dy;}}}}}}
 bool sent=false;if(valid){sent=mavpub.send(fc.fd,(uint64_t)(now/1000),true,quality,x,y,vx,vy);if(sent)++mav_sent;else ++mav_skipped;}
 csv<<now<<','<<frame<<','<<(valid?1:0)<<','<<quality<<','<<dx<<','<<dy<<','<<vx<<','<<vy<<','<<x<<','<<y<<','<<h<<','<<lm<<','<<inliers<<','<<scatter<<','<<att_age<<','<<lage<<','<<att.roll<<','<<att.pitch<<','<<att.yaw<<','<<(sent?1:0)<<'\n';
 if(frame%100==0)std::cerr<<"GM frame="<<frame<<" valid="<<(valid?1:0)<<" ODOMETRY sent="<<mav_sent<<" skipped="<<mav_skipped<<"\r"<<std::flush;
 if(sensors){prev=gray.clone();prev_att=att;prev_h=h;prev_ts=ts;}else{prev.release();prev_att.valid=false;prev_h=0;prev_ts=0;}
 }}g_running=false;fc.stop();luna.stop();std::cerr<<"\nОстановлено. CSV: "<<csvpath<<" ODOMETRY sent="<<mav_sent<<" skipped="<<mav_skipped<<"\n";return 0;}catch(const std::exception&e){g_running=false;std::cerr<<"ОШИБКА: "<<e.what()<<"\n";return 1;}
}
