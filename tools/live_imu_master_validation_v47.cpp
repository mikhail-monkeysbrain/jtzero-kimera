// JT-ZERO v47: comprehensive IMU correction validation logger.
// One physical run validates RAW, independently validated v42 ZXY correction,
// and ZXY + causal adaptive BG_X while collecting broad orientation excitation.

#define JTZERO_V10_NO_MAIN
#include "live_mono_imu_integrator_v10.cpp"
#undef JTZERO_V10_NO_MAIN

#include <array>
#include <deque>
#include <fstream>
#include <iomanip>
#include <string>

namespace jtzero_v47 {
using namespace jtzero_v10;

constexpr const char* kCsv = "/home/vio/jtzero_imu_master_v47.csv";
constexpr const char* kWindow = "JT-ZERO: MASTER VALIDATION IMU v47";
constexpr double G = 9.81;
constexpr double CX = 0.014570;
constexpr double CY = 0.082383;
constexpr double BG_TAU = 2.0;
constexpr double FC_STATIC_RATE = 0.010;
constexpr double ACC_NORM_TOL = 0.35;
constexpr double ACC_STD_MAX = 0.16;
constexpr double STATIC_WINDOW_SEC = 0.50;

struct Fc {
  bool valid=false,have_prev=false;
  double roll=0,pitch=0,yaw=0,accum_yaw=0,prev_yaw=0;
  double rs=0,ps=0,ys=0;
};
struct Cal {
  bool ready=false; size_t n=0;
  Eigen::Vector3d sa=Eigen::Vector3d::Zero(),sg=Eigen::Vector3d::Zero();
  Eigen::Vector3d ba=Eigen::Vector3d::Zero(),bg=Eigen::Vector3d::Zero();
  Eigen::Matrix3d R0=Eigen::Matrix3d::Identity();
};
struct AccWin { uint64_t us=0; double n=0; };
struct Branch { Eigen::Matrix3d R=Eigen::Matrix3d::Identity(); Eigen::Vector3d v=Eigen::Vector3d::Zero(),aw=Eigen::Vector3d::Zero(); };
struct State {
  bool ready=false; uint64_t prev_us=0;
  Branch raw,zxy,adapt;
  double bx=0, static_conf=0;
  std::deque<AccWin> awin;
  double asum=0,asum2=0;
  double max_raw=0,max_zxy=0,max_adapt=0;
};

enum class Stage:int {
 CALIB=0, STATIC_A,
 YAW_SLOW_R,YAW_RETURN_1,YAW_FAST_L,YAW_RETURN_2,
 ROLL_POS,ROLL_ZERO_1,ROLL_NEG,ROLL_ZERO_2,
 PITCH_POS,PITCH_ZERO_1,PITCH_NEG,PITCH_ZERO_2,
 RP_PP,RP_ZERO_1,RP_NP,RP_ZERO_2,RP_PN,RP_ZERO_3,RP_NN,RP_ZERO_4,
 YAW90_STATIC,YAW90_ROLL,YAW90_ZERO,
 REPEAT_ROLL_POS,REPEAT_ZERO_1,REPEAT_PITCH_NEG,REPEAT_ZERO_2,
 FINAL_STATIC,DONE
};
const char* name(Stage s){
 static const char* n[]={"CALIB","STATIC_A","YAW_SLOW_R","YAW_RETURN_1","YAW_FAST_L","YAW_RETURN_2","ROLL_POS","ROLL_ZERO_1","ROLL_NEG","ROLL_ZERO_2","PITCH_POS","PITCH_ZERO_1","PITCH_NEG","PITCH_ZERO_2","RP_PP","RP_ZERO_1","RP_NP","RP_ZERO_2","RP_PN","RP_ZERO_3","RP_NN","RP_ZERO_4","YAW90_STATIC","YAW90_ROLL","YAW90_ZERO","REPEAT_ROLL_POS","REPEAT_ZERO_1","REPEAT_PITCH_NEG","REPEAT_ZERO_2","FINAL_STATIC","DONE"};
 return n[(int)s];
}
const char* ru(Stage s){
 switch(s){
 case Stage::CALIB:return "НЕ ДВИГАТЬ. НАЧАЛЬНАЯ КАЛИБРОВКА";
 case Stage::STATIC_A:return "ИСХОДНОЕ ПОЛОЖЕНИЕ. НЕ ДВИГАТЬ";
 case Stage::YAW_SLOW_R:return "МЕДЛЕННО YAW ВПРАВО 60–100°, ОСТАНОВИТЬСЯ";
 case Stage::YAW_RETURN_1:return "МЕДЛЕННО ВЕРНУТЬ YAW В ИСХОДНОЕ";
 case Stage::YAW_FAST_L:return "БЫСТРО, ПЛАВНО YAW ВЛЕВО 60–100°";
 case Stage::YAW_RETURN_2:return "БЫСТРО, ПЛАВНО ВЕРНУТЬ YAW";
 case Stage::ROLL_POS:return "ROLL +20–35°, ОСТАНОВИТЬСЯ";
 case Stage::ROLL_ZERO_1:return "ВЕРНУТЬ ROLL В ИСХОДНОЕ";
 case Stage::ROLL_NEG:return "ROLL -20–35°, ОСТАНОВИТЬСЯ";
 case Stage::ROLL_ZERO_2:return "ВЕРНУТЬ ROLL В ИСХОДНОЕ";
 case Stage::PITCH_POS:return "PITCH +20–35°, ОСТАНОВИТЬСЯ";
 case Stage::PITCH_ZERO_1:return "ВЕРНУТЬ PITCH В ИСХОДНОЕ";
 case Stage::PITCH_NEG:return "PITCH -20–35°, ОСТАНОВИТЬСЯ";
 case Stage::PITCH_ZERO_2:return "ВЕРНУТЬ PITCH В ИСХОДНОЕ";
 case Stage::RP_PP:return "ROLL +20° И PITCH +20°, ОСТАНОВИТЬСЯ";
 case Stage::RP_ZERO_1:return "ВЕРНУТЬ ОБА НАКЛОНА В ИСХОДНОЕ";
 case Stage::RP_NP:return "ROLL -20° И PITCH +20°, ОСТАНОВИТЬСЯ";
 case Stage::RP_ZERO_2:return "ВЕРНУТЬ ОБА НАКЛОНА В ИСХОДНОЕ";
 case Stage::RP_PN:return "ROLL +20° И PITCH -20°, ОСТАНОВИТЬСЯ";
 case Stage::RP_ZERO_3:return "ВЕРНУТЬ ОБА НАКЛОНА В ИСХОДНОЕ";
 case Stage::RP_NN:return "ROLL -20° И PITCH -20°, ОСТАНОВИТЬСЯ";
 case Stage::RP_ZERO_4:return "ВЕРНУТЬ ОБА НАКЛОНА В ИСХОДНОЕ";
 case Stage::YAW90_STATIC:return "YAW ПРИМЕРНО 90°, БЕЗ НАКЛОНА";
 case Stage::YAW90_ROLL:return "ПРИ YAW~90° ДОБАВИТЬ ROLL +20–30°";
 case Stage::YAW90_ZERO:return "ВЕРНУТЬ YAW И НАКЛОН В ИСХОДНОЕ";
 case Stage::REPEAT_ROLL_POS:return "ПОВТОРИТЬ ROLL +20–35°";
 case Stage::REPEAT_ZERO_1:return "ВЕРНУТЬСЯ В ИСХОДНОЕ";
 case Stage::REPEAT_PITCH_NEG:return "ПОВТОРИТЬ PITCH -20–35°";
 case Stage::REPEAT_ZERO_2:return "ВЕРНУТЬСЯ В ИСХОДНОЕ";
 case Stage::FINAL_STATIC:return "ФИНАЛЬНАЯ СТАТИКА. НЕ ДВИГАТЬ";
 case Stage::DONE:return "ТЕСТ ЗАВЕРШЁН";
 }
 return "";
}
Stage next(Stage s){int x=(int)s;return x>=(int)Stage::DONE?Stage::DONE:(Stage)(x+1);}
Eigen::Matrix3d expSO3(const Eigen::Vector3d& q){double a=q.norm();return a<1e-12?Eigen::Matrix3d::Identity():Eigen::AngleAxisd(a,q/a).toRotationMatrix();}
Eigen::Vector3d rpy(const Eigen::Matrix3d& R){return R.eulerAngles(0,1,2)*180.0/kPi;}
void reticle(cv::Mat& im){int x=im.cols/2,y=im.rows/2;cv::Scalar c(0,255,255);cv::line(im,{x-45,y},{x-8,y},c,2);cv::line(im,{x+8,y},{x+45,y},c,2);cv::line(im,{x,y-45},{x,y-8},c,2);cv::line(im,{x,y+8},{x,y+45},c,2);cv::circle(im,{x,y},18,c,2);}
void hud(const cv::Mat& gray,Stage st,const Fc& fc,const Cal& cal,const State& q,const Eigen::Vector3d& acc,const Eigen::Vector3d& gyr,bool guarded,double astd){
 cv::Mat c(940,1440,CV_8UC3,cv::Scalar(15,15,15)),v;if(gray.empty())v=cv::Mat(kHeight,kWidth,CV_8UC3,cv::Scalar(0));else cv::cvtColor(gray,v,cv::COLOR_GRAY2BGR);cv::resize(v,v,{820,615});reticle(v);v.copyTo(c(cv::Rect(20,80,820,615)));
 cv::Scalar w(235,235,235),g(80,220,80),r(40,40,245),y(0,220,255),m(155,155,155);uiText(c,"JT-ZERO: MASTER VALIDATION IMU v47",20,48,.68,w,2);uiText(c,"ТОЧНЫЕ УГЛЫ НЕ НУЖНЫ. ПОСЛЕ ДВИЖЕНИЯ ОСТАНОВИТЕСЬ И SPACE",45,735,.48,y,2);
 cv::Mat s=c(cv::Rect(860,70,560,790));cv::rectangle(s,{0,0},{559,789},cv::Scalar(45,45,45),1);uiText(s,"ТЕКУЩИЙ ЭТАП",18,38,.45,m,1);uiText(s,ru(st),18,76,.40,w,2);char b[256];
 snprintf(b,sizeof(b),"ROLL %.1f°  PITCH %.1f°",fc.roll,fc.pitch);uiText(s,b,18,135,.43,w,1);snprintf(b,sizeof(b),"YAW накопл. %.1f°",fc.accum_yaw);uiText(s,b,18,170,.43,w,1);snprintf(b,sizeof(b),"FC |rate| %.4f",std::sqrt(fc.rs*fc.rs+fc.ps*fc.ps+fc.ys*fc.ys));uiText(s,b,18,205,.43,w,1);
 snprintf(b,sizeof(b),"|acc| %.4f   std %.4f",acc.norm(),astd);uiText(s,b,18,250,.43,w,1);snprintf(b,sizeof(b),"STATIC GUARD: %s",guarded?"ДА":"НЕТ");uiText(s,b,18,285,.46,guarded?g:r,2);snprintf(b,sizeof(b),"adaptive BG_X %.7f",q.bx);uiText(s,b,18,325,.43,w,1);
 snprintf(b,sizeof(b),"RAW Vxy %.3f",q.raw.v.head<2>().norm());uiText(s,b,18,390,.47,r,2);snprintf(b,sizeof(b),"ZXY Vxy %.3f",q.zxy.v.head<2>().norm());uiText(s,b,18,430,.47,w,2);snprintf(b,sizeof(b),"ZXY+BGX Vxy %.3f",q.adapt.v.head<2>().norm());uiText(s,b,18,470,.47,g,2);
 snprintf(b,sizeof(b),"RAW axy %.3f",q.raw.aw.head<2>().norm());uiText(s,b,18,525,.42,w,1);snprintf(b,sizeof(b),"ZXY axy %.3f",q.zxy.aw.head<2>().norm());uiText(s,b,18,560,.42,w,1);snprintf(b,sizeof(b),"ADAPT axy %.3f",q.adapt.aw.head<2>().norm());uiText(s,b,18,595,.42,w,1);
 snprintf(b,sizeof(b),"Калибровка samples: %zu",cal.n);uiText(s,b,18,640,.40,cal.ready?g:y,1);uiText(s,"SPACE — следующий этап",18,695,.40,y,1);uiText(s,"ESC / Q — сохранить и выйти",18,730,.37,m,1);uiText(c,std::string("CSV: ")+kCsv,25,910,.39,m,1);cv::imshow(kWindow,c);
}
}

int main(int argc,char** argv){
 using namespace jtzero_v10;using namespace jtzero_v47;google::InitGoogleLogging(argv[0]);FLAGS_visualize=false;FLAGS_use_lcd=false;FLAGS_log_output=false;FLAGS_extract_planes_from_the_scene=false;
 int cfd=-1,sfd=-1;bool streaming=false,imu_req=false,att_req=false,aborted=false;uint8_t sys=0,comp=0;std::vector<CameraBuffer> bufs;std::ofstream csv;
 try{
  sfd=openSerial();mavlink_status_t mst{};mavlink_message_t msg{};std::cout<<"[MAV] waiting HEARTBEAT...\n";int64_t dl=monotonicNs()+10000000000LL;while(monotonicNs()<dl&&!sys){pollfd p{sfd,POLLIN,0};if(poll(&p,1,100)<=0)continue;uint8_t b[2048];ssize_t n=read(sfd,b,sizeof(b));if(n<=0)continue;for(ssize_t i=0;i<n;++i)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst)&&msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=msg.sysid;comp=msg.compid;break;}}if(!sys)throw std::runtime_error("HEARTBEAT timeout");
  requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,kImuRateHz);imu_req=true;requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,kAttRateHz);att_req=true;
  cfd=open(kCameraDevice,O_RDWR|O_NONBLOCK);if(cfd==-1)fail("open camera");configureCamera(cfd);bufs=initCameraBuffers(cfd);v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;if(xioctl(cfd,VIDIOC_STREAMON,&type)==-1)fail("STREAMON");streaming=true;discardWarmup(cfd);
  csv.open(kCsv,std::ios::trunc);if(!csv)throw std::runtime_error("cannot open CSV");csv<<"stage_id,stage,imu_us,rx_mono_ns,dt,fields_updated,temp_c,frd_ax,frd_ay,frd_az,frd_gx,frd_gy,frd_gz,flu_ax,flu_ay,flu_az,raw_gx,raw_gy,raw_gz,zxy_gx,zxy_gy,zxy_gz,adapt_gx,adapt_gy,adapt_gz,adaptive_bgx,static_guard,acc_window_std,fc_roll,fc_pitch,fc_yaw,fc_accum_yaw,fc_rollspeed,fc_pitchspeed,fc_yawspeed,raw_rr,raw_rp,raw_ry,zxy_rr,zxy_rp,zxy_ry,adapt_rr,adapt_rp,adapt_ry,raw_awx,raw_awy,raw_awz,zxy_awx,zxy_awy,zxy_awz,adapt_awx,adapt_awy,adapt_awz,raw_vx,raw_vy,raw_vz,zxy_vx,zxy_vy,zxy_vz,adapt_vx,adapt_vy,adapt_vz\n"<<std::fixed<<std::setprecision(9);
  cv::setNumThreads(1);cv::namedWindow(kWindow,cv::WINDOW_NORMAL);cv::resizeWindow(kWindow,1440,940);cv::Mat gray(kHeight,kWidth,CV_8UC1,cv::Scalar(0));Fc fc;Cal cal;State q;Stage st=Stage::CALIB;Eigen::Vector3d lasta=Eigen::Vector3d::Zero(),lastg=Eigen::Vector3d::Zero();double laststd=999;bool lastguard=false;int64_t next_hud=monotonicNs();
  std::cout<<"JT-ZERO v47. Один master-прогон. После каждого достигнутого положения остановитесь и нажмите SPACE.\n";
  while(true){pollfd pf[2]={{cfd,POLLIN,0},{sfd,POLLIN,0}};int rc=poll(pf,2,2);if(rc<0){if(errno==EINTR)continue;fail("poll");}
   if(pf[1].revents&POLLIN){uint8_t b[8192];for(;;){ssize_t n=read(sfd,b,sizeof(b));if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;for(ssize_t i=0;i<n;++i){if(!mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst))continue;int64_t rx=monotonicNs();if(msg.msgid==MAVLINK_MSG_ID_ATTITUDE){mavlink_attitude_t a{};mavlink_msg_attitude_decode(&msg,&a);double yd=a.yaw*180.0/kPi;if(fc.have_prev)fc.accum_yaw+=wrapDeg(yd-fc.prev_yaw);fc.prev_yaw=yd;fc.have_prev=true;fc.valid=true;fc.roll=a.roll*180.0/kPi;fc.pitch=a.pitch*180.0/kPi;fc.yaw=yd;fc.rs=a.rollspeed;fc.ps=a.pitchspeed;fc.ys=a.yawspeed;}
    else if(msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){mavlink_highres_imu_t h{};mavlink_msg_highres_imu_decode(&msg,&h);Eigen::Vector3d af(h.xacc,h.yacc,h.zacc),gf(h.xgyro,h.ygyro,h.zgyro),acc(h.xacc,-h.yacc,-h.zacc),gyr(h.xgyro,-h.ygyro,-h.zgyro);lasta=acc;lastg=gyr;double dt=0;if(q.prev_us&&h.time_usec>q.prev_us)dt=(h.time_usec-q.prev_us)*1e-6;q.prev_us=h.time_usec;if(dt<0||dt>0.03)dt=0;
     q.awin.push_back({h.time_usec,acc.norm()});q.asum+=acc.norm();q.asum2+=acc.squaredNorm();while(!q.awin.empty()&&(h.time_usec-q.awin.front().us)>uint64_t(STATIC_WINDOW_SEC*1e6)){q.asum-=q.awin.front().n;q.asum2-=q.awin.front().n*q.awin.front().n;q.awin.pop_front();}double anmean=q.awin.empty()?0:q.asum/q.awin.size();double avar=q.awin.empty()?999:std::max(0.0,q.asum2/q.awin.size()-anmean*anmean);laststd=std::sqrt(avar);double fcr=std::sqrt(fc.rs*fc.rs+fc.ps*fc.ps+fc.ys*fc.ys);lastguard=fc.valid&&q.awin.size()>50&&fcr<FC_STATIC_RATE&&laststd<ACC_STD_MAX&&std::abs(anmean-G)<ACC_NORM_TOL;
     bool initial_still=gyr.norm()<0.08&&std::abs(acc.norm()-G)<ACC_NORM_TOL;if(st==Stage::CALIB&&!cal.ready&&initial_still){cal.sa+=acc;cal.sg+=gyr;++cal.n;}
     Eigen::Vector3d wr=gyr-cal.bg,wz=wr;wz.x()+=CX*wr.z();wz.y()+=CY*wr.z();if(q.ready&&dt>0&&lastguard){double a=1.0-std::exp(-dt/BG_TAU);q.bx+=a*(wz.x()-q.bx);}Eigen::Vector3d wa=wz;wa.x()-=q.bx;
     if(q.ready&&dt>0){auto step=[&](Branch& z,const Eigen::Vector3d& w){z.R=z.R*expSO3(w*dt);z.aw=z.R*(acc-cal.ba)+Eigen::Vector3d(0,0,G);z.v+=z.aw*dt;};step(q.raw,wr);step(q.zxy,wz);step(q.adapt,wa);q.max_raw=std::max(q.max_raw,q.raw.v.head<2>().norm());q.max_zxy=std::max(q.max_zxy,q.zxy.v.head<2>().norm());q.max_adapt=std::max(q.max_adapt,q.adapt.v.head<2>().norm());}
     auto rr=rpy(q.raw.R),rz=rpy(q.zxy.R),ra=rpy(q.adapt.R);csv<<(int)st<<','<<name(st)<<','<<h.time_usec<<','<<rx<<','<<dt<<','<<h.fields_updated<<','<<h.temperature<<','<<af.x()<<','<<af.y()<<','<<af.z()<<','<<gf.x()<<','<<gf.y()<<','<<gf.z()<<','<<acc.x()<<','<<acc.y()<<','<<acc.z()<<','<<wr.x()<<','<<wr.y()<<','<<wr.z()<<','<<wz.x()<<','<<wz.y()<<','<<wz.z()<<','<<wa.x()<<','<<wa.y()<<','<<wa.z()<<','<<q.bx<<','<<(lastguard?1:0)<<','<<laststd<<','<<fc.roll<<','<<fc.pitch<<','<<fc.yaw<<','<<fc.accum_yaw<<','<<fc.rs<<','<<fc.ps<<','<<fc.ys<<','<<rr.x()<<','<<rr.y()<<','<<rr.z()<<','<<rz.x()<<','<<rz.y()<<','<<rz.z()<<','<<ra.x()<<','<<ra.y()<<','<<ra.z()<<','<<q.raw.aw.x()<<','<<q.raw.aw.y()<<','<<q.raw.aw.z()<<','<<q.zxy.aw.x()<<','<<q.zxy.aw.y()<<','<<q.zxy.aw.z()<<','<<q.adapt.aw.x()<<','<<q.adapt.aw.y()<<','<<q.adapt.aw.z()<<','<<q.raw.v.x()<<','<<q.raw.v.y()<<','<<q.raw.v.z()<<','<<q.zxy.v.x()<<','<<q.zxy.v.y()<<','<<q.zxy.v.z()<<','<<q.adapt.v.x()<<','<<q.adapt.v.y()<<','<<q.adapt.v.z()<<'\n';
    }}}}
   if(pf[0].revents&POLLIN){for(;;){v4l2_buffer z{};z.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;z.memory=V4L2_MEMORY_MMAP;if(xioctl(cfd,VIDIOC_DQBUF,&z)==-1){if(errno==EAGAIN)break;fail("DQBUF");}std::vector<unsigned char> j(z.bytesused);memcpy(j.data(),bufs[z.index].start,z.bytesused);cv::Mat g=cv::imdecode(j,cv::IMREAD_GRAYSCALE);if(!g.empty())gray=g;if(xioctl(cfd,VIDIOC_QBUF,&z)==-1)fail("QBUF");}}
   int64_t now=monotonicNs();if(now>=next_hud){hud(gray,st,fc,cal,q,lasta,lastg,lastguard,laststd);next_hud=now+kHudPeriodNs;}int key=cv::waitKey(1)&0xff;if(key==27||key=='q'||key=='Q'){aborted=st!=Stage::DONE;break;}if(key==' '){if(st==Stage::CALIB){if(cal.n<100){std::cout<<"[WAIT] samples="<<cal.n<<"\n";continue;}Eigen::Vector3d ma=cal.sa/cal.n,mg=cal.sg/cal.n;cal.R0=fcRnedFlu(fc.roll,fc.pitch,fc.yaw);cal.ba=ma-cal.R0.transpose()*Eigen::Vector3d(0,0,-G);cal.bg=mg;cal.ready=true;q.ready=true;q.raw.R=q.zxy.R=q.adapt.R=cal.R0;q.raw.v.setZero();q.zxy.v.setZero();q.adapt.v.setZero();q.bx=0;st=next(st);std::cout<<"[CAL] BA="<<cal.ba.transpose()<<" BG="<<cal.bg.transpose()<<"\n";}else if(st!=Stage::DONE){std::cout<<"[STEP] "<<name(st)<<" RAW="<<q.raw.v.head<2>().norm()<<" ZXY="<<q.zxy.v.head<2>().norm()<<" ADAPT="<<q.adapt.v.head<2>().norm()<<" BGX="<<q.bx<<"\n";st=next(st);if(st==Stage::DONE)break;}else break;}}
  }
  csv.flush();csv.close();if(imu_req)requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);if(att_req)requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);if(streaming){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;ioctl(cfd,VIDIOC_STREAMOFF,&t);}for(auto& b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);cv::destroyAllWindows();std::cout<<"\nCSV: "<<kCsv<<"\nMAX Vxy RAW="<<q.max_raw<<" ZXY="<<q.max_zxy<<" ZXY+BGX="<<q.max_adapt<<"\n"<<(aborted?"ABORTED\n":"DONE\n");return aborted?2:0;
 }catch(const std::exception& e){std::cerr<<"[FATAL] "<<e.what()<<"\n";return 1;}
}
