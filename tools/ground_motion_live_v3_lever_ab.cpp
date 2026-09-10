// JT-Zero Ground Motion V3 CURRENT vs LEVER strict A/B.
// Один поток кадров/KLT/RANSAC/Luna/SYNC attitude; отличается только высота камеры.
// CURRENT: Hcam = Hluna.
// LEVER:   Hcam = Hluna + (R_W_B * t_LC).z.
// t_LC body FLU default = [+49.16,+0.22,0.00] mm; Z намеренно не угадываем.
// Override: JTZERO_LEVER_X_MM / Y / Z.
//
// ВАЖНО ПО СТРУКТУРЕ: включаем только базовый V2/V3 translation unit с переименованным main.
// sync_ab.cpp НЕ включается: у него собственный main. Нужные FcHistoryReader/v3Step находятся здесь.
#define main jtzero_v2_v3_base_unused_main
#include "ground_motion_live_v2_v3_ab.cpp"
#undef main
#include <cstdlib>
#include <deque>

namespace {

static int64_t frameTvToNs(const timeval& tv){
  return (int64_t)tv.tv_sec*1000000000LL + (int64_t)tv.tv_usec*1000LL;
}
static double wrapRadLever(double a){
  while(a>kPi)a-=2*kPi;
  while(a<-kPi)a+=2*kPi;
  return a;
}

struct LeverFcHistoryReader {
  int fd=-1; std::thread th; std::mutex mu; std::deque<Attitude> hist;
  ~LeverFcHistoryReader(){stop();}
  static void writeAll(int fd,const uint8_t*p,size_t n){
    size_t o=0; while(o<n){ssize_t k=write(fd,p+o,n-o);if(k>0){o+=k;continue;}if(k<0&&(errno==EAGAIN||errno==EWOULDBLOCK)){pollfd q{fd,POLLOUT,0};poll(&q,1,10);continue;}if(k<0&&errno==EINTR)continue;fail("FC write");}
  }
  static void requestRate(int fd,uint8_t sys,uint8_t comp,uint32_t msgid,int hz){
    mavlink_message_t m{};mavlink_msg_command_long_pack(255,190,&m,sys,comp,MAV_CMD_SET_MESSAGE_INTERVAL,0,msgid,1000000.0f/hz,0,0,0,0,0);uint8_t b[MAVLINK_MAX_PACKET_LEN];auto n=mavlink_msg_to_send_buffer(b,&m);writeAll(fd,b,n);
  }
  void start(const std::string&dev){
    fd=::open(dev.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK);if(fd<0)fail("open FC");
    termios t{};if(tcgetattr(fd,&t)<0)fail("FC tcgetattr");cfmakeraw(&t);cfsetispeed(&t,B460800);cfsetospeed(&t,B460800);t.c_cflag|=CLOCAL|CREAD;t.c_cflag&=~CRTSCTS;t.c_cflag&=~PARENB;t.c_cflag&=~CSTOPB;t.c_cflag&=~CSIZE;t.c_cflag|=CS8;if(tcsetattr(fd,TCSANOW,&t)<0)fail("FC tcsetattr");tcflush(fd,TCIFLUSH);
    th=std::thread([this]{
      mavlink_status_t st{};mavlink_message_t m{};uint8_t sys=0,comp=0,buf[4096];int64_t deadline=monoNs()+10000000000LL;
      while(g_running&&!sys&&monoNs()<deadline){pollfd p{fd,POLLIN,0};if(poll(&p,1,100)<=0)continue;ssize_t n=read(fd,buf,sizeof(buf));if(n<=0)continue;for(ssize_t i=0;i<n;i++)if(mavlink_parse_char(MAVLINK_COMM_0,buf[i],&m,&st)&&m.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=m.sysid;comp=m.compid;break;}}
      if(!sys){std::cerr<<"FC: HEARTBEAT timeout\n";g_running=false;return;}
      requestRate(fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,50);
      while(g_running){pollfd p{fd,POLLIN,0};if(poll(&p,1,50)<=0)continue;for(;;){ssize_t n=read(fd,buf,sizeof(buf));if(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;for(ssize_t i=0;i<n;i++)if(mavlink_parse_char(MAVLINK_COMM_0,buf[i],&m,&st)&&m.msgid==MAVLINK_MSG_ID_ATTITUDE){mavlink_attitude_t a{};mavlink_msg_attitude_decode(&m,&a);Attitude x{a.roll,a.pitch,a.yaw,monoNs(),true};std::lock_guard<std::mutex>l(mu);hist.push_back(x);const int64_t keep=x.recv_ns-3000000000LL;while(hist.size()>2&&hist.front().recv_ns<keep)hist.pop_front();}}}
    });
  }
  bool sampleAt(int64_t target,Attitude*out,double*bracket_ms,double*nearest_ms){
    std::lock_guard<std::mutex>l(mu);if(hist.size()<2||target<hist.front().recv_ns||target>hist.back().recv_ns)return false;size_t hi=1;while(hi<hist.size()&&hist[hi].recv_ns<target)++hi;if(hi>=hist.size())return false;const Attitude&a=hist[hi-1];const Attitude&b=hist[hi];const int64_t den=b.recv_ns-a.recv_ns;if(den<=0)return false;double u=std::clamp((double)(target-a.recv_ns)/(double)den,0.0,1.0);out->roll=a.roll+u*(b.roll-a.roll);out->pitch=a.pitch+u*(b.pitch-a.pitch);out->yaw=wrapRadLever(a.yaw+u*wrapRadLever(b.yaw-a.yaw));out->recv_ns=target;out->valid=true;if(bracket_ms)*bracket_ms=den*1e-6;if(nearest_ms)*nearest_ms=std::min(std::llabs(target-a.recv_ns),std::llabs(b.recv_ns-target))*1e-6;return true;
  }
  void stop(){if(th.joinable())th.join();if(fd>=0){::close(fd);fd=-1;}}
};

static bool leverV3Step(const std::vector<cv::Point2f>&ai,const std::vector<cv::Point2f>&bi,const CameraCalib&calib,const Attitude&a0,double h0,const Attitude&a1,double h1,cv::Vec2d*out,int*used,double*scatter){
  std::vector<cv::Point2f>au,bu;cv::undistortPoints(ai,au,calib.K,calib.D);cv::undistortPoints(bi,bu,calib.K,calib.D);cv::Matx33d W_R_C0=attitudeFluToNwu(a0)*calib.B_R_C;cv::Matx33d W_R_C1=attitudeFluToNwu(a1)*calib.B_R_C;std::vector<cv::Vec2d>deltas;deltas.reserve(au.size());for(size_t i=0;i<au.size();++i){cv::Vec2d g0,g1;if(footprint(au[i],W_R_C0,h0,&g0)&&footprint(bu[i],W_R_C1,h1,&g1))deltas.push_back(g0-g1);}return robustDelta(deltas,out,used,scatter);
}

double envMm(const char*name,double def_mm){const char*s=std::getenv(name);if(!s||!*s)return def_mm;try{return std::stod(s);}catch(...){return def_mm;}}

}

int main(int argc,char**argv){
  if(argc<7){std::cerr<<"Использование: "<<argv[0]<<" <camera> <luna> <fc> <csv> <camera_yaml> <camera_offset_mm>\n";return 2;}
  const std::string camdev=argv[1],lunadev=argv[2],fcdev=argv[3],csvpath=argv[4],yaml=argv[5];const double offset_m=std::stod(argv[6])/1000.0;
  const cv::Vec3d t_lc_b(envMm("JTZERO_LEVER_X_MM",49.16)/1000.0,envMm("JTZERO_LEVER_Y_MM",0.22)/1000.0,envMm("JTZERO_LEVER_Z_MM",0.0)/1000.0);
  try{
    CameraCalib calib=loadCameraCalib(yaml);Camera cam;cam.openDev(camdev);LunaReader luna;luna.start(lunadev);LeverFcHistoryReader fc;fc.start(fcdev);std::ofstream csv(csvpath,std::ios::trunc);
    csv<<"state,now_ns,frame_ts_ns,frame,camera_age_ms,att_sync_nearest_ms,att_sync_bracket_ms,luna_age_ms,luna_slant_m,h_luna_m,lever_dh_m,h_camera_m,current_x_m,current_y_m,current_path_m,lever_x_m,lever_y_m,lever_path_m,current_inliers,lever_inliers,current_scatter_m,lever_scatter_m,roll,pitch,yaw\n";
    cv::setNumThreads(1);std::signal(SIGINT,onSignal);std::signal(SIGTERM,onSignal);const char*window="JT-ZERO — V3 A/B: CURRENT vs CAD LEVER";cv::namedWindow(window,cv::WINDOW_NORMAL);cv::resizeWindow(window,1280,720);cv::moveWindow(window,0,0);
    enum class State{READY=0,MOVING=1,DONE=2};State state=State::READY;cv::Mat prev;Attitude prev_att{};double prev_h_current=0,prev_h_lever=0;int64_t prev_frame_ts=0;Estimate current_est,lever_est;uint64_t frame_id=0;double rcx=0,rcy=0,rlx=0,rly=0;
    while(g_running){
      pollfd p{cam.fd,POLLIN,0};int pr=poll(&p,1,20);if(pr<0){if(errno==EINTR)continue;fail("camera poll");}if(pr<=0)continue;
      while(g_running){
        v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;if(xioctl(cam.fd,VIDIOC_DQBUF,&b)<0){if(errno==EAGAIN)break;fail("VIDIOC_DQBUF");}
        const int64_t now=monoNs(),frame_ts=frameTvToNs(b.timestamp);const double camera_age_ms=(now-frame_ts)*1e-6;cv::Mat raw(1,(int)b.bytesused,CV_8UC1,cam.bufs[b.index].p);cv::Mat gray=cv::imdecode(raw,cv::IMREAD_GRAYSCALE);if(xioctl(cam.fd,VIDIOC_QBUF,&b)<0)fail("VIDIOC_QBUF");if(gray.empty())continue;++frame_id;
        double luna_m=0;int strength=0;int64_t luna_ns=0;Attitude att{};const bool have_luna=luna.latest(&luna_m,&strength,&luna_ns);double sync_bracket_ms=0,sync_nearest_ms=0;const bool have_att=fc.sampleAt(frame_ts,&att,&sync_bracket_ms,&sync_nearest_ms);const double luna_age_ms=have_luna?(now-luna_ns)*1e-6:1e9;
        double h_current=0,h_lever=0,lever_dh=0,down=0;if(have_luna&&have_att){const cv::Matx33d W_R_B=attitudeFluToNwu(att);const cv::Vec3d beam_w=W_R_B*cv::Vec3d(0,0,-1);down=-beam_w[2];h_current=luna_m*down-offset_m;lever_dh=(W_R_B*t_lc_b)[2];h_lever=h_current+lever_dh;}
        const bool sensors_ok=have_luna&&have_att&&down>0.20&&h_current>0.05&&h_lever>0.05&&luna_age_ms<200;
        if(state==State::MOVING&&sensors_ok&&!prev.empty()&&prev_att.valid&&prev_h_current>0&&prev_h_lever>0){std::vector<cv::Point2f>p0,p1;cv::goodFeaturesToTrack(prev,p0,700,0.01,7);if(p0.size()>=30){std::vector<uchar>st;std::vector<float>err;cv::calcOpticalFlowPyrLK(prev,gray,p0,p1,st,err,{21,21},3);std::vector<cv::Point2f>a,bp;for(size_t i=0;i<p0.size();++i)if(st[i]){a.push_back(p0[i]);bp.push_back(p1[i]);}if(a.size()>=20){cv::Mat mask;cv::findHomography(a,bp,cv::RANSAC,2.0,mask);if(!mask.empty()){std::vector<cv::Point2f>ai,bi;for(size_t i=0;i<a.size();++i)if(mask.at<uchar>((int)i)){ai.push_back(a[i]);bi.push_back(bp[i]);}const double dt=prev_frame_ts?((frame_ts-prev_frame_ts)*1e-9):0;if(ai.size()>=15&&dt>0&&dt<0.2){cv::Vec2d dc,dl;int uc=0,ul=0;double sc=0,sl=0;if(leverV3Step(ai,bi,calib,prev_att,prev_h_current,att,h_current,&dc,&uc,&sc)&&cv::norm(dc)<0.20){current_est.x+=dc[0];current_est.y+=dc[1];current_est.path+=cv::norm(dc);current_est.inliers=uc;current_est.scatter=sc;++current_est.frames;}if(leverV3Step(ai,bi,calib,prev_att,prev_h_lever,att,h_lever,&dl,&ul,&sl)&&cv::norm(dl)<0.20){lever_est.x+=dl[0];lever_est.y+=dl[1];lever_est.path+=cv::norm(dl);lever_est.inliers=ul;lever_est.scatter=sl;++lever_est.frames;}}}}}}
        if(sensors_ok){prev=gray.clone();prev_att=att;prev_h_current=h_current;prev_h_lever=h_lever;prev_frame_ts=frame_ts;}else{prev.release();prev_att.valid=false;prev_h_current=prev_h_lever=0;prev_frame_ts=0;}
        cv::Mat bgr,video;cv::cvtColor(gray,bgr,cv::COLOR_GRAY2BGR);cv::resize(bgr,video,{870,653});cv::Mat canvas(720,1280,CV_8UC3,cv::Scalar(12,12,12));video.copyTo(canvas(cv::Rect(0,67,870,653)));ru(canvas,"JT-ZERO — V3 CURRENT vs CAD LEVER",{22,35},17,{245,245,245},cv::QT_FONT_BOLD);cv::Mat panel=canvas(cv::Rect(870,0,410,720));
        ru(panel,sensors_ok?"СИСТЕМА ГОТОВА":"ЖДИТЕ ДАННЫЕ ДАТЧИКОВ",{18,42},13,sensors_ok?cv::Scalar(90,220,90):cv::Scalar(0,210,255),cv::QT_FONT_BOLD);const char*now_text=state==State::READY?"СЕЙЧАС: ТОЧКА A — НЕ ДВИГАТЬ":state==State::MOVING?"СЕЙЧАС: ДВИЖЕНИЕ A -> B":"СЕЙЧАС: РЕЗУЛЬТАТ ЗАФИКСИРОВАН";ru(panel,now_text,{18,82},10,{255,255,255},cv::QT_FONT_BOLD);
        char z[256];snprintf(z,sizeof(z),"Pitch/Roll: %+.2f / %+.2f°",att.pitch*180/kPi,att.roll*180/kPi);ru(panel,z,{18,125},10,{220,220,220});snprintf(z,sizeof(z),"H Luna / H cam: %.1f / %.1f мм",h_current*1000,h_lever*1000);ru(panel,z,{18,160},10,{220,220,220});snprintf(z,sizeof(z),"Lever dH: %+.2f мм",lever_dh*1000);ru(panel,z,{18,195},10,{220,220,220});snprintf(z,sizeof(z),"CURRENT NET/PATH: %.1f / %.1f мм",std::hypot(current_est.x,current_est.y)*1000,current_est.path*1000);ru(panel,z,{18,260},12,{245,245,245},cv::QT_FONT_BOLD);snprintf(z,sizeof(z),"LEVER NET/PATH: %.1f / %.1f мм",std::hypot(lever_est.x,lever_est.y)*1000,lever_est.path*1000);ru(panel,z,{18,320},12,{245,245,245},cv::QT_FONT_BOLD);snprintf(z,sizeof(z),"LEVER-CURRENT NET: %+.1f мм",(std::hypot(lever_est.x,lever_est.y)-std::hypot(current_est.x,current_est.y))*1000);ru(panel,z,{18,370},11,{230,230,230},cv::QT_FONT_BOLD);snprintf(z,sizeof(z),"Точек current/lever: %d / %d",current_est.inliers,lever_est.inliers);ru(panel,z,{18,430},10,{220,220,220});snprintf(z,sizeof(z),"Разброс: %.2f / %.2f мм",current_est.scatter*1000,lever_est.scatter*1000);ru(panel,z,{18,465},10,{220,220,220});snprintf(z,sizeof(z),"CAD tLC: [%.2f %.2f %.2f] мм",t_lc_b[0]*1000,t_lc_b[1]*1000,t_lc_b[2]*1000);ru(panel,z,{18,515},9,{220,220,220});if(state==State::DONE){snprintf(z,sizeof(z),"ИТОГ CURRENT/LEVER: %.1f / %.1f мм",std::hypot(rcx,rcy)*1000,std::hypot(rlx,rly)*1000);ru(panel,z,{18,610},11,{245,245,245},cv::QT_FONT_BOLD);}ru(panel,state==State::READY?"ПРОБЕЛ — СТАРТ":state==State::MOVING?"В B: ПРОБЕЛ — СТОП":"Q / ESC — ВЫХОД",{18,675},11,{210,210,210},cv::QT_FONT_BOLD);
        cv::imshow(window,canvas);int rk=cv::waitKeyEx(10),key=rk<0?-1:(rk&0xff);if(cv::getWindowProperty(window,cv::WND_PROP_VISIBLE)<1){g_running=false;break;}if(key==' '&&sensors_ok){if(state==State::READY){current_est={};lever_est={};prev.release();prev_att.valid=false;prev_h_current=prev_h_lever=0;prev_frame_ts=0;state=State::MOVING;}else if(state==State::MOVING){rcx=current_est.x;rcy=current_est.y;rlx=lever_est.x;rly=lever_est.y;state=State::DONE;}}if(key=='q'||key=='Q'||key==27){g_running=false;break;}
        if(csv){csv<<(int)state<<','<<now<<','<<frame_ts<<','<<frame_id<<','<<std::fixed<<std::setprecision(6)<<camera_age_ms<<','<<sync_nearest_ms<<','<<sync_bracket_ms<<','<<luna_age_ms<<','<<luna_m<<','<<h_current<<','<<lever_dh<<','<<h_lever<<','<<current_est.x<<','<<current_est.y<<','<<current_est.path<<','<<lever_est.x<<','<<lever_est.y<<','<<lever_est.path<<','<<current_est.inliers<<','<<lever_est.inliers<<','<<current_est.scatter<<','<<lever_est.scatter<<','<<att.roll<<','<<att.pitch<<','<<att.yaw<<'\n';if((frame_id&3u)==0u)csv.flush();}
      }
    }
    cv::destroyAllWindows();std::cout<<"CSV: "<<csvpath<<"\n";return 0;
  }catch(const std::exception&e){std::cerr<<"GROUND MOTION V3 LEVER AB FAIL: "<<e.what()<<"\n";return 1;}
}
