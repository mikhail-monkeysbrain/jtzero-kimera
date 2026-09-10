// JT-Zero Ground Motion V3 timing A/B.
// Один и тот же поток кадров/KLT/RANSAC/Luna считает V3 двумя способами:
//   LATEST — ATTITUDE, последний полученный к моменту обработки кадра (старое поведение)
//   SYNC   — ATTITUDE, интерполированный к V4L2 timestamp кадра по recv_ns истории FC.
// Важно: Luna намеренно одинакова для обеих веток, чтобы изолировать влияние timing ATTITUDE.
#define main jtzero_v2_v3_unused_main
#include "ground_motion_live_v2_v3_ab.cpp"
#undef main
#include <deque>

namespace {

static int64_t tvToNs(const timeval& tv){
  return (int64_t)tv.tv_sec*1000000000LL + (int64_t)tv.tv_usec*1000LL;
}
static double wrapRad(double a){
  while(a>kPi) a-=2*kPi;
  while(a<-kPi) a+=2*kPi;
  return a;
}

struct FcHistoryReader {
  int fd=-1;
  std::thread th;
  std::mutex mu;
  std::deque<Attitude> hist;
  ~FcHistoryReader(){ stop(); }

  static void writeAll2(int fd,const uint8_t*p,size_t n){
    size_t o=0;
    while(o<n){
      ssize_t k=write(fd,p+o,n-o);
      if(k>0){o+=k;continue;}
      if(k<0&&(errno==EAGAIN||errno==EWOULDBLOCK)){pollfd q{fd,POLLOUT,0};poll(&q,1,10);continue;}
      if(k<0&&errno==EINTR)continue;
      fail("FC write");
    }
  }
  static void requestRate2(int fd,uint8_t sys,uint8_t comp,uint32_t msgid,int hz){
    mavlink_message_t m{};
    mavlink_msg_command_long_pack(255,190,&m,sys,comp,MAV_CMD_SET_MESSAGE_INTERVAL,0,msgid,1000000.0f/hz,0,0,0,0,0);
    uint8_t b[MAVLINK_MAX_PACKET_LEN];auto n=mavlink_msg_to_send_buffer(b,&m);writeAll2(fd,b,n);
  }

  void start(const std::string&dev){
    fd=::open(dev.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK);if(fd<0)fail("open FC");
    termios t{};if(tcgetattr(fd,&t)<0)fail("FC tcgetattr");cfmakeraw(&t);
    cfsetispeed(&t,B460800);cfsetospeed(&t,B460800);
    t.c_cflag|=CLOCAL|CREAD;t.c_cflag&=~CRTSCTS;t.c_cflag&=~PARENB;t.c_cflag&=~CSTOPB;t.c_cflag&=~CSIZE;t.c_cflag|=CS8;
    if(tcsetattr(fd,TCSANOW,&t)<0)fail("FC tcsetattr");tcflush(fd,TCIFLUSH);
    th=std::thread([this]{
      mavlink_status_t st{};mavlink_message_t m{};uint8_t sys=0,comp=0,buf[4096];
      int64_t deadline=monoNs()+10000000000LL;
      while(g_running&&!sys&&monoNs()<deadline){
        pollfd p{fd,POLLIN,0};if(poll(&p,1,100)<=0)continue;
        ssize_t n=read(fd,buf,sizeof(buf));if(n<=0)continue;
        for(ssize_t i=0;i<n;i++)if(mavlink_parse_char(MAVLINK_COMM_0,buf[i],&m,&st)&&m.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=m.sysid;comp=m.compid;break;}
      }
      if(!sys){std::cerr<<"FC: HEARTBEAT timeout\n";g_running=false;return;}
      requestRate2(fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,50);
      while(g_running){
        pollfd p{fd,POLLIN,0};if(poll(&p,1,50)<=0)continue;
        for(;;){
          ssize_t n=read(fd,buf,sizeof(buf));
          if(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK))break;
          if(n<=0)break;
          for(ssize_t i=0;i<n;i++)if(mavlink_parse_char(MAVLINK_COMM_0,buf[i],&m,&st)&&m.msgid==MAVLINK_MSG_ID_ATTITUDE){
            mavlink_attitude_t a{};mavlink_msg_attitude_decode(&m,&a);
            Attitude x{a.roll,a.pitch,a.yaw,monoNs(),true};
            std::lock_guard<std::mutex>l(mu);
            hist.push_back(x);
            const int64_t keep_after=x.recv_ns-3000000000LL;
            while(hist.size()>2&&hist.front().recv_ns<keep_after)hist.pop_front();
          }
        }
      }
    });
  }

  bool latest(Attitude*out){
    std::lock_guard<std::mutex>l(mu);
    if(hist.empty())return false;*out=hist.back();return true;
  }

  // Интерполяция по receive timestamp в общей CLOCK_MONOTONIC шкале RPi.
  // Это ещё не аппаратная синхронизация FC; цель теста — проверить именно известные ~137 ms camera age.
  bool sampleAt(int64_t target_ns,Attitude*out,double*bracket_ms,double*nearest_ms){
    std::lock_guard<std::mutex>l(mu);
    if(hist.size()<2||target_ns<hist.front().recv_ns||target_ns>hist.back().recv_ns)return false;
    size_t hi=1;
    while(hi<hist.size()&&hist[hi].recv_ns<target_ns)++hi;
    if(hi>=hist.size())return false;
    const Attitude&a=hist[hi-1];const Attitude&b=hist[hi];
    const int64_t den=b.recv_ns-a.recv_ns;if(den<=0)return false;
    const double u=std::clamp((double)(target_ns-a.recv_ns)/(double)den,0.0,1.0);
    out->roll=a.roll+u*(b.roll-a.roll);
    out->pitch=a.pitch+u*(b.pitch-a.pitch);
    out->yaw=a.yaw+u*wrapRad(b.yaw-a.yaw);
    out->yaw=wrapRad(out->yaw);
    out->recv_ns=target_ns;out->valid=true;
    if(bracket_ms)*bracket_ms=den*1e-6;
    if(nearest_ms)*nearest_ms=std::min(std::llabs(target_ns-a.recv_ns),std::llabs(b.recv_ns-target_ns))*1e-6;
    return true;
  }

  void stop(){if(th.joinable())th.join();if(fd>=0){::close(fd);fd=-1;}}
};

static bool v3Step(const std::vector<cv::Point2f>& ai,const std::vector<cv::Point2f>& bi,
                   const CameraCalib& calib,const Attitude&a0,double h0,const Attitude&a1,double h1,
                   cv::Vec2d*out,int*used,double*scatter){
  std::vector<cv::Point2f> au,bu;
  cv::undistortPoints(ai,au,calib.K,calib.D);cv::undistortPoints(bi,bu,calib.K,calib.D);
  cv::Matx33d W_R_C0=attitudeFluToNwu(a0)*calib.B_R_C;
  cv::Matx33d W_R_C1=attitudeFluToNwu(a1)*calib.B_R_C;
  std::vector<cv::Vec2d>deltas;deltas.reserve(au.size());
  for(size_t i=0;i<au.size();++i){cv::Vec2d g0,g1;if(footprint(au[i],W_R_C0,h0,&g0)&&footprint(bu[i],W_R_C1,h1,&g1))deltas.push_back(g0-g1);}
  return robustDelta(deltas,out,used,scatter);
}

}

int main(int argc,char**argv){
  if(argc<7){std::cerr<<"Использование: "<<argv[0]<<" <camera> <luna> <fc> <csv> <camera_yaml> <camera_offset_mm>\n";return 2;}
  const std::string camdev=argv[1],lunadev=argv[2],fcdev=argv[3],csvpath=argv[4],yaml=argv[5];
  const double offset_m=std::stod(argv[6])/1000.0;
  try{
    CameraCalib calib=loadCameraCalib(yaml);Camera cam;cam.openDev(camdev);LunaReader luna;luna.start(lunadev);FcHistoryReader fc;fc.start(fcdev);
    std::ofstream csv(csvpath,std::ios::trunc);
    csv<<"state,now_ns,frame_ts_ns,frame,camera_age_ms,att_latest_age_ms,att_sync_nearest_ms,att_sync_bracket_ms,luna_age_ms,luna_slant_m,"
          "latest_h_m,sync_h_m,latest_x_m,latest_y_m,latest_path_m,sync_x_m,sync_y_m,sync_path_m,latest_inliers,sync_inliers,latest_scatter_m,sync_scatter_m,"
          "latest_roll,latest_pitch,latest_yaw,sync_roll,sync_pitch,sync_yaw\n";
    cv::setNumThreads(1);std::signal(SIGINT,onSignal);std::signal(SIGTERM,onSignal);
    const char* window="JT-ZERO — V3 TIMING A/B: LATEST vs SYNC";cv::namedWindow(window,cv::WINDOW_NORMAL);cv::resizeWindow(window,1280,720);cv::moveWindow(window,0,0);
    enum class State{READY=0,MOVING=1,DONE=2};State state=State::READY;
    cv::Mat prev;Attitude prev_latest{},prev_sync{};double prev_h_latest=0,prev_h_sync=0;int64_t prev_frame_ts=0;
    Estimate latest_est,sync_est;uint64_t frame_id=0;double rlx=0,rly=0,rsx=0,rsy=0;

    while(g_running){
      pollfd p{cam.fd,POLLIN,0};int pr=poll(&p,1,20);if(pr<0){if(errno==EINTR)continue;fail("camera poll");}if(pr<=0)continue;
      while(g_running){
        v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;
        if(xioctl(cam.fd,VIDIOC_DQBUF,&b)<0){if(errno==EAGAIN)break;fail("VIDIOC_DQBUF");}
        const int64_t now=monoNs();const int64_t frame_ts=tvToNs(b.timestamp);const double camera_age_ms=(now-frame_ts)*1e-6;
        cv::Mat raw(1,(int)b.bytesused,CV_8UC1,cam.bufs[b.index].p);cv::Mat gray=cv::imdecode(raw,cv::IMREAD_GRAYSCALE);
        if(xioctl(cam.fd,VIDIOC_QBUF,&b)<0)fail("VIDIOC_QBUF");if(gray.empty())continue;++frame_id;

        double luna_m=0;int strength=0;int64_t luna_ns=0;Attitude att_latest{},att_sync{};
        const bool have_luna=luna.latest(&luna_m,&strength,&luna_ns);const bool have_latest=fc.latest(&att_latest);
        double sync_bracket_ms=0,sync_nearest_ms=0;const bool have_sync=fc.sampleAt(frame_ts,&att_sync,&sync_bracket_ms,&sync_nearest_ms);
        const double att_latest_age_ms=have_latest?(now-att_latest.recv_ns)*1e-6:1e9;
        const double luna_age_ms=have_luna?(now-luna_ns)*1e-6:1e9;

        double h_latest=0,h_sync=0;double down_latest=0,down_sync=0;
        if(have_luna&&have_latest){cv::Vec3d bw=attitudeFluToNwu(att_latest)*cv::Vec3d(0,0,-1);down_latest=-bw[2];h_latest=luna_m*down_latest-offset_m;}
        if(have_luna&&have_sync){cv::Vec3d bw=attitudeFluToNwu(att_sync)*cv::Vec3d(0,0,-1);down_sync=-bw[2];h_sync=luna_m*down_sync-offset_m;}
        const bool sensors_ok=have_luna&&have_latest&&have_sync&&down_latest>0.20&&down_sync>0.20&&h_latest>0.05&&h_sync>0.05&&luna_age_ms<200&&att_latest_age_ms<200;

        if(state==State::MOVING&&sensors_ok&&!prev.empty()&&prev_latest.valid&&prev_sync.valid&&prev_h_latest>0&&prev_h_sync>0){
          std::vector<cv::Point2f>p0,p1;cv::goodFeaturesToTrack(prev,p0,700,0.01,7);
          if(p0.size()>=30){std::vector<uchar>st;std::vector<float>err;cv::calcOpticalFlowPyrLK(prev,gray,p0,p1,st,err,{21,21},3);std::vector<cv::Point2f>a,bp;
            for(size_t i=0;i<p0.size();++i)if(st[i]){a.push_back(p0[i]);bp.push_back(p1[i]);}
            if(a.size()>=20){cv::Mat mask;cv::findHomography(a,bp,cv::RANSAC,2.0,mask);if(!mask.empty()){
              std::vector<cv::Point2f>ai,bi;for(size_t i=0;i<a.size();++i)if(mask.at<uchar>((int)i)){ai.push_back(a[i]);bi.push_back(bp[i]);}
              const double dt=prev_frame_ts?((frame_ts-prev_frame_ts)*1e-9):0;
              if(ai.size()>=15&&dt>0&&dt<0.2){
                cv::Vec2d dl,ds;int ul=0,us=0;double sl=0,ss=0;
                if(v3Step(ai,bi,calib,prev_latest,prev_h_latest,att_latest,h_latest,&dl,&ul,&sl)&&cv::norm(dl)<0.20){latest_est.x+=dl[0];latest_est.y+=dl[1];latest_est.path+=cv::norm(dl);latest_est.inliers=ul;latest_est.scatter=sl;++latest_est.frames;}
                if(v3Step(ai,bi,calib,prev_sync,prev_h_sync,att_sync,h_sync,&ds,&us,&ss)&&cv::norm(ds)<0.20){sync_est.x+=ds[0];sync_est.y+=ds[1];sync_est.path+=cv::norm(ds);sync_est.inliers=us;sync_est.scatter=ss;++sync_est.frames;}
              }
            }}
          }
        }

        if(sensors_ok){prev=gray.clone();prev_latest=att_latest;prev_sync=att_sync;prev_h_latest=h_latest;prev_h_sync=h_sync;prev_frame_ts=frame_ts;}
        else{prev.release();prev_latest.valid=false;prev_sync.valid=false;prev_h_latest=prev_h_sync=0;prev_frame_ts=0;}

        cv::Mat bgr,video;cv::cvtColor(gray,bgr,cv::COLOR_GRAY2BGR);cv::resize(bgr,video,{870,653});cv::Mat canvas(720,1280,CV_8UC3,cv::Scalar(12,12,12));video.copyTo(canvas(cv::Rect(0,67,870,653)));
        ru(canvas,"JT-ZERO — V3 TIMING A/B — LATEST vs SYNC",{22,35},17,{245,245,245},cv::QT_FONT_BOLD);cv::Mat panel=canvas(cv::Rect(870,0,410,720));
        ru(panel,sensors_ok?"СИСТЕМА ГОТОВА":"ЖДИТЕ ИСТОРИЮ FC",{18,42},14,sensors_ok?cv::Scalar(90,220,90):cv::Scalar(0,210,255),cv::QT_FONT_BOLD);
        const char* now_text=state==State::READY?"СЕЙЧАС: ТОЧКА A — НЕ ДВИГАТЬ":state==State::MOVING?"СЕЙЧАС: ДВИЖЕНИЕ A -> B":"СЕЙЧАС: РЕЗУЛЬТАТ ЗАФИКСИРОВАН";ru(panel,now_text,{18,82},10,{255,255,255},cv::QT_FONT_BOLD);
        char z[220];snprintf(z,sizeof(z),"Возраст кадра: %.1f мс",camera_age_ms);ru(panel,z,{18,125},11,{220,220,220});snprintf(z,sizeof(z),"ATT latest: %.1f мс  SYNC nearest: %.1f мс",att_latest_age_ms,sync_nearest_ms);ru(panel,z,{18,160},10,{220,220,220});
        snprintf(z,sizeof(z),"LATEST NET/PATH: %.1f / %.1f мм",std::hypot(latest_est.x,latest_est.y)*1000,latest_est.path*1000);ru(panel,z,{18,235},13,{245,245,245},cv::QT_FONT_BOLD);
        snprintf(z,sizeof(z),"SYNC NET/PATH: %.1f / %.1f мм",std::hypot(sync_est.x,sync_est.y)*1000,sync_est.path*1000);ru(panel,z,{18,295},13,{245,245,245},cv::QT_FONT_BOLD);
        snprintf(z,sizeof(z),"SYNC-LATEST NET: %+.1f мм",(std::hypot(sync_est.x,sync_est.y)-std::hypot(latest_est.x,latest_est.y))*1000);ru(panel,z,{18,345},11,{230,230,230},cv::QT_FONT_BOLD);
        snprintf(z,sizeof(z),"Pitch latest/sync: %+.2f / %+.2f°",att_latest.pitch*180/kPi,att_sync.pitch*180/kPi);ru(panel,z,{18,410},10,{220,220,220});snprintf(z,sizeof(z),"Roll latest/sync: %+.2f / %+.2f°",att_latest.roll*180/kPi,att_sync.roll*180/kPi);ru(panel,z,{18,445},10,{220,220,220});
        snprintf(z,sizeof(z),"Точек latest/sync: %d / %d",latest_est.inliers,sync_est.inliers);ru(panel,z,{18,495},10,{220,220,220});snprintf(z,sizeof(z),"Разброс: %.2f / %.2f мм",latest_est.scatter*1000,sync_est.scatter*1000);ru(panel,z,{18,530},10,{220,220,220});
        if(state==State::DONE){snprintf(z,sizeof(z),"ИТОГ LATEST/SYNC: %.1f / %.1f мм",std::hypot(rlx,rly)*1000,std::hypot(rsx,rsy)*1000);ru(panel,z,{18,610},12,{245,245,245},cv::QT_FONT_BOLD);}ru(panel,state==State::READY?"ПРОБЕЛ — СТАРТ":state==State::MOVING?"В B: ПРОБЕЛ — СТОП":"Q / ESC — ВЫХОД",{18,675},11,{210,210,210},cv::QT_FONT_BOLD);
        cv::imshow(window,canvas);int rk=cv::waitKeyEx(10),key=rk<0?-1:(rk&0xff);if(cv::getWindowProperty(window,cv::WND_PROP_VISIBLE)<1){g_running=false;break;}
        if(key==' '&&sensors_ok){if(state==State::READY){latest_est={};sync_est={};prev.release();prev_latest.valid=false;prev_sync.valid=false;prev_h_latest=prev_h_sync=0;prev_frame_ts=0;state=State::MOVING;}else if(state==State::MOVING){rlx=latest_est.x;rly=latest_est.y;rsx=sync_est.x;rsy=sync_est.y;state=State::DONE;}}
        if(key=='q'||key=='Q'||key==27){g_running=false;break;}
        if(csv){csv<<(int)state<<','<<now<<','<<frame_ts<<','<<frame_id<<','<<std::fixed<<std::setprecision(6)<<camera_age_ms<<','<<att_latest_age_ms<<','<<sync_nearest_ms<<','<<sync_bracket_ms<<','<<luna_age_ms<<','<<luna_m<<','<<h_latest<<','<<h_sync<<','<<latest_est.x<<','<<latest_est.y<<','<<latest_est.path<<','<<sync_est.x<<','<<sync_est.y<<','<<sync_est.path<<','<<latest_est.inliers<<','<<sync_est.inliers<<','<<latest_est.scatter<<','<<sync_est.scatter<<','<<att_latest.roll<<','<<att_latest.pitch<<','<<att_latest.yaw<<','<<att_sync.roll<<','<<att_sync.pitch<<','<<att_sync.yaw<<'\n';if((frame_id&3u)==0u)csv.flush();}
      }
    }
    cv::destroyAllWindows();std::cout<<"CSV: "<<csvpath<<"\n";return 0;
  }catch(const std::exception&e){std::cerr<<"GROUND MOTION V3 TIMING AB FAIL: "<<e.what()<<"\n";return 1;}
}
