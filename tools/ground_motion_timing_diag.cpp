// JT-Zero Ground Motion timing diagnostic.
// Цель: измерить реальный возраст кадра OV9281 в момент DQBUF/после decode
// и возраст последних FC ATTITUDE / TF-Luna без изменения алгоритмов V2/V3/V4.
#define main jtzero_ab_unused_main
#include "ground_motion_live_v2_v3_ab.cpp"
#undef main

#include <deque>
#include <limits>

namespace {

double med(std::deque<double> v){
  if(v.empty()) return std::numeric_limits<double>::quiet_NaN();
  std::vector<double> a(v.begin(),v.end());
  size_t n=a.size()/2;
  std::nth_element(a.begin(),a.begin()+n,a.end());
  double m=a[n];
  if(a.size()%2==0){std::nth_element(a.begin(),a.begin()+n-1,a.end());m=0.5*(m+a[n-1]);}
  return m;
}

int64_t timevalNs(const timeval& tv){
  return int64_t(tv.tv_sec)*1000000000LL + int64_t(tv.tv_usec)*1000LL;
}

void pushBounded(std::deque<double>& q,double v,size_t n=240){
  if(std::isfinite(v)) q.push_back(v);
  while(q.size()>n) q.pop_front();
}

}

int main(int argc,char**argv){
  if(argc<5){
    std::cerr<<"Использование: "<<argv[0]<<" <camera> <luna> <fc> <csv>\n";
    return 2;
  }
  const std::string camdev=argv[1], lunadev=argv[2], fcdev=argv[3], csvpath=argv[4];
  try{
    Camera cam; cam.openDev(camdev);
    LunaReader luna; luna.start(lunadev);
    FcReader fc; fc.start(fcdev);

    std::ofstream csv(csvpath,std::ios::trunc);
    csv<<"frame,dq_ns,cam_ts_ns,cam_ts_monotonic,cam_age_dq_ms,decode_ms,cam_age_postdecode_ms,"
          "att_recv_ns,att_age_dq_ms,luna_recv_ns,luna_age_dq_ms,roll,pitch,yaw,luna_m,v4l2_flags\n";

    std::signal(SIGINT,onSignal); std::signal(SIGTERM,onSignal);
    cv::setNumThreads(1);
    const char* window="JT-ZERO — ДИАГНОСТИКА СИНХРОНИЗАЦИИ";
    cv::namedWindow(window,cv::WINDOW_NORMAL); cv::resizeWindow(window,1100,650); cv::moveWindow(window,0,0);

    uint64_t frame=0;
    std::deque<double> camAgeQ,decodeQ,attAgeQ,lunaAgeQ;
    bool everMono=false, everNonMono=false;

    while(g_running){
      pollfd p{cam.fd,POLLIN,0};
      int pr=poll(&p,1,50);
      if(pr<0){if(errno==EINTR)continue;fail("camera poll");}
      if(pr<=0) continue;

      while(g_running){
        v4l2_buffer b{}; b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory=V4L2_MEMORY_MMAP;
        if(xioctl(cam.fd,VIDIOC_DQBUF,&b)<0){if(errno==EAGAIN)break;fail("VIDIOC_DQBUF");}
        const int64_t dq_ns=monoNs();
        const bool ts_mono=(b.flags & V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC)!=0;
        everMono|=ts_mono; everNonMono|=!ts_mono;
        const int64_t cam_ts_ns=timevalNs(b.timestamp);
        const double cam_age_dq_ms=ts_mono ? (dq_ns-cam_ts_ns)*1e-6 : std::numeric_limits<double>::quiet_NaN();

        cv::Mat raw(1,(int)b.bytesused,CV_8UC1,cam.bufs[b.index].p);
        const int64_t dec0=monoNs();
        cv::Mat gray=cv::imdecode(raw,cv::IMREAD_GRAYSCALE);
        const int64_t dec1=monoNs();
        if(xioctl(cam.fd,VIDIOC_QBUF,&b)<0)fail("VIDIOC_QBUF");
        if(gray.empty())continue;
        ++frame;

        Attitude att{}; double luna_m=0; int strength=0; int64_t luna_ns=0;
        const bool have_att=fc.latest(&att);
        const bool have_luna=luna.latest(&luna_m,&strength,&luna_ns);
        const double decode_ms=(dec1-dec0)*1e-6;
        const double cam_age_post_ms=ts_mono ? (dec1-cam_ts_ns)*1e-6 : std::numeric_limits<double>::quiet_NaN();
        const double att_age_ms=have_att ? (dq_ns-att.recv_ns)*1e-6 : std::numeric_limits<double>::quiet_NaN();
        const double luna_age_ms=have_luna ? (dq_ns-luna_ns)*1e-6 : std::numeric_limits<double>::quiet_NaN();

        pushBounded(camAgeQ,cam_age_dq_ms); pushBounded(decodeQ,decode_ms);
        pushBounded(attAgeQ,att_age_ms); pushBounded(lunaAgeQ,luna_age_ms);

        csv<<frame<<','<<dq_ns<<','<<cam_ts_ns<<','<<(ts_mono?1:0)<<','
           <<std::fixed<<std::setprecision(6)<<cam_age_dq_ms<<','<<decode_ms<<','<<cam_age_post_ms<<','
           <<(have_att?att.recv_ns:0)<<','<<att_age_ms<<','<<(have_luna?luna_ns:0)<<','<<luna_age_ms<<','
           <<(have_att?att.roll:0)<<','<<(have_att?att.pitch:0)<<','<<(have_att?att.yaw:0)<<','
           <<(have_luna?luna_m:0)<<','<<b.flags<<'\n';
        if((frame&7u)==0u)csv.flush();

        cv::Mat bgr,canvas; cv::cvtColor(gray,bgr,cv::COLOR_GRAY2BGR); cv::resize(bgr,bgr,{800,600});
        canvas=cv::Mat(650,1100,CV_8UC3,cv::Scalar(12,12,12)); bgr.copyTo(canvas(cv::Rect(0,50,800,600)));
        ru(canvas,"JT-ZERO — СИНХРОНИЗАЦИЯ КАМЕРА / FC / LUNA",{20,34},17,{245,245,245},cv::QT_FONT_BOLD);
        cv::Mat panel=canvas(cv::Rect(800,0,300,650)); char z[180];
        ru(panel,"ТОЛЬКО ДИАГНОСТИКА",{15,45},13,{90,220,90},cv::QT_FONT_BOLD);
        ru(panel,"Ничего не двигать",{15,82},11,{230,230,230},cv::QT_FONT_BOLD);
        snprintf(z,sizeof(z),"frame: %llu",(unsigned long long)frame);ru(panel,z,{15,130},10,{220,220,220});
        snprintf(z,sizeof(z),"V4L2 MONO: %s",ts_mono?"ДА":"НЕТ");ru(panel,z,{15,170},11,ts_mono?cv::Scalar(90,220,90):cv::Scalar(0,210,255),cv::QT_FONT_BOLD);
        snprintf(z,sizeof(z),"camera age med: %.2f ms",med(camAgeQ));ru(panel,z,{15,220},10,{235,235,235});
        snprintf(z,sizeof(z),"decode med: %.2f ms",med(decodeQ));ru(panel,z,{15,255},10,{235,235,235});
        snprintf(z,sizeof(z),"ATT age med: %.2f ms",med(attAgeQ));ru(panel,z,{15,305},10,{235,235,235});
        snprintf(z,sizeof(z),"Luna age med: %.2f ms",med(lunaAgeQ));ru(panel,z,{15,340},10,{235,235,235});
        if(have_att){snprintf(z,sizeof(z),"R/P: %+.2f / %+.2f deg",att.roll*180/kPi,att.pitch*180/kPi);ru(panel,z,{15,395},10,{220,220,220});}
        if(have_luna){snprintf(z,sizeof(z),"Luna: %.1f mm",luna_m*1000);ru(panel,z,{15,430},10,{220,220,220});}
        if(everNonMono)ru(panel,"ВНИМАНИЕ: есть non-MONO timestamps",{15,485},9,{0,210,255},cv::QT_FONT_BOLD);
        ru(panel,"Q / ESC — ВЫХОД",{15,610},10,{220,220,220},cv::QT_FONT_BOLD);
        cv::imshow(window,canvas);
        int raw_key=cv::waitKeyEx(10),key=raw_key<0?-1:(raw_key&0xff);
        if(cv::getWindowProperty(window,cv::WND_PROP_VISIBLE)<1){g_running=false;break;}
        if(key=='q'||key=='Q'||key==27){g_running=false;break;}
      }
    }

    csv.flush(); cv::destroyAllWindows();
    std::cout<<"CSV: "<<csvpath<<"\n";
    std::cout<<"V4L2 timestamps: monotonic="<<(everMono?"yes":"no")
             <<" non_monotonic_seen="<<(everNonMono?"yes":"no")<<"\n";
    std::cout<<"median camera age @DQBUF: "<<med(camAgeQ)<<" ms\n";
    std::cout<<"median decode: "<<med(decodeQ)<<" ms\n";
    std::cout<<"median ATT age @DQBUF: "<<med(attAgeQ)<<" ms\n";
    std::cout<<"median Luna age @DQBUF: "<<med(lunaAgeQ)<<" ms\n";
    return 0;
  }catch(const std::exception&e){std::cerr<<"TIMING DIAG FAIL: "<<e.what()<<"\n";return 1;}
}
