// JT-Zero Ground Motion V2/V3/V4 A/B diagnostic.
// V4 = robust fixed-scale SE(2) fit on the same ground-plane correspondences as V3.
// The old V2/V3 implementation is included only to reuse camera/sensor/calibration helpers.
#define main jtzero_v2_v3_unused_main
#include "ground_motion_live_v2_v3_ab.cpp"
#undef main

namespace {

struct Rigid2DResult {
  cv::Vec2d t{0,0};
  double yaw_rad=0;
  int used=0;
  double scatter=0;
};

static bool fitRigidFixedScaleOnce(const std::vector<cv::Vec2d>& src,
                                   const std::vector<cv::Vec2d>& dst,
                                   const std::vector<int>& idx,
                                   Rigid2DResult* out) {
  if(idx.size()<3) return false;
  cv::Vec2d cs(0,0), cd(0,0);
  for(int i:idx){ cs+=src[i]; cd+=dst[i]; }
  cs*=1.0/idx.size(); cd*=1.0/idx.size();

  double c=0,s=0;
  for(int i:idx){
    cv::Vec2d a=src[i]-cs, b=dst[i]-cd;
    c += a[0]*b[0] + a[1]*b[1];
    s += a[0]*b[1] - a[1]*b[0];
  }
  if(std::abs(c)+std::abs(s)<1e-12) return false;
  const double th=std::atan2(s,c);
  const double ct=std::cos(th), st=std::sin(th);
  cv::Vec2d rcs(ct*cs[0]-st*cs[1], st*cs[0]+ct*cs[1]);
  out->t=cd-rcs;
  out->yaw_rad=th;
  out->used=(int)idx.size();
  return true;
}

static bool robustRigidFixedScale(const std::vector<cv::Vec2d>& src,
                                  const std::vector<cv::Vec2d>& dst,
                                  Rigid2DResult* out) {
  if(src.size()!=dst.size() || src.size()<12) return false;
  std::vector<int> idx(src.size());
  for(size_t i=0;i<src.size();++i) idx[i]=(int)i;

  Rigid2DResult fit;
  std::vector<double> residuals;
  for(int iter=0;iter<4;++iter){
    if(!fitRigidFixedScaleOnce(src,dst,idx,&fit)) return false;
    const double ct=std::cos(fit.yaw_rad), st=std::sin(fit.yaw_rad);
    residuals.clear(); residuals.reserve(src.size());
    for(size_t i=0;i<src.size();++i){
      cv::Vec2d p(ct*src[i][0]-st*src[i][1]+fit.t[0],
                  st*src[i][0]+ct*src[i][1]+fit.t[1]);
      residuals.push_back(cv::norm(p-dst[i]));
    }
    const double med=median(residuals);
    const double gate=std::max(0.0020,3.5*med+0.0005);
    std::vector<int> next;
    next.reserve(src.size());
    for(size_t i=0;i<residuals.size();++i) if(residuals[i]<=gate) next.push_back((int)i);
    if(next.size()<10) return false;
    if(next==idx) break;
    idx.swap(next);
  }
  if(!fitRigidFixedScaleOnce(src,dst,idx,&fit)) return false;
  const double ct=std::cos(fit.yaw_rad), st=std::sin(fit.yaw_rad);
  residuals.clear(); residuals.reserve(idx.size());
  for(int i:idx){
    cv::Vec2d p(ct*src[i][0]-st*src[i][1]+fit.t[0],
                st*src[i][0]+ct*src[i][1]+fit.t[1]);
    residuals.push_back(cv::norm(p-dst[i]));
  }
  fit.scatter=median(residuals);
  fit.used=(int)idx.size();
  *out=fit;
  return true;
}

}

int main(int argc,char**argv){
  if(argc<7){
    std::cerr<<"Использование: "<<argv[0]<<" <camera> <luna> <fc> <csv> <camera_yaml> <camera_offset_mm>\n";
    return 2;
  }
  const std::string camdev=argv[1],lunadev=argv[2],fcdev=argv[3],csvpath=argv[4],yaml=argv[5];
  const double offset_m=std::stod(argv[6])/1000.0;
  try{
    CameraCalib calib=loadCameraCalib(yaml);
    Camera cam; cam.openDev(camdev);
    LunaReader luna; luna.start(lunadev);
    FcReader fc; fc.start(fcdev);
    std::ofstream csv(csvpath,std::ios::trunc);
    csv<<"mono_ns,frame,luna_slant_m,height_v3_m,"
          "v2_x_m,v2_y_m,v2_path_m,"
          "v3_x_m,v3_y_m,v3_path_m,"
          "v4_x_m,v4_y_m,v4_path_m,v4_yaw_step_rad,v4_inliers,v4_scatter_m,"
          "inliers,scatter_m,roll,pitch,yaw\n";

    cv::setNumThreads(1);
    std::signal(SIGINT,onSignal); std::signal(SIGTERM,onSignal);
    const char* window="JT-ZERO — GROUND MOTION V2/V3/V4 A/B";
    cv::namedWindow(window,cv::WINDOW_NORMAL); cv::resizeWindow(window,1280,720); cv::moveWindow(window,0,0);

    enum class State{READY,MOVING,DONE};
    State state=State::READY;
    cv::Mat prev; Attitude prev_att{}; double prev_h_v3=0; int64_t prev_ns=0;
    Estimate v2,v3,v4; uint64_t frame_id=0;
    double r2x=0,r2y=0,r3x=0,r3y=0,r4x=0,r4y=0;
    double v4_yaw_step=0; int v4_used=0; double v4_scatter=0;

    while(g_running){
      pollfd p{cam.fd,POLLIN,0}; int pr=poll(&p,1,20);
      if(pr<0){ if(errno==EINTR) continue; fail("camera poll"); }
      if(pr<=0) continue;
      while(g_running){
        v4l2_buffer b{}; b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory=V4L2_MEMORY_MMAP;
        if(xioctl(cam.fd,VIDIOC_DQBUF,&b)<0){ if(errno==EAGAIN) break; fail("VIDIOC_DQBUF"); }
        int64_t now=monoNs();
        cv::Mat raw(1,(int)b.bytesused,CV_8UC1,cam.bufs[b.index].p);
        cv::Mat gray=cv::imdecode(raw,cv::IMREAD_GRAYSCALE);
        if(xioctl(cam.fd,VIDIOC_QBUF,&b)<0) fail("VIDIOC_QBUF");
        if(gray.empty()) continue; ++frame_id;

        double luna_m=0; int strength=0; int64_t luna_ns=0; Attitude att{};
        bool have_luna=luna.latest(&luna_m,&strength,&luna_ns), have_att=fc.latest(&att);
        cv::Matx33d W_R_B=have_att?attitudeFluToNwu(att):cv::Matx33d::eye();
        cv::Vec3d beam_w=W_R_B*cv::Vec3d(0,0,-1);
        double beam_down=-beam_w[2];
        double h_v3=(have_luna&&have_att)?luna_m*beam_down-offset_m:0;
        double h_v2=have_luna?luna_m-offset_m:0;
        bool sensors_ok=have_luna&&have_att&&beam_down>0.20&&h_v3>0.05&&
                        (now-luna_ns)<200000000LL&&(now-att.recv_ns)<200000000LL;

        if(state==State::MOVING&&sensors_ok&&!prev.empty()&&prev_att.valid&&prev_h_v3>0){
          std::vector<cv::Point2f> p0,p1;
          cv::goodFeaturesToTrack(prev,p0,700,0.01,7);
          if(p0.size()>=30){
            std::vector<uchar> st; std::vector<float> err;
            cv::calcOpticalFlowPyrLK(prev,gray,p0,p1,st,err,{21,21},3);
            std::vector<cv::Point2f>a,bp;
            for(size_t i=0;i<p0.size();++i) if(st[i]){a.push_back(p0[i]);bp.push_back(p1[i]);}
            if(a.size()>=20){
              cv::Mat mask; cv::Mat H=cv::findHomography(a,bp,cv::RANSAC,2.0,mask);
              if(!H.empty()&&!mask.empty()){
                int nin=cv::countNonZero(mask);
                double dt=prev_ns?((now-prev_ns)*1e-9):0;
                if(nin>=15&&dt>0&&dt<0.2){
                  // V2 legacy, unchanged.
                  cv::Matx33d K(calib.fx,0,calib.cx,0,calib.fy,calib.cy,0,0,1),Ki=K.inv();
                  cv::Matx33d L_R_C0=RzRyRx(prev_att.roll,prev_att.pitch,prev_att.yaw)*calib.B_R_C;
                  cv::Matx33d L_R_C1=RzRyRx(att.roll,att.pitch,att.yaw)*calib.B_R_C;
                  cv::Matx33d Hr=K*(L_R_C1.t()*L_R_C0)*Ki;
                  cv::Mat Hrd(3,3,CV_64F); for(int r=0;r<3;r++)for(int c=0;c<3;c++)Hrd.at<double>(r,c)=Hr(r,c);
                  cv::Mat Hd=Hrd.inv()*H;
                  if(std::abs(Hd.at<double>(2,2))>1e-12){
                    Hd/=Hd.at<double>(2,2); cv::Matx31d c0(calib.cx,calib.cy,1),q;
                    for(int r=0;r<3;r++)q(r)=Hd.at<double>(r,0)*c0(0)+Hd.at<double>(r,1)*c0(1)+Hd.at<double>(r,2);
                    if(std::abs(q(2))>1e-12){
                      double dx=q(0)/q(2)-calib.cx,dy=q(1)/q(2)-calib.cy;
                      double mx=-dx*h_v2/calib.fx,my=-dy*h_v2/calib.fy;
                      if(std::hypot(mx,my)<0.20){v2.x+=mx;v2.y+=my;v2.path+=std::hypot(mx,my);v2.vx=mx/dt;v2.vy=my/dt;v2.height=h_v2;v2.inliers=nin;++v2.frames;}
                    }
                  }

                  // Common ground-plane correspondences for V3 and V4.
                  std::vector<cv::Point2f> ai,bi;
                  for(size_t i=0;i<a.size();++i) if(mask.at<uchar>((int)i)){ai.push_back(a[i]);bi.push_back(bp[i]);}
                  std::vector<cv::Point2f> au,bu;
                  cv::undistortPoints(ai,au,calib.K,calib.D); cv::undistortPoints(bi,bu,calib.K,calib.D);
                  cv::Matx33d W_R_C0=attitudeFluToNwu(prev_att)*calib.B_R_C;
                  cv::Matx33d W_R_C1=W_R_B*calib.B_R_C;
                  std::vector<cv::Vec2d> g0s,g1s,deltas;
                  g0s.reserve(au.size()); g1s.reserve(au.size()); deltas.reserve(au.size());
                  for(size_t i=0;i<au.size();++i){
                    cv::Vec2d g0,g1;
                    if(footprint(au[i],W_R_C0,prev_h_v3,&g0)&&footprint(bu[i],W_R_C1,h_v3,&g1)){
                      g0s.push_back(g0); g1s.push_back(g1); deltas.push_back(g0-g1);
                    }
                  }

                  cv::Vec2d dxy; int used=0; double scatter=0;
                  if(robustDelta(deltas,&dxy,&used,&scatter)&&cv::norm(dxy)<0.20){
                    v3.x+=dxy[0];v3.y+=dxy[1];v3.path+=cv::norm(dxy);v3.vx=dxy[0]/dt;v3.vy=dxy[1]/dt;
                    v3.height=h_v3;v3.inliers=used;v3.scatter=scatter;++v3.frames;
                  }

                  Rigid2DResult rigid;
                  // Map current-frame ground vectors g1 -> previous-frame vectors g0.
                  // For pure camera translation, g0 = g1 + dC, hence rigid.t = dC.
                  if(robustRigidFixedScale(g1s,g0s,&rigid)&&cv::norm(rigid.t)<0.20){
                    v4.x+=rigid.t[0];v4.y+=rigid.t[1];v4.path+=cv::norm(rigid.t);v4.vx=rigid.t[0]/dt;v4.vy=rigid.t[1]/dt;
                    v4.height=h_v3;v4.inliers=rigid.used;v4.scatter=rigid.scatter;++v4.frames;
                    v4_yaw_step=rigid.yaw_rad;v4_used=rigid.used;v4_scatter=rigid.scatter;
                  }
                }
              }
            }
          }
        }

        if(sensors_ok){prev=gray.clone();prev_att=att;prev_h_v3=h_v3;prev_ns=now;}
        else{prev.release();prev_att.valid=false;prev_h_v3=0;prev_ns=0;}

        cv::Mat bgr,video;cv::cvtColor(gray,bgr,cv::COLOR_GRAY2BGR);cv::resize(bgr,video,{870,653});
        cv::Mat canvas(720,1280,CV_8UC3,cv::Scalar(12,12,12));video.copyTo(canvas(cv::Rect(0,67,870,653)));
        ru(canvas,"JT-ZERO — V2 / V3 / V4 — ОДНИ И ТЕ ЖЕ ДАННЫЕ",{22,35},17,{245,245,245},cv::QT_FONT_BOLD);
        cv::Mat panel=canvas(cv::Rect(870,0,410,720));
        ru(panel,sensors_ok?"СИСТЕМА ГОТОВА":"ЖДИТЕ ДАТЧИКИ",{18,42},15,sensors_ok?cv::Scalar(90,220,90):cv::Scalar(0,210,255),cv::QT_FONT_BOLD);
        const char* now_text=state==State::READY?"СЕЙЧАС: ТОЧКА A — НЕ ДВИГАТЬ":state==State::MOVING?"СЕЙЧАС: ДВИЖЕНИЕ A -> B":"СЕЙЧАС: РЕЗУЛЬТАТ ЗАФИКСИРОВАН";
        ru(panel,now_text,{18,82},10,{255,255,255},cv::QT_FONT_BOLD);
        char z[200];
        snprintf(z,sizeof(z),"Luna %.1f мм   H %.1f мм",luna_m*1000,h_v3*1000);ru(panel,z,{18,118},10,{220,220,220});
        snprintf(z,sizeof(z),"V2 NET/PATH: %.1f / %.1f мм",std::hypot(v2.x,v2.y)*1000,v2.path*1000);ru(panel,z,{18,180},13,{235,235,235},cv::QT_FONT_BOLD);
        snprintf(z,sizeof(z),"V3 NET/PATH: %.1f / %.1f мм",std::hypot(v3.x,v3.y)*1000,v3.path*1000);ru(panel,z,{18,240},13,{235,235,235},cv::QT_FONT_BOLD);
        snprintf(z,sizeof(z),"V4 NET/PATH: %.1f / %.1f мм",std::hypot(v4.x,v4.y)*1000,v4.path*1000);ru(panel,z,{18,300},14,{245,245,245},cv::QT_FONT_BOLD);
        snprintf(z,sizeof(z),"V4 dYaw: %+.3f°",v4_yaw_step*180/kPi);ru(panel,z,{18,355},10,{220,220,220});
        snprintf(z,sizeof(z),"V4 точек/разброс: %d / %.2f мм",v4_used,v4_scatter*1000);ru(panel,z,{18,390},10,{220,220,220});
        snprintf(z,sizeof(z),"Крен/тангаж: %+.2f / %+.2f°",att.roll*180/kPi,att.pitch*180/kPi);ru(panel,z,{18,445},10,{220,220,220});
        if(state==State::DONE){
          snprintf(z,sizeof(z),"ИТОГ V2: %.1f мм",std::hypot(r2x,r2y)*1000);ru(panel,z,{18,525},11,{230,230,230});
          snprintf(z,sizeof(z),"ИТОГ V3: %.1f мм",std::hypot(r3x,r3y)*1000);ru(panel,z,{18,565},11,{230,230,230});
          snprintf(z,sizeof(z),"ИТОГ V4: %.1f мм",std::hypot(r4x,r4y)*1000);ru(panel,z,{18,605},13,{250,250,250},cv::QT_FONT_BOLD);
        }
        ru(panel,state==State::READY?"ПРОБЕЛ — СТАРТ":state==State::MOVING?"В B: ПРОБЕЛ — СТОП":"Q / ESC — ВЫХОД",{18,680},11,{210,210,210},cv::QT_FONT_BOLD);
        cv::imshow(window,canvas);
        int raw_key=cv::waitKeyEx(10),key=raw_key<0?-1:(raw_key&0xff);
        if(cv::getWindowProperty(window,cv::WND_PROP_VISIBLE)<1){g_running=false;break;}
        if(key==' '&&sensors_ok){
          if(state==State::READY){v2={};v3={};v4={};v4_yaw_step=0;v4_used=0;v4_scatter=0;prev.release();prev_att.valid=false;prev_h_v3=0;prev_ns=0;state=State::MOVING;}
          else if(state==State::MOVING){r2x=v2.x;r2y=v2.y;r3x=v3.x;r3y=v3.y;r4x=v4.x;r4y=v4.y;v2.vx=v2.vy=v3.vx=v3.vy=v4.vx=v4.vy=0;state=State::DONE;}
        }
        if(key=='q'||key=='Q'||key==27){g_running=false;break;}

        if(csv&&sensors_ok){
          csv<<now<<','<<frame_id<<','<<std::fixed<<std::setprecision(6)
             <<luna_m<<','<<h_v3<<','
             <<v2.x<<','<<v2.y<<','<<v2.path<<','
             <<v3.x<<','<<v3.y<<','<<v3.path<<','
             <<v4.x<<','<<v4.y<<','<<v4.path<<','<<v4_yaw_step<<','<<v4_used<<','<<v4_scatter<<','
             <<v3.inliers<<','<<v3.scatter<<','<<att.roll<<','<<att.pitch<<','<<att.yaw<<'\n';
          if((frame_id&3u)==0u) csv.flush();
        }
      }
    }
    cv::destroyAllWindows();
    return 0;
  }catch(const std::exception&e){std::cerr<<"GROUND MOTION V2/V3/V4 AB FAIL: "<<e.what()<<"\n";return 1;}
}
