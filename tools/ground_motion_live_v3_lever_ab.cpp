// JT-Zero Ground Motion V3 CURRENT vs LEVER strict A/B.
// Один и тот же поток кадров/KLT/RANSAC/Luna/SYNC attitude.
// CURRENT: высота камеры = вертикальная высота TF-Luna (старое поведение).
// LEVER:   высота камеры = H_luna + (R_W_B * t_LunaToCamera_B).z.
//
// ВАЖНО: по умолчанию используем только надежно измеренный CAD lever arm в плоскости:
//   t_LC body FLU = [+49.16, +0.22, 0.00] mm
// Z намеренно 0: STEP не определяет pinhole-Z OV9281 достаточно точно.
// Переопределение через env:
//   JTZERO_LEVER_X_MM, JTZERO_LEVER_Y_MM, JTZERO_LEVER_Z_MM
//
// ground_motion_live_v3_sync_ab.cpp сам переименовывает main своего базового include,
// поэтому здесь задаём имя, которое он ожидает, чтобы его собственный main тоже был скрыт.
#define jtzero_v2_v3_unused_main jtzero_v3_sync_ab_unused_main
#include "ground_motion_live_v3_sync_ab.cpp"
#undef jtzero_v2_v3_unused_main
#include <cstdlib>

namespace {

double envMm(const char* name,double def_mm){
  const char* s=std::getenv(name);
  if(!s||!*s)return def_mm;
  try{return std::stod(s);}catch(...){return def_mm;}
}

}

int main(int argc,char**argv){
  if(argc<7){
    std::cerr<<"Использование: "<<argv[0]<<" <camera> <luna> <fc> <csv> <camera_yaml> <camera_offset_mm>\n";
    return 2;
  }
  const std::string camdev=argv[1],lunadev=argv[2],fcdev=argv[3],csvpath=argv[4],yaml=argv[5];
  const double offset_m=std::stod(argv[6])/1000.0;
  const cv::Vec3d t_lc_b(envMm("JTZERO_LEVER_X_MM",49.16)/1000.0,
                         envMm("JTZERO_LEVER_Y_MM",0.22)/1000.0,
                         envMm("JTZERO_LEVER_Z_MM",0.0)/1000.0);
  try{
    CameraCalib calib=loadCameraCalib(yaml);
    Camera cam;cam.openDev(camdev);
    LunaReader luna;luna.start(lunadev);
    FcHistoryReader fc;fc.start(fcdev);
    std::ofstream csv(csvpath,std::ios::trunc);
    csv<<"state,now_ns,frame_ts_ns,frame,camera_age_ms,att_sync_nearest_ms,att_sync_bracket_ms,luna_age_ms,luna_slant_m,"
          "h_luna_m,lever_dh_m,h_camera_m,current_x_m,current_y_m,current_path_m,lever_x_m,lever_y_m,lever_path_m,"
          "current_inliers,lever_inliers,current_scatter_m,lever_scatter_m,roll,pitch,yaw\n";
    cv::setNumThreads(1);std::signal(SIGINT,onSignal);std::signal(SIGTERM,onSignal);
    const char* window="JT-ZERO — V3 A/B: CURRENT vs CAD LEVER";
    cv::namedWindow(window,cv::WINDOW_NORMAL);cv::resizeWindow(window,1280,720);cv::moveWindow(window,0,0);
    enum class State{READY=0,MOVING=1,DONE=2};State state=State::READY;
    cv::Mat prev;Attitude prev_att{};double prev_h_current=0,prev_h_lever=0;int64_t prev_frame_ts=0;
    Estimate current_est,lever_est;uint64_t frame_id=0;double rcx=0,rcy=0,rlx=0,rly=0;

    while(g_running){
      pollfd p{cam.fd,POLLIN,0};int pr=poll(&p,1,20);
      if(pr<0){if(errno==EINTR)continue;fail("camera poll");}
      if(pr<=0)continue;
      while(g_running){
        v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;
        if(xioctl(cam.fd,VIDIOC_DQBUF,&b)<0){if(errno==EAGAIN)break;fail("VIDIOC_DQBUF");}
        const int64_t now=monoNs();const int64_t frame_ts=tvToNs(b.timestamp);const double camera_age_ms=(now-frame_ts)*1e-6;
        cv::Mat raw(1,(int)b.bytesused,CV_8UC1,cam.bufs[b.index].p);cv::Mat gray=cv::imdecode(raw,cv::IMREAD_GRAYSCALE);
        if(xioctl(cam.fd,VIDIOC_QBUF,&b)<0)fail("VIDIOC_QBUF");if(gray.empty())continue;++frame_id;

        double luna_m=0;int strength=0;int64_t luna_ns=0;Attitude att{};
        const bool have_luna=luna.latest(&luna_m,&strength,&luna_ns);
        double sync_bracket_ms=0,sync_nearest_ms=0;const bool have_att=fc.sampleAt(frame_ts,&att,&sync_bracket_ms,&sync_nearest_ms);
        const double luna_age_ms=have_luna?(now-luna_ns)*1e-6:1e9;
        double h_current=0,h_lever=0,lever_dh=0,down=0;
        if(have_luna&&have_att){
          const cv::Matx33d W_R_B=attitudeFluToNwu(att);
          const cv::Vec3d beam_w=W_R_B*cv::Vec3d(0,0,-1);
          down=-beam_w[2];
          h_current=luna_m*down-offset_m;
          lever_dh=(W_R_B*t_lc_b)[2];
          h_lever=h_current+lever_dh;
        }
        const bool sensors_ok=have_luna&&have_att&&down>0.20&&h_current>0.05&&h_lever>0.05&&luna_age_ms<200;

        if(state==State::MOVING&&sensors_ok&&!prev.empty()&&prev_att.valid&&prev_h_current>0&&prev_h_lever>0){
          std::vector<cv::Point2f>p0,p1;cv::goodFeaturesToTrack(prev,p0,700,0.01,7);
          if(p0.size()>=30){
            std::vector<uchar>st;std::vector<float>err;cv::calcOpticalFlowPyrLK(prev,gray,p0,p1,st,err,{21,21},3);
            std::vector<cv::Point2f>a,bp;for(size_t i=0;i<p0.size();++i)if(st[i]){a.push_back(p0[i]);bp.push_back(p1[i]);}
            if(a.size()>=20){
              cv::Mat mask;cv::findHomography(a,bp,cv::RANSAC,2.0,mask);
              if(!mask.empty()){
                std::vector<cv::Point2f>ai,bi;for(size_t i=0;i<a.size();++i)if(mask.at<uchar>((int)i)){ai.push_back(a[i]);bi.push_back(bp[i]);}
                const double dt=prev_frame_ts?((frame_ts-prev_frame_ts)*1e-9):0;
                if(ai.size()>=15&&dt>0&&dt<0.2){
                  cv::Vec2d dc,dl;int uc=0,ul=0;double sc=0,sl=0;
                  if(v3Step(ai,bi,calib,prev_att,prev_h_current,att,h_current,&dc,&uc,&sc)&&cv::norm(dc)<0.20){current_est.x+=dc[0];current_est.y+=dc[1];current_est.path+=cv::norm(dc);current_est.inliers=uc;current_est.scatter=sc;++current_est.frames;}
                  if(v3Step(ai,bi,calib,prev_att,prev_h_lever,att,h_lever,&dl,&ul,&sl)&&cv::norm(dl)<0.20){lever_est.x+=dl[0];lever_est.y+=dl[1];lever_est.path+=cv::norm(dl);lever_est.inliers=ul;lever_est.scatter=sl;++lever_est.frames;}
                }
              }
            }
          }
        }

        if(sensors_ok){prev=gray.clone();prev_att=att;prev_h_current=h_current;prev_h_lever=h_lever;prev_frame_ts=frame_ts;}
        else{prev.release();prev_att.valid=false;prev_h_current=prev_h_lever=0;prev_frame_ts=0;}

        cv::Mat bgr,video;cv::cvtColor(gray,bgr,cv::COLOR_GRAY2BGR);cv::resize(bgr,video,{870,653});
        cv::Mat canvas(720,1280,CV_8UC3,cv::Scalar(12,12,12));video.copyTo(canvas(cv::Rect(0,67,870,653)));
        ru(canvas,"JT-ZERO — V3 CURRENT vs CAD LEVER",{22,35},17,{245,245,245},cv::QT_FONT_BOLD);cv::Mat panel=canvas(cv::Rect(870,0,410,720));
        ru(panel,sensors_ok?"СИСТЕМА ГОТОВА":"ЖДИТЕ ДАННЫЕ ДАТЧИКОВ",{18,42},13,sensors_ok?cv::Scalar(90,220,90):cv::Scalar(0,210,255),cv::QT_FONT_BOLD);
        const char* now_text=state==State::READY?"СЕЙЧАС: ТОЧКА A — НЕ ДВИГАТЬ":state==State::MOVING?"СЕЙЧАС: ДВИЖЕНИЕ A -> B":"СЕЙЧАС: РЕЗУЛЬТАТ ЗАФИКСИРОВАН";ru(panel,now_text,{18,82},10,{255,255,255},cv::QT_FONT_BOLD);
        char z[256];snprintf(z,sizeof(z),"Pitch/Roll: %+.2f / %+.2f°",att.pitch*180/kPi,att.roll*180/kPi);ru(panel,z,{18,125},10,{220,220,220});
        snprintf(z,sizeof(z),"H Luna / H cam: %.1f / %.1f мм",h_current*1000,h_lever*1000);ru(panel,z,{18,160},10,{220,220,220});
        snprintf(z,sizeof(z),"Lever dH: %+.2f мм",lever_dh*1000);ru(panel,z,{18,195},10,{220,220,220});
        snprintf(z,sizeof(z),"CURRENT NET/PATH: %.1f / %.1f мм",std::hypot(current_est.x,current_est.y)*1000,current_est.path*1000);ru(panel,z,{18,260},12,{245,245,245},cv::QT_FONT_BOLD);
        snprintf(z,sizeof(z),"LEVER NET/PATH: %.1f / %.1f мм",std::hypot(lever_est.x,lever_est.y)*1000,lever_est.path*1000);ru(panel,z,{18,320},12,{245,245,245},cv::QT_FONT_BOLD);
        snprintf(z,sizeof(z),"LEVER-CURRENT NET: %+.1f мм",(std::hypot(lever_est.x,lever_est.y)-std::hypot(current_est.x,current_est.y))*1000);ru(panel,z,{18,370},11,{230,230,230},cv::QT_FONT_BOLD);
        snprintf(z,sizeof(z),"Точек current/lever: %d / %d",current_est.inliers,lever_est.inliers);ru(panel,z,{18,430},10,{220,220,220});
        snprintf(z,sizeof(z),"Разброс: %.2f / %.2f мм",current_est.scatter*1000,lever_est.scatter*1000);ru(panel,z,{18,465},10,{220,220,220});
        snprintf(z,sizeof(z),"CAD tLC: [%.2f %.2f %.2f] мм",t_lc_b[0]*1000,t_lc_b[1]*1000,t_lc_b[2]*1000);ru(panel,z,{18,515},9,{220,220,220});
        if(state==State::DONE){snprintf(z,sizeof(z),"ИТОГ CURRENT/LEVER: %.1f / %.1f мм",std::hypot(rcx,rcy)*1000,std::hypot(rlx,rly)*1000);ru(panel,z,{18,610},11,{245,245,245},cv::QT_FONT_BOLD);}
        ru(panel,state==State::READY?"ПРОБЕЛ — СТАРТ":state==State::MOVING?"В B: ПРОБЕЛ — СТОП":"Q / ESC — ВЫХОД",{18,675},11,{210,210,210},cv::QT_FONT_BOLD);
        cv::imshow(window,canvas);int rk=cv::waitKeyEx(10),key=rk<0?-1:(rk&0xff);
        if(cv::getWindowProperty(window,cv::WND_PROP_VISIBLE)<1){g_running=false;break;}
        if(key==' '&&sensors_ok){
          if(state==State::READY){current_est={};lever_est={};prev.release();prev_att.valid=false;prev_h_current=prev_h_lever=0;prev_frame_ts=0;state=State::MOVING;}
          else if(state==State::MOVING){rcx=current_est.x;rcy=current_est.y;rlx=lever_est.x;rly=lever_est.y;state=State::DONE;}
        }
        if(key=='q'||key=='Q'||key==27){g_running=false;break;}
        if(csv){csv<<(int)state<<','<<now<<','<<frame_ts<<','<<frame_id<<','<<std::fixed<<std::setprecision(6)<<camera_age_ms<<','<<sync_nearest_ms<<','<<sync_bracket_ms<<','<<luna_age_ms<<','<<luna_m<<','<<h_current<<','<<lever_dh<<','<<h_lever<<','<<current_est.x<<','<<current_est.y<<','<<current_est.path<<','<<lever_est.x<<','<<lever_est.y<<','<<lever_est.path<<','<<current_est.inliers<<','<<lever_est.inliers<<','<<current_est.scatter<<','<<lever_est.scatter<<','<<att.roll<<','<<att.pitch<<','<<att.yaw<<'\n';if((frame_id&3u)==0u)csv.flush();}
      }
    }
    cv::destroyAllWindows();std::cout<<"CSV: "<<csvpath<<"\n";return 0;
  }catch(const std::exception&e){std::cerr<<"GROUND MOTION V3 LEVER AB FAIL: "<<e.what()<<"\n";return 1;}
}
