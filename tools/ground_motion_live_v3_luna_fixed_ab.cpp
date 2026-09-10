// JT-Zero V3 strict A/B: LUNA height vs FIXED camera height.
// Same frames, KLT, homography/RANSAC, SYNC attitude and accepted correspondences.
// LUNA branch uses the current production-like height including the validated CAD lever arm.
// FIXED branch uses one constant camera-center height for the whole measurement.
// Fixed height is captured automatically as the median LUNA camera height from READY samples
// immediately before SPACE; it is NOT tuned to the known 500 mm motion.
#define JTZERO_V3_LEVER_NO_MAIN
#include "ground_motion_live_v3_lever_ab.cpp"
#undef JTZERO_V3_LEVER_NO_MAIN
#include <deque>

namespace {
static double medianLocal(std::deque<double> v){
  if(v.empty()) return 0.0;
  std::vector<double> a(v.begin(),v.end());
  size_t n=a.size()/2;std::nth_element(a.begin(),a.begin()+n,a.end());double m=a[n];
  if(a.size()%2==0){std::nth_element(a.begin(),a.begin()+n-1,a.end());m=0.5*(m+a[n-1]);}
  return m;
}
}

int main(int argc,char**argv){
  if(argc<7){std::cerr<<"Использование: "<<argv[0]<<" <camera> <luna> <fc> <csv> <camera_yaml> <camera_offset_mm>\n";return 2;}
  const std::string camdev=argv[1],lunadev=argv[2],fcdev=argv[3],csvpath=argv[4],yaml=argv[5];const double offset_m=std::stod(argv[6])/1000.0;
  const cv::Vec3d t_lc_b(envMm("JTZERO_LEVER_X_MM",49.16)/1000.0,envMm("JTZERO_LEVER_Y_MM",0.22)/1000.0,envMm("JTZERO_LEVER_Z_MM",0.0)/1000.0);
  try{
    CameraCalib calib=loadCameraCalib(yaml);Camera cam;cam.openDev(camdev);LunaReader luna;luna.start(lunadev);LeverFcHistoryReader fc;fc.start(fcdev);std::ofstream csv(csvpath,std::ios::trunc);
    csv<<"state,now_ns,frame_ts_ns,frame,camera_age_ms,att_sync_nearest_ms,luna_age_ms,luna_slant_m,h_luna_origin_m,lever_dh_m,h_luna_camera_m,h_fixed_camera_m,luna_x_m,luna_y_m,luna_path_m,fixed_x_m,fixed_y_m,fixed_path_m,luna_inliers,fixed_inliers,luna_scatter_m,fixed_scatter_m,roll,pitch,yaw\n";
    cv::setNumThreads(1);std::signal(SIGINT,onSignal);std::signal(SIGTERM,onSignal);const char*window="JT-ZERO — V3 A/B: LUNA vs FIXED H";cv::namedWindow(window,cv::WINDOW_NORMAL);cv::resizeWindow(window,1280,720);cv::moveWindow(window,0,0);
    enum class State{READY=0,MOVING=1,DONE=2};State state=State::READY;cv::Mat prev;Attitude prev_att{};double prev_h_luna=0,prev_h_fixed=0;int64_t prev_frame_ts=0;Estimate luna_est,fixed_est;uint64_t frame_id=0;double fixed_h=0;std::deque<double> ready_h;double rlx=0,rly=0,rfx=0,rfy=0;
    while(g_running){pollfd p{cam.fd,POLLIN,0};int pr=poll(&p,1,20);if(pr<0){if(errno==EINTR)continue;fail("camera poll");}if(pr<=0)continue;while(g_running){
      v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;if(xioctl(cam.fd,VIDIOC_DQBUF,&b)<0){if(errno==EAGAIN)break;fail("VIDIOC_DQBUF");}
      const int64_t now=monoNs(),frame_ts=frameTvToNs(b.timestamp);const double camera_age_ms=(now-frame_ts)*1e-6;cv::Mat raw(1,(int)b.bytesused,CV_8UC1,cam.bufs[b.index].p);cv::Mat gray=cv::imdecode(raw,cv::IMREAD_GRAYSCALE);if(xioctl(cam.fd,VIDIOC_QBUF,&b)<0)fail("VIDIOC_QBUF");if(gray.empty())continue;++frame_id;
      double luna_m=0;int strength=0;int64_t luna_ns=0;Attitude att{};const bool have_luna=luna.latest(&luna_m,&strength,&luna_ns);double sync_bracket_ms=0,sync_nearest_ms=0;const bool have_att=fc.sampleAt(frame_ts,&att,&sync_bracket_ms,&sync_nearest_ms);const double luna_age_ms=have_luna?(now-luna_ns)*1e-6:1e9;
      double h_origin=0,lever_dh=0,h_luna_cam=0,down=0;if(have_luna&&have_att){const cv::Matx33d W_R_B=attitudeFluToNwu(att);down=-(W_R_B*cv::Vec3d(0,0,-1))[2];h_origin=luna_m*down-offset_m;lever_dh=(W_R_B*t_lc_b)[2];h_luna_cam=h_origin+lever_dh;}
      const bool sensors_ok=have_luna&&have_att&&down>0.20&&h_luna_cam>0.05&&luna_age_ms<200;
      if(state==State::READY&&sensors_ok){ready_h.push_back(h_luna_cam);while(ready_h.size()>100)ready_h.pop_front();}
      if(state==State::MOVING&&sensors_ok&&!prev.empty()&&prev_att.valid&&prev_h_luna>0&&prev_h_fixed>0){std::vector<cv::Point2f>p0,p1;cv::goodFeaturesToTrack(prev,p0,700,0.01,7);if(p0.size()>=30){std::vector<uchar>st;std::vector<float>err;cv::calcOpticalFlowPyrLK(prev,gray,p0,p1,st,err,{21,21},3);std::vector<cv::Point2f>a,bp;for(size_t i=0;i<p0.size();++i)if(st[i]){a.push_back(p0[i]);bp.push_back(p1[i]);}if(a.size()>=20){cv::Mat mask;cv::findHomography(a,bp,cv::RANSAC,2.0,mask);if(!mask.empty()){std::vector<cv::Point2f>ai,bi;for(size_t i=0;i<a.size();++i)if(mask.at<uchar>((int)i)){ai.push_back(a[i]);bi.push_back(bp[i]);}const double dt=prev_frame_ts?((frame_ts-prev_frame_ts)*1e-9):0;if(ai.size()>=15&&dt>0&&dt<0.2){cv::Vec2d dl,df;int ul=0,uf=0;double sl=0,sf=0;if(leverV3Step(ai,bi,calib,prev_att,prev_h_luna,att,h_luna_cam,&dl,&ul,&sl)&&cv::norm(dl)<0.20){luna_est.x+=dl[0];luna_est.y+=dl[1];luna_est.path+=cv::norm(dl);luna_est.inliers=ul;luna_est.scatter=sl;++luna_est.frames;}if(leverV3Step(ai,bi,calib,prev_att,prev_h_fixed,att,fixed_h,&df,&uf,&sf)&&cv::norm(df)<0.20){fixed_est.x+=df[0];fixed_est.y+=df[1];fixed_est.path+=cv::norm(df);fixed_est.inliers=uf;fixed_est.scatter=sf;++fixed_est.frames;}}}}}}
      if(sensors_ok){prev=gray.clone();prev_att=att;prev_h_luna=h_luna_cam;prev_h_fixed=(fixed_h>0?fixed_h:h_luna_cam);prev_frame_ts=frame_ts;}else{prev.release();prev_att.valid=false;prev_h_luna=prev_h_fixed=0;prev_frame_ts=0;}
      cv::Mat bgr,video;cv::cvtColor(gray,bgr,cv::COLOR_GRAY2BGR);cv::resize(bgr,video,{870,653});cv::Mat canvas(720,1280,CV_8UC3,cv::Scalar(12,12,12));video.copyTo(canvas(cv::Rect(0,67,870,653)));ru(canvas,"JT-ZERO — V3 LUNA vs FIXED HEIGHT",{22,35},17,{245,245,245},cv::QT_FONT_BOLD);cv::Mat panel=canvas(cv::Rect(870,0,410,720));
      ru(panel,sensors_ok?"СИСТЕМА ГОТОВА":"ЖДИТЕ ДАННЫЕ ДАТЧИКОВ",{18,42},13,sensors_ok?cv::Scalar(90,220,90):cv::Scalar(0,210,255),cv::QT_FONT_BOLD);const char*act=state==State::READY?"СЕЙЧАС: ТОЧКА A — НЕ ДВИГАТЬ":state==State::MOVING?"СЕЙЧАС: ДВИЖЕНИЕ A -> B":"СЕЙЧАС: РЕЗУЛЬТАТ ЗАФИКСИРОВАН";ru(panel,act,{18,82},10,{255,255,255},cv::QT_FONT_BOLD);char z[256];
      snprintf(z,sizeof(z),"Pitch/Roll: %+.2f / %+.2f°",att.pitch*180/kPi,att.roll*180/kPi);ru(panel,z,{18,125},10,{220,220,220});snprintf(z,sizeof(z),"H Luna camera: %.1f мм",h_luna_cam*1000);ru(panel,z,{18,160},10,{220,220,220});snprintf(z,sizeof(z),"H FIXED: %.1f мм",fixed_h*1000);ru(panel,z,{18,195},10,{220,220,220});snprintf(z,sizeof(z),"LUNA NET/PATH: %.1f / %.1f мм",std::hypot(luna_est.x,luna_est.y)*1000,luna_est.path*1000);ru(panel,z,{18,260},12,{245,245,245},cv::QT_FONT_BOLD);snprintf(z,sizeof(z),"FIXED NET/PATH: %.1f / %.1f мм",std::hypot(fixed_est.x,fixed_est.y)*1000,fixed_est.path*1000);ru(panel,z,{18,320},12,{245,245,245},cv::QT_FONT_BOLD);snprintf(z,sizeof(z),"FIXED-LUNA NET: %+.1f мм",(std::hypot(fixed_est.x,fixed_est.y)-std::hypot(luna_est.x,luna_est.y))*1000);ru(panel,z,{18,370},11,{230,230,230},cv::QT_FONT_BOLD);ru(panel,state==State::READY?"ПРОБЕЛ — ЗАФИКСИРОВАТЬ H И СТАРТ":state==State::MOVING?"В B: ПРОБЕЛ — СТОП":"Q / ESC — ВЫХОД",{18,675},10,{210,210,210},cv::QT_FONT_BOLD);
      cv::imshow(window,canvas);int rk=cv::waitKeyEx(10),key=rk<0?-1:(rk&0xff);if(cv::getWindowProperty(window,cv::WND_PROP_VISIBLE)<1){g_running=false;break;}
      if(key==' '&&sensors_ok){if(state==State::READY){if(ready_h.size()<10)continue;fixed_h=medianLocal(ready_h);luna_est={};fixed_est={};prev.release();prev_att.valid=false;prev_h_luna=prev_h_fixed=0;prev_frame_ts=0;state=State::MOVING;std::cout<<"FIXED H camera: "<<fixed_h*1000<<" mm\n";}else if(state==State::MOVING){rlx=luna_est.x;rly=luna_est.y;rfx=fixed_est.x;rfy=fixed_est.y;state=State::DONE;std::cout<<"RESULT LUNA/FIXED NET: "<<std::hypot(rlx,rly)*1000<<" / "<<std::hypot(rfx,rfy)*1000<<" mm\n";}}
      if(key=='q'||key=='Q'||key==27){g_running=false;break;}
      if(csv){csv<<(int)state<<','<<now<<','<<frame_ts<<','<<frame_id<<','<<std::fixed<<std::setprecision(6)<<camera_age_ms<<','<<sync_nearest_ms<<','<<luna_age_ms<<','<<luna_m<<','<<h_origin<<','<<lever_dh<<','<<h_luna_cam<<','<<fixed_h<<','<<luna_est.x<<','<<luna_est.y<<','<<luna_est.path<<','<<fixed_est.x<<','<<fixed_est.y<<','<<fixed_est.path<<','<<luna_est.inliers<<','<<fixed_est.inliers<<','<<luna_est.scatter<<','<<fixed_est.scatter<<','<<att.roll<<','<<att.pitch<<','<<att.yaw<<'\n';if((frame_id&3u)==0u)csv.flush();}
    }}
    cv::destroyAllWindows();std::cout<<"CSV: "<<csvpath<<"\n";return 0;
  }catch(const std::exception&e){std::cerr<<"GROUND MOTION V3 LUNA/FIXED AB FAIL: "<<e.what()<<"\n";return 1;}
}
