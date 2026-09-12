// JT-Zero deterministic synthetic rotation self-test.
// Reuses the exact production estimateRawFlow() implementation.

#define JTZERO_OPTFLOW_LIBRARY
#include "optical_flow_mavlink_mvp_v2.cpp"

namespace {

cv::Mat captureFreshGray(Camera& cam){
  // Let camera settle and then drain everything currently queued.
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  const int64_t deadline=monoNs()+3000000000LL;
  std::vector<uint8_t> latest;
  while(monoNs()<deadline){
    pollfd p{cam.fd,POLLIN,0};
    if(poll(&p,1,100)<=0) continue;
    bool got=false;
    while(true){
      v4l2_buffer b{}; b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory=V4L2_MEMORY_MMAP;
      if(xioctl(cam.fd,VIDIOC_DQBUF,&b)<0){
        if(errno==EAGAIN) break;
        fail("VIDIOC_DQBUF selftest");
      }
      const uint8_t* q=reinterpret_cast<const uint8_t*>(cam.bufs[b.index].p);
      latest.assign(q,q+b.bytesused);
      got=true;
      if(xioctl(cam.fd,VIDIOC_QBUF,&b)<0) fail("VIDIOC_QBUF selftest");
    }
    if(got && !latest.empty()){
      cv::Mat raw(1,(int)latest.size(),CV_8UC1,latest.data());
      cv::Mat g=cv::imdecode(raw,cv::IMREAD_GRAYSCALE);
      if(!g.empty()) return g;
    }
  }
  throw std::runtime_error("Не удалось получить кадр OV9281");
}

cv::Mat undistortToEffectiveK(const cv::Mat& src,const CameraCalib& calib){
  cv::Mat dst;
  cv::undistort(src,dst,calib.K,calib.D,calib.K);
  return dst;
}

cv::Matx33d rodriguesMat(const cv::Vec3d& rv){
  cv::Mat Rm;
  cv::Rodrigues(cv::Mat(rv),Rm);
  cv::Matx33d R;
  for(int r=0;r<3;r++)for(int c=0;c<3;c++)R(r,c)=Rm.at<double>(r,c);
  return R;
}

cv::Mat syntheticRotate(const cv::Mat& base,const CameraCalib& c,const cv::Vec3d& omega_c,double dt){
  const cv::Matx33d R_delta=rodriguesMat(omega_c*dt);
  const cv::Matx33d K(c.fx,0,c.cx, 0,c.fy,c.cy, 0,0,1);
  const cv::Matx33d H=K*R_delta.t()*K.inv();
  cv::Mat out;
  cv::warpPerspective(base,out,cv::Mat(H),base.size(),cv::INTER_LINEAR,cv::BORDER_REFLECT101);
  return out;
}

struct Case { std::string name; cv::Vec3d omega_b; double dt; };

void printCase(const Case& tc,const FlowStep& s,const cv::Vec3d& expected_b){
  const double ex=expected_b[0], ey=expected_b[1];
  const double err=std::hypot(s.flow_body_x-ex,s.flow_body_y-ey);
  const double emag=std::hypot(ex,ey);
  const double rel=emag>1e-9?err/emag:err;
  std::cout<<std::left<<std::setw(20)<<tc.name
           <<" expected=("<<std::showpos<<std::fixed<<std::setprecision(4)<<ex<<","<<ey<<")"
           <<" measured=("<<s.flow_body_x<<","<<s.flow_body_y<<")"
           <<std::noshowpos
           <<" err="<<err
           <<" rel="<<(emag>1e-9?100.0*rel:0.0)<<"%"
           <<" valid="<<(s.valid?1:0)
           <<" inliers="<<s.inliers<<"/"<<s.tracked
           <<" ratio="<<s.inlier_ratio
           <<"
";
}

}

int main(int argc,char** argv){
  if(argc<4){
    std::cerr<<"Использование: "<<argv[0]<<" <camera> <camera_yaml> <focal_scale>\n";
    return 2;
  }
  const std::string camera=argv[1];
  const std::string yaml=argv[2];
  const double focal_scale=std::stod(argv[3]);

  try{
    CameraCalib calib=loadCameraCalib(yaml);
    calib.fx*=focal_scale; calib.fy*=focal_scale;
    calib.K=(cv::Mat_<double>(3,3)<<calib.fx,0,calib.cx,0,calib.fy,calib.cy,0,0,1);

    Camera cam; cam.openDev(camera);
    cv::Mat raw=captureFreshGray(cam);
    cam.close();

    // Synthetic pair is made in the undistorted effective-K image.
    // Use the exact production estimator with D=0 on that pair.
    cv::Mat base=undistortToEffectiveK(raw,calib);
    CameraCalib test=calib;
    test.D=cv::Mat::zeros(calib.D.size(),calib.D.type());

    const cv::Matx33d FLU_TO_FRD(1,0,0, 0,-1,0, 0,0,-1);
    const cv::Matx33d FRD_R_C=FLU_TO_FRD*calib.B_R_C;
    const cv::Matx33d C_R_FRD=FRD_R_C.t();

    g_feature_roi={0.20,0.32,0.80,0.90};

    const double dt=0.10;
    const double d5=5.0*M_PI/180.0/dt;
    const double d10=10.0*M_PI/180.0/dt;
    std::vector<Case> cases{
      {"КРЕН +5°",   {+d5,0,0},dt},
      {"КРЕН -5°",   {-d5,0,0},dt},
      {"ТАНГАЖ +5°", {0,+d5,0},dt},
      {"ТАНГАЖ -5°", {0,-d5,0},dt},
      {"КУРС +10°",  {0,0,+d10},dt},
      {"КУРС -10°",  {0,0,-d10},dt},
    };

    std::cout<<"===== JT-ZERO — СИНТЕТИЧЕСКИЙ ТЕСТ ВРАЩЕНИЯ =====\n";
    std::cout<<"Кадр: "<<raw.cols<<"x"<<raw.rows
             <<"  fx/fy="<<calib.fx<<"/"<<calib.fy
             <<"  focal_scale="<<focal_scale<<"\n";
    std::cout<<"Используется ТОТ ЖЕ estimateRawFlow(), что и production publisher.\n";
    std::cout<<"Синтетические пары строятся без физического движения аппарата.\n\n";

    int fail_count=0;
    for(const auto& tc:cases){
      const cv::Vec3d omega_c=C_R_FRD*tc.omega_b;
      cv::Mat rot=syntheticRotate(base,test,omega_c,tc.dt);
      FlowStep s=estimateRawFlow(base,rot,tc.dt,test);

      // For roll/pitch pure rotation, production flow_body should reproduce body X/Y rates.
      // For yaw, the expected compensated X/Y rate is zero; any residual reflects ROI/estimator bias.
      cv::Vec3d expected=tc.omega_b;
      if(std::abs(tc.omega_b[2])>0 && std::abs(tc.omega_b[0])+std::abs(tc.omega_b[1])<1e-12){
        expected[0]=0; expected[1]=0;
      }
      printCase(tc,s,expected);

      const double err=std::hypot(s.flow_body_x-expected[0],s.flow_body_y-expected[1]);
      const double emag=std::hypot(expected[0],expected[1]);
      if(!s.valid || s.inliers<20 || (emag>0.1 && err/emag>0.15) || (emag<0.1 && err>0.08)) fail_count++;
    }

    std::cout<<"\nКРИТЕРИЙ:\n"
             <<"  roll/pitch: ошибка по вектору <= 15%\n"
             <<"  yaw: паразитный XY flow <= 0.08 рад/с\n";
    if(fail_count==0){
      std::cout<<"\nРЕЗУЛЬТАТ: PASS — оси/знаки/масштаб production KLT для чистого вращения согласованы.\n";
      return 0;
    }
    std::cout<<"\nРЕЗУЛЬТАТ: FAIL — "<<fail_count<<" из "<<cases.size()
             <<" синтетических проверок не прошли.\n";
    return 1;
  }catch(const std::exception& e){
    std::cerr<<"ОШИБКА: "<<e.what()<<"\n";
    return 1;
  }
}
