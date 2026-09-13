// JT-Zero — deterministic feature-count sweep on the SAME real OV9281 frame.
// No FC/Luna required. Captures one frame, synthesizes controlled translations,
// and runs the exact production estimateRawFlow() with feature caps
// 500/400/300/200/150 on identical image pairs.

#define JTZERO_OPTFLOW_LIBRARY
#include "optical_flow_mavlink_mvp_v2.cpp"

#include <algorithm>
#include <iomanip>
#include <numeric>

namespace {

struct CaseResult {
  int cap=0;
  double dx=0,dy=0;
  int valid=0;
  int features=0,tracked=0,inliers=0;
  double inlier_ratio=0;
  double lk_med=0,feat_med=0,ransac_med=0,post_med=0;
  double du_px=0,dv_px=0;
  double err_px=0;
};

double med(std::vector<double> v){
  if(v.empty()) return NAN;
  const size_t n=v.size();
  std::nth_element(v.begin(),v.begin()+n/2,v.end());
  double m=v[n/2];
  if(n%2==0){
    std::nth_element(v.begin(),v.begin()+n/2-1,v.end());
    m=0.5*(m+v[n/2-1]);
  }
  return m;
}

cv::Mat shiftImage(const cv::Mat& src,double dx,double dy){
  cv::Mat dst;
  cv::Matx23d M(1,0,dx, 0,1,dy);
  cv::warpAffine(src,dst,M,src.size(),cv::INTER_LINEAR,cv::BORDER_REFLECT101);
  return dst;
}

CaseResult runCase(const cv::Mat& base,const cv::Mat& moved,const CameraCalib& calib,
                   int cap,double dx,double dy,int repeats){
  g_max_features=cap;
  std::vector<double> tlk,tfeat,trans,tpost;
  FlowStep last;
  for(int k=0;k<repeats;k++){
    auto s=estimateRawFlow(base,moved,0.060,calib);
    tlk.push_back(s.t_lk_ms);
    tfeat.push_back(s.t_features_ms);
    trans.push_back(s.t_ransac_ms);
    tpost.push_back(s.t_post_ms);
    last=s;
  }
  CaseResult r;
  r.cap=cap;r.dx=dx;r.dy=dy;r.valid=last.valid?1:0;
  r.features=last.features;r.tracked=last.tracked;r.inliers=last.inliers;
  r.inlier_ratio=last.inlier_ratio;
  r.lk_med=med(tlk);r.feat_med=med(tfeat);r.ransac_med=med(trans);r.post_med=med(tpost);
  r.du_px=last.du_px;r.dv_px=last.dv_px;
  r.err_px=std::hypot(r.du_px-dx,r.dv_px-dy);
  return r;
}

}

int main(int argc,char**argv){
  if(argc<4){
    std::cerr<<"Использование: "<<argv[0]<<" <camera> <camera_yaml> <focal_scale>\n";
    return 2;
  }
  const std::string camdev=argv[1], yaml=argv[2];
  const double focal_scale=std::stod(argv[3]);
  try{
    CameraCalib calib=loadCameraCalib(yaml);
    calib.fx*=focal_scale; calib.fy*=focal_scale;
    calib.K=(cv::Mat_<double>(3,3)<<calib.fx,0,calib.cx,0,calib.fy,calib.cy,0,0,1);

    // Current production ROI.
    g_feature_roi={0.20,0.32,0.80,0.90};

    Camera cam; cam.openDev(camdev);
    cv::Mat gray;
    std::vector<uint8_t> latest;
    int64_t ts=0;
    for(int tries=0;tries<100 && gray.empty();++tries){
      pollfd p{cam.fd,POLLIN,0};
      if(poll(&p,1,100)<=0) continue;
      while(true){
        v4l2_buffer b{}; b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory=V4L2_MEMORY_MMAP;
        if(xioctl(cam.fd,VIDIOC_DQBUF,&b)<0){
          if(errno==EAGAIN) break;
          fail("VIDIOC_DQBUF");
        }
        latest.assign((uint8_t*)cam.bufs[b.index].p,(uint8_t*)cam.bufs[b.index].p+b.bytesused);
        ts=(int64_t)b.timestamp.tv_sec*1000000000LL+(int64_t)b.timestamp.tv_usec*1000LL;
        if(xioctl(cam.fd,VIDIOC_QBUF,&b)<0) fail("VIDIOC_QBUF");
      }
      if(!latest.empty()){
        cv::Mat one(1,(int)latest.size(),CV_8UC1,latest.data());
        gray=cv::imdecode(one,cv::IMREAD_GRAYSCALE);
      }
    }
    if(gray.empty()) throw std::runtime_error("не удалось получить кадр OV9281");

    const std::vector<int> caps={500,400,300,200,150};
    const std::vector<std::pair<double,double>> shifts={
      {5,0},{10,0},{20,0},{30,0},{40,0},{50,0},{60,0},
      {20,10},{40,20},{50,25}
    };
    constexpr int repeats=7;

    std::cout<<"===== JT-ZERO — ДЕТЕРМИНИРОВАННЫЙ SWEEP ЧИСЛА FEATURES =====\n";
    std::cout<<"Кадр: "<<gray.cols<<"x"<<gray.rows
             <<"  fx/fy="<<calib.fx<<"/"<<calib.fy
             <<"  ROI=0.20 0.32 0.80 0.90\n";
    std::cout<<"Один и тот же реальный кадр. Движение создаётся программно.\n";
    std::cout<<"Каждый вариант прогоняется "<<repeats<<" раз; времена — медиана.\n\n";

    std::cout<<" shift(px) cap valid feat trk inl ratio  LK_ms  FEAT_ms RANSAC_ms  err_px  measured_du/dv\n";
    std::cout<<std::fixed<<std::setprecision(3);

    std::vector<CaseResult> all;
    for(const auto& sh:shifts){
      cv::Mat moved=shiftImage(gray,sh.first,sh.second);
      for(int cap:caps){
        auto r=runCase(gray,moved,calib,cap,sh.first,sh.second,repeats);
        all.push_back(r);
        std::cout<<std::setw(3)<<(int)sh.first<<","<<std::setw(3)<<(int)sh.second
                 <<"  "<<std::setw(3)<<cap
                 <<"   "<<r.valid
                 <<"  "<<std::setw(4)<<r.features
                 <<" "<<std::setw(4)<<r.tracked
                 <<" "<<std::setw(4)<<r.inliers
                 <<" "<<std::setw(5)<<r.inlier_ratio
                 <<"  "<<std::setw(6)<<r.lk_med
                 <<"  "<<std::setw(7)<<r.feat_med
                 <<"  "<<std::setw(8)<<r.ransac_med
                 <<"  "<<std::setw(6)<<r.err_px
                 <<"  ("<<r.du_px<<","<<r.dv_px<<")\n";
      }
      std::cout<<"\n";
    }

    std::cout<<"===== СВОДКА ПО CAP =====\n";
    std::cout<<"cap  valid_cases  LK_med_all  err_med_px  max_err_px  median_inliers\n";
    for(int cap:caps){
      std::vector<double> lk,er,ins; int ok=0,total=0;
      for(const auto&r:all) if(r.cap==cap){
        ++total;if(r.valid)++ok;lk.push_back(r.lk_med);er.push_back(r.err_px);ins.push_back(r.inliers);
      }
      std::cout<<cap<<"      "<<ok<<"/"<<total
               <<"       "<<med(lk)
               <<"       "<<med(er)
               <<"       "<<*std::max_element(er.begin(),er.end())
               <<"       "<<med(ins)<<"\n";
    }

    std::cout<<"\nКРИТЕРИЙ ДЛЯ СНИЖЕНИЯ CAP:\n";
    std::cout<<"  - все 10 synthetic shifts остаются valid;\n";
    std::cout<<"  - median error не ухудшается существенно относительно cap=500;\n";
    std::cout<<"  - max error остаётся малым;\n";
    std::cout<<"  - LK_ms заметно уменьшается.\n";
    std::cout<<"Этот тест НЕ доказывает поведение на реальном parallax/blur/occlusion;\n";
    std::cout<<"он только даёт контролируемый первый A/B для feature cap.\n";
    return 0;
  }catch(const std::exception&e){
    std::cerr<<"ОШИБКА: "<<e.what()<<"\n";
    return 1;
  }
}
