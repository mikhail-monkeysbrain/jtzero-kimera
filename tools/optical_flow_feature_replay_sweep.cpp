// JT-Zero — deterministic replay of ONE real motion dataset across feature caps.
// Uses exact production estimateRawFlow() on identical selected frame pairs.
#define JTZERO_OPTFLOW_LIBRARY
#include "optical_flow_mavlink_mvp_v2.cpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>

namespace {
struct Meta{uint64_t frame=0,off=0,size=0;int64_t ts=0;};
std::vector<std::string> split(const std::string&s){
  std::vector<std::string>v;std::stringstream ss(s);std::string x;
  while(std::getline(ss,x,','))v.push_back(x);return v;
}
std::vector<Meta> readMeta(const std::filesystem::path&p){
  std::ifstream f(p);if(!f)throw std::runtime_error("не удалось открыть frames.csv");
  std::string line;std::getline(f,line);std::vector<Meta>v;
  while(std::getline(f,line)){
    if(line.empty())continue;auto c=split(line);if(c.size()<4)continue;
    v.push_back({std::stoull(c[0]),std::stoull(c[2]),std::stoull(c[3]),std::stoll(c[1])});
  }
  return v;
}
cv::Mat load(std::ifstream&bin,const Meta&m){
  std::vector<uint8_t>raw((size_t)m.size);
  bin.clear();bin.seekg((std::streamoff)m.off);bin.read((char*)raw.data(),(std::streamsize)raw.size());
  if((size_t)bin.gcount()!=raw.size())throw std::runtime_error("короткое чтение frame");
  cv::Mat one(1,(int)raw.size(),CV_8UC1,raw.data());
  return cv::imdecode(one,cv::IMREAD_GRAYSCALE);
}
double med(std::vector<double>v){
  if(v.empty())return NAN;size_t n=v.size();std::nth_element(v.begin(),v.begin()+n/2,v.end());double x=v[n/2];
  if(n%2==0){std::nth_element(v.begin(),v.begin()+n/2-1,v.end());x=.5*(x+v[n/2-1]);}return x;
}
struct Sum{
  int cap=0,pairs=0,valid=0,reason2=0,reason3=0,reason4=0,reason5=0,reason6=0;
  std::vector<double>lk,feat,ran,post,flow,du,ratio;
};
}

int main(int argc,char**argv){
  if(argc<5){
    std::cerr<<"Использование: "<<argv[0]<<" <dataset_dir> <camera_yaml> <focal_scale> <target_dt_ms>\n";
    return 2;
  }
  const std::filesystem::path dir=argv[1];
  const std::string yaml=argv[2];const double scale=std::stod(argv[3]);const double target_ms=std::stod(argv[4]);
  try{
    auto meta=readMeta(dir/"frames.csv");if(meta.size()<10)throw std::runtime_error("слишком мало кадров");
    CameraCalib calib=loadCameraCalib(yaml);calib.fx*=scale;calib.fy*=scale;
    calib.K=(cv::Mat_<double>(3,3)<<calib.fx,0,calib.cx,0,calib.fy,calib.cy,0,0,1);
    g_feature_roi={0.20,0.32,0.80,0.90};
    const std::vector<int>caps={500,400,300,250,200,150};
    std::map<int,Sum>sums;for(int c:caps)sums[c].cap=c;

    // Deterministic frame selection: choose the first camera frame whose timestamp
    // reaches previous_selected + target_dt. Same indices are reused for every cap.
    std::vector<size_t>sel;sel.push_back(0);
    int64_t want=meta[0].ts+(int64_t)llround(target_ms*1e6);
    for(size_t i=1;i<meta.size();++i){
      if(meta[i].ts>=want){sel.push_back(i);want=meta[i].ts+(int64_t)llround(target_ms*1e6);}
    }
    std::cout<<"===== JT-ZERO — REAL-MOTION FEATURE CAP REPLAY =====\n";
    std::cout<<"dataset="<<dir<<" raw_frames="<<meta.size()<<" selected="<<sel.size()<<" target_dt="<<target_ms<<" ms\n";
    std::cout<<"Одинаковые реальные пары используются для всех feature cap.\n\n";

    for(int cap:caps){
      g_max_features=cap;
      std::ifstream bin(dir/"frames.mjpgbin",std::ios::binary);if(!bin)throw std::runtime_error("не удалось открыть bin");
      cv::Mat prev=load(bin,meta[sel[0]]);int64_t pts=meta[sel[0]].ts;
      for(size_t k=1;k<sel.size();++k){
        cv::Mat cur=load(bin,meta[sel[k]]);const int64_t cts=meta[sel[k]].ts;
        if(prev.empty()||cur.empty()){prev=cur;pts=cts;continue;}
        double dt=(cts-pts)*1e-9;
        auto s=estimateRawFlow(prev,cur,dt,calib);
        auto&z=sums[cap];++z.pairs;if(s.valid)++z.valid;
        if(s.invalid_reason==2)++z.reason2;if(s.invalid_reason==3)++z.reason3;
        if(s.invalid_reason==4)++z.reason4;if(s.invalid_reason==5)++z.reason5;if(s.invalid_reason==6)++z.reason6;
        z.lk.push_back(s.t_lk_ms);z.feat.push_back(s.t_features_ms);z.ran.push_back(s.t_ransac_ms);z.post.push_back(s.t_post_ms);
        if(s.valid){z.flow.push_back(std::hypot(s.flow_body_x,s.flow_body_y));z.du.push_back(std::hypot(s.du_px,s.dv_px));z.ratio.push_back(s.inlier_ratio);}
        prev=cur;pts=cts;
      }
    }

    const auto&base=sums[500];
    std::cout<<"cap pairs valid% invalid[r2/r3/r4/r5/r6] LK_med FEAT_med RANSAC_med pix_med flow_med ratio_med\n";
    std::cout<<std::fixed<<std::setprecision(3);
    for(int cap:caps){
      const auto&z=sums[cap];
      std::cout<<std::setw(3)<<cap<<" "<<z.pairs<<" "<<(100.0*z.valid/std::max(1,z.pairs))
               <<" ["<<z.reason2<<"/"<<z.reason3<<"/"<<z.reason4<<"/"<<z.reason5<<"/"<<z.reason6<<"] "
               <<med(z.lk)<<" "<<med(z.feat)<<" "<<med(z.ran)<<" "<<med(z.du)<<" "<<med(z.flow)<<" "<<med(z.ratio)<<"\n";
    }

    std::cout<<"\nОГРАНИЧЕНИЕ: replay проверяет frontend на одном реальном движении и одинаковых парах.\n";
    std::cout<<"Он НЕ моделирует feedback 'медленный LK -> следующий кадр позже'. Для этого после выбора cap нужен отдельный live stress-run.\n";
    return 0;
  }catch(const std::exception&e){std::cerr<<"ОШИБКА: "<<e.what()<<"\n";return 1;}
}
