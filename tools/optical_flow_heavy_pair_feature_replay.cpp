// JT-Zero — deterministic heavy-pair replay across feature caps.
// Finds real frame pairs with the largest image displacement on the SAME recorded dataset,
// then replays exactly those pairs through production estimateRawFlow() for caps 500..150.

#define JTZERO_OPTFLOW_LIBRARY
#include "optical_flow_mavlink_mvp_v2.cpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <tuple>

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
struct PairDiag{
  size_t ia=0,ib=0;
  double dt=0;
  double disp=0;
  int valid=0;
  int tracked=0,inliers=0;
};
struct Result{
  int cap=0; int valid=0; int reason=0; int tracked=0,inliers=0; double ratio=0;
  double lk=0,feat=0,ran=0,post=0,du=0,dv=0,flow=0;
};
}

int main(int argc,char**argv){
  if(argc<5){
    std::cerr<<"Использование: "<<argv[0]<<" <dataset_dir> <camera_yaml> <focal_scale> <topN>\n";
    return 2;
  }
  const std::filesystem::path dir=argv[1];
  const std::string yaml=argv[2];
  const double scale=std::stod(argv[3]);
  const int topN=std::stoi(argv[4]);
  try{
    auto meta=readMeta(dir/"frames.csv");
    if(meta.size()<20)throw std::runtime_error("слишком мало кадров");

    CameraCalib calib=loadCameraCalib(yaml);
    calib.fx*=scale;calib.fy*=scale;
    calib.K=(cv::Mat_<double>(3,3)<<calib.fx,0,calib.cx,0,calib.fy,calib.cy,0,0,1);
    g_feature_roi={0.20,0.32,0.80,0.90};

    // Search candidate pairs on cap=500 using several realistic temporal baselines.
    const std::vector<int64_t> targets_ns={
      40'000'000LL,60'000'000LL,80'000'000LL,100'000'000LL,120'000'000LL
    };

    std::ifstream bin(dir/"frames.mjpgbin",std::ios::binary);
    if(!bin)throw std::runtime_error("не удалось открыть frames.mjpgbin");

    std::vector<PairDiag> cand;
    g_max_features=500;

    std::cerr<<"Поиск тяжёлых пар: 0/"<<meta.size()<<"\r"<<std::flush;
    for(size_t ia=0;ia<meta.size();++ia){
      if(ia%200==0){
        std::cerr<<"Поиск тяжёлых пар: "<<ia<<"/"<<meta.size()
                 <<" candidates="<<cand.size()<<"\r"<<std::flush;
      }
      cv::Mat a=load(bin,meta[ia]); if(a.empty())continue;
      for(int64_t target:targets_ns){
        const int64_t want=meta[ia].ts+target;
        auto it=std::lower_bound(meta.begin()+ia+1,meta.end(),want,
          [](const Meta&m,int64_t t){return m.ts<t;});
        if(it==meta.end())continue;
        const size_t ib=(size_t)std::distance(meta.begin(),it);
        const double dt=(meta[ib].ts-meta[ia].ts)*1e-9;
        if(!(dt>0.02&&dt<0.15))continue;
        cv::Mat b=load(bin,meta[ib]); if(b.empty())continue;
        auto s=estimateRawFlow(a,b,dt,calib);
        const double disp=s.valid?std::hypot(s.du_px,s.dv_px):0.0;
        cand.push_back({ia,ib,dt,disp,s.valid?1:0,s.tracked,s.inliers});
      }
    }
    std::cerr<<"Поиск тяжёлых пар: "<<meta.size()<<"/"<<meta.size()
             <<" candidates="<<cand.size()<<"\n";

    // Deduplicate same pair and sort by measured displacement first, then prefer valid baseline.
    std::sort(cand.begin(),cand.end(),[](const PairDiag&x,const PairDiag&y){
      if(x.ia!=y.ia)return x.ia<y.ia;
      return x.ib<y.ib;
    });
    cand.erase(std::unique(cand.begin(),cand.end(),[](const PairDiag&x,const PairDiag&y){
      return x.ia==y.ia&&x.ib==y.ib;
    }),cand.end());
    std::sort(cand.begin(),cand.end(),[](const PairDiag&x,const PairDiag&y){
      if(x.disp!=y.disp)return x.disp>y.disp;
      return x.dt>y.dt;
    });

    if((int)cand.size()>topN)cand.resize((size_t)topN);

    std::cout<<"===== JT-ZERO — HEAVY REAL-PAIR FEATURE CAP REPLAY =====\n";
    std::cout<<"dataset="<<dir<<" candidates_selected="<<cand.size()<<"\n";
    std::cout<<"Пары выбраны по максимальному measured pixel displacement при cap=500.\n";
    std::cout<<"Затем ВСЕ caps прогоняются на ТОЧНО тех же парах.\n\n";

    const std::vector<int> caps={500,400,300,250,200,150};
    std::map<int,std::vector<Result>> all;

    std::cout<<std::fixed<<std::setprecision(3);
    for(size_t p=0;p<cand.size();++p){
      const auto &q=cand[p];
      std::cout<<"PAIR "<<p
               <<" frames="<<meta[q.ia].frame<<"->"<<meta[q.ib].frame
               <<" dt="<<q.dt*1000.0<<"ms"
               <<" base_disp="<<q.disp<<"px"
               <<" base_trk/inl="<<q.tracked<<"/"<<q.inliers<<"\n";

      for(int cap:caps){
        g_max_features=cap;
        cv::Mat a=load(bin,meta[q.ia]);
        cv::Mat b=load(bin,meta[q.ib]);
        auto s=estimateRawFlow(a,b,q.dt,calib);
        Result r;
        r.cap=cap;r.valid=s.valid?1:0;r.reason=s.invalid_reason;r.tracked=s.tracked;r.inliers=s.inliers;
        r.ratio=s.inlier_ratio;r.lk=s.t_lk_ms;r.feat=s.t_features_ms;r.ran=s.t_ransac_ms;r.post=s.t_post_ms;
        r.du=s.du_px;r.dv=s.dv_px;r.flow=std::hypot(s.flow_body_x,s.flow_body_y);
        all[cap].push_back(r);
        std::cout<<"  cap="<<std::setw(3)<<cap
                 <<" valid="<<r.valid<<" reason="<<r.reason
                 <<" trk/inl="<<r.tracked<<"/"<<r.inliers
                 <<" ratio="<<r.ratio
                 <<" LK="<<r.lk<<"ms"
                 <<" disp="<<std::hypot(r.du,r.dv)<<"px"
                 <<" flow="<<r.flow<<"\n";
      }
      std::cout<<"\n";
    }

    std::cout<<"===== СВОДКА =====\n";
    std::cout<<"cap  valid  mean_LK_ms  mean_abs_disp_diff_vs500_px  mean_flow_diff_vs500\n";
    for(int cap:caps){
      const auto &v=all[cap];
      int ok=0; double slk=0,sd=0,sf=0; int nd=0;
      for(size_t k=0;k<v.size();++k){
        if(v[k].valid)++ok; slk+=v[k].lk;
        const auto&b=all[500][k];
        if(v[k].valid&&b.valid){
          sd+=std::hypot(v[k].du-b.du,v[k].dv-b.dv);
          sf+=std::abs(v[k].flow-b.flow);
          ++nd;
        }
      }
      std::cout<<cap<<"  "<<ok<<"/"<<v.size()
               <<"  "<<(v.empty()?NAN:slk/v.size())
               <<"  "<<(nd?sd/nd:NAN)
               <<"  "<<(nd?sf/nd:NAN)<<"\n";
    }

    std::cout<<"\nИНТЕРПРЕТАЦИЯ:\n";
    std::cout<<"  Кандидат cap можно считать перспективным только если на ЭТИХ ЖЕ тяжёлых парах\n";
    std::cout<<"  valid-rate не хуже 500 и измеренный displacement/flow почти не меняется,\n";
    std::cout<<"  при этом LK runtime заметно падает.\n";
    std::cout<<"  Это всё ещё offline replay: feedback live cadence здесь не моделируется.\n";
    return 0;

  }catch(const std::exception&e){
    std::cerr<<"ОШИБКА: "<<e.what()<<"\n";
    return 1;
  }
}
