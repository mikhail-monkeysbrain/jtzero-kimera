#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/imu-frontend/ImuFrontend-definitions.h"
#include "kimera-vio/pipeline/MonoImuPipeline.h"
#include "kimera-vio/pipeline/Pipeline-definitions.h"

DECLARE_bool(visualize);
DECLARE_int32(viz_type);
DECLARE_bool(use_lcd);
DECLARE_bool(log_output);
DECLARE_bool(extract_planes_from_the_scene);

namespace {

struct FrameRec {
  uint32_t sequence=0;
  int64_t ts=0;
  uint64_t off=0;
  uint32_t bytes=0;
};
struct ImuRec {
  int64_t ts=0;
  uint64_t fc_us=0;
  double ax=0,ay=0,az=0,gx=0,gy=0,gz=0;
};
struct OutState {
  int64_t ts=0,kf=0;
  double px=0,py=0,pz=0,vx=0,vy=0,vz=0;
};

std::vector<std::string> split(const std::string&s,char sep){
  std::vector<std::string>v;std::stringstream ss(s);std::string x;
  while(std::getline(ss,x,sep))v.push_back(x);return v;
}

std::vector<FrameRec> loadFrames(const std::string& path){
  std::ifstream f(path);if(!f)throw std::runtime_error("cannot open "+path);
  std::string line;std::getline(f,line);std::vector<FrameRec> out;
  while(std::getline(f,line)){
    if(line.empty())continue;auto v=split(line,',');if(v.size()<4)continue;
    FrameRec r;r.sequence=std::stoul(v[0]);r.ts=std::stoll(v[1]);r.off=std::stoull(v[2]);r.bytes=std::stoul(v[3]);out.push_back(r);
  }
  return out;
}
std::vector<ImuRec> loadImu(const std::string& path){
  std::ifstream f(path);if(!f)throw std::runtime_error("cannot open "+path);
  std::string line;std::getline(f,line);std::vector<ImuRec> out;
  while(std::getline(f,line)){
    if(line.empty())continue;auto v=split(line,',');if(v.size()<15)continue;
    ImuRec r;r.ts=std::stoll(v[1]);r.fc_us=std::stoull(v[2]);
    r.ax=std::stod(v[9]);r.ay=std::stod(v[10]);r.az=std::stod(v[11]);
    r.gx=std::stod(v[12]);r.gy=std::stod(v[13]);r.gz=std::stod(v[14]);out.push_back(r);
  }
  return out;
}


constexpr double kStartupStaticSec = 3.0;
constexpr size_t kStartupMinSamples = 450;
constexpr double kStartupMaxMeanGyroRadS = 0.010;
constexpr double kStartupMaxGyroStdRadS = 0.005;
constexpr double kStartupMinAccelNorm = 9.60;
constexpr double kStartupMaxAccelNorm = 10.00;
constexpr double kStartupMaxAccelNormStd = 0.080;

struct StartupStaticGate {
  uint64_t first_us=0,last_us=0;
  size_t n=0;
  double sgx=0,sgy=0,sgz=0,sgx2=0,sgy2=0,sgz2=0;
  double san=0,san2=0;

  void reset(){
    first_us=last_us=0;n=0;
    sgx=sgy=sgz=sgx2=sgy2=sgz2=0;
    san=san2=0;
  }

  bool add(const ImuRec& r){
    if(first_us==0)first_us=r.fc_us;
    last_us=r.fc_us;++n;
    sgx+=r.gx;sgy+=r.gy;sgz+=r.gz;
    sgx2+=r.gx*r.gx;sgy2+=r.gy*r.gy;sgz2+=r.gz*r.gz;
    const double an=std::sqrt(r.ax*r.ax+r.ay*r.ay+r.az*r.az);
    san+=an;san2+=an*an;

    if(n<kStartupMinSamples)return false;
    const double elapsed=(last_us>first_us)?(last_us-first_us)*1e-6:0.0;
    if(elapsed<kStartupStaticSec)return false;

    const double dn=static_cast<double>(n);
    const double mgx=sgx/dn,mgy=sgy/dn,mgz=sgz/dn;
    const double mean_g=std::sqrt(mgx*mgx+mgy*mgy+mgz*mgz);
    const double sx=std::sqrt(std::max(0.0,sgx2/dn-mgx*mgx));
    const double sy=std::sqrt(std::max(0.0,sgy2/dn-mgy*mgy));
    const double sz=std::sqrt(std::max(0.0,sgz2/dn-mgz*mgz));
    const double man=san/dn;
    const double sanstd=std::sqrt(std::max(0.0,san2/dn-man*man));

    const bool ok=
      mean_g<=kStartupMaxMeanGyroRadS &&
      std::max({sx,sy,sz})<=kStartupMaxGyroStdRadS &&
      man>=kStartupMinAccelNorm &&
      man<=kStartupMaxAccelNorm &&
      sanstd<=kStartupMaxAccelNormStd;

    if(ok)return true;
    reset();
    return false;
  }
};

size_t findLiveFeedStart(const std::vector<ImuRec>& imu){
  StartupStaticGate gate;
  for(size_t i=0;i<imu.size();++i){
    if(gate.add(imu[i]))return i;
  }
  throw std::runtime_error("replay could not reproduce CLEAN01 static gate pass");
}

class ReplayPipeline final:public VIO::MonoImuPipeline{
 public:
  explicit ReplayPipeline(const VIO::VioParams&p):VIO::MonoImuPipeline(p){}
  void install(){
    registerBackendOutputCallback([this](const std::shared_ptr<VIO::BackendOutput>&o){
      if(!o)return;const auto&s=o->W_State_Blkf_;const auto p=s.pose_.translation();const auto&v=s.velocity_;
      OutState x;x.ts=s.timestamp_;x.kf=o->cur_kf_id_;x.px=p.x();x.py=p.y();x.pz=p.z();x.vx=v.x();x.vy=v.y();x.vz=v.z();
      out_.push_back(x);
    });
  }
  std::vector<OutState> out_;
};

} // namespace

int main(int argc,char**argv){
  if(argc<3){std::cerr<<"usage: "<<argv[0]<<" <run_dir> <params_dir>\n";return 2;}
  const std::string run=argv[1],params=argv[2];
  google::InitGoogleLogging(argv[0]);
  FLAGS_visualize=false;FLAGS_viz_type=2;FLAGS_use_lcd=false;FLAGS_log_output=false;FLAGS_extract_planes_from_the_scene=false;

  try{
    auto frames=loadFrames(run+"/selected_frames.csv");
    auto imu=loadImu(run+"/imu.csv");
    if(frames.empty()||imu.empty())throw std::runtime_error("empty replay input");
    const size_t feed_start=findLiveFeedStart(imu);
    const int64_t feed_start_ts=imu[feed_start].ts;
    std::ifstream mj(run+"/selected.mjpg",std::ios::binary);if(!mj)throw std::runtime_error("cannot open selected.mjpg");

    setenv("JTZERO_DIAG_CHAIN_CSV","1",1);
    unsetenv("JTZERO_USE_STATE_OUTPUT");

    VIO::VioParams vp(params);
    ReplayPipeline pipe(vp);pipe.install();
    std::thread th([&](){pipe.spin();});

    size_t ii=feed_start;VIO::FrameId fid=0;
    for(const auto&fr:frames){
      while(ii<imu.size()&&imu[ii].ts<=fr.ts){
        VIO::ImuAccGyr d;d<<imu[ii].ax,imu[ii].ay,imu[ii].az,imu[ii].gx,imu[ii].gy,imu[ii].gz;
        pipe.fillSingleImuQueue(VIO::ImuMeasurement(imu[ii].ts,d));++ii;
      }
      mj.seekg(static_cast<std::streamoff>(fr.off));
      std::vector<unsigned char>b(fr.bytes);mj.read(reinterpret_cast<char*>(b.data()),fr.bytes);
      if(static_cast<uint32_t>(mj.gcount())!=fr.bytes)throw std::runtime_error("short mjpeg read");
      cv::Mat g=cv::imdecode(b,cv::IMREAD_GRAYSCALE);if(g.empty())throw std::runtime_error("jpeg decode failed");
      pipe.fillLeftFrameQueue(std::make_unique<VIO::Frame>(fid++,fr.ts,vp.camera_params_.at(0),g));
    }
    while(ii<imu.size()){
      VIO::ImuAccGyr d;d<<imu[ii].ax,imu[ii].ay,imu[ii].az,imu[ii].gx,imu[ii].gy,imu[ii].gz;
      pipe.fillSingleImuQueue(VIO::ImuMeasurement(imu[ii].ts,d));++ii;
    }

    for(int k=0;k<200;++k){
      if(!pipe.out_.empty()&&pipe.out_.back().ts>=frames.back().ts)break;
      std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    pipe.shutdown();if(th.joinable())th.join();

    std::ofstream fo(run+"/replay_output.csv");
    fo<<"timestamp_ns,keyframe,px,py,pz,vx,vy,vz\n";
    fo.setf(std::ios::fixed);fo.precision(9);
    for(const auto&s:pipe.out_)fo<<s.ts<<','<<s.kf<<','<<s.px<<','<<s.py<<','<<s.pz<<','<<s.vx<<','<<s.vy<<','<<s.vz<<'\n';

    std::cout<<"CLEAN01_REPLAY frames="<<frames.size()
             <<" imu_total="<<imu.size()
             <<" imu_feed_start_index="<<feed_start
             <<" imu_fed="<<(imu.size()-feed_start)
             <<" feed_start_ts="<<feed_start_ts
             <<" first_frame_ts="<<frames.front().ts
             <<" lead_ms="<<(frames.front().ts-feed_start_ts)/1e6
             <<" backend="<<pipe.out_.size()<<"\n";
    std::cout<<"CHAIN_CSV=/home/vio/jtzero_kimera_chain.csv\n";
    std::cout<<"REPLAY_OUTPUT="<<run<<"/replay_output.csv\n";
    return pipe.out_.empty()?1:0;
  }catch(const std::exception&e){std::cerr<<"ERROR: "<<e.what()<<"\n";return 2;}
}
