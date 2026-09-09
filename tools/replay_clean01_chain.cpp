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
    ImuRec r;r.ts=std::stoll(v[1]);
    r.ax=std::stod(v[9]);r.ay=std::stod(v[10]);r.az=std::stod(v[11]);
    r.gx=std::stod(v[12]);r.gy=std::stod(v[13]);r.gz=std::stod(v[14]);out.push_back(r);
  }
  return out;
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
    std::ifstream mj(run+"/selected.mjpg",std::ios::binary);if(!mj)throw std::runtime_error("cannot open selected.mjpg");

    setenv("JTZERO_DIAG_CHAIN_CSV","1",1);
    unsetenv("JTZERO_USE_STATE_OUTPUT");

    VIO::VioParams vp(params);
    ReplayPipeline pipe(vp);pipe.install();
    std::thread th([&](){pipe.spin();});

    size_t ii=0;VIO::FrameId fid=0;
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

    std::cout<<"CLEAN01_REPLAY frames="<<frames.size()<<" imu="<<imu.size()<<" backend="<<pipe.out_.size()<<"\n";
    std::cout<<"CHAIN_CSV=/home/vio/jtzero_kimera_chain.csv\n";
    std::cout<<"REPLAY_OUTPUT="<<run<<"/replay_output.csv\n";
    return pipe.out_.empty()?1:0;
  }catch(const std::exception&e){std::cerr<<"ERROR: "<<e.what()<<"\n";return 2;}
}
