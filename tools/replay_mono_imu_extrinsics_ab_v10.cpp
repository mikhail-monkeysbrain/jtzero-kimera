// JT-ZERO Stage 11.5 v10 deterministic Camera+IMU replay.
// Replays exactly one recorded dataset through Kimera. Run once with REAL params and once
// with ZERO-lever params; camera bytes, timestamps and IMU samples remain identical.

#define JTZERO_V7_EMBEDDED
#include "live_mono_imu_300mm_repeat_hud_v7.cpp"
#undef JTZERO_V7_EMBEDDED
#undef main

#include <cstdio>
#include <cstdlib>
#include <sstream>

namespace {

constexpr const char* kDefaultCombined = "/home/vio/camera_imu_extrinsics.csv";
constexpr const char* kDefaultCamIndex = "/home/vio/camera_imu_extrinsics_camera.csv";
constexpr const char* kDefaultMjpeg = "/home/vio/camera_imu_extrinsics.mjpg";
constexpr int64_t kReplayCameraPeriodNs = 30000000LL;  // same ~33 Hz selection as v7/v8.
constexpr int64_t kReplayRawMaxDtNs = 20000000LL;

struct ReplayImu {
  int64_t source_ns=0, mapped_ns=0;
  double ax=0,ay=0,az=0,gx=0,gy=0,gz=0;
};
struct ReplayCam {
  uint32_t seq=0;
  int64_t ts_ns=0;
  uint64_t offset=0;
  size_t bytes=0;
  bool valid_raw=true;
};

std::vector<std::string> splitCsv10(const std::string& line) {
  std::vector<std::string> out; std::string cur;
  for(char ch:line){if(ch==','){out.push_back(cur);cur.clear();}else cur.push_back(ch);} out.push_back(cur);
  return out;
}
long long i64v10(const std::string&s){return s.empty()?0:std::stoll(s);}
double dv10(const std::string&s){return s.empty()?0.0:std::stod(s);}

std::vector<ReplayImu> loadImu10(const std::string& path) {
  std::ifstream f(path); if(!f)throw std::runtime_error("Cannot open combined CSV: "+path);
  std::vector<ReplayImu> out; std::string line; std::getline(f,line);
  while(std::getline(f,line)){
    auto c=splitCsv10(line); if(c.size()<14||c[0]!="IMU")continue;
    ReplayImu s; s.source_ns=i64v10(c[2]); s.mapped_ns=i64v10(c[3]);
    s.ax=dv10(c[8]);s.ay=dv10(c[9]);s.az=dv10(c[10]);
    s.gx=dv10(c[11]);s.gy=dv10(c[12]);s.gz=dv10(c[13]);
    if(s.source_ns>0&&s.mapped_ns>0)out.push_back(s);
  }
  if(out.empty())throw std::runtime_error("No IMU rows in combined CSV");
  return out;
}

std::vector<ReplayCam> loadCam10(const std::string& path,size_t*raw_count,size_t*rejected) {
  std::ifstream f(path); if(!f)throw std::runtime_error("Cannot open camera index CSV: "+path);
  std::vector<ReplayCam> out; std::string line; std::getline(f,line);
  bool have=false;uint32_t prev_seq=0;int64_t prev_ts=0,last_selected=0;size_t raw=0,rej=0;
  while(std::getline(f,line)){
    auto c=splitCsv10(line); if(c.size()<7)continue; ++raw;
    ReplayCam s; s.seq=(uint32_t)std::stoul(c[0]);s.ts_ns=i64v10(c[2]);s.offset=(uint64_t)std::stoull(c[5]);s.bytes=(size_t)std::stoull(c[6]);
    bool ok=true;if(have){const int64_t dt=s.ts_ns-prev_ts;ok=s.seq==prev_seq+1U&&dt>0&&dt<=kReplayRawMaxDtNs;if(!ok)++rej;}
    prev_seq=s.seq;prev_ts=s.ts_ns;have=true;
    const bool due=last_selected==0||s.ts_ns-last_selected>=kReplayCameraPeriodNs;
    if(ok&&due&&s.bytes>0){out.push_back(s);last_selected=s.ts_ns;}
  }
  if(raw_count)*raw_count=raw;if(rejected)*rejected=rej;
  if(out.empty())throw std::runtime_error("No selected camera rows");
  return out;
}

void saveStates10(const std::string& path,const std::vector<VioState>& states) {
  std::ofstream f(path,std::ios::trunc);if(!f)throw std::runtime_error("Cannot create output CSV: "+path);
  f<<"keyframe,timestamp_ns,px_m,py_m,pz_m,vx_m_s,vy_m_s,vz_m_s,roll_deg,pitch_deg,yaw_deg\n";
  f<<std::fixed<<std::setprecision(9);
  for(const auto&s:states)f<<s.keyframe<<','<<s.timestamp_ns<<','<<s.px<<','<<s.py<<','<<s.pz<<','<<s.vx<<','<<s.vy<<','<<s.vz<<','<<s.roll_deg<<','<<s.pitch_deg<<','<<s.yaw_deg<<'\n';
}

void summary10(const std::string&tag,const std::string&params,const Eigen::Vector3d&tbc,
               const std::vector<VioState>&s,size_t imu_n,size_t raw,size_t rej,size_t selected,size_t decoded) {
  std::cout<<"\n================ EXTRINSICS A/B REPLAY V10 ================\n";
  std::cout<<"tag: "<<tag<<"\nparams: "<<params<<"\nt_BC mm: ["<<(tbc*1000.0).transpose()<<"]\n"
           <<"IMU rows: "<<imu_n<<"\nraw camera rows: "<<raw<<"\nrejected raw pairs: "<<rej
           <<"\nselected camera rows: "<<selected<<"\ndecoded camera rows: "<<decoded
           <<"\nbackend states: "<<s.size()<<"\n";
  if(s.empty()){std::cout<<"REPLAY RESULT: FAIL (no backend states)\n";return;}
  const auto&p0=s.front();const auto&pn=s.back();double path=0,max_exc=0,max_speed=0,max_roll=0,max_pitch=0,max_yaw=0;
  for(size_t i=0;i<s.size();++i){const auto&q=s[i];double dx=q.px-p0.px,dy=q.py-p0.py,dz=q.pz-p0.pz;max_exc=std::max(max_exc,std::sqrt(dx*dx+dy*dy+dz*dz));max_speed=std::max(max_speed,std::sqrt(q.vx*q.vx+q.vy*q.vy+q.vz*q.vz));max_roll=std::max(max_roll,std::abs(wrapDeg(q.roll_deg-p0.roll_deg)));max_pitch=std::max(max_pitch,std::abs(wrapDeg(q.pitch_deg-p0.pitch_deg)));max_yaw=std::max(max_yaw,std::abs(wrapDeg(q.yaw_deg-p0.yaw_deg)));if(i){double x=q.px-s[i-1].px,y=q.py-s[i-1].py,z=q.pz-s[i-1].pz;path+=std::sqrt(x*x+y*y+z*z);}}
  double fx=pn.px-p0.px,fy=pn.py-p0.py,fz=pn.pz-p0.pz,fd=std::sqrt(fx*fx+fy*fy+fz*fz);
  std::cout<<std::fixed<<std::setprecision(3)
           <<"final dP mm: ["<<fx*1000<<' '<<fy*1000<<' '<<fz*1000<<"]\n"
           <<"final |dP| mm: "<<fd*1000<<"\npath length mm: "<<path*1000<<"\nmax excursion mm: "<<max_exc*1000<<"\nmax speed mm/s: "<<max_speed*1000<<"\n"
           <<"orientation span deg (R/P/Y): ["<<max_roll<<' '<<max_pitch<<' '<<max_yaw<<"]\n"
           <<"final dRPY deg: ["<<wrapDeg(pn.roll_deg-p0.roll_deg)<<' '<<wrapDeg(pn.pitch_deg-p0.pitch_deg)<<' '<<wrapDeg(pn.yaw_deg-p0.yaw_deg)<<"]\n"
           <<"REPLAY RESULT: PASS\n";
}

} // namespace

int main(int argc,char**argv){
  google::InitGoogleLogging(argv[0]);FLAGS_visualize=false;FLAGS_viz_type=2;FLAGS_use_lcd=false;FLAGS_log_output=false;FLAGS_extract_planes_from_the_scene=false;
  const std::string params=argc>1?argv[1]:"params/JTZeroMonoFLU";
  const std::string tag=argc>2?argv[2]:"REAL";
  const std::string combined=argc>3?argv[3]:kDefaultCombined;
  const std::string camindex=argc>4?argv[4]:kDefaultCamIndex;
  const std::string mjpeg=argc>5?argv[5]:kDefaultMjpeg;
  std::shared_ptr<HudPipeline>pipe;std::thread worker;bool started=false;
  try{
    auto imu=loadImu10(combined);size_t raw=0,rej=0;auto cam=loadCam10(camindex,&raw,&rej);
    VIO::VioParams vp(params);if(vp.camera_params_.empty())throw std::runtime_error("No camera params loaded");
    const auto tt=vp.camera_params_.at(0).body_Pose_cam_.translation();const Eigen::Vector3d tbc(tt.x(),tt.y(),tt.z());
    pipe=std::make_shared<HudPipeline>(vp);pipe->installBackendCallback();worker=std::thread([pipe](){pipe->spin();});started=true;
    std::ifstream jf(mjpeg,std::ios::binary);if(!jf)throw std::runtime_error("Cannot open MJPEG: "+mjpeg);
    jtzero::ImuCorrection corr;size_t ii=0,ci=0,decoded=0;VIO::FrameId fid=0;
    std::cout<<"[REPLAY] "<<tag<<" params="<<params<<" IMU="<<imu.size()<<" selected_camera="<<cam.size()<<"\n";
    while(ii<imu.size()||ci<cam.size()){
      const bool use_imu=ci>=cam.size()||(ii<imu.size()&&imu[ii].mapped_ns<=cam[ci].ts_ns);
      if(use_imu){const auto&s=imu[ii++];const Eigen::Vector3d acc=jtzero::ImuCorrection::accelFrdToFlu(s.ax,s.ay,s.az),g=jtzero::ImuCorrection::gyroFrdToFlu(s.gx,s.gy,s.gz);const Eigen::Vector3d w=corr.correctGyro((uint64_t)(s.source_ns/1000LL),acc,g,true);VIO::ImuAccGyr d;d<<acc.x(),acc.y(),acc.z(),w.x(),w.y(),w.z();pipe->fillSingleImuQueue(VIO::ImuMeasurement(s.mapped_ns,d));}
      else{const auto&s=cam[ci++];std::vector<unsigned char>jpg(s.bytes);jf.clear();jf.seekg((std::streamoff)s.offset,std::ios::beg);jf.read((char*)jpg.data(),(std::streamsize)s.bytes);if((size_t)jf.gcount()!=s.bytes)throw std::runtime_error("Short MJPEG read at sequence "+std::to_string(s.seq));cv::Mat g=cv::imdecode(jpg,cv::IMREAD_GRAYSCALE);if(!g.empty()){pipe->fillLeftFrameQueue(std::make_unique<VIO::Frame>(fid++,s.ts_ns,vp.camera_params_.at(0),g));++decoded;}}
      if(((ii+ci)%250)==0)std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    int64_t deadline=monotonicNs()+30000000000LL,last_change=monotonicNs();size_t prev=0;
    while(monotonicNs()<deadline){auto st=pipe->states();if(st.size()!=prev){prev=st.size();last_change=monotonicNs();}if(prev>0&&monotonicNs()-last_change>2000000000LL)break;std::this_thread::sleep_for(std::chrono::milliseconds(100));}
    pipe->shutdown();if(worker.joinable())worker.join();started=false;auto states=pipe->states();
    const std::string out="/home/vio/jtzero_extrinsics_replay_v10_"+tag+".csv";saveStates10(out,states);summary10(tag,params,tbc,states,imu.size(),raw,rej,cam.size(),decoded);std::cout<<"states CSV: "<<out<<"\n";return states.empty()?1:0;
  }catch(const std::exception&e){std::cerr<<"[FATAL] "<<e.what()<<"\n";if(pipe)pipe->shutdown();if(started&&worker.joinable())worker.join();return 1;}
}
