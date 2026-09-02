// JT-ZERO Stage 11.6 v11 deterministic ZXY A/B replay.
// Replays exactly one recorded Camera+IMU dataset through Kimera while changing
// only the fixed gyro Z->X/Y cross-axis correction:
//   CURRENT : FRD->FLU -> ZXY -> same static gravity feedback -> Kimera
//   NO_ZXY  : FRD->FLU ->       same static gravity feedback -> Kimera
// Camera bytes, timestamps, IMU samples, R_BC, t_BC and all Kimera params remain identical.

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
constexpr int64_t kReplayCameraPeriodNs = 30000000LL;
constexpr int64_t kReplayRawMaxDtNs = 20000000LL;
constexpr double kRadToDeg = 180.0 / M_PI;

struct ReplayImu {
  int64_t source_ns=0, mapped_ns=0;
  double ax=0,ay=0,az=0,gx=0,gy=0,gz=0;
};
struct ReplayCam {
  uint32_t seq=0;
  int64_t ts_ns=0;
  uint64_t offset=0;
  size_t bytes=0;
};

struct GyroIntegral {
  bool have_prev=false;
  uint64_t prev_us=0;
  Eigen::Vector3d raw_rad=Eigen::Vector3d::Zero();
  Eigen::Vector3d fed_rad=Eigen::Vector3d::Zero();
  size_t integrated_samples=0;

  void add(uint64_t us,const Eigen::Vector3d&raw,const Eigen::Vector3d&fed){
    if(have_prev&&us>prev_us){
      const double dt=(us-prev_us)*1e-6;
      if(dt>0.0&&dt<=0.03){
        raw_rad+=raw*dt;
        fed_rad+=fed*dt;
        ++integrated_samples;
      }
    }
    prev_us=us;have_prev=true;
  }
};

// Local copy of the production ImuCorrection gravity logic. The only switch is
// whether the already validated fixed ZXY term is applied before that logic.
// This deliberately avoids modifying tools/jtzero_imu_correction.h or any live tool.
class ReplayImuCorrection11 {
 public:
  explicit ReplayImuCorrection11(bool use_zxy):use_zxy_(use_zxy){}

  Eigen::Vector3d correctGyro(uint64_t imu_us,
                              const Eigen::Vector3d& accel_flu,
                              const Eigen::Vector3d& gyro_flu,
                              bool allow_gravity_feedback=true){
    const Eigen::Vector3d gyro_in=use_zxy_?jtzero::ImuCorrection::applyZxy(gyro_flu):gyro_flu;

    double dt=0.0;
    if(last_imu_us_!=0&&imu_us>last_imu_us_){
      dt=static_cast<double>(imu_us-last_imu_us_)*1e-6;
    }
    last_imu_us_=imu_us;
    if(dt<=0.0||dt>0.03)return gyro_in;

    if(!accel_lp_initialized_){
      accel_lp_=accel_flu;
      accel_lp_initialized_=true;
    }else{
      const double alpha=std::exp(-dt/jtzero::ImuCorrection::kAccelLpTauSec);
      accel_lp_=alpha*accel_lp_+(1.0-alpha)*accel_flu;
    }

    const double acc_norm=accel_flu.norm();
    const bool gravity_magnitude_ok=
        acc_norm>1e-6&&
        std::abs(acc_norm-jtzero::ImuCorrection::kGravityMps2)<=jtzero::ImuCorrection::kGravityAccTolMps2;
    const bool gyro_quiet=gyro_in.norm()<=jtzero::ImuCorrection::kStaticGyroMaxRadS;
    const bool accel_quiet=
        (accel_flu-accel_lp_).norm()<=jtzero::ImuCorrection::kStaticAccelResidualMaxMps2;
    const bool static_sample=
        allow_gravity_feedback&&gravity_magnitude_ok&&gyro_quiet&&accel_quiet;

    if(static_sample)static_time_sec_+=dt;
    else static_time_sec_=0.0;

    const bool static_confirmed=static_time_sec_>=jtzero::ImuCorrection::kStaticHoldSec;

    if(!initialized_){
      if(static_confirmed){
        gravity_body_=accel_lp_.normalized();
        initialized_=true;
      }
      return gyro_in;
    }

    Eigen::Vector3d corrected=gyro_in;
    if(static_confirmed){
      const Eigen::Vector3d measured_gravity=accel_lp_.normalized();
      Eigen::Vector3d gravity_error=gravity_body_.cross(measured_gravity);
      const double error_norm=gravity_error.norm();
      if(error_norm<=std::sin(jtzero::ImuCorrection::kMaxGravityErrorRad)){
        Eigen::Vector3d correction=jtzero::ImuCorrection::kGravityKp*gravity_error;
        const double correction_norm=correction.norm();
        if(correction_norm>jtzero::ImuCorrection::kMaxGravityCorrectionRadS){
          correction*=jtzero::ImuCorrection::kMaxGravityCorrectionRadS/correction_norm;
        }
        corrected-=correction;
      }
    }

    const Eigen::Vector3d theta=-corrected*dt;
    const double angle=theta.norm();
    if(angle>1e-12){
      gravity_body_=Eigen::AngleAxisd(angle,theta/angle)*gravity_body_;
      gravity_body_.normalize();
    }
    return corrected;
  }

 private:
  bool use_zxy_=true;
  bool initialized_=false;
  bool accel_lp_initialized_=false;
  uint64_t last_imu_us_=0;
  double static_time_sec_=0.0;
  Eigen::Vector3d gravity_body_=Eigen::Vector3d(0.0,0.0,1.0);
  Eigen::Vector3d accel_lp_=Eigen::Vector3d::Zero();
};

std::vector<std::string> splitCsv11(const std::string&line){
  std::vector<std::string>out;std::string cur;
  for(char ch:line){if(ch==','){out.push_back(cur);cur.clear();}else cur.push_back(ch);}out.push_back(cur);
  return out;
}
long long i64v11(const std::string&s){return s.empty()?0:std::stoll(s);}
double dv11(const std::string&s){return s.empty()?0.0:std::stod(s);}

std::vector<ReplayImu> loadImu11(const std::string&path){
  std::ifstream f(path);if(!f)throw std::runtime_error("Cannot open combined CSV: "+path);
  std::vector<ReplayImu>out;std::string line;std::getline(f,line);
  while(std::getline(f,line)){
    auto c=splitCsv11(line);if(c.size()<14||c[0]!="IMU")continue;
    ReplayImu s;s.source_ns=i64v11(c[2]);s.mapped_ns=i64v11(c[3]);
    s.ax=dv11(c[8]);s.ay=dv11(c[9]);s.az=dv11(c[10]);
    s.gx=dv11(c[11]);s.gy=dv11(c[12]);s.gz=dv11(c[13]);
    if(s.source_ns>0&&s.mapped_ns>0)out.push_back(s);
  }
  if(out.empty())throw std::runtime_error("No IMU rows in combined CSV");
  return out;
}

std::vector<ReplayCam> loadCam11(const std::string&path,size_t*raw_count,size_t*rejected){
  std::ifstream f(path);if(!f)throw std::runtime_error("Cannot open camera index CSV: "+path);
  std::vector<ReplayCam>out;std::string line;std::getline(f,line);
  bool have=false;uint32_t prev_seq=0;int64_t prev_ts=0,last_selected=0;size_t raw=0,rej=0;
  while(std::getline(f,line)){
    auto c=splitCsv11(line);if(c.size()<7)continue;++raw;
    ReplayCam s;s.seq=(uint32_t)std::stoul(c[0]);s.ts_ns=i64v11(c[2]);s.offset=(uint64_t)std::stoull(c[5]);s.bytes=(size_t)std::stoull(c[6]);
    bool ok=true;if(have){const int64_t dt=s.ts_ns-prev_ts;ok=s.seq==prev_seq+1U&&dt>0&&dt<=kReplayRawMaxDtNs;if(!ok)++rej;}
    prev_seq=s.seq;prev_ts=s.ts_ns;have=true;
    const bool due=last_selected==0||s.ts_ns-last_selected>=kReplayCameraPeriodNs;
    if(ok&&due&&s.bytes>0){out.push_back(s);last_selected=s.ts_ns;}
  }
  if(raw_count)*raw_count=raw;if(rejected)*rejected=rej;
  if(out.empty())throw std::runtime_error("No selected camera rows");
  return out;
}

void saveStates11(const std::string&path,const std::vector<VioState>&states){
  std::ofstream f(path,std::ios::trunc);if(!f)throw std::runtime_error("Cannot create output CSV: "+path);
  f<<"keyframe,timestamp_ns,px_m,py_m,pz_m,vx_m_s,vy_m_s,vz_m_s,roll_deg,pitch_deg,yaw_deg\n";
  f<<std::fixed<<std::setprecision(9);
  for(const auto&s:states)f<<s.keyframe<<','<<s.timestamp_ns<<','<<s.px<<','<<s.py<<','<<s.pz<<','<<s.vx<<','<<s.vy<<','<<s.vz<<','<<s.roll_deg<<','<<s.pitch_deg<<','<<s.yaw_deg<<'\n';
}

void summary11(const std::string&mode,const std::string&params,const Eigen::Vector3d&tbc,
               const std::vector<VioState>&s,const GyroIntegral&gi,size_t imu_n,
               size_t raw,size_t rej,size_t selected,size_t decoded){
  std::cout<<"\n================ ZXY A/B REPLAY V11 ================\n";
  std::cout<<"mode: "<<mode<<"\nparams: "<<params<<"\nt_BC mm: ["<<(tbc*1000.0).transpose()<<"]\n"
           <<"ZXY enabled: "<<(mode=="CURRENT"?"yes":"no")<<"\n"
           <<"ZXY coefficients: Cx="<<jtzero::ImuCorrection::kGyroCx
           <<" Cy="<<jtzero::ImuCorrection::kGyroCy<<"\n"
           <<"IMU rows: "<<imu_n<<"\nraw camera rows: "<<raw<<"\nrejected raw pairs: "<<rej
           <<"\nselected camera rows: "<<selected<<"\ndecoded camera rows: "<<decoded
           <<"\nbackend states: "<<s.size()<<"\n"
           <<"gyro integrated samples: "<<gi.integrated_samples<<"\n";
  std::cout<<std::fixed<<std::setprecision(3)
           <<"integral RAW FLU gyro deg XYZ: ["<<(gi.raw_rad*kRadToDeg).transpose()<<"]\n"
           <<"integral FED gyro deg XYZ: ["<<(gi.fed_rad*kRadToDeg).transpose()<<"]\n"
           <<"integral FED-RAW deg XYZ: ["<<((gi.fed_rad-gi.raw_rad)*kRadToDeg).transpose()<<"]\n";
  if(s.empty()){std::cout<<"REPLAY RESULT: FAIL (no backend states)\n";return;}
  const auto&p0=s.front();const auto&pn=s.back();double path=0,max_exc=0,max_speed=0,max_roll=0,max_pitch=0,max_yaw=0;
  for(size_t i=0;i<s.size();++i){const auto&q=s[i];double dx=q.px-p0.px,dy=q.py-p0.py,dz=q.pz-p0.pz;max_exc=std::max(max_exc,std::sqrt(dx*dx+dy*dy+dz*dz));max_speed=std::max(max_speed,std::sqrt(q.vx*q.vx+q.vy*q.vy+q.vz*q.vz));max_roll=std::max(max_roll,std::abs(wrapDeg(q.roll_deg-p0.roll_deg)));max_pitch=std::max(max_pitch,std::abs(wrapDeg(q.pitch_deg-p0.pitch_deg)));max_yaw=std::max(max_yaw,std::abs(wrapDeg(q.yaw_deg-p0.yaw_deg)));if(i){double x=q.px-s[i-1].px,y=q.py-s[i-1].py,z=q.pz-s[i-1].pz;path+=std::sqrt(x*x+y*y+z*z);}}
  const double fx=pn.px-p0.px,fy=pn.py-p0.py,fz=pn.pz-p0.pz,fd=std::sqrt(fx*fx+fy*fy+fz*fz);
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
  const std::string mode=argc>2?argv[2]:"CURRENT";
  const std::string combined=argc>3?argv[3]:kDefaultCombined;
  const std::string camindex=argc>4?argv[4]:kDefaultCamIndex;
  const std::string mjpeg=argc>5?argv[5]:kDefaultMjpeg;
  if(mode!="CURRENT"&&mode!="NO_ZXY"){
    std::cerr<<"Usage: "<<argv[0]<<" [params] [CURRENT|NO_ZXY] [combined.csv] [camera_index.csv] [camera.mjpg]\n";
    return 2;
  }
  const bool use_zxy=mode=="CURRENT";
  std::shared_ptr<HudPipeline>pipe;std::thread worker;bool started=false;
  try{
    auto imu=loadImu11(combined);size_t raw=0,rej=0;auto cam=loadCam11(camindex,&raw,&rej);
    VIO::VioParams vp(params);if(vp.camera_params_.empty())throw std::runtime_error("No camera params loaded");
    const auto tt=vp.camera_params_.at(0).body_Pose_cam_.translation();const Eigen::Vector3d tbc(tt.x(),tt.y(),tt.z());
    pipe=std::make_shared<HudPipeline>(vp);pipe->installBackendCallback();worker=std::thread([pipe](){pipe->spin();});started=true;
    std::ifstream jf(mjpeg,std::ios::binary);if(!jf)throw std::runtime_error("Cannot open MJPEG: "+mjpeg);
    ReplayImuCorrection11 corr(use_zxy);GyroIntegral gint;size_t ii=0,ci=0,decoded=0;VIO::FrameId fid=0;
    std::cout<<"[REPLAY] mode="<<mode<<" params="<<params<<" IMU="<<imu.size()<<" selected_camera="<<cam.size()<<"\n";
    while(ii<imu.size()||ci<cam.size()){
      const bool use_imu=ci>=cam.size()||(ii<imu.size()&&imu[ii].mapped_ns<=cam[ci].ts_ns);
      if(use_imu){
        const auto&s=imu[ii++];const uint64_t us=(uint64_t)(s.source_ns/1000LL);
        const Eigen::Vector3d acc=jtzero::ImuCorrection::accelFrdToFlu(s.ax,s.ay,s.az);
        const Eigen::Vector3d g=jtzero::ImuCorrection::gyroFrdToFlu(s.gx,s.gy,s.gz);
        const Eigen::Vector3d w=corr.correctGyro(us,acc,g,true);
        gint.add(us,g,w);
        VIO::ImuAccGyr d;d<<acc.x(),acc.y(),acc.z(),w.x(),w.y(),w.z();
        pipe->fillSingleImuQueue(VIO::ImuMeasurement(s.mapped_ns,d));
      }else{
        const auto&s=cam[ci++];std::vector<unsigned char>jpg(s.bytes);jf.clear();jf.seekg((std::streamoff)s.offset,std::ios::beg);jf.read((char*)jpg.data(),(std::streamsize)s.bytes);if((size_t)jf.gcount()!=s.bytes)throw std::runtime_error("Short MJPEG read at sequence "+std::to_string(s.seq));cv::Mat g=cv::imdecode(jpg,cv::IMREAD_GRAYSCALE);if(!g.empty()){pipe->fillLeftFrameQueue(std::make_unique<VIO::Frame>(fid++,s.ts_ns,vp.camera_params_.at(0),g));++decoded;}
      }
      if(((ii+ci)%250)==0)std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    int64_t deadline=monotonicNs()+30000000000LL,last_change=monotonicNs();size_t prev=0;
    while(monotonicNs()<deadline){auto st=pipe->states();if(st.size()!=prev){prev=st.size();last_change=monotonicNs();}if(prev>0&&monotonicNs()-last_change>2000000000LL)break;std::this_thread::sleep_for(std::chrono::milliseconds(100));}
    pipe->shutdown();if(worker.joinable())worker.join();started=false;auto states=pipe->states();
    const std::string out="/home/vio/jtzero_zxy_replay_v11_"+mode+".csv";saveStates11(out,states);summary11(mode,params,tbc,states,gint,imu.size(),raw,rej,cam.size(),decoded);std::cout<<"states CSV: "<<out<<"\n";return states.empty()?1:0;
  }catch(const std::exception&e){std::cerr<<"[FATAL] "<<e.what()<<"\n";if(pipe)pipe->shutdown();if(started&&worker.joinable())worker.join();return 1;}
}
