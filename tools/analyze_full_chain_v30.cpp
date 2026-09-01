// JT-ZERO v30: offline phase-reset / drift localization analyzer for v26 CSV.
// No new physical run required.
// Purpose: determine whether cumulative FC-vs-IMU orientation error is injected
// mainly during yaw motion or during the stationary/settling portions of each phase.

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace {
constexpr double kPi = 3.14159265358979323846;

struct Row {
  int phase = 0;
  double dt = 0.0;
  Eigen::Vector3d gyr = Eigen::Vector3d::Zero();
  Eigen::Vector3d bg_static = Eigen::Vector3d::Zero();
  Eigen::Vector3d fc_euler_deg = Eigen::Vector3d::Zero();
};

std::vector<std::string> split(const std::string& s){
  std::vector<std::string> out; std::stringstream ss(s); std::string x;
  while(std::getline(ss,x,',')) out.push_back(x); return out;
}

double val(const std::vector<std::string>& v,
           const std::unordered_map<std::string,size_t>& h,
           const std::string& k){
  auto it=h.find(k); if(it==h.end()||it->second>=v.size()||v[it->second].empty()) return 0.0;
  return std::stod(v[it->second]);
}

Eigen::Matrix3d fcR(const Eigen::Vector3d& e_deg){
  const Eigen::Vector3d e=e_deg*(kPi/180.0);
  return Eigen::AngleAxisd(e.x(),Eigen::Vector3d::UnitX()).toRotationMatrix()*
         Eigen::AngleAxisd(e.y(),Eigen::Vector3d::UnitY()).toRotationMatrix()*
         Eigen::AngleAxisd(e.z(),Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

Eigen::Vector3d logR(const Eigen::Matrix3d& R){
  Eigen::AngleAxisd aa(R); if(!std::isfinite(aa.angle())) return Eigen::Vector3d::Zero();
  return aa.axis()*aa.angle();
}

double rotDeg(const Eigen::Matrix3d& A,const Eigen::Matrix3d& B){
  return logR(A.transpose()*B).norm()*180.0/kPi;
}

double gravDeg(const Eigen::Matrix3d& A,const Eigen::Matrix3d& B){
  Eigen::Vector3d za=A.transpose()*Eigen::Vector3d::UnitZ();
  Eigen::Vector3d zb=B.transpose()*Eigen::Vector3d::UnitZ();
  return std::acos(std::clamp(za.dot(zb),-1.0,1.0))*180.0/kPi;
}

struct PhaseStats{
  size_t n=0, ndyn=0, nstill=0;
  double duration=0, dyn_duration=0, still_duration=0;
  double rms_rot=0,max_rot=0,rms_grav=0,max_grav=0;
  double final_rot=0,final_grav=0;
  double dyn_rms_rate=0,still_rms_rate=0;
  Eigen::Vector3d dyn_mean_res=Eigen::Vector3d::Zero();
  Eigen::Vector3d still_mean_res=Eigen::Vector3d::Zero();
  Eigen::Vector3d net_fc=Eigen::Vector3d::Zero();
  Eigen::Vector3d net_imu=Eigen::Vector3d::Zero();
  Eigen::Vector3d net_err=Eigen::Vector3d::Zero();
};

PhaseStats analyzePhase(const std::vector<Row>& rows,int p){
  PhaseStats st;
  size_t first=rows.size(), last=rows.size();
  for(size_t i=0;i<rows.size();++i) if(rows[i].phase==p){if(first==rows.size()) first=i; last=i;}
  if(first==rows.size()||last<=first) return st;

  Eigen::Matrix3d Rimu=fcR(rows[first].fc_euler_deg); // reset at phase start
  const Eigen::Matrix3d Rfc0=fcR(rows[first].fc_euler_deg);
  double ssr=0,ssg=0,ss_dyn_rate=0,ss_still_rate=0;
  Eigen::Vector3d sum_dyn=Eigen::Vector3d::Zero(),sum_still=Eigen::Vector3d::Zero();

  for(size_t i=first+1;i<=last;++i){
    const double dt=rows[i].dt;
    if(!(dt>0.001&&dt<0.02)) continue;
    const Eigen::Matrix3d F0=fcR(rows[i-1].fc_euler_deg);
    const Eigen::Matrix3d F1=fcR(rows[i].fc_euler_deg);
    const Eigen::Vector3d wfc=logR(F0.transpose()*F1)/dt;
    const Eigen::Vector3d wimu=rows[i].gyr-rows[i].bg_static;
    const Eigen::Vector3d res=wfc-wimu;
    const bool dyn=std::abs(wimu.z())>0.03;

    const Eigen::Vector3d th=wimu*dt; const double a=th.norm();
    if(a>1e-12) Rimu=Rimu*Eigen::AngleAxisd(a,th/a).toRotationMatrix();

    const double er=rotDeg(F1,Rimu), eg=gravDeg(F1,Rimu);
    ssr+=er*er; ssg+=eg*eg; st.max_rot=std::max(st.max_rot,er); st.max_grav=std::max(st.max_grav,eg);
    ++st.n; st.duration+=dt;
    if(dyn){++st.ndyn;st.dyn_duration+=dt;sum_dyn+=res;ss_dyn_rate+=res.squaredNorm();}
    else{++st.nstill;st.still_duration+=dt;sum_still+=res;ss_still_rate+=res.squaredNorm();}
  }

  if(st.n){st.rms_rot=std::sqrt(ssr/st.n);st.rms_grav=std::sqrt(ssg/st.n);}
  if(st.ndyn){st.dyn_mean_res=sum_dyn/double(st.ndyn);st.dyn_rms_rate=std::sqrt(ss_dyn_rate/st.ndyn);}
  if(st.nstill){st.still_mean_res=sum_still/double(st.nstill);st.still_rms_rate=std::sqrt(ss_still_rate/st.nstill);}

  const Eigen::Matrix3d Rfc1=fcR(rows[last].fc_euler_deg);
  st.final_rot=rotDeg(Rfc1,Rimu); st.final_grav=gravDeg(Rfc1,Rimu);
  st.net_fc=logR(Rfc0.transpose()*Rfc1)*180.0/kPi;
  st.net_imu=logR(Rfc0.transpose()*Rimu)*180.0/kPi;
  st.net_err=logR(Rfc1.transpose()*Rimu)*180.0/kPi;
  return st;
}

} // namespace

int main(int argc,char**argv){
  if(argc<2){std::cerr<<"usage: "<<argv[0]<<" /home/vio/jtzero_live_full_chain_v26.csv\n";return 2;}
  std::ifstream f(argv[1]); if(!f){std::cerr<<"cannot open "<<argv[1]<<"\n";return 2;}
  std::string line; if(!std::getline(f,line)) return 2; auto hh=split(line);
  std::unordered_map<std::string,size_t> h; for(size_t i=0;i<hh.size();++i) h[hh[i]]=i;
  std::vector<Row> rows;
  while(std::getline(f,line)){
    if(line.empty()) continue; auto v=split(line); try{
      Row r; r.phase=(int)val(v,h,"phase"); r.dt=val(v,h,"dt");
      r.gyr<<val(v,h,"gx"),val(v,h,"gy"),val(v,h,"gz");
      r.bg_static<<val(v,h,"bg_static_x"),val(v,h,"bg_static_y"),val(v,h,"bg_static_z");
      r.fc_euler_deg<<val(v,h,"fc_roll"),val(v,h,"fc_pitch"),val(v,h,"fc_yaw");
      rows.push_back(r);
    }catch(...){}
  }
  if(rows.size()<20){std::cerr<<"too few rows\n";return 2;}

  static const char* names[4]={"+30","0_after_plus","-30","0_final"};
  std::cout<<std::fixed<<std::setprecision(9);
  std::cout<<"============================================================\n";
  std::cout<<"JT-ZERO PHASE-RESET / DRIFT LOCALIZATION ANALYZER v30\n";
  std::cout<<"============================================================\n";
  std::cout<<"rows="<<rows.size()<<"\n";
  std::cout<<"dynamic threshold: |wz| > 0.03 rad/s\n";

  double worst_final=0, dyn_weight=0, still_weight=0;
  for(int p=0;p<4;++p){
    PhaseStats s=analyzePhase(rows,p);
    std::cout<<"\n"<<names[p]<<"\n";
    std::cout<<"  duration="<<s.duration<<" s  dynamic="<<s.dyn_duration<<" s  still="<<s.still_duration<<" s\n";
    std::cout<<"  phase-reset RMS rot="<<s.rms_rot<<" deg max="<<s.max_rot<<" deg\n";
    std::cout<<"  phase-reset RMS grav="<<s.rms_grav<<" deg max="<<s.max_grav<<" deg\n";
    std::cout<<"  phase-reset final rot="<<s.final_rot<<" deg final grav="<<s.final_grav<<" deg\n";
    std::cout<<"  net FC dR vec = ["<<s.net_fc.transpose()<<"] deg\n";
    std::cout<<"  net IMU dR vec= ["<<s.net_imu.transpose()<<"] deg\n";
    std::cout<<"  net error vec = ["<<s.net_err.transpose()<<"] deg\n";
    std::cout<<"  dynamic residual mean=["<<s.dyn_mean_res.transpose()<<"] rad/s RMSnorm="<<s.dyn_rms_rate<<"\n";
    std::cout<<"  still residual mean  =["<<s.still_mean_res.transpose()<<"] rad/s RMSnorm="<<s.still_rms_rate<<"\n";
    worst_final=std::max(worst_final,s.final_grav);
    dyn_weight+=s.dyn_rms_rate*s.dyn_duration;
    still_weight+=s.still_rms_rate*s.still_duration;
  }

  std::cout<<"\nDecision helper:\n";
  std::cout<<"  worst phase-reset final gravity error="<<worst_final<<" deg\n";
  std::cout<<"  duration-weighted residual score dynamic="<<dyn_weight<<" still="<<still_weight<<"\n";
  if(worst_final<0.8){
    std::cout<<"  RESULT: most global drift is inherited across phase boundaries; local per-phase gyro integration is comparatively good. Focus on estimator corrections/holds and state reference semantics.\n";
  }else if(still_weight>dyn_weight*1.25){
    std::cout<<"  RESULT: stationary/settling intervals contribute disproportionately. FC estimator attitude corrections during holds are a strong candidate.\n";
  }else if(dyn_weight>still_weight*1.25){
    std::cout<<"  RESULT: motion intervals contribute disproportionately. Investigate gyro source/calibration/coning during yaw.\n";
  }else{
    std::cout<<"  RESULT: error is distributed across both motion and hold intervals. A live source-semantics test logging ATTITUDE rates and multiple IMU instances is justified next.\n";
  }
  return 0;
}
