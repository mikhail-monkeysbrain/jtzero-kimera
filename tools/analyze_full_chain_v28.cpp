// JT-ZERO v28: offline residual-structure and ATTITUDE/HIGHRES_IMU lag analyzer.
// Uses the existing v26 CSV. No new physical run is required.
// Goals:
//   1) measure FC body-rate minus HIGHRES_IMU body-rate residual per route phase;
//   2) see whether a fixed yaw-affine gyro calibration is stable across phases;
//   3) scan relative ATTITUDE/IMU sample lag and quantify how much timing can explain.

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace {
constexpr double kPi = 3.14159265358979323846;

struct Row {
  int phase = 0;
  double dt = 0;
  Eigen::Vector3d gyr = Eigen::Vector3d::Zero();
  Eigen::Vector3d bg_static = Eigen::Vector3d::Zero();
  Eigen::Vector3d fc_euler_deg = Eigen::Vector3d::Zero();
};

std::vector<std::string> split(const std::string& s) {
  std::vector<std::string> out; std::stringstream ss(s); std::string x;
  while (std::getline(ss, x, ',')) out.push_back(x);
  return out;
}

double val(const std::vector<std::string>& v,
           const std::unordered_map<std::string,size_t>& h,
           const std::string& k) {
  auto it=h.find(k); if(it==h.end()||it->second>=v.size()||v[it->second].empty()) return 0.0;
  return std::stod(v[it->second]);
}

Eigen::Matrix3d fcR(const Eigen::Vector3d& e_deg) {
  Eigen::Vector3d e=e_deg*(kPi/180.0);
  return Eigen::AngleAxisd(e.x(),Eigen::Vector3d::UnitX()).toRotationMatrix()*
         Eigen::AngleAxisd(e.y(),Eigen::Vector3d::UnitY()).toRotationMatrix()*
         Eigen::AngleAxisd(e.z(),Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

Eigen::Vector3d logR(const Eigen::Matrix3d& R) {
  Eigen::AngleAxisd aa(R); if(!std::isfinite(aa.angle())) return Eigen::Vector3d::Zero();
  return aa.axis()*aa.angle();
}

struct Sample {
  int phase=0;
  double dt=0;
  Eigen::Vector3d wimu=Eigen::Vector3d::Zero();
  Eigen::Vector3d wfc=Eigen::Vector3d::Zero();
};

struct Fit { Eigen::Vector3d c=Eigen::Vector3d::Zero(), k=Eigen::Vector3d::Zero(); double rms=0; size_t n=0; };

Fit fitAffine(const std::vector<Sample>& s, int phase_filter, bool dynamic_only) {
  Eigen::Matrix2d H=Eigen::Matrix2d::Zero();
  Eigen::Matrix<double,2,3> B=Eigen::Matrix<double,2,3>::Zero();
  size_t n=0;
  for(const auto& q:s) {
    if(phase_filter>=0 && q.phase!=phase_filter) continue;
    if(dynamic_only && std::abs(q.wimu.z())<=0.03) continue;
    Eigen::Vector2d x(1.0,q.wimu.z());
    Eigen::Vector3d y=q.wfc-q.wimu;
    H+=x*x.transpose(); B+=x*y.transpose(); ++n;
  }
  Fit f; f.n=n; if(n<10 || std::abs(H.determinant())<1e-14) return f;
  Eigen::Matrix<double,2,3> P=H.ldlt().solve(B); f.c=P.row(0).transpose(); f.k=P.row(1).transpose();
  double ss=0;
  for(const auto& q:s) {
    if(phase_filter>=0 && q.phase!=phase_filter) continue;
    if(dynamic_only && std::abs(q.wimu.z())<=0.03) continue;
    Eigen::Vector3d pred=q.wimu+f.c+f.k*q.wimu.z(); ss+=(q.wfc-pred).squaredNorm();
  }
  f.rms=std::sqrt(ss/std::max<size_t>(1,n)); return f;
}

void printResidualStats(const std::vector<Sample>& s,int p,const char* name) {
  Eigen::Vector3d sum=Eigen::Vector3d::Zero(), ss=Eigen::Vector3d::Zero(); size_t n=0, nd=0;
  Eigen::Vector3d sumd=Eigen::Vector3d::Zero(), ssd=Eigen::Vector3d::Zero();
  for(const auto&q:s) if(q.phase==p) {
    Eigen::Vector3d e=q.wfc-q.wimu; sum+=e; ss+=e.cwiseProduct(e); ++n;
    if(std::abs(q.wimu.z())>0.03){sumd+=e;ssd+=e.cwiseProduct(e);++nd;}
  }
  Eigen::Vector3d mean=Eigen::Vector3d::Zero();
  Eigen::Vector3d rms=Eigen::Vector3d::Zero();
  Eigen::Vector3d meand=Eigen::Vector3d::Zero();
  Eigen::Vector3d rmsd=Eigen::Vector3d::Zero();
  if(n){mean=sum/double(n);rms=(ss/double(n)).cwiseSqrt();}
  if(nd){meand=sumd/double(nd);rmsd=(ssd/double(nd)).cwiseSqrt();}
  std::cout<<"\n"<<name<<" residual FC-IMU all n="<<n<<"\n"
           <<"  mean=["<<mean.transpose()<<"] rad/s\n"
           <<"  rms =["<<rms.transpose()<<"] rad/s\n"
           <<"  dynamic |wz|>0.03 n="<<nd<<"\n"
           <<"  mean=["<<meand.transpose()<<"] rad/s\n"
           <<"  rms =["<<rmsd.transpose()<<"] rad/s\n";
  Fit fa=fitAffine(s,p,false);
  std::cout<<"  phase affine c=["<<fa.c.transpose()<<"] k=["<<fa.k.transpose()<<"] RMS="<<fa.rms<<" rad/s\n";
}

struct LagResult { int shift=0; double rms=std::numeric_limits<double>::infinity(); size_t n=0; };

LagResult lagRms(const std::vector<Sample>& s,int shift,int phase_filter) {
  double ss=0; size_t n=0;
  const int N=(int)s.size();
  for(int i=0;i<N;++i){
    int j=i+shift; if(j<0||j>=N) continue;
    if(phase_filter>=0 && (s[i].phase!=phase_filter || s[j].phase!=phase_filter)) continue;
    Eigen::Vector3d e=s[i].wfc-s[j].wimu; ss+=e.squaredNorm(); ++n;
  }
  LagResult r; r.shift=shift;r.n=n;if(n)r.rms=std::sqrt(ss/n);return r;
}

LagResult bestLag(const std::vector<Sample>& s,int phase_filter) {
  LagResult best;
  for(int sh=-20;sh<=20;++sh){auto r=lagRms(s,sh,phase_filter);if(r.rms<best.rms)best=r;}
  return best;
}

} // namespace

int main(int argc,char**argv){
  if(argc<2){std::cerr<<"usage: "<<argv[0]<<" /home/vio/jtzero_live_full_chain_v26.csv\n";return 2;}
  std::ifstream f(argv[1]); if(!f){std::cerr<<"cannot open "<<argv[1]<<"\n";return 2;}
  std::string line; if(!std::getline(f,line))return 2; auto hh=split(line);
  std::unordered_map<std::string,size_t> h;for(size_t i=0;i<hh.size();++i)h[hh[i]]=i;
  std::vector<Row> rows;
  while(std::getline(f,line)){
    if(line.empty())continue; auto v=split(line); try{
      Row r; r.phase=(int)val(v,h,"phase"); r.dt=val(v,h,"dt");
      r.gyr<<val(v,h,"gx"),val(v,h,"gy"),val(v,h,"gz");
      r.bg_static<<val(v,h,"bg_static_x"),val(v,h,"bg_static_y"),val(v,h,"bg_static_z");
      r.fc_euler_deg<<val(v,h,"fc_roll"),val(v,h,"fc_pitch"),val(v,h,"fc_yaw"); rows.push_back(r);
    }catch(...){}
  }
  if(rows.size()<20){std::cerr<<"too few rows\n";return 2;}
  std::vector<Sample> s; s.reserve(rows.size()-1);
  for(size_t i=1;i<rows.size();++i){double dt=rows[i].dt;if(!(dt>0.001&&dt<0.02))continue;
    Eigen::Vector3d wf=logR(fcR(rows[i-1].fc_euler_deg).transpose()*fcR(rows[i].fc_euler_deg))/dt;
    if(!wf.allFinite()||wf.norm()>4.0)continue;
    Sample q;q.phase=rows[i].phase;q.dt=dt;q.wfc=wf;q.wimu=rows[i].gyr-rows[i].bg_static;s.push_back(q);
  }
  static const char* names[4]={"+30","0_after_plus","-30","0_final"};
  std::cout<<std::fixed<<std::setprecision(9);
  std::cout<<"============================================================\nJT-ZERO RESIDUAL / LAG ANALYZER v28\n============================================================\n";
  std::cout<<"rows="<<rows.size()<<" valid intervals="<<s.size()<<" nominal dt≈5 ms\n";
  for(int p=0;p<4;++p)printResidualStats(s,p,names[p]);
  Fit global=fitAffine(s,-1,false);
  std::cout<<"\nGLOBAL affine (diagnostic only):\n  c=["<<global.c.transpose()<<"]\n  k=["<<global.k.transpose()<<"]\n  RMS="<<global.rms<<" rad/s\n";
  auto b0=lagRms(s,0,-1),bg=bestLag(s,-1);
  std::cout<<"\nATTITUDE/IMU lag scan, shifts -20..+20 samples (~±100 ms):\n";
  std::cout<<"  global shift 0 RMS="<<b0.rms<<" rad/s\n";
  std::cout<<"  global best shift="<<bg.shift<<" samples (~"<<bg.shift*5.0<<" ms) RMS="<<bg.rms<<" rad/s ratio="<<bg.rms/std::max(1e-12,b0.rms)<<"\n";
  for(int p=0;p<4;++p){auto z=lagRms(s,0,p),b=bestLag(s,p);std::cout<<"  "<<names[p]<<": best shift="<<b.shift<<" (~"<<b.shift*5.0<<" ms) RMS "<<z.rms<<" -> "<<b.rms<<" ratio="<<b.rms/std::max(1e-12,z.rms)<<"\n";}
  std::cout<<"\nDecision helper:\n";
  double ratio=bg.rms/std::max(1e-12,b0.rms);
  if(ratio<0.70) std::cout<<"  RESULT: a relative ATTITUDE/IMU lag explains a large fraction of instantaneous rate mismatch. Verify ATTITUDE timestamp/source semantics next.\n";
  else if(ratio<0.90) std::cout<<"  RESULT: timing contributes, but is not sufficient by itself. Compare phase-local residuals/corrections.\n";
  else std::cout<<"  RESULT: constant relative timing shift is not the main cause. Inspect phase-dependent estimator corrections/source semantics.\n";
  std::cout<<"  If phase affine c/k change strongly between +yaw, return, -yaw, return, a single sensor calibration model is not supported by this run.\n";
  return 0;
}
