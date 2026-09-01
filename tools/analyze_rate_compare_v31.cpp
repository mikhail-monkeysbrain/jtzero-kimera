// JT-ZERO v31: offline analyzer for the existing v24 CSV.
// Decisive question: does integrating MAVLink ATTITUDE body rates reproduce
// the direct ATTITUDE orientation, or does it follow HIGHRES_IMU instead?
// No new physical run is required if jtzero_live_rate_compare_v24.csv exists.

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
  int phase=0;
  double dt=0;
  Eigen::Vector3d wi=Eigen::Vector3d::Zero(); // HIGHRES_IMU, already FLU and bias-corrected in v24
  Eigen::Vector3d wf=Eigen::Vector3d::Zero(); // ATTITUDE body rates, already FLU in v24
  double roll=0,pitch=0,yaw=0;               // direct ATTITUDE, degrees, FRD Euler convention
};

std::vector<std::string> split(const std::string&s){
  std::vector<std::string>o;std::stringstream ss(s);std::string x;
  while(std::getline(ss,x,','))o.push_back(x);return o;
}

double val(const std::vector<std::string>&v,const std::unordered_map<std::string,size_t>&h,const std::string&k){
  auto it=h.find(k);if(it==h.end()||it->second>=v.size()||v[it->second].empty())return 0.0;return std::stod(v[it->second]);
}

Eigen::Matrix3d frdToFlu(){Eigen::Matrix3d D=Eigen::Matrix3d::Identity();D(1,1)=-1;D(2,2)=-1;return D;}
Eigen::Matrix3d directR(double roll,double pitch,double yaw){
  const double r=roll*kPi/180.0,p=pitch*kPi/180.0,y=yaw*kPi/180.0;
  return Eigen::AngleAxisd(y,Eigen::Vector3d::UnitZ()).toRotationMatrix()*
         Eigen::AngleAxisd(p,Eigen::Vector3d::UnitY()).toRotationMatrix()*
         Eigen::AngleAxisd(r,Eigen::Vector3d::UnitX()).toRotationMatrix()*frdToFlu();
}
Eigen::Matrix3d stepR(const Eigen::Matrix3d&R,const Eigen::Vector3d&w,double dt){
  if(!(dt>0.001&&dt<0.03))return R;Eigen::Vector3d th=w*dt;double a=th.norm();
  if(a<1e-12)return R;return R*Eigen::AngleAxisd(a,th/a).toRotationMatrix();
}
Eigen::Vector3d logR(const Eigen::Matrix3d&R){Eigen::AngleAxisd aa(R);if(!std::isfinite(aa.angle()))return Eigen::Vector3d::Zero();return aa.axis()*aa.angle();}
double rotDeg(const Eigen::Matrix3d&A,const Eigen::Matrix3d&B){return logR(A.transpose()*B).norm()*180.0/kPi;}
double gravDeg(const Eigen::Matrix3d&A,const Eigen::Matrix3d&B){
  Eigen::Vector3d a=A.transpose()*Eigen::Vector3d::UnitZ(),b=B.transpose()*Eigen::Vector3d::UnitZ();
  return std::acos(std::clamp(a.dot(b),-1.0,1.0))*180.0/kPi;
}

struct M {double rmsR=0,maxR=0,rmsG=0,maxG=0,finalR=0,finalG=0;size_t n=0;};
void add(M& m,double r,double g){m.rmsR+=r*r;m.rmsG+=g*g;m.maxR=std::max(m.maxR,r);m.maxG=std::max(m.maxG,g);m.finalR=r;m.finalG=g;++m.n;}
void finish(M&m){if(m.n){m.rmsR=std::sqrt(m.rmsR/m.n);m.rmsG=std::sqrt(m.rmsG/m.n);}}
void printM(const char*label,const M&m){
  std::cout<<"  "<<label<<": RMSrot="<<m.rmsR<<" maxRot="<<m.maxR
           <<" RMSgrav="<<m.rmsG<<" maxGrav="<<m.maxG
           <<" finalRot="<<m.finalR<<" finalGrav="<<m.finalG<<" deg\n";
}

struct Triplet {M imu_direct, rate_direct, imu_rate;};
Triplet analyzeRange(const std::vector<Row>&r,size_t a,size_t z,bool reset){
  Triplet out;if(a>=z||z>r.size())return out;
  Eigen::Matrix3d Ri=directR(r[a].roll,r[a].pitch,r[a].yaw);
  Eigen::Matrix3d Rf=Ri;
  const Eigen::Matrix3d D0=Ri;
  (void)D0;
  for(size_t i=a;i<z;++i){
    if(i>a){Ri=stepR(Ri,r[i].wi,r[i].dt);Rf=stepR(Rf,r[i].wf,r[i].dt);}
    Eigen::Matrix3d Rd=directR(r[i].roll,r[i].pitch,r[i].yaw);
    add(out.imu_direct,rotDeg(Rd,Ri),gravDeg(Rd,Ri));
    add(out.rate_direct,rotDeg(Rd,Rf),gravDeg(Rd,Rf));
    add(out.imu_rate,rotDeg(Rf,Ri),gravDeg(Rf,Ri));
  }
  finish(out.imu_direct);finish(out.rate_direct);finish(out.imu_rate);return out;
}

} // namespace

int main(int argc,char**argv){
  if(argc<2){std::cerr<<"usage: "<<argv[0]<<" /home/vio/jtzero_live_rate_compare_v24.csv\n";return 2;}
  std::ifstream f(argv[1]);if(!f){std::cerr<<"cannot open "<<argv[1]<<"\n";return 2;}
  std::string line;if(!std::getline(f,line))return 2;auto hh=split(line);std::unordered_map<std::string,size_t>h;for(size_t i=0;i<hh.size();++i)h[hh[i]]=i;
  std::vector<Row> rows;
  while(std::getline(f,line)){if(line.empty())continue;auto v=split(line);try{
    Row q;q.phase=(int)val(v,h,"phase");q.dt=val(v,h,"dt");
    q.wi<<val(v,h,"imu_wx"),val(v,h,"imu_wy"),val(v,h,"imu_wz");
    q.wf<<val(v,h,"fc_wx"),val(v,h,"fc_wy"),val(v,h,"fc_wz");
    q.roll=val(v,h,"fc_roll_deg");q.pitch=val(v,h,"fc_pitch_deg");q.yaw=val(v,h,"fc_yaw_deg");rows.push_back(q);
  }catch(...) {}}
  if(rows.size()<20){std::cerr<<"too few rows\n";return 2;}
  std::cout<<std::fixed<<std::setprecision(9);
  std::cout<<"============================================================\nJT-ZERO FC RATE / ATTITUDE SEMANTICS ANALYZER v31\n============================================================\nrows="<<rows.size()<<"\n";

  Triplet global=analyzeRange(rows,0,rows.size(),false);
  std::cout<<"\nGLOBAL cumulative from test start\n";printM("HIGHRES_IMU vs direct ATTITUDE",global.imu_direct);printM("ATTITUDE rates vs direct ATTITUDE",global.rate_direct);printM("HIGHRES_IMU vs ATTITUDE rates",global.imu_rate);

  static const char*names[4]={"+30","0_after_plus","-30","0_final"};
  for(int p=0;p<4;++p){size_t a=rows.size(),z=0;for(size_t i=0;i<rows.size();++i)if(rows[i].phase==p){a=std::min(a,i);z=i+1;}if(a>=z)continue;Triplet t=analyzeRange(rows,a,z,true);
    std::cout<<"\n"<<names[p]<<" phase-reset\n";printM("HIGHRES_IMU vs direct ATTITUDE",t.imu_direct);printM("ATTITUDE rates vs direct ATTITUDE",t.rate_direct);printM("HIGHRES_IMU vs ATTITUDE rates",t.imu_rate);
  }

  std::cout<<"\nDecision helper:\n";
  const double gd=global.imu_direct.rmsG, fd=global.rate_direct.rmsG, ir=global.imu_rate.rmsG;
  std::cout<<"  gravity RMS: IMU-direct="<<gd<<"  FC-rate-direct="<<fd<<"  IMU-FC-rate="<<ir<<" deg\n";
  if(ir<0.5*std::min(gd,fd) && gd>0.5 && fd>0.5)
    std::cout<<"  RESULT: HIGHRES_IMU and ATTITUDE body rates agree with each other, while both diverge from direct ATTITUDE. FC estimator corrections/source semantics are primary.\n";
  else if(fd<0.5*gd && fd<0.7)
    std::cout<<"  RESULT: ATTITUDE body rates reproduce direct ATTITUDE much better than HIGHRES_IMU. HIGHRES_IMU source/selected IMU mismatch is primary suspect.\n";
  else if(ir>0.7)
    std::cout<<"  RESULT: HIGHRES_IMU and ATTITUDE body rates themselves integrate differently. Compare IMU instances / raw sources next.\n";
  else
    std::cout<<"  RESULT: mixed. Use the phase-reset blocks to determine whether source mismatch or estimator corrections dominate.\n";
  return 0;
}
