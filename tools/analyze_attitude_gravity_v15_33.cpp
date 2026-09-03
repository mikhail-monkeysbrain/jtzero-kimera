// JT-ZERO Stage 11 diagnostic v15.33
// Offline analysis of v15.32: compare FC quaternion body-Z tilt with measured gravity direction.
// No Kimera, no physical rerun.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr double PI = 3.14159265358979323846;
constexpr double ACC_LP_TAU = 0.25;

struct Phase { std::string name; int64_t a=0,b=0; double target_yaw=0; };
struct Euler { int64_t t=0; double rr=0,rp=0,ry=0; };
struct Quat { int64_t t=0; double tilt=0, rr=0,rp=0,ry=0; };
struct Imu { int64_t t=0; double ax=0,ay=0,az=0,gx=0,gy=0,gz=0; double acc_tilt=0; };

std::vector<std::string> split(const std::string& s){
  std::vector<std::string> o; std::stringstream ss(s); std::string x;
  while(std::getline(ss,x,',')) o.push_back(x); return o;
}
double deg(double r){ return r*180.0/PI; }
double wrap180(double x){ while(x>180)x-=360; while(x<-180)x+=360; return x; }
double vecTilt(double ax,double ay,double az,double bx,double by,double bz){
  const double na=std::sqrt(ax*ax+ay*ay+az*az), nb=std::sqrt(bx*bx+by*by+bz*bz);
  if(na<1e-9 || nb<1e-9) return 0;
  double d=(ax*bx+ay*by+az*bz)/(na*nb); d=std::clamp(d,-1.0,1.0); return deg(std::acos(d));
}
double pct(std::vector<double> v,double q){
  if(v.empty()) return 0; std::sort(v.begin(),v.end());
  double p=q*(v.size()-1); size_t lo=(size_t)std::floor(p), hi=std::min(lo+1,v.size()-1); double f=p-lo;
  return v[lo]*(1-f)+v[hi]*f;
}
double maxv(const std::vector<double>& v){ return v.empty()?0:*std::max_element(v.begin(),v.end()); }
double maxabs(const std::vector<double>& v){ double m=0; for(double x:v)m=std::max(m,std::abs(x)); return m; }

template<class T> std::vector<T> inPhase(const std::vector<T>& v,const Phase& p){
  std::vector<T> o; for(const auto& x:v) if(x.t>=p.a && x.t<p.b) o.push_back(x); return o;
}

std::vector<Phase> loadPhases(const std::string& f){
  std::ifstream in(f); if(!in) throw std::runtime_error("cannot open phases: "+f);
  std::string l; std::getline(in,l); std::vector<Phase> v;
  while(std::getline(in,l)){ if(l.empty())continue; auto c=split(l); if(c.size()<4)continue; v.push_back({c[0],std::stoll(c[1]),std::stoll(c[2]),std::stod(c[3])}); }
  if(v.empty()) throw std::runtime_error("no phases"); return v;
}
std::vector<Euler> loadEuler(const std::string& f){
  std::ifstream in(f); if(!in) throw std::runtime_error("cannot open Euler: "+f); std::string l; std::getline(in,l); std::vector<Euler> v;
  while(std::getline(in,l)){auto c=split(l); if(c.size()<8)continue; v.push_back({std::stoll(c[0]),std::stod(c[5]),std::stod(c[6]),std::stod(c[7])});} return v;
}
std::vector<Quat> loadQuat(const std::string& f){
  std::ifstream in(f); if(!in) throw std::runtime_error("cannot open quaternion: "+f); std::string l; std::getline(in,l); std::vector<Quat> v;
  while(std::getline(in,l)){auto c=split(l); if(c.size()<10)continue; v.push_back({std::stoll(c[0]),std::stod(c[6]),std::stod(c[7]),std::stod(c[8]),std::stod(c[9])});} return v;
}
std::vector<Imu> loadImu(const std::string& f){
  std::ifstream in(f); if(!in) throw std::runtime_error("cannot open IMU: "+f); std::string l; std::getline(in,l); std::vector<Imu> v;
  while(std::getline(in,l)){auto c=split(l); if(c.size()<8)continue; v.push_back({std::stoll(c[0]),std::stod(c[2]),std::stod(c[3]),std::stod(c[4]),std::stod(c[5]),std::stod(c[6]),std::stod(c[7]),0});} return v;
}

void replayAccTilt(std::vector<Imu>& im,const std::vector<Phase>& ph){
  const int64_t t0=ph.front().a;
  double a0x=0,a0y=0,a0z=0; size_t n0=0;
  for(const auto& s:im) if(s.t<t0){a0x+=s.ax;a0y+=s.ay;a0z+=s.az;++n0;}
  if(n0<20) throw std::runtime_error("too few pre-phase IMU samples for gravity baseline");
  a0x/=n0;a0y/=n0;a0z/=n0;
  double lx=0,ly=0,lz=0; bool init=false; int64_t last=0;
  for(auto& s:im){
    if(!init){lx=s.ax;ly=s.ay;lz=s.az;last=s.t;init=true;}
    else { double dt=std::max(0.0,(s.t-last)*1e-9), alpha=dt/(ACC_LP_TAU+dt); lx+=alpha*(s.ax-lx);ly+=alpha*(s.ay-ly);lz+=alpha*(s.az-lz);last=s.t; }
    s.acc_tilt=vecTilt(a0x,a0y,a0z,lx,ly,lz);
  }
  std::cout<<"Gravity baseline: ["<<std::fixed<<std::setprecision(4)<<a0x<<", "<<a0y<<", "<<a0z<<"] m/s^2  |g|="<<std::sqrt(a0x*a0x+a0y*a0y+a0z*a0z)<<"\n";
}

struct Summary { double qmed=0,qp95=0,qmax=0,amed=0,ap95=0,amax=0,dmed=0,dp95=0; };
Summary summarize(const std::vector<Quat>& q,const std::vector<Imu>& im){
  std::vector<double> qv,av,dv;
  for(auto&s:q)qv.push_back(s.tilt); for(auto&s:im)av.push_back(s.acc_tilt);
  // nearest-time comparison, quaternion ~=50 Hz, IMU ~=200 Hz
  size_t j=0; for(auto&s:q){while(j+1<im.size() && std::llabs(im[j+1].t-s.t)<std::llabs(im[j].t-s.t))++j;if(j<im.size())dv.push_back(std::abs(s.tilt-im[j].acc_tilt));}
  return {pct(qv,.5),pct(qv,.95),maxv(qv),pct(av,.5),pct(av,.95),maxv(av),pct(dv,.5),pct(dv,.95)};
}

} // namespace

int main(int argc,char**argv){
  const std::string e=argc>1?argv[1]:"/home/vio/jtzero_attitude_v15_32.csv";
  const std::string q=argc>2?argv[2]:"/home/vio/jtzero_quaternion_v15_32.csv";
  const std::string i=argc>3?argv[3]:"/home/vio/jtzero_imu_v15_32.csv";
  const std::string p=argc>4?argv[4]:"/home/vio/jtzero_phases_v15_32.csv";
  try{
    auto ph=loadPhases(p); auto ev=loadEuler(e); auto qv=loadQuat(q); auto iv=loadImu(i); replayAccTilt(iv,ph);
    std::cout<<"Rows: Euler="<<ev.size()<<" Quaternion="<<qv.size()<<" IMU="<<iv.size()<<"\n\n";
    std::cout<<"================ V15.33 PHASE ANALYSIS ================\n";
    std::cout<<std::left<<std::setw(12)<<"PHASE"<<std::right
             <<std::setw(9)<<"nQ"<<std::setw(9)<<"nIMU"
             <<std::setw(10)<<"Qmed"<<std::setw(10)<<"Qp95"<<std::setw(10)<<"Qmax"
             <<std::setw(10)<<"Amed"<<std::setw(10)<<"Ap95"<<std::setw(10)<<"Amax"
             <<std::setw(10)<<"Dmed"<<std::setw(10)<<"Dp95"
             <<std::setw(10)<<"|R|max"<<std::setw(10)<<"|P|max"<<std::setw(10)<<"dYaw"<<"\n";
    std::map<std::string,Summary> sm;
    for(const auto& x:ph){
      auto ee=inPhase(ev,x), qq=inPhase(qv,x), ii=inPhase(iv,x); auto s=summarize(qq,ii); sm[x.name]=s;
      std::vector<double> rv,pv; for(auto&a:ee){rv.push_back(a.rr);pv.push_back(a.rp);} double dy=ee.size()>1?wrap180(ee.back().ry-ee.front().ry):0;
      std::cout<<std::left<<std::setw(12)<<x.name<<std::right<<std::setw(9)<<qq.size()<<std::setw(9)<<ii.size()
               <<std::fixed<<std::setprecision(2)
               <<std::setw(10)<<s.qmed<<std::setw(10)<<s.qp95<<std::setw(10)<<s.qmax
               <<std::setw(10)<<s.amed<<std::setw(10)<<s.ap95<<std::setw(10)<<s.amax
               <<std::setw(10)<<s.dmed<<std::setw(10)<<s.dp95
               <<std::setw(10)<<maxabs(rv)<<std::setw(10)<<maxabs(pv)<<std::setw(10)<<dy<<"\n";
    }

    // Primary verdict uses the stationary 90-degree hold, avoiding rotational acceleration contamination.
    if(!sm.count("STILL90")) throw std::runtime_error("STILL90 phase missing");
    const auto s=sm.at("STILL90");
    std::string verdict;
    if(s.qmed>=3.0 && s.amed<=2.5 && s.dmed>=3.0)
      verdict="FC_ATTITUDE_TILT_WITHOUT_GRAVITY_TILT";
    else if(s.qmed>=3.0 && s.amed>=2.5 && s.dmed<=3.0)
      verdict="PHYSICAL_OR_IMU_MEASURED_TILT";
    else if(s.qmed<2.5 && s.amed<2.5)
      verdict="NO_SIGNIFICANT_TILT_AT_STILL90";
    else
      verdict="INCONCLUSIVE_MISMATCH";

    std::cout<<"\n================ V15.33 VERDICT ================\n";
    std::cout<<"STILL90 median BODY-Z tilt : "<<s.qmed<<" deg\n";
    std::cout<<"STILL90 median ACC tilt    : "<<s.amed<<" deg\n";
    std::cout<<"STILL90 median disagreement: "<<s.dmed<<" deg\n";
    std::cout<<"RESULT: "<<verdict<<"\n";
    std::cout<<"NOTE: verdict is based on stationary STILL90, not on transient yaw acceleration.\n";
    return 0;
  }catch(const std::exception& ex){std::cerr<<"[FATAL] "<<ex.what()<<"\n";return 1;}
}
