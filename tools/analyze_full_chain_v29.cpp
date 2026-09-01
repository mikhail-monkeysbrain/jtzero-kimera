// JT-ZERO v29: timestamp-based ATTITUDE/HIGHRES_IMU shift analyzer from v26 CSV.
// Uses the existing v26 run. No new physical run required.
// Scans a continuous FC-attitude time shift and evaluates cumulative gyro-vs-FC
// rotation/gravity error using BG_static.

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace {
constexpr double kPi=3.14159265358979323846;
struct Row{int phase=0;uint64_t us=0;double dt=0;Eigen::Vector3d gyr=Eigen::Vector3d::Zero();Eigen::Vector3d bg=Eigen::Vector3d::Zero();Eigen::Vector3d fc=Eigen::Vector3d::Zero();};
std::vector<std::string> split(const std::string&s){std::vector<std::string>o;std::stringstream ss(s);std::string x;while(std::getline(ss,x,','))o.push_back(x);return o;}
double val(const std::vector<std::string>&v,const std::unordered_map<std::string,size_t>&h,const std::string&k){auto it=h.find(k);if(it==h.end()||it->second>=v.size()||v[it->second].empty())return 0;return std::stod(v[it->second]);}
Eigen::Matrix3d fcR(const Eigen::Vector3d&e_deg){Eigen::Vector3d e=e_deg*(kPi/180.0);return Eigen::AngleAxisd(e.x(),Eigen::Vector3d::UnitX()).toRotationMatrix()*Eigen::AngleAxisd(e.y(),Eigen::Vector3d::UnitY()).toRotationMatrix()*Eigen::AngleAxisd(e.z(),Eigen::Vector3d::UnitZ()).toRotationMatrix();}
Eigen::Vector3d logR(const Eigen::Matrix3d&R){Eigen::AngleAxisd aa(R);if(!std::isfinite(aa.angle()))return Eigen::Vector3d::Zero();return aa.axis()*aa.angle();}
double rotDeg(const Eigen::Matrix3d&A,const Eigen::Matrix3d&B){return logR(A.transpose()*B).norm()*180.0/kPi;}
double gravDeg(const Eigen::Matrix3d&A,const Eigen::Matrix3d&B){Eigen::Vector3d a=A.transpose()*Eigen::Vector3d::UnitZ(),b=B.transpose()*Eigen::Vector3d::UnitZ();double d=std::clamp(a.dot(b),-1.0,1.0);return std::acos(d)*180.0/kPi;}

bool interpFc(const std::vector<Row>&r,double target_us,Eigen::Matrix3d*out){if(r.size()<2||target_us<r.front().us||target_us>r.back().us)return false;auto it=std::lower_bound(r.begin(),r.end(),target_us,[](const Row&a,double t){return double(a.us)<t;});if(it==r.begin()){*out=fcR(it->fc);return true;}if(it==r.end())return false;size_t i1=size_t(it-r.begin()),i0=i1-1;double t0=r[i0].us,t1=r[i1].us;if(t1<=t0){*out=fcR(r[i1].fc);return true;}double a=(target_us-t0)/(t1-t0);a=std::clamp(a,0.0,1.0);Eigen::Quaterniond q0(fcR(r[i0].fc)),q1(fcR(r[i1].fc));*out=q0.slerp(a,q1).normalized().toRotationMatrix();return true;}

struct M{double rmsR=0,maxR=0,rmsG=0,maxG=0,finalR=0,finalG=0;size_t n=0;};
M eval(const std::vector<Row>&r,double shift_us){M m;if(r.size()<2)return m;size_t start=0;Eigen::Matrix3d F0;while(start<r.size()&&!interpFc(r,double(r[start].us)+shift_us,&F0))++start;if(start>=r.size())return m;Eigen::Matrix3d R=F0;double ssR=0,ssG=0;for(size_t i=start;i<r.size();++i){Eigen::Matrix3d F;if(!interpFc(r,double(r[i].us)+shift_us,&F))break;if(i>start&&r[i].dt>0.001&&r[i].dt<0.02){Eigen::Vector3d w=r[i].gyr-r[i].bg,th=w*r[i].dt;double a=th.norm();if(a>1e-12)R=R*Eigen::AngleAxisd(a,th/a).toRotationMatrix();}double er=rotDeg(F,R),eg=gravDeg(F,R);ssR+=er*er;ssG+=eg*eg;m.maxR=std::max(m.maxR,er);m.maxG=std::max(m.maxG,eg);m.finalR=er;m.finalG=eg;++m.n;}if(m.n){m.rmsR=std::sqrt(ssR/m.n);m.rmsG=std::sqrt(ssG/m.n);}return m;}
}

int main(int argc,char**argv){if(argc<2){std::cerr<<"usage: "<<argv[0]<<" /home/vio/jtzero_live_full_chain_v26.csv\n";return 2;}std::ifstream f(argv[1]);if(!f){std::cerr<<"cannot open "<<argv[1]<<"\n";return 2;}std::string line;if(!std::getline(f,line))return 2;auto hh=split(line);std::unordered_map<std::string,size_t>h;for(size_t i=0;i<hh.size();++i)h[hh[i]]=i;std::vector<Row>r;while(std::getline(f,line)){if(line.empty())continue;auto v=split(line);try{Row q;q.phase=int(val(v,h,"phase"));q.us=uint64_t(val(v,h,"imu_us"));q.dt=val(v,h,"dt");q.gyr<<val(v,h,"gx"),val(v,h,"gy"),val(v,h,"gz");q.bg<<val(v,h,"bg_static_x"),val(v,h,"bg_static_y"),val(v,h,"bg_static_z");q.fc<<val(v,h,"fc_roll"),val(v,h,"fc_pitch"),val(v,h,"fc_yaw");r.push_back(q);}catch(...){}}if(r.size()<20){std::cerr<<"too few rows\n";return 2;}
std::cout<<std::fixed<<std::setprecision(9);std::cout<<"============================================================\nJT-ZERO TIMESTAMP SHIFT / CUMULATIVE ORIENTATION ANALYZER v29\n============================================================\n";std::cout<<"rows="<<r.size()<<"\n";
M zero=eval(r,0.0),best;double bestShift=0;best.rmsG=std::numeric_limits<double>::infinity();for(int us=-30000;us<=30000;us+=500){M m=eval(r,double(us));if(m.n>100&&m.rmsG<best.rmsG){best=m;bestShift=us;}}
auto p=[](const char*n,double sh,const M&m){std::cout<<"\n"<<n<<" shift="<<sh/1000.0<<" ms\n"<<"  RMS rot="<<m.rmsR<<" deg  max rot="<<m.maxR<<" deg\n"<<"  RMS grav="<<m.rmsG<<" deg max grav="<<m.maxG<<" deg\n"<<"  final rot="<<m.finalR<<" deg final grav="<<m.finalG<<" deg n="<<m.n<<"\n";};p("SHIFT 0",0,zero);p("BEST",bestShift,best);M p10=eval(r,10000.0),m10=eval(r,-10000.0);p("SHIFT +10",10000,p10);p("SHIFT -10",-10000,m10);
std::cout<<"\nDecision helper:\n";double ratio=best.rmsG/std::max(1e-12,zero.rmsG);std::cout<<"  best shift="<<bestShift/1000.0<<" ms gravity RMS ratio="<<ratio<<"\n";if(ratio<0.70)std::cout<<"  RESULT: constant ATTITUDE/IMU time shift explains a large fraction of cumulative gravity-direction error.\n";else if(ratio<0.90)std::cout<<"  RESULT: timing contributes materially but does not explain the whole cumulative error.\n";else std::cout<<"  RESULT: timing improves instantaneous rates but not cumulative orientation enough; estimator/source semantics remain primary.\n";return 0;}
