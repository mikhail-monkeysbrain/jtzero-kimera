// JT-ZERO v45: offline multi-hypothesis analysis of v44 master CSV.
// No physical test. Uses variance-based static windows and compares gyro-bias models.

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr double G = 9.81;
constexpr double CX = 0.014570;
constexpr double CY = 0.082383;
constexpr double PI = 3.14159265358979323846;

struct Row {
  int stage_id=0; std::string stage; uint64_t us=0; double dt=0;
  Eigen::Vector3d a=Eigen::Vector3d::Zero();
  Eigen::Vector3d w=Eigen::Vector3d::Zero(); // v44 raw_g*: already initial-BG corrected
  Eigen::Vector3d fc_rpy=Eigen::Vector3d::Zero();
};
struct Win { size_t b=0,e=0; std::string stage; Eigen::Vector3d ma=Eigen::Vector3d::Zero(), mw=Eigen::Vector3d::Zero(); double acc_std=0,gyro_std=0; };
struct Model { std::string name; int mode=0; double tau=0; };
struct Result { std::string name; double final_gravity_deg=0, final_axy=0, static_dv=0, endpoint_rms=0, endpoint_max=0; size_t static_samples=0; };

std::vector<std::string> split(const std::string& s){std::vector<std::string> v;std::stringstream ss(s);std::string x;while(std::getline(ss,x,','))v.push_back(x);return v;}
double d(const std::string&s){return s.empty()?0.0:std::stod(s);} 
Eigen::Matrix3d expSO3(const Eigen::Vector3d& q){double a=q.norm();if(a<1e-12)return Eigen::Matrix3d::Identity();return Eigen::AngleAxisd(a,q/a).toRotationMatrix();}
double angleDeg(const Eigen::Vector3d&a,const Eigen::Vector3d&b){double c=a.normalized().dot(b.normalized());c=std::max(-1.0,std::min(1.0,c));return std::acos(c)*180.0/PI;}
Eigen::Matrix3d fcR(double r,double p,double y){return Eigen::AngleAxisd(y,Eigen::Vector3d::UnitZ()).toRotationMatrix()*Eigen::AngleAxisd(p,Eigen::Vector3d::UnitY()).toRotationMatrix()*Eigen::AngleAxisd(r,Eigen::Vector3d::UnitX()).toRotationMatrix();}

std::vector<Row> load(const std::string& path){
 std::ifstream f(path); if(!f)throw std::runtime_error("cannot open "+path); std::string line; std::getline(f,line); auto h=split(line); std::map<std::string,int> c;for(int i=0;i<(int)h.size();++i)c[h[i]]=i;
 const char* need[]={"stage_id","stage","imu_us","dt","flu_ax","flu_ay","flu_az","raw_gx","raw_gy","raw_gz","fc_roll","fc_pitch","fc_yaw"};for(auto n:need)if(!c.count(n))throw std::runtime_error(std::string("missing column: ")+n);
 std::vector<Row> v; while(std::getline(f,line)){if(line.empty())continue;auto x=split(line);if(x.size()<h.size())continue;Row r;r.stage_id=(int)d(x[c["stage_id"]]);r.stage=x[c["stage"]];r.us=(uint64_t)std::stoull(x[c["imu_us"]]);r.dt=d(x[c["dt"]]);r.a={d(x[c["flu_ax"]]),d(x[c["flu_ay"]]),d(x[c["flu_az"]])};r.w={d(x[c["raw_gx"]]),d(x[c["raw_gy"]]),d(x[c["raw_gz"]])};r.fc_rpy={d(x[c["fc_roll"]]),d(x[c["fc_pitch"]]),d(x[c["fc_yaw"]])};v.push_back(r);}return v;
}

std::vector<bool> staticMask(const std::vector<Row>&v){
 // 1.0 s centered windows at ~200 Hz. Selection uses variance, NOT mean gyro.
 const int H=100; std::vector<bool> m(v.size(),false); if(v.size()<201)return m;
 std::vector<Eigen::Vector3d> pa(v.size()+1,Eigen::Vector3d::Zero()),pw=pa,pa2=pa,pw2=pa;
 for(size_t i=0;i<v.size();++i){pa[i+1]=pa[i]+v[i].a;pw[i+1]=pw[i]+v[i].w;pa2[i+1]=pa2[i]+v[i].a.cwiseProduct(v[i].a);pw2[i+1]=pw2[i]+v[i].w.cwiseProduct(v[i].w);}
 for(size_t i=H;i+H<v.size();++i){size_t b=i-H,e=i+H,n=e-b;auto ma=(pa[e]-pa[b])/double(n),mw=(pw[e]-pw[b])/double(n);auto va=(pa2[e]-pa2[b])/double(n)-ma.cwiseProduct(ma);auto vw=(pw2[e]-pw2[b])/double(n)-mw.cwiseProduct(mw);double as=std::sqrt(std::max(0.0,va.sum())),ws=std::sqrt(std::max(0.0,vw.sum()));double an=ma.norm(); if(ws<0.0030 && as<0.16 && std::abs(an-G)<0.30)m[i]=true; }
 return m;
}

std::vector<Win> windows(const std::vector<Row>&v,const std::vector<bool>&m){
 std::vector<Win> out;size_t i=0;while(i<v.size()){while(i<v.size()&&!m[i])++i;if(i==v.size())break;size_t b=i;std::string st=v[i].stage;while(i<v.size()&&m[i]&&v[i].stage==st)++i;size_t e=i;if(e-b<100)continue;Win w;w.b=b;w.e=e;w.stage=st;for(size_t k=b;k<e;++k){w.ma+=v[k].a;w.mw+=v[k].w;}w.ma/=double(e-b);w.mw/=double(e-b);double av=0,gv=0;for(size_t k=b;k<e;++k){av+=(v[k].a-w.ma).squaredNorm();gv+=(v[k].w-w.mw).squaredNorm();}w.acc_std=std::sqrt(av/double(e-b));w.gyro_std=std::sqrt(gv/double(e-b));out.push_back(w);}return out;
}

Result runModel(const std::vector<Row>&v,const std::vector<bool>&sm,const std::vector<Win>&wins,const Model&M){
 Result z;z.name=M.name;if(v.empty())return z;
 // v44 raw_g is already corrected by initial BG. Adaptive terms below estimate residual BG only.
 Eigen::Vector3d b=Eigen::Vector3d::Zero();
 Eigen::Vector3d a0=Eigen::Vector3d::Zero();size_t n0=0;for(size_t i=0;i<v.size()&&i<4000;++i)if(sm[i]){a0+=v[i].a;++n0;}if(!n0){a0=v.front().a;n0=1;}a0/=double(n0);
 // Initial orientation from gravity direction only; yaw is irrelevant to gravity-error metric.
 Eigen::Quaterniond q0=Eigen::Quaterniond::FromTwoVectors(a0.normalized(),Eigen::Vector3d(0,0,-1));
 Eigen::Matrix3d R=q0.toRotationMatrix(); Eigen::Vector3d vel=Eigen::Vector3d::Zero(); double sdv=0;
 std::vector<double> ep;
 size_t nextWin=0; bool prevStatic=false;
 for(size_t i=0;i<v.size();++i){
   const Row&r=v[i];double dt=(r.dt>0&&r.dt<0.03)?r.dt:0;
   Eigen::Vector3d w=r.w-b; w.x()+=CX*w.z();w.y()+=CY*w.z();
   if(dt>0)R=R*expSO3(w*dt);
   Eigen::Vector3d aw=R*r.a+Eigen::Vector3d(0,0,G);if(dt>0){vel+=aw*dt;if(sm[i])sdv+=aw.head<2>().norm()*dt;}
   // Bias update only from variance-confirmed static samples. Mean magnitude is not used for selection.
   if(sm[i]&&dt>0&&M.mode){double alpha=1.0-std::exp(-dt/M.tau);Eigen::Vector3d target=r.w;if(M.mode==1)b.x()+=alpha*(target.x()-b.x());else if(M.mode==2){b.x()+=alpha*(target.x()-b.x());b.y()+=alpha*(target.y()-b.y());}else b+=alpha*(target-b);}
   bool now=sm[i]; if(prevStatic&&!now){ // endpoint at end of static island
     size_t q=i-1;size_t bb=q;while(bb>0&&sm[bb-1]&&v[bb-1].stage==v[q].stage)--bb;Eigen::Vector3d ma=Eigen::Vector3d::Zero();size_t nn=0;for(size_t k=bb;k<=q;++k){ma+=v[k].a;++nn;}if(nn>20){ma/=double(nn);Eigen::Vector3d pred=R.transpose()*Eigen::Vector3d(0,0,-1);ep.push_back(angleDeg(pred,ma));}
   }prevStatic=now;
 }
 // final static mean accel and world horizontal acceleration
 Eigen::Vector3d ma=Eigen::Vector3d::Zero();size_t nn=0;for(size_t i=0;i<v.size();++i)if(sm[i]&&v[i].stage=="FINAL_STATIC"){ma+=v[i].a;++nn;}if(nn){ma/=double(nn);Eigen::Vector3d pred=R.transpose()*Eigen::Vector3d(0,0,-1);z.final_gravity_deg=angleDeg(pred,ma);z.final_axy=(R*ma+Eigen::Vector3d(0,0,G)).head<2>().norm();}
 z.static_dv=sdv;z.static_samples=std::count(sm.begin(),sm.end(),true);if(!ep.empty()){double s=0;for(double x:ep){s+=x*x;z.endpoint_max=std::max(z.endpoint_max,x);}z.endpoint_rms=std::sqrt(s/ep.size());}return z;
}

int main(int argc,char**argv){
 const std::string path=argc>1?argv[1]:"/home/vio/jtzero_imu_master_v44.csv";const std::string outp=argc>2?argv[2]:"/home/vio/jtzero_imu_master_v45_result.txt";
 try{auto v=load(path);auto sm=staticMask(v);auto ws=windows(v,sm);std::vector<Model> ms={{"CONST_BG + ZXY",0,1},{"STATIC_BG_X tau1",1,1},{"STATIC_BG_X tau2",1,2},{"STATIC_BG_X tau5",1,5},{"STATIC_BG_X tau10",1,10},{"STATIC_BG_XY tau5",2,5},{"STATIC_BG_XYZ tau5",3,5}};std::vector<Result> rr;for(auto&m:ms)rr.push_back(runModel(v,sm,ws,m));std::sort(rr.begin(),rr.end(),[](auto&a,auto&b){return a.final_gravity_deg<b.final_gravity_deg;});
 std::ostringstream o;o<<std::fixed<<std::setprecision(6);o<<"============================================================\nJT-ZERO v45 OFFLINE MASTER IMU ANALYZER\n============================================================\n";o<<"input: "<<path<<"\nrows: "<<v.size()<<"\nvariance-static samples: "<<std::count(sm.begin(),sm.end(),true)<<"\nstatic windows: "<<ws.size()<<"\n\nSTATIC WINDOWS (selection independent of mean gyro)\n";for(auto&w:ws)o<<w.stage<<" n="<<(w.e-w.b)<<" |a|="<<w.ma.norm()<<" gyro_mean=["<<w.mw.transpose()<<"] gyro_std="<<w.gyro_std<<" acc_std="<<w.acc_std<<"\n";
 o<<"\nMODEL RANKING\n"<<"model | final_gravity_deg | final_axy | endpoint_rms | endpoint_max | static_integral_abs_axy\n";for(auto&r:rr)o<<r.name<<" | "<<r.final_gravity_deg<<" | "<<r.final_axy<<" | "<<r.endpoint_rms<<" | "<<r.endpoint_max<<" | "<<r.static_dv<<"\n";
 o<<"\nCONTEXT CHECKS\n";o<<"v37: optimizer/GTSAM cannot explain error because independent SO(3) reproduced runaway.\n";o<<"v38: gyro-propagated attitude was dominant; ACC_RP strongly reduced residual.\n";o<<"v39: dt/midpoint/static-bias alone did not explain original yaw catastrophe.\n";o<<"v42: Z->XY correction independently validated; do not reject it from this single dataset.\n";o<<"v43: Z->XY sharply reduced live PIM/custom runaway but residual remained.\n";o<<"v44: final static continued accumulating false velocity; gravity-direction error remains diagnostic target.\n";
 o<<"\nINTERPRETATION RULES\n";o<<"1) Adaptive BG is supported only if it improves final gravity AND endpoint RMS/max without contradicting v42.\n";o<<"2) X-only beating XY/XYZ supports dominant effective X-bias; otherwise do not claim it.\n";o<<"3) Large variation of static-window gyro means with low std supports changing effective bias/upstream processing.\n";o<<"4) If all adaptive models fail, next master test must prioritize temperature, full gyro matrix, rate/history terms and accel calibration.\n";
 std::ofstream fo(outp);fo<<o.str();std::cout<<o.str()<<"\nResult TXT: "<<outp<<"\n";return 0;}catch(const std::exception&e){std::cerr<<"[FATAL] "<<e.what()<<"\n";return 1;}
}
