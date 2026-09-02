// JT-ZERO v45.1: exact offline replay validation of v44 recorded FIXED branch.
// No physical test. Verifies CSV semantics before any further hypothesis fitting.

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr double CX=0.014570, CY=0.082383, PI=3.14159265358979323846;
struct Row{std::string stage;double dt=0;Eigen::Vector3d a=Eigen::Vector3d::Zero(),wr=Eigen::Vector3d::Zero(),wf=Eigen::Vector3d::Zero(),er=Eigen::Vector3d::Zero(),aw=Eigen::Vector3d::Zero(),v=Eigen::Vector3d::Zero();};
std::vector<std::string> split(const std::string&s){std::vector<std::string>v;std::stringstream ss(s);std::string x;while(std::getline(ss,x,','))v.push_back(x);return v;}
double num(const std::string&s){return s.empty()?0.0:std::stod(s);}
Eigen::Matrix3d expSO3(const Eigen::Vector3d&q){double a=q.norm();if(a<1e-14)return Eigen::Matrix3d::Identity();return Eigen::AngleAxisd(a,q/a).toRotationMatrix();}
Eigen::Matrix3d fromEulerXYZdeg(const Eigen::Vector3d&e){Eigen::Vector3d r=e*PI/180.0;return Eigen::AngleAxisd(r.x(),Eigen::Vector3d::UnitX()).toRotationMatrix()*Eigen::AngleAxisd(r.y(),Eigen::Vector3d::UnitY()).toRotationMatrix()*Eigen::AngleAxisd(r.z(),Eigen::Vector3d::UnitZ()).toRotationMatrix();}
double rotErrDeg(const Eigen::Matrix3d&A,const Eigen::Matrix3d&B){double c=( (A.transpose()*B).trace()-1.0 )*0.5;c=std::max(-1.0,std::min(1.0,c));return std::acos(c)*180.0/PI;}
std::vector<Row> load(const std::string&p){std::ifstream f(p);if(!f)throw std::runtime_error("cannot open "+p);std::string line;std::getline(f,line);auto h=split(line);std::map<std::string,int>c;for(int i=0;i<(int)h.size();++i)c[h[i]]=i;const char* need[]={"stage","dt","flu_ax","flu_ay","flu_az","raw_gx","raw_gy","raw_gz","fixed_gx","fixed_gy","fixed_gz","fixed_R_roll","fixed_R_pitch","fixed_R_yaw","fixed_aw_x","fixed_aw_y","fixed_aw_z","fixed_vx","fixed_vy","fixed_vz"};for(auto n:need)if(!c.count(n))throw std::runtime_error(std::string("missing column: ")+n);std::vector<Row>v;while(std::getline(f,line)){if(line.empty())continue;auto x=split(line);if(x.size()<h.size())continue;Row r;r.stage=x[c["stage"]];r.dt=num(x[c["dt"]]);r.a={num(x[c["flu_ax"]]),num(x[c["flu_ay"]]),num(x[c["flu_az"]])};r.wr={num(x[c["raw_gx"]]),num(x[c["raw_gy"]]),num(x[c["raw_gz"]])};r.wf={num(x[c["fixed_gx"]]),num(x[c["fixed_gy"]]),num(x[c["fixed_gz"]])};r.er={num(x[c["fixed_R_roll"]]),num(x[c["fixed_R_pitch"]]),num(x[c["fixed_R_yaw"]])};r.aw={num(x[c["fixed_aw_x"]]),num(x[c["fixed_aw_y"]]),num(x[c["fixed_aw_z"]])};r.v={num(x[c["fixed_vx"]]),num(x[c["fixed_vy"]]),num(x[c["fixed_vz"]])};v.push_back(r);}return v;}
}

int main(int argc,char**argv){
 const std::string in=argc>1?argv[1]:"/home/vio/jtzero_imu_master_v44.csv";
 const std::string out=argc>2?argv[2]:"/home/vio/jtzero_imu_replay_v45_1.txt";
 try{
  auto v=load(in);if(v.size()<2)throw std::runtime_error("too few rows");
  size_t s=0;while(s<v.size()&&v[s].stage=="CALIB_STATIC")++s;if(s>=v.size())throw std::runtime_error("no post-calibration rows");
  // In v44 CALIB_STATIC rows are written BEFORE SPACE sets cal.bg/cal.ba/R0.
  // Therefore raw_g during CALIB_STATIC is not comparable to post-calibration raw_g.
  double maxFixFormula=0,rmsFixFormula=0;size_t nf=0;
  for(size_t i=s;i<v.size();++i){Eigen::Vector3d e=v[i].wr;e.x()+=CX*e.z();e.y()+=CY*e.z();double q=(e-v[i].wf).norm();maxFixFormula=std::max(maxFixFormula,q);rmsFixFormula+=q*q;++nf;}
  rmsFixFormula=std::sqrt(rmsFixFormula/std::max<size_t>(1,nf));

  // Anchor replay to the first recorded post-calibration FIXED orientation.
  // Then replay the exact v44 recurrence from subsequent rows.
  Eigen::Matrix3d R=fromEulerXYZdeg(v[s].er);double maxRot=0,rmsRot=0;size_t nr=0;
  for(size_t i=s+1;i<v.size();++i){double dt=(v[i].dt>0&&v[i].dt<0.03)?v[i].dt:0;if(dt>0)R=R*expSO3(v[i].wf*dt);Eigen::Matrix3d Rcsv=fromEulerXYZdeg(v[i].er);double e=rotErrDeg(R,Rcsv);maxRot=std::max(maxRot,e);rmsRot+=e*e;++nr;}
  rmsRot=std::sqrt(rmsRot/std::max<size_t>(1,nr));

  // Velocity replay directly from the already-recorded v44 world acceleration.
  Eigen::Vector3d vel=v[s].v;double maxVel=0,rmsVel=0;size_t nv=0;
  for(size_t i=s+1;i<v.size();++i){double dt=(v[i].dt>0&&v[i].dt<0.03)?v[i].dt:0;if(dt>0)vel+=v[i].aw*dt;double e=(vel-v[i].v).norm();maxVel=std::max(maxVel,e);rmsVel+=e*e;++nv;}
  rmsVel=std::sqrt(rmsVel/std::max<size_t>(1,nv));

  // Infer the constant BA that v44 used from recorded R, accel and world accel.
  // aw = R*(a-BA)+gN -> BA = a - R^T*(aw-gN).
  Eigen::Vector3d ba=Eigen::Vector3d::Zero();size_t nb=0;double baSpread=0;
  for(size_t i=s;i<v.size();++i){Eigen::Matrix3d Rc=fromEulerXYZdeg(v[i].er);Eigen::Vector3d ac=Rc.transpose()*(v[i].aw-Eigen::Vector3d(0,0,9.81));ba+=v[i].a-ac;++nb;}ba/=double(nb);
  for(size_t i=s;i<v.size();++i){Eigen::Matrix3d Rc=fromEulerXYZdeg(v[i].er);Eigen::Vector3d ac=Rc.transpose()*(v[i].aw-Eigen::Vector3d(0,0,9.81));Eigen::Vector3d bi=v[i].a-ac;baSpread+=(bi-ba).squaredNorm();}baSpread=std::sqrt(baSpread/double(nb));

  std::ostringstream o;o<<std::fixed<<std::setprecision(9);
  o<<"============================================================\nJT-ZERO v45.1 EXACT REPLAY VALIDATION\n============================================================\n";
  o<<"input: "<<in<<"\nrows: "<<v.size()<<"\nfirst post-cal row: "<<s<<" stage="<<v[s].stage<<"\n\n";
  o<<"IMPORTANT SEMANTICS\nCALIB_STATIC CSV rows were recorded before SPACE finalized BG/BA/R0.\nTherefore CALIB_STATIC raw_g is raw FLU gyro (cal.bg was still zero).\nPost-calibration raw_g is FLU gyro minus startup BG.\n\n";
  o<<"REPLAY CHECKS\n";
  o<<"fixed gyro formula RMS error [rad/s]: "<<rmsFixFormula<<"\n";
  o<<"fixed gyro formula MAX error [rad/s]: "<<maxFixFormula<<"\n";
  o<<"orientation replay RMS error [deg]: "<<rmsRot<<"\n";
  o<<"orientation replay MAX error [deg]: "<<maxRot<<"\n";
  o<<"velocity replay RMS error [m/s]: "<<rmsVel<<"\n";
  o<<"velocity replay MAX error [m/s]: "<<maxVel<<"\n";
  o<<"inferred BA: ["<<ba.transpose()<<"] m/s^2\n";
  o<<"inferred BA RMS spread: "<<baSpread<<" m/s^2\n\n";
  const bool pass=rmsFixFormula<2e-6&&maxFixFormula<1e-5&&rmsRot<0.01&&maxRot<0.05&&rmsVel<1e-5&&maxVel<5e-5;
  o<<"REPLAY RESULT: "<<(pass?"PASS":"FAIL")<<"\n";
  o<<"If PASS, future offline A/B must start at the post-calibration boundary, use recorded startup state semantics, and subtract the v44 BA.\n";
  o<<"If FAIL, do not interpret v45 adaptive-bias rankings until the replay mismatch is fixed.\n";
  std::ofstream f(out);f<<o.str();std::cout<<o.str()<<"\nResult TXT: "<<out<<"\n";return pass?0:2;
 }catch(const std::exception&e){std::cerr<<"[FATAL] "<<e.what()<<"\n";return 1;}
}
