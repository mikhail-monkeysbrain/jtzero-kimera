// JT-ZERO v33: offline accelerometer residual / yaw lever-arm analyzer for v26 CSV.
// No physical test required.
// Build:
//   g++ -std=c++17 -O2 tools/analyze_accel_residual_v33.cpp -o /tmp/analyze_accel_residual_v33 -I/usr/include/eigen3
// Run:
//   /tmp/analyze_accel_residual_v33 /home/vio/jtzero_live_full_chain_v26.csv

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace {
constexpr double kPi=3.141592653589793238462643383279502884;
constexpr double kG=9.81;
constexpr double kDynWz=0.03;
constexpr double kMaxDt=0.03;

struct Row {
  int phase=0;
  std::string phase_name;
  double dt=0;
  Eigen::Vector3d acc=Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro=Eigen::Vector3d::Zero();
  Eigen::Vector3d bg_static=Eigen::Vector3d::Zero();
  Eigen::Matrix3d Rfc=Eigen::Matrix3d::Identity();
};

struct Accum {
  size_t n=0;
  Eigen::Vector2d sum=Eigen::Vector2d::Zero();
  Eigen::Vector2d ss=Eigen::Vector2d::Zero();
  Eigen::Vector2d dv=Eigen::Vector2d::Zero();
  double ssnorm=0,maxnorm=0,duration=0;
  void add(const Eigen::Vector2d&a,double dt){
    ++n;sum+=a;ss+=a.cwiseProduct(a);ssnorm+=a.squaredNorm();maxnorm=std::max(maxnorm,a.norm());
    if(dt>0&&dt<=kMaxDt){dv+=a*dt;duration+=dt;}
  }
  Eigen::Vector2d mean()const{Eigen::Vector2d x=Eigen::Vector2d::Zero();if(n)x=sum/double(n);return x;}
  Eigen::Vector2d rmsAxis()const{Eigen::Vector2d x=Eigen::Vector2d::Zero();if(n)x=(ss/double(n)).cwiseSqrt();return x;}
  double rmsNorm()const{return n?std::sqrt(ssnorm/double(n)):0.0;}
};

std::vector<std::string> splitCsv(const std::string&s){std::vector<std::string>o;std::string c;bool q=false;for(char ch:s){if(ch=='"')q=!q;else if(ch==','&&!q){o.push_back(c);c.clear();}else c.push_back(ch);}o.push_back(c);return o;}
double getd(const std::vector<std::string>&f,const std::unordered_map<std::string,size_t>&h,const char*n){auto it=h.find(n);if(it==h.end()||it->second>=f.size())throw std::runtime_error(std::string("missing column: ")+n);return std::stod(f[it->second]);}
std::string gets(const std::vector<std::string>&f,const std::unordered_map<std::string,size_t>&h,const char*n){auto it=h.find(n);if(it==h.end()||it->second>=f.size())throw std::runtime_error(std::string("missing column: ")+n);return f[it->second];}
Eigen::Matrix3d fcFromLoggedEuler(double xdeg,double ydeg,double zdeg){double x=xdeg*kPi/180.0,y=ydeg*kPi/180.0,z=zdeg*kPi/180.0;return Eigen::AngleAxisd(x,Eigen::Vector3d::UnitX()).toRotationMatrix()*Eigen::AngleAxisd(y,Eigen::Vector3d::UnitY()).toRotationMatrix()*Eigen::AngleAxisd(z,Eigen::Vector3d::UnitZ()).toRotationMatrix();}
Eigen::Matrix3d expR(const Eigen::Vector3d&v){double a=v.norm();if(a<1e-15)return Eigen::Matrix3d::Identity();return Eigen::AngleAxisd(a,v/a).toRotationMatrix();}

std::vector<Row> load(const std::string&path){
  std::ifstream in(path);if(!in)throw std::runtime_error("cannot open: "+path);std::string line;if(!std::getline(in,line))throw std::runtime_error("empty file");
  auto hh=splitCsv(line);std::unordered_map<std::string,size_t>h;for(size_t i=0;i<hh.size();++i)h[hh[i]]=i;std::vector<Row>r;
  while(std::getline(in,line)){if(line.empty())continue;auto f=splitCsv(line);try{Row x;x.phase=(int)getd(f,h,"phase");x.phase_name=gets(f,h,"phase_name");x.dt=getd(f,h,"dt");x.acc<<getd(f,h,"ax"),getd(f,h,"ay"),getd(f,h,"az");x.gyro<<getd(f,h,"gx"),getd(f,h,"gy"),getd(f,h,"gz");x.bg_static<<getd(f,h,"bg_static_x"),getd(f,h,"bg_static_y"),getd(f,h,"bg_static_z");x.Rfc=fcFromLoggedEuler(getd(f,h,"fc_roll"),getd(f,h,"fc_pitch"),getd(f,h,"fc_yaw"));r.push_back(x);}catch(...){}}
  if(r.size()<100)throw std::runtime_error("too few valid rows");return r;
}

Eigen::Vector3d estimateStaticBA(const std::vector<Row>&r){
  // For a stationary body: R*(acc-BA)+gW=0 => BA=acc+R^T*gW.
  // Use early low-rate samples only, capped at about 3 s of valid dt.
  const Eigen::Vector3d gW(0,0,kG);Eigen::Vector3d s=Eigen::Vector3d::Zero();size_t n=0;double t=0;
  for(size_t i=0;i<r.size()&&t<3.0;++i){double wz=std::abs(r[i].gyro.z()-r[i].bg_static.z());double wn=(r[i].gyro-r[i].bg_static).norm();if(wn<0.025&&wz<0.015){s+=r[i].acc+r[i].Rfc.transpose()*gW;++n;}if(r[i].dt>0&&r[i].dt<=kMaxDt)t+=r[i].dt;}
  if(n<50)throw std::runtime_error("not enough initial still samples for BA_static estimate");return s/double(n);
}

struct SampleEval {Eigen::Vector2d afc=Eigen::Vector2d::Zero(),agyro=Eigen::Vector2d::Zero(),abody=Eigen::Vector2d::Zero();double w=0,w2=0,alpha=0,dt=0;int phase=0;bool dyn=false;};

void printAccum(const char*name,const Accum&a){std::cout<<"    "<<name<<": n="<<a.n<<" duration="<<a.duration<<" s mean=["<<a.mean().transpose()<<"] RMSaxis=["<<a.rmsAxis().transpose()<<"] RMS|aXY|="<<a.rmsNorm()<<" max="<<a.maxnorm<<" m/s^2 dVxy=["<<a.dv.transpose()<<"] m/s\n";}

double corr(const std::vector<double>&x,const std::vector<double>&y){if(x.size()!=y.size()||x.size()<3)return 0;double mx=std::accumulate(x.begin(),x.end(),0.0)/x.size(),my=std::accumulate(y.begin(),y.end(),0.0)/y.size();double sxx=0,syy=0,sxy=0;for(size_t i=0;i<x.size();++i){double a=x[i]-mx,b=y[i]-my;sxx+=a*a;syy+=b*b;sxy+=a*b;}return (sxx>0&&syy>0)?sxy/std::sqrt(sxx*syy):0;}

} // namespace

int main(int argc,char**argv){
  if(argc<2){std::cerr<<"Usage: "<<argv[0]<<" <v26 csv/txt>\n";return 2;}
  try{
    const auto r=load(argv[1]);const Eigen::Vector3d ba=estimateStaticBA(r);const Eigen::Vector3d bg=r.front().bg_static;const Eigen::Vector3d gW(0,0,kG);
    std::vector<SampleEval> e;e.reserve(r.size());Eigen::Matrix3d Rg=r.front().Rfc;double prevw=r.front().gyro.z()-bg.z();
    for(size_t i=0;i<r.size();++i){double dt=r[i].dt;if(i>0&&dt>0&&dt<=kMaxDt)Rg=Rg*expR((r[i].gyro-bg)*dt);Eigen::Vector3d f=r[i].acc-ba;Eigen::Vector3d awfc=r[i].Rfc*f+gW;Eigen::Vector3d awg=Rg*f+gW;Eigen::Vector3d ab=f+r[i].Rfc.transpose()*gW;double w=r[i].gyro.z()-bg.z();double al=(i>0&&dt>0&&dt<=kMaxDt)?(w-prevw)/dt:0;prevw=w;SampleEval q;q.afc=awfc.head<2>();q.agyro=awg.head<2>();q.abody=ab.head<2>();q.w=w;q.w2=w*w;q.alpha=al;q.dt=dt;q.phase=r[i].phase;q.dyn=std::abs(w)>kDynWz;e.push_back(q);}

    std::cout<<std::fixed<<std::setprecision(9);
    std::cout<<"============================================================\nJT-ZERO ACCEL RESIDUAL / YAW LEVER-ARM ANALYZER v33\n============================================================\n";
    std::cout<<"rows="<<r.size()<<" dynamic threshold |wz|>"<<kDynWz<<" rad/s\n";
    std::cout<<"BG_static=["<<bg.transpose()<<"] rad/s\nBA_static(est)=["<<ba.transpose()<<"] m/s^2\n";
    static const char*pn[4]={"+30","0_after_plus","-30","0_final"};
    for(int p=0;p<4;++p){Accum fcd,fcs,gyd,gys,bd,bs;for(const auto&q:e)if(q.phase==p){if(q.dyn){fcd.add(q.afc,q.dt);gyd.add(q.agyro,q.dt);bd.add(q.abody,q.dt);}else{fcs.add(q.afc,q.dt);gys.add(q.agyro,q.dt);bs.add(q.abody,q.dt);}}std::cout<<"\n"<<pn[p]<<"\n  FC attitude gravity compensation:\n";printAccum("dynamic",fcd);printAccum("still  ",fcs);std::cout<<"  Gyro-integrated gravity compensation:\n";printAccum("dynamic",gyd);printAccum("still  ",gys);std::cout<<"  Body residual using direct FC attitude:\n";printAccum("dynamic",bd);printAccum("still  ",bs);}

    // Fit a fixed XY lever arm r from body residual under yaw:
    // ax = -w^2*rx - alpha*ry ; ay = alpha*rx - w^2*ry.
    Eigen::MatrixXd A(2*e.size(),2);Eigen::VectorXd b(2*e.size());int m=0;std::vector<double>w2,ax,ay,al;
    for(const auto&q:e){if(!q.dyn||q.dt<=0||q.dt>kMaxDt||std::abs(q.alpha)>20.0)continue;A.row(m)<<-q.w2,-q.alpha;b[m++]=q.abody.x();A.row(m)<<q.alpha,-q.w2;b[m++]=q.abody.y();w2.push_back(q.w2);ax.push_back(q.abody.x());ay.push_back(q.abody.y());al.push_back(q.alpha);}
    A.conservativeResize(m,2);b.conservativeResize(m);Eigen::Vector2d lever=Eigen::Vector2d::Zero();if(m>=20)lever=A.colPivHouseholderQr().solve(b);double ss0=0,ss1=0;size_t nf=0;for(const auto&q:e){if(!q.dyn||q.dt<=0||q.dt>kMaxDt||std::abs(q.alpha)>20.0)continue;Eigen::Vector2d pred(-q.w2*lever.x()-q.alpha*lever.y(),q.alpha*lever.x()-q.w2*lever.y());ss0+=q.abody.squaredNorm();ss1+=(q.abody-pred).squaredNorm();++nf;}
    double rms0=nf?std::sqrt(ss0/nf):0,rms1=nf?std::sqrt(ss1/nf):0;
    std::cout<<"\nYaw lever-arm fit (dynamic samples):\n  r_xy=["<<lever.transpose()<<"] m  |r|="<<lever.norm()*1000.0<<" mm\n  body residual RMS before="<<rms0<<" after lever model="<<rms1<<" m/s^2 ratio="<<(rms0>0?rms1/rms0:0)<<"\n";
    std::cout<<"  corr(w^2, body_ax)="<<corr(w2,ax)<<" corr(w^2, body_ay)="<<corr(w2,ay)<<"\n  corr(alpha_z, body_ax)="<<corr(al,ax)<<" corr(alpha_z, body_ay)="<<corr(al,ay)<<"\n";

    Accum fcAll,gyAll;for(const auto&q:e)if(q.dyn){fcAll.add(q.afc,q.dt);gyAll.add(q.agyro,q.dt);}std::cout<<"\nDecision helper:\n  dynamic RMS |aXY| direct-FC="<<fcAll.rmsNorm()<<" gyro-integrated="<<gyAll.rmsNorm()<<" m/s^2\n";
    if(fcAll.rmsNorm()<0.05)std::cout<<"  RESULT: accelerometer is nearly clean under direct FC attitude; orientation/gyro path remains primary.\n";
    else if(rms0>0&&rms1/rms0<0.60&&lever.norm()<0.50)std::cout<<"  RESULT: yaw lever-arm model explains a large fraction of body acceleration. Stand/IMU rotation geometry is a major contributor.\n";
    else if(fcAll.rmsNorm()>=0.10)std::cout<<"  RESULT: substantial horizontal acceleration remains even with direct FC attitude. Investigate accelerometer bias/misalignment, real stand motion, vibration, and lever-arm geometry before further gyro tuning.\n";
    else std::cout<<"  RESULT: both orientation error and non-gravity accelerometer residual are material; quantify lever model and bias separately.\n";
    return 0;
  }catch(const std::exception&e){std::cerr<<"ERROR: "<<e.what()<<"\n";return 1;}
}
