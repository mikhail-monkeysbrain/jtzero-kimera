// JT-ZERO v45.4: causal analysis of effective gyro X-bias using v44 master CSV.
// No new physical test. Uses FC-rate guarded static windows, then tests time/history/cross-axis predictors.

#include <Eigen/Dense>
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
struct Row {
  std::string stage;
  uint64_t us=0;
  double dt=0;
  Eigen::Vector3d w=Eigen::Vector3d::Zero();
  Eigen::Vector3d fcw=Eigen::Vector3d::Zero();
  Eigen::Vector3d a=Eigen::Vector3d::Zero();
};
struct Win {
  size_t b=0,e=0;
  std::string stage;
  double t=0,bx=0,by=0,bz=0;
  double prev_abs=0,prev_e=0,prev_ex=0,prev_ey=0,prev_ez=0;
  double prev_sx=0,prev_sy=0,prev_sz=0;
};
std::vector<std::string> split(const std::string&s){std::vector<std::string>v;std::stringstream q(s);std::string x;while(std::getline(q,x,','))v.push_back(x);return v;}
double D(const std::string&s){return s.empty()?0.0:std::stod(s);}
std::vector<Row> load(const std::string&p){
  std::ifstream f(p); if(!f) throw std::runtime_error("cannot open "+p);
  std::string l; std::getline(f,l); auto h=split(l); std::map<std::string,int> c; for(int i=0;i<(int)h.size();++i)c[h[i]]=i;
  const char* need[]={"stage","imu_us","dt","flu_ax","flu_ay","flu_az","raw_gx","raw_gy","raw_gz","fc_rollspeed","fc_pitchspeed","fc_yawspeed"};
  for(auto n:need) if(!c.count(n)) throw std::runtime_error(std::string("missing ")+n);
  std::vector<Row>v; while(std::getline(f,l)){if(l.empty())continue;auto x=split(l);if(x.size()<h.size())continue;Row r;r.stage=x[c["stage"]];r.us=std::stoull(x[c["imu_us"]]);r.dt=D(x[c["dt"]]);r.a={D(x[c["flu_ax"]]),D(x[c["flu_ay"]]),D(x[c["flu_az"]])};r.w={D(x[c["raw_gx"]]),D(x[c["raw_gy"]]),D(x[c["raw_gz"]])};r.fcw={D(x[c["fc_rollspeed"]]),D(x[c["fc_pitchspeed"]]),D(x[c["fc_yawspeed"]])};v.push_back(r);}return v;
}
std::vector<bool> staticMask(const std::vector<Row>&v,size_t s){
  const int H=100; std::vector<bool>m(v.size(),false); if(v.size()<2*H+1)return m;
  std::vector<Eigen::Vector3d> pa(v.size()+1,Eigen::Vector3d::Zero()),pa2(v.size()+1,Eigen::Vector3d::Zero());
  for(size_t i=s;i<v.size();++i){pa[i+1]=pa[i]+v[i].a;pa2[i+1]=pa2[i]+v[i].a.cwiseProduct(v[i].a);}
  for(size_t i=s+H;i+H<v.size();++i){size_t b=i-H,e=i+H;double n=double(e-b);Eigen::Vector3d ma=((pa[e]-pa[b])/n).eval();Eigen::Vector3d va=((pa2[e]-pa2[b])/n-ma.cwiseProduct(ma)).eval();double as=std::sqrt(std::max(0.0,va.sum()));if(v[i].fcw.norm()<0.010 && as<0.160 && std::abs(ma.norm()-9.81)<0.30)m[i]=true;}
  return m;
}
std::vector<Win> buildWins(const std::vector<Row>&v,size_t s,const std::vector<bool>&sm){
  std::vector<Win>wins; size_t i=s,last_end=s; double t0=v[s].us*1e-6;
  while(i<v.size()){
    while(i<v.size()&&!sm[i])++i; if(i>=v.size())break; size_t b=i; std::string st=v[i].stage; while(i<v.size()&&sm[i]&&v[i].stage==st)++i; size_t e=i; if(e-b<150)continue;
    Win w;w.b=b;w.e=e;w.stage=st; w.t=0.5*(v[b].us+v[e-1].us)*1e-6-t0;
    Eigen::Vector3d mw=Eigen::Vector3d::Zero(); for(size_t k=b;k<e;++k)mw+=v[k].w; mw/=double(e-b); w.bx=mw.x();w.by=mw.y();w.bz=mw.z();
    for(size_t k=last_end;k<b;++k){double dt=(v[k].dt>0&&v[k].dt<0.03)?v[k].dt:0;if(!dt)continue;double n=v[k].w.norm();w.prev_abs+=n*dt;w.prev_e+=v[k].w.squaredNorm()*dt;w.prev_ex+=v[k].w.x()*v[k].w.x()*dt;w.prev_ey+=v[k].w.y()*v[k].w.y()*dt;w.prev_ez+=v[k].w.z()*v[k].w.z()*dt;w.prev_sx+=v[k].w.x()*dt;w.prev_sy+=v[k].w.y()*dt;w.prev_sz+=v[k].w.z()*dt;}
    wins.push_back(w); last_end=e;
  }
  return wins;
}
Eigen::VectorXd fit(const Eigen::MatrixXd&X,const Eigen::VectorXd&y){Eigen::MatrixXd A=X.transpose()*X;A.diagonal().array()+=1e-12;return A.ldlt().solve(X.transpose()*y);}
double rmse(const Eigen::VectorXd&a,const Eigen::VectorXd&b){return std::sqrt((a-b).squaredNorm()/double(a.size()));}
struct CV{double fwd=0,rev=0;};
CV testModel(const std::vector<Win>&w,int model){
  int n=(int)w.size(),mid=n/2; if(mid<3||n-mid<3)return {};
  auto feat=[&](const Win&z){std::vector<double>f{1.0};if(model==1){f.push_back(z.t);}else if(model==2){f.insert(f.end(),{z.prev_abs,z.prev_e,z.prev_ex,z.prev_ey,z.prev_ez});}else if(model==3){f.insert(f.end(),{z.by,z.bz});}else if(model==4){f.insert(f.end(),{z.t,z.prev_abs,z.prev_e,z.prev_ex,z.prev_ey,z.prev_ez,z.prev_sx,z.prev_sy,z.prev_sz,z.by,z.bz});}return f;};
  int p=(int)feat(w[0]).size();
  auto one=[&](int tb,int te,int vb,int ve){Eigen::MatrixXd X(te-tb,p),V(ve-vb,p);Eigen::VectorXd y(te-tb),q(ve-vb);for(int i=tb;i<te;++i){auto f=feat(w[i]);for(int j=0;j<p;++j)X(i-tb,j)=f[j];y(i-tb)=w[i].bx;}for(int i=vb;i<ve;++i){auto f=feat(w[i]);for(int j=0;j<p;++j)V(i-vb,j)=f[j];q(i-vb)=w[i].bx;}auto b=fit(X,y);return rmse(V*b,q);};
  return {one(0,mid,mid,n),one(mid,n,0,mid)};
}
double holdLastRmse(const std::vector<Win>&w){if(w.size()<2)return 0;double s=0;for(size_t i=1;i<w.size();++i){double e=w[i].bx-w[i-1].bx;s+=e*e;}return std::sqrt(s/double(w.size()-1));}
double corr(const std::vector<double>&a,const std::vector<double>&b){if(a.size()!=b.size()||a.size()<2)return 0;double ma=std::accumulate(a.begin(),a.end(),0.0)/a.size(),mb=std::accumulate(b.begin(),b.end(),0.0)/b.size(),s=0,sa=0,sb=0;for(size_t i=0;i<a.size();++i){double x=a[i]-ma,y=b[i]-mb;s+=x*y;sa+=x*x;sb+=y*y;}return (sa>0&&sb>0)?s/std::sqrt(sa*sb):0;}
}
int main(int ac,char**av){
  std::string p=ac>1?av[1]:"/home/vio/jtzero_imu_master_v44.csv",op=ac>2?av[2]:"/home/vio/jtzero_imu_bias_causality_v45_4.txt";
  try{auto v=load(p);size_t s=0;while(s<v.size()&&v[s].stage=="CALIB_STATIC")++s;if(s>=v.size())throw std::runtime_error("no post-cal boundary");auto sm=staticMask(v,s);auto w=buildWins(v,s,sm);std::vector<double>bx,t,ab,en,ex,ey,ez,by,bz;for(auto&z:w){bx.push_back(z.bx);t.push_back(z.t);ab.push_back(z.prev_abs);en.push_back(z.prev_e);ex.push_back(z.prev_ex);ey.push_back(z.prev_ey);ez.push_back(z.prev_ez);by.push_back(z.by);bz.push_back(z.bz);}CV c1=testModel(w,1),c2=testModel(w,2),c3=testModel(w,3),c4=testModel(w,4);std::ostringstream o;o<<std::fixed<<std::setprecision(9);o<<"============================================================\nJT-ZERO v45.4 CAUSAL EFFECTIVE BG_X ANALYSIS\n============================================================\ninput: "<<p<<"\npost-cal boundary: "<<s<<"\nindependent static windows: "<<w.size()<<"\nstatic samples: "<<std::count(sm.begin()+s,sm.end(),true)<<"\n\nSTATIC WINDOWS\n";for(size_t i=0;i<w.size();++i)o<<i<<" stage="<<w[i].stage<<" t="<<w[i].t<<" bx="<<w[i].bx<<" by="<<w[i].by<<" bz="<<w[i].bz<<" prev_abs="<<w[i].prev_abs<<" prev_E="<<w[i].prev_e<<" Ex="<<w[i].prev_ex<<" Ey="<<w[i].prev_ey<<" Ez="<<w[i].prev_ez<<"\n";
  o<<"\nCORRELATIONS WITH STATIC bx\n";o<<"time: "<<corr(bx,t)<<"\nprev |w| integral: "<<corr(bx,ab)<<"\nprev w^2 energy: "<<corr(bx,en)<<"\nprev wx^2: "<<corr(bx,ex)<<"\nprev wy^2: "<<corr(bx,ey)<<"\nprev wz^2: "<<corr(bx,ez)<<"\nstatic by: "<<corr(bx,by)<<"\nstatic bz: "<<corr(bx,bz)<<"\n";
  o<<"\nCAUSAL CROSS-VALIDATION RMSE [rad/s]\nmodel | train first -> validate second | train second -> validate first\n";o<<"M1 time | "<<c1.fwd<<" | "<<c1.rev<<"\n";o<<"M2 previous-motion history | "<<c2.fwd<<" | "<<c2.rev<<"\n";o<<"M3 instantaneous static Y/Z coupling | "<<c3.fwd<<" | "<<c3.rev<<"\n";o<<"M4 combined | "<<c4.fwd<<" | "<<c4.rev<<"\n";o<<"causal hold-last static bx | "<<holdLastRmse(w)<<" | same one-step metric\n";
  o<<"\nINTERPRETATION\n";o<<"- Strong M1 transfer supports slow time/temperature-like drift.\n";o<<"- Strong M2 transfer supports motion-history/rate-dependent effective bias.\n";o<<"- Strong M3 transfer supports shared Y/Z bias/cross-axis behavior even in static windows.\n";o<<"- If parametric models transfer poorly but hold-last is good, causal static BG_X adaptation is the most robust practical model.\n";o<<"- Instantaneous linear cross-axis from real angular rate cannot by itself explain a persistent static bx when body rates are near zero.\n";o<<"- Compare conclusions against v39/v42/v45.3; do not discard independently validated Z->XY correction.\n";
  std::ofstream f(op);f<<o.str();std::cout<<o.str()<<"\nResult TXT: "<<op<<"\n";return 0;}catch(const std::exception&e){std::cerr<<"[FATAL] "<<e.what()<<"\n";return 1;}
}
