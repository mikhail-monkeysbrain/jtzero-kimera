// JT-ZERO Stage 11 diagnostic v15.34
// Offline check of MAVLink ATTITUDE_QUATERNION convention against simultaneous ATTITUDE.
// Uses existing v15.32 data. No physical rerun.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr double PI=3.14159265358979323846;
struct E{int64_t t=0;double r=0,p=0,y=0;};
struct Q{int64_t t=0;double w=1,x=0,y=0,z=0;};
struct Quat{double w=1,x=0,y=0,z=0;};
struct Err{std::string name;std::vector<double> er,ep,ey,e3;};
std::vector<std::string> split(const std::string&s){std::vector<std::string>o;std::stringstream ss(s);std::string x;while(std::getline(ss,x,','))o.push_back(x);return o;}
double deg(double x){return x*180.0/PI;}
double wrap(double x){while(x>180)x-=360;while(x<-180)x+=360;return x;}
Quat norm(Quat q){double n=std::sqrt(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z);if(n<1e-12)return{};q.w/=n;q.x/=n;q.y/=n;q.z/=n;return q;}
Quat conj(Quat q){return{q.w,-q.x,-q.y,-q.z};}
Quat mul(Quat a,Quat b){return{a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z,a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w};}
void euler(Quat q,double&r,double&p,double&y){q=norm(q);r=deg(std::atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y)));p=deg(std::asin(std::clamp(2*(q.w*q.y-q.z*q.x),-1.0,1.0)));y=deg(std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)));}
double pct(std::vector<double>v,double q){if(v.empty())return 0;std::sort(v.begin(),v.end());double z=q*(v.size()-1);size_t a=(size_t)std::floor(z),b=std::min(a+1,v.size()-1);return v[a]+(v[b]-v[a])*(z-a);}
double mx(const std::vector<double>&v){return v.empty()?0:*std::max_element(v.begin(),v.end());}
std::vector<E> loadE(const std::string&f){std::ifstream in(f);if(!in)throw std::runtime_error("cannot open "+f);std::string l;std::getline(in,l);std::vector<E>v;while(std::getline(in,l)){auto c=split(l);if(c.size()>=5)v.push_back({std::stoll(c[0]),std::stod(c[2]),std::stod(c[3]),std::stod(c[4])});}return v;}
std::vector<Q> loadQ(const std::string&f){std::ifstream in(f);if(!in)throw std::runtime_error("cannot open "+f);std::string l;std::getline(in,l);std::vector<Q>v;while(std::getline(in,l)){auto c=split(l);if(c.size()>=6)v.push_back({std::stoll(c[0]),std::stod(c[2]),std::stod(c[3]),std::stod(c[4]),std::stod(c[5])});}return v;}
void add(Err&z,const E&e,Quat q){double r,p,y;euler(q,r,p,y);double a=std::abs(wrap(r-e.r)),b=std::abs(wrap(p-e.p)),c=std::abs(wrap(y-e.y));z.er.push_back(a);z.ep.push_back(b);z.ey.push_back(c);z.e3.push_back(std::sqrt(a*a+b*b+c*c));}
void print(const Err&z){std::cout<<std::left<<std::setw(18)<<z.name<<std::right<<std::fixed<<std::setprecision(3)<<std::setw(10)<<pct(z.er,.5)<<std::setw(10)<<pct(z.ep,.5)<<std::setw(10)<<pct(z.ey,.5)<<std::setw(10)<<pct(z.e3,.5)<<std::setw(10)<<pct(z.e3,.95)<<std::setw(10)<<mx(z.e3)<<"\n";}
}
int main(int argc,char**argv){try{
 std::string ef=argc>1?argv[1]:"/home/vio/jtzero_attitude_v15_32.csv";
 std::string qf=argc>2?argv[2]:"/home/vio/jtzero_quaternion_v15_32.csv";
 std::vector<E> ev=loadE(ef);
 std::vector<Q> qv=loadQ(qf);
 if(ev.empty()||qv.empty())throw std::runtime_error("empty input");
 Err raw{"RAW q"},inv{"CONJ(q)"},swapxy{"SWAP_XY"},flipxy{"FLIP_XY"};size_t j=0,n=0;double maxdt=0;
 for(const auto&q:qv){while(j+1<ev.size()&&std::llabs(ev[j+1].t-q.t)<std::llabs(ev[j].t-q.t))++j;if(j>=ev.size())break;double dt=std::abs(ev[j].t-q.t)*1e-6;if(dt>30)continue;maxdt=std::max(maxdt,dt);Quat a=norm({q.w,q.x,q.y,q.z});add(raw,ev[j],a);add(inv,ev[j],conj(a));add(swapxy,ev[j],norm({q.w,q.y,q.x,q.z}));add(flipxy,ev[j],norm({q.w,-q.x,-q.y,q.z}));++n;}
 std::cout<<"================ V15.34 ABSOLUTE QUATERNION CHECK ================\n";
 std::cout<<"Euler rows="<<ev.size()<<" Quaternion rows="<<qv.size()<<" matched="<<n<<" max_dt_ms="<<std::fixed<<std::setprecision(3)<<maxdt<<"\n";
 std::cout<<std::left<<std::setw(18)<<"INTERPRETATION"<<std::right<<std::setw(10)<<"Rmed"<<std::setw(10)<<"Pmed"<<std::setw(10)<<"Ymed"<<std::setw(10)<<"E3med"<<std::setw(10)<<"E3p95"<<std::setw(10)<<"E3max"<<"\n";print(raw);print(inv);print(swapxy);print(flipxy);
 const Err*best=&raw;for(const Err*z:{&inv,&swapxy,&flipxy})if(pct(z->e3,.5)<pct(best->e3,.5))best=z;
 std::cout<<"\nBEST_ABSOLUTE_INTERPRETATION: "<<best->name<<"  median_combined_error="<<pct(best->e3,.5)<<" deg\n";
 size_t k0=0;while(k0<qv.size()&&qv[k0].t<ev.front().t)++k0;if(k0>=qv.size())k0=0;Quat q0=norm({qv[k0].w,qv[k0].x,qv[k0].y,qv[k0].z});
 std::vector<double>a1,a2,a3,a4;for(const auto&q:qv){Quat x=norm({q.w,q.x,q.y,q.z});Quat z[4]={mul(conj(q0),x),mul(x,conj(q0)),mul(conj(x),q0),mul(q0,conj(x))};for(int m=0;m<4;++m){double r,p,y;euler(z[m],r,p,y);double v=std::sqrt(r*r+p*p);(m==0?a1:m==1?a2:m==2?a3:a4).push_back(v);}}
 std::cout<<"\nRELATIVE RP magnitude (diagnostic, full-record median/p95):\n";
 const char*nms[4]={"q0^-1*q","q*q0^-1","q^-1*q0","q0*q^-1"};std::vector<double>*vv[4]={&a1,&a2,&a3,&a4};for(int m=0;m<4;++m)std::cout<<std::setw(12)<<nms[m]<<" med="<<pct(*vv[m],.5)<<" p95="<<pct(*vv[m],.95)<<" deg\n";
 if(pct(raw.e3,.5)<1.0)std::cout<<"\nRESULT: RAW_QUATERNION_MATCHES_ATTITUDE\n";else if(best!=&raw&&pct(best->e3,.5)<1.0)std::cout<<"\nRESULT: ALTERNATE_CONVENTION_MATCHES_ATTITUDE\n";else std::cout<<"\nRESULT: QUATERNION_DOES_NOT_DIRECTLY_MATCH_ATTITUDE\n";
 std::cout<<"NOTE: v15.32 did not record MAVLink repr_offset_q, so this test cannot evaluate that extension from the existing dataset.\n";
 return 0;}catch(const std::exception&e){std::cerr<<"[FATAL] "<<e.what()<<"\n";return 1;}}
