// JT-ZERO v15.29: analyze replay trajectory by recorder phase boundaries.
// Usage: analyze_final_suite_segments_v15_29 replay_states.csv phases.csv label
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

struct S{long long k=0,t=0;double x=0,y=0,z=0,r=0,p=0,yaw=0;};
struct P{std::string n;long long a=0,b=0;double ty=0,ex=0;};
static std::vector<std::string> sp(const std::string&s){std::vector<std::string>v;std::string c;for(char ch:s){if(ch==','){v.push_back(c);c.clear();}else c.push_back(ch);}v.push_back(c);return v;}
static double wd(double a){while(a>180)a-=360;while(a<-180)a+=360;return a;}
static std::vector<S> loadS(const std::string&f){std::ifstream q(f);if(!q)throw std::runtime_error("cannot open "+f);std::vector<S>v;std::string l;std::getline(q,l);while(std::getline(q,l)){auto c=sp(l);if(c.size()<11)continue;S s;s.k=std::stoll(c[0]);s.t=std::stoll(c[1]);s.x=std::stod(c[2]);s.y=std::stod(c[3]);s.z=std::stod(c[4]);s.r=std::stod(c[8]);s.p=std::stod(c[9]);s.yaw=std::stod(c[10]);v.push_back(s);}return v;}
static std::vector<P> loadP(const std::string&f){std::ifstream q(f);if(!q)throw std::runtime_error("cannot open "+f);std::vector<P>v;std::string l;std::getline(q,l);while(std::getline(q,l)){auto c=sp(l);if(c.size()<5)continue;P p;p.n=c[0];p.a=std::stoll(c[1]);p.b=std::stoll(c[2]);p.ty=std::stod(c[3]);p.ex=std::stod(c[4]);v.push_back(p);}return v;}
int main(int ac,char**av){if(ac<4){std::cerr<<"usage: "<<av[0]<<" states.csv phases.csv label\n";return 2;}try{auto s=loadS(av[1]);auto ph=loadP(av[2]);std::string lab=av[3];if(s.empty())throw std::runtime_error("no states");std::cout<<"label\tphase\tn\tdxy_mm\tdp3_mm\tpath_mm\tdyaw_deg\tmax_abs_droll_deg\tmax_abs_dpitch_deg\texpected_xy_mm\n";for(const auto&p:ph){std::vector<const S*>v;for(const auto&x:s)if(x.t>=p.a&&x.t<=p.b)v.push_back(&x);if(v.size()<2){std::cout<<lab<<'\t'<<p.n<<'\t'<<v.size()<<"\tNA\tNA\tNA\tNA\tNA\tNA\t"<<p.ex<<"\n";continue;}const S&a=*v.front(),&b=*v.back();double dx=b.x-a.x,dy=b.y-a.y,dz=b.z-a.z,dxy=std::hypot(dx,dy),dp=std::sqrt(dx*dx+dy*dy+dz*dz),path=0,mr=0,mp=0;for(size_t i=0;i<v.size();++i){mr=std::max(mr,std::abs(wd(v[i]->r-a.r)));mp=std::max(mp,std::abs(wd(v[i]->p-a.p)));if(i){double xx=v[i]->x-v[i-1]->x,yy=v[i]->y-v[i-1]->y,zz=v[i]->z-v[i-1]->z;path+=std::sqrt(xx*xx+yy*yy+zz*zz);}}std::cout<<std::fixed<<std::setprecision(3)<<lab<<'\t'<<p.n<<'\t'<<v.size()<<'\t'<<dxy*1000<<'\t'<<dp*1000<<'\t'<<path*1000<<'\t'<<wd(b.yaw-a.yaw)<<'\t'<<mr<<'\t'<<mp<<'\t'<<p.ex<<"\n";}return 0;}catch(const std::exception&e){std::cerr<<"[FATAL] "<<e.what()<<"\n";return 1;}}
