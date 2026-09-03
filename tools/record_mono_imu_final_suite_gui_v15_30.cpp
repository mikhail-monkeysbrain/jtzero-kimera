// JT-ZERO Stage 11 final-suite recorder guard v15.30.
// Wraps the proven v15.29 acquisition/GUI, but refuses low-disk starts and
// validates the actually persisted dataset plus physical yaw quality before
// returning success to the master runner.

#include <sys/statvfs.h>
#include <sys/stat.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <iomanip>

#define main jtzero_final_suite_v1529_main
#include "record_mono_imu_final_suite_gui_v15_29.cpp"
#undef main

namespace {
constexpr unsigned long long MIN_FREE_BYTES_1530 = 1024ULL * 1024ULL * 1024ULL;
constexpr const char* RAW_1530 = "/home/vio/jtzero_final_suite_v15_29.csv";
constexpr const char* CAM_1530 = "/home/vio/jtzero_final_suite_v15_29_camera.csv";
constexpr const char* MJPG_1530 = "/home/vio/jtzero_final_suite_v15_29.mjpg";
constexpr const char* ATT_1530 = "/home/vio/jtzero_final_suite_v15_29_attitude.csv";
constexpr const char* PHASES_1530 = "/home/vio/jtzero_final_suite_v15_29_phases.csv";
constexpr const char* QUALITY_1530 = "/home/vio/jtzero_final_suite_v15_29_quality.tsv";

struct Phase1530 { std::string name; long long a=0,b=0; double target=0, expected=0; };
struct Att1530 { long long t=0; double rr=0,rp=0,ry=0; };

unsigned long long freeBytes1530(){
  struct statvfs s{};
  if(statvfs("/home/vio",&s)!=0) return 0;
  return static_cast<unsigned long long>(s.f_bavail)*static_cast<unsigned long long>(s.f_frsize);
}
unsigned long long fileSize1530(const char* p){ struct stat s{}; return stat(p,&s)==0?static_cast<unsigned long long>(s.st_size):0ULL; }
std::vector<std::string> split1530(const std::string& s){ std::vector<std::string> v; std::stringstream ss(s); std::string x; while(std::getline(ss,x,','))v.push_back(x); return v; }

bool loadPhases1530(std::vector<Phase1530>& out){
  std::ifstream f(PHASES_1530); std::string l; if(!std::getline(f,l)) return false;
  while(std::getline(f,l)){ auto v=split1530(l); if(v.size()<5) continue; try{ out.push_back({v[0],std::stoll(v[1]),std::stoll(v[2]),std::stod(v[3]),std::stod(v[4])}); }catch(...){} }
  return !out.empty();
}
bool loadAtt1530(std::vector<Att1530>& out){
  std::ifstream f(ATT_1530); std::string l; if(!std::getline(f,l)) return false;
  while(std::getline(f,l)){ auto v=split1530(l); if(v.size()<9) continue; try{ out.push_back({std::stoll(v[0]),std::stod(v[6]),std::stod(v[7]),std::stod(v[8])}); }catch(...){} }
  return out.size()>100;
}

double wrap1530(double x){ while(x>180)x-=360; while(x<-180)x+=360; return x; }
bool isPureYaw1530(const std::string& n){ return n=="YAW_A" || n=="YAW_RETURN_A" || n=="YAW_RETURN_500"; }

bool quality1530(){
  std::vector<Phase1530> ph; std::vector<Att1530> at;
  if(!loadPhases1530(ph)||!loadAtt1530(at)){ std::cerr<<"[V15.30] ERROR: cannot parse PHASES/ATTITUDE\n"; return false; }
  std::ofstream q(QUALITY_1530,std::ios::trunc);
  q<<"phase\tn\tstart_yaw_deg\tend_yaw_deg\tdyaw_deg\tmax_abs_rel_roll_deg\tmax_abs_rel_pitch_deg\tverdict\n";
  bool all=true;
  std::cout<<"\n================ PHYSICAL QUALITY V15.30 ================\n";
  for(const auto& p:ph){
    std::vector<const Att1530*> s; for(const auto& a:at) if(a.t>=p.a&&a.t<=p.b) s.push_back(&a);
    if(s.empty()){ q<<p.name<<"\t0\tNA\tNA\tNA\tNA\tNA\tINVALID_NO_ATTITUDE\n"; std::cout<<p.name<<": INVALID_NO_ATTITUDE\n"; all=false; continue; }
    double mr=0,mp=0; for(auto a:s){mr=std::max(mr,std::abs(a->rr));mp=std::max(mp,std::abs(a->rp));}
    double sy=s.front()->ry, ey=s.back()->ry, dy=wrap1530(ey-sy);
    std::string verdict="PASS";
    if(isPureYaw1530(p.name)){
      // Clean yaw phases are deliberately strict: >2 deg Euler R/P contamination
      // invalidates the phase. 1..2 deg is retained as WARN for later review.
      if(mr>2.0||mp>2.0) verdict="INVALID_RP";
      else if(mr>1.0||mp>1.0) verdict="WARN_RP";
      if(std::abs(dy)<60.0) verdict="INVALID_YAW_SPAN";
    }
    q<<p.name<<'\t'<<s.size()<<'\t'<<std::fixed<<std::setprecision(3)<<sy<<'\t'<<ey<<'\t'<<dy<<'\t'<<mr<<'\t'<<mp<<'\t'<<verdict<<'\n';
    std::cout<<std::left<<std::setw(18)<<p.name<<" dYaw="<<std::setw(8)<<std::fixed<<std::setprecision(2)<<dy<<" max|R|="<<std::setw(7)<<mr<<" max|P|="<<std::setw(7)<<mp<<" "<<verdict<<'\n';
    if(verdict.rfind("INVALID",0)==0) all=false;
  }
  q.flush(); q.close();
  if(!q.good()){ std::cerr<<"[V15.30] ERROR writing quality report\n"; return false; }
  std::cout<<"QUALITY: "<<QUALITY_1530<<"\n";
  return all;
}
}

int main(){
  const auto free0=freeBytes1530();
  std::cout<<"[V15.30] free disk before recording: "<<std::fixed<<std::setprecision(1)<<(free0/1024.0/1024.0)<<" MiB\n";
  if(free0<MIN_FREE_BYTES_1530){
    std::cerr<<"[V15.30] FAIL: less than 1024 MiB free. Physical test NOT started.\n";
    return 10;
  }

  const int rc=jtzero_final_suite_v1529_main();
  if(rc!=0){ std::cerr<<"[V15.30] underlying recorder rc="<<rc<<"\n"; return rc; }

  // At this point v15.29 main has returned, so all of its local ofstreams have
  // been destructed/closed. Validate the files on disk, not in-memory counters.
  const struct {const char* p; unsigned long long min;} req[]={{RAW_1530,1024},{CAM_1530,1024},{MJPG_1530,1024*1024},{ATT_1530,1024},{PHASES_1530,128}};
  bool files_ok=true;
  std::cout<<"\n================ PERSISTENCE CHECK V15.30 ================\n";
  for(const auto& r:req){ const auto n=fileSize1530(r.p); std::cout<<r.p<<" : "<<n<<" bytes\n"; if(n<r.min){std::cerr<<"[V15.30] FAIL: persisted file missing/too small: "<<r.p<<"\n";files_ok=false;} }
  if(!files_ok) return 11;

  const auto free1=freeBytes1530();
  std::cout<<"[V15.30] free disk after recording: "<<(free1/1024.0/1024.0)<<" MiB\n";
  if(!quality1530()){
    std::cerr<<"[V15.30] RESULT: PHYSICAL_TEST_INVALID. Offline Kimera matrix must NOT run.\n";
    return 12;
  }
  std::cout<<"[V15.30] RESULT: PASS_PERSISTED_AND_PHYSICALLY_VALID\n";
  return 0;
}
