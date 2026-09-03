// JT-ZERO Stage 11 final-suite recorder guard v15.30.
// v15.30 intentionally does NOT include v15.29 C++ source. It runs the proven
// v15.29 recorder as a separate executable, then validates persisted files and
// physical yaw quality. This avoids all nested-main/preprocessor conflicts.

#include <sys/statvfs.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <iomanip>

namespace {
constexpr unsigned long long MIN_FREE_BYTES = 1024ULL*1024ULL*1024ULL;
constexpr const char* RECORDER = "/tmp/record_mono_imu_final_suite_gui_v15_29_proven";
constexpr const char* RAW = "/home/vio/jtzero_final_suite_v15_29.csv";
constexpr const char* CAM = "/home/vio/jtzero_final_suite_v15_29_camera.csv";
constexpr const char* MJPG = "/home/vio/jtzero_final_suite_v15_29.mjpg";
constexpr const char* ATT = "/home/vio/jtzero_final_suite_v15_29_attitude.csv";
constexpr const char* PHASES = "/home/vio/jtzero_final_suite_v15_29_phases.csv";
constexpr const char* QUALITY = "/home/vio/jtzero_final_suite_v15_29_quality.tsv";

struct Phase { std::string name; long long a=0,b=0; double target=0,expected=0; };
struct AttRow { long long t=0; double rr=0,rp=0,ry=0; };

unsigned long long freeBytes(){ struct statvfs s{}; if(statvfs("/home/vio",&s)!=0)return 0; return (unsigned long long)s.f_bavail*(unsigned long long)s.f_frsize; }
unsigned long long fileSize(const char* p){ struct stat s{}; return stat(p,&s)==0?(unsigned long long)s.st_size:0ULL; }
std::vector<std::string> split(const std::string&s){ std::vector<std::string>v; std::stringstream ss(s); std::string x; while(std::getline(ss,x,','))v.push_back(x); return v; }
double wrap(double x){ while(x>180)x-=360; while(x<-180)x+=360; return x; }
bool pureYaw(const std::string&n){ return n=="YAW_A"||n=="YAW_RETURN_A"||n=="YAW_RETURN_500"; }

bool loadPhases(std::vector<Phase>&out){ std::ifstream f(PHASES); std::string l; if(!std::getline(f,l))return false; while(std::getline(f,l)){auto v=split(l);if(v.size()<5)continue;try{out.push_back({v[0],std::stoll(v[1]),std::stoll(v[2]),std::stod(v[3]),std::stod(v[4])});}catch(...){}} return !out.empty(); }
bool loadAtt(std::vector<AttRow>&out){ std::ifstream f(ATT); std::string l; if(!std::getline(f,l))return false; while(std::getline(f,l)){auto v=split(l);if(v.size()<9)continue;try{out.push_back({std::stoll(v[0]),std::stod(v[6]),std::stod(v[7]),std::stod(v[8])});}catch(...){}} return out.size()>100; }

bool physicalQuality(){
  std::vector<Phase> ph; std::vector<AttRow> at;
  if(!loadPhases(ph)||!loadAtt(at)){std::cerr<<"[V15.30] ERROR: cannot parse PHASES/ATTITUDE\n";return false;}
  std::ofstream q(QUALITY,std::ios::trunc);
  if(!q){std::cerr<<"[V15.30] ERROR: cannot create quality report\n";return false;}
  q<<"phase\tn\tstart_yaw_deg\tend_yaw_deg\tdyaw_deg\tmax_abs_rel_roll_deg\tmax_abs_rel_pitch_deg\tverdict\n";
  bool all=true;
  std::cout<<"\n================ PHYSICAL QUALITY V15.30 ================\n";
  for(const auto&p:ph){
    std::vector<const AttRow*> s; for(const auto&a:at)if(a.t>=p.a&&a.t<=p.b)s.push_back(&a);
    if(s.empty()){q<<p.name<<"\t0\tNA\tNA\tNA\tNA\tNA\tINVALID_NO_ATTITUDE\n";std::cout<<p.name<<": INVALID_NO_ATTITUDE\n";all=false;continue;}
    double mr=0,mp=0;for(auto a:s){mr=std::max(mr,std::abs(a->rr));mp=std::max(mp,std::abs(a->rp));}
    double sy=s.front()->ry,ey=s.back()->ry,dy=wrap(ey-sy);std::string verdict="PASS";
    if(pureYaw(p.name)){
      if(mr>2.0||mp>2.0)verdict="INVALID_RP";else if(mr>1.0||mp>1.0)verdict="WARN_RP";
      if(std::abs(dy)<60.0)verdict="INVALID_YAW_SPAN";
    }
    q<<p.name<<'\t'<<s.size()<<'\t'<<std::fixed<<std::setprecision(3)<<sy<<'\t'<<ey<<'\t'<<dy<<'\t'<<mr<<'\t'<<mp<<'\t'<<verdict<<'\n';
    std::cout<<std::left<<std::setw(18)<<p.name<<" dYaw="<<std::setw(8)<<std::fixed<<std::setprecision(2)<<dy<<" max|R|="<<std::setw(7)<<mr<<" max|P|="<<std::setw(7)<<mp<<" "<<verdict<<'\n';
    if(verdict.rfind("INVALID",0)==0)all=false;
  }
  q.flush(); if(!q.good()){std::cerr<<"[V15.30] ERROR writing quality report\n";return false;} q.close();
  std::cout<<"QUALITY: "<<QUALITY<<"\n"; return all;
}
}

int main(){
  const auto f0=freeBytes();
  std::cout<<"[V15.30] free disk before recording: "<<std::fixed<<std::setprecision(1)<<(f0/1024.0/1024.0)<<" MiB\n";
  if(f0<MIN_FREE_BYTES){std::cerr<<"[V15.30] FAIL: less than 1024 MiB free. Physical test NOT started.\n";return 10;}
  if(fileSize(RECORDER)==0){std::cerr<<"[V15.30] FAIL: proven recorder executable missing: "<<RECORDER<<"\n";return 13;}

  std::cout<<"[V15.30] launching proven v15.29 acquisition as isolated process\n";
  const int sr=std::system(RECORDER);
  if(sr==-1){std::cerr<<"[V15.30] FAIL: cannot launch recorder\n";return 14;}
  const int rc=WIFEXITED(sr)?WEXITSTATUS(sr):128;
  if(rc!=0){std::cerr<<"[V15.30] underlying recorder rc="<<rc<<"\n";return rc;}

  const struct{const char*p;unsigned long long min;} req[]={{RAW,1024},{CAM,1024},{MJPG,1024*1024},{ATT,1024},{PHASES,128}};
  bool ok=true;std::cout<<"\n================ PERSISTENCE CHECK V15.30 ================\n";
  for(const auto&r:req){auto n=fileSize(r.p);std::cout<<r.p<<" : "<<n<<" bytes\n";if(n<r.min){std::cerr<<"[V15.30] FAIL: persisted file missing/too small: "<<r.p<<"\n";ok=false;}}
  if(!ok)return 11;
  std::cout<<"[V15.30] free disk after recording: "<<(freeBytes()/1024.0/1024.0)<<" MiB\n";
  if(!physicalQuality()){std::cerr<<"[V15.30] RESULT: PHYSICAL_TEST_INVALID. Offline Kimera matrix must NOT run.\n";return 12;}
  std::cout<<"[V15.30] RESULT: PASS_PERSISTED_AND_PHYSICALLY_VALID\n";return 0;
}
