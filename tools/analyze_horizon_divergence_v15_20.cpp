#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <vector>

struct State {
  int kf = -1;
  long long ts = 0;
  double px=0, py=0, pz=0;
  double vx=0, vy=0, vz=0;
  double r=0, p=0, y=0;
};

static std::vector<std::string> split(const std::string& s, char d) {
  std::vector<std::string> out; std::stringstream ss(s); std::string x;
  while (std::getline(ss, x, d)) out.push_back(x);
  return out;
}

static std::vector<State> load_states(const std::string& path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("cannot open " + path);
  std::string line; std::getline(f, line);
  std::vector<State> v;
  while (std::getline(f, line)) {
    if (line.empty()) continue;
    auto c = split(line, ',');
    if (c.size() < 11) continue;
    State s;
    s.kf = std::stoi(c[0]); s.ts = std::stoll(c[1]);
    s.px = std::stod(c[2]); s.py = std::stod(c[3]); s.pz = std::stod(c[4]);
    s.vx = std::stod(c[5]); s.vy = std::stod(c[6]); s.vz = std::stod(c[7]);
    s.r = std::stod(c[8]); s.p = std::stod(c[9]); s.y = std::stod(c[10]);
    v.push_back(s);
  }
  return v;
}

struct YawWindow { long long begin=0, end=0; bool ok=false; };

static YawWindow detect_yaw(const std::string& combined) {
  std::ifstream f(combined);
  if (!f) throw std::runtime_error("cannot open " + combined);
  std::string line; long long first=0,last=0; bool any=false;
  const double th = 5.0 * M_PI / 180.0;
  while (std::getline(f,line)) {
    if (line.empty()) continue;
    auto c=split(line,',');
    if (c.size()<14 || c[0]!="IMU") continue;
    try {
      long long mapped=std::stoll(c[3]);
      double gz_frd=std::stod(c[13]);
      double gz_flu=-gz_frd;
      if (std::abs(gz_flu)>=th) {
        if (!any) first=mapped;
        last=mapped; any=true;
      }
    } catch (...) {}
  }
  return {first,last,any};
}

static double norm3(double x,double y,double z){ return std::sqrt(x*x+y*y+z*z); }
static double wrap_deg(double a){ while(a>180)a-=360; while(a<-180)a+=360; return a; }

struct PairMetric {
  double dp_mm=0,dv_mm_s=0,dr=0,dp=0,dy=0,drnorm=0;
};
static PairMetric metric(const State&a,const State&b){
  PairMetric m;
  m.dp_mm=1000.0*norm3(a.px-b.px,a.py-b.py,a.pz-b.pz);
  m.dv_mm_s=1000.0*norm3(a.vx-b.vx,a.vy-b.vy,a.vz-b.vz);
  m.dr=wrap_deg(a.r-b.r); m.dp=wrap_deg(a.p-b.p); m.dy=wrap_deg(a.y-b.y);
  m.drnorm=norm3(m.dr,m.dp,m.dy); return m;
}

static const char* phase(long long ts,const YawWindow&w){
  if(!w.ok) return "UNKNOWN";
  if(ts<w.begin) return "PRE";
  if(ts<=w.end) return "YAW";
  return "POST";
}

static void report_threshold(const char* name,const std::vector<State>&a,const std::vector<State>&b,const YawWindow&w,double th,int ha,int hb){
  for(size_t i=0;i<std::min(a.size(),b.size());++i){
    auto m=metric(a[i],b[i]);
    if(m.dp_mm>=th){
      int k=a[i].kf;
      std::cout<<"  >="<<th<<" mm: kf="<<k<<" phase="<<phase(a[i].ts,w)
               <<" dP="<<std::fixed<<std::setprecision(3)<<m.dp_mm<<" mm"
               <<" dV="<<m.dv_mm_s<<" mm/s dR="<<std::setprecision(5)<<m.drnorm<<" deg"
               <<" | H"<<ha<<" drops~x"<<(k-ha)<<", H"<<hb<<" drops~x"<<(k-hb)<<"\n";
      return;
    }
  }
  std::cout<<"  >="<<th<<" mm: not reached\n";
}

int main(int argc,char**argv){
  try{
    std::string root = argc>1?argv[1]:"/home/vio/jtzero_horizon_threshold_v15_19";
    std::string combined = argc>2?argv[2]:"/home/vio/jtzero_yaw_only_v13.csv";
    auto h28=load_states(root+"/H28.csv");
    auto h29=load_states(root+"/H29.csv");
    auto h30=load_states(root+"/H30.csv");
    auto yw=detect_yaw(combined);
    if(h28.size()!=h29.size()||h29.size()!=h30.size()) throw std::runtime_error("state count mismatch H28/H29/H30");
    for(size_t i=0;i<h28.size();++i){
      if(h28[i].kf!=h29[i].kf||h29[i].kf!=h30[i].kf||h28[i].ts!=h29[i].ts||h29[i].ts!=h30[i].ts)
        throw std::runtime_error("keyframe/timestamp mismatch at row "+std::to_string(i));
    }
    std::cout<<"============================================================\n";
    std::cout<<"JT-ZERO HORIZON DIVERGENCE ANALYZER v15.20\n";
    std::cout<<"============================================================\n";
    std::cout<<"states aligned: "<<h28.size()<<"\n";
    if(yw.ok) std::cout<<"yaw mapped interval: "<<std::fixed<<std::setprecision(3)<<(yw.begin-h28.front().ts)/1e9<<" .. "<<(yw.end-h28.front().ts)/1e9<<" s\n";

    report_threshold("H28/H29",h28,h29,yw,0.1,28,29);
    report_threshold("H28/H29",h28,h29,yw,1.0,28,29);
    report_threshold("H28/H29",h28,h29,yw,10.0,28,29);
    report_threshold("H28/H29",h28,h29,yw,50.0,28,29);
    report_threshold("H28/H29",h28,h29,yw,100.0,28,29);
    std::cout<<"H29 vs H30 thresholds:\n";
    report_threshold("H29/H30",h29,h30,yw,0.1,29,30);
    report_threshold("H29/H30",h29,h30,yw,1.0,29,30);
    report_threshold("H29/H30",h29,h30,yw,10.0,29,30);
    report_threshold("H29/H30",h29,h30,yw,50.0,29,30);

    std::ofstream out(root+"/divergence_v15_20.csv");
    out<<"kf,timestamp_ns,t_s,phase,h28_h29_dp_mm,h28_h29_dv_mm_s,h28_h29_dr_deg,h29_h30_dp_mm,h29_h30_dv_mm_s,h29_h30_dr_deg,h28_drop_est,h29_drop_est,h30_drop_est\n";
    double max2829=0,max2930=0; int imax2829=-1,imax2930=-1;
    for(size_t i=0;i<h28.size();++i){
      auto a=metric(h28[i],h29[i]); auto b=metric(h29[i],h30[i]);
      if(a.dp_mm>max2829){max2829=a.dp_mm;imax2829=h28[i].kf;}
      if(b.dp_mm>max2930){max2930=b.dp_mm;imax2930=h28[i].kf;}
      out<<h28[i].kf<<','<<h28[i].ts<<','<<std::fixed<<std::setprecision(6)<<(h28[i].ts-h28.front().ts)/1e9<<','<<phase(h28[i].ts,yw)<<','
         <<std::setprecision(6)<<a.dp_mm<<','<<a.dv_mm_s<<','<<a.drnorm<<','<<b.dp_mm<<','<<b.dv_mm_s<<','<<b.drnorm<<','
         <<h28[i].kf-28<<','<<h28[i].kf-29<<','<<h28[i].kf-30<<'\n';
    }
    std::cout<<"max H28/H29 separation: "<<std::fixed<<std::setprecision(3)<<max2829<<" mm at kf="<<imax2829<<"\n";
    std::cout<<"max H29/H30 separation: "<<max2930<<" mm at kf="<<imax2930<<"\n";
    std::cout<<"CSV: "<<root<<"/divergence_v15_20.csv\n";
    std::cout<<"RESULT: PASS\n";
  }catch(const std::exception&e){ std::cerr<<"RESULT: FAIL: "<<e.what()<<"\n"; return 2; }
}
