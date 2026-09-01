// JT-ZERO v16: offline keyframe analyzer for v15 CSV.
// Purpose: localize whether gravity-direction error appears smoothly during motion
// or as jumps at backend/PIM keyframe boundaries.

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
#include <unordered_map>
#include <vector>

namespace {

constexpr const char* kDefaultCsv = "/home/vio/jtzero_live_attitude_error_v15.csv";

std::vector<std::string> splitCsv(const std::string& s) {
  std::vector<std::string> out;
  std::string cur;
  bool quoted = false;
  for (char c : s) {
    if (c == '"') quoted = !quoted;
    else if (c == ',' && !quoted) { out.push_back(cur); cur.clear(); }
    else cur.push_back(c);
  }
  out.push_back(cur);
  return out;
}

double num(const std::vector<std::string>& v, const std::unordered_map<std::string,size_t>& h,
           const std::string& name, double def=0.0) {
  auto it=h.find(name); if(it==h.end() || it->second>=v.size()) return def;
  try { return std::stod(v[it->second]); } catch(...) { return def; }
}

int64_t i64(const std::vector<std::string>& v, const std::unordered_map<std::string,size_t>& h,
            const std::string& name, int64_t def=0) {
  auto it=h.find(name); if(it==h.end() || it->second>=v.size()) return def;
  try { return std::stoll(v[it->second]); } catch(...) { return def; }
}

struct Sample {
  int64_t keyframe=0;
  int64_t backend_ns=0;
  double age_ms=0;
  double dt=0;
  double gx=0,gy=0,gz=0;
  double ge_opt=0,ge_pim=0;
  double re_opt=0,re_pim=0;
  double derr_opt=0,derr_pim=0;
  double verr_opt=0,verr_pim=0;
  double backend_vxy=0,pim_vxy=0;
  double gyro_norm=0;
};

struct Kf {
  int64_t id=0;
  int64_t backend_ns=0;
  size_t n=0;
  double first_pim=0,last_pim=0,min_pim=1e9,max_pim=0,sum_pim=0;
  double first_opt=0,last_opt=0,min_opt=1e9,max_opt=0,sum_opt=0;
  double first_rot_pim=0,last_rot_pim=0,max_rot_pim=0;
  double max_gyro=0,sum_gyro=0;
  double max_derr_pim=0,max_verr_pim=0,max_pim_vxy=0;
  double first_age=0,last_age=0;
  bool init=false;
};

void add(Kf& k,const Sample&s){
  if(!k.init){k.id=s.keyframe;k.backend_ns=s.backend_ns;k.first_pim=s.ge_pim;k.first_opt=s.ge_opt;k.first_rot_pim=s.re_pim;k.first_age=s.age_ms;k.init=true;}
  k.last_pim=s.ge_pim;k.last_opt=s.ge_opt;k.last_rot_pim=s.re_pim;k.last_age=s.age_ms;
  k.min_pim=std::min(k.min_pim,s.ge_pim);k.max_pim=std::max(k.max_pim,s.ge_pim);k.sum_pim+=s.ge_pim;
  k.min_opt=std::min(k.min_opt,s.ge_opt);k.max_opt=std::max(k.max_opt,s.ge_opt);k.sum_opt+=s.ge_opt;
  k.max_rot_pim=std::max(k.max_rot_pim,s.re_pim);
  k.max_gyro=std::max(k.max_gyro,s.gyro_norm);k.sum_gyro+=s.gyro_norm;
  k.max_derr_pim=std::max(k.max_derr_pim,s.derr_pim);k.max_verr_pim=std::max(k.max_verr_pim,s.verr_pim);k.max_pim_vxy=std::max(k.max_pim_vxy,s.pim_vxy);
  ++k.n;
}

struct Jump {int64_t from=0,to=0;double jump=0;double prev_last=0,next_first=0;double gyro=0;};

} // namespace

int main(int argc,char**argv){
  const std::string path = argc>1 ? argv[1] : kDefaultCsv;
  std::ifstream f(path);
  if(!f){std::cerr<<"[FATAL] Не удалось открыть CSV: "<<path<<"\n";return 1;}

  std::string line;
  if(!std::getline(f,line)){std::cerr<<"[FATAL] CSV пуст.\n";return 1;}
  const auto header=splitCsv(line);
  std::unordered_map<std::string,size_t> h;
  for(size_t i=0;i<header.size();++i)h[header[i]]=i;
  const char* required[]={"keyframe","backend_ns","backend_age_ms","gx","gy","gz","gravity_err_opt_deg","gravity_err_pim_deg","rot_err_pim_deg","derr_pim_xy","verr_pim_xy","pim_vxy"};
  for(const char* n:required)if(!h.count(n)){std::cerr<<"[FATAL] Нет колонки "<<n<<". Нужен CSV от v15.\n";return 1;}

  std::vector<Sample> samples;
  std::map<int64_t,Kf> kfs;
  double max_ge_pim=0,max_ge_opt=0,max_gyro=0;
  size_t moving_n=0,still_n=0; double moving_sum=0,still_sum=0;
  int64_t first_gt1=-1,first_gt2=-1,first_gt4=-1;

  while(std::getline(f,line)){
    if(line.empty())continue;
    auto v=splitCsv(line);
    Sample s;
    s.keyframe=i64(v,h,"keyframe");s.backend_ns=i64(v,h,"backend_ns");s.age_ms=num(v,h,"backend_age_ms");s.dt=num(v,h,"dt");
    s.gx=num(v,h,"gx");s.gy=num(v,h,"gy");s.gz=num(v,h,"gz");s.gyro_norm=std::sqrt(s.gx*s.gx+s.gy*s.gy+s.gz*s.gz);
    s.ge_opt=num(v,h,"gravity_err_opt_deg");s.ge_pim=num(v,h,"gravity_err_pim_deg");
    s.re_opt=num(v,h,"rot_err_opt_deg");s.re_pim=num(v,h,"rot_err_pim_deg");
    s.derr_opt=num(v,h,"derr_opt_xy");s.derr_pim=num(v,h,"derr_pim_xy");
    s.verr_opt=num(v,h,"verr_opt_xy");s.verr_pim=num(v,h,"verr_pim_xy");
    s.backend_vxy=num(v,h,"backend_vxy");s.pim_vxy=num(v,h,"pim_vxy");
    samples.push_back(s); add(kfs[s.keyframe],s);
    max_ge_pim=std::max(max_ge_pim,s.ge_pim);max_ge_opt=std::max(max_ge_opt,s.ge_opt);max_gyro=std::max(max_gyro,s.gyro_norm);
    if(s.gyro_norm>0.08){moving_sum+=s.ge_pim;++moving_n;}else{still_sum+=s.ge_pim;++still_n;}
    if(first_gt1<0&&s.ge_pim>=1.0)first_gt1=s.keyframe;
    if(first_gt2<0&&s.ge_pim>=2.0)first_gt2=s.keyframe;
    if(first_gt4<0&&s.ge_pim>=4.0)first_gt4=s.keyframe;
  }
  if(samples.empty()){std::cerr<<"[FATAL] Нет строк данных.\n";return 1;}

  std::vector<Kf> kv;kv.reserve(kfs.size());for(const auto& p:kfs)kv.push_back(p.second);
  std::sort(kv.begin(),kv.end(),[](const Kf&a,const Kf&b){return a.backend_ns<b.backend_ns;});
  std::vector<Jump> jumps;
  for(size_t i=1;i<kv.size();++i){Jump j;j.from=kv[i-1].id;j.to=kv[i].id;j.prev_last=kv[i-1].last_pim;j.next_first=kv[i].first_pim;j.jump=j.next_first-j.prev_last;j.gyro=std::max(kv[i-1].max_gyro,kv[i].max_gyro);jumps.push_back(j);}
  std::sort(jumps.begin(),jumps.end(),[](const Jump&a,const Jump&b){return std::abs(a.jump)>std::abs(b.jump);});

  std::vector<Kf> byPeak=kv;
  std::sort(byPeak.begin(),byPeak.end(),[](const Kf&a,const Kf&b){return a.max_pim>b.max_pim;});

  std::cout<<std::fixed<<std::setprecision(4);
  std::cout<<"\n============================================================\n";
  std::cout<<"JT-ZERO v16 — АНАЛИЗ ОШИБКИ GRAVITY ПО KEYFRAME\n";
  std::cout<<"============================================================\n";
  std::cout<<"CSV: "<<path<<"\n";
  std::cout<<"строк: "<<samples.size()<<"\nkeyframes: "<<kv.size()<<"\n";
  std::cout<<"max gravity error PIM: "<<max_ge_pim<<" deg\n";
  std::cout<<"max gravity error OPT: "<<max_ge_opt<<" deg\n";
  std::cout<<"max |gyro|: "<<max_gyro<<" rad/s\n";
  std::cout<<"mean gravity error PIM при движении (|gyro|>0.08): "<<(moving_n?moving_sum/moving_n:0)<<" deg\n";
  std::cout<<"mean gravity error PIM в покое: "<<(still_n?still_sum/still_n:0)<<" deg\n";
  std::cout<<"первый KF с PIM gravity >=1 deg: "<<first_gt1<<"\n";
  std::cout<<"первый KF с PIM gravity >=2 deg: "<<first_gt2<<"\n";
  std::cout<<"первый KF с PIM gravity >=4 deg: "<<first_gt4<<"\n";

  std::cout<<"\nТОП СКАЧКОВ НА ГРАНИЦЕ KEYFRAME (first[next] - last[prev]):\n";
  std::cout<<" from -> to      jump_deg   prev_last   next_first   maxGyro\n";
  for(size_t i=0;i<std::min<size_t>(12,jumps.size());++i){const auto&j=jumps[i];std::cout<<std::setw(5)<<j.from<<" -> "<<std::setw(5)<<j.to<<"   "<<std::setw(9)<<j.jump<<"   "<<std::setw(9)<<j.prev_last<<"   "<<std::setw(10)<<j.next_first<<"   "<<std::setw(8)<<j.gyro<<"\n";}

  std::cout<<"\nТОП KEYFRAME ПО PIM GRAVITY ERROR:\n";
  std::cout<<" kf      first   last    min     max     mean    maxGyro   max_dA   max_int_dV   maxPIMV\n";
  for(size_t i=0;i<std::min<size_t>(15,byPeak.size());++i){const auto&k=byPeak[i];std::cout<<std::setw(5)<<k.id<<"   "<<std::setw(6)<<k.first_pim<<"  "<<std::setw(6)<<k.last_pim<<"  "<<std::setw(6)<<k.min_pim<<"  "<<std::setw(6)<<k.max_pim<<"  "<<std::setw(7)<<(k.n?k.sum_pim/k.n:0)<<"   "<<std::setw(8)<<k.max_gyro<<"   "<<std::setw(7)<<k.max_derr_pim<<"   "<<std::setw(10)<<k.max_verr_pim<<"   "<<std::setw(7)<<k.max_pim_vxy<<"\n";}

  double max_abs_jump=0;for(const auto&j:jumps)max_abs_jump=std::max(max_abs_jump,std::abs(j.jump));
  std::cout<<"\nПРЕДВАРИТЕЛЬНАЯ КЛАССИФИКАЦИЯ:\n";
  if(max_abs_jump>=2.0)std::cout<<"Крупные скачки >=2 deg на границах KF присутствуют. Следующий шаг: инструментировать start-R и deltaR PIM.\n";
  else if(max_ge_pim>=4.0)std::cout<<"Ошибка большая, но без крупных boundary-jump. Вероятнее постепенное накопление/рассогласование во время вращения. Следующий шаг: логировать start-R и deltaR PIM.\n";
  else std::cout<<"Большая gravity-ошибка в этом прогоне не подтверждается. Нужен повтор v15 или проверка выбранного CSV.\n";
  std::cout<<"============================================================\n";
  return 0;
}
