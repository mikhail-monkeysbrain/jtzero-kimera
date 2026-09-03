// JT-ZERO Stage 11 gravity/trajectory causality diagnostic v15.9.
// Offline only. Does NOT run Kimera and does NOT modify production code.
// Correlates the exact production gravity-feedback correction reconstructed from
// the recorded IMU with already generated CURRENT vs OBSERVE_ONLY backend states.

#include "jtzero_imu_correction.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr double kRadToDeg = 180.0 / M_PI;
constexpr double kYawActiveRadS = 5.0 * M_PI / 180.0;

std::vector<std::string> splitCsv(const std::string& line) {
  std::vector<std::string> out;
  std::string cur;
  for (char ch : line) {
    if (ch == ',') { out.push_back(cur); cur.clear(); }
    else cur.push_back(ch);
  }
  out.push_back(cur);
  return out;
}

long long i64v(const std::string& s) { return s.empty() ? 0 : std::stoll(s); }
double dv(const std::string& s) { return s.empty() ? 0.0 : std::stod(s); }

struct ImuRow {
  int64_t source_ns = 0;
  int64_t mapped_ns = 0;
  double ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
};

struct StateRow {
  long long keyframe = 0;
  int64_t timestamp_ns = 0;
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
};

std::vector<ImuRow> loadImu(const std::string& path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("Cannot open combined CSV: " + path);
  std::vector<ImuRow> out;
  std::string line;
  std::getline(f, line);
  while (std::getline(f, line)) {
    const auto c = splitCsv(line);
    if (c.size() < 14 || c[0] != "IMU") continue;
    ImuRow r;
    r.source_ns=i64v(c[2]); r.mapped_ns=i64v(c[3]);
    r.ax=dv(c[8]); r.ay=dv(c[9]); r.az=dv(c[10]);
    r.gx=dv(c[11]); r.gy=dv(c[12]); r.gz=dv(c[13]);
    if (r.source_ns > 0 && r.mapped_ns > 0) out.push_back(r);
  }
  if (out.empty()) throw std::runtime_error("No IMU rows in combined CSV");
  return out;
}

std::vector<StateRow> loadStates(const std::string& path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("Cannot open states CSV: " + path);
  std::vector<StateRow> out;
  std::string line;
  std::getline(f, line);
  while (std::getline(f, line)) {
    const auto c = splitCsv(line);
    if (c.size() < 11) continue;
    StateRow r;
    r.keyframe=i64v(c[0]); r.timestamp_ns=i64v(c[1]);
    r.p=Eigen::Vector3d(dv(c[2]),dv(c[3]),dv(c[4]));
    r.rpy=Eigen::Vector3d(dv(c[8]),dv(c[9]),dv(c[10]));
    out.push_back(r);
  }
  if (out.empty()) throw std::runtime_error("No states in CSV: " + path);
  return out;
}

double wrapDeg(double x) {
  while (x > 180.0) x -= 360.0;
  while (x < -180.0) x += 360.0;
  return x;
}

struct CorrSample {
  int64_t mapped_ns = 0;
  double source_t_s = 0.0;
  Eigen::Vector3d correction = Eigen::Vector3d::Zero();
  double dt = 0.0;
  bool applied = false;
};

class DiagnosticCorrection {
 public:
  CorrSample process(const ImuRow& r, uint64_t source0_us) {
    CorrSample out;
    const uint64_t us = static_cast<uint64_t>(r.source_ns / 1000LL);
    out.mapped_ns = r.mapped_ns;
    out.source_t_s = static_cast<double>(us - source0_us) * 1e-6;

    const Eigen::Vector3d accel = jtzero::ImuCorrection::accelFrdToFlu(r.ax,r.ay,r.az);
    const Eigen::Vector3d gyro = jtzero::ImuCorrection::gyroFrdToFlu(r.gx,r.gy,r.gz);
    const Eigen::Vector3d gyro_zxy = jtzero::ImuCorrection::applyZxy(gyro);

    if (last_us_ != 0 && us > last_us_) out.dt = static_cast<double>(us-last_us_)*1e-6;
    last_us_ = us;
    if (out.dt <= 0.0 || out.dt > 0.03) return out;

    if (!accel_lp_initialized_) { accel_lp_=accel; accel_lp_initialized_=true; }
    else {
      const double alpha=std::exp(-out.dt/jtzero::ImuCorrection::kAccelLpTauSec);
      accel_lp_=alpha*accel_lp_+(1.0-alpha)*accel;
    }

    const double acc_norm=accel.norm();
    const bool gravity_ok=acc_norm>1e-6 &&
      std::abs(acc_norm-jtzero::ImuCorrection::kGravityMps2)<=jtzero::ImuCorrection::kGravityAccTolMps2;
    const bool gyro_quiet=gyro_zxy.norm()<=jtzero::ImuCorrection::kStaticGyroMaxRadS;
    const bool accel_quiet=(accel-accel_lp_).norm()<=jtzero::ImuCorrection::kStaticAccelResidualMaxMps2;
    const bool static_sample=gravity_ok&&gyro_quiet&&accel_quiet;
    if (static_sample) static_time_sec_ += out.dt; else static_time_sec_=0.0;
    const bool static_confirmed=static_time_sec_>=jtzero::ImuCorrection::kStaticHoldSec;

    if (!initialized_) {
      if (static_confirmed) { gravity_body_=accel_lp_.normalized(); initialized_=true; }
      return out;
    }

    Eigen::Vector3d corrected=gyro_zxy;
    if (static_confirmed) {
      const Eigen::Vector3d measured=accel_lp_.normalized();
      const Eigen::Vector3d err=gravity_body_.cross(measured);
      if (err.norm()<=std::sin(jtzero::ImuCorrection::kMaxGravityErrorRad)) {
        out.correction=jtzero::ImuCorrection::kGravityKp*err;
        const double n=out.correction.norm();
        if (n>jtzero::ImuCorrection::kMaxGravityCorrectionRadS)
          out.correction*=jtzero::ImuCorrection::kMaxGravityCorrectionRadS/n;
        corrected-=out.correction;
        out.applied=out.correction.norm()>0.0;
      }
    }

    const Eigen::Vector3d theta=-corrected*out.dt;
    const double a=theta.norm();
    if (a>1e-12) {
      gravity_body_=Eigen::AngleAxisd(a,theta/a)*gravity_body_;
      gravity_body_.normalize();
    }
    return out;
  }
 private:
  bool initialized_=false;
  bool accel_lp_initialized_=false;
  uint64_t last_us_=0;
  double static_time_sec_=0.0;
  Eigen::Vector3d gravity_body_=Eigen::Vector3d(0,0,1);
  Eigen::Vector3d accel_lp_=Eigen::Vector3d::Zero();
};

struct PairRow {
  StateRow a,b;
  double dp_mm=0.0;
  double dr_deg=0.0;
  Eigen::Vector3d drpy=Eigen::Vector3d::Zero();
  Eigen::Vector3d corr_integral_deg=Eigen::Vector3d::Zero();
  int corr_count=0;
  double last_corr_age_s=std::numeric_limits<double>::quiet_NaN();
};

void printThreshold(const std::vector<PairRow>& pairs, double mm, int64_t mapped0,
                    int64_t yaw_start_map, int64_t yaw_end_map) {
  auto it=std::find_if(pairs.begin(),pairs.end(),[&](const PairRow&r){return r.dp_mm>=mm;});
  if (it==pairs.end()) {
    std::cout << "first trajectory divergence >= " << mm << " mm: never\n";
    return;
  }
  const double t=(it->a.timestamp_ns-mapped0)*1e-9;
  const char* phase=it->a.timestamp_ns<yaw_start_map?"PRE":(it->a.timestamp_ns<=yaw_end_map?"YAW":"POST");
  std::cout<<std::fixed<<std::setprecision(3)
           <<"first trajectory divergence >= "<<mm<<" mm: kf="<<it->a.keyframe
           <<" t="<<t<<" s phase="<<phase
           <<" dP_AB="<<it->dp_mm<<" mm"
           <<" dR_AB="<<it->dr_deg<<" deg"
           <<" corr_count_before="<<it->corr_count
           <<" corr_integral_before_deg=["<<it->corr_integral_deg.transpose()<<"]";
  if(std::isfinite(it->last_corr_age_s)) std::cout<<" last_corr_age="<<it->last_corr_age_s<<" s";
  std::cout<<"\n";
}

} // namespace

int main(int argc,char**argv){
  try {
    const std::string combined=argc>1?argv[1]:"/home/vio/jtzero_yaw_only_v13.csv";
    const std::string current=argc>2?argv[2]:"/home/vio/jtzero_gravity_v15_8_CURRENT.csv";
    const std::string observe=argc>3?argv[3]:"/home/vio/jtzero_gravity_v15_8_OBSERVE_ONLY.csv";
    const std::string output=argc>4?argv[4]:"/home/vio/jtzero_gravity_trajectory_v15_9.csv";

    const auto imu=loadImu(combined);
    const auto a=loadStates(current);
    const auto b=loadStates(observe);
    if(a.size()!=b.size()) throw std::runtime_error("CURRENT/OBSERVE state counts differ");

    for(size_t i=0;i<a.size();++i){
      if(a[i].keyframe!=b[i].keyframe || a[i].timestamp_ns!=b[i].timestamp_ns)
        throw std::runtime_error("CURRENT/OBSERVE backend state identity mismatch at row "+std::to_string(i));
    }

    size_t first_yaw=imu.size(),last_yaw=0;
    for(size_t i=0;i<imu.size();++i){
      const Eigen::Vector3d g=jtzero::ImuCorrection::gyroFrdToFlu(imu[i].gx,imu[i].gy,imu[i].gz);
      const Eigen::Vector3d z=jtzero::ImuCorrection::applyZxy(g);
      if(std::abs(z.z())>=kYawActiveRadS){if(first_yaw==imu.size())first_yaw=i;last_yaw=i;}
    }
    if(first_yaw==imu.size()) throw std::runtime_error("No yaw interval found");

    const uint64_t source0_us=static_cast<uint64_t>(imu.front().source_ns/1000LL);
    const int64_t mapped0=imu.front().mapped_ns;
    const int64_t yaw_start_map=imu[first_yaw].mapped_ns;
    const int64_t yaw_end_map=imu[last_yaw].mapped_ns;

    DiagnosticCorrection diag;
    std::vector<CorrSample> corr;
    corr.reserve(imu.size());
    for(const auto&r:imu) corr.push_back(diag.process(r,source0_us));

    size_t ci=0;
    Eigen::Vector3d cumulative=Eigen::Vector3d::Zero();
    int corr_count=0;
    int64_t last_corr_map=0;
    std::vector<PairRow> pairs;
    pairs.reserve(a.size());

    for(size_t i=0;i<a.size();++i){
      while(ci<corr.size() && corr[ci].mapped_ns<=a[i].timestamp_ns){
        if(corr[ci].applied){
          cumulative += corr[ci].correction*corr[ci].dt*kRadToDeg;
          ++corr_count;
          last_corr_map=corr[ci].mapped_ns;
        }
        ++ci;
      }
      PairRow p; p.a=a[i];p.b=b[i];
      p.dp_mm=(a[i].p-b[i].p).norm()*1000.0;
      p.drpy=Eigen::Vector3d(wrapDeg(a[i].rpy.x()-b[i].rpy.x()),
                            wrapDeg(a[i].rpy.y()-b[i].rpy.y()),
                            wrapDeg(a[i].rpy.z()-b[i].rpy.z()));
      p.dr_deg=p.drpy.norm();
      p.corr_integral_deg=cumulative;
      p.corr_count=corr_count;
      if(last_corr_map>0) p.last_corr_age_s=(a[i].timestamp_ns-last_corr_map)*1e-9;
      pairs.push_back(p);
    }

    std::ofstream f(output);
    if(!f) throw std::runtime_error("Cannot create output CSV: "+output);
    f<<"keyframe,t_s,phase,dp_ab_mm,droll_ab_deg,dpitch_ab_deg,dyaw_ab_deg,dr_ab_norm_deg,corr_count_before,corr_int_x_deg,corr_int_y_deg,corr_int_z_deg,last_corr_age_s\n";
    f<<std::fixed<<std::setprecision(9);
    for(const auto&p:pairs){
      const double t=(p.a.timestamp_ns-mapped0)*1e-9;
      const char* phase=p.a.timestamp_ns<yaw_start_map?"PRE":(p.a.timestamp_ns<=yaw_end_map?"YAW":"POST");
      f<<p.a.keyframe<<','<<t<<','<<phase<<','<<p.dp_mm<<','
       <<p.drpy.x()<<','<<p.drpy.y()<<','<<p.drpy.z()<<','<<p.dr_deg<<','
       <<p.corr_count<<','<<p.corr_integral_deg.x()<<','<<p.corr_integral_deg.y()<<','<<p.corr_integral_deg.z()<<',';
      if(std::isfinite(p.last_corr_age_s))f<<p.last_corr_age_s;
      f<<'\n';
    }

    size_t applied_total=0; Eigen::Vector3d total_int=Eigen::Vector3d::Zero();
    double first_corr_t=std::numeric_limits<double>::quiet_NaN();
    double last_corr_t=std::numeric_limits<double>::quiet_NaN();
    for(const auto&c:corr)if(c.applied){
      ++applied_total; total_int+=c.correction*c.dt*kRadToDeg;
      const double mt=(c.mapped_ns-mapped0)*1e-9;
      if(!std::isfinite(first_corr_t))first_corr_t=mt; last_corr_t=mt;
    }

    std::cout<<"============================================================\n";
    std::cout<<"JT-ZERO GRAVITY -> TRAJECTORY CAUSALITY v15.9\n";
    std::cout<<"============================================================\n";
    std::cout<<"IMU rows: "<<imu.size()<<"\nbackend paired states: "<<pairs.size()<<"\n";
    std::cout<<std::fixed<<std::setprecision(3)
             <<"yaw mapped interval: "<<(yaw_start_map-mapped0)*1e-9<<" .. "<<(yaw_end_map-mapped0)*1e-9<<" s\n"
             <<"gravity correction events total: "<<applied_total<<"\n"
             <<"gravity correction integral total deg XYZ: ["<<total_int.transpose()<<"]\n"
             <<"first correction mapped t: "<<first_corr_t<<" s\n"
             <<"last correction mapped t: "<<last_corr_t<<" s\n\n";

    printThreshold(pairs,0.1,mapped0,yaw_start_map,yaw_end_map);
    printThreshold(pairs,1.0,mapped0,yaw_start_map,yaw_end_map);
    printThreshold(pairs,10.0,mapped0,yaw_start_map,yaw_end_map);
    printThreshold(pairs,50.0,mapped0,yaw_start_map,yaw_end_map);
    printThreshold(pairs,100.0,mapped0,yaw_start_map,yaw_end_map);

    auto maxit=std::max_element(pairs.begin(),pairs.end(),[](const PairRow&x,const PairRow&y){return x.dp_mm<y.dp_mm;});
    if(maxit!=pairs.end()){
      std::cout<<"\nmax CURRENT/OBSERVE separation: "<<maxit->dp_mm<<" mm at kf="<<maxit->a.keyframe
               <<" t="<<(maxit->a.timestamp_ns-mapped0)*1e-9<<" s\n";
    }

    std::cout<<"CSV: "<<output<<"\nRESULT: PASS\n";
    return 0;
  } catch(const std::exception&e){
    std::cerr<<"[FATAL] "<<e.what()<<"\n";
    return 1;
  }
}
