// JT-ZERO: offline analysis of live_mono_imu_full_chain_v26 CSV/TXT.
// Finds one constant gyro bias that best aligns integrated HIGHRES_IMU
// orientation with the time-aligned FC ATTITUDE orientation over the full run.
//
// Build:
//   g++ -std=c++17 -O2 tools/analyze_full_chain_v26.cpp -o /tmp/analyze_full_chain_v26 -I/usr/include/eigen3
// Run:
//   /tmp/analyze_full_chain_v26 /home/vio/jtzero_live_full_chain_v26.csv

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace {
constexpr double kPi = 3.141592653589793238462643383279502884;

struct Row {
  int phase = 0;
  std::string phase_name;
  double dt = 0.0;
  Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d ba = Eigen::Vector3d::Zero();
  Eigen::Vector3d bg_backend = Eigen::Vector3d::Zero();
  Eigen::Vector3d bg_static = Eigen::Vector3d::Zero();
  Eigen::Matrix3d Rfc = Eigen::Matrix3d::Identity();
  Eigen::Vector3d pim = Eigen::Vector3d::Zero();
};

struct Metrics {
  double rms_rot_deg = 0.0;
  double max_rot_deg = 0.0;
  double rms_grav_deg = 0.0;
  double max_grav_deg = 0.0;
  double final_rot_deg = 0.0;
  double final_grav_deg = 0.0;
  Eigen::Vector3d final_v = Eigen::Vector3d::Zero();
  double max_vxy = 0.0;
};

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

double getd(const std::vector<std::string>& f,
            const std::unordered_map<std::string,size_t>& h,
            const char* name) {
  auto it = h.find(name);
  if (it == h.end() || it->second >= f.size()) throw std::runtime_error(std::string("missing column: ") + name);
  return std::stod(f[it->second]);
}

std::string gets(const std::vector<std::string>& f,
                 const std::unordered_map<std::string,size_t>& h,
                 const char* name) {
  auto it = h.find(name);
  if (it == h.end() || it->second >= f.size()) throw std::runtime_error(std::string("missing column: ") + name);
  return f[it->second];
}

Eigen::Matrix3d fcFromLoggedEuler(double x_deg, double y_deg, double z_deg) {
  // v26 logged RfcN.eulerAngles(0,1,2), therefore reconstruct exactly as
  // R = Rx(x) * Ry(y) * Rz(z).  This avoids interpreting the values as
  // conventional roll/pitch/yaw from MAVLink.
  const double x = x_deg * kPi / 180.0;
  const double y = y_deg * kPi / 180.0;
  const double z = z_deg * kPi / 180.0;
  return Eigen::AngleAxisd(x, Eigen::Vector3d::UnitX()).toRotationMatrix() *
         Eigen::AngleAxisd(y, Eigen::Vector3d::UnitY()).toRotationMatrix() *
         Eigen::AngleAxisd(z, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

Eigen::Matrix3d expR(const Eigen::Vector3d& wdt) {
  const double a = wdt.norm();
  if (a < 1e-15) return Eigen::Matrix3d::Identity();
  return Eigen::AngleAxisd(a, wdt / a).toRotationMatrix();
}

double rotAngleRad(const Eigen::Matrix3d& A, const Eigen::Matrix3d& B) {
  const Eigen::Matrix3d E = A.transpose() * B;
  double c = (E.trace() - 1.0) * 0.5;
  c = std::max(-1.0, std::min(1.0, c));
  return std::acos(c);
}

double gravAngleRad(const Eigen::Matrix3d& A, const Eigen::Matrix3d& B) {
  const Eigen::Vector3d z = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d a = A.transpose() * z;
  Eigen::Vector3d b = B.transpose() * z;
  double c = a.normalized().dot(b.normalized());
  c = std::max(-1.0, std::min(1.0, c));
  return std::acos(c);
}

std::vector<Row> load(const std::string& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("cannot open: " + path);
  std::string line;
  if (!std::getline(in, line)) throw std::runtime_error("empty file");
  const auto header = splitCsv(line);
  std::unordered_map<std::string,size_t> h;
  for (size_t i=0;i<header.size();++i) h[header[i]]=i;
  std::vector<Row> rows;
  while (std::getline(in,line)) {
    if (line.empty()) continue;
    const auto f = splitCsv(line);
    try {
      Row r;
      r.phase = static_cast<int>(getd(f,h,"phase"));
      r.phase_name = gets(f,h,"phase_name");
      r.dt = getd(f,h,"dt");
      r.acc << getd(f,h,"ax"), getd(f,h,"ay"), getd(f,h,"az");
      r.gyro << getd(f,h,"gx"), getd(f,h,"gy"), getd(f,h,"gz");
      r.ba << getd(f,h,"ba_x"), getd(f,h,"ba_y"), getd(f,h,"ba_z");
      r.bg_backend << getd(f,h,"bg_x"), getd(f,h,"bg_y"), getd(f,h,"bg_z");
      r.bg_static << getd(f,h,"bg_static_x"), getd(f,h,"bg_static_y"), getd(f,h,"bg_static_z");
      r.Rfc = fcFromLoggedEuler(getd(f,h,"fc_roll"), getd(f,h,"fc_pitch"), getd(f,h,"fc_yaw"));
      r.pim << getd(f,h,"pim_dv_x"), getd(f,h,"pim_dv_y"), getd(f,h,"pim_dv_z");
      rows.push_back(r);
    } catch (...) {
      // Ignore a truncated final line if a TXT copy was cut during transfer.
    }
  }
  if (rows.size() < 100) throw std::runtime_error("too few valid rows");
  return rows;
}

// A constant left alignment is allowed because v26's integrated orientation
// and logged FC matrix may be expressed in different fixed world frames.
// We align both at sample zero and judge only the subsequent relative motion.
double objective(const std::vector<Row>& rows, const Eigen::Vector3d& bg) {
  Eigen::Matrix3d R = rows.front().Rfc;
  double ss = 0.0;
  size_t n = 0;
  for (size_t i=1;i<rows.size();++i) {
    const double dt = rows[i].dt;
    if (dt > 0.0 && dt <= 0.03) R = R * expR((rows[i].gyro - bg) * dt);
    // Decimate cost evaluation, while integration still uses every sample.
    if ((i % 5) != 0 && i + 1 != rows.size()) continue;
    const double e = rotAngleRad(R, rows[i].Rfc);
    ss += e*e;
    ++n;
  }
  return ss / std::max<size_t>(1,n);
}

Eigen::Vector3d optimize(const std::vector<Row>& rows, Eigen::Vector3d start) {
  Eigen::Vector3d x = start;
  double best = objective(rows,x);
  const std::array<double,11> steps = {0.002,0.001,0.0005,0.00025,0.0001,0.00005,0.000025,0.00001,0.000005,0.0000025,0.000001};
  for (double step : steps) {
    bool improved = true;
    while (improved) {
      improved = false;
      for (int a=0;a<3;++a) {
        for (double sgn : {-1.0,1.0}) {
          Eigen::Vector3d y=x; y[a]+=sgn*step;
          const double c=objective(rows,y);
          if (c < best) { best=c; x=y; improved=true; }
        }
      }
    }
  }
  return x;
}

Metrics evaluate(const std::vector<Row>& rows, const Eigen::Vector3d& bg,
                 int only_phase = -1) {
  Eigen::Matrix3d R = rows.front().Rfc;
  Eigen::Vector3d v = Eigen::Vector3d::Zero();
  const Eigen::Vector3d gW(0,0,9.81);
  double ssr=0, ssg=0, maxr=0, maxg=0, maxv=0;
  size_t n=0;
  double last_r=0,last_g=0;
  for (size_t i=1;i<rows.size();++i) {
    const double dt=rows[i].dt;
    if (dt>0 && dt<=0.03) {
      R = R * expR((rows[i].gyro-bg)*dt);
      const Eigen::Vector3d aw = R * (rows[i].acc-rows[i].ba) + gW;
      v += aw*dt;
    }
    const double er=rotAngleRad(R,rows[i].Rfc);
    const double eg=gravAngleRad(R,rows[i].Rfc);
    if (only_phase<0 || rows[i].phase==only_phase) {
      ssr+=er*er; ssg+=eg*eg; maxr=std::max(maxr,er); maxg=std::max(maxg,eg); ++n;
      maxv=std::max(maxv,std::hypot(v.x(),v.y()));
      last_r=er; last_g=eg;
    }
  }
  Metrics m;
  if(n){m.rms_rot_deg=std::sqrt(ssr/n)*180/kPi;m.rms_grav_deg=std::sqrt(ssg/n)*180/kPi;}
  m.max_rot_deg=maxr*180/kPi;m.max_grav_deg=maxg*180/kPi;
  m.final_rot_deg=last_r*180/kPi;m.final_grav_deg=last_g*180/kPi;
  m.final_v=v;m.max_vxy=maxv;
  return m;
}

void printMetrics(const char* name, const Eigen::Vector3d& bg,
                  const std::vector<Row>& rows) {
  const Metrics m=evaluate(rows,bg);
  std::cout<<"\n"<<name<<" = ["<<bg.transpose()<<"] rad/s\n";
  std::cout<<"  RMS rot="<<m.rms_rot_deg<<" deg   max rot="<<m.max_rot_deg<<" deg\n";
  std::cout<<"  RMS grav="<<m.rms_grav_deg<<" deg  max grav="<<m.max_grav_deg<<" deg\n";
  std::cout<<"  final rot="<<m.final_rot_deg<<" deg final grav="<<m.final_grav_deg<<" deg\n";
  std::cout<<"  final V=["<<m.final_v.transpose()<<"] m/s   max Vxy="<<m.max_vxy<<" m/s\n";
  static const char* pn[4]={"+30","0_after_plus","-30","0_final"};
  for(int p=0;p<4;++p){const Metrics q=evaluate(rows,bg,p);std::cout<<"    "<<pn[p]<<": RMSrot="<<q.rms_rot_deg<<" maxRot="<<q.max_rot_deg<<" RMSgrav="<<q.rms_grav_deg<<" maxGrav="<<q.max_grav_deg<<" deg\n";}
}

} // namespace

int main(int argc,char**argv){
  if(argc<2){std::cerr<<"Usage: "<<argv[0]<<" <v26 csv/txt>\n";return 2;}
  try{
    const auto rows=load(argv[1]);
    const Eigen::Vector3d bg_backend=rows.front().bg_backend;
    const Eigen::Vector3d bg_static=rows.front().bg_static;
    const Eigen::Vector3d opt_from_static=optimize(rows,bg_static);
    const Eigen::Vector3d opt_from_backend=optimize(rows,bg_backend);
    const double cs=objective(rows,opt_from_static), cb=objective(rows,opt_from_backend);
    const Eigen::Vector3d bg_opt=(cs<=cb)?opt_from_static:opt_from_backend;

    std::cout<<std::fixed<<std::setprecision(9);
    std::cout<<"============================================================\n";
    std::cout<<"JT-ZERO OFFLINE BG OPTIMIZER FOR v26\n";
    std::cout<<"============================================================\n";
    std::cout<<"rows: "<<rows.size()<<"\n";
    std::cout<<"BG backend: ["<<bg_backend.transpose()<<"]\n";
    std::cout<<"BG static : ["<<bg_static.transpose()<<"]\n";
    std::cout<<"BG opt    : ["<<bg_opt.transpose()<<"]\n";
    std::cout<<"opt-backend: ["<<(bg_opt-bg_backend).transpose()<<"]\n";
    std::cout<<"opt-static : ["<<(bg_opt-bg_static).transpose()<<"]\n";
    printMetrics("BACKEND",bg_backend,rows);
    printMetrics("STATIC",bg_static,rows);
    printMetrics("OPT",bg_opt,rows);

    const double e_static=std::sqrt(objective(rows,bg_static))*180/kPi;
    const double e_opt=std::sqrt(objective(rows,bg_opt))*180/kPi;
    std::cout<<"\nDecision helper:\n";
    std::cout<<"  orientation RMS objective static="<<e_static<<" deg opt="<<e_opt<<" deg\n";
    if(e_opt < 0.35 && e_opt < e_static*0.45)
      std::cout<<"  RESULT: one constant BG explains most orientation drift. Focus on gyro-bias initialization.\n";
    else if(e_opt < e_static*0.75)
      std::cout<<"  RESULT: constant BG helps materially, but does not explain the full error. Check gyro scale/misalignment/cross-axis next.\n";
    else
      std::cout<<"  RESULT: one constant BG is insufficient. Bias alone is not the root cause; check scale/misalignment/timing/source semantics.\n";
    return 0;
  }catch(const std::exception&e){std::cerr<<"ERROR: "<<e.what()<<"\n";return 1;}
}
