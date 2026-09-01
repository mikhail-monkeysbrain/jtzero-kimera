#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <Eigen/Dense>
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
constexpr int kStride = 4;                    // 120 FPS -> ~30 FPS
constexpr int kMinCorners = 12;
constexpr double kMaxReprojPx = 1.5;
constexpr int64_t kMaxRawDtNs = 20'000'000LL;
constexpr double kPi = 3.14159265358979323846;
constexpr double kStationaryAngularDegS = 3.0;
constexpr double kStationaryLinearMS = 0.020;
constexpr double kMinSegmentS = 0.50;
constexpr double kMergeGapS = 0.40;
constexpr double kMergeAngleDeg = 2.0;
constexpr double kMergePositionM = 0.010;
constexpr double kExpectedX = 0.0;
constexpr double kExpectedY = 0.0;
constexpr double kExpectedZ = 0.055;

struct Frame {
  size_t raw_index{};
  uint32_t sequence{};
  int64_t ts_ns{};
  uint64_t offset{};
  uint32_t bytes{};
  bool continuous{};
};

struct Pose {
  size_t raw_index{};
  int64_t ts_ns{};
  Eigen::Matrix3d R_WC = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p_WC = Eigen::Vector3d::Zero();
  double reproj{};
};

struct Segment {
  size_t begin{};
  size_t end{};
  double duration_s{};
  Eigen::Matrix3d R_WC = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p_WC = Eigen::Vector3d::Zero();
};

std::vector<std::string> split(const std::string& s) {
  std::vector<std::string> out; std::stringstream ss(s); std::string f;
  while (std::getline(ss,f,',')) out.push_back(f);
  if (!s.empty() && s.back()==',') out.emplace_back();
  return out;
}

double median(std::vector<double> v) {
  if (v.empty()) return std::numeric_limits<double>::quiet_NaN();
  std::sort(v.begin(),v.end()); const size_t n=v.size();
  return (n&1U)?v[n/2]:0.5*(v[n/2-1]+v[n/2]);
}

double percentile(std::vector<double> v,double q) {
  if(v.empty()) return std::numeric_limits<double>::quiet_NaN();
  std::sort(v.begin(),v.end());
  const double p=q*(v.size()-1);
  const size_t lo=(size_t)std::floor(p), hi=std::min(lo+1,v.size()-1);
  const double a=p-lo;
  return v[lo]*(1.0-a)+v[hi]*a;
}

std::vector<Frame> loadIndex(const std::string& path,size_t* discontinuities) {
  std::ifstream in(path); if(!in) throw std::runtime_error("Cannot open camera index: "+path);
  std::string line; if(!std::getline(in,line)) throw std::runtime_error("Empty camera index");
  std::vector<Frame> out; bool have=false; uint32_t prev_seq=0; int64_t prev_ts=0; *discontinuities=0;
  while(std::getline(in,line)) {
    if(line.empty()) continue; const auto c=split(line); if(c.size()<7) continue;
    Frame f; f.raw_index=out.size(); f.sequence=(uint32_t)std::stoul(c[0]); f.ts_ns=std::stoll(c[2]); f.offset=std::stoull(c[5]); f.bytes=(uint32_t)std::stoul(c[6]);
    if(have) {
      const int64_t dt=f.ts_ns-prev_ts;
      f.continuous=(f.sequence==prev_seq+1U && dt>0 && dt<=kMaxRawDtNs);
      if(!f.continuous) ++*discontinuities;
    }
    prev_seq=f.sequence; prev_ts=f.ts_ns; have=true; out.push_back(f);
  }
  return out;
}

bool intervalContinuous(const std::vector<Frame>& f,size_t a,size_t b) {
  if(b<=a || b>=f.size()) return false;
  for(size_t i=a+1;i<=b;++i) if(!f[i].continuous) return false;
  return true;
}

bool readFrame(std::ifstream& in,const Frame& f,cv::Mat* image) {
  std::vector<unsigned char> bytes(f.bytes);
  in.clear(); in.seekg((std::streamoff)f.offset,std::ios::beg); if(!in) return false;
  in.read((char*)bytes.data(),(std::streamsize)bytes.size()); if(in.gcount()!=(std::streamsize)bytes.size()) return false;
  *image=cv::imdecode(bytes,cv::IMREAD_GRAYSCALE); return !image->empty();
}

Eigen::Matrix3d mat33(const cv::Mat& m) {
  Eigen::Matrix3d R; for(int r=0;r<3;++r) for(int c=0;c<3;++c) R(r,c)=m.at<double>(r,c); return R;
}
Eigen::Vector3d vec3(const cv::Mat& m) { return {m.at<double>(0),m.at<double>(1),m.at<double>(2)}; }

double reprojRmse(const std::vector<cv::Point3f>& obj,const std::vector<cv::Point2f>& img,
                   const cv::Mat& rv,const cv::Mat& tv,const cv::Mat& K,const cv::Mat& D) {
  std::vector<cv::Point2f> pr; cv::projectPoints(obj,rv,tv,K,D,pr); if(pr.empty()) return 1e9;
  double s=0; for(size_t i=0;i<pr.size();++i) { const auto d=pr[i]-img[i]; s += double(d.x*d.x+d.y*d.y); }
  return std::sqrt(s/pr.size());
}

double rotAngleDeg(const Eigen::Matrix3d& A,const Eigen::Matrix3d& B) {
  const Eigen::Matrix3d d=A.transpose()*B;
  const double c=std::clamp((d.trace()-1.0)*0.5,-1.0,1.0);
  return std::acos(c)*180.0/kPi;
}

Eigen::Matrix3d meanRotation(const std::vector<Pose>& p,size_t a,size_t b) {
  Eigen::Matrix3d M=Eigen::Matrix3d::Zero(); for(size_t i=a;i<=b;++i) M+=p[i].R_WC;
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(M,Eigen::ComputeFullU|Eigen::ComputeFullV);
  Eigen::Matrix3d U=svd.matrixU(); Eigen::Matrix3d R=U*svd.matrixV().transpose();
  if(R.determinant()<0) { U.col(2)*=-1; R=U*svd.matrixV().transpose(); }
  return R;
}

Eigen::Vector3d meanPosition(const std::vector<Pose>& p,size_t a,size_t b) {
  Eigen::Vector3d x=Eigen::Vector3d::Zero(); for(size_t i=a;i<=b;++i) x+=p[i].p_WC; return x/double(b-a+1);
}

Segment makeSegment(const std::vector<Pose>& poses,size_t a,size_t b) {
  Segment s; s.begin=a; s.end=b; s.duration_s=(poses[b].ts_ns-poses[a].ts_ns)*1e-9;
  s.R_WC=meanRotation(poses,a,b); s.p_WC=meanPosition(poses,a,b); return s;
}

std::vector<Segment> mergeSegments(const std::vector<Pose>& poses,const std::vector<Segment>& input) {
  if(input.empty()) return {};
  std::vector<Segment> out; out.push_back(input.front());
  for(size_t i=1;i<input.size();++i) {
    Segment& prev=out.back(); const Segment& cur=input[i];
    const double gap=(poses[cur.begin].ts_ns-poses[prev.end].ts_ns)*1e-9;
    const double da=rotAngleDeg(prev.R_WC,cur.R_WC);
    const double dp=(prev.p_WC-cur.p_WC).norm();
    if(gap<=kMergeGapS && da<=kMergeAngleDeg && dp<=kMergePositionM) {
      prev=makeSegment(poses,prev.begin,cur.end);
    } else out.push_back(cur);
  }
  return out;
}

} // namespace

int main(int argc,char** argv) {
  try {
    const std::string camera_csv=argc>1?argv[1]:"/home/vio/camera_imu_translation_centered_yaw_40s_camera.csv";
    const std::string mjpeg_path=argc>2?argv[2]:"/home/vio/camera_imu_translation_centered_yaw_40s.mjpg";
    const std::string intrinsics_path=argc>3?argv[3]:"calibration/ov9281_intrinsics.yaml";

    cv::FileStorage fs(intrinsics_path,cv::FileStorage::READ); if(!fs.isOpened()) throw std::runtime_error("Cannot open intrinsics YAML");
    int sx=0,sy=0; double square_mm=0,marker_mm=0; cv::Mat K,D;
    fs["board_squares_x"]>>sx; fs["board_squares_y"]>>sy; fs["board_square_length_mm"]>>square_mm; fs["board_marker_length_mm"]>>marker_mm;
    fs["camera_matrix"]>>K; fs["distortion_coefficients"]>>D;
    if(sx<=1||sy<=1||square_mm<=0||marker_mm<=0||K.empty()||D.empty()) throw std::runtime_error("Invalid intrinsics/board parameters");

    size_t raw_disc=0; const auto frames=loadIndex(camera_csv,&raw_disc);
    std::ifstream mjpeg(mjpeg_path,std::ios::binary); if(!mjpeg) throw std::runtime_error("Cannot open MJPEG: "+mjpeg_path);
    auto dict=cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::CharucoBoard board(cv::Size(sx,sy),(float)(square_mm*1e-3),(float)(marker_mm*1e-3),dict);
    cv::aruco::CharucoDetector detector(board);

    std::vector<Pose> poses; size_t attempts=0,decode_fail=0,detect_fail=0,reproj_reject=0,continuity_reject=0;
    for(size_t i=0;i<frames.size();i+=kStride) {
      ++attempts;
      if(i>0 && !intervalContinuous(frames,i-kStride,i)) { ++continuity_reject; continue; }
      cv::Mat image; if(!readFrame(mjpeg,frames[i],&image)){++decode_fail;continue;}
      cv::Mat cc,ci; detector.detectBoard(image,cc,ci);
      if(ci.total()<(size_t)kMinCorners){++detect_fail;continue;}
      std::vector<cv::Point3f> obj; std::vector<cv::Point2f> img; board.matchImagePoints(cc,ci,obj,img);
      if(obj.size()<(size_t)kMinCorners){++detect_fail;continue;}
      cv::Mat rv,tv; if(!cv::solvePnP(obj,img,K,D,rv,tv,false,cv::SOLVEPNP_ITERATIVE)){++detect_fail;continue;}
      const double e=reprojRmse(obj,img,rv,tv,K,D); if(!std::isfinite(e)||e>kMaxReprojPx){++reproj_reject;continue;}
      cv::Mat Rcv; cv::Rodrigues(rv,Rcv);
      Pose p; p.raw_index=i; p.ts_ns=frames[i].ts_ns;
      const Eigen::Matrix3d R_CW=mat33(Rcv); const Eigen::Vector3d t_CW=vec3(tv);
      p.R_WC=R_CW.transpose(); p.p_WC=-p.R_WC*t_CW; p.reproj=e; poses.push_back(p);
    }
    if(poses.size()<50) throw std::runtime_error("Too few valid ChArUco poses");

    std::vector<bool> stationary(poses.size(),false);
    for(size_t i=1;i<poses.size();++i) {
      const double dt=(poses[i].ts_ns-poses[i-1].ts_ns)*1e-9; if(dt<=0||dt>0.2) continue;
      const double w=rotAngleDeg(poses[i-1].R_WC,poses[i].R_WC)/dt;
      const double v=(poses[i].p_WC-poses[i-1].p_WC).norm()/dt;
      stationary[i]=(w<=kStationaryAngularDegS && v<=kStationaryLinearMS);
    }

    std::vector<Segment> raw_segments; size_t i=1;
    while(i<stationary.size()) {
      while(i<stationary.size()&&!stationary[i]) ++i; if(i>=stationary.size()) break;
      const size_t a=i; while(i+1<stationary.size()&&stationary[i+1]) ++i; const size_t b=i;
      const double dur=(poses[b].ts_ns-poses[a].ts_ns)*1e-9;
      if(dur>=kMinSegmentS) raw_segments.push_back(makeSegment(poses,a,b));
      ++i;
    }
    const auto segs=mergeSegments(poses,raw_segments);
    if(segs.size()<4) throw std::runtime_error("Too few stationary segments after merging; need at least 4");

    // Accepted Stage-11 rotation from the previous mount. For centered XY validation,
    // small residual rotation error has little effect because expected X/Y are zero.
    Eigen::Matrix3d R_CB; R_CB <<
      -0.010624404,  0.995285416,  0.096405718,
      -0.998107308, -0.016395603,  0.059270445,
       0.060571639, -0.095593539,  0.993575841;

    const Eigen::Matrix3d R_WB0=segs[0].R_WC*R_CB;
    const Eigen::Vector3d p0=segs[0].p_WC;
    Eigen::MatrixXd A(3*(segs.size()-1),2); Eigen::VectorXd b(3*(segs.size()-1));
    std::vector<double> spans;
    for(size_t k=1;k<segs.size();++k) {
      const Eigen::Matrix3d R_WB=segs[k].R_WC*R_CB;
      const Eigen::Matrix3d dR=R_WB-R_WB0;
      const Eigen::Vector3d dp=segs[k].p_WC-p0;
      A.block<3,1>(3*(k-1),0)=dR.col(0); A.block<3,1>(3*(k-1),1)=dR.col(1); b.segment<3>(3*(k-1))=dp;
      spans.push_back(rotAngleDeg(R_WB0,R_WB));
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A,Eigen::ComputeThinU|Eigen::ComputeThinV);
    const Eigen::Vector2d txy=svd.solve(b);
    const Eigen::VectorXd residual=A*txy-b;
    std::vector<double> residual_mm; for(int r=0;r<residual.size()/3;++r) residual_mm.push_back(residual.segment<3>(3*r).norm()*1000.0);
    std::vector<double> reproj; for(const auto& p:poses) reproj.push_back(p.reproj);

    const double span=spans.empty()?0.0:*std::max_element(spans.begin(),spans.end());
    const double xy_norm=txy.norm();
    const double cond=(svd.singularValues().size()>=2 && svd.singularValues()(1)>1e-12)?svd.singularValues()(0)/svd.singularValues()(1):1e12;
    const bool motion_pass=span>=45.0;
    const bool x_pass=std::abs(txy.x()-kExpectedX)<=0.010;
    const bool y_pass=std::abs(txy.y()-kExpectedY)<=0.010;
    const bool norm_pass=xy_norm<=0.012;
    const bool fit_pass=percentile(residual_mm,0.95)<=15.0;
    const bool cond_pass=cond<=10.0;

    std::cout<<std::fixed<<std::setprecision(6);
    std::cout<<"============================================================\n";
    std::cout<<"CENTERED CAMERA-IMU TRANSLATION / YAW VALIDATION\n";
    std::cout<<"============================================================\n";
    std::cout<<"raw camera frames:          "<<frames.size()<<"\n";
    std::cout<<"raw discontinuities:        "<<raw_disc<<"\n";
    std::cout<<"pose attempts:              "<<attempts<<"\n";
    std::cout<<"valid ChArUco poses:        "<<poses.size()<<"\n";
    std::cout<<"decode failures:            "<<decode_fail<<"\n";
    std::cout<<"detection failures:         "<<detect_fail<<"\n";
    std::cout<<"reprojection rejects:       "<<reproj_reject<<"\n";
    std::cout<<"continuity rejects:         "<<continuity_reject<<"\n";
    std::cout<<"reprojection median/p95:    "<<median(reproj)<<" / "<<percentile(reproj,0.95)<<" px\n";
    std::cout<<"raw stationary segments:    "<<raw_segments.size()<<"\n";
    std::cout<<"merged stationary segments: "<<segs.size()<<"\n";
    std::cout<<"max orientation span:       "<<span<<" deg\n";

    for(size_t k=0;k<segs.size();++k) {
      const Eigen::Matrix3d R_WBk=segs[k].R_WC*R_CB;
      const double ang=rotAngleDeg(R_WB0,R_WBk);
      const Eigen::Vector3d dp=(segs[k].p_WC-p0)*1000.0;
      std::cout<<"  segment "<<k<<": duration="<<segs[k].duration_s
               <<" s, angle_from_0="<<ang
               <<" deg, dP_WC=["<<dp.x()<<", "<<dp.y()<<", "<<dp.z()<<"] mm\n";
    }

    std::cout<<"\nEstimated horizontal t_BC:\n";
    std::cout<<"  X:                       "<<txy.x()*1000.0<<" mm\n";
    std::cout<<"  Y:                       "<<txy.y()*1000.0<<" mm\n";
    std::cout<<"  norm XY:                 "<<xy_norm*1000.0<<" mm\n";
    std::cout<<"expected X/Y:               0.000 / 0.000 mm\n";
    std::cout<<"mechanical Z:               +"<<kExpectedZ*1000.0<<" mm (not observable in yaw)\n";
    std::cout<<"fit residual median/p95:    "<<median(residual_mm)<<" / "<<percentile(residual_mm,0.95)<<" mm\n";
    std::cout<<"solve condition number:     "<<cond<<"\n";

    std::cout<<"\nChecks:\n";
    std::cout<<"  motion >=45 deg:          "<<(motion_pass?"PASS":"FAIL")<<"\n";
    std::cout<<"  |X| <=10 mm:              "<<(x_pass?"PASS":"FAIL")<<"\n";
    std::cout<<"  |Y| <=10 mm:              "<<(y_pass?"PASS":"FAIL")<<"\n";
    std::cout<<"  XY norm <=12 mm:          "<<(norm_pass?"PASS":"FAIL")<<"\n";
    std::cout<<"  fit p95 <=15 mm:          "<<(fit_pass?"PASS":"FAIL")<<"\n";
    std::cout<<"  condition <=10:           "<<(cond_pass?"PASS":"FAIL")<<"\n";
    const bool pass=motion_pass&&x_pass&&y_pass&&norm_pass&&fit_pass&&cond_pass;
    std::cout<<"\nFINAL RESULT: "<<(pass?"VALIDATED":"NOT VALIDATED")<<"\n";
    return pass?0:2;
  } catch(const std::exception& e) {
    std::cerr<<"ERROR: "<<e.what()<<"\n"; return 1;
  }
}
