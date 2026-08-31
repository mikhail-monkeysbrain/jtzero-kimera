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
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr int kDetectionStride = 4;          // 120 FPS -> ~30 FPS pose stream
constexpr int kMinCharucoCorners = 12;
constexpr double kMaxReprojectionRmsePx = 1.5;
constexpr int64_t kMaxRawCameraDtNs = 20'000'000LL;
constexpr double kPi = 3.14159265358979323846;

struct CameraFrame {
    size_t raw_index = 0;
    uint32_t sequence = 0;
    int64_t timestamp_ns = 0;
    uint64_t mjpeg_offset = 0;
    uint32_t bytes_used = 0;
    bool continuous_from_prev = false;
};

struct PoseSample {
    size_t raw_index = 0;
    uint32_t sequence = 0;
    int64_t timestamp_ns = 0;
    Eigen::Matrix3d R_CW = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_CW = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R_WC = Eigen::Matrix3d::Identity();
    Eigen::Vector3d p_WC = Eigen::Vector3d::Zero();
    double reprojection_rmse_px = 0.0;
    int charuco_corners = 0;
};

std::vector<std::string> splitCsv(const std::string& line) {
    std::vector<std::string> out;
    std::string field;
    std::stringstream ss(line);
    while (std::getline(ss, field, ',')) out.push_back(field);
    if (!line.empty() && line.back() == ',') out.emplace_back();
    return out;
}

double median(std::vector<double> v) {
    if (v.empty()) return std::numeric_limits<double>::quiet_NaN();
    std::sort(v.begin(), v.end());
    const size_t n = v.size();
    return (n & 1U) ? v[n/2] : 0.5 * (v[n/2-1] + v[n/2]);
}

double percentile(std::vector<double> v, double q) {
    if (v.empty()) return std::numeric_limits<double>::quiet_NaN();
    std::sort(v.begin(), v.end());
    const double p = q * static_cast<double>(v.size()-1);
    const size_t lo = static_cast<size_t>(std::floor(p));
    const size_t hi = std::min(lo+1, v.size()-1);
    const double f = p-static_cast<double>(lo);
    return v[lo]*(1.0-f)+v[hi]*f;
}

std::vector<CameraFrame> loadCameraIndex(const std::string& path, size_t* discontinuities) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("Cannot open camera index: " + path);
    std::string line;
    if (!std::getline(in,line)) throw std::runtime_error("Empty camera index");
    std::vector<CameraFrame> frames;
    bool have_prev=false;
    uint32_t prev_seq=0;
    int64_t prev_ts=0;
    *discontinuities=0;
    while (std::getline(in,line)) {
        if (line.empty()) continue;
        const auto c=splitCsv(line);
        if (c.size()<7) continue;
        CameraFrame f;
        f.raw_index=frames.size();
        f.sequence=static_cast<uint32_t>(std::stoul(c[0]));
        f.timestamp_ns=std::stoll(c[2]);
        f.mjpeg_offset=std::stoull(c[5]);
        f.bytes_used=static_cast<uint32_t>(std::stoul(c[6]));
        if (have_prev) {
            const int64_t dt=f.timestamp_ns-prev_ts;
            f.continuous_from_prev=(f.sequence==prev_seq+1U && dt>0 && dt<=kMaxRawCameraDtNs);
            if (!f.continuous_from_prev) ++(*discontinuities);
        }
        prev_seq=f.sequence;
        prev_ts=f.timestamp_ns;
        have_prev=true;
        frames.push_back(f);
    }
    return frames;
}

bool readMjpegFrame(std::ifstream& mjpeg,const CameraFrame& f,cv::Mat* image) {
    if (!image || f.bytes_used==0) return false;
    std::vector<unsigned char> bytes(f.bytes_used);
    mjpeg.clear();
    mjpeg.seekg(static_cast<std::streamoff>(f.mjpeg_offset),std::ios::beg);
    if (!mjpeg) return false;
    mjpeg.read(reinterpret_cast<char*>(bytes.data()),static_cast<std::streamsize>(bytes.size()));
    if (mjpeg.gcount()!=static_cast<std::streamsize>(bytes.size())) return false;
    *image=cv::imdecode(bytes,cv::IMREAD_GRAYSCALE);
    return !image->empty();
}

Eigen::Matrix3d cvMat33ToEigen(const cv::Mat& R) {
    Eigen::Matrix3d out;
    for(int r=0;r<3;++r) for(int c=0;c<3;++c) out(r,c)=R.at<double>(r,c);
    return out;
}

Eigen::Vector3d cvVec3ToEigen(const cv::Mat& v) {
    return Eigen::Vector3d(v.at<double>(0),v.at<double>(1),v.at<double>(2));
}

double reprojectionRmse(const std::vector<cv::Point3f>& obj,const std::vector<cv::Point2f>& img,
                        const cv::Mat& rvec,const cv::Mat& tvec,const cv::Mat& K,const cv::Mat& D) {
    std::vector<cv::Point2f> projected;
    cv::projectPoints(obj,rvec,tvec,K,D,projected);
    if(projected.empty()) return 1e9;
    double s=0.0;
    for(size_t i=0;i<projected.size();++i){
        const cv::Point2f d=projected[i]-img[i];
        s+=static_cast<double>(d.x*d.x+d.y*d.y);
    }
    return std::sqrt(s/projected.size());
}

bool rawIntervalContinuous(const std::vector<CameraFrame>& frames,size_t a,size_t b) {
    if(b<=a || b>=frames.size()) return false;
    for(size_t i=a+1;i<=b;++i) if(!frames[i].continuous_from_prev) return false;
    return true;
}

Eigen::Matrix3d rotZ(double a) {
    const double c=std::cos(a),s=std::sin(a);
    Eigen::Matrix3d R;
    R<<c,-s,0,s,c,0,0,0,1;
    return R;
}

} // namespace

int main(int argc,char** argv) {
    try {
        const std::string camera_csv=argc>1?argv[1]:"/home/vio/camera_imu_translation_yaw_40s_camera.csv";
        const std::string mjpeg_path=argc>2?argv[2]:"/home/vio/camera_imu_translation_yaw_40s.mjpg";
        const std::string intrinsics_path=argc>3?argv[3]:"calibration/ov9281_intrinsics.yaml";

        cv::FileStorage fs(intrinsics_path,cv::FileStorage::READ);
        if(!fs.isOpened()) throw std::runtime_error("Cannot open intrinsics YAML");
        int squares_x=0,squares_y=0;
        double square_mm=0.0,marker_mm=0.0;
        cv::Mat K,D;
        fs["board_squares_x"]>>squares_x;
        fs["board_squares_y"]>>squares_y;
        fs["board_square_length_mm"]>>square_mm;
        fs["board_marker_length_mm"]>>marker_mm;
        fs["camera_matrix"]>>K;
        fs["distortion_coefficients"]>>D;
        if(squares_x<=1 || squares_y<=1 || square_mm<=0 || marker_mm<=0 || K.empty() || D.empty())
            throw std::runtime_error("Invalid intrinsics/board parameters");

        size_t raw_discontinuities=0;
        const auto frames=loadCameraIndex(camera_csv,&raw_discontinuities);
        std::ifstream mjpeg(mjpeg_path,std::ios::binary);
        if(!mjpeg) throw std::runtime_error("Cannot open MJPEG: "+mjpeg_path);

        auto dict=cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        cv::aruco::CharucoBoard board(cv::Size(squares_x,squares_y),
                                     static_cast<float>(square_mm*1e-3),
                                     static_cast<float>(marker_mm*1e-3),dict);
        cv::aruco::CharucoDetector detector(board);

        std::vector<PoseSample> poses;
        size_t attempts=0,decode_fail=0,detect_fail=0,reproj_reject=0,continuity_reject=0;
        for(size_t i=0;i<frames.size();i+=kDetectionStride) {
            ++attempts;
            if(i>0 && !rawIntervalContinuous(frames,i>=kDetectionStride?i-kDetectionStride:0,i)) {
                ++continuity_reject;
                continue;
            }
            cv::Mat image;
            if(!readMjpegFrame(mjpeg,frames[i],&image)){++decode_fail;continue;}
            cv::Mat charuco_corners,charuco_ids;
            detector.detectBoard(image,charuco_corners,charuco_ids);
            if(charuco_ids.total()<static_cast<size_t>(kMinCharucoCorners)){++detect_fail;continue;}

            std::vector<cv::Point3f> obj;
            std::vector<cv::Point2f> img;
            board.matchImagePoints(charuco_corners,charuco_ids,obj,img);
            if(obj.size()<static_cast<size_t>(kMinCharucoCorners)){++detect_fail;continue;}
            cv::Mat rvec,tvec;
            if(!cv::solvePnP(obj,img,K,D,rvec,tvec,false,cv::SOLVEPNP_ITERATIVE)){++detect_fail;continue;}
            const double rmse=reprojectionRmse(obj,img,rvec,tvec,K,D);
            if(!std::isfinite(rmse) || rmse>kMaxReprojectionRmsePx){++reproj_reject;continue;}
            cv::Mat Rcv;
            cv::Rodrigues(rvec,Rcv);
            PoseSample p;
            p.raw_index=i;p.sequence=frames[i].sequence;p.timestamp_ns=frames[i].timestamp_ns;
            p.R_CW=cvMat33ToEigen(Rcv);p.t_CW=cvVec3ToEigen(tvec);
            p.R_WC=p.R_CW.transpose();
            p.p_WC=-p.R_WC*p.t_CW;
            p.reprojection_rmse_px=rmse;p.charuco_corners=static_cast<int>(obj.size());
            poses.push_back(p);
        }
        if(poses.size()<20) throw std::runtime_error("Too few valid ChArUco poses");

        // For pure body yaw about a fixed body origin F, camera position is
        // p_WC = p_WF + R_WB * t_BC.  With yaw-dominant motion and t_BC=[x,y,z],
        // horizontal p_WC therefore traces a circle of radius sqrt(x^2+y^2).
        // Fit a 3-D circle plane first by PCA, then a 2-D algebraic circle in that plane.
        Eigen::Vector3d centroid=Eigen::Vector3d::Zero();
        for(const auto& p:poses) centroid+=p.p_WC;
        centroid/=static_cast<double>(poses.size());
        Eigen::Matrix3d cov=Eigen::Matrix3d::Zero();
        for(const auto& p:poses){const Eigen::Vector3d d=p.p_WC-centroid;cov+=d*d.transpose();}
        cov/=static_cast<double>(poses.size());
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
        if(es.info()!=Eigen::Success) throw std::runtime_error("PCA failed");
        const Eigen::Vector3d normal=es.eigenvectors().col(0).normalized();
        const Eigen::Vector3d u=es.eigenvectors().col(2).normalized();
        const Eigen::Vector3d v=normal.cross(u).normalized();

        const int n=static_cast<int>(poses.size());
        Eigen::MatrixXd A(n,3);
        Eigen::VectorXd b(n);
        std::vector<Eigen::Vector2d> q;
        q.reserve(poses.size());
        std::vector<double> plane_abs;
        for(int i=0;i<n;++i){
            const Eigen::Vector3d d=poses[i].p_WC-centroid;
            const double x=d.dot(u),y=d.dot(v);
            q.emplace_back(x,y);
            A(i,0)=2*x;A(i,1)=2*y;A(i,2)=1.0;
            b(i)=x*x+y*y;
            plane_abs.push_back(std::abs(d.dot(normal)));
        }
        const Eigen::Vector3d sol=A.colPivHouseholderQr().solve(b);
        const Eigen::Vector2d center2(sol(0),sol(1));
        const double radius2=sol(2)+center2.squaredNorm();
        if(radius2<=0) throw std::runtime_error("Invalid circle fit");
        const double radius=std::sqrt(radius2);
        std::vector<double> radial_abs;
        for(const auto& z:q) radial_abs.push_back(std::abs((z-center2).norm()-radius));

        std::vector<double> reproj;
        for(const auto& p:poses) reproj.push_back(p.reprojection_rmse_px);

        // Rotation span from camera orientations. For a yaw test this is a useful
        // sanity metric; it is not used to derive the circle radius.
        const Eigen::Matrix3d R0=poses.front().R_WC;
        std::vector<double> orientation_deg;
        for(const auto& p:poses){
            const Eigen::Matrix3d dR=R0.transpose()*p.R_WC;
            const double c=std::clamp((dR.trace()-1.0)*0.5,-1.0,1.0);
            orientation_deg.push_back(std::acos(c)*180.0/kPi);
        }

        const double expected_xy=0.074; // measured FC datum -> camera, Y ~= 0
        const double radius_error=radius-expected_xy;
        const bool enough_motion=percentile(orientation_deg,0.95)>=45.0;
        const bool radius_pass=std::abs(radius_error)<=0.015;
        const bool fit_pass=percentile(radial_abs,0.95)<=0.015;

        std::cout<<std::fixed<<std::setprecision(6);
        std::cout<<"============================================================\n";
        std::cout<<"CAMERA-IMU TRANSLATION / YAW VALIDATION\n";
        std::cout<<"============================================================\n";
        std::cout<<"raw camera frames:          "<<frames.size()<<"\n";
        std::cout<<"raw discontinuities:        "<<raw_discontinuities<<"\n";
        std::cout<<"pose attempts:              "<<attempts<<"\n";
        std::cout<<"valid ChArUco poses:        "<<poses.size()<<"\n";
        std::cout<<"decode failures:            "<<decode_fail<<"\n";
        std::cout<<"detection failures:         "<<detect_fail<<"\n";
        std::cout<<"reprojection rejects:       "<<reproj_reject<<"\n";
        std::cout<<"continuity rejects:         "<<continuity_reject<<"\n";
        std::cout<<"reprojection median/p95:    "<<median(reproj)<<" / "<<percentile(reproj,0.95)<<" px\n";
        std::cout<<"orientation change p95:     "<<percentile(orientation_deg,0.95)<<" deg\n";
        std::cout<<"circle radius:              "<<radius*1000.0<<" mm\n";
        std::cout<<"expected XY lever arm:      "<<expected_xy*1000.0<<" mm\n";
        std::cout<<"radius error:               "<<radius_error*1000.0<<" mm\n";
        std::cout<<"radial residual med/p95:    "<<median(radial_abs)*1000.0<<" / "<<percentile(radial_abs,0.95)*1000.0<<" mm\n";
        std::cout<<"plane residual med/p95:     "<<median(plane_abs)*1000.0<<" / "<<percentile(plane_abs,0.95)*1000.0<<" mm\n";
        std::cout<<"PCA eigenvalues:            "<<es.eigenvalues().transpose()<<"\n";
        std::cout<<"\nChecks:\n";
        std::cout<<"  motion >=45 deg:          "<<(enough_motion?"PASS":"FAIL")<<"\n";
        std::cout<<"  radius 74 +/-15 mm:       "<<(radius_pass?"PASS":"FAIL")<<"\n";
        std::cout<<"  radial p95 <=15 mm:       "<<(fit_pass?"PASS":"FAIL")<<"\n";
        std::cout<<"\nFINAL RESULT: "<<((enough_motion&&radius_pass&&fit_pass)?"PASS":"NOT VALIDATED")<<"\n";
        return 0;
    } catch(const std::exception& e) {
        std::cerr<<"ERROR: "<<e.what()<<"\n";
        return 1;
    }
}
