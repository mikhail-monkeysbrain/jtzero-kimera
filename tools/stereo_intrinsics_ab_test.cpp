#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <string>
#include <vector>

namespace fs = std::filesystem;

static constexpr int BOARD_SQUARES_X = 7;
static constexpr int BOARD_SQUARES_Y = 5;
static constexpr int MIN_MONO_CORNERS = 6;
static constexpr int MIN_SHARED_CORNERS = 6;
static constexpr int MIN_STEREO_PAIRS = 12;

struct PairFiles {
    fs::path left;
    fs::path right;
    std::string stem;
};

struct Detection {
    std::vector<cv::Point2f> corners;
    std::vector<int> ids;
};

struct StereoResult {
    std::string name;
    size_t initial_pairs{0};
    size_t final_pairs{0};
    size_t rejected_pairs{0};
    double stereo_rms{0.0};
    double mean_residual{0.0};
    double median_residual{0.0};
    double p95_residual{0.0};
    double max_residual{0.0};
    double baseline_mm{0.0};
    bool vertical{false};
    cv::Mat R, T, R1, R2, P1, P2, Q;
};

static std::vector<PairFiles> findPairs(const fs::path &dir)
{
    std::vector<PairFiles> pairs;
    if (!fs::exists(dir))
        return pairs;

    const std::string suffix = "_ov9281.png";

    for (const auto &entry : fs::directory_iterator(dir)) {
        if (!entry.is_regular_file())
            continue;

        const std::string name = entry.path().filename().string();
        if (name.size() <= suffix.size() ||
            name.substr(name.size() - suffix.size()) != suffix)
            continue;

        const std::string stem =
            name.substr(0, name.size() - suffix.size());

        const fs::path right = dir / (stem + "_ov5647.png");
        if (fs::exists(right))
            pairs.push_back({entry.path(), right, stem});
    }

    std::sort(pairs.begin(), pairs.end(),
              [](const PairFiles &a, const PairFiles &b) {
                  return a.stem < b.stem;
              });

    return pairs;
}

static Detection detectCharuco(
    const cv::Mat &gray,
    const cv::Ptr<cv::aruco::Dictionary> &dictionary,
    const cv::Ptr<cv::aruco::CharucoBoard> &board)
{
    std::vector<std::vector<cv::Point2f>> marker_corners;
    std::vector<int> marker_ids;
    auto params = cv::makePtr<cv::aruco::DetectorParameters>();

    cv::aruco::detectMarkers(
        gray, dictionary, marker_corners, marker_ids, params);

    Detection out;
    if (marker_ids.empty())
        return out;

    cv::Mat cc, ci;
    const int n = cv::aruco::interpolateCornersCharuco(
        marker_corners, marker_ids, gray, board, cc, ci);

    if (n <= 0 || ci.empty())
        return out;

    for (int i = 0; i < n; ++i) {
        out.corners.push_back(cc.at<cv::Point2f>(i));
        out.ids.push_back(ci.at<int>(i));
    }

    return out;
}

static void buildMono(
    const Detection &d,
    const std::vector<cv::Point3f> &board_corners,
    std::vector<cv::Point3f> &obj,
    std::vector<cv::Point2f> &img)
{
    for (size_t i = 0; i < d.ids.size(); ++i) {
        const int id = d.ids[i];
        if (id < 0 || static_cast<size_t>(id) >= board_corners.size())
            continue;
        obj.push_back(board_corners[static_cast<size_t>(id)]);
        img.push_back(d.corners[i]);
    }
}

static void buildShared(
    const Detection &left,
    const Detection &right,
    const std::vector<cv::Point3f> &board_corners,
    std::vector<cv::Point3f> &obj,
    std::vector<cv::Point2f> &lp,
    std::vector<cv::Point2f> &rp)
{
    std::map<int, cv::Point2f> lmap;
    std::map<int, cv::Point2f> rmap;

    for (size_t i = 0; i < left.ids.size(); ++i)
        lmap[left.ids[i]] = left.corners[i];
    for (size_t i = 0; i < right.ids.size(); ++i)
        rmap[right.ids[i]] = right.corners[i];

    for (const auto &[id, p] : lmap) {
        const auto it = rmap.find(id);
        if (it == rmap.end())
            continue;
        if (id < 0 || static_cast<size_t>(id) >= board_corners.size())
            continue;

        obj.push_back(board_corners[static_cast<size_t>(id)]);
        lp.push_back(p);
        rp.push_back(it->second);
    }
}

static double medianOf(std::vector<double> v)
{
    if (v.empty())
        return 0.0;
    std::sort(v.begin(), v.end());
    const size_t n = v.size();
    return (n & 1) ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

static double robustThreshold(const std::vector<double> &errors)
{
    if (errors.empty())
        return 2.5;

    const double med = medianOf(errors);
    std::vector<double> dev;
    dev.reserve(errors.size());

    for (double e : errors)
        dev.push_back(std::abs(e - med));

    const double mad = medianOf(dev);
    return std::max(2.5, med + 2.5 * 1.4826 * mad);
}

static void loadIntrinsics(
    const fs::path &path,
    bool prefer_fix_k3,
    cv::Mat &K,
    cv::Mat &D,
    cv::Size &size,
    std::string &source)
{
    cv::FileStorage fsin(path.string(), cv::FileStorage::READ);
    if (!fsin.isOpened())
        throw std::runtime_error("Cannot open intrinsics: " + path.string());

    int w = 0, h = 0;
    fsin["image_width"] >> w;
    fsin["image_height"] >> h;

    if (prefer_fix_k3 &&
        !fsin["fix_k3_camera_matrix"].empty() &&
        !fsin["fix_k3_distortion_coefficients"].empty()) {
        fsin["fix_k3_camera_matrix"] >> K;
        fsin["fix_k3_distortion_coefficients"] >> D;
        source = "FIX_K3";
    } else {
        fsin["camera_matrix"] >> K;
        fsin["distortion_coefficients"] >> D;
        source = "PRIMARY";
    }

    if (K.empty() || D.empty() || w <= 0 || h <= 0)
        throw std::runtime_error("Invalid intrinsics: " + path.string());

    size = cv::Size(w, h);
}

static std::vector<double> pairResiduals(
    const std::vector<std::vector<cv::Point2f>> &left_points,
    const std::vector<std::vector<cv::Point2f>> &right_points,
    const cv::Mat &K1, const cv::Mat &D1,
    const cv::Mat &K2, const cv::Mat &D2,
    const cv::Mat &R1, const cv::Mat &R2,
    const cv::Mat &P1, const cv::Mat &P2)
{
    const bool vertical =
        std::abs(P2.at<double>(1, 3)) >
        std::abs(P2.at<double>(0, 3));

    std::vector<double> result;
    result.reserve(left_points.size());

    for (size_t i = 0; i < left_points.size(); ++i) {
        std::vector<cv::Point2f> lr, rr;

        cv::undistortPoints(left_points[i], lr, K1, D1, R1, P1);
        cv::undistortPoints(right_points[i], rr, K2, D2, R2, P2);

        if (lr.empty() || lr.size() != rr.size()) {
            result.push_back(1e9);
            continue;
        }

        double sum = 0.0;
        for (size_t j = 0; j < lr.size(); ++j) {
            sum += vertical
                ? std::abs(double(lr[j].x) - double(rr[j].x))
                : std::abs(double(lr[j].y) - double(rr[j].y));
        }

        result.push_back(sum / static_cast<double>(lr.size()));
    }

    return result;
}

static StereoResult runStereo(
    const std::string &name,
    const std::vector<std::vector<cv::Point3f>> &obj_in,
    const std::vector<std::vector<cv::Point2f>> &left_in,
    const std::vector<std::vector<cv::Point2f>> &right_in,
    const std::vector<std::string> &stems_in,
    const cv::Mat &K1_in, const cv::Mat &D1_in,
    const cv::Mat &K2_in, const cv::Mat &D2_in,
    const cv::Size &image_size)
{
    auto obj = obj_in;
    auto left = left_in;
    auto right = right_in;
    auto stems = stems_in;

    const cv::Mat K1 = K1_in.clone();
    const cv::Mat D1 = D1_in.clone();
    const cv::Mat K2 = K2_in.clone();
    const cv::Mat D2 = D2_in.clone();

    StereoResult result;
    result.name = name;
    result.initial_pairs = obj.size();

    cv::Mat E, F;
    cv::Rect roi1, roi2;

    const cv::TermCriteria criteria(
        cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100, 1e-9);

    for (int iteration = 0; iteration < 3; ++iteration) {
        result.stereo_rms = cv::stereoCalibrate(
            obj, left, right,
            K1, D1, K2, D2, image_size,
            result.R, result.T, E, F,
            cv::CALIB_FIX_INTRINSIC,
            criteria);

        cv::stereoRectify(
            K1, D1, K2, D2, image_size,
            result.R, result.T,
            result.R1, result.R2,
            result.P1, result.P2, result.Q,
            cv::CALIB_ZERO_DISPARITY, 0.0,
            image_size, &roi1, &roi2);

        const auto errors = pairResiduals(
            left, right, K1, D1, K2, D2,
            result.R1, result.R2, result.P1, result.P2);

        const double threshold = robustThreshold(errors);

        std::vector<size_t> keep, reject;
        for (size_t i = 0; i < errors.size(); ++i) {
            if (errors[i] > threshold)
                reject.push_back(i);
            else
                keep.push_back(i);
        }

        std::cout << "[" << name << " REFINE " << (iteration + 1)
                  << "] pairs=" << obj.size()
                  << " threshold=" << std::fixed << std::setprecision(3)
                  << threshold << " reject=" << reject.size() << "\n";

        if (reject.empty() ||
            keep.size() < static_cast<size_t>(MIN_STEREO_PAIRS))
            break;

        std::vector<std::vector<cv::Point3f>> nobj;
        std::vector<std::vector<cv::Point2f>> nl, nr;
        std::vector<std::string> nstems;

        for (size_t idx : keep) {
            nobj.push_back(obj[idx]);
            nl.push_back(left[idx]);
            nr.push_back(right[idx]);
            nstems.push_back(stems[idx]);
        }

        result.rejected_pairs += reject.size();
        obj.swap(nobj);
        left.swap(nl);
        right.swap(nr);
        stems.swap(nstems);
    }

    result.stereo_rms = cv::stereoCalibrate(
        obj, left, right,
        K1, D1, K2, D2, image_size,
        result.R, result.T, E, F,
        cv::CALIB_FIX_INTRINSIC,
        criteria);

    cv::stereoRectify(
        K1, D1, K2, D2, image_size,
        result.R, result.T,
        result.R1, result.R2,
        result.P1, result.P2, result.Q,
        cv::CALIB_ZERO_DISPARITY, 0.0,
        image_size, &roi1, &roi2);

    result.final_pairs = obj.size();
    result.baseline_mm = cv::norm(result.T) * 1000.0;
    result.vertical =
        std::abs(result.P2.at<double>(1, 3)) >
        std::abs(result.P2.at<double>(0, 3));

    std::vector<double> all;

    for (size_t i = 0; i < left.size(); ++i) {
        std::vector<cv::Point2f> lr, rr;
        cv::undistortPoints(left[i], lr, K1, D1, result.R1, result.P1);
        cv::undistortPoints(right[i], rr, K2, D2, result.R2, result.P2);

        for (size_t j = 0; j < lr.size(); ++j) {
            all.push_back(
                result.vertical
                    ? std::abs(double(lr[j].x) - double(rr[j].x))
                    : std::abs(double(lr[j].y) - double(rr[j].y)));
        }
    }

    result.mean_residual =
        std::accumulate(all.begin(), all.end(), 0.0) /
        static_cast<double>(all.size());

    result.median_residual = medianOf(all);

    std::sort(all.begin(), all.end());
    result.p95_residual =
        all[static_cast<size_t>(std::floor(0.95 * (all.size() - 1)))];
    result.max_residual = all.back();

    return result;
}

int main(int argc, char **argv)
{
    if (argc < 6) {
        std::cerr
            << "Usage:\n  " << argv[0]
            << " <stereo_capture_dir> <ov9281_intrinsics.yaml>"
               " <ov5647_intrinsics.yaml> <square_mm> <marker_mm>\n";
        return 1;
    }

    const fs::path capture_dir = argv[1];
    const fs::path left_yaml = argv[2];
    const fs::path right_yaml = argv[3];
    const double square_mm = std::stod(argv[4]);
    const double marker_mm = std::stod(argv[5]);

    cv::Mat K1_fixed, D1_fixed, K2_fixed, D2_fixed;
    cv::Size size1, size2;
    std::string src1, src2;

    try {
        loadIntrinsics(left_yaml, false, K1_fixed, D1_fixed, size1, src1);
        loadIntrinsics(right_yaml, true, K2_fixed, D2_fixed, size2, src2);
    } catch (const std::exception &e) {
        std::cerr << e.what() << "\n";
        return 1;
    }

    if (size1 != size2) {
        std::cerr << "Image size mismatch between intrinsics\n";
        return 1;
    }

    const cv::aruco::Dictionary dictionary_value =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

    const auto dictionary =
        cv::makePtr<cv::aruco::Dictionary>(dictionary_value);

    const auto board =
        cv::makePtr<cv::aruco::CharucoBoard>(
            cv::Size(BOARD_SQUARES_X, BOARD_SQUARES_Y),
            static_cast<float>(square_mm / 1000.0),
            static_cast<float>(marker_mm / 1000.0),
            dictionary_value);

    const auto board_corners = board->getChessboardCorners();
    const auto pairs = findPairs(capture_dir);

    std::vector<std::vector<cv::Point3f>> mono_obj_l, mono_obj_r;
    std::vector<std::vector<cv::Point2f>> mono_img_l, mono_img_r;

    std::vector<std::vector<cv::Point3f>> stereo_obj;
    std::vector<std::vector<cv::Point2f>> stereo_l, stereo_r;
    std::vector<std::string> stereo_stems;

    for (const auto &pair : pairs) {
        cv::Mat left = cv::imread(pair.left.string(), cv::IMREAD_GRAYSCALE);
        cv::Mat right = cv::imread(pair.right.string(), cv::IMREAD_GRAYSCALE);

        if (left.empty() || right.empty() ||
            left.size() != size1 || right.size() != size1)
            continue;

        const Detection dl = detectCharuco(left, dictionary, board);
        const Detection dr = detectCharuco(right, dictionary, board);

        std::vector<cv::Point3f> ol, orr, os;
        std::vector<cv::Point2f> il, ir, sl, sr;

        buildMono(dl, board_corners, ol, il);
        buildMono(dr, board_corners, orr, ir);
        buildShared(dl, dr, board_corners, os, sl, sr);

        if (static_cast<int>(ol.size()) >= MIN_MONO_CORNERS) {
            mono_obj_l.push_back(std::move(ol));
            mono_img_l.push_back(std::move(il));
        }

        if (static_cast<int>(orr.size()) >= MIN_MONO_CORNERS) {
            mono_obj_r.push_back(std::move(orr));
            mono_img_r.push_back(std::move(ir));
        }

        if (static_cast<int>(os.size()) >= MIN_SHARED_CORNERS) {
            stereo_obj.push_back(std::move(os));
            stereo_l.push_back(std::move(sl));
            stereo_r.push_back(std::move(sr));
            stereo_stems.push_back(pair.stem);
        }
    }

    if (mono_obj_l.size() < 10 ||
        mono_obj_r.size() < 10 ||
        stereo_obj.size() < MIN_STEREO_PAIRS) {
        std::cerr << "Insufficient usable calibration data\n";
        return 2;
    }

    cv::Mat K1_local = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat D1_local = cv::Mat::zeros(5, 1, CV_64F);
    cv::Mat K2_local = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat D2_local = cv::Mat::zeros(5, 1, CV_64F);
    std::vector<cv::Mat> rvecs, tvecs;

    const cv::TermCriteria criteria(
        cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100, 1e-9);

    const double left_local_rms = cv::calibrateCamera(
        mono_obj_l, mono_img_l, size1,
        K1_local, D1_local, rvecs, tvecs, 0, criteria);

    rvecs.clear();
    tvecs.clear();

    const double right_local_rms = cv::calibrateCamera(
        mono_obj_r, mono_img_r, size1,
        K2_local, D2_local, rvecs, tvecs,
        cv::CALIB_FIX_K3, criteria);

    std::cout << "========================================\n";
    std::cout << " JT-Zero stereo intrinsics A/B test\n";
    std::cout << "========================================\n";
    std::cout << "Stereo pairs      : " << stereo_obj.size() << "\n";
    std::cout << "Local left RMS    : " << left_local_rms << " px\n";
    std::cout << "Local right RMS   : " << right_local_rms << " px\n";
    std::cout << "Fixed left source : " << src1 << "\n";
    std::cout << "Fixed right source: " << src2 << "\n\n";

    const auto A = runStereo(
        "A_LOCAL_LOCAL",
        stereo_obj, stereo_l, stereo_r, stereo_stems,
        K1_local, D1_local, K2_local, D2_local, size1);

    const auto B = runStereo(
        "B_FIXED_LEFT",
        stereo_obj, stereo_l, stereo_r, stereo_stems,
        K1_fixed, D1_fixed, K2_local, D2_local, size1);

    const auto C = runStereo(
        "C_FIXED_RIGHT",
        stereo_obj, stereo_l, stereo_r, stereo_stems,
        K1_local, D1_local, K2_fixed, D2_fixed, size1);

    const auto D = runStereo(
        "D_BOTH_FIXED",
        stereo_obj, stereo_l, stereo_r, stereo_stems,
        K1_fixed, D1_fixed, K2_fixed, D2_fixed, size1);

    const std::vector<StereoResult> results{A, B, C, D};

    std::cout << "\n========================================\n";
    std::cout << " COMPARISON\n";
    std::cout << "========================================\n";

    for (const auto &r : results) {
        std::cout << std::fixed << std::setprecision(3);
        std::cout << r.name
                  << " pairs=" << r.final_pairs
                  << " rms=" << r.stereo_rms
                  << " mean=" << r.mean_residual
                  << " median=" << r.median_residual
                  << " p95=" << r.p95_residual
                  << " max=" << r.max_residual
                  << " baseline=" << r.baseline_mm << "mm"
                  << " type=" << (r.vertical ? "VERTICAL" : "HORIZONTAL")
                  << "\n";
    }

    std::cout << "========================================\n";

    return 0;
}
