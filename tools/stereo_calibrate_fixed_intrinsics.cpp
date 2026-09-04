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
        gray, dictionary,
        marker_corners, marker_ids,
        params);

    Detection out;
    if (marker_ids.empty())
        return out;

    cv::Mat cc, ci;

    const int n = cv::aruco::interpolateCornersCharuco(
        marker_corners, marker_ids,
        gray, board, cc, ci);

    if (n <= 0 || ci.empty())
        return out;

    out.corners.reserve(static_cast<size_t>(n));
    out.ids.reserve(static_cast<size_t>(n));

    for (int i = 0; i < n; ++i) {
        out.corners.push_back(cc.at<cv::Point2f>(i));
        out.ids.push_back(ci.at<int>(i));
    }

    return out;
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

    if (n & 1)
        return v[n / 2];

    return 0.5 * (v[n / 2 - 1] + v[n / 2]);
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

static std::vector<double> rectifiedPairResiduals(
    const std::vector<std::vector<cv::Point2f>> &left_points,
    const std::vector<std::vector<cv::Point2f>> &right_points,
    const cv::Mat &K1,
    const cv::Mat &D1,
    const cv::Mat &K2,
    const cv::Mat &D2,
    const cv::Mat &R1,
    const cv::Mat &R2,
    const cv::Mat &P1,
    const cv::Mat &P2)
{
    std::vector<double> result;
    result.reserve(left_points.size());

    const bool vertical_stereo =
        std::abs(P2.at<double>(1, 3)) >
        std::abs(P2.at<double>(0, 3));

    for (size_t i = 0; i < left_points.size(); ++i) {
        std::vector<cv::Point2f> lr, rr;

        cv::undistortPoints(
            left_points[i], lr,
            K1, D1, R1, P1);

        cv::undistortPoints(
            right_points[i], rr,
            K2, D2, R2, P2);

        double sum = 0.0;

        for (size_t j = 0; j < lr.size(); ++j) {
            sum += vertical_stereo
                ? std::abs(double(lr[j].x) - double(rr[j].x))
                : std::abs(double(lr[j].y) - double(rr[j].y));
        }

        result.push_back(
            lr.empty() ? 1e9 :
            sum / static_cast<double>(lr.size()));
    }

    return result;
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

    int w = 0;
    int h = 0;

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

    fsin.release();

    if (K.empty() || D.empty() || w <= 0 || h <= 0)
        throw std::runtime_error("Invalid intrinsics file: " + path.string());

    size = cv::Size(w, h);
}

int main(int argc, char **argv)
{
    if (argc < 7) {
        std::cerr
            << "Usage:\n"
            << "  " << argv[0]
            << " <capture_dir> <left_intrinsics.yaml> <right_intrinsics.yaml>"
               " <square_mm> <marker_mm> <output_yaml>\n";
        return 1;
    }

    const fs::path capture_dir = argv[1];
    const fs::path left_intrinsics = argv[2];
    const fs::path right_intrinsics = argv[3];
    const double square_mm = std::stod(argv[4]);
    const double marker_mm = std::stod(argv[5]);
    const fs::path output = argv[6];

    cv::Mat K1, D1, K2, D2;
    cv::Size size1, size2;
    std::string source1, source2;

    try {
        loadIntrinsics(
            left_intrinsics, false,
            K1, D1, size1, source1);

        loadIntrinsics(
            right_intrinsics, true,
            K2, D2, size2, source2);
    } catch (const std::exception &e) {
        std::cerr << e.what() << "\n";
        return 1;
    }

    if (size1 != size2) {
        std::cerr << "Intrinsics image sizes differ: left="
                  << size1.width << "x" << size1.height
                  << " right=" << size2.width << "x" << size2.height
                  << "\n";
        return 1;
    }

    const cv::Size image_size = size1;

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

    if (pairs.empty()) {
        std::cerr << "No synchronized stereo pairs found in "
                  << capture_dir << "\n";
        return 1;
    }

    std::vector<std::vector<cv::Point3f>> stereo_obj;
    std::vector<std::vector<cv::Point2f>> stereo_l;
    std::vector<std::vector<cv::Point2f>> stereo_r;
    std::vector<std::string> stems;

    std::cout << "========================================\n";
    std::cout << " JT-Zero fixed-intrinsics stereo calibration\n";
    std::cout << "========================================\n";
    std::cout << "Left intrinsics   : " << left_intrinsics
              << " (" << source1 << ")\n";
    std::cout << "Right intrinsics  : " << right_intrinsics
              << " (" << source2 << ")\n";
    std::cout << "Image size        : "
              << image_size.width << "x" << image_size.height << "\n";
    std::cout << "Pairs on disk     : " << pairs.size() << "\n\n";

    for (const auto &pair : pairs) {
        cv::Mat left =
            cv::imread(pair.left.string(), cv::IMREAD_GRAYSCALE);

        cv::Mat right =
            cv::imread(pair.right.string(), cv::IMREAD_GRAYSCALE);

        if (left.empty() || right.empty() ||
            left.size() != image_size ||
            right.size() != image_size) {
            std::cout << "[SKIP] " << pair.stem
                      << " read/size mismatch\n";
            continue;
        }

        const Detection dl =
            detectCharuco(left, dictionary, board);

        const Detection dr =
            detectCharuco(right, dictionary, board);

        std::vector<cv::Point3f> obj;
        std::vector<cv::Point2f> lp, rp;

        buildShared(
            dl, dr, board_corners,
            obj, lp, rp);

        if (static_cast<int>(obj.size()) < MIN_SHARED_CORNERS) {
            std::cout << "[SKIP] " << pair.stem
                      << " shared=" << obj.size() << "\n";
            continue;
        }

        stereo_obj.push_back(std::move(obj));
        stereo_l.push_back(std::move(lp));
        stereo_r.push_back(std::move(rp));
        stems.push_back(pair.stem);

        std::cout << "[OK]   " << pair.stem
                  << " shared=" << stereo_obj.back().size()
                  << "\n";
    }

    if (stereo_obj.size() < MIN_STEREO_PAIRS) {
        std::cerr << "Need at least " << MIN_STEREO_PAIRS
                  << " stereo pairs, got "
                  << stereo_obj.size() << "\n";
        return 2;
    }

    const size_t initial_pairs = stereo_obj.size();
    std::vector<std::string> rejected_stems;

    cv::Mat R, T, E, F;
    cv::Mat R1, R2, P1, P2, Q;
    cv::Rect roi1, roi2;

    const cv::TermCriteria criteria(
        cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
        100, 1e-9);

    double stereo_rms = 0.0;

    for (int iteration = 0; iteration < 3; ++iteration) {
        stereo_rms = cv::stereoCalibrate(
            stereo_obj,
            stereo_l,
            stereo_r,
            K1, D1,
            K2, D2,
            image_size,
            R, T, E, F,
            cv::CALIB_FIX_INTRINSIC,
            criteria);

        cv::stereoRectify(
            K1, D1,
            K2, D2,
            image_size,
            R, T,
            R1, R2,
            P1, P2,
            Q,
            cv::CALIB_ZERO_DISPARITY,
            0.0,
            image_size,
            &roi1,
            &roi2);

        const std::vector<double> errors =
            rectifiedPairResiduals(
                stereo_l, stereo_r,
                K1, D1, K2, D2,
                R1, R2, P1, P2);

        const double threshold = robustThreshold(errors);

        std::vector<size_t> keep;
        std::vector<size_t> reject;

        for (size_t i = 0; i < errors.size(); ++i) {
            if (errors[i] > threshold)
                reject.push_back(i);
            else
                keep.push_back(i);
        }

        std::cout << "\n[REFINE " << (iteration + 1) << "]"
                  << " pairs=" << stereo_obj.size()
                  << " threshold=" << std::fixed << std::setprecision(3)
                  << threshold << " px"
                  << " reject=" << reject.size() << "\n";

        for (size_t idx : reject) {
            std::cout << "  reject " << stems[idx]
                      << " residual=" << errors[idx] << " px\n";
        }

        if (reject.empty())
            break;

        if (keep.size() < static_cast<size_t>(MIN_STEREO_PAIRS))
            break;

        std::vector<std::vector<cv::Point3f>> new_obj;
        std::vector<std::vector<cv::Point2f>> new_l;
        std::vector<std::vector<cv::Point2f>> new_r;
        std::vector<std::string> new_stems;

        for (size_t idx : reject)
            rejected_stems.push_back(stems[idx]);

        for (size_t idx : keep) {
            new_obj.push_back(stereo_obj[idx]);
            new_l.push_back(stereo_l[idx]);
            new_r.push_back(stereo_r[idx]);
            new_stems.push_back(stems[idx]);
        }

        stereo_obj.swap(new_obj);
        stereo_l.swap(new_l);
        stereo_r.swap(new_r);
        stems.swap(new_stems);
    }

    stereo_rms = cv::stereoCalibrate(
        stereo_obj,
        stereo_l,
        stereo_r,
        K1, D1,
        K2, D2,
        image_size,
        R, T, E, F,
        cv::CALIB_FIX_INTRINSIC,
        criteria);

    cv::stereoRectify(
        K1, D1,
        K2, D2,
        image_size,
        R, T,
        R1, R2,
        P1, P2,
        Q,
        cv::CALIB_ZERO_DISPARITY,
        0.0,
        image_size,
        &roi1,
        &roi2);

    const std::vector<double> final_pair_errors =
        rectifiedPairResiduals(
            stereo_l, stereo_r,
            K1, D1, K2, D2,
            R1, R2, P1, P2);

    std::vector<double> all_residuals;

    const bool vertical_stereo =
        std::abs(P2.at<double>(1, 3)) >
        std::abs(P2.at<double>(0, 3));

    for (size_t i = 0; i < stereo_l.size(); ++i) {
        std::vector<cv::Point2f> lr, rr;

        cv::undistortPoints(
            stereo_l[i], lr,
            K1, D1, R1, P1);

        cv::undistortPoints(
            stereo_r[i], rr,
            K2, D2, R2, P2);

        for (size_t j = 0; j < lr.size(); ++j) {
            all_residuals.push_back(
                vertical_stereo
                    ? std::abs(double(lr[j].x) - double(rr[j].x))
                    : std::abs(double(lr[j].y) - double(rr[j].y)));
        }
    }

    const double mean_residual =
        std::accumulate(
            all_residuals.begin(),
            all_residuals.end(),
            0.0) /
        static_cast<double>(all_residuals.size());

    const double median_residual =
        medianOf(all_residuals);

    std::sort(all_residuals.begin(), all_residuals.end());

    const size_t p95_idx =
        static_cast<size_t>(
            std::floor(0.95 * (all_residuals.size() - 1)));

    const double p95_residual = all_residuals[p95_idx];
    const double max_residual = all_residuals.back();

    if (!output.parent_path().empty())
        fs::create_directories(output.parent_path());

    cv::FileStorage out(output.string(), cv::FileStorage::WRITE);

    out << "camera_left" << "OV9281_USB";
    out << "camera_right" << "OV5647_CSI";
    out << "image_width" << image_size.width;
    out << "image_height" << image_size.height;

    out << "left_intrinsics_file" << left_intrinsics.string();
    out << "right_intrinsics_file" << right_intrinsics.string();
    out << "right_intrinsics_source" << source2;

    out << "K1" << K1;
    out << "D1" << D1;
    out << "K2" << K2;
    out << "D2" << D2;

    out << "R_ov9281_to_ov5647" << R;
    out << "T_ov9281_to_ov5647_m" << T;
    out << "E" << E;
    out << "F" << F;

    out << "R1" << R1;
    out << "R2" << R2;
    out << "P1" << P1;
    out << "P2" << P2;
    out << "Q" << Q;

    out << "baseline_m" << cv::norm(T);
    out << "stereo_rms_px" << stereo_rms;
    out << "rectified_mean_residual_px" << mean_residual;
    out << "rectified_median_residual_px" << median_residual;
    out << "rectified_p95_residual_px" << p95_residual;
    out << "rectified_max_residual_px" << max_residual;
    out << "stereo_pairs_initial" << static_cast<int>(initial_pairs);
    out << "stereo_pairs_final" << static_cast<int>(stereo_obj.size());

    out << "rejected_pair_stems" << "[";
    for (const auto &s : rejected_stems)
        out << s;
    out << "]";

    out.release();

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "\n========================================\n";
    std::cout << " RESULT\n";
    std::cout << "========================================\n";
    std::cout << "Stereo pairs init : " << initial_pairs << "\n";
    std::cout << "Stereo pairs final: " << stereo_obj.size() << "\n";
    std::cout << "Rejected pairs    : " << rejected_stems.size() << "\n";
    std::cout << "Stereo RMS        : " << stereo_rms << " px\n";
    std::cout << "Rect mean residual: " << mean_residual << " px\n";
    std::cout << "Rect median resid : " << median_residual << " px\n";
    std::cout << "Rect p95 residual : " << p95_residual << " px\n";
    std::cout << "Rect max residual : " << max_residual << " px\n";
    std::cout << "Baseline          : " << cv::norm(T) * 1000.0 << " mm\n";
    std::cout << "Stereo type       : "
              << (vertical_stereo ? "VERTICAL" : "HORIZONTAL") << "\n";
    std::cout << "Output             : " << output << "\n";
    std::cout << "========================================\n";

    return 0;
}
