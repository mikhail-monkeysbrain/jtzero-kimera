#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <map>
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

static double meanReprojectionError(
    const std::vector<std::vector<cv::Point3f>> &object_points,
    const std::vector<std::vector<cv::Point2f>> &image_points,
    const std::vector<cv::Mat> &rvecs,
    const std::vector<cv::Mat> &tvecs,
    const cv::Mat &K,
    const cv::Mat &D)
{
    double sum = 0.0;
    size_t count = 0;

    for (size_t i = 0; i < object_points.size(); ++i) {
        std::vector<cv::Point2f> projected;
        cv::projectPoints(
            object_points[i], rvecs[i], tvecs[i],
            K, D, projected);

        for (size_t j = 0; j < projected.size(); ++j) {
            sum += cv::norm(projected[j] - image_points[i][j]);
            ++count;
        }
    }

    return count ? sum / static_cast<double>(count) : 0.0;
}

static double meanEpipolarError(
    const std::vector<std::vector<cv::Point2f>> &left_points,
    const std::vector<std::vector<cv::Point2f>> &right_points,
    const cv::Mat &F)
{
    double total = 0.0;
    size_t count = 0;

    for (size_t i = 0; i < left_points.size(); ++i) {
        std::vector<cv::Vec3f> lines;
        cv::computeCorrespondEpilines(left_points[i], 1, F, lines);

        for (size_t j = 0; j < lines.size(); ++j) {
            const auto &l = lines[j];
            const auto &p = right_points[i][j];
            const double denom =
                std::sqrt(double(l[0]) * l[0] +
                          double(l[1]) * l[1]);

            if (denom > 1e-12) {
                total += std::abs(
                    l[0] * p.x + l[1] * p.y + l[2]) / denom;
                ++count;
            }
        }
    }

    return count ? total / static_cast<double>(count) : 0.0;
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

    const double tx = std::abs(P2.at<double>(0, 3));
    const double ty = std::abs(P2.at<double>(1, 3));
    const bool vertical_stereo = ty > tx;

    for (size_t i = 0; i < left_points.size(); ++i) {
        std::vector<cv::Point2f> lr, rr;

        cv::undistortPoints(
            left_points[i], lr,
            K1, D1, R1, P1);

        cv::undistortPoints(
            right_points[i], rr,
            K2, D2, R2, P2);

        if (lr.empty() || lr.size() != rr.size()) {
            result.push_back(1e9);
            continue;
        }

        double sum = 0.0;

        for (size_t j = 0; j < lr.size(); ++j) {
            sum += vertical_stereo
                ? std::abs(double(lr[j].x) - double(rr[j].x))
                : std::abs(double(lr[j].y) - double(rr[j].y));
        }

        result.push_back(sum / static_cast<double>(lr.size()));
    }

    return result;
}

static double robustOutlierThreshold(const std::vector<double> &errors)
{
    if (errors.empty())
        return 2.5;

    const double med = medianOf(errors);

    std::vector<double> deviations;
    deviations.reserve(errors.size());

    for (double e : errors)
        deviations.push_back(std::abs(e - med));

    const double mad = medianOf(deviations);

    // 1.4826 converts MAD to a Gaussian sigma estimate.
    // Keep a practical floor so normal sub-pixel/low-pixel noise is not over-pruned.
    return std::max(2.5, med + 2.5 * 1.4826 * mad);
}

int main(int argc, char **argv)
{
    if (argc < 4) {
        std::cerr
            << "Usage: " << argv[0]
            << " <capture_dir> <square_mm> <marker_mm> [output_yaml]\n";
        return 1;
    }

    const fs::path capture_dir = argv[1];
    const double square_mm = std::stod(argv[2]);
    const double marker_mm = std::stod(argv[3]);

    if (square_mm <= 0.0 || marker_mm <= 0.0 || marker_mm >= square_mm) {
        std::cerr << "Invalid ChArUco dimensions\n";
        return 1;
    }

    const fs::path output =
        argc > 4
            ? fs::path(argv[4])
            : fs::path("calibration/stereo_ov9281_ov5647.yaml");

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
        std::cerr << "No synchronized pairs found in " << capture_dir << "\n";
        return 1;
    }

    std::vector<std::vector<cv::Point3f>> mono_obj_l, mono_obj_r;
    std::vector<std::vector<cv::Point2f>> mono_img_l, mono_img_r;

    std::vector<std::vector<cv::Point3f>> stereo_obj;
    std::vector<std::vector<cv::Point2f>> stereo_l, stereo_r;
    std::vector<std::string> stereo_stems;

    cv::Size image_size;

    std::cout << "========================================\n";
    std::cout << " JT-Zero ChArUco stereo calibration v2\n";
    std::cout << "========================================\n";
    std::cout << "Board             : 7x5 / DICT_4X4_50\n";
    std::cout << "Square            : " << square_mm << " mm\n";
    std::cout << "Marker            : " << marker_mm << " mm\n";
    std::cout << "Pairs on disk     : " << pairs.size() << "\n\n";

    for (const auto &pair : pairs) {
        cv::Mat left = cv::imread(pair.left.string(), cv::IMREAD_GRAYSCALE);
        cv::Mat right = cv::imread(pair.right.string(), cv::IMREAD_GRAYSCALE);

        if (left.empty() || right.empty() || left.size() != right.size()) {
            std::cout << "[SKIP] " << pair.stem << " image read/size\n";
            continue;
        }

        if (image_size.empty())
            image_size = left.size();

        const Detection dl = detectCharuco(left, dictionary, board);
        const Detection dr = detectCharuco(right, dictionary, board);

        std::vector<cv::Point3f> obj_l, obj_r;
        std::vector<cv::Point2f> img_l, img_r;

        buildMono(dl, board_corners, obj_l, img_l);
        buildMono(dr, board_corners, obj_r, img_r);

        if (static_cast<int>(obj_l.size()) >= MIN_MONO_CORNERS) {
            mono_obj_l.push_back(obj_l);
            mono_img_l.push_back(img_l);
        }

        if (static_cast<int>(obj_r.size()) >= MIN_MONO_CORNERS) {
            mono_obj_r.push_back(obj_r);
            mono_img_r.push_back(img_r);
        }

        std::vector<cv::Point3f> sobj;
        std::vector<cv::Point2f> sl, sr;
        buildShared(dl, dr, board_corners, sobj, sl, sr);

        if (static_cast<int>(sobj.size()) >= MIN_SHARED_CORNERS) {
            stereo_obj.push_back(std::move(sobj));
            stereo_l.push_back(std::move(sl));
            stereo_r.push_back(std::move(sr));
            stereo_stems.push_back(pair.stem);

            std::cout << "[OK]   " << pair.stem
                      << " L=" << dl.ids.size()
                      << " R=" << dr.ids.size()
                      << " shared=" << stereo_obj.back().size() << "\n";
        } else {
            std::cout << "[SKIP] " << pair.stem
                      << " L=" << dl.ids.size()
                      << " R=" << dr.ids.size()
                      << " shared=" << sobj.size()
                      << " (<" << MIN_SHARED_CORNERS << ")\n";
        }
    }

    std::cout << "\nMono views left   : " << mono_obj_l.size() << "\n";
    std::cout << "Mono views right  : " << mono_obj_r.size() << "\n";
    std::cout << "Stereo pairs init : " << initial_stereo_pairs << "\n";
    std::cout << "Stereo pairs final: " << stereo_obj.size() << "\n";
    std::cout << "Rejected pairs    : " << rejected_stems.size() << "\n";

    if (mono_obj_l.size() < 10 || mono_obj_r.size() < 10) {
        std::cerr << "Need >=10 mono views for each camera\n";
        return 2;
    }

    if (stereo_obj.size() < MIN_STEREO_PAIRS) {
        std::cerr << "Need >= " << MIN_STEREO_PAIRS
                  << " stereo pairs with shared >= "
                  << MIN_SHARED_CORNERS << "\n";
        return 3;
    }

    cv::Mat K1 = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat D1 = cv::Mat::zeros(5, 1, CV_64F);
    cv::Mat K2 = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat D2 = cv::Mat::zeros(5, 1, CV_64F);

    std::vector<cv::Mat> rvecs1, tvecs1, rvecs2, tvecs2;

    const cv::TermCriteria criteria(
        cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
        100, 1e-9);

    const double rms1 = cv::calibrateCamera(
        mono_obj_l, mono_img_l, image_size,
        K1, D1, rvecs1, tvecs1, 0, criteria);

    const double rms2 = cv::calibrateCamera(
        mono_obj_r, mono_img_r, image_size,
        K2, D2, rvecs2, tvecs2, 0, criteria);

    const double reproj1 = meanReprojectionError(
        mono_obj_l, mono_img_l, rvecs1, tvecs1, K1, D1);

    const double reproj2 = meanReprojectionError(
        mono_obj_r, mono_img_r, rvecs2, tvecs2, K2, D2);

    const size_t initial_stereo_pairs = stereo_obj.size();
    std::vector<std::string> rejected_stems;

    cv::Mat R, T, E, F;
    cv::Mat R1, R2, P1, P2, Q;
    cv::Rect roi1, roi2;
    double stereo_rms = 0.0;

    // Robust refinement: fit, measure rectified residual per pair,
    // remove obvious outliers, then fit again. Intrinsics stay fixed.
    for (int iteration = 0; iteration < 3; ++iteration) {
        stereo_rms = cv::stereoCalibrate(
            stereo_obj, stereo_l, stereo_r,
            K1, D1, K2, D2, image_size,
            R, T, E, F,
            cv::CALIB_FIX_INTRINSIC,
            criteria);

        cv::stereoRectify(
            K1, D1, K2, D2, image_size,
            R, T, R1, R2, P1, P2, Q,
            cv::CALIB_ZERO_DISPARITY, 0.0,
            image_size, &roi1, &roi2);

        const std::vector<double> pair_errors =
            rectifiedPairResiduals(
                stereo_l, stereo_r,
                K1, D1, K2, D2,
                R1, R2, P1, P2);

        const double threshold =
            robustOutlierThreshold(pair_errors);

        std::vector<size_t> keep;
        std::vector<size_t> reject;

        for (size_t i = 0; i < pair_errors.size(); ++i) {
            if (pair_errors[i] > threshold)
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
            std::cout << "  reject " << stereo_stems[idx]
                      << " mean_rectified_residual="
                      << pair_errors[idx] << " px\n";
        }

        if (reject.empty())
            break;

        if (keep.size() < static_cast<size_t>(MIN_STEREO_PAIRS)) {
            std::cout << "  stop: pruning would leave fewer than "
                      << MIN_STEREO_PAIRS << " stereo pairs\n";
            break;
        }

        std::vector<std::vector<cv::Point3f>> new_obj;
        std::vector<std::vector<cv::Point2f>> new_l;
        std::vector<std::vector<cv::Point2f>> new_r;
        std::vector<std::string> new_stems;

        new_obj.reserve(keep.size());
        new_l.reserve(keep.size());
        new_r.reserve(keep.size());
        new_stems.reserve(keep.size());

        for (size_t idx : reject)
            rejected_stems.push_back(stereo_stems[idx]);

        for (size_t idx : keep) {
            new_obj.push_back(stereo_obj[idx]);
            new_l.push_back(stereo_l[idx]);
            new_r.push_back(stereo_r[idx]);
            new_stems.push_back(stereo_stems[idx]);
        }

        stereo_obj.swap(new_obj);
        stereo_l.swap(new_l);
        stereo_r.swap(new_r);
        stereo_stems.swap(new_stems);
    }

    // Final fit after the last pruning step.
    stereo_rms = cv::stereoCalibrate(
        stereo_obj, stereo_l, stereo_r,
        K1, D1, K2, D2, image_size,
        R, T, E, F,
        cv::CALIB_FIX_INTRINSIC,
        criteria);

    const double epi_error =
        meanEpipolarError(stereo_l, stereo_r, F);

    cv::stereoRectify(
        K1, D1, K2, D2, image_size,
        R, T, R1, R2, P1, P2, Q,
        cv::CALIB_ZERO_DISPARITY, 0.0,
        image_size, &roi1, &roi2);

    const std::vector<double> final_pair_errors =
        rectifiedPairResiduals(
            stereo_l, stereo_r,
            K1, D1, K2, D2,
            R1, R2, P1, P2);

    std::vector<double> final_all_corner_residuals;
    for (size_t i = 0; i < stereo_l.size(); ++i) {
        std::vector<cv::Point2f> lr, rr;

        cv::undistortPoints(
            stereo_l[i], lr,
            K1, D1, R1, P1);

        cv::undistortPoints(
            stereo_r[i], rr,
            K2, D2, R2, P2);

        const bool vertical_stereo =
            std::abs(P2.at<double>(1, 3)) >
            std::abs(P2.at<double>(0, 3));

        for (size_t j = 0; j < lr.size(); ++j) {
            final_all_corner_residuals.push_back(
                vertical_stereo
                    ? std::abs(double(lr[j].x) - double(rr[j].x))
                    : std::abs(double(lr[j].y) - double(rr[j].y)));
        }
    }

    const double final_rect_mean =
        final_all_corner_residuals.empty()
            ? 0.0
            : std::accumulate(
                  final_all_corner_residuals.begin(),
                  final_all_corner_residuals.end(),
                  0.0) /
              static_cast<double>(final_all_corner_residuals.size());

    const double final_rect_median =
        medianOf(final_all_corner_residuals);

    std::sort(
        final_all_corner_residuals.begin(),
        final_all_corner_residuals.end());

    const size_t p95_idx =
        final_all_corner_residuals.empty()
            ? 0
            : static_cast<size_t>(
                  std::floor(
                      0.95 * (final_all_corner_residuals.size() - 1)));

    const double final_rect_p95 =
        final_all_corner_residuals.empty()
            ? 0.0
            : final_all_corner_residuals[p95_idx];

    const double final_rect_max =
        final_all_corner_residuals.empty()
            ? 0.0
            : final_all_corner_residuals.back();

    if (!output.parent_path().empty())
        fs::create_directories(output.parent_path());

    cv::FileStorage out(output.string(), cv::FileStorage::WRITE);

    out << "camera_left" << "OV9281_USB";
    out << "camera_right" << "OV5647_CSI";
    out << "image_width" << image_size.width;
    out << "image_height" << image_size.height;
    out << "board_type" << "charuco";
    out << "board_squares_x" << BOARD_SQUARES_X;
    out << "board_squares_y" << BOARD_SQUARES_Y;
    out << "dictionary" << "DICT_4X4_50";
    out << "square_length_mm" << square_mm;
    out << "marker_length_mm" << marker_mm;
    out << "mono_views_left" << static_cast<int>(mono_obj_l.size());
    out << "mono_views_right" << static_cast<int>(mono_obj_r.size());
    out << "stereo_pairs_initial" << static_cast<int>(initial_stereo_pairs);
    out << "stereo_pairs_final" << static_cast<int>(stereo_obj.size());
    out << "rejected_stereo_pairs" << static_cast<int>(rejected_stems.size());
    out << "rms_left" << rms1;
    out << "rms_right" << rms2;
    out << "mean_reprojection_left_px" << reproj1;
    out << "mean_reprojection_right_px" << reproj2;
    out << "rms_stereo" << stereo_rms;
    out << "mean_epipolar_error_px" << epi_error;
    out << "rectified_mean_residual_px" << final_rect_mean;
    out << "rectified_median_residual_px" << final_rect_median;
    out << "rectified_p95_residual_px" << final_rect_p95;
    out << "rectified_max_residual_px" << final_rect_max;
    out << "baseline_m" << cv::norm(T);
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

    out << "rejected_pair_stems" << "[";
    for (const std::string &stem : rejected_stems)
        out << stem;
    out << "]";

    out << "final_pair_stems" << "[";
    for (const std::string &stem : stereo_stems)
        out << stem;
    out << "]";

    out.release();

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "\n========================================\n";
    std::cout << " RESULT\n";
    std::cout << "========================================\n";
    std::cout << "Mono views left   : " << mono_obj_l.size() << "\n";
    std::cout << "Mono views right  : " << mono_obj_r.size() << "\n";
    std::cout << "Stereo pairs      : " << stereo_obj.size() << "\n";
    std::cout << "Left RMS          : " << rms1 << " px\n";
    std::cout << "Right RMS         : " << rms2 << " px\n";
    std::cout << "Left mean reproj  : " << reproj1 << " px\n";
    std::cout << "Right mean reproj : " << reproj2 << " px\n";
    std::cout << "Stereo RMS        : " << stereo_rms << " px\n";
    std::cout << "Epipolar error    : " << epi_error << " px\n";
    std::cout << "Rect mean residual: " << final_rect_mean << " px\n";
    std::cout << "Rect median resid : " << final_rect_median << " px\n";
    std::cout << "Rect p95 residual : " << final_rect_p95 << " px\n";
    std::cout << "Rect max residual : " << final_rect_max << " px\n";
    std::cout << "Baseline          : " << cv::norm(T) * 1000.0 << " mm\n";
    std::cout << "Calibration file  : " << output << "\n";
    std::cout << "========================================\n";

    return 0;
}
