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

struct PairFiles {
    fs::path left;
    fs::path right;
    std::string stem;
};

struct Detection {
    std::vector<cv::Point2f> corners;
    std::vector<int> ids;
};

struct PairStats {
    std::string stem;
    int shared{0};
    double mean_abs_dy{0.0};
    double median_abs_dy{0.0};
    double p95_abs_dy{0.0};
    double max_abs_dy{0.0};
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

static void sharedPoints(
    const Detection &left,
    const Detection &right,
    std::vector<cv::Point2f> &lp,
    std::vector<cv::Point2f> &rp,
    std::vector<int> &ids)
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

        ids.push_back(id);
        lp.push_back(p);
        rp.push_back(it->second);
    }
}

static double percentile(std::vector<double> v, double p)
{
    if (v.empty())
        return 0.0;

    std::sort(v.begin(), v.end());

    const double pos = p * (v.size() - 1);
    const size_t lo = static_cast<size_t>(std::floor(pos));
    const size_t hi = static_cast<size_t>(std::ceil(pos));

    if (lo == hi)
        return v[lo];

    const double f = pos - lo;
    return v[lo] * (1.0 - f) + v[hi] * f;
}

int main(int argc, char **argv)
{
    if (argc < 3) {
        std::cerr
            << "Usage: " << argv[0]
            << " <capture_dir> <calibration_yaml>\n";
        return 1;
    }

    const fs::path capture_dir = argv[1];
    const fs::path calib_path = argv[2];

    cv::FileStorage fsin(calib_path.string(), cv::FileStorage::READ);
    if (!fsin.isOpened()) {
        std::cerr << "Cannot open calibration: " << calib_path << "\n";
        return 1;
    }

    cv::Mat K1, D1, K2, D2, R1, R2, P1, P2;
    double square_mm = 27.324;
    double marker_mm = 20.043;

    fsin["K1"] >> K1;
    fsin["D1"] >> D1;
    fsin["K2"] >> K2;
    fsin["D2"] >> D2;
    fsin["R1"] >> R1;
    fsin["R2"] >> R2;
    fsin["P1"] >> P1;
    fsin["P2"] >> P2;

    if (!fsin["square_length_mm"].empty())
        fsin["square_length_mm"] >> square_mm;

    if (!fsin["marker_length_mm"].empty())
        fsin["marker_length_mm"] >> marker_mm;

    fsin.release();

    if (K1.empty() || D1.empty() || K2.empty() || D2.empty() ||
        R1.empty() || R2.empty() || P1.empty() || P2.empty()) {
        std::cerr << "Calibration YAML is missing K/D/R/P matrices\n";
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

    const auto pairs = findPairs(capture_dir);

    if (pairs.empty()) {
        std::cerr << "No synchronized pairs found in " << capture_dir << "\n";
        return 1;
    }

    const fs::path preview_dir =
        capture_dir / "rectify_check";

    fs::create_directories(preview_dir);

    std::vector<PairStats> stats;
    std::vector<double> all_abs_dy;

    std::cout << "========================================\n";
    std::cout << " JT-Zero stereo rectification check\n";
    std::cout << "========================================\n";
    std::cout << "Pairs             : " << pairs.size() << "\n";
    std::cout << "Calibration       : " << calib_path << "\n\n";

    for (const auto &pair : pairs) {
        cv::Mat left_gray =
            cv::imread(pair.left.string(), cv::IMREAD_GRAYSCALE);

        cv::Mat right_gray =
            cv::imread(pair.right.string(), cv::IMREAD_GRAYSCALE);

        if (left_gray.empty() || right_gray.empty())
            continue;

        const Detection dl =
            detectCharuco(left_gray, dictionary, board);

        const Detection dr =
            detectCharuco(right_gray, dictionary, board);

        std::vector<cv::Point2f> lp, rp;
        std::vector<int> ids;

        sharedPoints(dl, dr, lp, rp, ids);

        if (lp.size() < 4) {
            std::cout << "[SKIP] " << pair.stem
                      << " shared=" << lp.size() << "\n";
            continue;
        }

        std::vector<cv::Point2f> lrect, rrect;

        cv::undistortPoints(
            lp, lrect,
            K1, D1,
            R1, P1);

        cv::undistortPoints(
            rp, rrect,
            K2, D2,
            R2, P2);

        std::vector<double> abs_dy;
        abs_dy.reserve(lrect.size());

        for (size_t i = 0; i < lrect.size(); ++i) {
            const double dy =
                std::abs(double(lrect[i].y) -
                         double(rrect[i].y));

            abs_dy.push_back(dy);
            all_abs_dy.push_back(dy);
        }

        const double mean =
            std::accumulate(abs_dy.begin(), abs_dy.end(), 0.0) /
            static_cast<double>(abs_dy.size());

        PairStats ps;
        ps.stem = pair.stem;
        ps.shared = static_cast<int>(abs_dy.size());
        ps.mean_abs_dy = mean;
        ps.median_abs_dy = percentile(abs_dy, 0.50);
        ps.p95_abs_dy = percentile(abs_dy, 0.95);
        ps.max_abs_dy = *std::max_element(abs_dy.begin(), abs_dy.end());

        stats.push_back(ps);

        std::cout
            << "[PAIR] " << std::setw(11) << pair.stem
            << " shared=" << std::setw(2) << ps.shared
            << " mean|dy|=" << std::fixed << std::setprecision(3)
            << ps.mean_abs_dy
            << " p95=" << ps.p95_abs_dy
            << " max=" << ps.max_abs_dy
            << " px\n";

        cv::Mat left_color, right_color;

        cv::cvtColor(left_gray, left_color, cv::COLOR_GRAY2BGR);
        cv::cvtColor(right_gray, right_color, cv::COLOR_GRAY2BGR);

        cv::Mat map1x, map1y, map2x, map2y;

        cv::initUndistortRectifyMap(
            K1, D1, R1, P1,
            left_gray.size(),
            CV_32FC1,
            map1x, map1y);

        cv::initUndistortRectifyMap(
            K2, D2, R2, P2,
            right_gray.size(),
            CV_32FC1,
            map2x, map2y);

        cv::Mat rl, rr;

        cv::remap(
            left_color, rl,
            map1x, map1y,
            cv::INTER_LINEAR);

        cv::remap(
            right_color, rr,
            map2x, map2y,
            cv::INTER_LINEAR);

        for (size_t i = 0; i < lrect.size(); ++i) {
            cv::circle(
                rl, lrect[i], 4,
                cv::Scalar(0,255,255), 1);

            cv::circle(
                rr, rrect[i], 4,
                cv::Scalar(0,255,255), 1);

            cv::putText(
                rl, std::to_string(ids[i]),
                lrect[i] + cv::Point2f(4,-4),
                cv::FONT_HERSHEY_SIMPLEX,
                0.4,
                cv::Scalar(0,255,255), 1);

            cv::putText(
                rr, std::to_string(ids[i]),
                rrect[i] + cv::Point2f(4,-4),
                cv::FONT_HERSHEY_SIMPLEX,
                0.4,
                cv::Scalar(0,255,255), 1);
        }

        cv::Mat joined;
        cv::hconcat(rl, rr, joined);

        for (int y = 30; y < joined.rows; y += 30) {
            cv::line(
                joined,
                cv::Point(0, y),
                cv::Point(joined.cols - 1, y),
                cv::Scalar(0,255,0), 1);
        }

        cv::putText(
            joined,
            "mean|dy|=" + cv::format("%.3f px", ps.mean_abs_dy),
            cv::Point(15, joined.rows - 18),
            cv::FONT_HERSHEY_SIMPLEX,
            0.65,
            cv::Scalar(255,255,255), 2);

        cv::imwrite(
            (preview_dir /
             (pair.stem + "_rectify_check.png")).string(),
            joined);
    }

    if (stats.empty()) {
        std::cerr << "No pairs with >=4 shared ChArUco corners\n";
        return 2;
    }

    std::sort(
        stats.begin(), stats.end(),
        [](const PairStats &a, const PairStats &b) {
            return a.mean_abs_dy > b.mean_abs_dy;
        });

    const double global_mean =
        std::accumulate(
            all_abs_dy.begin(),
            all_abs_dy.end(),
            0.0) /
        static_cast<double>(all_abs_dy.size());

    std::cout << "\n========================================\n";
    std::cout << " GLOBAL\n";
    std::cout << "========================================\n";
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Pairs evaluated   : " << stats.size() << "\n";
    std::cout << "Corners evaluated : " << all_abs_dy.size() << "\n";
    std::cout << "mean |dy|         : " << global_mean << " px\n";
    std::cout << "median |dy|       : "
              << percentile(all_abs_dy, 0.50) << " px\n";
    std::cout << "p95 |dy|          : "
              << percentile(all_abs_dy, 0.95) << " px\n";
    std::cout << "max |dy|          : "
              << *std::max_element(
                     all_abs_dy.begin(),
                     all_abs_dy.end())
              << " px\n";

    std::cout << "\nWorst pairs:\n";

    const size_t worst_n =
        std::min<size_t>(10, stats.size());

    for (size_t i = 0; i < worst_n; ++i) {
        std::cout
            << "  " << stats[i].stem
            << " shared=" << stats[i].shared
            << " mean|dy|=" << stats[i].mean_abs_dy
            << " p95=" << stats[i].p95_abs_dy
            << " max=" << stats[i].max_abs_dy
            << " px\n";
    }

    std::cout << "\nPreviews          : " << preview_dir << "\n";
    std::cout << "========================================\n";

    return 0;
}
