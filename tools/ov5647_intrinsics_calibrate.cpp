#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

namespace fs = std::filesystem;

static constexpr int BOARD_SQUARES_X = 7;
static constexpr int BOARD_SQUARES_Y = 5;
static constexpr int MIN_CORNERS_PER_VIEW = 4;
static constexpr int MIN_VIEWS = 15;

struct Detection {
    std::vector<cv::Point2f> corners;
    std::vector<int> ids;
};

static Detection detectCharuco(
    const cv::Mat &gray,
    const cv::Ptr<cv::aruco::Dictionary> &dictionary,
    const cv::Ptr<cv::aruco::CharucoBoard> &board)
{
    std::vector<std::vector<cv::Point2f>> marker_corners;
    std::vector<int> marker_ids;

    auto params = cv::makePtr<cv::aruco::DetectorParameters>();

    cv::aruco::detectMarkers(
        gray,
        dictionary,
        marker_corners,
        marker_ids,
        params);

    Detection out;

    if (marker_ids.empty())
        return out;

    cv::Mat charuco_corners;
    cv::Mat charuco_ids;

    const int n = cv::aruco::interpolateCornersCharuco(
        marker_corners,
        marker_ids,
        gray,
        board,
        charuco_corners,
        charuco_ids);

    if (n <= 0 || charuco_ids.empty())
        return out;

    out.corners.reserve(static_cast<size_t>(n));
    out.ids.reserve(static_cast<size_t>(n));

    for (int i = 0; i < n; ++i) {
        out.corners.push_back(charuco_corners.at<cv::Point2f>(i));
        out.ids.push_back(charuco_ids.at<int>(i));
    }

    return out;
}

static void buildCorrespondences(
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

static double meanReprojectionError(
    const std::vector<std::vector<cv::Point3f>> &object_points,
    const std::vector<std::vector<cv::Point2f>> &image_points,
    const std::vector<cv::Mat> &rvecs,
    const std::vector<cv::Mat> &tvecs,
    const cv::Mat &K,
    const cv::Mat &D)
{
    double total = 0.0;
    size_t count = 0;

    for (size_t i = 0; i < object_points.size(); ++i) {
        std::vector<cv::Point2f> projected;

        cv::projectPoints(
            object_points[i],
            rvecs[i],
            tvecs[i],
            K,
            D,
            projected);

        for (size_t j = 0; j < projected.size(); ++j) {
            total += cv::norm(projected[j] - image_points[i][j]);
            ++count;
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

static std::vector<double> perViewReprojectionError(
    const std::vector<std::vector<cv::Point3f>> &object_points,
    const std::vector<std::vector<cv::Point2f>> &image_points,
    const std::vector<cv::Mat> &rvecs,
    const std::vector<cv::Mat> &tvecs,
    const cv::Mat &K,
    const cv::Mat &D)
{
    std::vector<double> errors;
    errors.reserve(object_points.size());

    for (size_t i = 0; i < object_points.size(); ++i) {
        std::vector<cv::Point2f> projected;

        cv::projectPoints(
            object_points[i],
            rvecs[i],
            tvecs[i],
            K,
            D,
            projected);

        double sum = 0.0;

        for (size_t j = 0; j < projected.size(); ++j)
            sum += cv::norm(projected[j] - image_points[i][j]);

        errors.push_back(
            projected.empty() ? 1e9 :
            sum / static_cast<double>(projected.size()));
    }

    return errors;
}

static double robustViewThreshold(const std::vector<double> &errors)
{
    if (errors.empty())
        return 1.0;

    const double med = medianOf(errors);

    std::vector<double> dev;
    dev.reserve(errors.size());

    for (double e : errors)
        dev.push_back(std::abs(e - med));

    const double mad = medianOf(dev);

    return std::max(1.0, med + 2.5 * 1.4826 * mad);
}

int main(int argc, char **argv)
{
    if (argc < 4) {
        std::cerr
            << "Usage:\n"
            << "  " << argv[0]
            << " <capture_dir> <square_mm> <marker_mm> [output_yaml]\n\n"
            << "Example:\n"
            << "  " << argv[0]
            << " ov5647_intrinsics_captures 27.324 20.043 "
               "calibration/ov5647_intrinsics.yaml\n";
        return 1;
    }

    const fs::path capture_dir = argv[1];
    const double square_mm = std::stod(argv[2]);
    const double marker_mm = std::stod(argv[3]);

    const fs::path output =
        argc > 4
            ? fs::path(argv[4])
            : fs::path("calibration/ov5647_intrinsics.yaml");

    if (!fs::exists(capture_dir)) {
        std::cerr << "Capture directory does not exist: "
                  << capture_dir << "\n";
        return 1;
    }

    if (square_mm <= 0.0 ||
        marker_mm <= 0.0 ||
        marker_mm >= square_mm) {
        std::cerr << "Invalid ChArUco dimensions\n";
        return 1;
    }

    const cv::aruco::Dictionary dictionary_value =
        cv::aruco::getPredefinedDictionary(
            cv::aruco::DICT_4X4_50);

    const auto dictionary =
        cv::makePtr<cv::aruco::Dictionary>(
            dictionary_value);

    const auto board =
        cv::makePtr<cv::aruco::CharucoBoard>(
            cv::Size(BOARD_SQUARES_X, BOARD_SQUARES_Y),
            static_cast<float>(square_mm / 1000.0),
            static_cast<float>(marker_mm / 1000.0),
            dictionary_value);

    const std::vector<cv::Point3f> board_corners =
        board->getChessboardCorners();

    std::vector<fs::path> images;

    for (const auto &entry : fs::directory_iterator(capture_dir)) {
        if (!entry.is_regular_file())
            continue;

        const auto ext = entry.path().extension().string();

        if (ext == ".png" || ext == ".jpg" || ext == ".jpeg")
            images.push_back(entry.path());
    }

    std::sort(images.begin(), images.end());

    if (images.empty()) {
        std::cerr << "No images found in " << capture_dir << "\n";
        return 1;
    }

    const fs::path preview_dir =
        capture_dir / "charuco_detection";

    fs::create_directories(preview_dir);

    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> image_points;
    std::vector<std::string> accepted_files;

    cv::Size image_size;

    std::cout << "========================================\n";
    std::cout << " JT-Zero OV5647 intrinsics calibration\n";
    std::cout << "========================================\n";
    std::cout << "Board             : 7x5 / DICT_4X4_50\n";
    std::cout << "Square            : " << square_mm << " mm\n";
    std::cout << "Marker            : " << marker_mm << " mm\n";
    std::cout << "Images on disk    : " << images.size() << "\n\n";

    for (const auto &path : images) {
        cv::Mat gray =
            cv::imread(path.string(), cv::IMREAD_GRAYSCALE);

        if (gray.empty()) {
            std::cout << "[SKIP] " << path.filename().string()
                      << " read failed\n";
            continue;
        }

        if (image_size.empty())
            image_size = gray.size();

        if (gray.size() != image_size) {
            std::cout << "[SKIP] " << path.filename().string()
                      << " size mismatch\n";
            continue;
        }

        const Detection d =
            detectCharuco(gray, dictionary, board);

        std::vector<cv::Point3f> obj;
        std::vector<cv::Point2f> img;

        buildCorrespondences(
            d,
            board_corners,
            obj,
            img);

        cv::Mat preview;
        cv::cvtColor(gray, preview, cv::COLOR_GRAY2BGR);

        for (size_t i = 0; i < d.corners.size(); ++i) {
            cv::circle(
                preview,
                d.corners[i],
                4,
                cv::Scalar(0,255,0),
                1);

            cv::putText(
                preview,
                std::to_string(d.ids[i]),
                d.corners[i] + cv::Point2f(4,-4),
                cv::FONT_HERSHEY_SIMPLEX,
                0.4,
                cv::Scalar(0,255,0),
                1);
        }

        cv::imwrite(
            (preview_dir /
             (path.stem().string() + "_detected.png")).string(),
            preview);

        if (static_cast<int>(obj.size()) < MIN_CORNERS_PER_VIEW) {
            std::cout << "[SKIP] " << path.filename().string()
                      << " corners=" << obj.size()
                      << " (<" << MIN_CORNERS_PER_VIEW << ")\n";
            continue;
        }

        object_points.push_back(std::move(obj));
        image_points.push_back(std::move(img));
        accepted_files.push_back(path.filename().string());

        std::cout << "[OK]   " << path.filename().string()
                  << " corners=" << object_points.back().size()
                  << "\n";
    }

    std::cout << "\nUsable views      : "
              << object_points.size()
              << " / " << images.size()
              << "\n";

    if (static_cast<int>(object_points.size()) < MIN_VIEWS) {
        std::cerr
            << "Need at least " << MIN_VIEWS
            << " usable views. 25-40 diverse views are recommended.\n";
        return 2;
    }

    const size_t initial_views = object_points.size();
    std::vector<std::string> rejected_files;

    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat D = cv::Mat::zeros(5, 1, CV_64F);

    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;

    const cv::TermCriteria criteria(
        cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
        100,
        1e-9);

    double rms = 0.0;

    for (int iteration = 0; iteration < 3; ++iteration) {
        K = cv::Mat::eye(3, 3, CV_64F);
        D = cv::Mat::zeros(5, 1, CV_64F);
        rvecs.clear();
        tvecs.clear();

        rms = cv::calibrateCamera(
            object_points,
            image_points,
            image_size,
            K,
            D,
            rvecs,
            tvecs,
            0,
            criteria);

        const std::vector<double> view_errors =
            perViewReprojectionError(
                object_points,
                image_points,
                rvecs,
                tvecs,
                K,
                D);

        const double threshold =
            robustViewThreshold(view_errors);

        std::vector<size_t> keep;
        std::vector<size_t> reject;

        for (size_t i = 0; i < view_errors.size(); ++i) {
            if (view_errors[i] > threshold)
                reject.push_back(i);
            else
                keep.push_back(i);
        }

        std::cout << "\n[REFINE " << (iteration + 1) << "]"
                  << " views=" << object_points.size()
                  << " threshold=" << std::fixed << std::setprecision(3)
                  << threshold << " px"
                  << " reject=" << reject.size() << "\n";

        for (size_t idx : reject) {
            std::cout << "  reject " << accepted_files[idx]
                      << " mean_reproj=" << view_errors[idx] << " px\n";
        }

        if (reject.empty())
            break;

        if (keep.size() < static_cast<size_t>(MIN_VIEWS)) {
            std::cout << "  stop: pruning would leave fewer than "
                      << MIN_VIEWS << " views\n";
            break;
        }

        std::vector<std::vector<cv::Point3f>> new_obj;
        std::vector<std::vector<cv::Point2f>> new_img;
        std::vector<std::string> new_files;

        new_obj.reserve(keep.size());
        new_img.reserve(keep.size());
        new_files.reserve(keep.size());

        for (size_t idx : reject)
            rejected_files.push_back(accepted_files[idx]);

        for (size_t idx : keep) {
            new_obj.push_back(object_points[idx]);
            new_img.push_back(image_points[idx]);
            new_files.push_back(accepted_files[idx]);
        }

        object_points.swap(new_obj);
        image_points.swap(new_img);
        accepted_files.swap(new_files);
    }

    // Final calibration after pruning.
    K = cv::Mat::eye(3, 3, CV_64F);
    D = cv::Mat::zeros(5, 1, CV_64F);
    rvecs.clear();
    tvecs.clear();

    rms = cv::calibrateCamera(
        object_points,
        image_points,
        image_size,
        K,
        D,
        rvecs,
        tvecs,
        0,
        criteria);

    const double mean_reprojection =
        meanReprojectionError(
            object_points,
            image_points,
            rvecs,
            tvecs,
            K,
            D);

    if (!output.parent_path().empty())
        fs::create_directories(output.parent_path());

    cv::FileStorage out(
        output.string(),
        cv::FileStorage::WRITE);

    if (!out.isOpened()) {
        std::cerr << "Cannot write " << output << "\n";
        return 3;
    }

    out << "camera_name" << "OV5647_CSI";
    out << "image_width" << image_size.width;
    out << "image_height" << image_size.height;
    out << "camera_model" << "pinhole";
    out << "distortion_model" << "radtan_5";

    out << "board_squares_x" << BOARD_SQUARES_X;
    out << "board_squares_y" << BOARD_SQUARES_Y;
    out << "board_square_length_mm" << square_mm;
    out << "board_marker_length_mm" << marker_mm;

    out << "camera_matrix" << K;
    out << "distortion_coefficients" << D;

    out << "rms_reprojection_error_px" << rms;
    out << "mean_reprojection_error_px" << mean_reprojection;
    out << "initial_usable_views" << static_cast<int>(initial_views);
    out << "usable_views" << static_cast<int>(object_points.size());
    out << "rejected_views" << static_cast<int>(rejected_files.size());

    out << "accepted_files" << "[";
    for (const auto &name : accepted_files)
        out << name;
    out << "]";

    out << "rejected_files" << "[";
    for (const auto &name : rejected_files)
        out << name;
    out << "]";

    out.release();

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "\n========================================\n";
    std::cout << " RESULT\n";
    std::cout << "========================================\n";
    std::cout << "Views initial     : " << initial_views << "\n";
    std::cout << "Views final       : " << object_points.size() << "\n";
    std::cout << "Rejected views    : " << rejected_files.size() << "\n";
    std::cout << "RMS               : " << rms << " px\n";
    std::cout << "Mean reprojection : " << mean_reprojection << " px\n";
    std::cout << "fx                 : " << K.at<double>(0,0) << "\n";
    std::cout << "fy                 : " << K.at<double>(1,1) << "\n";
    std::cout << "cx                 : " << K.at<double>(0,2) << "\n";
    std::cout << "cy                 : " << K.at<double>(1,2) << "\n";
    std::cout << "D                  : "
              << D.at<double>(0) << ", "
              << D.at<double>(1) << ", "
              << D.at<double>(2) << ", "
              << D.at<double>(3) << ", "
              << D.at<double>(4) << "\n";
    std::cout << "Output             : " << output << "\n";
    std::cout << "Detection previews : " << preview_dir << "\n";
    std::cout << "========================================\n";

    return 0;
}
