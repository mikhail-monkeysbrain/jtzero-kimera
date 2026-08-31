#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {
std::vector<fs::path> findImages(const fs::path& dir) {
    std::vector<fs::path> images;
    if (!fs::exists(dir)) return images;
    for (const auto& e : fs::directory_iterator(dir)) {
        if (!e.is_regular_file()) continue;
        const auto name = e.path().filename().string();
        if (name.rfind("frame_", 0) == 0 && e.path().extension() == ".png") images.push_back(e.path());
    }
    std::sort(images.begin(), images.end());
    return images;
}
}

int main(int argc, char** argv) {
    const fs::path input_dir = argc > 1 ? argv[1] : "/home/vio/charuco_validation";
    const fs::path calib_path = argc > 2 ? argv[2] : "/home/vio/charuco_calibration/ov9281_intrinsics.yaml";
    const fs::path output_dir = argc > 3 ? argv[3] : "/home/vio/charuco_undistorted";

    cv::FileStorage fs_calib(calib_path.string(), cv::FileStorage::READ);
    if (!fs_calib.isOpened()) {
        std::cerr << "ERROR: cannot open calibration: " << calib_path << '\n';
        return 1;
    }
    int width = 0, height = 0;
    cv::Mat K, D;
    fs_calib["image_width"] >> width;
    fs_calib["image_height"] >> height;
    fs_calib["camera_matrix"] >> K;
    fs_calib["distortion_coefficients"] >> D;
    fs_calib.release();
    if (width <= 0 || height <= 0 || K.empty() || D.empty()) {
        std::cerr << "ERROR: incomplete calibration file\n";
        return 2;
    }

    const auto images = findImages(input_dir);
    if (images.empty()) {
        std::cerr << "ERROR: no frame_*.png in " << input_dir << '\n';
        return 3;
    }
    fs::create_directories(output_dir);

    std::cout << std::fixed << std::setprecision(6)
              << "=== OV9281 UNDISTORTION CHECK ===\n"
              << "input: " << input_dir << '\n'
              << "images: " << images.size() << '\n'
              << "calibration: " << calib_path << '\n'
              << "output: " << output_dir << '\n'
              << "mode: " << width << 'x' << height << '\n'
              << "fx=" << K.at<double>(0,0) << " fy=" << K.at<double>(1,1)
              << " cx=" << K.at<double>(0,2) << " cy=" << K.at<double>(1,2) << '\n';

    cv::Mat map1, map2;
    cv::initUndistortRectifyMap(K, D, cv::Mat(), K, cv::Size(width, height), CV_16SC2, map1, map2);

    int written = 0;
    for (const auto& path : images) {
        cv::Mat src = cv::imread(path.string(), cv::IMREAD_GRAYSCALE);
        if (src.empty() || src.cols != width || src.rows != height) {
            std::cout << "SKIP " << path.filename().string() << " invalid image/mode\n";
            continue;
        }
        cv::Mat undist;
        cv::remap(src, undist, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);

        const fs::path out = output_dir / path.filename();
        if (!cv::imwrite(out.string(), undist)) {
            std::cerr << "ERROR: cannot write " << out << '\n';
            return 4;
        }
        ++written;
    }

    // Build one contact sheet from evenly distributed validation frames.
    const int sample_count = std::min<int>(9, images.size());
    const int thumb_w = width / 2;
    const int thumb_h = height / 2;
    cv::Mat sheet(thumb_h * 3, thumb_w * 6, CV_8UC1, cv::Scalar(0));
    for (int i = 0; i < sample_count; ++i) {
        const size_t idx = sample_count == 1 ? 0 : (images.size() - 1) * i / (sample_count - 1);
        cv::Mat src = cv::imread(images[idx].string(), cv::IMREAD_GRAYSCALE);
        if (src.empty()) continue;
        cv::Mat undist;
        cv::remap(src, undist, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
        cv::Mat a, b;
        cv::resize(src, a, cv::Size(thumb_w, thumb_h));
        cv::resize(undist, b, cv::Size(thumb_w, thumb_h));
        const int row = i / 3;
        const int pair_col = i % 3;
        a.copyTo(sheet(cv::Rect(pair_col * 2 * thumb_w, row * thumb_h, thumb_w, thumb_h)));
        b.copyTo(sheet(cv::Rect((pair_col * 2 + 1) * thumb_w, row * thumb_h, thumb_w, thumb_h)));
        cv::putText(sheet, "RAW", cv::Point(pair_col * 2 * thumb_w + 8, row * thumb_h + 22),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255), 1);
        cv::putText(sheet, "UNDIST", cv::Point((pair_col * 2 + 1) * thumb_w + 8, row * thumb_h + 22),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255), 1);
    }
    const fs::path sheet_path = output_dir / "comparison_contact_sheet.png";
    cv::imwrite(sheet_path.string(), sheet);

    std::cout << "written=" << written << '\n'
              << "contact_sheet=" << sheet_path << '\n'
              << "CHECK: straight target edges/lines must become straight without waves, bends, or asymmetric warping.\n"
              << "CHECK: inspect especially all four image edges and corners.\n"
              << "RESULT: files generated; visual inspection required.\n";
    return written > 0 ? 0 : 5;
}
