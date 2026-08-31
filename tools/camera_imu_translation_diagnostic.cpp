#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

struct Frame {
    uint32_t sequence{};
    int64_t timestamp_ns{};
    uint64_t offset{};
    uint32_t bytes{};
};

static std::vector<std::string> splitCsv(const std::string& s) {
    std::vector<std::string> v;
    std::stringstream ss(s);
    std::string x;
    while (std::getline(ss, x, ',')) v.push_back(x);
    return v;
}

static std::vector<Frame> loadFrames(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("Cannot open camera CSV: " + path);
    std::string line;
    std::getline(in, line);
    std::vector<Frame> out;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        auto c = splitCsv(line);
        if (c.size() < 7) continue;
        Frame f;
        f.sequence = static_cast<uint32_t>(std::stoul(c[0]));
        f.timestamp_ns = std::stoll(c[2]);
        f.offset = std::stoull(c[5]);
        f.bytes = static_cast<uint32_t>(std::stoul(c[6]));
        out.push_back(f);
    }
    return out;
}

static bool readFrame(std::ifstream& in, const Frame& f, cv::Mat* img) {
    std::vector<unsigned char> buf(f.bytes);
    in.clear();
    in.seekg(static_cast<std::streamoff>(f.offset), std::ios::beg);
    if (!in) return false;
    in.read(reinterpret_cast<char*>(buf.data()), static_cast<std::streamsize>(buf.size()));
    if (in.gcount() != static_cast<std::streamsize>(buf.size())) return false;
    *img = cv::imdecode(buf, cv::IMREAD_GRAYSCALE);
    return !img->empty();
}

int main(int argc, char** argv) {
    try {
        const std::string camera_csv = argc > 1 ? argv[1] : "/home/vio/camera_imu_translation_yaw_40s_camera.csv";
        const std::string mjpeg_path = argc > 2 ? argv[2] : "/home/vio/camera_imu_translation_yaw_40s.mjpg";
        const std::string intrinsics_path = argc > 3 ? argv[3] : "calibration/ov9281_intrinsics.yaml";

        cv::FileStorage fs(intrinsics_path, cv::FileStorage::READ);
        if (!fs.isOpened()) throw std::runtime_error("Cannot open intrinsics YAML");
        int sx = 0, sy = 0;
        double square_mm = 0.0, marker_mm = 0.0;
        fs["board_squares_x"] >> sx;
        fs["board_squares_y"] >> sy;
        fs["board_square_length_mm"] >> square_mm;
        fs["board_marker_length_mm"] >> marker_mm;
        if (sx <= 1 || sy <= 1 || square_mm <= 0 || marker_mm <= 0)
            throw std::runtime_error("Invalid board parameters in intrinsics YAML");

        auto frames = loadFrames(camera_csv);
        std::ifstream mjpeg(mjpeg_path, std::ios::binary);
        if (!mjpeg) throw std::runtime_error("Cannot open MJPEG: " + mjpeg_path);

        auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        cv::aruco::CharucoBoard board(cv::Size(sx, sy),
                                     static_cast<float>(square_mm * 1e-3),
                                     static_cast<float>(marker_mm * 1e-3), dict);
        cv::aruco::CharucoDetector detector(board);

        constexpr size_t stride = 4;
        size_t attempts = 0, decode_fail = 0;
        size_t ge4 = 0, ge6 = 0, ge8 = 0, ge12 = 0;
        int max_corners = 0;
        std::map<int, size_t> histogram;

        for (size_t i = 0; i < frames.size(); i += stride) {
            ++attempts;
            cv::Mat img;
            if (!readFrame(mjpeg, frames[i], &img)) { ++decode_fail; continue; }
            cv::Mat corners, ids;
            detector.detectBoard(img, corners, ids);
            const int n = static_cast<int>(ids.total());
            histogram[n]++;
            max_corners = std::max(max_corners, n);
            if (n >= 4) ++ge4;
            if (n >= 6) ++ge6;
            if (n >= 8) ++ge8;
            if (n >= 12) ++ge12;
        }

        std::cout << "============================================================\n";
        std::cout << "CHARUCO VISIBILITY DIAGNOSTIC\n";
        std::cout << "============================================================\n";
        std::cout << "raw frames:          " << frames.size() << "\n";
        std::cout << "sampled attempts:    " << attempts << "\n";
        std::cout << "decode failures:     " << decode_fail << "\n";
        std::cout << "max corners/frame:   " << max_corners << "\n";
        std::cout << ">= 4 corners:        " << ge4 << "\n";
        std::cout << ">= 6 corners:        " << ge6 << "\n";
        std::cout << ">= 8 corners:        " << ge8 << "\n";
        std::cout << ">=12 corners:        " << ge12 << "\n";
        std::cout << "\nHistogram (corners : frames)\n";
        for (const auto& [n, count] : histogram)
            std::cout << std::setw(3) << n << " : " << count << "\n";

        if (ge6 >= 20) {
            std::cout << "\nRESULT: capture is probably salvageable with a 6-corner pose threshold.\n";
        } else {
            std::cout << "\nRESULT: too little board visibility; recapture is recommended.\n";
        }
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << "\n";
        return 1;
    }
}
