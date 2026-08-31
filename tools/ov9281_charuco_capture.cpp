#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace fs = std::filesystem;

namespace {
constexpr int kWidth = 640;
constexpr int kHeight = 480;
constexpr int kCaptureFps = 120;
constexpr int kSquaresX = 7;
constexpr int kSquaresY = 5;
constexpr float kSquareLengthMm = 27.324F;
constexpr float kMarkerLengthMm = 20.043F;
constexpr int kMinCharucoCorners = 12;
constexpr double kMinSaveIntervalSec = 0.35;

std::string frameName(int index) {
    std::ostringstream ss;
    ss << "frame_" << std::setw(3) << std::setfill('0') << index << ".png";
    return ss.str();
}
}

int main(int argc, char** argv) {
    const std::string device = argc > 1 ? argv[1] : "/dev/video0";
    const fs::path out_dir = argc > 2 ? argv[2] : "/home/vio/charuco_calibration";
    fs::create_directories(out_dir);

    cv::VideoCapture cap(device, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        std::cerr << "ERROR: cannot open " << device << '\n';
        return 1;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, kWidth);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, kHeight);
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    cap.set(cv::CAP_PROP_FPS, kCaptureFps);

    const int width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    const int height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    const double fps = cap.get(cv::CAP_PROP_FPS);
    if (width != kWidth || height != kHeight) {
        std::cerr << "ERROR: camera mode is " << width << 'x' << height
                  << ", expected 640x480\n";
        return 2;
    }

    auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::CharucoBoard board(cv::Size(kSquaresX, kSquaresY),
                                  kSquareLengthMm,
                                  kMarkerLengthMm,
                                  dictionary);
    cv::aruco::CharucoDetector detector(board);

    std::cout << "=== OV9281 CHARUCO CAPTURE ===\n"
              << "device: " << device << '\n'
              << "mode: " << width << 'x' << height << " MJPEG @ " << fps << " FPS\n"
              << "board: 7x5, DICT_4X4_50\n"
              << "squareLength: " << kSquareLengthMm << " mm\n"
              << "markerLength: " << kMarkerLengthMm << " mm\n"
              << "output: " << out_dir << '\n'
              << "SPACE = save detected frame, Q/ESC = quit\n";

    int saved = 0;
    auto last_save = std::chrono::steady_clock::now() - std::chrono::seconds(1);

    for (;;) {
        cv::Mat frame;
        if (!cap.read(frame) || frame.empty()) {
            std::cerr << "WARN: frame capture failed\n";
            continue;
        }

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        cv::Mat charuco_corners, charuco_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;
        std::vector<int> marker_ids;
        detector.detectBoard(gray, charuco_corners, charuco_ids,
                             marker_corners, marker_ids);

        cv::Mat view = frame.clone();
        if (!marker_ids.empty()) {
            cv::aruco::drawDetectedMarkers(view, marker_corners, marker_ids);
        }
        const int corner_count = charuco_ids.empty() ? 0 : charuco_ids.total();
        if (corner_count > 0) {
            cv::aruco::drawDetectedCornersCharuco(view, charuco_corners,
                                                   charuco_ids,
                                                   cv::Scalar(0, 255, 0));
        }

        const bool good = corner_count >= kMinCharucoCorners;
        std::ostringstream status;
        status << "corners=" << corner_count << " saved=" << saved
               << (good ? " READY" : " MOVE BOARD");
        cv::putText(view, status.str(), cv::Point(12, 28),
                    cv::FONT_HERSHEY_SIMPLEX, 0.65,
                    good ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);
        cv::putText(view, "SPACE save | Q/ESC quit", cv::Point(12, 456),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 255), 1);

        cv::imshow("OV9281 ChArUco calibration capture", view);
        const int key = cv::waitKey(1) & 0xff;
        if (key == 27 || key == 'q' || key == 'Q') break;

        if (key == ' ' && good) {
            const auto now = std::chrono::steady_clock::now();
            const double since_last = std::chrono::duration<double>(now - last_save).count();
            if (since_last >= kMinSaveIntervalSec) {
                ++saved;
                const fs::path image_path = out_dir / frameName(saved);
                if (!cv::imwrite(image_path.string(), gray)) {
                    std::cerr << "ERROR: cannot save " << image_path << '\n';
                    return 3;
                }
                last_save = now;
                std::cout << "SAVE " << saved << ": " << image_path
                          << " corners=" << corner_count << '\n';
            }
        }
    }

    cv::destroyAllWindows();
    std::cout << "Saved frames: " << saved << '\n';
    if (saved < 30) {
        std::cout << "WARNING: collect at least 40-60 geometrically diverse frames.\n";
    }
    return 0;
}
