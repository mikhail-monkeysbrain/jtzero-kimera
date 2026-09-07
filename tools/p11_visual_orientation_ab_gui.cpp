#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cctype>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace fs = std::filesystem;

static std::string nowStamp() {
    const auto t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);

    char b[32];
    std::strftime(b, sizeof(b), "%Y%m%d_%H%M%S", &tm);
    return b;
}

static std::string lowerCopy(std::string s) {
    std::transform(
        s.begin(), s.end(), s.begin(),
        [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return s;
}

static bool isOv9281Node(const fs::path& dev) {
    const int fd = ::open(dev.c_str(), O_RDWR | O_NONBLOCK);
    if (fd < 0) {
        return false;
    }

    v4l2_capability caps{};
    const bool ok = (::ioctl(fd, VIDIOC_QUERYCAP, &caps) == 0);
    ::close(fd);

    if (!ok) {
        return false;
    }

    const std::string card =
        lowerCopy(reinterpret_cast<const char*>(caps.card));
    const std::string bus =
        lowerCopy(reinterpret_cast<const char*>(caps.bus_info));

    return card.find("ov9281") != std::string::npos ||
           card.find("arducam") != std::string::npos ||
           bus.find("arducam") != std::string::npos;
}

static std::string findOv9281() {
    std::vector<fs::path> nodes;

    for (const auto& e : fs::directory_iterator("/dev")) {
        if (!e.is_character_file() && !e.is_symlink()) {
            continue;
        }

        const std::string name = e.path().filename().string();
        if (name.rfind("video", 0) == 0) {
            nodes.push_back(e.path());
        }
    }

    std::sort(nodes.begin(), nodes.end());

    for (const auto& dev : nodes) {
        if (isOv9281Node(dev)) {
            return dev.string();
        }
    }

    return {};
}

int main(int argc, char** argv) {
    std::string dev = (argc > 1) ? argv[1] : "auto";

    if (dev == "auto") {
        dev = findOv9281();
        if (dev.empty()) {
            std::cerr
                << "[ОШИБКА] Не удалось автоматически найти Arducam OV9281 "
                << "среди /dev/video*.\n";
            return 1;
        }
    }

    std::cout << "[КАМЕРА] выбрана " << dev << std::endl;

    cv::VideoCapture cap(dev, cv::CAP_V4L2);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(
        cv::CAP_PROP_FOURCC,
        cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(cv::CAP_PROP_FPS, 100);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);

    if (!cap.isOpened()) {
        std::cerr << "[ОШИБКА] Не удалось открыть " << dev << "\n";
        return 2;
    }

    std::cout
        << "[КАМЕРА] поток открыт. Ожидание первого кадра..."
        << std::endl;

    cv::Mat first;
    if (!cap.read(first) || first.empty()) {
        std::cerr
            << "[ОШИБКА] Камера открылась, но первый кадр не получен.\n"
            << "Устройство: " << dev << "\n"
            << "Проверьте, не занята ли камера другим процессом.\n";
        return 3;
    }

    std::cout
        << "[КАМЕРА] Первый кадр получен: "
        << first.cols << "x" << first.rows
        << std::endl;

    const std::vector<std::string> stages = {
        "A1", "B1", "A2", "B2", "A3", "B3", "A4"};

    size_t stage_idx = 0;
    bool recording = false;
    int saved = 0;

    const fs::path out =
        fs::path("/home/vio/jtzero_runs") /
        (nowStamp() + "_P11_VISUAL_ORIENTATION_AB_GUI");

    fs::create_directories(out);

    std::ofstream ev(out / "p11_visual_events.csv");
    if (!ev) {
        std::cerr
            << "[ОШИБКА] Не удалось создать журнал в "
            << out << "\n";
        return 4;
    }

    ev << "event,stage,file\n";

    const std::string win =
        "P11: независимая визуальная проверка A/B";

    cv::namedWindow(win, cv::WINDOW_NORMAL);
    cv::setWindowProperty(
        win,
        cv::WND_PROP_FULLSCREEN,
        cv::WINDOW_FULLSCREEN);

    cv::Mat frame = first;

    for (;;) {
        if (frame.empty()) {
            if (!cap.read(frame) || frame.empty()) {
                std::cerr
                    << "[ОШИБКА] Поток камеры остановился.\n";
                return 5;
            }
        }

        cv::Mat view = frame.clone();

        cv::rectangle(
            view,
            {0, 0},
            {view.cols, 145},
            cv::Scalar(0, 0, 0),
            cv::FILLED);

        const std::string& st = stages[stage_idx];

        cv::putText(
            view,
            "P11 — ВНЕШНЯЯ ПРОВЕРКА ОРИЕНТАЦИИ",
            {20, 35},
            cv::FONT_HERSHEY_SIMPLEX,
            0.72,
            cv::Scalar(255, 255, 255),
            2);

        cv::putText(
            view,
            "Позиция: " + st +
                "   Кадров: " + std::to_string(saved) + "/20",
            {20, 72},
            cv::FONT_HERSHEY_SIMPLEX,
            0.68,
            cv::Scalar(255, 255, 255),
            2);

        cv::putText(
            view,
            recording
                ? "ИДЕТ ЗАПИСЬ — не перемещайте БПЛА"
                : "Установите БПЛА в " + st +
                      " и нажмите ПРОБЕЛ",
            {20, 108},
            cv::FONT_HERSHEY_SIMPLEX,
            0.62,
            recording
                ? cv::Scalar(80, 220, 80)
                : cv::Scalar(0, 220, 255),
            2);

        cv::putText(
            view,
            "ПРОБЕЛ — запись 20 кадров   Q/ESC — выход",
            {20, 137},
            cv::FONT_HERSHEY_SIMPLEX,
            0.50,
            cv::Scalar(210, 210, 210),
            1);

        cv::imshow(win, view);

        const int key = cv::waitKey(1);

        if (key == 27 || key == 'q' || key == 'Q') {
            break;
        }

        if (key == ' ' && !recording) {
            recording = true;
            saved = 0;

            ev << "START," << st << ",\n";
            ev.flush();
        }

        if (recording && saved < 20) {
            const std::string fn =
                st + "_" + cv::format("%02d", saved) + ".png";

            if (!cv::imwrite((out / fn).string(), frame)) {
                std::cerr
                    << "[ОШИБКА] Не удалось сохранить "
                    << fn << "\n";
                return 6;
            }

            ev << "FRAME," << st << "," << fn << "\n";
            ++saved;

            cv::waitKey(45);

            if (saved == 20) {
                ev << "END," << st << ",\n";
                ev.flush();

                recording = false;

                if (stage_idx + 1 < stages.size()) {
                    ++stage_idx;
                    saved = 0;
                } else {
                    cv::Mat done(
                        480,
                        900,
                        CV_8UC3,
                        cv::Scalar(0, 0, 0));

                    cv::putText(
                        done,
                        "ТЕСТ ЗАВЕРШЕН",
                        {170, 180},
                        cv::FONT_HERSHEY_SIMPLEX,
                        1.3,
                        cv::Scalar(80, 220, 80),
                        3);

                    cv::putText(
                        done,
                        "Все 7 позиций записаны. "
                        "Нажмите любую клавишу.",
                        {55, 250},
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.65,
                        cv::Scalar(255, 255, 255),
                        2);

                    cv::imshow(win, done);
                    cv::waitKey(0);
                    break;
                }
            }
        }

        if (!cap.read(frame) || frame.empty()) {
            std::cerr
                << "[ОШИБКА] Поток камеры остановился "
                << "во время работы.\n";
            return 7;
        }
    }

    std::cout << "[ГОТОВО] " << out << "\n";
    return 0;
}
