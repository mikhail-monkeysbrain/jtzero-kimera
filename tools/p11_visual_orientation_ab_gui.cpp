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
    if (fd < 0) return false;

    v4l2_capability caps{};
    const bool ok = (::ioctl(fd, VIDIOC_QUERYCAP, &caps) == 0);
    ::close(fd);
    if (!ok) return false;

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
        const std::string name = e.path().filename().string();
        if (name.rfind("video", 0) == 0) nodes.push_back(e.path());
    }
    std::sort(nodes.begin(), nodes.end());
    for (const auto& dev : nodes) {
        if (isOv9281Node(dev)) return dev.string();
    }
    return {};
}

static void txt(
    cv::Mat& img,
    const std::string& s,
    int x,
    int y,
    int px = 24,
    const cv::Scalar& col = cv::Scalar(235,235,235),
    int th = 1) {
    cv::addText(
        img,
        s,
        {x,y},
        "DejaVu Sans",
        px,
        col,
        th,
        cv::LINE_AA,
        false);
}

static void panel(
    cv::Mat& img,
    int x,
    int y,
    int w,
    int h,
    const cv::Scalar& fill,
    const cv::Scalar& border = cv::Scalar(75,82,95)) {
    cv::rectangle(img, {x,y}, {x+w,y+h}, fill, cv::FILLED);
    cv::rectangle(img, {x,y}, {x+w,y+h}, border, 2);
}

static cv::Mat fitInto(const cv::Mat& src, int w, int h) {
    const double scale =
        std::min(double(w) / src.cols, double(h) / src.rows);
    cv::Mat dst;
    cv::resize(
        src,
        dst,
        {},
        scale,
        scale,
        cv::INTER_AREA);
    return dst;
}

int main(int argc, char** argv) {
    std::string dev = (argc > 1) ? argv[1] : "auto";
    if (dev == "auto") {
        dev = findOv9281();
        if (dev.empty()) {
            std::cerr
                << "[ОШИБКА] Не удалось автоматически найти Arducam OV9281 среди /dev/video*.\n";
            return 1;
        }
    }

    std::cout << "[КАМЕРА] выбрана " << dev << std::endl;

    cv::VideoCapture cap(dev, cv::CAP_V4L2);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(
        cv::CAP_PROP_FOURCC,
        cv::VideoWriter::fourcc('M','J','P','G'));
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
            << "Устройство: " << dev << "\n";
        return 3;
    }

    std::cout
        << "[КАМЕРА] Первый кадр получен: "
        << first.cols << "x" << first.rows
        << std::endl;

    const std::vector<std::string> stages =
        {"A1","B1","A2","B2","A3","B3","A4"};

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

    const int SW = 1280;
    const int SH = 720;
    const int PREVIEW_X = 24;
    const int PREVIEW_Y = 116;
    const int PREVIEW_W = 850;
    const int PREVIEW_H = 516;

    cv::Mat frame = first;

    for (;;) {
        cv::Mat ui(SH, SW, CV_8UC3, cv::Scalar(20,22,26));

        panel(
            ui,
            16,14,SW-32,82,
            cv::Scalar(31,35,42),
            cv::Scalar(65,70,80));

        txt(
            ui,
            "P11 — независимая визуальная проверка A/B",
            34,50,28);

        txt(
            ui,
            "Этап " + std::to_string(stage_idx + 1) + " из 7",
            34,78,18,
            cv::Scalar(180,185,195));

        for (int k = 0; k < 7; ++k) {
            const int x = 610 + k * 86;
            cv::Scalar fill =
                (k < static_cast<int>(stage_idx))
                    ? cv::Scalar(55,115,70)
                    : (k == static_cast<int>(stage_idx)
                        ? cv::Scalar(60,95,170)
                        : cv::Scalar(48,52,60));
            panel(
                ui,
                x,31,68,38,
                fill,
                cv::Scalar(90,100,115));
            txt(ui, stages[k], x+18,58,21);
        }

        panel(
            ui,
            PREVIEW_X,PREVIEW_Y,PREVIEW_W,PREVIEW_H,
            cv::Scalar(10,10,10),
            cv::Scalar(75,82,95));

        cv::Mat preview =
            fitInto(frame, PREVIEW_W-8, PREVIEW_H-8);

        const int px =
            PREVIEW_X + (PREVIEW_W - preview.cols) / 2;
        const int py =
            PREVIEW_Y + (PREVIEW_H - preview.rows) / 2;

        preview.copyTo(
            ui(cv::Rect(px, py, preview.cols, preview.rows)));

        panel(
            ui,
            896,116,360,516,
            cv::Scalar(27,30,36),
            cv::Scalar(75,82,95));

        const std::string& st = stages[stage_idx];

        txt(
            ui,
            "СЕЙЧАС",
            920,154,20,
            cv::Scalar(160,170,185));

        txt(
            ui,
            "ЗАПИСЬ " + st,
            920,214,42,
            cv::Scalar(250,250,250),
            2);

        if (recording) {
            txt(
                ui,
                "ИДЁТ ЗАПИСЬ",
                920,272,25,
                cv::Scalar(100,215,255),
                2);

            txt(
                ui,
                "Не перемещайте БПЛА",
                920,318,22,
                cv::Scalar(100,120,255),
                2);

            txt(
                ui,
                "Кадров: " + std::to_string(saved) + " / 20",
                920,378,28,
                cv::Scalar(120,220,250),
                2);
        } else {
            const std::string physical =
                (st[0] == 'A') ? "A" : "B";
            const std::string previous =
                (stage_idx == 0) ? "" :
                ((stages[stage_idx - 1][0] == 'A') ? "A" : "B");

            if (stage_idx == 0) {
                txt(ui, "УСТАНОВИТЕ БПЛА", 920,270,22);
                txt(ui, "В ОТМЕЧЕННУЮ ТОЧКУ A", 920,304,20,
                    cv::Scalar(120,220,250), 2);
            } else {
                txt(ui, "ПЕРЕМЕСТИТЕ БПЛА", 920,270,22);
                txt(ui, "ИЗ ТОЧКИ " + previous + " В ТОЧКУ " + physical,
                    920,304,20, cv::Scalar(120,220,250), 2);
            }

            txt(
                ui,
                "После установки отпустите",
                920,350,19);

            txt(
                ui,
                "БПЛА и дождитесь покоя.",
                920,382,19);

            panel(
                ui,
                916,430,320,72,
                cv::Scalar(45,105,70),
                cv::Scalar(75,145,95));

            txt(
                ui,
                "ПРОБЕЛ — записать",
                938,462,20,
                cv::Scalar(245,245,245),
                2);

            txt(
                ui,
                "20 кадров",
                938,490,20,
                cv::Scalar(245,245,245),
                2);
        }

        cv::line(
            ui,
            {920,530},
            {1230,530},
            cv::Scalar(70,75,85),
            1);

        txt(
            ui,
            "Источник:",
            920,565,18,
            cv::Scalar(165,170,180));

        txt(
            ui,
            "OV9281, без FC IMU",
            920,594,19,
            cv::Scalar(205,210,220));

        panel(
            ui,
            24,650,1232,52,
            cv::Scalar(31,35,42),
            cv::Scalar(65,70,80));

        txt(
            ui,
            "ПРОБЕЛ — запись",
            48,684,20,
            cv::Scalar(120,235,160),
            2);

        txt(
            ui,
            "ESC — выход",
            330,684,20,
            cv::Scalar(120,160,245),
            2);

        txt(
            ui,
            "A1 → B1 → A2 → B2 → A3 → B3 → A4",
            620,684,18,
            cv::Scalar(175,180,190));

        cv::imshow(win, ui);

        const int key = cv::waitKey(1) & 255;
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
                return 5;
            }

            ev << "FRAME," << st << "," << fn << "\n";
            ++saved;

            if (saved == 20) {
                ev << "END," << st << ",\n";
                ev.flush();
                recording = false;

                if (stage_idx + 1 < stages.size()) {
                    ++stage_idx;
                    saved = 0;
                } else {
                    cv::Mat done(
                        SH,SW,CV_8UC3,
                        cv::Scalar(20,22,26));

                    txt(
                        done,
                        "ТЕСТ ЗАВЕРШЕН",
                        410,300,46,
                        cv::Scalar(120,235,160),
                        2);

                    txt(
                        done,
                        "Все 7 подтверждённых этапов записаны.",
                        420,360,28,
                        cv::Scalar(245,245,245),
                        2);

                    txt(
                        done,
                        "Нажмите любую клавишу.",
                        430,410,24,
                        cv::Scalar(200,205,215));

                    cv::imshow(win, done);
                    cv::waitKey(0);
                    break;
                }
            }
        }

        if (!cap.read(frame) || frame.empty()) {
            std::cerr
                << "[ОШИБКА] Поток камеры остановился во время работы.\n";
            return 6;
        }
    }

    std::cout << "[ГОТОВО] " << out << "\n";
    return 0;
}
