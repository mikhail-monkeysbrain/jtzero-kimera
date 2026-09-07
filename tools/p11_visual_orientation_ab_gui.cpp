#include <opencv2/opencv.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <ctime>

namespace fs = std::filesystem;

static std::string nowStamp() {
    const auto t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    char b[32];
    std::strftime(b, sizeof(b), "%Y%m%d_%H%M%S", &tm);
    return b;
}

int main(int argc, char** argv) {
    const int cam = argc > 1 ? std::atoi(argv[1]) : 0;

    cv::VideoCapture cap(cam, cv::CAP_V4L2);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    cap.set(cv::CAP_PROP_FPS, 100);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);

    if (!cap.isOpened()) {
        std::cerr << "[ОШИБКА] OV9281 не открыта: /dev/video" << cam << "\n";
        return 1;
    }

    std::cout << "[КАМЕРА] OV9281 открыта, формат 640x480 MJPG. Ожидание первого кадра..." << std::endl;

    cv::Mat first;
    if (!cap.read(first) || first.empty()) {
        std::cerr << "[ОШИБКА] Камера открылась, но первый кадр не получен.\n"
                  << "Проверьте номер /dev/video, занятость камеры и V4L2-поток.\n";
        return 2;
    }

    std::cout << "[КАМЕРА] Первый кадр получен: "
              << first.cols << "x" << first.rows << std::endl;

    const std::vector<std::string> stages = {"A1","B1","A2","B2","A3","B3","A4"};
    size_t stage_idx = 0;
    bool recording = false;
    int saved = 0;

    const fs::path out =
        fs::path("/home/vio/jtzero_runs") /
        (nowStamp() + "_P11_VISUAL_ORIENTATION_AB_GUI");

    fs::create_directories(out);

    std::ofstream ev(out / "p11_visual_events.csv");
    if (!ev) {
        std::cerr << "[ОШИБКА] Не удалось создать журнал в " << out << "\n";
        return 3;
    }
    ev << "event,stage,file\n";

    const std::string win = "P11: независимая визуальная проверка A/B";
    cv::namedWindow(win, cv::WINDOW_NORMAL);
    cv::setWindowProperty(win, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

    cv::Mat frame = first;

    for (;;) {
        if (frame.empty()) {
            if (!cap.read(frame) || frame.empty()) {
                std::cerr << "[ОШИБКА] Поток OV9281 остановился.\n";
                return 4;
            }
        }

        cv::Mat view = frame.clone();
        cv::rectangle(view, {0,0}, {view.cols,145}, cv::Scalar(0,0,0), cv::FILLED);

        const std::string& st = stages[stage_idx];

        cv::putText(view,
                    "P11 — ВНЕШНЯЯ ПРОВЕРКА ОРИЕНТАЦИИ",
                    {20,35},
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.72,
                    cv::Scalar(255,255,255),
                    2);

        cv::putText(view,
                    "Позиция: " + st + "   Кадров: " + std::to_string(saved) + "/20",
                    {20,72},
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.68,
                    cv::Scalar(255,255,255),
                    2);

        cv::putText(view,
                    recording
                        ? "ИДЕТ ЗАПИСЬ — не перемещайте БПЛА"
                        : "Установите БПЛА в " + st + " и нажмите ПРОБЕЛ",
                    {20,108},
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.62,
                    recording ? cv::Scalar(80,220,80) : cv::Scalar(0,220,255),
                    2);

        cv::putText(view,
                    "ПРОБЕЛ — запись 20 кадров   Q/ESC — выход",
                    {20,137},
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.50,
                    cv::Scalar(210,210,210),
                    1);

        cv::imshow(win, view);

        const int key = cv::waitKey(1);
        if (key == 27 || key == 'q' || key == 'Q')
            break;

        if (key == ' ' && !recording) {
            recording = true;
            saved = 0;
            ev << "START," << st << ",\n";
            ev.flush();
        }

        if (recording && saved < 20) {
            const std::string fn = st + "_" + cv::format("%02d", saved) + ".png";

            if (!cv::imwrite((out / fn).string(), frame)) {
                std::cerr << "[ОШИБКА] Не удалось сохранить " << fn << "\n";
                return 5;
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
                    cv::Mat done(480, 900, CV_8UC3, cv::Scalar(0,0,0));
                    cv::putText(done,
                                "ТЕСТ ЗАВЕРШЕН",
                                {170,180},
                                cv::FONT_HERSHEY_SIMPLEX,
                                1.3,
                                cv::Scalar(80,220,80),
                                3);
                    cv::putText(done,
                                "Все 7 позиций записаны. Нажмите любую клавишу.",
                                {55,250},
                                cv::FONT_HERSHEY_SIMPLEX,
                                0.65,
                                cv::Scalar(255,255,255),
                                2);
                    cv::imshow(win, done);
                    cv::waitKey(0);
                    break;
                }
            }
        }

        if (!cap.read(frame) || frame.empty()) {
            std::cerr << "[ОШИБКА] Поток OV9281 остановился во время работы.\n";
            return 6;
        }
    }

    std::cout << "[ГОТОВО] " << out << "\n";
    return 0;
}
