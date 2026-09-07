#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
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
using Clock = std::chrono::steady_clock;

static std::string nowStamp() {
    const auto t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    char b[32];
    std::strftime(b, sizeof(b), "%Y%m%d_%H%M%S", &tm);
    return b;
}

static long long nowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        Clock::now().time_since_epoch()).count();
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
    cv::resize(src, dst, {}, scale, scale, cv::INTER_AREA);
    return dst;
}

enum class State {
    WAIT_A_CONFIRM,
    PRE_STILL,
    MOVE_A_TO_B,
    POST_STILL,
    DONE
};

static const char* stateName(State s) {
    switch (s) {
        case State::WAIT_A_CONFIRM: return "WAIT_A_CONFIRM";
        case State::PRE_STILL: return "PRE_STILL_A";
        case State::MOVE_A_TO_B: return "MOVE_A_TO_B";
        case State::POST_STILL: return "POST_STILL_B";
        case State::DONE: return "DONE";
    }
    return "UNKNOWN";
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

    cv::Mat frame;
    if (!cap.read(frame) || frame.empty()) {
        std::cerr << "[ОШИБКА] Камера открылась, но первый кадр не получен.\n";
        return 3;
    }

    std::cout
        << "[КАМЕРА] Первый кадр получен: "
        << frame.cols << "x" << frame.rows
        << std::endl;

    const fs::path out =
        fs::path("/home/vio/jtzero_runs") /
        (nowStamp() + "_P11_RULER_PASS_GUI");
    fs::create_directories(out);

    const fs::path videoPath = out / "ov9281_ruler_pass.avi";
    const fs::path csvPath = out / "p11_ruler_pass_frames.csv";
    const fs::path eventsPath = out / "p11_ruler_pass_events.csv";

    cv::VideoWriter writer(
        videoPath.string(),
        cv::VideoWriter::fourcc('M','J','P','G'),
        100.0,
        cv::Size(640,480),
        true);

    if (!writer.isOpened()) {
        std::cerr
            << "[ОШИБКА] Не удалось открыть video writer: "
            << videoPath << "\n";
        return 4;
    }

    std::ofstream framesCsv(csvPath);
    std::ofstream events(eventsPath);

    if (!framesCsv || !events) {
        std::cerr << "[ОШИБКА] Не удалось создать CSV-файлы.\n";
        return 5;
    }

    framesCsv << "frame_id,steady_ns,state\n";
    events << "steady_ns,event,state\n";

    const std::string win =
        "P11: проход A → B вдоль направляющей";

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

    State state = State::WAIT_A_CONFIRM;
    Clock::time_point stateStart = Clock::now();
    bool testStarted = false;
    int frameId = 0;

    auto enterState = [&](State next, const std::string& eventName) {
        state = next;
        stateStart = Clock::now();
        events
            << nowNs() << ","
            << eventName << ","
            << stateName(state) << "\n";
        events.flush();
    };

    for (;;) {
        const auto now = Clock::now();
        const double stateSec =
            std::chrono::duration<double>(now - stateStart).count();

        if (state == State::PRE_STILL && stateSec >= 3.0) {
            enterState(State::MOVE_A_TO_B, "PRE_STILL_COMPLETE");
        }

        if (state == State::POST_STILL && stateSec >= 3.0) {
            enterState(State::DONE, "POST_STILL_COMPLETE");
        }

        if (testStarted && state != State::DONE) {
            writer.write(frame);
            framesCsv
                << frameId << ","
                << nowNs() << ","
                << stateName(state) << "\n";
            ++frameId;
        }

        cv::Mat ui(SH, SW, CV_8UC3, cv::Scalar(20,22,26));

        panel(
            ui,
            16,14,SW-32,82,
            cv::Scalar(31,35,42),
            cv::Scalar(65,70,80));

        txt(
            ui,
            "P11 — проход A → B вдоль направляющей",
            34,50,28);

        txt(
            ui,
            "Цель: проверить, меняется ли геометрия стенда относительно рулетки при движении",
            34,78,16,
            cv::Scalar(180,185,195));

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

        if (state == State::WAIT_A_CONFIRM) {
            txt(ui, "ШАГ 1 ИЗ 3", 920,154,20, cv::Scalar(160,170,185));
            txt(ui, "ТОЧКА A", 920,214,40, cv::Scalar(250,250,250), 2);

            txt(ui, "1. Установите стенд", 920,274,20);
            txt(ui, "   в отмеченную точку A.", 920,306,18);
            txt(ui, "2. Рулетку не двигайте.", 920,348,18);
            txt(ui, "3. Отпустите стенд", 920,390,18);
            txt(ui, "   и дождитесь покоя.", 920,420,18);

            panel(
                ui,
                916,456,320,78,
                cv::Scalar(45,105,70),
                cv::Scalar(75,145,95));

            txt(
                ui,
                "ПРОБЕЛ — НАЧАТЬ",
                950,490,20,
                cv::Scalar(245,245,245),
                2);

            txt(
                ui,
                "После нажатия: 3 с покоя",
                934,520,15,
                cv::Scalar(220,225,230));
        }
        else if (state == State::PRE_STILL) {
            const double left = std::max(0.0, 3.0 - stateSec);

            txt(ui, "ШАГ 1 ИЗ 3", 920,154,20, cv::Scalar(160,170,185));
            txt(ui, "ЗАПИСЬ В ТОЧКЕ A", 920,214,28, cv::Scalar(120,235,160), 2);
            txt(ui, "СТЕНД НЕ ПЕРЕМЕЩАТЬ", 920,282,21, cv::Scalar(110,160,255), 2);
            txt(ui, "Осталось:", 920,350,20);
            txt(
                ui,
                cv::format("%.1f с", left),
                920,410,42,
                cv::Scalar(120,220,250),
                2);
            txt(ui, "Дальше GUI сам разрешит движение.", 920,490,15);
        }
        else if (state == State::MOVE_A_TO_B) {
            txt(ui, "ШАГ 2 ИЗ 3", 920,154,20, cv::Scalar(160,170,185));
            txt(ui, "ДВИЖЕНИЕ A → B", 920,214,30, cv::Scalar(120,220,250), 2);

            txt(ui, "ПЛАВНО ПЕРЕМЕЩАЙТЕ", 920,274,19, cv::Scalar(245,245,245), 2);
            txt(ui, "СТЕНД ВДОЛЬ РУЛЕТКИ", 920,306,19, cv::Scalar(245,245,245), 2);
            txt(ui, "ИЗ A В B", 920,338,24, cv::Scalar(120,220,250), 2);

            txt(ui, "Не меняйте угол специально.", 920,390,17);
            txt(ui, "Не приподнимайте стенд.", 920,420,17);

            panel(
                ui,
                916,456,320,78,
                cv::Scalar(55,90,150),
                cv::Scalar(90,125,190));

            txt(
                ui,
                "В B: ОТПУСТИТЕ СТЕНД",
                930,486,16,
                cv::Scalar(245,245,245),
                2);

            txt(
                ui,
                "и нажмите ПРОБЕЛ",
                972,516,18,
                cv::Scalar(245,245,245),
                2);
        }
        else if (state == State::POST_STILL) {
            const double left = std::max(0.0, 3.0 - stateSec);

            txt(ui, "ШАГ 3 ИЗ 3", 920,154,20, cv::Scalar(160,170,185));
            txt(ui, "ЗАПИСЬ В ТОЧКЕ B", 920,214,28, cv::Scalar(120,235,160), 2);
            txt(ui, "СТЕНД НЕ ПЕРЕМЕЩАТЬ", 920,282,21, cv::Scalar(110,160,255), 2);
            txt(ui, "Осталось:", 920,350,20);
            txt(
                ui,
                cv::format("%.1f с", left),
                920,410,42,
                cv::Scalar(120,220,250),
                2);
            txt(ui, "После этого тест завершится сам.", 920,490,15);
        }
        else {
            txt(ui, "ТЕСТ ЗАВЕРШЕН", 920,214,30, cv::Scalar(120,235,160), 2);
            txt(ui, "Записано исходное видео OV9281", 920,282,17);
            txt(ui, "и временные метки этапов.", 920,314,17);
            txt(ui, "Нажмите любую клавишу.", 920,390,20);
        }

        panel(
            ui,
            24,650,1232,52,
            cv::Scalar(31,35,42),
            cv::Scalar(65,70,80));

        txt(
            ui,
            "Рулетка остаётся направляющей и НЕ перемещается",
            48,684,17,
            cv::Scalar(180,185,195));

        txt(
            ui,
            "ESC — выход",
            1080,684,17,
            cv::Scalar(120,160,245),
            2);

        cv::imshow(win, ui);

        const int key = cv::waitKey(1) & 255;

        if (key == 27 || key == 'q' || key == 'Q') {
            events << nowNs() << ",ABORT," << stateName(state) << "\n";
            events.flush();
            break;
        }

        if (state == State::WAIT_A_CONFIRM && key == ' ') {
            testStarted = true;
            enterState(State::PRE_STILL, "START_CONFIRMED_A");
        }
        else if (state == State::MOVE_A_TO_B && key == ' ') {
            enterState(State::POST_STILL, "ARRIVED_B_CONFIRMED");
        }
        else if (state == State::DONE && key >= 0) {
            break;
        }

        if (!cap.read(frame) || frame.empty()) {
            std::cerr
                << "[ОШИБКА] Поток камеры остановился во время работы.\n";
            return 6;
        }
    }

    writer.release();
    framesCsv.flush();
    events.flush();

    std::cout << "[ГОТОВО] " << out << "\n";
    std::cout << "[ВИДЕО] " << videoPath << "\n";
    std::cout << "[КАДРЫ] " << framesCsv << "\n";
    std::cout << "[СОБЫТИЯ] " << eventsPath << "\n";

    return 0;
}
