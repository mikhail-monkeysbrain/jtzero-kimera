#include <libcamera/libcamera.h>

#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <fcntl.h>
#include <unistd.h>
#include <glob.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <set>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <climits>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <numeric>
#include <thread>
#include <vector>

using namespace libcamera;
namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;

static constexpr double MAX_STEREO_DT_MS = 7.0;
static constexpr int TARGET_PAIRS_PER_STAGE = 20;
static constexpr int SAVE_INTERVAL_MS = 150;
static constexpr int WAIT_BETWEEN_SERIES_SEC = 20;
static constexpr int MIN_SHARED_CHARUCO = 8;
static constexpr int BOARD_SQUARES_X = 7;
static constexpr int BOARD_SQUARES_Y = 5;
static constexpr float BOARD_SQUARE_MM = 27.324f;
static constexpr float BOARD_MARKER_MM = 20.043f;

struct UsbFrame {
    uint64_t seq{0};
    int64_t ts_ns{0};
    std::vector<uint8_t> jpeg;
};

struct MMapBuf {
    void *ptr{nullptr};
    size_t len{0};
};

struct StereoPair {
    uint64_t pair_id{0};
    uint64_t usb_seq{0};
    int64_t usb_ts_ns{0};
    int64_t csi_ts_ns{0};
    double dt_ms{0.0};
    cv::Mat left;
    cv::Mat right;
};

static int xioctl(int fd, unsigned long request, void *arg)
{
    int r;
    do {
        r = ::ioctl(fd, request, arg);
    } while (r == -1 && errno == EINTR);
    return r;
}

static std::string nowStamp()
{
    const auto t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    char b[32];
    std::strftime(b, sizeof(b), "%Y%m%d_%H%M%S", &tm);
    return b;
}

static int64_t steadyNs()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        Clock::now().time_since_epoch()).count();
}

static void txt(cv::Mat &img,
                const std::string &s,
                int x,
                int y,
                int px = 22,
                const cv::Scalar &col = cv::Scalar(235,235,235),
                int th = 1)
{
    cv::addText(img, s, {x,y}, "DejaVu Sans", px, col, th, cv::LINE_AA, false);
}

static void panel(cv::Mat &img,
                  int x, int y, int w, int h,
                  const cv::Scalar &fill = cv::Scalar(28,31,37),
                  const cv::Scalar &border = cv::Scalar(75,82,95))
{
    cv::rectangle(img, {x,y}, {x+w,y+h}, fill, cv::FILLED);
    cv::rectangle(img, {x,y}, {x+w,y+h}, border, 2);
}

static cv::Mat fitInto(const cv::Mat &src, int w, int h)
{
    const double scale = std::min(double(w) / src.cols, double(h) / src.rows);
    cv::Mat out;
    cv::resize(src, out, {}, scale, scale, cv::INTER_AREA);
    return out;
}

struct CharucoDetection {
    std::vector<cv::Point2f> corners;
    std::vector<int> ids;
};

class P11StereoLogger {
public:
    ~P11StereoLogger() { cleanup(); }

    bool initialize(const std::string &usb_dev)
    {
        std::string resolved = usb_dev;
        if (resolved == "auto") {
            resolved = findUsbCamera();
            if (resolved.empty()) {
                std::cerr << "[ОШИБКА] OV9281 не найдена среди /dev/video*\n";
                return false;
            }
        }

        std::cout << "[USB] выбрана " << resolved << "\n";

        if (!openUsb(resolved.c_str()))
            return false;
        if (!openCsi())
            return false;

        camera_->requestCompleted.connect(this, &P11StereoLogger::onCsiFrame);
        return true;
    }

    int run()
    {
        const fs::path out =
            fs::path("/home/vio/jtzero_runs") /
            (nowStamp() + "_P11_STEREO_STABILITY_GUI");

        fs::create_directories(out);

        std::ofstream pairs_csv(out / "p11_stereo_pairs.csv");
        std::ofstream events_csv(out / "p11_stereo_events.csv");

        if (!pairs_csv || !events_csv) {
            std::cerr << "[ОШИБКА] Не удалось создать CSV-файлы в " << out << "\n";
            return 2;
        }

        pairs_csv
            << "stage,position,pair_index,pair_id,usb_seq,usb_ts_ns,csi_ts_ns,dt_ms,left_file,right_file\n";
        events_csv
            << "steady_ns,event,stage,position\n";

        running_.store(true);
        usb_thread_ = std::thread(&P11StereoLogger::usbLoop, this);

        if (camera_->start()) {
            std::cerr << "[ОШИБКА] Не удалось запустить OV5647\n";
            return 3;
        }
        camera_started_ = true;

        for (auto &request : requests_) {
            if (camera_->queueRequest(request.get()) < 0) {
                std::cerr << "[ОШИБКА] queueRequest failed\n";
                return 4;
            }
        }

        std::cout << "[СТЕРЕО] ожидание первой синхронной пары...\n";

        const auto wait_begin = Clock::now();
        while (running_.load()) {
            {
                std::lock_guard<std::mutex> lock(pair_mutex_);
                if (latest_pair_.pair_id > 0)
                    break;
            }
            if (std::chrono::duration<double>(Clock::now() - wait_begin).count() > 8.0) {
                std::cerr << "[ОШИБКА] За 8 секунд не получено ни одной синхронной stereo-пары.\n";
                return 5;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        std::cout << "[СТЕРЕО] первая синхронная пара получена\n";

        const std::vector<std::string> stages =
            {"S1","S2","S3","S4"};

        size_t stage_idx = 0;
        bool recording = false;
        bool waiting = false;
        bool started_once = false;
        bool finished = false;
        auto wait_until = Clock::now();
        int saved_in_stage = 0;
        uint64_t last_saved_pair_id = 0;
        auto last_save_tp = Clock::now() - std::chrono::seconds(1);

        const std::string win = "P11: стерео A/B";
        cv::namedWindow(win, cv::WINDOW_NORMAL);
        cv::setWindowProperty(win, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

        while (running_.load()) {
            StereoPair pair;
            {
                std::lock_guard<std::mutex> lock(pair_mutex_);
                if (latest_pair_.pair_id > 0) {
                    pair.pair_id = latest_pair_.pair_id;
                    pair.usb_seq = latest_pair_.usb_seq;
                    pair.usb_ts_ns = latest_pair_.usb_ts_ns;
                    pair.csi_ts_ns = latest_pair_.csi_ts_ns;
                    pair.dt_ms = latest_pair_.dt_ms;
                    pair.left = latest_pair_.left.clone();
                    pair.right = latest_pair_.right.clone();
                }
            }

            if (pair.pair_id == 0 || pair.left.empty() || pair.right.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            const std::string stage = finished ? "S4" : stages[stage_idx];
            const std::string pos = "FIXED";

            const CharucoDetection dl = detectCharuco(pair.left);
            const CharucoDetection dr = detectCharuco(pair.right);
            const int shared = countSharedIds(dl, dr);
            const bool target_ok = shared >= MIN_SHARED_CHARUCO;

            if (recording) {
                const auto now = Clock::now();
                const bool interval_ok =
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        now - last_save_tp).count() >= SAVE_INTERVAL_MS;

                if (interval_ok &&
                    pair.pair_id != last_saved_pair_id &&
                    std::abs(pair.dt_ms) <= MAX_STEREO_DT_MS &&
                    target_ok) {

                    const fs::path stage_dir = out / stage;
                    fs::create_directories(stage_dir);

                    const std::string stem =
                        cv::format("pair_%03d", saved_in_stage + 1);

                    const fs::path left_path =
                        stage_dir / (stem + "_ov9281.png");
                    const fs::path right_path =
                        stage_dir / (stem + "_ov5647.png");

                    const bool ok_l =
                        cv::imwrite(left_path.string(), pair.left);
                    const bool ok_r =
                        cv::imwrite(right_path.string(), pair.right);

                    if (ok_l && ok_r) {
                        ++saved_in_stage;
                        last_saved_pair_id = pair.pair_id;
                        last_save_tp = now;

                        pairs_csv
                            << stage << ","
                            << pos << ","
                            << saved_in_stage << ","
                            << pair.pair_id << ","
                            << pair.usb_seq << ","
                            << pair.usb_ts_ns << ","
                            << pair.csi_ts_ns << ","
                            << std::fixed << std::setprecision(6)
                            << pair.dt_ms << ","
                            << fs::relative(left_path, out).string() << ","
                            << fs::relative(right_path, out).string() << "\n";
                        pairs_csv.flush();
                    }

                    if (saved_in_stage >= TARGET_PAIRS_PER_STAGE) {
                        events_csv
                            << steadyNs() << ",STAGE_COMPLETE,"
                            << stage << "," << pos << "\n";
                        events_csv.flush();

                        recording = false;
                        saved_in_stage = 0;

                        if (stage_idx + 1 >= stages.size()) {
                            finished = true;
                        } else {
                            ++stage_idx;
                            waiting = true;
                            wait_until = Clock::now() + std::chrono::seconds(WAIT_BETWEEN_SERIES_SEC);
                        }
                    }
                }
            }

            cv::Mat ui(720,1280,CV_8UC3,cv::Scalar(20,22,26));

            panel(ui,16,14,1248,82,cv::Scalar(31,35,42));
            txt(ui,"P11 — проверка стабильности стереокалибровки",34,50,28);
            txt(ui,
                "Цель: проверить stereo extrinsics на неподвижной известной ChArUco-мишени",
                34,79,16,cv::Scalar(180,185,195));

            panel(ui,24,116,820,480,cv::Scalar(12,12,12));

            cv::Mat l = fitInto(pair.left,390,360);
            cv::Mat r = fitInto(pair.right,390,360);

            const int ly = 170;
            const int lx = 40;
            const int rx = 442;

            l.copyTo(ui(cv::Rect(lx,ly,l.cols,l.rows)));
            r.copyTo(ui(cv::Rect(rx,ly,r.cols,r.rows)));

            drawCharuco(l, dl);
            drawCharuco(r, dr);
            l.copyTo(ui(cv::Rect(lx,ly,l.cols,l.rows)));
            r.copyTo(ui(cv::Rect(rx,ly,r.cols,r.rows)));
            txt(ui,"ЛЕВАЯ: OV9281",lx,150,18,cv::Scalar(205,210,220));
            txt(ui,"ПРАВАЯ: OV5647",rx,150,18,cv::Scalar(205,210,220));

            txt(ui,
                std::string("ChArUco общих углов: ") + std::to_string(shared) +
                    (target_ok ? "  — ГОТОВО" : "  — НЕДОСТАТОЧНО"),
                40,540,17,
                target_ok ? cv::Scalar(120,235,160) : cv::Scalar(110,160,255));

            txt(ui,
                std::string("Синхронизация: dt = ") +
                    std::string(cv::format("%+.2f мс", pair.dt_ms)),
                40,570,17,
                (std::abs(pair.dt_ms) <= MAX_STEREO_DT_MS)
                    ? cv::Scalar(120,235,160)
                    : cv::Scalar(110,160,255));

            panel(ui,864,116,392,480,cv::Scalar(27,30,36));

            if (finished) {
                txt(ui,"ТЕСТ ЗАВЕРШЁН",892,175,30,cv::Scalar(120,235,160),2);
                txt(ui,"4 серии записаны.",892,235,18);
                txt(ui,"БПЛА и мишень не перемещались.",892,270,16);
                txt(ui,"Нажмите любую клавишу.",892,340,20);
            }
            else if (recording) {
                txt(ui,"ЗАПИСЬ " + stage,892,175,32,cv::Scalar(120,235,160),2);
                txt(ui,"НИЧЕГО НЕ ПЕРЕМЕЩАТЬ",892,235,21,cv::Scalar(110,160,255),2);
                txt(ui,"Стереопар: " + std::to_string(saved_in_stage) +
                        " / " + std::to_string(TARGET_PAIRS_PER_STAGE),
                        892,300,22);
                txt(ui,"Нужно общих ChArUco углов: >= " +
                        std::to_string(MIN_SHARED_CHARUCO),892,345,16);
                if (!target_ok)
                    txt(ui,"ЖДЁМ МИШЕНЬ В ОБЕИХ КАМЕРАХ",892,390,16,cv::Scalar(110,160,255),2);
            }
            else if (waiting) {
                const auto remain = std::chrono::duration_cast<std::chrono::seconds>(
                    wait_until - Clock::now()).count();
                txt(ui,"ПАУЗА МЕЖДУ СЕРИЯМИ",892,175,25,cv::Scalar(120,220,250),2);
                txt(ui,"НИЧЕГО НЕ ТРОГАТЬ",892,235,22,cv::Scalar(110,160,255),2);
                txt(ui,"Следующая серия автоматически через:",892,300,16);
                txt(ui,std::to_string(std::max<int64_t>(0,remain)) + " с",892,355,34,cv::Scalar(245,245,245),2);
                if (Clock::now() >= wait_until) {
                    waiting = false;
                    recording = true;
                    saved_in_stage = 0;
                    last_saved_pair_id = 0;
                    last_save_tp = Clock::now() - std::chrono::seconds(1);
                    events_csv << steadyNs() << ",SERIES_START," << stage << ",FIXED\n";
                    events_csv.flush();
                }
            }
            else {
                txt(ui,"ПОДГОТОВКА",892,165,24,cv::Scalar(160,170,185));
                txt(ui,"1. БПЛА НЕ ДВИГАТЬ.",892,225,19);
                txt(ui,"2. Установите ChArUco-мишень",892,265,17);
                txt(ui,"   так, чтобы её видели ОБЕ камеры.",892,295,17);
                txt(ui,"3. После этого мишень тоже не трогать.",892,335,17);
                txt(ui,"Общих углов сейчас: " + std::to_string(shared),892,390,20,
                    target_ok ? cv::Scalar(120,235,160) : cv::Scalar(110,160,255),2);
                panel(ui,886,445,348,84,
                      target_ok ? cv::Scalar(45,105,70) : cv::Scalar(60,60,65),
                      cv::Scalar(75,145,95));
                txt(ui,target_ok ? "ПРОБЕЛ — НАЧАТЬ 4 СЕРИИ" : "СНАЧАЛА ПОКАЖИТЕ МИШЕНЬ",
                    900,485,16,cv::Scalar(245,245,245),2);
                txt(ui,"После старта ничего не перемещать.",904,515,14);
            }

            panel(ui,24,620,1232,72,cv::Scalar(31,35,42));
            txt(ui,
                "Серии: S1 → 20 с → S2 → 20 с → S3 → 20 с → S4",
                48,653,17,cv::Scalar(180,185,195));
            txt(ui,
                "ESC — аварийный выход",
                1010,653,16,cv::Scalar(120,160,245),2);

            cv::imshow(win,ui);
            const int key = cv::waitKey(5) & 255;

            if (key == 27 || key == 'q' || key == 'Q') {
                events_csv
                    << steadyNs() << ",ABORT,"
                    << (finished ? "DONE" : stage) << ","
                    << (finished ? "-" : pos) << "\n";
                events_csv.flush();
                running_.store(false);
                break;
            }

            if (finished && key >= 0) {
                break;
            }

            if (!finished && !recording && !waiting && !started_once && key == ' ' && target_ok) {
                events_csv
                    << steadyNs() << ",SERIES_START,"
                    << stage << ",FIXED\n";
                events_csv.flush();

                started_once = true;
                recording = true;
                saved_in_stage = 0;
                last_saved_pair_id = 0;
                last_save_tp = Clock::now() - std::chrono::seconds(1);
            }
        }

        cv::destroyAllWindows();

        cleanup();

        std::cout << "[ГОТОВО] " << out << "\n";
        std::cout << "[ПАРЫ] " << (out / "p11_stereo_pairs.csv") << "\n";
        std::cout << "[СОБЫТИЯ] " << (out / "p11_stereo_events.csv") << "\n";
        std::cout << "[СТАТИСТИКА] accepted_sync=" << accepted_pairs_.load()
                  << " rejected_sync=" << rejected_pairs_.load() << "\n";

        return finished ? 0 : 10;
    }

private:
    CharucoDetection detectCharuco(const cv::Mat &bgr)
    {
        CharucoDetection out;
        cv::Mat gray;
        if (bgr.channels()==1) gray=bgr;
        else cv::cvtColor(bgr,gray,cv::COLOR_BGR2GRAY);

        std::vector<std::vector<cv::Point2f>> marker_corners;
        std::vector<int> marker_ids;
        auto params=cv::makePtr<cv::aruco::DetectorParameters>();
        cv::aruco::detectMarkers(gray,charuco_dictionary_,marker_corners,marker_ids,params);
        if (marker_ids.empty()) return out;

        cv::Mat cc,ci;
        const int n=cv::aruco::interpolateCornersCharuco(
            marker_corners,marker_ids,gray,charuco_board_,cc,ci);
        if (n<=0 || ci.empty()) return out;

        for(int i=0;i<n;++i){
            out.corners.push_back(cc.at<cv::Point2f>(i));
            out.ids.push_back(ci.at<int>(i));
        }
        return out;
    }

    static int countSharedIds(const CharucoDetection &a,const CharucoDetection &b)
    {
        std::set<int> ids(a.ids.begin(),a.ids.end());
        int n=0;
        for(int id:b.ids) if(ids.count(id)) ++n;
        return n;
    }

    static void drawCharuco(cv::Mat &img,const CharucoDetection &d)
    {
        for(size_t i=0;i<d.corners.size();++i){
            cv::circle(img,d.corners[i],4,cv::Scalar(0,255,0),1);
            cv::putText(img,std::to_string(d.ids[i]),d.corners[i]+cv::Point2f(4,-4),
                        cv::FONT_HERSHEY_SIMPLEX,0.4,cv::Scalar(0,255,0),1);
        }
    }

    static std::string findUsbCamera()
    {
        glob_t g{};
        if (::glob("/dev/video*",0,nullptr,&g) != 0)
            return {};

        std::string best;

        for (size_t i=0;i<g.gl_pathc;++i) {
            const char *path=g.gl_pathv[i];
            const int fd=::open(path,O_RDWR|O_NONBLOCK);
            if (fd<0) continue;

            v4l2_capability cap{};
            const bool ok=xioctl(fd,VIDIOC_QUERYCAP,&cap)==0;
            ::close(fd);
            if (!ok) continue;

            const std::string card(
                reinterpret_cast<const char*>(cap.card));

            if (card.find("OV9281")!=std::string::npos ||
                card.find("Arducam")!=std::string::npos) {
                best=path;
                break;
            }
        }

        ::globfree(&g);
        return best;
    }

    bool openUsb(const char *dev)
    {
        usb_fd_=::open(dev,O_RDWR|O_NONBLOCK);
        if (usb_fd_<0) {
            perror("open USB");
            return false;
        }

        v4l2_capability cap{};
        if (xioctl(usb_fd_,VIDIOC_QUERYCAP,&cap)<0) {
            perror("VIDIOC_QUERYCAP");
            return false;
        }

        std::cout << "[USB] " << cap.card << "\n";

        v4l2_format fmt{};
        fmt.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width=640;
        fmt.fmt.pix.height=480;
        fmt.fmt.pix.pixelformat=V4L2_PIX_FMT_MJPEG;
        fmt.fmt.pix.field=V4L2_FIELD_ANY;

        if (xioctl(usb_fd_,VIDIOC_S_FMT,&fmt)<0) {
            perror("VIDIOC_S_FMT");
            return false;
        }

        v4l2_streamparm parm{};
        parm.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
        parm.parm.capture.timeperframe.numerator=1;
        parm.parm.capture.timeperframe.denominator=100;
        xioctl(usb_fd_,VIDIOC_S_PARM,&parm);

        v4l2_requestbuffers req{};
        req.count=4;
        req.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory=V4L2_MEMORY_MMAP;

        if (xioctl(usb_fd_,VIDIOC_REQBUFS,&req)<0) {
            perror("VIDIOC_REQBUFS");
            return false;
        }

        usb_buffers_.resize(req.count);

        for (unsigned i=0;i<req.count;++i) {
            v4l2_buffer buf{};
            buf.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory=V4L2_MEMORY_MMAP;
            buf.index=i;

            if (xioctl(usb_fd_,VIDIOC_QUERYBUF,&buf)<0)
                return false;

            usb_buffers_[i].len=buf.length;
            usb_buffers_[i].ptr=::mmap(
                nullptr,buf.length,PROT_READ|PROT_WRITE,
                MAP_SHARED,usb_fd_,buf.m.offset);

            if (usb_buffers_[i].ptr==MAP_FAILED)
                return false;

            if (xioctl(usb_fd_,VIDIOC_QBUF,&buf)<0)
                return false;
        }

        v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (xioctl(usb_fd_,VIDIOC_STREAMON,&type)<0)
            return false;

        usb_streaming_=true;
        return true;
    }

    bool openCsi()
    {
        if (camera_manager_.start())
            return false;
        manager_started_=true;

        for (const auto &cam:camera_manager_.cameras()) {
            if (cam->id().find("ov5647")!=std::string::npos)
                camera_=cam;
        }

        if (!camera_) {
            std::cerr << "[ОШИБКА] OV5647 не найдена libcamera\n";
            return false;
        }

        std::cout << "[CSI] выбрана " << camera_->id() << "\n";

        if (camera_->acquire())
            return false;
        camera_acquired_=true;

        config_=camera_->generateConfiguration({StreamRole::Viewfinder});
        if (!config_ || config_->empty())
            return false;

        auto &sc=config_->at(0);
        sc.size.width=640;
        sc.size.height=480;
        sc.pixelFormat=formats::YUV420;

        if (config_->validate()==CameraConfiguration::Invalid)
            return false;
        if (camera_->configure(config_.get()))
            return false;

        stream_=sc.stream();
        csi_width_=sc.size.width;
        csi_height_=sc.size.height;
        csi_stride_=sc.stride;

        allocator_=std::make_unique<FrameBufferAllocator>(camera_);
        if (allocator_->allocate(stream_)<0)
            return false;

        for (const auto &buffer:allocator_->buffers(stream_)) {
            auto request=camera_->createRequest();
            if (!request || request->addBuffer(stream_,buffer.get())<0)
                return false;
            requests_.push_back(std::move(request));
        }

        return true;
    }

    void usbLoop()
    {
        while (running_.load()) {
            fd_set fds;
            FD_ZERO(&fds);
            FD_SET(usb_fd_,&fds);
            timeval tv{0,200000};

            const int r=::select(usb_fd_+1,&fds,nullptr,nullptr,&tv);
            if (r<=0) continue;

            v4l2_buffer buf{};
            buf.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory=V4L2_MEMORY_MMAP;

            if (xioctl(usb_fd_,VIDIOC_DQBUF,&buf)<0) {
                if (errno==EAGAIN) continue;
                break;
            }

            UsbFrame f;
            f.seq=buf.sequence;
            f.ts_ns=
                int64_t(buf.timestamp.tv_sec)*1000000000LL+
                int64_t(buf.timestamp.tv_usec)*1000LL;
            f.jpeg.resize(buf.bytesused);
            std::memcpy(f.jpeg.data(),
                        usb_buffers_[buf.index].ptr,
                        buf.bytesused);

            {
                std::lock_guard<std::mutex> lock(usb_mutex_);
                usb_history_.push_back(std::move(f));
                while (usb_history_.size()>120)
                    usb_history_.pop_front();
            }

            if (xioctl(usb_fd_,VIDIOC_QBUF,&buf)<0)
                break;
        }
    }

    cv::Mat mapCsiBuffer(FrameBuffer *fb)
    {
        if (!fb || fb->planes().empty())
            return {};

        const auto &plane=fb->planes()[0];
        const int dma_fd=plane.fd.get();

        const size_t map_len=
            static_cast<size_t>(plane.offset)+
            static_cast<size_t>(plane.length);

        if (map_len==0)
            return {};

        void *base=::mmap(nullptr,map_len,PROT_READ,MAP_SHARED,dma_fd,0);
        if (base==MAP_FAILED)
            return {};

        const uint8_t *y=
            static_cast<const uint8_t*>(base)+plane.offset;

        const size_t stride=
            csi_stride_>0 ? static_cast<size_t>(csi_stride_)
                          : static_cast<size_t>(csi_width_);

        const size_t required=
            stride*static_cast<size_t>(csi_height_);

        if (required>plane.length) {
            ::munmap(base,map_len);
            return {};
        }

        cv::Mat gray(csi_height_,
                     csi_width_,
                     CV_8UC1,
                     const_cast<uint8_t*>(y),
                     stride);

        cv::Mat copy=gray.clone();
        ::munmap(base,map_len);

        cv::Mat bgr;
        cv::cvtColor(copy,bgr,cv::COLOR_GRAY2BGR);
        return bgr;
    }

    void onCsiFrame(Request *request)
    {
        if (request->status()==Request::RequestCancelled)
            return;

        int64_t csi_ts_ns=0;
        auto sensor_ts=request->metadata().get(controls::SensorTimestamp);
        if (sensor_ts.has_value())
            csi_ts_ns=sensor_ts.value();
        else if (!request->buffers().empty())
            csi_ts_ns=int64_t(
                request->buffers().begin()->second->metadata().timestamp);

        UsbFrame best;
        bool found=false;
        int64_t best_abs=INT64_MAX;

        {
            std::lock_guard<std::mutex> lock(usb_mutex_);
            for (const auto &u:usb_history_) {
                const int64_t adt=std::llabs(u.ts_ns-csi_ts_ns);
                if (adt<best_abs) {
                    best_abs=adt;
                    best=u;
                    found=true;
                }
            }
        }

        if (found) {
            const double dt_ms=double(best.ts_ns-csi_ts_ns)/1e6;

            if (std::abs(dt_ms)<=MAX_STEREO_DT_MS) {
                auto it=request->buffers().find(stream_);
                if (it!=request->buffers().end()) {
                    cv::Mat right=mapCsiBuffer(it->second);
                    cv::Mat left=cv::imdecode(best.jpeg,cv::IMREAD_COLOR);

                    if (!left.empty() && !right.empty()) {
                        if (left.size()!=right.size())
                            cv::resize(left,left,right.size());

                        StereoPair pair;
                        pair.pair_id=++pair_counter_;
                        pair.usb_seq=best.seq;
                        pair.usb_ts_ns=best.ts_ns;
                        pair.csi_ts_ns=csi_ts_ns;
                        pair.dt_ms=dt_ms;
                        pair.left=left;
                        pair.right=right;

                        {
                            std::lock_guard<std::mutex> lock(pair_mutex_);
                            latest_pair_=pair;
                        }

                        ++accepted_pairs_;
                    }
                }
            } else {
                ++rejected_pairs_;
            }
        }

        if (running_.load()) {
            request->reuse(Request::ReuseBuffers);
            if (camera_->queueRequest(request)<0)
                std::cerr << "[ОШИБКА] queueRequest failed\n";
        }
    }

    void cleanup()
    {
        if (cleaned_)
            return;
        cleaned_=true;

        running_.store(false);

        if (camera_started_ && camera_) {
            camera_->stop();
            camera_started_=false;
        }

        if (usb_thread_.joinable())
            usb_thread_.join();

        if (camera_) {
            camera_->requestCompleted.disconnect(this);
            requests_.clear();

            if (allocator_) {
                allocator_->free(stream_);
                allocator_.reset();
            }

            if (camera_acquired_) {
                camera_->release();
                camera_acquired_=false;
            }

            camera_.reset();
        }

        config_.reset();

        if (manager_started_) {
            camera_manager_.stop();
            manager_started_=false;
        }

        if (usb_streaming_ && usb_fd_>=0) {
            v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
            xioctl(usb_fd_,VIDIOC_STREAMOFF,&type);
            usb_streaming_=false;
        }

        for (auto &b:usb_buffers_) {
            if (b.ptr && b.ptr!=MAP_FAILED)
                ::munmap(b.ptr,b.len);
        }
        usb_buffers_.clear();

        if (usb_fd_>=0) {
            ::close(usb_fd_);
            usb_fd_=-1;
        }
    }

    bool cleaned_{false};

    int usb_fd_{-1};
    bool usb_streaming_{false};
    std::vector<MMapBuf> usb_buffers_;
    std::thread usb_thread_;
    std::mutex usb_mutex_;
    std::deque<UsbFrame> usb_history_;

    CameraManager camera_manager_;
    bool manager_started_{false};
    std::shared_ptr<Camera> camera_;
    bool camera_acquired_{false};
    bool camera_started_{false};
    std::unique_ptr<CameraConfiguration> config_;
    Stream *stream_{nullptr};
    unsigned int csi_width_{0};
    unsigned int csi_height_{0};
    unsigned int csi_stride_{0};
    std::unique_ptr<FrameBufferAllocator> allocator_;
    std::vector<std::unique_ptr<Request>> requests_;

    std::atomic<bool> running_{false};

    std::mutex pair_mutex_;
    StereoPair latest_pair_;
    std::atomic<uint64_t> pair_counter_{0};
    std::atomic<uint64_t> accepted_pairs_{0};
    std::atomic<uint64_t> rejected_pairs_{0};

    const cv::aruco::Dictionary dictionary_value_ =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    const cv::Ptr<cv::aruco::Dictionary> charuco_dictionary_ =
        cv::makePtr<cv::aruco::Dictionary>(dictionary_value_);
    const cv::Ptr<cv::aruco::CharucoBoard> charuco_board_ =
        cv::makePtr<cv::aruco::CharucoBoard>(
            cv::Size(BOARD_SQUARES_X,BOARD_SQUARES_Y),
            BOARD_SQUARE_MM/1000.0f,
            BOARD_MARKER_MM/1000.0f,
            dictionary_value_);
};

int main(int argc,char **argv)
{
    const std::string usb_dev=argc>1 ? argv[1] : "auto";

    std::cout << "========================================\n";
    std::cout << " P11 — stereo calibration stability\n";
    std::cout << "========================================\n";
    std::cout << "Левая камера : OV9281 USB\n";
    std::cout << "Правая камера: OV5647 CSI\n";
    std::cout << "Макс. |dt|   : " << MAX_STEREO_DT_MS << " мс\n";

    P11StereoLogger app;
    if (!app.initialize(usb_dev))
        return 1;

    return app.run();
}
