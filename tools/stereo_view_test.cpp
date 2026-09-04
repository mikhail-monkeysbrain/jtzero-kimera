#include <libcamera/libcamera.h>

#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <fcntl.h>
#include <unistd.h>

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

using namespace libcamera;

static constexpr double MAX_STEREO_DT_MS = 7.0;

struct UsbFrame {
    uint64_t seq{0};
    int64_t ts_ns{0};
    std::vector<uint8_t> jpeg;
};

struct MMapBuf {
    void *ptr{nullptr};
    size_t len{0};
};

static int xioctl(int fd, unsigned long request, void *arg)
{
    int r;
    do {
        r = ::ioctl(fd, request, arg);
    } while (r == -1 && errno == EINTR);
    return r;
}

class StereoViewTest {
public:
    ~StereoViewTest() { cleanup(); }

    bool initialize(const char *usb_dev)
    {
        if (!openUsb(usb_dev))
            return false;

        if (!openCsi())
            return false;

        camera_->requestCompleted.connect(this, &StereoViewTest::onCsiFrame);
        return true;
    }

    int run()
    {
        running_.store(true);
        usb_thread_ = std::thread(&StereoViewTest::usbLoop, this);

        if (camera_->start()) {
            std::cerr << "[CSI] camera start failed\n";
            return 1;
        }
        camera_started_ = true;

        for (auto &request : requests_) {
            if (camera_->queueRequest(request.get()) < 0) {
                std::cerr << "[CSI] initial queueRequest failed\n";
                return 1;
            }
        }

        std::cout << "\n[VIEW] Press Q or ESC to quit\n\n";

        cv::namedWindow("JT-Zero Stereo View", cv::WINDOW_NORMAL);

        while (running_.load()) {
            cv::Mat composed;
            {
                std::lock_guard<std::mutex> lock(display_mutex_);
                if (!latest_display_.empty())
                    composed = latest_display_.clone();
            }

            if (!composed.empty())
                cv::imshow("JT-Zero Stereo View", composed);

            const int key = cv::waitKey(10);
            if (key == 27 || key == 'q' || key == 'Q')
                running_.store(false);
        }

        cv::destroyAllWindows();
        cleanup();

        std::cout << "\nAccepted pairs : " << accepted_pairs_.load() << "\n";
        std::cout << "Rejected pairs : " << rejected_pairs_.load() << "\n";

        const uint64_t total = accepted_pairs_.load() + rejected_pairs_.load();
        if (total > 0) {
            std::cout << "Acceptance     : "
                      << std::fixed << std::setprecision(1)
                      << (100.0 * accepted_pairs_.load() / double(total))
                      << "%\n";
        }

        return 0;
    }

private:
    bool openUsb(const char *dev)
    {
        usb_fd_ = ::open(dev, O_RDWR | O_NONBLOCK);
        if (usb_fd_ < 0) {
            perror("open USB");
            return false;
        }

        v4l2_capability cap{};
        if (xioctl(usb_fd_, VIDIOC_QUERYCAP, &cap) < 0) {
            perror("VIDIOC_QUERYCAP");
            return false;
        }

        std::cout << "[USB] card   : " << cap.card << "\n";
        std::cout << "[USB] driver : " << cap.driver << "\n";

        v4l2_format fmt{};
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = 640;
        fmt.fmt.pix.height = 480;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
        fmt.fmt.pix.field = V4L2_FIELD_ANY;

        if (xioctl(usb_fd_, VIDIOC_S_FMT, &fmt) < 0) {
            perror("VIDIOC_S_FMT");
            return false;
        }

        v4l2_streamparm parm{};
        parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        parm.parm.capture.timeperframe.numerator = 1;
        parm.parm.capture.timeperframe.denominator = 100;
        if (xioctl(usb_fd_, VIDIOC_S_PARM, &parm) < 0)
            perror("VIDIOC_S_PARM");

        std::cout << "[USB] format : 640x480 MJPG\n";

        v4l2_requestbuffers req{};
        req.count = 4;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;

        if (xioctl(usb_fd_, VIDIOC_REQBUFS, &req) < 0) {
            perror("VIDIOC_REQBUFS");
            return false;
        }

        usb_buffers_.resize(req.count);

        for (unsigned i = 0; i < req.count; ++i) {
            v4l2_buffer buf{};
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;

            if (xioctl(usb_fd_, VIDIOC_QUERYBUF, &buf) < 0) {
                perror("VIDIOC_QUERYBUF");
                return false;
            }

            usb_buffers_[i].len = buf.length;
            usb_buffers_[i].ptr = ::mmap(nullptr,
                                         buf.length,
                                         PROT_READ | PROT_WRITE,
                                         MAP_SHARED,
                                         usb_fd_,
                                         buf.m.offset);

            if (usb_buffers_[i].ptr == MAP_FAILED) {
                perror("mmap");
                return false;
            }

            if (xioctl(usb_fd_, VIDIOC_QBUF, &buf) < 0) {
                perror("VIDIOC_QBUF");
                return false;
            }
        }

        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (xioctl(usb_fd_, VIDIOC_STREAMON, &type) < 0) {
            perror("VIDIOC_STREAMON");
            return false;
        }

        usb_streaming_ = true;
        std::cout << "[USB] streaming started\n";
        return true;
    }

    bool openCsi()
    {
        if (camera_manager_.start()) {
            std::cerr << "[CSI] CameraManager::start failed\n";
            return false;
        }
        manager_started_ = true;

        for (const auto &cam : camera_manager_.cameras()) {
            if (cam->id().find("ov5647") != std::string::npos)
                camera_ = cam;
        }

        if (!camera_) {
            std::cerr << "[CSI] OV5647 not found\n";
            return false;
        }

        std::cout << "[CSI] selected: " << camera_->id() << "\n";

        if (camera_->acquire()) {
            std::cerr << "[CSI] acquire failed\n";
            return false;
        }
        camera_acquired_ = true;

        config_ = camera_->generateConfiguration({StreamRole::Viewfinder});
        if (!config_ || config_->empty()) {
            std::cerr << "[CSI] generateConfiguration failed\n";
            return false;
        }

        auto &sc = config_->at(0);
        sc.size.width = 640;
        sc.size.height = 480;
        sc.pixelFormat = formats::YUV420;

        if (config_->validate() == CameraConfiguration::Invalid) {
            std::cerr << "[CSI] configuration invalid\n";
            return false;
        }

        if (camera_->configure(config_.get())) {
            std::cerr << "[CSI] configure failed\n";
            return false;
        }

        stream_ = sc.stream();
        csi_width_ = sc.size.width;
        csi_height_ = sc.size.height;
        csi_stride_ = sc.stride;

        std::cout << "[CSI] configured output: "
                  << sc.size.width << "x" << sc.size.height << " "
                  << sc.pixelFormat.toString()
                  << " stride=" << sc.stride << "\n";

        allocator_ = std::make_unique<FrameBufferAllocator>(camera_);
        if (allocator_->allocate(stream_) < 0) {
            std::cerr << "[CSI] buffer allocation failed\n";
            return false;
        }

        for (const auto &buffer : allocator_->buffers(stream_)) {
            auto request = camera_->createRequest();
            if (!request || request->addBuffer(stream_, buffer.get()) < 0) {
                std::cerr << "[CSI] request setup failed\n";
                return false;
            }
            requests_.push_back(std::move(request));
        }

        return true;
    }

    void usbLoop()
    {
        while (running_.load()) {
            fd_set fds;
            FD_ZERO(&fds);
            FD_SET(usb_fd_, &fds);

            timeval tv{0, 200000};

            const int r = ::select(usb_fd_ + 1, &fds, nullptr, nullptr, &tv);
            if (r <= 0)
                continue;

            v4l2_buffer buf{};
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;

            if (xioctl(usb_fd_, VIDIOC_DQBUF, &buf) < 0) {
                if (errno == EAGAIN)
                    continue;
                break;
            }

            const int64_t ts_ns =
                int64_t(buf.timestamp.tv_sec) * 1000000000LL +
                int64_t(buf.timestamp.tv_usec) * 1000LL;

            UsbFrame frame;
            frame.seq = buf.sequence;
            frame.ts_ns = ts_ns;
            frame.jpeg.resize(buf.bytesused);
            std::memcpy(frame.jpeg.data(),
                        usb_buffers_[buf.index].ptr,
                        buf.bytesused);

            {
                std::lock_guard<std::mutex> lock(usb_mutex_);
                usb_history_.push_back(std::move(frame));
                while (usb_history_.size() > 100)
                    usb_history_.pop_front();
            }

            if (xioctl(usb_fd_, VIDIOC_QBUF, &buf) < 0)
                break;
        }
    }

    cv::Mat mapCsiBuffer(FrameBuffer *fb)
    {
        if (!fb || fb->planes().empty() || csi_width_ == 0 || csi_height_ == 0)
            return {};

        const auto &plane = fb->planes()[0];
        const int dma_fd = plane.fd.get();

        // For the diagnostic viewer we only need the Y plane.
        // Do not assume that libcamera's YUV420 planes are one tightly-packed
        // 640x720 block: on PiSP they may have offsets/strides in a DMA buffer.
        const size_t map_len =
            static_cast<size_t>(plane.offset) +
            static_cast<size_t>(plane.length);

        if (map_len == 0)
            return {};

        void *base = ::mmap(nullptr, map_len, PROT_READ, MAP_SHARED, dma_fd, 0);
        if (base == MAP_FAILED) {
            perror("mmap CSI");
            return {};
        }

        const uint8_t *y =
            static_cast<const uint8_t *>(base) + plane.offset;

        const size_t stride =
            csi_stride_ > 0 ? static_cast<size_t>(csi_stride_)
                            : static_cast<size_t>(csi_width_);

        const size_t required =
            stride * static_cast<size_t>(csi_height_);

        if (required > plane.length) {
            std::cerr << "[CSI] Y plane too small: length=" << plane.length
                      << " required=" << required
                      << " stride=" << stride << "\n";
            ::munmap(base, map_len);
            return {};
        }

        cv::Mat gray(csi_height_,
                     csi_width_,
                     CV_8UC1,
                     const_cast<uint8_t *>(y),
                     stride);

        // Clone before munmap: returned Mat must own its memory.
        cv::Mat gray_copy = gray.clone();
        ::munmap(base, map_len);

        cv::Mat bgr;
        cv::cvtColor(gray_copy, bgr, cv::COLOR_GRAY2BGR);
        return bgr;
    }

    void onCsiFrame(Request *request)
    {
        if (request->status() == Request::RequestCancelled)
            return;

        int64_t csi_ts_ns = 0;
        auto sensor_ts = request->metadata().get(controls::SensorTimestamp);
        if (sensor_ts.has_value())
            csi_ts_ns = sensor_ts.value();
        else if (!request->buffers().empty())
            csi_ts_ns = int64_t(request->buffers().begin()->second->metadata().timestamp);

        UsbFrame best;
        bool found = false;
        int64_t best_abs_ns = INT64_MAX;

        {
            std::lock_guard<std::mutex> lock(usb_mutex_);
            for (const auto &u : usb_history_) {
                const int64_t dt = u.ts_ns - csi_ts_ns;
                const int64_t adt = std::llabs(dt);
                if (adt < best_abs_ns) {
                    best_abs_ns = adt;
                    best = u;
                    found = true;
                }
            }
        }

        if (found) {
            const double dt_ms = double(best.ts_ns - csi_ts_ns) / 1e6;

            if (std::abs(dt_ms) <= MAX_STEREO_DT_MS) {
                auto it = request->buffers().find(stream_);
                if (it != request->buffers().end()) {
                    cv::Mat csi = mapCsiBuffer(it->second);

                    cv::Mat usb = cv::imdecode(best.jpeg, cv::IMREAD_COLOR);

                    if (!csi.empty() && !usb.empty()) {
                        if (usb.size() != csi.size())
                            cv::resize(usb, usb, csi.size());

                        cv::putText(usb,
                                    "OV9281 USB",
                                    cv::Point(15, 30),
                                    cv::FONT_HERSHEY_SIMPLEX,
                                    0.8,
                                    cv::Scalar(255,255,255),
                                    2);

                        cv::putText(csi,
                                    "OV5647 CSI",
                                    cv::Point(15, 30),
                                    cv::FONT_HERSHEY_SIMPLEX,
                                    0.8,
                                    cv::Scalar(255,255,255),
                                    2);

                        std::vector<cv::Mat> pair{usb, csi};
                        cv::Mat joined;
                        cv::hconcat(pair, joined);

                        const std::string status =
                            "dt=" + cv::format("%+.3f ms", dt_ms) +
                            "  max=7.0 ms";

                        cv::putText(joined,
                                    status,
                                    cv::Point(15, joined.rows - 20),
                                    cv::FONT_HERSHEY_SIMPLEX,
                                    0.7,
                                    cv::Scalar(255,255,255),
                                    2);

                        {
                            std::lock_guard<std::mutex> lock(display_mutex_);
                            latest_display_ = joined;
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
            if (camera_->queueRequest(request) < 0)
                std::cerr << "[CSI] queueRequest failed\n";
        }
    }

    void cleanup()
    {
        if (cleaned_)
            return;
        cleaned_ = true;

        running_.store(false);

        if (camera_started_ && camera_) {
            camera_->stop();
            camera_started_ = false;
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
                camera_acquired_ = false;
            }

            camera_.reset();
        }

        config_.reset();

        if (manager_started_) {
            camera_manager_.stop();
            manager_started_ = false;
        }

        if (usb_streaming_ && usb_fd_ >= 0) {
            v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            xioctl(usb_fd_, VIDIOC_STREAMOFF, &type);
            usb_streaming_ = false;
        }

        for (auto &b : usb_buffers_) {
            if (b.ptr && b.ptr != MAP_FAILED)
                ::munmap(b.ptr, b.len);
        }
        usb_buffers_.clear();

        if (usb_fd_ >= 0) {
            ::close(usb_fd_);
            usb_fd_ = -1;
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

    std::mutex display_mutex_;
    cv::Mat latest_display_;

    std::atomic<uint64_t> accepted_pairs_{0};
    std::atomic<uint64_t> rejected_pairs_{0};
};

int main(int argc, char **argv)
{
    const char *usb_dev = argc > 1 ? argv[1] : "/dev/video0";

    std::cout << "========================================\n";
    std::cout << " JT-Zero stereo synchronized viewer\n";
    std::cout << "========================================\n";
    std::cout << "USB camera : " << usb_dev << "\n";
    std::cout << "CSI camera : OV5647\n";
    std::cout << "Max |dt|   : " << MAX_STEREO_DT_MS << " ms\n";

    StereoViewTest app;
    if (!app.initialize(usb_dev))
        return 1;

    return app.run();
}
