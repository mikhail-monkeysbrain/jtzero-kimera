#include <libcamera/libcamera.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <numeric>
#include <thread>
#include <vector>

using namespace libcamera;

struct UsbStamp {
    uint64_t seq{0};
    int64_t ts_ns{0};
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

class StereoSyncTest {
public:
    ~StereoSyncTest() { cleanup(); }

    bool initialize(const char *usb_dev)
    {
        if (!openUsb(usb_dev))
            return false;
        if (!openCsi())
            return false;
        camera_->requestCompleted.connect(this, &StereoSyncTest::onCsiFrame);
        return true;
    }

    int run(int seconds)
    {
        running_.store(true);
        usb_thread_ = std::thread(&StereoSyncTest::usbLoop, this);

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

        std::cout << "\n[TEST] capture running...\n\n";
        std::this_thread::sleep_for(std::chrono::seconds(seconds));
        running_.store(false);

        if (camera_started_) {
            camera_->stop();
            camera_started_ = false;
        }

        if (usb_thread_.joinable())
            usb_thread_.join();

        printSummary();
        cleanup();
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

        const double fps = parm.parm.capture.timeperframe.numerator
            ? double(parm.parm.capture.timeperframe.denominator) /
              double(parm.parm.capture.timeperframe.numerator)
            : 0.0;

        std::cout << "[USB] format : " << fmt.fmt.pix.width << "x"
                  << fmt.fmt.pix.height << " MJPG\n";
        std::cout << "[USB] FPS    : " << std::fixed << std::setprecision(2)
                  << fps << "\n";

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
            usb_buffers_[i].ptr = ::mmap(nullptr, buf.length,
                                         PROT_READ | PROT_WRITE,
                                         MAP_SHARED, usb_fd_, buf.m.offset);
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

        std::cout << "\n[CSI] libcamera cameras:\n";
        for (const auto &cam : camera_manager_.cameras()) {
            std::cout << "      " << cam->id() << "\n";
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
        std::cout << "[CSI] configured output: " << sc.size.width << "x"
                  << sc.size.height << " " << sc.pixelFormat.toString() << "\n";

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

        std::cout << "[CSI] request buffers: " << requests_.size() << "\n";
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

            if (!have_usb_flags_.exchange(true))
                usb_flags_.store(buf.flags);

            const int64_t ts_ns =
                int64_t(buf.timestamp.tv_sec) * 1000000000LL +
                int64_t(buf.timestamp.tv_usec) * 1000LL;

            {
                std::lock_guard<std::mutex> lock(usb_mutex_);
                usb_history_.push_back({uint64_t(buf.sequence), ts_ns});
                while (usb_history_.size() > 2000)
                    usb_history_.pop_front();
            }

            ++usb_frames_;
            xioctl(usb_fd_, VIDIOC_QBUF, &buf);
        }
    }

    void onCsiFrame(Request *request)
    {
        if (request->status() == Request::RequestCancelled)
            return;

        const uint64_t csi_seq = ++csi_frames_;
        int64_t csi_ts_ns = 0;

        auto sensor_ts = request->metadata().get(controls::SensorTimestamp);
        if (sensor_ts.has_value()) {
            csi_ts_ns = sensor_ts.value();
            sensor_timestamp_used_.store(true);
        } else if (!request->buffers().empty()) {
            csi_ts_ns = int64_t(request->buffers().begin()->second->metadata().timestamp);
        }

        UsbStamp best{};
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
            {
                std::lock_guard<std::mutex> lock(stats_mutex_);
                signed_dt_ms_.push_back(dt_ms);
                abs_dt_ms_.push_back(std::abs(dt_ms));
            }
            ++paired_frames_;

            if (csi_seq <= 20 || csi_seq % 10 == 0) {
                std::cout << "CSI #" << std::setw(5) << csi_seq
                          << "  USB #" << std::setw(5) << best.seq
                          << "  dt=" << std::showpos << std::fixed
                          << std::setprecision(3) << dt_ms << " ms"
                          << std::noshowpos << "\n";
            }
        }

        if (running_.load()) {
            request->reuse(Request::ReuseBuffers);
            if (camera_->queueRequest(request) < 0)
                std::cerr << "[CSI] queueRequest failed\n";
        }
    }

    static double percentile(const std::vector<double> &sorted, double p)
    {
        if (sorted.empty())
            return 0.0;
        const double pos = p * (sorted.size() - 1);
        const size_t lo = size_t(std::floor(pos));
        const size_t hi = size_t(std::ceil(pos));
        if (lo == hi)
            return sorted[lo];
        const double f = pos - lo;
        return sorted[lo] * (1.0 - f) + sorted[hi] * f;
    }

    void printSummary()
    {
        std::vector<double> abs_dt;
        std::vector<double> signed_dt;
        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            abs_dt = abs_dt_ms_;
            signed_dt = signed_dt_ms_;
        }

        std::sort(abs_dt.begin(), abs_dt.end());

        std::cout << "\n========================================\n";
        std::cout << " SUMMARY\n";
        std::cout << "========================================\n";
        std::cout << "USB frames       : " << usb_frames_.load() << "\n";
        std::cout << "CSI frames       : " << csi_frames_.load() << "\n";
        std::cout << "Paired frames    : " << paired_frames_.load() << "\n";
        std::cout << "CSI timestamp    : "
                  << (sensor_timestamp_used_.load() ? "SensorTimestamp" : "FrameBuffer timestamp")
                  << "\n";

        const uint32_t flags = usb_flags_.load();
        std::cout << "USB ts flags     : 0x" << std::hex << flags << std::dec << "\n";
        std::cout << "USB timestamp    : "
                  << (((flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) ==
                       V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC) ? "MONOTONIC" : "OTHER")
                  << "\n";

        if (!abs_dt.empty()) {
            const double mean_abs =
                std::accumulate(abs_dt.begin(), abs_dt.end(), 0.0) / abs_dt.size();
            const double mean_signed =
                std::accumulate(signed_dt.begin(), signed_dt.end(), 0.0) / signed_dt.size();

            std::cout << std::fixed << std::setprecision(3);
            std::cout << "mean dt          : " << mean_signed << " ms\n";
            std::cout << "mean |dt|        : " << mean_abs << " ms\n";
            std::cout << "median |dt|      : " << percentile(abs_dt, 0.50) << " ms\n";
            std::cout << "p95 |dt|         : " << percentile(abs_dt, 0.95) << " ms\n";
            std::cout << "max |dt|         : " << abs_dt.back() << " ms\n";
        }

        std::cout << "========================================\n";
    }

    void cleanup()
    {
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

    int usb_fd_{-1};
    bool usb_streaming_{false};
    std::vector<MMapBuf> usb_buffers_;
    std::thread usb_thread_;
    std::mutex usb_mutex_;
    std::deque<UsbStamp> usb_history_;

    CameraManager camera_manager_;
    bool manager_started_{false};
    std::shared_ptr<Camera> camera_;
    bool camera_acquired_{false};
    bool camera_started_{false};
    std::unique_ptr<CameraConfiguration> config_;
    Stream *stream_{nullptr};
    std::unique_ptr<FrameBufferAllocator> allocator_;
    std::vector<std::unique_ptr<Request>> requests_;

    std::atomic<bool> running_{false};
    std::atomic<uint64_t> usb_frames_{0};
    std::atomic<uint64_t> csi_frames_{0};
    std::atomic<uint64_t> paired_frames_{0};
    std::atomic<uint32_t> usb_flags_{0};
    std::atomic<bool> have_usb_flags_{false};
    std::atomic<bool> sensor_timestamp_used_{false};

    std::mutex stats_mutex_;
    std::vector<double> signed_dt_ms_;
    std::vector<double> abs_dt_ms_;
};

int main(int argc, char **argv)
{
    const char *usb_dev = argc > 1 ? argv[1] : "/dev/video0";
    const int seconds = argc > 2 ? std::atoi(argv[2]) : 10;

    std::cout << "========================================\n";
    std::cout << " JT-Zero stereo timestamp diagnostic\n";
    std::cout << "========================================\n";
    std::cout << "USB camera : " << usb_dev << "\n";
    std::cout << "CSI camera : OV5647 / libcamera\n";
    std::cout << "Duration   : " << seconds << " s\n\n";

    StereoSyncTest test;
    if (!test.initialize(usb_dev))
        return 1;
    return test.run(seconds);
}
