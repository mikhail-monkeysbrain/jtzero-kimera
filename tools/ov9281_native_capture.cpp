#include <linux/videodev2.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

constexpr int WIDTH = 640;
constexpr int HEIGHT = 480;
constexpr int CAMERA_FPS = 120;
constexpr double OUTPUT_FPS = 30.0;
constexpr int BUFFER_COUNT = 6;
constexpr int WARMUP_FRAMES = 30;
constexpr int EXPOSURE_ABSOLUTE = 50;
constexpr int GAIN = 0;
constexpr int64_t TARGET_PERIOD_NS = 33333333LL;

struct MmapBuffer {
    void* start = nullptr;
    size_t length = 0;
};

struct HeldFrame {
    v4l2_buffer buf{};
    bool valid = false;
};

int xioctl(int fd, unsigned long request, void* arg) {
    int rc;
    do {
        rc = ioctl(fd, request, arg);
    } while (rc == -1 && errno == EINTR);
    return rc;
}

[[noreturn]] void fail(const std::string& message) {
    throw std::runtime_error(message + ": " + std::strerror(errno));
}

int64_t timevalToNs(const timeval& tv) {
    return static_cast<int64_t>(tv.tv_sec) * 1000000000LL +
           static_cast<int64_t>(tv.tv_usec) * 1000LL;
}

double nsToMs(int64_t ns) {
    return static_cast<double>(ns) / 1.0e6;
}

std::string timestampFlags(uint32_t flags) {
    std::string result;

    if ((flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) ==
        V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC) {
        result = "monotonic";
    } else {
        result = "other-clock";
    }

    result += ",";

    if ((flags & V4L2_BUF_FLAG_TSTAMP_SRC_MASK) ==
        V4L2_BUF_FLAG_TSTAMP_SRC_SOE) {
        result += "soe";
    } else {
        result += "eof";
    }

    return result;
}

void queueBuffer(int fd, v4l2_buffer& buf) {
    if (xioctl(fd, VIDIOC_QBUF, &buf) == -1) {
        fail("VIDIOC_QBUF failed");
    }
}

void setControl(int fd, uint32_t id, int32_t value, const char* name) {
    v4l2_control ctrl{};
    ctrl.id = id;
    ctrl.value = value;

    if (xioctl(fd, VIDIOC_S_CTRL, &ctrl) == -1) {
        std::cerr << "[WARN] Cannot set " << name << "=" << value << ": "
                  << std::strerror(errno) << '\n';
        return;
    }

    std::cout << "[CAM] " << name << "=" << value << '\n';
}

void configureCamera(int fd) {
    v4l2_capability cap{};

    if (xioctl(fd, VIDIOC_QUERYCAP, &cap) == -1) {
        fail("VIDIOC_QUERYCAP failed");
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        throw std::runtime_error("Device is not a video capture device");
    }

    if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
        throw std::runtime_error("Device does not support streaming I/O");
    }

    std::cout << "[CAM] driver=" << cap.driver << " card=" << cap.card << '\n';

    v4l2_format fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = WIDTH;
    fmt.fmt.pix.height = HEIGHT;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;

    if (xioctl(fd, VIDIOC_S_FMT, &fmt) == -1) {
        fail("VIDIOC_S_FMT failed");
    }

    if (fmt.fmt.pix.width != WIDTH || fmt.fmt.pix.height != HEIGHT ||
        fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_MJPEG) {
        throw std::runtime_error("Camera did not accept 640x480 MJPEG");
    }

    std::cout << "[CAM] format=" << fmt.fmt.pix.width << "x" << fmt.fmt.pix.height
              << " MJPEG\n";

    v4l2_streamparm parm{};
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = CAMERA_FPS;

    if (xioctl(fd, VIDIOC_S_PARM, &parm) == -1) {
        fail("VIDIOC_S_PARM failed");
    }

    const auto& tpf = parm.parm.capture.timeperframe;
    double actual_fps = 0.0;
    if (tpf.numerator != 0) {
        actual_fps = static_cast<double>(tpf.denominator) /
                     static_cast<double>(tpf.numerator);
    }

    std::cout << "[CAM] requested_fps=" << CAMERA_FPS << " actual_fps="
              << std::fixed << std::setprecision(3) << actual_fps << '\n';

    setControl(fd, V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL,
               "auto_exposure");
    setControl(fd, V4L2_CID_EXPOSURE_AUTO_PRIORITY, 0, "dynamic_framerate");
    setControl(fd, V4L2_CID_EXPOSURE_ABSOLUTE, EXPOSURE_ABSOLUTE,
               "exposure_absolute");
    setControl(fd, V4L2_CID_GAIN, GAIN, "gain");
}

std::vector<MmapBuffer> initMmap(int fd) {
    v4l2_requestbuffers req{};
    req.count = BUFFER_COUNT;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (xioctl(fd, VIDIOC_REQBUFS, &req) == -1) {
        fail("VIDIOC_REQBUFS failed");
    }

    if (req.count < 4) {
        throw std::runtime_error("Not enough V4L2 mmap buffers");
    }

    std::vector<MmapBuffer> buffers(req.count);

    for (uint32_t i = 0; i < req.count; ++i) {
        v4l2_buffer buf{};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (xioctl(fd, VIDIOC_QUERYBUF, &buf) == -1) {
            fail("VIDIOC_QUERYBUF failed");
        }

        buffers[i].length = buf.length;
        buffers[i].start = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE,
                                MAP_SHARED, fd, buf.m.offset);

        if (buffers[i].start == MAP_FAILED) {
            fail("mmap failed");
        }
    }

    for (uint32_t i = 0; i < buffers.size(); ++i) {
        v4l2_buffer buf{};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        queueBuffer(fd, buf);
    }

    std::cout << "[CAM] mmap_buffers=" << buffers.size() << '\n';
    return buffers;
}

cv::Mat decodeGray(const v4l2_buffer& buf,
                   const std::vector<MmapBuffer>& buffers) {
    if (buf.index >= buffers.size()) {
        throw std::runtime_error("Invalid mmap buffer index");
    }

    const auto& mapped = buffers[buf.index];

    if (buf.bytesused == 0 || buf.bytesused > mapped.length) {
        throw std::runtime_error("Invalid MJPEG buffer size");
    }

    // Zero-copy compressed input. The mmap buffer remains owned by userspace
    // until imdecode() finishes; only then is VIDIOC_QBUF called.
    cv::Mat compressed(1, static_cast<int>(buf.bytesused), CV_8UC1,
                       mapped.start);

    cv::Mat gray = cv::imdecode(compressed, cv::IMREAD_GRAYSCALE);

    if (gray.empty()) {
        throw std::runtime_error("MJPEG decode failed");
    }

    if (gray.cols != WIDTH || gray.rows != HEIGHT || gray.type() != CV_8UC1) {
        throw std::runtime_error("Decoded image is not 640x480 CV_8UC1");
    }

    return gray;
}

double percentile(std::vector<double> values, double q) {
    if (values.empty()) {
        return 0.0;
    }

    std::sort(values.begin(), values.end());
    const double pos = q * static_cast<double>(values.size() - 1);
    const size_t lo = static_cast<size_t>(std::floor(pos));
    const size_t hi = std::min(lo + 1, values.size() - 1);
    const double f = pos - static_cast<double>(lo);
    return values[lo] * (1.0 - f) + values[hi] * f;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        double duration_sec = 30.0;
        int save_every = 30;

        if (argc >= 2) {
            duration_sec = std::stod(argv[1]);
        }
        if (argc >= 3) {
            save_every = std::stoi(argv[2]);
        }
        if (duration_sec <= 0.0) {
            throw std::runtime_error("Duration must be > 0");
        }

        const std::string device = "/dev/video0";
        const std::string csv_path = "/home/vio/ov9281_native_capture_v2.csv";
        const std::string image_dir = "/home/vio/ov9281_native_frames_v2";

        fs::create_directories(image_dir);

        const int fd = open(device.c_str(), O_RDWR | O_NONBLOCK);
        if (fd == -1) {
            fail("Cannot open " + device);
        }

        configureCamera(fd);
        auto buffers = initMmap(fd);

        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (xioctl(fd, VIDIOC_STREAMON, &type) == -1) {
            fail("VIDIOC_STREAMON failed");
        }

        std::ofstream csv(csv_path);
        if (!csv) {
            throw std::runtime_error("Cannot create CSV");
        }

        csv << "selected_index,source_sequence,timestamp_ns,dt_ms,"
               "target_error_ms,source_sequence_gap,bytes_used,mean_gray,"
               "timestamp_flags\n";

        std::cout << '\n'
                  << "=== OV9281 NATIVE CAPTURE V2 ===\n"
                  << "Input : 640x480 MJPEG @ 120 FPS\n"
                  << "Output: grayscale target " << OUTPUT_FPS << " FPS\n"
                  << "Mode  : zero-copy source / decode selected only\n"
                  << "Warmup: " << WARMUP_FRAMES << " source frames\n"
                  << "Duration: " << duration_sec << " s\n\n";

        uint64_t source_frames = 0;
        uint64_t source_drops = 0;
        uint64_t selected_frames = 0;
        uint64_t decode_errors = 0;
        uint64_t skipped_targets = 0;

        bool have_last_source_sequence = false;
        uint32_t last_source_sequence = 0;
        bool target_initialized = false;
        int64_t next_target_ns = 0;
        HeldFrame previous;
        int64_t first_selected_ts = 0;
        int64_t last_selected_ts = 0;
        bool have_selected_sequence = false;
        uint32_t last_selected_sequence = 0;

        std::vector<double> dt_values_ms;
        std::vector<double> target_errors_ms;

        const auto wall_start = std::chrono::steady_clock::now();

        auto processSelected = [&](const v4l2_buffer& buf, int64_t target_ns) {
            cv::Mat gray;
            try {
                gray = decodeGray(buf, buffers);
            } catch (const std::exception& e) {
                ++decode_errors;
                std::cerr << "[DECODE] seq=" << buf.sequence << " " << e.what()
                          << '\n';
                return;
            }

            const int64_t timestamp_ns = timevalToNs(buf.timestamp);
            ++selected_frames;

            if (selected_frames == 1) {
                first_selected_ts = timestamp_ns;
            }

            double dt_ms = 0.0;
            if (selected_frames > 1) {
                dt_ms = nsToMs(timestamp_ns - last_selected_ts);
                dt_values_ms.push_back(dt_ms);
            }

            const double target_error_ms = nsToMs(timestamp_ns - target_ns);
            target_errors_ms.push_back(target_error_ms);

            uint32_t selected_sequence_gap = 0;
            if (have_selected_sequence) {
                selected_sequence_gap = buf.sequence - last_selected_sequence;
            }

            have_selected_sequence = true;
            last_selected_sequence = buf.sequence;
            last_selected_ts = timestamp_ns;

            const double mean_gray = cv::mean(gray)[0];

            csv << selected_frames << ',' << buf.sequence << ',' << timestamp_ns
                << ',' << std::fixed << std::setprecision(3) << dt_ms << ','
                << target_error_ms << ',' << selected_sequence_gap << ','
                << buf.bytesused << ',' << std::setprecision(2) << mean_gray
                << ',' << timestampFlags(buf.flags) << '\n';

            if (save_every > 0 &&
                selected_frames % static_cast<uint64_t>(save_every) == 0) {
                const std::string path = image_dir + "/frame_" +
                                         std::to_string(selected_frames) + ".jpg";
                cv::imwrite(path, gray);
            }

            if (selected_frames % 30 == 0) {
                double actual_output_fps = 0.0;
                if (selected_frames > 1) {
                    const double span_sec =
                        static_cast<double>(last_selected_ts - first_selected_ts) /
                        1.0e9;
                    if (span_sec > 0.0) {
                        actual_output_fps =
                            static_cast<double>(selected_frames - 1) / span_sec;
                    }
                }

                std::cout << "[OUT] frames=" << selected_frames
                          << " seq=" << buf.sequence << " dt=" << std::fixed
                          << std::setprecision(3) << dt_ms << " ms err="
                          << target_error_ms << " ms fps=" << actual_output_fps
                          << " mean=" << mean_gray << '\n';
            }
        };

        while (true) {
            const auto now = std::chrono::steady_clock::now();
            const double elapsed =
                std::chrono::duration<double>(now - wall_start).count();

            if (elapsed >= duration_sec) {
                break;
            }

            pollfd pfd{};
            pfd.fd = fd;
            pfd.events = POLLIN;

            const int poll_rc = poll(&pfd, 1, 1000);
            if (poll_rc == -1) {
                if (errno == EINTR) {
                    continue;
                }
                fail("poll failed");
            }
            if (poll_rc == 0) {
                std::cerr << "[WARN] camera poll timeout\n";
                continue;
            }

            v4l2_buffer current{};
            current.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            current.memory = V4L2_MEMORY_MMAP;

            if (xioctl(fd, VIDIOC_DQBUF, &current) == -1) {
                if (errno == EAGAIN) {
                    continue;
                }
                fail("VIDIOC_DQBUF failed");
            }

            ++source_frames;

            if (have_last_source_sequence) {
                const uint32_t expected = last_source_sequence + 1;
                if (current.sequence != expected) {
                    uint32_t lost = 0;
                    if (current.sequence > expected) {
                        lost = current.sequence - expected;
                    }
                    if (lost > 0) {
                        source_drops += lost;
                        std::cerr << "[DROP] expected seq=" << expected
                                  << " got=" << current.sequence
                                  << " lost=" << lost << '\n';
                    }
                }
            }

            have_last_source_sequence = true;
            last_source_sequence = current.sequence;

            if (source_frames <= WARMUP_FRAMES) {
                queueBuffer(fd, current);
                continue;
            }

            const int64_t current_ts = timevalToNs(current.timestamp);

            if (!target_initialized) {
                target_initialized = true;
                next_target_ns = current_ts;
                processSelected(current, next_target_ns);
                next_target_ns += TARGET_PERIOD_NS;
                queueBuffer(fd, current);
                continue;
            }

            if (!previous.valid) {
                previous.buf = current;
                previous.valid = true;
                continue;
            }

            const int64_t previous_ts = timevalToNs(previous.buf.timestamp);

            if (current_ts < next_target_ns) {
                queueBuffer(fd, previous.buf);
                previous.buf = current;
                previous.valid = true;
                continue;
            }

            const int64_t previous_error =
                std::llabs(previous_ts - next_target_ns);
            const int64_t current_error =
                std::llabs(current_ts - next_target_ns);

            if (previous_error <= current_error) {
                processSelected(previous.buf, next_target_ns);
            } else {
                processSelected(current, next_target_ns);
            }

            queueBuffer(fd, previous.buf);
            queueBuffer(fd, current);
            previous.valid = false;

            next_target_ns += TARGET_PERIOD_NS;

            while (next_target_ns <= current_ts) {
                next_target_ns += TARGET_PERIOD_NS;
                ++skipped_targets;
            }
        }

        if (previous.valid) {
            queueBuffer(fd, previous.buf);
            previous.valid = false;
        }

        if (xioctl(fd, VIDIOC_STREAMOFF, &type) == -1) {
            std::cerr << "[WARN] VIDIOC_STREAMOFF: " << std::strerror(errno)
                      << '\n';
        }

        for (auto& buffer : buffers) {
            if (buffer.start && buffer.start != MAP_FAILED) {
                munmap(buffer.start, buffer.length);
            }
        }

        close(fd);
        csv.flush();

        std::cout << '\n' << "========== RESULT ==========\n"
                  << "source_frames     = " << source_frames << '\n'
                  << "source_drops      = " << source_drops << '\n'
                  << "selected_frames   = " << selected_frames << '\n'
                  << "decode_errors     = " << decode_errors << '\n'
                  << "skipped_targets   = " << skipped_targets << '\n';

        if (selected_frames > 1) {
            const double span_sec =
                static_cast<double>(last_selected_ts - first_selected_ts) / 1.0e9;
            const double actual_output_fps =
                static_cast<double>(selected_frames - 1) / span_sec;

            std::cout << "timestamp_span    = " << std::fixed
                      << std::setprecision(6) << span_sec << " s\n"
                      << "output_fps        = " << std::setprecision(3)
                      << actual_output_fps << '\n';
        }

        if (!dt_values_ms.empty()) {
            const auto [min_it, max_it] =
                std::minmax_element(dt_values_ms.begin(), dt_values_ms.end());
            const double mean =
                std::accumulate(dt_values_ms.begin(), dt_values_ms.end(), 0.0) /
                static_cast<double>(dt_values_ms.size());

            std::cout << "dt_min_ms         = " << std::fixed
                      << std::setprecision(3) << *min_it << '\n'
                      << "dt_mean_ms        = " << mean << '\n'
                      << "dt_p95_ms         = "
                      << percentile(dt_values_ms, 0.95) << '\n'
                      << "dt_max_ms         = " << *max_it << '\n';
        }

        if (!target_errors_ms.empty()) {
            std::vector<double> absolute_errors;
            absolute_errors.reserve(target_errors_ms.size());

            for (double value : target_errors_ms) {
                absolute_errors.push_back(std::abs(value));
            }

            const auto max_it =
                std::max_element(absolute_errors.begin(), absolute_errors.end());
            const double mean =
                std::accumulate(absolute_errors.begin(), absolute_errors.end(),
                                0.0) /
                static_cast<double>(absolute_errors.size());

            std::cout << "target_abs_mean_ms= " << std::fixed
                      << std::setprecision(3) << mean << '\n'
                      << "target_abs_p95_ms = "
                      << percentile(absolute_errors, 0.95) << '\n'
                      << "target_abs_max_ms = " << *max_it << '\n';
        }

        std::cout << "CSV               = " << csv_path << '\n'
                  << "frames_dir        = " << image_dir << '\n';

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << '\n';
        return 1;
    }
}
