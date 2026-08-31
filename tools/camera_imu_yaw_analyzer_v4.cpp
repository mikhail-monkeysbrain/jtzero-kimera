#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cctype>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

struct CameraIndex {
    uint32_t sequence = 0;
    int64_t timestamp_ns = 0;
    uint64_t offset = 0;
    uint32_t bytes = 0;
};

struct ImuSample {
    int64_t mapped_ns = 0;
    double gx = 0.0;
    double gy = 0.0;
    double gz = 0.0;
};

struct VisualSample {
    int64_t timestamp_ns = 0;
    double omega = 0.0;
    int tracked = 0;
    int inliers = 0;
};

struct VisualGapStats {
    size_t rejected_pairs = 0;
    size_t sequence_gaps = 0;
    size_t timestamp_gaps = 0;
    uint64_t source_frames_missing = 0;
    double max_rejected_dt_ms = 0.0;
};

struct SearchResult {
    int axis = -1;
    double offset_ms = 0.0;
    double corr = 0.0;
    int count = 0;
};

struct Segment {
    size_t begin = 0;
    size_t end = 0; // exclusive
    int64_t start_ns = 0;
    int64_t end_ns = 0;
    double duration_s = 0.0;
    double peak_abs_omega = 0.0;
};

struct SegmentResult {
    int index = 0;
    Segment segment;
    SearchResult best;
};

static std::vector<std::string> splitCsv(const std::string& line) {
    std::vector<std::string> out;
    std::string item;
    std::stringstream ss(line);
    while (std::getline(ss, item, ',')) out.push_back(item);
    return out;
}

static double median(std::vector<double> v) {
    if (v.empty()) return std::numeric_limits<double>::quiet_NaN();
    const size_t n = v.size();
    std::nth_element(v.begin(), v.begin() + n / 2, v.end());
    double m = v[n / 2];
    if ((n & 1U) == 0) {
        std::nth_element(v.begin(), v.begin() + n / 2 - 1, v.end());
        m = 0.5 * (m + v[n / 2 - 1]);
    }
    return m;
}

static double percentile(std::vector<double> v, double p) {
    if (v.empty()) return std::numeric_limits<double>::quiet_NaN();
    std::sort(v.begin(), v.end());
    const double x = (v.size() - 1) * p / 100.0;
    const size_t lo = static_cast<size_t>(std::floor(x));
    const size_t hi = static_cast<size_t>(std::ceil(x));
    if (lo == hi) return v[lo];
    const double f = x - lo;
    return v[lo] * (1.0 - f) + v[hi] * f;
}

static std::vector<CameraIndex> readCameraIndex(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("Cannot open camera index: " + path);

    std::string line;
    if (!std::getline(f, line)) throw std::runtime_error("Empty camera index");

    const auto header = splitCsv(line);
    int i_seq = -1, i_ts_raw = -1, i_ts_corrected = -1;
    int i_off = -1, i_bytes = -1;
    for (int i = 0; i < static_cast<int>(header.size()); ++i) {
        if (header[i] == "sequence") i_seq = i;
        else if (header[i] == "v4l2_timestamp_ns") i_ts_raw = i;
        else if (header[i] == "camera_timestamp_corrected_ns") i_ts_corrected = i;
        else if (header[i] == "mjpeg_offset") i_off = i;
        else if (header[i] == "bytes_used") i_bytes = i;
    }
    const int i_ts = (i_ts_corrected >= 0) ? i_ts_corrected : i_ts_raw;

    if (i_seq < 0 || i_ts < 0 || i_off < 0 || i_bytes < 0)
        throw std::runtime_error("Unexpected camera CSV header");

    std::vector<CameraIndex> out;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        const auto c = splitCsv(line);
        const int need = std::max(std::max(i_seq, i_ts), std::max(i_off, i_bytes));
        if (static_cast<int>(c.size()) <= need) continue;

        try {
            CameraIndex s;
            s.sequence = static_cast<uint32_t>(std::stoul(c[i_seq]));
            s.timestamp_ns = std::stoll(c[i_ts]);
            s.offset = std::stoull(c[i_off]);
            s.bytes = static_cast<uint32_t>(std::stoul(c[i_bytes]));
            out.push_back(s);
        } catch (...) {}
    }
    return out;
}

static std::vector<ImuSample> readImu(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("Cannot open combined CSV: " + path);

    std::string line;
    if (!std::getline(f, line)) throw std::runtime_error("Empty combined CSV");
    const auto header = splitCsv(line);

    int i_event = -1, i_mapped = -1, i_gx = -1, i_gy = -1, i_gz = -1;

    for (int i = 0; i < static_cast<int>(header.size()); ++i) {
        std::string h = header[i];
        for (char& ch : h)
            ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));

        if (h == "event" || h == "type") i_event = i;
        if (h == "mapped_rpi_ns" ||
            (h.find("mapped") != std::string::npos &&
             h.find("ns") != std::string::npos)) {
            i_mapped = i;
        }
        if (h == "gx" || h == "xgyro" || h == "xgyro_rad_s" ||
            h.find("gyro_x") != std::string::npos ||
            h.find("xgyro") != std::string::npos) i_gx = i;
        if (h == "gy" || h == "ygyro" || h == "ygyro_rad_s" ||
            h.find("gyro_y") != std::string::npos ||
            h.find("ygyro") != std::string::npos) i_gy = i;
        if (h == "gz" || h == "zgyro" || h == "zgyro_rad_s" ||
            h.find("gyro_z") != std::string::npos ||
            h.find("zgyro") != std::string::npos) i_gz = i;
    }

    if (i_event < 0 || i_mapped < 0 || i_gx < 0 || i_gy < 0 || i_gz < 0)
        throw std::runtime_error("Unexpected combined CSV header");

    std::vector<ImuSample> out;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        const auto c = splitCsv(line);
        const int need = std::max({i_event, i_mapped, i_gx, i_gy, i_gz});
        if (static_cast<int>(c.size()) <= need) continue;
        if (c[i_event] != "IMU") continue;
        if (c[i_mapped].empty() || c[i_gx].empty() || c[i_gy].empty() || c[i_gz].empty()) continue;
        try {
            ImuSample s;
            s.mapped_ns = std::stoll(c[i_mapped]);
            s.gx = std::stod(c[i_gx]);
            s.gy = std::stod(c[i_gy]);
            s.gz = std::stod(c[i_gz]);
            out.push_back(s);
        } catch (...) {}
    }
    std::sort(out.begin(), out.end(), [](const auto& a, const auto& b) {
        return a.mapped_ns < b.mapped_ns;
    });
    return out;
}

static cv::Mat readJpeg(std::ifstream& mjpeg, const CameraIndex& idx) {
    std::vector<uchar> data(idx.bytes);
    mjpeg.clear();
    mjpeg.seekg(static_cast<std::streamoff>(idx.offset), std::ios::beg);
    if (!mjpeg.read(reinterpret_cast<char*>(data.data()), idx.bytes)) return {};
    return cv::imdecode(data, cv::IMREAD_GRAYSCALE);
}

static std::vector<VisualSample> computeVisualYaw(
    const std::string& mjpeg_path,
    const std::vector<CameraIndex>& cam,
    VisualGapStats* gap_stats) {

    std::ifstream mjpeg(mjpeg_path, std::ios::binary);
    if (!mjpeg) throw std::runtime_error("Cannot open MJPEG: " + mjpeg_path);

    std::vector<VisualSample> out;
    if (cam.size() < 2) return out;

    cv::Mat prev = readJpeg(mjpeg, cam[0]);
    if (prev.empty()) throw std::runtime_error("Cannot decode first JPEG");

    std::vector<cv::Point2f> prev_pts;
    cv::goodFeaturesToTrack(prev, prev_pts, 350, 0.005, 12.0);

    constexpr int64_t kMaxVisualPairDtNs = 20'000'000LL;

    for (size_t i = 1; i < cam.size(); ++i) {
        cv::Mat cur = readJpeg(mjpeg, cam[i]);
        if (cur.empty()) {
            prev.release();
            continue;
        }

        const int64_t pair_dt_ns = cam[i].timestamp_ns - cam[i - 1].timestamp_ns;
        const uint32_t seq_delta = cam[i].sequence - cam[i - 1].sequence;
        const bool sequence_ok = (seq_delta == 1U);
        const bool timestamp_ok = pair_dt_ns > 0 && pair_dt_ns <= kMaxVisualPairDtNs;

        if (!sequence_ok || !timestamp_ok) {
            if (gap_stats) {
                ++gap_stats->rejected_pairs;
                if (!sequence_ok) {
                    ++gap_stats->sequence_gaps;
                    if (seq_delta > 1U)
                        gap_stats->source_frames_missing += seq_delta - 1U;
                }
                if (!timestamp_ok) ++gap_stats->timestamp_gaps;
                if (pair_dt_ns > 0) {
                    gap_stats->max_rejected_dt_ms = std::max(
                        gap_stats->max_rejected_dt_ms,
                        static_cast<double>(pair_dt_ns) / 1.0e6);
                }
            }

            prev = cur;
            prev_pts.clear();
            cv::goodFeaturesToTrack(prev, prev_pts, 350, 0.005, 12.0);
            continue;
        }

        if (prev.empty()) {
            prev = cur;
            cv::goodFeaturesToTrack(prev, prev_pts, 350, 0.005, 12.0);
            continue;
        }

        if (prev_pts.size() < 50)
            cv::goodFeaturesToTrack(prev, prev_pts, 350, 0.005, 12.0);

        std::vector<cv::Point2f> next_pts;
        std::vector<uchar> status;
        std::vector<float> err;

        if (!prev_pts.empty()) {
            cv::calcOpticalFlowPyrLK(
                prev, cur, prev_pts, next_pts, status, err,
                cv::Size(21, 21), 3,
                cv::TermCriteria(cv::TermCriteria::COUNT |
                                 cv::TermCriteria::EPS, 30, 0.01));
        }

        std::vector<cv::Point2f> a, b;
        for (size_t k = 0; k < status.size(); ++k) {
            if (status[k] && std::isfinite(next_pts[k].x) &&
                std::isfinite(next_pts[k].y)) {
                a.push_back(prev_pts[k]);
                b.push_back(next_pts[k]);
            }
        }

        double omega = std::numeric_limits<double>::quiet_NaN();
        int inliers_count = 0;

        if (a.size() >= 20) {
            cv::Mat inliers;
            cv::Mat M = cv::estimateAffinePartial2D(
                a, b, inliers, cv::RANSAC, 2.0, 2000, 0.995, 10);

            if (!M.empty()) {
                for (int r = 0; r < inliers.rows; ++r)
                    if (inliers.at<uchar>(r, 0)) ++inliers_count;

                const double angle =
                    std::atan2(M.at<double>(1, 0), M.at<double>(0, 0));
                const double dt = pair_dt_ns * 1e-9;

                if (dt > 0.003 && inliers_count >= 15)
                    omega = angle / dt;
            }
        }

        if (std::isfinite(omega)) {
            VisualSample s;
            s.timestamp_ns =
                (cam[i].timestamp_ns + cam[i - 1].timestamp_ns) / 2;
            s.omega = omega;
            s.tracked = static_cast<int>(a.size());
            s.inliers = inliers_count;
            out.push_back(s);
        }

        prev = cur;
        prev_pts = b;
        if (prev_pts.size() < 80)
            cv::goodFeaturesToTrack(prev, prev_pts, 350, 0.005, 12.0);
    }

    return out;
}

static bool interpGyro(const std::vector<ImuSample>& imu,
                       int axis,
                       int64_t t_ns,
                       double* out) {
    if (imu.size() < 2 || t_ns < imu.front().mapped_ns || t_ns > imu.back().mapped_ns)
        return false;

    auto it = std::lower_bound(
        imu.begin(), imu.end(), t_ns,
        [](const ImuSample& s, int64_t t) { return s.mapped_ns < t; });

    if (it == imu.begin()) {
        *out = axis == 0 ? it->gx : axis == 1 ? it->gy : it->gz;
        return true;
    }
    if (it == imu.end()) return false;

    const auto& b = *it;
    const auto& a = *(it - 1);
    const double denom = static_cast<double>(b.mapped_ns - a.mapped_ns);
    if (denom <= 0.0) return false;
    const double f = static_cast<double>(t_ns - a.mapped_ns) / denom;
    const double va = axis == 0 ? a.gx : axis == 1 ? a.gy : a.gz;
    const double vb = axis == 0 ? b.gx : axis == 1 ? b.gy : b.gz;
    *out = va + f * (vb - va);
    return true;
}

static double corr(const std::vector<double>& a, const std::vector<double>& b) {
    if (a.size() != b.size() || a.size() < 3) return 0.0;
    const double ma = std::accumulate(a.begin(), a.end(), 0.0) / a.size();
    const double mb = std::accumulate(b.begin(), b.end(), 0.0) / b.size();
    double num = 0.0, da = 0.0, db = 0.0;
    for (size_t i = 0; i < a.size(); ++i) {
        const double xa = a[i] - ma;
        const double xb = b[i] - mb;
        num += xa * xb;
        da += xa * xa;
        db += xb * xb;
    }
    if (da <= 0.0 || db <= 0.0) return 0.0;
    return num / std::sqrt(da * db);
}

static SearchResult evaluateOffset(
    const std::vector<VisualSample>& visual,
    const std::vector<ImuSample>& imu,
    size_t begin,
    size_t end,
    int axis,
    double offset_ms) {

    std::vector<double> v, g;
    v.reserve(end - begin);
    g.reserve(end - begin);
    const int64_t offset_ns = static_cast<int64_t>(std::llround(offset_ms * 1.0e6));

    for (size_t i = begin; i < end; ++i) {
        double gyro = 0.0;
        if (!interpGyro(imu, axis, visual[i].timestamp_ns - offset_ns, &gyro)) continue;
        v.push_back(visual[i].omega);
        g.push_back(gyro);
    }

    SearchResult r;
    r.axis = axis;
    r.offset_ms = offset_ms;
    r.count = static_cast<int>(v.size());
    r.corr = corr(v, g);
    return r;
}

static SearchResult searchOffset(
    const std::vector<VisualSample>& visual,
    const std::vector<ImuSample>& imu,
    size_t begin,
    size_t end,
    int forced_axis,
    double lo_ms,
    double hi_ms,
    double step_ms) {

    SearchResult best;
    double best_abs = -1.0;
    const int a0 = forced_axis >= 0 ? forced_axis : 0;
    const int a1 = forced_axis >= 0 ? forced_axis : 2;

    for (int axis = a0; axis <= a1; ++axis) {
        for (double off = lo_ms; off <= hi_ms + step_ms * 0.25; off += step_ms) {
            SearchResult r = evaluateOffset(visual, imu, begin, end, axis, off);
            const double ac = std::abs(r.corr);
            if (r.count >= 20 && ac > best_abs) {
                best_abs = ac;
                best = r;
            }
        }
    }
    return best;
}

static std::vector<Segment> detectSegmentsRobust(
    const std::vector<VisualSample>& visual) {

    constexpr double threshold = 0.18;
    constexpr double merge_gap_s = 0.35;
    constexpr double margin_s = 0.25;

    std::vector<std::pair<size_t, size_t>> raw;
    bool active = false;
    size_t start = 0;
    for (size_t i = 0; i < visual.size(); ++i) {
        const bool now = std::abs(visual[i].omega) >= threshold;
        if (now && !active) { start = i; active = true; }
        if (!now && active) { raw.push_back({start, i}); active = false; }
    }
    if (active) raw.push_back({start, visual.size()});
    if (raw.empty()) return {};

    std::vector<std::pair<size_t, size_t>> merged;
    for (const auto& r : raw) {
        if (merged.empty()) {
            merged.push_back(r);
            continue;
        }
        const double gap_s =
            (visual[r.first].timestamp_ns -
             visual[merged.back().second - 1].timestamp_ns) * 1e-9;
        if (gap_s <= merge_gap_s) merged.back().second = r.second;
        else merged.push_back(r);
    }

    std::vector<Segment> out;
    const int64_t margin_ns = static_cast<int64_t>(margin_s * 1e9);
    for (const auto& r : merged) {
        const int64_t target_b = visual[r.first].timestamp_ns - margin_ns;
        const int64_t target_e = visual[r.second - 1].timestamp_ns + margin_ns;
        size_t b = r.first;
        size_t e = r.second;
        while (b > 0 && visual[b - 1].timestamp_ns >= target_b) --b;
        while (e < visual.size() && visual[e].timestamp_ns <= target_e) ++e;
        if (e <= b) continue;

        Segment s;
        s.begin = b;
        s.end = e;
        s.start_ns = visual[b].timestamp_ns;
        s.end_ns = visual[e - 1].timestamp_ns;
        s.duration_s = (s.end_ns - s.start_ns) * 1e-9;
        for (size_t i = b; i < e; ++i)
            s.peak_abs_omega = std::max(s.peak_abs_omega, std::abs(visual[i].omega));
        out.push_back(s);
    }
    return out;
}

static void writeLagCurve(
    const std::string& path,
    const std::vector<VisualSample>& visual,
    const std::vector<ImuSample>& imu,
    int axis,
    double center_ms) {
    std::ofstream f(path);
    if (!f) return;
    f << "offset_ms,correlation,samples\n";
    f << std::fixed << std::setprecision(6);
    for (double off = center_ms - 20.0; off <= center_ms + 20.0001; off += 0.1) {
        const auto r = evaluateOffset(visual, imu, 0, visual.size(), axis, off);
        f << off << ',' << r.corr << ',' << r.count << '\n';
    }
}

static const char* axisName(int axis) {
    return axis == 0 ? "X" : axis == 1 ? "Y" : "Z";
}

int main() {
    try {
        const std::string combined = "/home/vio/camera_imu_yaw.csv";
        const std::string camera = "/home/vio/camera_imu_yaw_camera.csv";
        const std::string mjpeg = "/home/vio/camera_imu_yaw.mjpg";

        std::cout << "=== CAMERA / IMU YAW OFFSET ANALYZER V4 ===\n";
        std::cout << "combined: " << combined << '\n';
        std::cout << "camera:   " << camera << '\n';
        std::cout << "mjpeg:    " << mjpeg << "\n\n";

        const auto cam = readCameraIndex(camera);
        const auto imu = readImu(combined);
        std::cout << "camera frames indexed: " << cam.size() << '\n';
        std::cout << "IMU samples:           " << imu.size() << '\n';

        VisualGapStats gap_stats;
        const auto visual = computeVisualYaw(mjpeg, cam, &gap_stats);
        std::cout << "visual yaw samples:    " << visual.size() << '\n';
        std::cout << "rejected gap pairs:    " << gap_stats.rejected_pairs << '\n';
        std::cout << "sequence gap pairs:    " << gap_stats.sequence_gaps << '\n';
        std::cout << "timestamp gap pairs:   " << gap_stats.timestamp_gaps << '\n';
        std::cout << "source frames missing: " << gap_stats.source_frames_missing << '\n';
        std::cout << "max rejected dt:       " << std::fixed << std::setprecision(3)
                  << gap_stats.max_rejected_dt_ms << " ms\n";

        if (visual.size() < 100)
            throw std::runtime_error("Too few valid visual yaw samples");

        std::vector<double> inliers;
        inliers.reserve(visual.size());
        for (const auto& s : visual) inliers.push_back(s.inliers);
        std::cout << "visual inliers median: " << std::setprecision(0) << median(inliers) << '\n';
        std::cout << "visual inliers p05:    " << percentile(inliers, 5.0) << '\n';

        const auto coarse = searchOffset(visual, imu, 0, visual.size(), -1,
                                         -100.0, 100.0, 1.0);
        const auto fine = searchOffset(visual, imu, 0, visual.size(), coarse.axis,
                                       coarse.offset_ms - 5.0,
                                       coarse.offset_ms + 5.0, 0.05);

        std::cout << "\n============================================================\n";
        std::cout << "GLOBAL ALIGNMENT\n";
        std::cout << "============================================================\n";
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "gyro axis:             " << axisName(fine.axis) << '\n';
        std::cout << "axis sign:             " << (fine.corr < 0 ? "-" : "+") << '\n';
        std::cout << "camera-IMU offset:     " << fine.offset_ms << " ms\n";
        std::cout << "correlation:           " << fine.corr << '\n';
        std::cout << "samples used:          " << fine.count << '\n';

        auto segments = detectSegmentsRobust(visual);
        std::cout << "\ndetected yaw segments: " << segments.size() << "\n\n";

        std::vector<SegmentResult> accepted;
        std::ofstream seg_csv("/home/vio/camera_imu_yaw_segments.csv");
        seg_csv << "segment,start_ns,end_ns,duration_s,peak_rad_s,offset_ms,correlation,samples,accepted\n";

        int idx = 0;
        for (const auto& seg : segments) {
            ++idx;
            auto r = searchOffset(visual, imu, seg.begin, seg.end, fine.axis,
                                  fine.offset_ms - 15.0,
                                  fine.offset_ms + 15.0, 0.1);
            const bool ok = r.count >= 40 && std::abs(r.corr) >= 0.70 &&
                            seg.peak_abs_omega >= 0.25;
            std::cout << "Segment " << idx
                      << ": duration=" << seg.duration_s << " s"
                      << " peak=" << seg.peak_abs_omega << " rad/s "
                      << " offset=" << r.offset_ms << " ms "
                      << " corr=" << r.corr
                      << " n=" << r.count
                      << (ok ? "  ACCEPT" : "  REJECT") << '\n';
            seg_csv << idx << ',' << seg.start_ns << ',' << seg.end_ns << ','
                    << seg.duration_s << ',' << seg.peak_abs_omega << ','
                    << r.offset_ms << ',' << r.corr << ',' << r.count << ','
                    << (ok ? 1 : 0) << '\n';
            if (ok) accepted.push_back({idx, seg, r});
        }

        std::cout << "\n============================================================\n";
        std::cout << "SEGMENT STATISTICS\n";
        std::cout << "============================================================\n";
        std::cout << "accepted segments:     " << accepted.size() << '\n';

        if (!accepted.empty()) {
            std::vector<double> offsets, abs_corr;
            for (const auto& x : accepted) {
                offsets.push_back(x.best.offset_ms);
                abs_corr.push_back(std::abs(x.best.corr));
            }
            const double med = median(offsets);
            std::vector<double> dev;
            for (double x : offsets) dev.push_back(std::abs(x - med));
            const double mad = median(dev);
            std::cout << std::fixed << std::setprecision(3);
            std::cout << "offset median:         " << med << " ms\n";
            std::cout << "offset MAD:            " << mad << " ms\n";
            std::cout << "offset P16..P84:       " << percentile(offsets, 16)
                      << " .. " << percentile(offsets, 84) << " ms\n";
            std::cout << "offset P05..P95:       " << percentile(offsets, 5)
                      << " .. " << percentile(offsets, 95) << " ms\n";
            std::cout << "abs(corr) median:      " << median(abs_corr) << '\n';
            std::cout << "\nGlobal-vs-segment delta: " << fine.offset_ms - med << " ms\n";

            const bool stable = accepted.size() >= 3 && mad <= 3.0 && median(abs_corr) >= 0.80;
            std::cout << "\nRESULT: "
                      << (stable ? "offset estimate is stable."
                                 : "offset detected, but stability is not yet strong enough.")
                      << '\n';
        }

        std::ofstream trace("/home/vio/camera_imu_yaw_visual_v2.csv");
        trace << "timestamp_ns,visual_omega_rad_s,tracked,inliers\n";
        trace << std::fixed << std::setprecision(9);
        for (const auto& s : visual)
            trace << s.timestamp_ns << ',' << s.omega << ',' << s.tracked << ',' << s.inliers << '\n';

        const std::string lag_path = "/home/vio/camera_imu_yaw_lag_curve.csv";
        writeLagCurve(lag_path, visual, imu, fine.axis, fine.offset_ms);

        std::cout << "\nFiles:\n";
        std::cout << "  /home/vio/camera_imu_yaw_segments.csv\n";
        std::cout << "  " << lag_path << '\n';
        std::cout << "  /home/vio/camera_imu_yaw_visual_v2.csv\n";
        std::cout << "\nGap policy:\n";
        std::cout << "  reject pair if V4L2 sequence is not contiguous\n";
        std::cout << "  reject pair if corrected camera dt <= 0 or > 20 ms\n";
        std::cout << "  reset LK tracking after every rejected pair\n";
        std::cout << "\nOffset convention:\n";
        std::cout << "  positive = camera measurement later than IMU\n";
        std::cout << "  negative = camera measurement earlier than IMU\n";

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << '\n';
        return 1;
    }
}
