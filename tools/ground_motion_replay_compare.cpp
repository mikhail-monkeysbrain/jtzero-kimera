// JT-Zero Ground Motion — детерминированный offline replay.
// Сравнивает текущую политику prev и диагностическую ANCHOR-политику
// на ОДНОЙ И ТОЙ ЖЕ записи. Production-код и пороги не меняет.
#define main jtzero_ground_motion_replay_base_unused_main
#include "ground_motion_live_v2_v3_ab.cpp"
#undef main

#include <filesystem>
#include <map>
#include <set>
#include <sstream>

namespace {

struct Meta {
    uint64_t frame = 0;
    int64_t mono_ns = 0;
    int64_t ts_ns = 0;
    uint64_t offset = 0;
    uint64_t size = 0;
    bool decode_ok = false;
    bool sensors = false;
    Attitude att{};
    double height = 0.0;
};

struct StatePoint {
    int64_t mono_ns = 0;
    double x = 0.0, y = 0.0;
};

struct StepDiag {
    bool valid = false;
    std::string reason = "UNKNOWN";
    double dt = 0.0;
    int features = 0;
    int tracked = 0;
    int hom_inliers = 0;
    int inliers = 0;
    double scatter = 0.0;
    double quality = 0.0;
    double dx = 0.0, dy = 0.0;
};

struct RefFrame {
    cv::Mat gray;
    Meta m{};
    bool set = false;
};

enum class Policy { CURRENT, ANCHOR };
enum class MaskMode { NONE, DROP, REJECT };

struct Scenario {
    std::string name;
    Policy policy = Policy::CURRENT;
    MaskMode mask_mode = MaskMode::NONE;
    std::set<size_t> masked;
};

struct RunResult {
    std::string name;
    double dx = NAN, dy = NAN, dist = NAN;
    std::map<std::string, int> reasons;
};

std::vector<std::string> splitCsv(const std::string& s) {
    std::vector<std::string> out;
    std::stringstream ss(s);
    std::string x;
    while (std::getline(ss, x, ',')) out.push_back(x);
    return out;
}

std::vector<Meta> readMeta(const std::filesystem::path& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("не удалось открыть frames.csv");
    std::string line;
    std::getline(f, line);
    std::vector<Meta> out;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        auto c = splitCsv(line);
        if (c.size() < 16) throw std::runtime_error("frames.csv: строка короче 16 колонок");
        Meta m;
        m.frame = std::stoull(c[0]);
        m.mono_ns = std::stoll(c[1]);
        m.ts_ns = std::stoll(c[2]);
        m.offset = std::stoull(c[3]);
        m.size = std::stoull(c[4]);
        m.decode_ok = std::stoi(c[5]) != 0;
        m.att.valid = std::stoi(c[9]) != 0;
        m.att.recv_ns = m.ts_ns;
        m.att.roll = std::stod(c[11]);
        m.att.pitch = std::stod(c[12]);
        m.att.yaw = std::stod(c[13]);
        m.height = std::stod(c[14]);
        m.sensors = std::stoi(c[15]) != 0;
        out.push_back(m);
    }
    return out;
}

struct Events {
    std::map<std::string, int64_t> t;
    std::map<std::string, std::string> note;
};

Events readEvents(const std::filesystem::path& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("не удалось открыть events.csv");
    Events e;
    std::string line;
    std::getline(f, line);
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        size_t a = line.find(',');
        size_t b = (a == std::string::npos) ? std::string::npos : line.find(',', a + 1);
        if (a == std::string::npos || b == std::string::npos) continue;
        const int64_t ts = std::stoll(line.substr(0, a));
        const std::string name = line.substr(a + 1, b - a - 1);
        const std::string note = line.substr(b + 1);
        e.t[name] = ts;
        e.note[name] = note;
    }
    return e;
}

cv::Mat loadGray(std::ifstream& bin, const Meta& m) {
    if (!m.decode_ok || m.size == 0) return {};
    std::vector<uint8_t> raw((size_t)m.size);
    bin.clear();
    bin.seekg((std::streamoff)m.offset, std::ios::beg);
    bin.read(reinterpret_cast<char*>(raw.data()), (std::streamsize)raw.size());
    if ((size_t)bin.gcount() != raw.size()) throw std::runtime_error("frames.mjpgbin: короткое чтение");
    cv::Mat one(1, (int)raw.size(), CV_8UC1, raw.data());
    return cv::imdecode(one, cv::IMREAD_GRAYSCALE);
}

bool v3StepReplay(const std::vector<cv::Point2f>& ai,
                  const std::vector<cv::Point2f>& bi,
                  const CameraCalib& c,
                  const Attitude& a0, double h0,
                  const Attitude& a1, double h1,
                  cv::Vec2d* out, int* used, double* scatter) {
    std::vector<cv::Point2f> au, bu;
    cv::undistortPoints(ai, au, c.K, c.D);
    cv::undistortPoints(bi, bu, c.K, c.D);
    const auto R0 = attitudeFluToNwu(a0) * c.B_R_C;
    const auto R1 = attitudeFluToNwu(a1) * c.B_R_C;
    std::vector<cv::Vec2d> d;
    d.reserve(au.size());
    for (size_t i = 0; i < au.size(); ++i) {
        cv::Vec2d g0, g1;
        if (footprint(au[i], R0, h0, &g0) && footprint(bu[i], R1, h1, &g1)) d.push_back(g0 - g1);
    }
    return robustDelta(d, out, used, scatter);
}

StepDiag calcStep(const RefFrame& ref, const cv::Mat& gray, const Meta& cur, const CameraCalib& calib) {
    StepDiag s;
    s.dt = (cur.ts_ns - ref.m.ts_ns) * 1e-9;
    if (!(s.dt > 0.0 && s.dt < 0.2)) { s.reason = "DT"; return s; }

    std::vector<cv::Point2f> p0, p1;
    cv::goodFeaturesToTrack(ref.gray, p0, 700, 0.01, 7);
    s.features = (int)p0.size();
    if (p0.size() < 30) { s.reason = "FEATURES"; return s; }

    std::vector<uchar> st;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(ref.gray, gray, p0, p1, st, err, {21, 21}, 3);
    std::vector<cv::Point2f> a, b;
    for (size_t i = 0; i < p0.size(); ++i) {
        if (st[i]) { a.push_back(p0[i]); b.push_back(p1[i]); }
    }
    s.tracked = (int)a.size();
    if (a.size() < 20) { s.reason = "LK"; return s; }

    cv::Mat mask;
    cv::findHomography(a, b, cv::RANSAC, 2.0, mask);
    if (mask.empty()) { s.reason = "HOMOGRAPHY"; return s; }

    std::vector<cv::Point2f> ai, bi;
    for (size_t i = 0; i < a.size(); ++i) {
        if (mask.at<uchar>((int)i)) { ai.push_back(a[i]); bi.push_back(b[i]); }
    }
    s.hom_inliers = (int)ai.size();
    if (ai.size() < 15) { s.reason = "HOM_INLIERS"; return s; }

    cv::Vec2d d;
    if (!v3StepReplay(ai, bi, calib, ref.m.att, ref.m.height, cur.att, cur.height,
                      &d, &s.inliers, &s.scatter)) {
        s.reason = "V3";
        return s;
    }
    s.dx = d[0]; s.dy = d[1];
    if (cv::norm(d) >= 0.20) { s.reason = "STEP_CAP"; return s; }

    s.quality = std::clamp((s.inliers / 150.0) * std::exp(-s.scatter / 0.003), 0.0, 1.0);
    if (s.inliers < 20) { s.reason = "INLIERS"; return s; }
    if (s.scatter >= 0.004) { s.reason = "SCATTER"; return s; }
    if (s.quality < 0.15) { s.reason = "QUALITY"; return s; }

    s.valid = true;
    s.reason = "VALID";
    return s;
}

std::pair<double,double> medianState(const std::vector<StatePoint>& states, int64_t a, int64_t b) {
    std::vector<double> xs, ys;
    for (const auto& s : states) {
        if (s.mono_ns >= a && s.mono_ns <= b) { xs.push_back(s.x); ys.push_back(s.y); }
    }
    if (xs.empty()) return {NAN, NAN};
    return {median(xs), median(ys)};
}

std::set<size_t> makeBurst(const std::vector<Meta>& meta, int64_t move_a, int64_t move_b,
                           double frac, int n) {
    std::vector<size_t> idx;
    for (size_t i = 0; i < meta.size(); ++i) {
        if (meta[i].mono_ns >= move_a && meta[i].mono_ns <= move_b) idx.push_back(i);
    }
    if (idx.empty()) throw std::runtime_error("в MOVE interval нет кадров");
    size_t center_pos = (size_t)std::llround(frac * double(idx.size() - 1));
    int first = (int)center_pos - n / 2;
    if (first < 0) first = 0;
    if (first + n > (int)idx.size()) first = std::max(0, (int)idx.size() - n);
    std::set<size_t> out;
    for (int k = 0; k < n; ++k) out.insert(idx[(size_t)(first + k)]);
    return out;
}

RunResult runScenario(const Scenario& sc,
                      const std::vector<Meta>& meta,
                      const CameraCalib& calib,
                      const std::filesystem::path& bin_path,
                      int64_t move_start, int64_t move_end,
                      std::ofstream& detail) {
    std::ifstream bin(bin_path, std::ios::binary);
    if (!bin) throw std::runtime_error("не удалось открыть frames.mjpgbin");

    RefFrame ref;
    double x = 0.0, y = 0.0;
    std::vector<StatePoint> states;
    states.reserve(meta.size());
    RunResult rr; rr.name = sc.name;

    for (size_t i = 0; i < meta.size(); ++i) {
        const Meta& m = meta[i];
        const bool masked = sc.masked.count(i) != 0;
        if (sc.mask_mode == MaskMode::DROP && masked) {
            rr.reasons["SYNTH_DROP"]++;
            detail << sc.name << ',' << m.frame << ",SYNTH_DROP,1,0,0,0,0,0,0,0,0,0," << x << ',' << y << '\n';
            states.push_back({m.mono_ns, x, y});
            continue;
        }

        cv::Mat gray = loadGray(bin, m);
        if (gray.empty()) {
            rr.reasons["DECODE"]++;
            detail << sc.name << ',' << m.frame << ",DECODE,0,0,0,0,0,0,0,0,0,0," << x << ',' << y << '\n';
            states.push_back({m.mono_ns, x, y});
            continue; // production: prev сохраняется при decode failure
        }

        if (!m.sensors) {
            rr.reasons["SENSORS"]++;
            detail << sc.name << ',' << m.frame << ",SENSORS,0,0,0,0,0,0,0,0,0,0," << x << ',' << y << '\n';
            ref = RefFrame{}; // production очищает prev при sensors=false
            states.push_back({m.mono_ns, x, y});
            continue;
        }

        if (!ref.set) {
            ref.gray = gray.clone(); ref.m = m; ref.set = true;
            rr.reasons["NO_PREV"]++;
            detail << sc.name << ',' << m.frame << ",NO_PREV,0,0,0,0,0,0,0,0,0,0," << x << ',' << y << '\n';
            states.push_back({m.mono_ns, x, y});
            continue;
        }

        StepDiag sd = calcStep(ref, gray, m, calib);
        const bool forced_reject = (sc.mask_mode == MaskMode::REJECT && masked);
        const bool accepted = sd.valid && !forced_reject;
        std::string reason = forced_reject && sd.valid ? "SYNTH_REJECT" : sd.reason;

        if (accepted) {
            x += sd.dx; y += sd.dy;
        }

        if (sc.policy == Policy::CURRENT) {
            // Точное production-поведение: при sensors=true prev всегда становится текущим,
            // даже когда visual step invalid/forced-reject.
            ref.gray = gray.clone(); ref.m = m; ref.set = true;
        } else {
            // ANCHOR: ref меняется только после учтённого displacement.
            if (accepted) {
                ref.gray = gray.clone(); ref.m = m; ref.set = true;
            } else if (!(sd.dt > 0.0 && sd.dt < 0.2)) {
                // Не расширяем production dt. Если anchor устарел — честно теряем участок
                // и начинаем новую опору без интеграции.
                reason = "ANCHOR_EXPIRED";
                ref.gray = gray.clone(); ref.m = m; ref.set = true;
            }
        }

        rr.reasons[reason]++;
        detail << sc.name << ',' << m.frame << ',' << reason << ',' << (forced_reject ? 1 : 0) << ','
               << (accepted ? 1 : 0) << ',' << sd.dt << ',' << sd.features << ',' << sd.tracked << ','
               << sd.hom_inliers << ',' << sd.inliers << ',' << sd.scatter << ',' << sd.quality << ','
               << sd.dx << ',' << sd.dy << ',' << x << ',' << y << '\n';
        states.push_back({m.mono_ns, x, y});
    }

    const int64_t W = 500000000LL;
    const auto pre = medianState(states, move_start - W, move_start);
    const auto post = medianState(states, move_end, move_end + W);
    rr.dx = post.first - pre.first;
    rr.dy = post.second - pre.second;
    rr.dist = std::hypot(rr.dx, rr.dy);
    return rr;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 6) {
        std::cerr << "Использование: " << argv[0]
                  << " <dataset_dir> <camera_yaml> <events.csv> <summary.csv> <detail.csv>\n";
        return 2;
    }

    try {
        const std::filesystem::path dir = argv[1];
        const std::string yaml = argv[2];
        const std::filesystem::path events_path = argv[3];
        const std::filesystem::path summary_path = argv[4];
        const std::filesystem::path detail_path = argv[5];

        const auto meta = readMeta(dir / "frames.csv");
        if (meta.size() < 20) throw std::runtime_error("слишком мало кадров в dataset");
        const auto ev = readEvents(events_path);
        if (!ev.t.count("MOVE_START") || !ev.t.count("MOVE_END")) {
            throw std::runtime_error("events.csv: нужны MOVE_START и MOVE_END");
        }
        const int64_t move_start = ev.t.at("MOVE_START");
        const int64_t move_end = ev.t.at("MOVE_END");
        double target_mm = NAN;
        if (ev.note.count("MOVE_TARGET_MM")) {
            try { target_mm = std::stod(ev.note.at("MOVE_TARGET_MM")); } catch (...) {}
        }

        const CameraCalib calib = loadCameraCalib(yaml);
        cv::setNumThreads(1);

        std::vector<Scenario> scenarios;
        scenarios.push_back({"BASELINE_CURRENT", Policy::CURRENT, MaskMode::NONE, {}});
        scenarios.push_back({"BASELINE_ANCHOR", Policy::ANCHOR, MaskMode::NONE, {}});

        struct Spec { double frac; int n; const char* tag; };
        const std::vector<Spec> specs = {
            {0.25, 3, "P25_N3"},
            {0.50, 1, "P50_N1"},
            {0.50, 3, "P50_N3"},
            {0.50, 5, "P50_N5"},
            {0.75, 3, "P75_N3"},
        };
        for (const auto& sp : specs) {
            auto mask = makeBurst(meta, move_start, move_end, sp.frac, sp.n);
            scenarios.push_back({std::string("DROP_CURRENT_") + sp.tag, Policy::CURRENT, MaskMode::DROP, mask});
            scenarios.push_back({std::string("REJECT_CURRENT_") + sp.tag, Policy::CURRENT, MaskMode::REJECT, mask});
            scenarios.push_back({std::string("REJECT_ANCHOR_") + sp.tag, Policy::ANCHOR, MaskMode::REJECT, mask});
        }

        std::ofstream summary(summary_path, std::ios::trunc);
        std::ofstream detail(detail_path, std::ios::trunc);
        if (!summary || !detail) throw std::runtime_error("не удалось создать replay CSV");
        summary << "scenario,dx_m,dy_m,dist_m,target_mm,dist_error_mm,vector_error_vs_baseline_mm,valid_count,anchor_expired,synthetic_drop,synthetic_reject\n";
        detail << "scenario,frame,reason,forced_reject,accepted,dt_s,features,tracked,hom_inliers,inliers,scatter_m,quality,dx_m,dy_m,x_m,y_m\n";

        std::vector<RunResult> results;
        results.reserve(scenarios.size());
        double base_dx = NAN, base_dy = NAN;

        std::cout << "===== DETERMINISTIC GROUND MOTION REPLAY =====\n";
        std::cout << "frames=" << meta.size() << " target=" << target_mm << " mm\n";
        std::cout << "scenario                         |d|mm   errTarget   vecErrBase  valid  anchorExp\n";

        for (const auto& sc : scenarios) {
            RunResult r = runScenario(sc, meta, calib, dir / "frames.mjpgbin",
                                      move_start, move_end, detail);
            if (sc.name == "BASELINE_CURRENT") { base_dx = r.dx; base_dy = r.dy; }
            const double dist_mm = r.dist * 1000.0;
            const double err_target = std::isfinite(target_mm) ? std::abs(dist_mm - target_mm) : NAN;
            const double vec_err = std::isfinite(base_dx) ? std::hypot(r.dx - base_dx, r.dy - base_dy) * 1000.0 : NAN;
            summary << r.name << ',' << r.dx << ',' << r.dy << ',' << r.dist << ',' << target_mm << ','
                    << err_target << ',' << vec_err << ',' << r.reasons["VALID"] << ','
                    << r.reasons["ANCHOR_EXPIRED"] << ',' << r.reasons["SYNTH_DROP"] << ','
                    << r.reasons["SYNTH_REJECT"] << '\n';
            std::cout << std::left << std::setw(32) << r.name << std::right
                      << std::setw(8) << std::fixed << std::setprecision(1) << dist_mm
                      << std::setw(12) << err_target
                      << std::setw(12) << vec_err
                      << std::setw(7) << r.reasons["VALID"]
                      << std::setw(11) << r.reasons["ANCHOR_EXPIRED"] << '\n';
            results.push_back(std::move(r));
        }

        std::cout << "\nSUMMARY: " << summary_path << "\nDETAIL:  " << detail_path << "\n";
        std::cout << "Важно: DROP и REJECT — разные эксперименты. DROP имитирует отсутствующий кадр; "
                     "REJECT — пришедший/декодированный кадр, displacement которого не принят.\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "ОШИБКА: " << e.what() << "\n";
        return 1;
    }
}
