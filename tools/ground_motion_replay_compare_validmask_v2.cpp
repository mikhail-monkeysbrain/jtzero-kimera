// JT-Zero Ground Motion — deterministic replay с масками только по реально valid production-шагам.
// Production-код не меняет. Все зависимости заранее подключаются в global namespace,
// после чего legacy replay включается внутрь jtzero_replay_base без создания вложенного std.
#include <linux/videodev2.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <termios.h>
#include <unistd.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include "common/mavlink.h"

namespace jtzero_replay_base {
#include "ground_motion_replay_compare.cpp"
}
using namespace jtzero_replay_base;

namespace {

std::vector<size_t> collectBaselineValidMoveSteps(const std::vector<Meta>& meta,
                                                  const CameraCalib& calib,
                                                  const std::filesystem::path& bin_path,
                                                  int64_t move_start,
                                                  int64_t move_end) {
    std::ifstream bin(bin_path, std::ios::binary);
    if (!bin) throw std::runtime_error("не удалось открыть frames.mjpgbin");

    RefFrame ref;
    std::vector<size_t> out;
    for (size_t i = 0; i < meta.size(); ++i) {
        const Meta& m = meta[i];
        cv::Mat gray = loadGray(bin, m);
        if (gray.empty()) {
            continue;
        }
        if (!m.sensors) {
            ref = RefFrame{};
            continue;
        }
        if (!ref.set) {
            ref.gray = gray.clone();
            ref.m = m;
            ref.set = true;
            continue;
        }

        const StepDiag sd = calcStep(ref, gray, m, calib);
        if (sd.valid && m.mono_ns >= move_start && m.mono_ns <= move_end) {
            out.push_back(i);
        }

        ref.gray = gray.clone();
        ref.m = m;
        ref.set = true;
    }
    return out;
}

std::set<size_t> makeValidBurst(const std::vector<size_t>& valid_idx,
                                const std::vector<Meta>& meta,
                                int64_t move_start,
                                int64_t move_end,
                                double frac,
                                int n) {
    if (valid_idx.empty()) throw std::runtime_error("в MOVE нет baseline-valid visual steps");
    const int64_t target_t = move_start + (int64_t)std::llround(frac * double(move_end - move_start));

    bool found = false;
    size_t best_pos = 0;
    int64_t best_dt = std::numeric_limits<int64_t>::max();

    if (n == 1) {
        for (size_t k = 0; k < valid_idx.size(); ++k) {
            const int64_t d = std::llabs(meta[valid_idx[k]].mono_ns - target_t);
            if (d < best_dt) { best_dt = d; best_pos = k; found = true; }
        }
    } else {
        for (size_t k = 0; k + (size_t)n <= valid_idx.size(); ++k) {
            bool consecutive = true;
            for (int j = 1; j < n; ++j) {
                if (valid_idx[k + (size_t)j] != valid_idx[k] + (size_t)j) {
                    consecutive = false;
                    break;
                }
            }
            if (!consecutive) continue;
            const size_t mid = k + (size_t)(n / 2);
            const int64_t d = std::llabs(meta[valid_idx[mid]].mono_ns - target_t);
            if (d < best_dt) { best_dt = d; best_pos = k; found = true; }
        }
    }

    if (!found) {
        throw std::runtime_error("не найдена серия из " + std::to_string(n) +
                                 " последовательных baseline-valid MOVE шагов");
    }

    std::set<size_t> out;
    for (int j = 0; j < n; ++j) out.insert(valid_idx[best_pos + (size_t)j]);
    return out;
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

        const auto valid_move = collectBaselineValidMoveSteps(meta, calib, dir / "frames.mjpgbin",
                                                              move_start, move_end);
        if (valid_move.size() < 10) {
            throw std::runtime_error("слишком мало baseline-valid MOVE шагов: " +
                                     std::to_string(valid_move.size()));
        }

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
            auto mask = makeValidBurst(valid_move, meta, move_start, move_end, sp.frac, sp.n);
            scenarios.push_back({std::string("DROP_CURRENT_") + sp.tag, Policy::CURRENT, MaskMode::DROP, mask});
            scenarios.push_back({std::string("REJECT_CURRENT_") + sp.tag, Policy::CURRENT, MaskMode::REJECT, mask});
            scenarios.push_back({std::string("REJECT_ANCHOR_") + sp.tag, Policy::ANCHOR, MaskMode::REJECT, mask});
        }

        std::ofstream summary(summary_path, std::ios::trunc);
        std::ofstream detail(detail_path, std::ios::trunc);
        if (!summary || !detail) throw std::runtime_error("не удалось создать replay CSV");
        summary << "scenario,dx_m,dy_m,dist_m,target_mm,dist_error_mm,vector_error_vs_baseline_mm,valid_count,anchor_expired,synthetic_drop,synthetic_reject\n";
        detail << "scenario,frame,reason,forced_reject,accepted,dt_s,features,tracked,hom_inliers,inliers,scatter_m,quality,dx_m,dy_m,x_m,y_m\n";

        double base_dx = NAN, base_dy = NAN;
        std::cout << "===== DETERMINISTIC GROUND MOTION REPLAY =====\n";
        std::cout << "frames=" << meta.size() << " target=" << target_mm << " mm"
                  << " baseline-valid MOVE steps=" << valid_move.size() << "\n";
        std::cout << "scenario                         |d|mm   errTarget   vecErrBase  valid  anchorExp  synthReject\n";

        for (const auto& sc : scenarios) {
            RunResult r = runScenario(sc, meta, calib, dir / "frames.mjpgbin",
                                      move_start, move_end, detail);
            if (sc.name == "BASELINE_CURRENT") { base_dx = r.dx; base_dy = r.dy; }
            const double dist_mm = r.dist * 1000.0;
            const double err_target = std::isfinite(target_mm) ? std::abs(dist_mm - target_mm) : NAN;
            const double vec_err = std::isfinite(base_dx) ?
                std::hypot(r.dx - base_dx, r.dy - base_dy) * 1000.0 : NAN;
            summary << r.name << ',' << r.dx << ',' << r.dy << ',' << r.dist << ',' << target_mm << ','
                    << err_target << ',' << vec_err << ',' << r.reasons["VALID"] << ','
                    << r.reasons["ANCHOR_EXPIRED"] << ',' << r.reasons["SYNTH_DROP"] << ','
                    << r.reasons["SYNTH_REJECT"] << '\n';
            std::cout << std::left << std::setw(32) << r.name << std::right
                      << std::setw(8) << std::fixed << std::setprecision(1) << dist_mm
                      << std::setw(12) << err_target
                      << std::setw(12) << vec_err
                      << std::setw(7) << r.reasons["VALID"]
                      << std::setw(11) << r.reasons["ANCHOR_EXPIRED"]
                      << std::setw(13) << r.reasons["SYNTH_REJECT"] << '\n';
        }

        std::cout << "\nSUMMARY: " << summary_path << "\nDETAIL:  " << detail_path << "\n";
        std::cout << "Маски DROP/REJECT выбираются только среди baseline-valid MOVE шагов.\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "ОШИБКА: " << e.what() << "\n";
        return 1;
    }
}
