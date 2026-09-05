        const double dx = s.px-a.px;
        const double dy = s.py-a.py;
        const double dz = s.pz-a.pz;
        if (std::sqrt(dx*dx+dy*dy+dz*dz) > kWarmupMaxSpanM) stable.clear();
      }
      stable.push_back(s);
      if (static_cast<int>(stable.size()) > kWarmupStableStatesNeeded) {
        stable.erase(stable.begin());
      }
    }

    const double elapsed = elapsedSec(wall_ns);
    if (last_log_wall_ns == 0 || wall_ns-last_log_wall_ns >= kWarmupLogPeriodNs) {
      last_log_wall_ns = wall_ns;
      const double dpx = have_first ? (s.px-first.px)*1000.0 : 0.0;
      const double dpy = have_first ? (s.py-first.py)*1000.0 : 0.0;
      const double dpz = have_first ? (s.pz-first.pz)*1000.0 : 0.0;
      std::cout << std::fixed << std::setprecision(2)
                << "[WARMUP] t=" << elapsed << "s KF=" << s.keyframe
                << " stable=" << stable.size() << '/' << kWarmupStableStatesNeeded
                << " span=" << stableSpanM()*1000.0 << "mm"
                << " |V|=" << speed(s)*1000.0 << "mm/s"
                << " dP0=[" << dpx << ',' << dpy << ',' << dpz << "]mm"
                << " RPY=[" << s.roll_deg << ',' << s.pitch_deg << ',' << s.yaw_deg << "]\n";
    }

    if (elapsed >= kPostStartupWarmupSec &&
        static_cast<int>(stable.size()) >= kWarmupStableStatesNeeded &&
        stableSpanM() <= kWarmupMaxSpanM) {
      passed = true;
      const double dpx = (s.px-first.px)*1000.0;
      const double dpy = (s.py-first.py)*1000.0;
      const double dpz = (s.pz-first.pz)*1000.0;
      std::cout << std::fixed << std::setprecision(3)
                << "[WARMUP] PASS t=" << elapsed << "s KF=" << s.keyframe
                << " stable=" << stable.size()
                << " span=" << stableSpanM()*1000.0 << "mm"
                << " drift_from_first=[" << dpx << ',' << dpy << ',' << dpz << "]mm\n";
    }
    return passed;
  }

  int stableCount() const { return static_cast<int>(stable.size()); }
};

enum class CapturePhase { WAIT_START, SETTLE_START, MOVING, SETTLE_END };

struct StableCapture {
  std::vector<VioState> samples;
  int64_t last_kf = -1;

  void reset() {
    samples.clear();
    last_kf = -1;
  }

  static double speed(const VioState& s) {
    return std::sqrt(s.vx*s.vx + s.vy*s.vy + s.vz*s.vz);
  }

  bool update(const VioState& s, Mark* out) {
    if (s.keyframe == last_kf) return false;
    last_kf = s.keyframe;

    if (speed(s) > kStableMaxSpeedMps) {
      samples.clear();
      return false;
    }

    if (!samples.empty()) {
      const VioState& first = samples.front();
      const double dx = s.px - first.px;
      const double dy = s.py - first.py;
      const double dz = s.pz - first.pz;
      if (std::sqrt(dx*dx + dy*dy + dz*dz) > kStableMaxSpanM) {
        samples.clear();
      }
    }

    samples.push_back(s);
    if (static_cast<int>(samples.size()) < kStableStatesNeeded) return false;

