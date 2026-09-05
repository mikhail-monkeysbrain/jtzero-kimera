      man>=kStartupMinAccelNorm && man<=kStartupMaxAccelNorm &&
      san<=kStartupMaxAccelNormStd;

    if(ok){
      passed=true;
      std::cout<<std::fixed<<std::setprecision(6)
               <<"[STARTUP] PASS samples="<<n<<" sec="<<elapsed
               <<" mean_acc=["<<ma.transpose()<<"] |acc|="<<man
               <<" std|acc|="<<san
               <<" mean_gyro=["<<mg.transpose()<<"] gyro_std=["<<sg.transpose()<<"]\n";
      return true;
    }

    std::cout<<std::fixed<<std::setprecision(6)
             <<"[STARTUP] RETRY samples="<<n<<" sec="<<elapsed
             <<" |mean_gyro|="<<mg.norm()
             <<" max_gyro_std="<<sg.maxCoeff()
             <<" |acc|="<<man<<" std|acc|="<<san<<"\n";
    reset();
    return false;
  }
};

struct VioWarmupGate {
  bool passed = false;
  int64_t feed_enabled_wall_ns = 0;
  int64_t last_log_wall_ns = 0;
  int64_t last_kf = -1;
  bool have_first = false;
  VioState first{};
  std::vector<VioState> stable;

  void start(int64_t wall_ns) {
    passed = false;
    feed_enabled_wall_ns = wall_ns;
    last_log_wall_ns = 0;
    last_kf = -1;
    have_first = false;
    stable.clear();
  }

  static double speed(const VioState& s) {
    return std::sqrt(s.vx*s.vx + s.vy*s.vy + s.vz*s.vz);
  }

  double elapsedSec(int64_t wall_ns) const {
    if (feed_enabled_wall_ns <= 0 || wall_ns <= feed_enabled_wall_ns) return 0.0;
    return (wall_ns - feed_enabled_wall_ns) * 1e-9;
  }

  double stableSpanM() const {
    if (stable.empty()) return 0.0;
    const auto& a = stable.front();
    double max_span = 0.0;
    for (const auto& s : stable) {
      const double dx = s.px-a.px;
      const double dy = s.py-a.py;
      const double dz = s.pz-a.pz;
      max_span = std::max(max_span, std::sqrt(dx*dx+dy*dy+dz*dz));
    }
    return max_span;
  }

  bool update(const VioState& s, int64_t wall_ns) {
    if (passed) return true;
    if (s.keyframe == last_kf) return false;
    last_kf = s.keyframe;

    if (!have_first) {
      first = s;
      have_first = true;
      std::cout << std::fixed << std::setprecision(4)
                << "[WARMUP] first backend KF=" << s.keyframe
                << " P=[" << s.px << ',' << s.py << ',' << s.pz << "]"
                << " RPYdeg=[" << s.roll_deg << ',' << s.pitch_deg << ',' << s.yaw_deg << "]\n";
    }

    if (speed(s) > kWarmupMaxSpeedMps) {
      stable.clear();
    } else {
      if (!stable.empty()) {
        const auto& a = stable.front();
