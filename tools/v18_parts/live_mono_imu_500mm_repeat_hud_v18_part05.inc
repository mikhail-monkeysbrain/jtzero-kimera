    const double angle=(i%2)==1?pairAngleDeg(rows[i-1].measured,r):0.0;
    f << r.index+1 << ',' << r.direction << ','
      << row.start_press.s.keyframe << ',' << r.a.s.keyframe << ','
      << row.end_press.s.keyframe << ',' << r.b.s.keyframe << ','
      << row.start_press.s.px << ',' << row.start_press.s.py << ',' << row.start_press.s.pz << ','
      << r.a.s.px << ',' << r.a.s.py << ',' << r.a.s.pz << ','
      << row.end_press.s.px << ',' << row.end_press.s.py << ',' << row.end_press.s.pz << ','
      << r.b.s.px << ',' << r.b.s.py << ',' << r.b.s.pz << ','
      << row.start_settle_shift_m << ',' << row.end_settle_shift_m << ','
      << r.dx << ',' << r.dy << ',' << r.dz << ',' << r.horizontal << ','
      << r.d3 << ',' << r.error << ',' << r.scale << ',' << angle << '\n';
  }
}

void saveBackendTraceV8(const std::vector<VioState>& states,
                        const std::vector<V8Leg>& rows,
                        const char* path = "/home/vio/jtzero_500mm_v18_backend.csv") {
  std::ofstream f(path, std::ios::trunc);
  if (!f) return;

  f << "leg,phase,keyframe,timestamp_ns,callback_wall_ns,"
       "px_m,py_m,pz_m,vx_m_s,vy_m_s,vz_m_s,"
       "speed_m_s,roll_deg,pitch_deg,yaw_deg,"
       "bgx,bgy,bgz,bax,bay,baz\n";

  f << std::fixed << std::setprecision(9);

  for (const auto& s : states) {
    int leg = 0;
    const char* phase = "OUTSIDE";

    for (size_t i = 0; i < rows.size(); ++i) {
      const auto& r = rows[i];

      const int64_t start_press = r.start_press.s.keyframe;
      const int64_t start_settled = r.measured.a.s.keyframe;
      const int64_t end_press = r.end_press.s.keyframe;
      const int64_t end_settled = r.measured.b.s.keyframe;

      if (s.keyframe >= start_press && s.keyframe <= end_settled) {
        leg = static_cast<int>(i) + 1;

        if (s.keyframe < start_settled)
          phase = "SETTLE_START";
        else if (s.keyframe <= end_press)
          phase = "MOVE";
        else
          phase = "SETTLE_END";

        break;
      }
    }

    const double speed =
        std::sqrt(s.vx*s.vx + s.vy*s.vy + s.vz*s.vz);

    f << leg << ','
      << phase << ','
      << s.keyframe << ','
      << s.timestamp_ns << ','
      << s.callback_wall_ns << ','
      << s.px << ',' << s.py << ',' << s.pz << ','
      << s.vx << ',' << s.vy << ',' << s.vz << ','
      << speed << ','
      << s.roll_deg << ','
      << s.pitch_deg << ','
      << s.yaw_deg << ','
      << s.bgx << ',' << s.bgy << ',' << s.bgz << ','
      << s.bax << ',' << s.bay << ',' << s.baz << '\n';
  }

  std::cout << "BACKEND TRACE CSV: " << path << '\n';
}

void printSummaryV8(const std::vector<V8Leg>& rows,size_t raw,size_t rejected,
                    size_t selected,size_t decoded,size_t imu_rx,size_t imu_fed,
                    size_t att_rx,size_t sync_count,const ClockMapping& mapping,
                    size_t states) {
#ifdef JTZERO_SUMMARY_TITLE
  std::cout << "\n================ " << JTZERO_SUMMARY_TITLE << " ================\n";
#else
  std::cout << "\n================ 500 MM x6 V18 RESULT ================\n";
#endif
#ifdef JTZERO_SUMMARY_FUSION
  std::cout << "fusion: " << JTZERO_SUMMARY_FUSION << "\n";
#else
  std::cout << "fusion: v7 static-only gravity feedback\n";
#endif
  std::cout << "startup convergence gate: " << kPostStartupWarmupSec << " s + " << kWarmupStableStatesNeeded << " stable backend states\n"
            << "endpoint capture: " << kStableStatesNeeded << " stable backend states, |V| <= "
            << kStableMaxSpeedMps*1000.0 << " mm/s, span <= " << kStableMaxSpanM*1000.0 << " mm\n"
            << "raw camera frames: " << raw << "\n"
            << "rejected raw pairs: " << rejected << "\n"
            << "selected frames: " << selected << "\n"
