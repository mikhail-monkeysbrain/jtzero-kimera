            << "decoded frames: " << decoded << "\n"
            << "IMU received: " << imu_rx << "\n"
            << "IMU fed: " << imu_fed << "\n"
            << "ATTITUDE received: " << att_rx << "\n"
            << "TIMESYNC samples: " << sync_count << "\n"
            << "mapping valid: " << (mapping.valid?"yes":"no") << "\n"
            << "backend states: " << states << "\n";

  double sum=0.0,sum2=0.0,ab=0.0,ba=0.0;
  int nab=0,nba=0;
  for (const auto& row : rows) {
    const auto& r=row.measured;
    std::cout << "LEG " << r.index+1 << ' ' << r.direction << ": "
              << std::fixed << std::setprecision(2) << r.horizontal*1000.0
              << " mm  error " << r.error*1000.0
              << " mm  dz " << r.dz*1000.0
              << " mm  settle START/END " << row.start_settle_shift_m*1000.0
              << "/" << row.end_settle_shift_m*1000.0 << " mm\n";
    sum+=r.horizontal; sum2+=r.horizontal*r.horizontal;
    if(r.direction=="A->B"){ab+=r.horizontal;++nab;}else if(r.direction=="B->A"){ba+=r.horizontal;++nba;}
  }
  for(size_t i=0;i+1<rows.size();i+=2){
    std::cout << "PAIR " << (i/2)+1 << " reversal angle: "
              << std::fixed << std::setprecision(2)
              << pairAngleDeg(rows[i].measured,rows[i+1].measured) << " deg\n";
  }
  if(!rows.empty()){
    const double mean=sum/rows.size();
    const double sd=std::sqrt(std::max(0.0,sum2/rows.size()-mean*mean));
    std::cout << "MEAN HORIZONTAL: " << mean*1000.0 << " mm\n"
              << "STD: " << sd*1000.0 << " mm\n"
              << "A->B mean: " << (nab?ab/nab*1000.0:0.0) << " mm\n"
              << "B->A mean: " << (nba?ba/nba*1000.0:0.0) << " mm\n"
              << "MEAN SCALE measured/true: " << std::setprecision(6) << mean/kTruth500M << "\n";
  }
  std::cout << "CSV: " << kV8CsvPath << "\n";
}

void drawHudV8(const cv::Mat& gray,const HudPipeline& pipeline,const FcAttitude& fc,
               int leg,CapturePhase phase,const StableCapture& stable,
               const std::vector<V8Leg>& rows,const std::string& notice,
