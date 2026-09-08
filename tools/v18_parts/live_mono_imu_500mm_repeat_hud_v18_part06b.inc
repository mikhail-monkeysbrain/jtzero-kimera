               const VioWarmupGate& warmup,int64_t now_ns) {
  cv::Mat bgr,video;
  cv::cvtColor(gray,bgr,cv::COLOR_GRAY2BGR);
  cv::resize(bgr,video,{900,675},0,0,cv::INTER_NEAREST);
  cv::Mat panel(720,380,CV_8UC3,cv::Scalar(24,24,24));
  cv::Mat canvas(720,1280,CV_8UC3,cv::Scalar(8,8,8));
  video.copyTo(canvas(cv::Rect(0,45,900,675)));

  ru(canvas,gV26ControlledAb4?"JT-ZERO — CONTROL 4× A→B":"JT-ZERO — 500 мм × 4 — V25",{24,32},23,{245,245,245},cv::QT_FONT_BOLD);
  ru(canvas,gV26ControlledAb4?"4 одинаковых A→B • цель 7.5 с • окно 6.5..8.5 с • B→A не измеряется":"12s warm-up • stabilized endpoints • TF-Luna • mono frontend",{430,30},14,{190,190,190});
  char b[256];
  std::snprintf(b,sizeof(b),"ПРОХОД %d / %d",std::min(leg+1,kLegCount),kLegCount);
  ru(panel,b,{18,42},22,{245,245,245},cv::QT_FONT_BOLD);
if(gV26ControlledAb4){
    std::snprintf(b,sizeof(b),"%dAB   A  →  B",std::min(leg+1,kLegCount));
    ru(panel,b,{18,82},27,{0,230,255},cv::QT_FONT_BOLD);
  }else{
#ifdef JTZERO_B_FIRST_HUD
  #ifdef JTZERO_B_FIRST_FLAG
    ru(panel,((leg%2==0)^JTZERO_B_FIRST_FLAG)?"A  →  B":"B  →  A",{18,82},27,{0,230,255},cv::QT_FONT_BOLD);
  #else
    ru(panel,((leg%2==0)^gV23BFirst)?"A  →  B":"B  →  A",{18,82},27,{0,230,255},cv::QT_FONT_BOLD);
  #endif
#else
    ru(panel,(leg%2==0)?"A  →  B":"B  →  A",{18,82},27,{0,230,255},cv::QT_FONT_BOLD);
#endif
  }

  VioState s; const bool have_state=pipeline.latest(&s);
  ru(panel,have_state?"КООРДИНАТЫ: ЕСТЬ":"КООРДИНАТЫ: НЕТ",{18,125},18,
     have_state?cv::Scalar(90,220,90):cv::Scalar(40,40,245),cv::QT_FONT_BOLD);
  if(have_state){
    std::snprintf(b,sizeof(b),"P [%.3f  %.3f  %.3f] м",s.px,s.py,s.pz);ru(panel,b,{18,158},15,{220,220,220});
    std::snprintf(b,sizeof(b),"|V| %.1f мм/с   KF %lld",StableCapture::speed(s)*1000.0,(long long)s.keyframe);ru(panel,b,{18,186},14,{190,190,190});
  }
  if(fc.valid){std::snprintf(b,sizeof(b),"FC: R %.2f  P %.2f  Y %.2f°",fc.roll_deg,fc.pitch_deg,fc.yaw_deg);ru(panel,b,{18,217},14,{190,190,190});}

  std::string status,action;
  cv::Scalar status_color(220,220,220);
  if(gV26ControlledAb4){
    if(!warmup.passed){status="ПОДГОТОВКА — НЕ ДВИГАТЬ";action="SPACE ПОКА ЗАБЛОКИРОВАН";status_color=cv::Scalar(0,210,255);}
    else if(phase==CapturePhase::WAIT_START){
      status=(leg==0)?"УСТАНОВИТЕ СТЕНД НА A":"ВЕРНИТЕ СТЕНД B → A";
      action="SPACE: СТАРТ → ДВИГАЙТЕ";
      status_color=cv::Scalar(90,220,90);
    }
    else if(phase==CapturePhase::SETTLE_START){status="ФИКСИРУЮ A — НЕ ДВИГАТЬ";action="ЖДИТЕ";status_color=cv::Scalar(0,210,255);}
    else if(phase==CapturePhase::MOVING){status="ДВИЖЕНИЕ A → B";action="НА B: SPACE → СТОП";status_color=cv::Scalar(0,210,255);}
    else {status="НА B — НЕ ДВИГАТЬ";action="ФИКСИРУЮ КОНЕЦ";status_color=cv::Scalar(0,210,255);}
  }else{
    if(!warmup.passed){status="VIO WARM-UP — НЕ ДВИГАТЬ";action="СТАРТ ПОКА ЗАБЛОКИРОВАН";status_color=cv::Scalar(0,210,255);}
    else if(phase==CapturePhase::WAIT_START){status="ГОТОВО К СТАРТУ";action="ПРОБЕЛ / ENTER: СТАРТ";status_color=cv::Scalar(90,220,90);}
    else if(phase==CapturePhase::SETTLE_START){status="СТАБИЛИЗАЦИЯ СТАРТА";action="НЕ ДВИГАТЬ";status_color=cv::Scalar(0,210,255);}
    else if(phase==CapturePhase::MOVING){status="ДВИЖЕНИЕ — ACC GRAVITY OFF";action="ПРОБЕЛ / ENTER: КОНЕЦ";status_color=cv::Scalar(0,210,255);}
    else {status="СТАБИЛИЗАЦИЯ КОНЦА";action="НЕ ДВИГАТЬ";status_color=cv::Scalar(0,210,255);}
  }
  ru(panel,status,{18,260},15,status_color,cv::QT_FONT_BOLD);

  if(gV26ControlledAb4 && phase==CapturePhase::MOVING && gV26MoveStartWallNs>0){
    const double elapsed=std::max(0.0,(now_ns-gV26MoveStartWallNs)*1e-9);
    const double ref=kV26TargetMoveSec;
    const double lo=ref-kV26DurationToleranceSec;
    const double hi=ref+kV26DurationToleranceSec;
    const double scale_sec=10.0;
    const int x0=55,y0=655,w=790,h=24;

    cv::rectangle(canvas,{x0,y0,w,h},cv::Scalar(35,35,35),-1);
    const int gx=x0+static_cast<int>(w*(lo/scale_sec));
    const int gw=std::max(2,static_cast<int>(w*((hi-lo)/scale_sec)));
    cv::rectangle(canvas,{gx,y0,gw,h},cv::Scalar(45,125,45),-1);
    cv::rectangle(canvas,{x0,y0,w,h},cv::Scalar(230,230,230),1);

    // Time ticks: 0, 1.5, 3.0, 3.75(mid), 4.5, 6.0, 7.5, 8.5, 10 s.
    const double ticks[]={0.0,1.5,3.0,3.75,4.5,6.0,7.5,8.5,10.0};
    for(double t:ticks){
      const int tx=x0+static_cast<int>(w*(t/scale_sec));
      const bool major=std::abs(t-3.75)<1e-6 || std::abs(t-7.5)<1e-6;
      cv::line(canvas,{tx,y0-(major?8:4)},{tx,y0+h+(major?8:4)},
               major?cv::Scalar(255,255,255):cv::Scalar(150,150,150),major?2:1);
    }

    const int midx=x0+static_cast<int>(w*((ref*0.5)/scale_sec));
    ru(canvas,"СЕРЕДИНА ~3.75 с",{midx-75,y0-14},13,{235,235,235},cv::QT_FONT_BOLD);

    const int targetx=x0+static_cast<int>(w*(ref/scale_sec));
    ru(canvas,"B / SPACE ~7.5 с",{targetx-85,y0-14},13,{235,235,235},cv::QT_FONT_BOLD);

    const int px=x0+static_cast<int>(w*std::min(1.0,elapsed/scale_sec));
    cv::circle(canvas,{px,y0+h/2},8,cv::Scalar(0,230,255),-1);

    char tb[256];
    std::snprintf(tb,sizeof(tb),"ВРЕМЯ %.2f с   ЦЕЛЬ 7.50 с   ДОПУСК 6.50..8.50 с",elapsed);
    ru(canvas,tb,{55,642},16,{255,255,255},cv::QT_FONT_BOLD);
  }
  if(!warmup.passed){
    std::snprintf(b,sizeof(b),"Warm-up %.1f / %.0f с; stable %d / %d",
                  warmup.elapsedSec(now_ns),kPostStartupWarmupSec,
                  warmup.stableCount(),kWarmupStableStatesNeeded);
    ru(panel,b,{18,292},14,{220,220,220});
  }else if(phase==CapturePhase::SETTLE_START||phase==CapturePhase::SETTLE_END){
    std::snprintf(b,sizeof(b),"Стабильных состояний: %d / %d",stable.count(),kStableStatesNeeded);
    ru(panel,b,{18,292},14,{220,220,220});
  }else{
