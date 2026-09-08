    if(gV26ControlledAb4){
      std::string guide;
      if(phase==CapturePhase::WAIT_START) guide=(leg==0)?"1) УПОР A  2) SPACE  3) СРАЗУ ДВИГАТЬ":"ВОЗВРАТ B→A НЕ ИЗМЕРЯЕТСЯ; СНОВА УПОР A";
      else if(phase==CapturePhase::MOVING) guide="ДВИГАЙТЕ ПЛАВНО; НА УПОРЕ B: SPACE, ЗАТЕМ СТОП";
      else guide="НЕ ДВИГАТЬ ДО СООБЩЕНИЯ О СОХРАНЕНИИ";
      ru(panel,guide,{18,355},13,{220,220,220},cv::QT_FONT_BOLD);
    }else{
      ru(panel,phase==CapturePhase::WAIT_START?"Установите камеру точно на метку":"Переместите камеру ровно на 500 мм",{18,355},14,{220,220,220});
    }
  }
  cv::rectangle(panel,{18,395,344,92},cv::Scalar(55,145,55),-1);
  cv::rectangle(panel,{18,395,344,92},cv::Scalar(220,220,220),2);
  ru(panel,action,{35,452},18,{255,255,255},cv::QT_FONT_BOLD);

  if(!rows.empty()){
    const auto&r=rows.back().measured;
    std::snprintf(b,sizeof(b),"Последний: %.1f мм",r.horizontal*1000.0);ru(panel,b,{18,555},20,{245,245,245},cv::QT_FONT_BOLD);
    std::snprintf(b,sizeof(b),"Ошибка: %+.1f мм",r.error*1000.0);ru(panel,b,{18,595},16,{220,220,220});
    std::snprintf(b,sizeof(b),"ΔZ: %+.1f мм",r.dz*1000.0);ru(panel,b,{18,630},15,{190,190,190});
  }
  ru(panel,"PASS: каждый ±30 мм, σ≤20 мм, ΔZ≤30 мм",{18,700},12,{185,185,185});
  ru(panel,"Разворот каждой пары ≥160°",{18,725},12,{185,185,185});
  ru(panel,"Q / ESC — выход",{18,755},14,{215,215,215});
  if(!notice.empty()){cv::rectangle(panel,{10,790,360,70},cv::Scalar(35,35,125),-1);ru(panel,notice,{20,835},13,{255,255,255},cv::QT_FONT_BOLD);}
  panel.copyTo(canvas(cv::Rect(900,0,380,720)));
  cv::imshow(kV8WindowName,canvas);
}

} // namespace

int main(int argc,char**argv){
  google::InitGoogleLogging(argv[0]);
  FLAGS_visualize=false;FLAGS_viz_type=2;FLAGS_use_lcd=false;FLAGS_log_output=false;FLAGS_extract_planes_from_the_scene=false;
  const std::string params=argc>1?argv[1]:"params/JTZeroMonoFLU_TBS_Rm1d5_Pm5d5";
  const std::string camera_device=argc>2?argv[2]:"/dev/video8";
#ifdef JTZERO_B_FIRST_HUD
  #ifdef JTZERO_B_FIRST_FLAG
  JTZERO_B_FIRST_FLAG=(argc>3 && std::string(argv[3])=="B_FIRST");
  #else
  gV23BFirst=(argc>3 && std::string(argv[3])=="B_FIRST");
  #endif
#endif
  gV26ControlledAb4=(argc>3 && std::string(argv[3])=="CONTROLLED_AB4");
  if(gV26ControlledAb4){
    gV25BFirst=false;
    if(const char* e=std::getenv("JTZERO_V26_TARGET_SEC")) gV26TargetMoveSec=std::max(0.5,std::atof(e));
    if(const char* e=std::getenv("JTZERO_V26_TOLERANCE_SEC")) gV26DurationToleranceSec=std::max(0.1,std::atof(e));
    gV26ReferenceMoveSec=gV26TargetMoveSec;
    std::cout<<"[MODE] CONTROLLED_AB4: four measured A->B passes; B->A returns are unmeasured; target "
             <<gV26TargetMoveSec<<" s; accepted "
             <<(gV26TargetMoveSec-gV26DurationToleranceSec)<<".."
             <<(gV26TargetMoveSec+gV26DurationToleranceSec)<<" s\n";
  }
  std::cout<<"[CAM] device="<<camera_device<<"\n";
  cv::namedWindow(kV8WindowName, cv::WINDOW_NORMAL);
  cv::setWindowProperty(kV8WindowName, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

  int camera_fd=-1,serial_fd=-1;bool streaming=false,imu_req=false,att_req=false,pipeline_started=false,aborted=false;
  uint8_t sys=0,comp=0;std::vector<CameraBuffer>buffers;std::shared_ptr<HudPipeline>pipeline;std::thread pipeline_thread;

  try{
    VIO::VioParams vp(params);if(vp.camera_params_.empty())throw std::runtime_error("No camera params loaded");
    pipeline=std::make_shared<HudPipeline>(vp);pipeline->installBackendCallback();pipeline->installFrontendDebugCallback();pipeline_thread=std::thread([pipeline](){pipeline->spin();});pipeline_started=true;
    serial_fd=openSerial();mavlink_status_t mst{};mavlink_message_t msg{};std::cout<<"[MAV] waiting for HEARTBEAT...\n";
    int64_t dl=monotonicNs()+10000000000LL;
    while(monotonicNs()<dl&&!sys){pollfd p{serial_fd,POLLIN,0};if(poll(&p,1,100)<=0)continue;uint8_t b[2048];ssize_t n=read(serial_fd,b,sizeof(b));if(n<=0)continue;for(ssize_t i=0;i<n;++i)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst)&&msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=msg.sysid;comp=msg.compid;break;}}
    if(!sys)throw std::runtime_error("HEARTBEAT timeout");
    requestRate(serial_fd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,kImuRateHz);imu_req=true;
    requestRate(serial_fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,kFcAttitudeRateHz);att_req=true;
