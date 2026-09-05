    requestRate(serial_fd,sys,comp,MAVLINK_MSG_ID_DISTANCE_SENSOR,20.0);

    camera_fd=open(camera_device.c_str(),O_RDWR|O_NONBLOCK);if(camera_fd==-1)fail("open camera");configureCamera(camera_fd);buffers=initCameraBuffers(camera_fd);
    v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;if(xioctl(camera_fd,VIDIOC_STREAMON,&type)==-1)fail("STREAMON");streaming=true;discardWarmup(camera_fd);
    cv::setNumThreads(1);cv::namedWindow(kV8WindowName,cv::WINDOW_NORMAL);cv::resizeWindow(kV8WindowName,1280,900);

    std::vector<TimeSyncSample>sync;sync.reserve(500);ClockMapping mapping;int64_t pending=0,next_sync=monotonicNs();
    size_t raw=0,rejected=0,selected=0,decoded=0,imu_rx=0,imu_fed=0,imu_skip=0,att_rx=0;
    uint32_t prev_seq=0;int64_t prev_ts=0,last_sel=0;bool have_prev=false;VIO::FrameId fid=0;FcAttitude fc;
    cv::Mat gray(kHeight,kWidth,CV_8UC1,cv::Scalar(0));jtzero::ImuCorrection imu_correction;StartupStaticGate startup_gate;VioWarmupGate warmup_gate;bool vio_feed_enabled=false;
    int leg=0;CapturePhase phase=CapturePhase::WAIT_START;StableCapture stable;Mark start_press,start_mark,end_press;
    std::vector<V8Leg>rows;std::string notice;int64_t notice_until=0,next_hud=0;

    std::ofstream raw_combined(kV11CombinedPath,std::ios::trunc);
    std::ofstream raw_camera(kV11CameraPath,std::ios::trunc);
    std::ofstream raw_mjpg(kV11MjpegPath,std::ios::binary|std::ios::trunc);
    std::ofstream raw_attitude(kV11AttitudePath,std::ios::trunc);
    std::ofstream raw_range(kV12RangePath,std::ios::trunc);
    if(!raw_combined||!raw_camera||!raw_mjpg||!raw_attitude||!raw_range)
      throw std::runtime_error("Cannot create v18 raw-recording files");

    raw_combined<<"type,recv_ns,source_ns,mapped_ns,fc_time_us,reserved1,reserved2,reserved3,ax,ay,az,gx,gy,gz\n";
    raw_camera<<"sequence,raw_timestamp_ns,corrected_timestamp_ns,mapping_valid,selected,offset,bytes\n";
    raw_attitude<<"recv_ns,time_boot_ms,roll_deg,pitch_deg,yaw_deg,rollspeed,pitchspeed,yawspeed\n";
    raw_range<<"recv_ns,time_boot_ms,min_cm,max_cm,current_cm,type,id,orientation,covariance,hfov,vfov,quat_w,quat_x,quat_y,quat_z,signal_quality,fc_roll_deg,fc_pitch_deg,fc_yaw_deg,vertical_m\n";
    raw_combined<<std::fixed<<std::setprecision(9);
    raw_camera<<std::fixed<<std::setprecision(0);
    raw_attitude<<std::fixed<<std::setprecision(9);
    raw_range<<std::fixed<<std::setprecision(9);
    uint64_t raw_mjpg_offset=0;
    size_t raw_imu_rows=0,raw_camera_rows=0,raw_attitude_rows=0,raw_range_rows=0;
    int64_t prev_loop_wall_ns=monotonicNs();
    size_t loop_stall_count=0;
    int64_t max_loop_stall_ns=0;

    while(leg<kLegCount){
      const int64_t now=monotonicNs();
      const int64_t loop_dt_ns=now-prev_loop_wall_ns;
      if(loop_dt_ns>100000000LL){
        ++loop_stall_count;
        max_loop_stall_ns=std::max(max_loop_stall_ns,loop_dt_ns);
        std::cout<<"[LOOP-STALL] dt="<<std::fixed<<std::setprecision(3)
                 <<loop_dt_ns/1e6<<" ms phase="<<static_cast<int>(phase)
                 <<" leg="<<(leg+1)<<" raw="<<raw<<" imu_fed="<<imu_fed<<"\n";
#ifdef JTZERO_STRICT_STALL_NS
        if(loop_dt_ns>JTZERO_STRICT_STALL_NS){
          if(phase==CapturePhase::SETTLE_START){
            std::cout<<"LEG "<<leg+1<<" START REJECTED due to loop stall "<<loop_dt_ns/1e6<<" ms\n";
            stable.reset();start_press.valid=false;phase=CapturePhase::WAIT_START;
            notice="START INVALID — LOOP STALL, НАЖМИТЕ START ЕЩЕ РАЗ";
            notice_until=monotonicNs()+2500000000LL;
          }else if(phase==CapturePhase::MOVING||phase==CapturePhase::SETTLE_END){
            std::cout<<"[RUN INVALID] loop stall "<<loop_dt_ns/1e6<<" ms during measured leg "<<leg+1<<"\n";
            aborted=true;
            break;
          }
        }
#endif
      }
      prev_loop_wall_ns=now;
      if(now>=next_sync&&pending==0){pending=now;sendTimesync(serial_fd,pending,sys,comp);next_sync=now+kTimesyncPeriodNs;}
      pollfd pf[2]={{camera_fd,POLLIN,0},{serial_fd,POLLIN,0}};int rc=poll(pf,2,2);if(rc<0){if(errno==EINTR)continue;fail("poll");}

      if(pf[1].revents&POLLIN){uint8_t b[8192];for(;;){ssize_t n=read(serial_fd,b,sizeof(b));if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;for(ssize_t i=0;i<n;++i){
        if(!mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst))continue;const int64_t recv=monotonicNs();
        if(msg.msgid==MAVLINK_MSG_ID_TIMESYNC){mavlink_timesync_t ts{};mavlink_msg_timesync_decode(&msg,&ts);if(ts.tc1!=0&&pending&&ts.ts1==pending){TimeSyncSample s;s.t0_rpi_ns=pending;s.t1_rpi_ns=recv;s.fc_ns=ts.tc1;s.rtt_ns=recv-pending;s.rpi_mid_ns=pending+s.rtt_ns/2;s.good=s.rtt_ns>0&&nsToMs(s.rtt_ns)<=kMaxTimesyncRttMs;sync.push_back(s);pending=0;mapping=estimateClockMapping(sync);}}
        else if(msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){++imu_rx;mavlink_highres_imu_t h{};mavlink_msg_highres_imu_decode(&msg,&h);if(!mapping.valid||recv-mapping.last_update_ns>kMappingStaleNs){++imu_skip;continue;}const Eigen::Vector3d acc=jtzero::ImuCorrection::accelFrdToFlu(h.xacc,h.yacc,h.zacc);const Eigen::Vector3d gyro=jtzero::ImuCorrection::gyroFrdToFlu(h.xgyro,h.ygyro,h.zgyro);if(!vio_feed_enabled){if(startup_gate.add(h.time_usec,acc,gyro)){imu_correction.reset();vio_feed_enabled=true;warmup_gate.start(monotonicNs());std::cout<<"[STARTUP] VIO FEED ENABLED — fresh IMU/camera only from this point\n"<<"[WARMUP] mandatory 12 s stationary VIO convergence gate started\n";}else{++imu_skip;continue;}}const int64_t mapped_imu_ns=mapping.map((int64_t)h.time_usec*1000LL);
