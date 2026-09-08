
      if(vio_feed_enabled&&!warmup_gate.passed){
        VioState ws;
        if(pipeline->latest(&ws)) warmup_gate.update(ws,now);
      }

      if(phase==CapturePhase::SETTLE_START||phase==CapturePhase::SETTLE_END){
        VioState s;if(pipeline->latest(&s)){Mark settled;if(stable.update(s,&settled)){
          if(phase==CapturePhase::SETTLE_START){
            const double start_shift_m=markDistance(start_press,settled);
            if(start_shift_m>kV25MaxStartSettleShiftM){
              notice="START INVALID — СДВИГ > 5 ММ, НАЖМИТЕ START ЕЩЕ РАЗ";
              notice_until=monotonicNs()+2500000000LL;
              std::cout<<"LEG "<<leg+1<<" START REJECTED shift="<<start_shift_m*1000.0<<" mm > "<<kV25MaxStartSettleShiftM*1000.0<<" mm\n";
              stable.reset();start_press.valid=false;phase=CapturePhase::WAIT_START;
            }else{
              start_mark=settled;phase=CapturePhase::MOVING;notice="СТАРТ СТАБИЛИЗИРОВАН — ДВИГАЙТЕ 500 ММ";notice_until=monotonicNs()+1500000000LL;
              std::cout<<"LEG "<<leg+1<<" START press KF="<<start_press.s.keyframe<<" settled KF="<<start_mark.s.keyframe<<" shift="<<start_shift_m*1000.0<<" mm [STRICT PASS, gravity feedback OFF]\n";
            }
          }
          else {V8Leg row;row.start_press=start_press;row.end_press=end_press;row.measured=makeLeg500(leg,start_mark,settled);
            row.measured.direction=gV26ControlledAb4?"A->B":(((leg%2==0)^gV25BFirst)?"A->B":"B->A");
            row.start_settle_shift_m=markDistance(start_press,start_mark);row.end_settle_shift_m=markDistance(end_press,settled);rows.push_back(row);std::cout<<"LEG "<<leg+1<<' '<<row.measured.direction<<" horizontal="<<std::fixed<<std::setprecision(2)<<row.measured.horizontal*1000.0<<" mm error="<<row.measured.error*1000.0<<" mm dz="<<row.measured.dz*1000.0<<" mm end_settle_shift="<<row.end_settle_shift_m*1000.0<<" mm\n";++leg;phase=CapturePhase::WAIT_START;notice=gV26ControlledAb4?"ПРОХОД СОХРАНЁН — ВЕРНИТЕ B → A БЕЗ SPACE; НА A SPACE = НОВЫЙ A → B":"КОНЕЦ СТАБИЛИЗИРОВАН — СЛЕДУЮЩИЙ ПРОХОД";notice_until=monotonicNs()+2500000000LL;}
        }}
      }

      if(now>=next_hud){if(monotonicNs()>notice_until)notice.clear();drawHudV8(gray,*pipeline,fc,leg,phase,stable,rows,notice,warmup_gate,now);next_hud=now+kHudPeriodNs;}
      static std::ofstream v25_events("/home/vio/jtzero_500mm_v25_events.csv",std::ios::trunc);
      static bool v25_events_header=false;
      if(v25_events && !v25_events_header){
        v25_events<<"event_wall_ns,event,leg,phase,keyframe,state_timestamp_ns,state_callback_wall_ns,state_age_at_event_ms,px_m,py_m,pz_m,vx_m_s,vy_m_s,vz_m_s\n";
        v25_events<<std::fixed<<std::setprecision(9);
        v25_events_header=true;
      }
      if(gV26ControlledAb4 && leg>=kLegCount){
        notice=(kLegCount==1)?"1AB СОХРАНЁН — Q / ESC: ВЫХОД":"4AB СОХРАНЁН — ТЕСТ ЗАВЕРШЁН. Q / ESC: ВЫХОД";
        notice_until=monotonicNs()+3600000000000LL;
      }
      const int key=cv::waitKey(1)&0xff;
      if(key==27||key=='q'||key=='Q'){
        if(!(gV26ControlledAb4 && leg>=kLegCount)) aborted=true;
        break;
      }
      if((key==32||key==13||key==10) && gV26ControlledAb4 && leg>=kLegCount){
        notice=(kLegCount==1)?"1AB УЖЕ СОХРАНЁН — Q / ESC ДЛЯ ВЫХОДА":"4AB УЖЕ СОХРАНЁН — НАЖМИТЕ Q / ESC ДЛЯ ВЫХОДА";
        notice_until=monotonicNs()+3000000000LL;
        continue;
      }
      if(key==32||key==13||key==10){VioState s;if(!pipeline->latest(&s)){notice="КООРДИНАТ ПОКА НЕТ";notice_until=monotonicNs()+1500000000LL;}
        else if(!warmup_gate.passed){notice="VIO WARM-UP НЕ ЗАВЕРШЕН — НЕ ДВИГАТЬ";notice_until=monotonicNs()+1500000000LL;std::cout<<"[WARMUP] START key ignored: t="<<std::fixed<<std::setprecision(2)<<warmup_gate.elapsedSec(monotonicNs())<<"s stable="<<warmup_gate.stableCount()<<"/"<<kWarmupStableStatesNeeded<<"\n";}
        else if(phase==CapturePhase::WAIT_START){
          const int64_t event_wall_ns=monotonicNs();
#ifdef JTZERO_V41_METRIC_CAMERA
          if(!gV41Cam.range_valid){
            notice="V41: НЕТ ВАЛИДНОЙ TF-LUNA ДАЛЬНОСТИ — START НЕ ПРИНЯТ";
            notice_until=monotonicNs()+2500000000LL;
            std::cout<<"[V41-PREFLIGHT] START REJECTED: TF-Luna range invalid, vertical_range="
                     <<gV41Cam.vertical_range_m<<" m\n";
            continue;
          }
          std::cout<<"[V41-PREFLIGHT] PASS: TF-Luna vertical="<<std::fixed<<std::setprecision(3)
                   <<gV41Cam.vertical_range_m<<" m, camera_height="<<gV41Cam.cameraHeight()<<" m\n";
#endif
          const double start_backend_age_ms=(event_wall_ns-s.callback_wall_ns)/1e6;
          if(start_backend_age_ms>kV25MaxStartBackendAgeMs){
            notice="START INVALID — BACKEND СЛИШКОМ СТАРЫЙ";
            notice_until=monotonicNs()+2500000000LL;
            std::cout<<"LEG "<<leg+1<<" START REJECTED backend_age="<<start_backend_age_ms<<" ms > "<<kV25MaxStartBackendAgeMs<<" ms\n";
            continue;
          }
          if(v25_events){
            v25_events<<event_wall_ns<<",START,"<<(leg+1)<<","<<static_cast<int>(phase)<<","<<s.keyframe<<","<<s.timestamp_ns<<","<<s.callback_wall_ns<<","<<(event_wall_ns-s.callback_wall_ns)/1e6<<","<<s.px<<","<<s.py<<","<<s.pz<<","<<s.vx<<","<<s.vy<<","<<s.vz<<"\n";
            v25_events.flush();
          }
          start_press.s=s;start_press.valid=true;stable.reset();
          if(gV26ControlledAb4){
            start_mark=start_press;
            gV26MoveStartWallNs=event_wall_ns;
#ifdef JTZERO_V41_METRIC_CAMERA
            if(const char* e=std::getenv("JTZERO_V41_CAMERA_HEIGHT_OFFSET_M")) gV41Cam.camera_height_offset_m=std::atof(e);
            gV41Cam.reset();
#endif
            phase=CapturePhase::MOVING;
            notice="СТАРТ ЗАФИКСИРОВАН — ДВИГАЙТЕ A → B";
            notice_until=monotonicNs()+1800000000LL;
            std::cout<<"PASS "<<leg+1<<"AB START KF="<<s.keyframe
                     <<" move_timer=0.000 s [CONTROLLED_AB4]\n";
          }else{
            phase=CapturePhase::SETTLE_START;
            notice="НЕ ДВИГАТЬ — ФИКСИРУЮ СТАРТ";
            notice_until=monotonicNs()+1500000000LL;
            std::cout<<"LEG "<<leg+1<<" START key accepted KF="<<s.keyframe<<" event_wall_ns="<<event_wall_ns<<" state_ts="<<s.timestamp_ns<<" cb_wall_ns="<<s.callback_wall_ns<<" age_ms="<<(event_wall_ns-s.callback_wall_ns)/1e6<<" [settling]\n";
          }}
        else if(phase==CapturePhase::MOVING){
          const int64_t event_wall_ns=monotonicNs();
          const double end_backend_age_ms=(event_wall_ns-s.callback_wall_ns)/1e6;
          if(end_backend_age_ms>kV25MaxStartBackendAgeMs){
            notice="END INVALID — BACKEND СЛИШКОМ СТАРЫЙ";
            notice_until=monotonicNs()+2500000000LL;
            std::cout<<"LEG "<<leg+1<<" END REJECTED backend_age="<<end_backend_age_ms
                     <<" ms > "<<kV25MaxStartBackendAgeMs<<" ms\n";
            continue;
          }
          if(v25_events){
            v25_events<<event_wall_ns<<",END,"<<(leg+1)<<","<<static_cast<int>(phase)<<","<<s.keyframe<<","<<s.timestamp_ns<<","<<s.callback_wall_ns<<","<<(event_wall_ns-s.callback_wall_ns)/1e6<<","<<s.px<<","<<s.py<<","<<s.pz<<","<<s.vx<<","<<s.vy<<","<<s.vz<<"\n";
            v25_events.flush();
          }
          end_press.s=s;end_press.valid=true;
#ifdef JTZERO_V41_METRIC_CAMERA
          gV41Cam.stop();
          std::cout<<"[V41-CAMERA] net="<<std::fixed<<std::setprecision(2)<<gV41Cam.net()*1000.0
                   <<" mm path="<<gV41Cam.path_m*1000.0<<" mm height="<<gV41Cam.cameraHeight()
                   <<" m good_updates="<<gV41Cam.good_updates<<" rejected="<<gV41Cam.rejected_updates<<"\n";
#endif
          if(gV26ControlledAb4 && leg>=0 && leg<4 && gV26MoveStartWallNs>0){
            const double move_sec=(event_wall_ns-gV26MoveStartWallNs)*1e-9;
            gV26MoveDurationsSec[leg]=move_sec;
            const double dt_target=move_sec-gV26TargetMoveSec;
            const bool timing_ok=std::abs(dt_target)<=gV26DurationToleranceSec;
            std::cout<<"PASS "<<leg+1<<"AB MOTION DURATION "<<std::fixed<<std::setprecision(3)<<move_sec<<" s"
                     <<"  delta_from_target="<<std::showpos<<dt_target<<std::noshowpos<<" s"
                     <<"  timing="<<(timing_ok?"PASS":"FAIL")<<"\n";
          }
          stable.reset();phase=CapturePhase::SETTLE_END;notice="SPACE ПРИНЯТ — ОСТАНОВИТЕ НА B И НЕ ДВИГАЙТЕ";notice_until=monotonicNs()+2200000000LL;std::cout<<"LEG "<<leg+1<<" END key accepted KF="<<s.keyframe<<" event_wall_ns="<<event_wall_ns<<" state_ts="<<s.timestamp_ns<<" cb_wall_ns="<<s.callback_wall_ns<<" age_ms="<<(event_wall_ns-s.callback_wall_ns)/1e6<<" [gravity feedback remains OFF, settling]\n";}
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));pipeline->shutdown();if(pipeline_thread.joinable())pipeline_thread.join();pipeline_started=false;cv::destroyAllWindows();const auto states=pipeline->states();
    const auto frontend_states=pipeline->frontendStates();
    {
      std::ofstream ff("/home/vio/jtzero_500mm_v25_frontend.csv",std::ios::trunc);
      if(ff){
        ff<<"timestamp_ns,callback_wall_ns,frame_id,is_keyframe,detected_features,tracked_features,mono_inliers,mono_putatives,mono_inlier_ratio,mono_ransac_iters,feature_detection_time_s,feature_tracking_time_s,mono_ransac_time_s,mono_status,mono_pose_valid,mono_tx,mono_ty,mono_tz,mono_t_norm,mono_body_tx,mono_body_ty,mono_body_tz,mono_body_t_norm,mono_roll_deg,mono_pitch_deg,mono_yaw_deg,pim_valid,pim_dt_s,pim_dpx,pim_dpy,pim_dpz,pim_dvx,pim_dvy,pim_dvz,pim_droll_deg,pim_dpitch_deg,pim_dyaw_deg\n";
        ff<<std::fixed<<std::setprecision(9);
        for(const auto& d:frontend_states){
          const double ratio=d.mono_putatives?static_cast<double>(d.mono_inliers)/static_cast<double>(d.mono_putatives):0.0;
          ff<<d.timestamp_ns<<','<<d.callback_wall_ns<<','<<d.frame_id<<','<<(d.is_keyframe?1:0)<<','
            <<d.detected_features<<','<<d.tracked_features<<','<<d.mono_inliers<<','<<d.mono_putatives<<','
            <<ratio<<','<<d.mono_ransac_iters<<','<<d.feature_detection_time_s<<','
            <<d.feature_tracking_time_s<<','<<d.mono_ransac_time_s<<','<<d.mono_status<<','
            <<(d.mono_pose_valid?1:0)<<','<<d.mono_tx<<','<<d.mono_ty<<','<<d.mono_tz<<','<<d.mono_t_norm<<','
            <<d.mono_body_tx<<','<<d.mono_body_ty<<','<<d.mono_body_tz<<','<<d.mono_body_t_norm<<','
            <<d.mono_roll_deg<<','<<d.mono_pitch_deg<<','<<d.mono_yaw_deg<<','
            <<(d.pim_valid?1:0)<<','<<d.pim_dt_s<<','
            <<d.pim_dpx<<','<<d.pim_dpy<<','<<d.pim_dpz<<','
            <<d.pim_dvx<<','<<d.pim_dvy<<','<<d.pim_dvz<<','
            <<d.pim_droll_deg<<','<<d.pim_dpitch_deg<<','<<d.pim_dyaw_deg<<'\n';
        }
      }
    }
    if(gV26ControlledAb4){
      for(size_t i=0;i<rows.size() && i<4;++i){
        const std::string path="/home/vio/jtzero_500mm_v26_"+std::to_string(i+1)+"AB.csv";
        std::ofstream pf(path,std::ios::trunc);
        if(!pf) continue;
        pf<<"pass,direction,keyframe,timestamp_ns,elapsed_ms,px_rel_mm,py_rel_mm,pz_rel_mm,vx_m_s,vy_m_s,vz_m_s,speed_m_s\n";
        pf<<std::fixed<<std::setprecision(9);
        const int64_t ta=rows[i].start_press.s.timestamp_ns;
        const int64_t tb=rows[i].end_press.s.timestamp_ns;
        const double x0=rows[i].start_press.s.px;
        const double y0=rows[i].start_press.s.py;
        const double z0=rows[i].start_press.s.pz;
        for(const auto& st:states){
          if(st.timestamp_ns<ta || st.timestamp_ns>tb) continue;
          pf<<(i+1)<<",A->B,"<<st.keyframe<<','<<st.timestamp_ns<<','
            <<(st.timestamp_ns-ta)/1e6<<','
            <<(st.px-x0)*1000.0<<','<<(st.py-y0)*1000.0<<','<<(st.pz-z0)*1000.0<<','
            <<st.vx<<','<<st.vy<<','<<st.vz<<','<<StableCapture::speed(st)<<'\n';
        }
        std::cout<<"CONTROL CSV "<<(i+1)<<"AB: "<<path
                 <<" motion_duration="<<gV26MoveDurationsSec[i]<<" s\n";
      }
    }
    saveCsvV8(rows);
    saveBackendTraceV8(states,rows,"/home/vio/jtzero_500mm_v25_backend.csv");
    if(imu_req){requestRate(serial_fd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);imu_req=false;}if(att_req){requestRate(serial_fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);att_req=false;}requestRate(serial_fd,sys,comp,MAVLINK_MSG_ID_DISTANCE_SENSOR,0);if(streaming){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(camera_fd,VIDIOC_STREAMOFF,&t);streaming=false;}for(auto&bf:buffers)if(bf.start&&bf.start!=MAP_FAILED)munmap(bf.start,bf.length);if(camera_fd!=-1)close(camera_fd);if(serial_fd!=-1)close(serial_fd);
    if(gV26ControlledAb4) std::cout<<"V26 MODE: CONTROLLED_AB4 — four A->B passes, target "<<gV26TargetMoveSec
      <<" s, accepted window "<<(gV26TargetMoveSec-gV26DurationToleranceSec)<<".."
      <<(gV26TargetMoveSec+gV26DurationToleranceSec)<<" s\n"; else std::cout<<"V25 MODE: A-B-A x2 closure test, strict gate, pure FRD->FLU, ZXY OFF, gravity feedback OFF, first_direction="<<(gV25BFirst?"B->A":"A->B")<<"\n";
    printSummaryV8(rows,raw,rejected,selected,decoded,imu_rx,imu_fed,att_rx,sync.size(),mapping,states.size());
    raw_combined.flush();raw_camera.flush();raw_mjpg.flush();raw_attitude.flush();raw_range.flush();
    std::cout<<"RAW IMU CSV: "<<kV11CombinedPath<<" rows="<<raw_imu_rows<<"\n"
             <<"RAW CAMERA CSV: "<<kV11CameraPath<<" rows="<<raw_camera_rows<<"\n"
             <<"RAW MJPEG: "<<kV11MjpegPath<<" bytes="<<raw_mjpg_offset<<"\n"
             <<"RAW ATTITUDE CSV: "<<kV11AttitudePath<<" rows="<<raw_attitude_rows<<"\n"
             <<"TF-LUNA RANGE CSV: "<<kV12RangePath<<" rows="<<raw_range_rows<<"\n"
             <<"LEGS CSV: "<<kV8CsvPath<<"\n"
             <<"BACKEND CSV: /home/vio/jtzero_500mm_v25_backend.csv\n"
             <<"FRONTEND CSV: /home/vio/jtzero_500mm_v25_frontend.csv rows="<<frontend_states.size()<<"\n"
             <<"EVENTS CSV: /home/vio/jtzero_500mm_v25_events.csv\n"
             <<"RAW MJPEG: DISABLED in V25 diagnostic\n"
             <<"LOOP STALLS >100ms: "<<loop_stall_count<<" max="<<std::fixed<<std::setprecision(3)
             <<max_loop_stall_ns/1e6<<" ms\n";
    double sd=0.0;
#ifdef JTZERO_SINGLE_LEG_MODE
    const bool metric_pass = rows.size()==1 && std::abs(rows[0].measured.error)<=kV8MaxLegErrorM && std::abs(rows[0].measured.dz)<=kV8MaxAbsDzM;
#else
    bool metric_pass=false;
    if(gV26ControlledAb4){
      if(rows.size()==4){
        double mean=0.0;
        for(const auto& r:rows) mean+=r.measured.horizontal;
        mean/=4.0;
        double ss=0.0;
        metric_pass=true;
        for(size_t i=0;i<rows.size();++i){
          const auto& r=rows[i];
          ss+=(r.measured.horizontal-mean)*(r.measured.horizontal-mean);
          metric_pass=metric_pass && std::abs(r.measured.error)<=kV8MaxLegErrorM && std::abs(r.measured.dz)<=kV8MaxAbsDzM;
          metric_pass=metric_pass && std::abs(gV26MoveDurationsSec[i]-gV26TargetMoveSec)<=gV26DurationToleranceSec;
        }
        sd=std::sqrt(ss/4.0);
        metric_pass=metric_pass && sd<=kV8MaxStdM;
      }
    }else{
      metric_pass=v8MeasurementPass(rows,&sd);
    }
#endif
    const bool pipeline_pass=!aborted&&rows.size()==kLegCount&&mapping.valid&&states.size()>0;
    std::cout<<"PIPELINE RESULT: "<<(pipeline_pass?"PASS":"FAIL")<<"\nMEASUREMENT RESULT: "<<(metric_pass?"PASS":"FAIL")<<"\n";
    const bool pass=pipeline_pass&&metric_pass;
    std::cout<<"RESULT: "<<(pass?"PASS":"FAIL")<<"\n";
    return pass?0:1;
  }catch(const std::exception&e){std::cerr<<"[FATAL] "<<e.what()<<"\n";if(pipeline)pipeline->shutdown();if(pipeline_started&&pipeline_thread.joinable())pipeline_thread.join();try{cv::destroyAllWindows();}catch(...){}if(serial_fd!=-1&&imu_req&&sys)try{requestRate(serial_fd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);}catch(...){}if(serial_fd!=-1&&att_req&&sys)try{requestRate(serial_fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);}catch(...){}if(serial_fd!=-1&&sys)try{requestRate(serial_fd,sys,comp,MAVLINK_MSG_ID_DISTANCE_SENSOR,0);}catch(...){}if(streaming&&camera_fd!=-1){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(camera_fd,VIDIOC_STREAMOFF,&t);}for(auto&bf:buffers)if(bf.start&&bf.start!=MAP_FAILED)munmap(bf.start,bf.length);if(camera_fd!=-1)close(camera_fd);if(serial_fd!=-1)close(serial_fd);return 1;}
}
