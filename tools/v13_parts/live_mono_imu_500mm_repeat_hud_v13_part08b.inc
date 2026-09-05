
      if(vio_feed_enabled&&!warmup_gate.passed){
        VioState ws;
        if(pipeline->latest(&ws)) warmup_gate.update(ws,now);
      }

      if(phase==CapturePhase::SETTLE_START||phase==CapturePhase::SETTLE_END){
        VioState s;if(pipeline->latest(&s)){Mark settled;if(stable.update(s,&settled)){
          if(phase==CapturePhase::SETTLE_START){start_mark=settled;phase=CapturePhase::MOVING;notice="СТАРТ СТАБИЛИЗИРОВАН — ДВИГАЙТЕ 500 ММ";notice_until=monotonicNs()+1500000000LL;std::cout<<"LEG "<<leg+1<<" START press KF="<<start_press.s.keyframe<<" settled KF="<<start_mark.s.keyframe<<" shift="<<markDistance(start_press,start_mark)*1000.0<<" mm [gravity feedback OFF]\n";}
          else {V8Leg row;row.start_press=start_press;row.end_press=end_press;row.measured=makeLeg500(leg,start_mark,settled);row.start_settle_shift_m=markDistance(start_press,start_mark);row.end_settle_shift_m=markDistance(end_press,settled);rows.push_back(row);std::cout<<"LEG "<<leg+1<<' '<<row.measured.direction<<" horizontal="<<std::fixed<<std::setprecision(2)<<row.measured.horizontal*1000.0<<" mm error="<<row.measured.error*1000.0<<" mm dz="<<row.measured.dz*1000.0<<" mm end_settle_shift="<<row.end_settle_shift_m*1000.0<<" mm\n";++leg;phase=CapturePhase::WAIT_START;notice="КОНЕЦ СТАБИЛИЗИРОВАН — СЛЕДУЮЩИЙ ПРОХОД";notice_until=monotonicNs()+1500000000LL;}
        }}
      }

      if(now>=next_hud){if(monotonicNs()>notice_until)notice.clear();drawHudV8(gray,*pipeline,fc,leg,phase,stable,rows,notice,warmup_gate,now);next_hud=now+kHudPeriodNs;}
      const int key=cv::waitKey(1)&0xff;if(key==27||key=='q'||key=='Q'){aborted=true;break;}
      if(key==32||key==13||key==10){VioState s;if(!pipeline->latest(&s)){notice="КООРДИНАТ ПОКА НЕТ";notice_until=monotonicNs()+1500000000LL;}
        else if(!warmup_gate.passed){notice="VIO WARM-UP НЕ ЗАВЕРШЕН — НЕ ДВИГАТЬ";notice_until=monotonicNs()+1500000000LL;std::cout<<"[WARMUP] START key ignored: t="<<std::fixed<<std::setprecision(2)<<warmup_gate.elapsedSec(monotonicNs())<<"s stable="<<warmup_gate.stableCount()<<"/"<<kWarmupStableStatesNeeded<<"\n";}
        else if(phase==CapturePhase::WAIT_START){start_press.s=s;start_press.valid=true;stable.reset();phase=CapturePhase::SETTLE_START;notice="НЕ ДВИГАТЬ — ФИКСИРУЮ СТАРТ";notice_until=monotonicNs()+1500000000LL;std::cout<<"LEG "<<leg+1<<" START key accepted KF="<<s.keyframe<<" [settling]\n";}
        else if(phase==CapturePhase::MOVING){end_press.s=s;end_press.valid=true;stable.reset();phase=CapturePhase::SETTLE_END;notice="НЕ ДВИГАТЬ — ФИКСИРУЮ КОНЕЦ";notice_until=monotonicNs()+1500000000LL;std::cout<<"LEG "<<leg+1<<" END key accepted KF="<<s.keyframe<<" [gravity feedback re-armed, settling]\n";}
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));pipeline->shutdown();if(pipeline_thread.joinable())pipeline_thread.join();pipeline_started=false;cv::destroyAllWindows();const auto states=pipeline->states();
    saveCsvV8(rows);
    saveBackendTraceV8(states,rows);
    if(imu_req){requestRate(serial_fd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);imu_req=false;}if(att_req){requestRate(serial_fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);att_req=false;}requestRate(serial_fd,sys,comp,MAVLINK_MSG_ID_DISTANCE_SENSOR,0);if(streaming){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(camera_fd,VIDIOC_STREAMOFF,&t);streaming=false;}for(auto&bf:buffers)if(bf.start&&bf.start!=MAP_FAILED)munmap(bf.start,bf.length);if(camera_fd!=-1)close(camera_fd);if(serial_fd!=-1)close(serial_fd);
    printSummaryV8(rows,raw,rejected,selected,decoded,imu_rx,imu_fed,att_rx,sync.size(),mapping,states.size());
    raw_combined.flush();raw_camera.flush();raw_mjpg.flush();raw_attitude.flush();raw_range.flush();
    std::cout<<"RAW IMU CSV: "<<kV11CombinedPath<<" rows="<<raw_imu_rows<<"\n"
             <<"RAW CAMERA CSV: "<<kV11CameraPath<<" rows="<<raw_camera_rows<<"\n"
             <<"RAW MJPEG: "<<kV11MjpegPath<<" bytes="<<raw_mjpg_offset<<"\n"
             <<"RAW ATTITUDE CSV: "<<kV11AttitudePath<<" rows="<<raw_attitude_rows<<"\n"
             <<"TF-LUNA RANGE CSV: "<<kV12RangePath<<" rows="<<raw_range_rows<<"\n"
             <<"LEGS CSV: "<<kV8CsvPath<<"\n"
             <<"BACKEND CSV: /home/vio/jtzero_500mm_v13_backend.csv\n";
    double sd=0.0;const bool metric_pass=v8MeasurementPass(rows,&sd);const bool pipeline_pass=!aborted&&rows.size()==kLegCount&&mapping.valid&&states.size()>0;std::cout<<"PIPELINE RESULT: "<<(pipeline_pass?"PASS":"FAIL")<<"\nMEASUREMENT RESULT: "<<(metric_pass?"PASS":"FAIL")<<"\n";const bool pass=pipeline_pass&&metric_pass;std::cout<<"RESULT: "<<(pass?"PASS":"FAIL")<<"\n";return pass?0:1;
  }catch(const std::exception&e){std::cerr<<"[FATAL] "<<e.what()<<"\n";if(pipeline)pipeline->shutdown();if(pipeline_started&&pipeline_thread.joinable())pipeline_thread.join();try{cv::destroyAllWindows();}catch(...){}if(serial_fd!=-1&&imu_req&&sys)try{requestRate(serial_fd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);}catch(...){}if(serial_fd!=-1&&att_req&&sys)try{requestRate(serial_fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);}catch(...){}if(serial_fd!=-1&&sys)try{requestRate(serial_fd,sys,comp,MAVLINK_MSG_ID_DISTANCE_SENSOR,0);}catch(...){}if(streaming&&camera_fd!=-1){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(camera_fd,VIDIOC_STREAMOFF,&t);}for(auto&bf:buffers)if(bf.start&&bf.start!=MAP_FAILED)munmap(bf.start,bf.length);if(camera_fd!=-1)close(camera_fd);if(serial_fd!=-1)close(serial_fd);return 1;}
}
