              raw_combined<<"IMU,"<<recv<<','<<((int64_t)h.time_usec*1000LL)<<','<<mapped_imu_ns<<','<<h.time_usec
                          <<",0,0,0,"<<h.xacc<<','<<h.yacc<<','<<h.zacc<<','
                          <<h.xgyro<<','<<h.ygyro<<','<<h.zgyro<<'\n';
              ++raw_imu_rows;
              const bool moving=(phase==CapturePhase::MOVING);(void)moving;const Eigen::Vector3d w=imu_correction.correctGyro(h.time_usec,acc,gyro,false,false);VIO::ImuAccGyr data;data<<acc.x(),acc.y(),acc.z(),w.x(),w.y(),w.z();pipeline->fillSingleImuQueue(VIO::ImuMeasurement(mapped_imu_ns,data));++imu_fed;}
        else if(msg.msgid==MAVLINK_MSG_ID_ATTITUDE){mavlink_attitude_t a{};mavlink_msg_attitude_decode(&msg,&a);++att_rx;fc.valid=true;fc.wall_ns=recv;fc.roll_deg=a.roll*180.0/kPi;fc.pitch_deg=a.pitch*180.0/kPi;fc.yaw_deg=a.yaw*180.0/kPi;
          if(vio_feed_enabled){raw_attitude<<recv<<','<<a.time_boot_ms<<','<<fc.roll_deg<<','<<fc.pitch_deg<<','<<fc.yaw_deg<<','<<a.rollspeed<<','<<a.pitchspeed<<','<<a.yawspeed<<'\n';++raw_attitude_rows;}}
        else if(msg.msgid==MAVLINK_MSG_ID_DISTANCE_SENSOR){
          mavlink_distance_sensor_t d{};mavlink_msg_distance_sensor_decode(&msg,&d);
          if(vio_feed_enabled){
            const double r_m=0.01*static_cast<double>(d.current_distance);
            const double rr=fc.valid?fc.roll_deg*kPi/180.0:0.0;
            const double pp=fc.valid?fc.pitch_deg*kPi/180.0:0.0;
            const double vertical_m=r_m*std::cos(rr)*std::cos(pp);
            raw_range<<recv<<','<<d.time_boot_ms<<','<<d.min_distance<<','<<d.max_distance<<','<<d.current_distance<<','
                     <<static_cast<int>(d.type)<<','<<static_cast<int>(d.id)<<','<<static_cast<int>(d.orientation)<<','
                     <<static_cast<int>(d.covariance)<<','<<d.horizontal_fov<<','<<d.vertical_fov<<','
                     <<d.quaternion[0]<<','<<d.quaternion[1]<<','<<d.quaternion[2]<<','<<d.quaternion[3]<<','
                     <<static_cast<int>(d.signal_quality)<<','
                     <<(fc.valid?fc.roll_deg:0.0)<<','<<(fc.valid?fc.pitch_deg:0.0)<<','<<(fc.valid?fc.yaw_deg:0.0)<<','
                     <<vertical_m<<'\n';
            ++raw_range_rows;
          }
        }
      }}}
      if(pending&&monotonicNs()-pending>20000000LL)pending=0;

      if(pf[0].revents&POLLIN){for(;;){v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;if(xioctl(camera_fd,VIDIOC_DQBUF,&b)==-1){if(errno==EAGAIN)break;fail("DQBUF");}++raw;const int64_t raw_ts_ns=timevalToNs(b.timestamp);const int64_t corrected=jtzero::timesync::correctCameraTimestampNs(raw_ts_ns);
        // V22 diagnostic: do NOT synchronously write raw MJPEG in the acquisition loop.
        // This isolates filesystem/write stalls from camera + MAVLink ingestion.
        const uint64_t this_offset=0;
        if(vio_feed_enabled&&b.bytesused>0){
          ++raw_camera_rows;
        }
        bool ok=true;if(have_prev){const int64_t dt=corrected-prev_ts;ok=b.sequence==prev_seq+1U&&dt>0&&dt<=20000000LL;if(!ok)++rejected;}prev_seq=b.sequence;prev_ts=corrected;have_prev=true;const bool due=last_sel==0||corrected-last_sel>=30000000LL;
        if(vio_feed_enabled){
          raw_camera<<b.sequence<<','<<raw_ts_ns<<','<<corrected<<','<<(mapping.valid?1:0)<<','<<((ok&&due&&mapping.valid)?1:0)<<','<<this_offset<<','<<b.bytesused<<'\n';
        }
        if(ok&&due&&mapping.valid&&vio_feed_enabled){std::vector<unsigned char>jpg(b.bytesused);std::memcpy(jpg.data(),buffers[b.index].start,b.bytesused);cv::Mat g=cv::imdecode(jpg,cv::IMREAD_GRAYSCALE);if(!g.empty()){++decoded;gray=g;pipeline->fillLeftFrameQueue(std::make_unique<VIO::Frame>(fid++,corrected,vp.camera_params_.at(0),g.clone()));last_sel=corrected;++selected;}}if(xioctl(camera_fd,VIDIOC_QBUF,&b)==-1)fail("QBUF");}}
