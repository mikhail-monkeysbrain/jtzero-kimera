// JT-ZERO Stage 11.7 v12: deterministic raw Camera+IMU yaw-only recorder.
// Protocol is fixed and timer-driven:
//   10 s STILL -> 15 s rotate about body Z (yaw ~+90 deg) -> 10 s STILL.
// No VIO is run here. The output is intended for replay_mono_imu_zxy_ab_v11.cpp.
//
// Outputs:
//   /home/vio/jtzero_yaw_only_v12.csv
//   /home/vio/jtzero_yaw_only_v12_camera.csv
//   /home/vio/jtzero_yaw_only_v12.mjpg

#define main jtzero_camera_imu_logger_unused_main_v12
#include "camera_imu_extrinsics_logger.cpp"
#undef main

#include <csignal>

namespace {
constexpr const char* V12_CSV="/home/vio/jtzero_yaw_only_v12.csv";
constexpr const char* V12_MJPEG="/home/vio/jtzero_yaw_only_v12.mjpg";
constexpr const char* V12_CAM_INDEX="/home/vio/jtzero_yaw_only_v12_camera.csv";
constexpr double V12_STILL1_SEC=10.0;
constexpr double V12_YAW_SEC=15.0;
constexpr double V12_STILL2_SEC=10.0;
constexpr double V12_TOTAL_SEC=V12_STILL1_SEC+V12_YAW_SEC+V12_STILL2_SEC;

struct AttitudeSample12 {
  int64_t recv_ns=0;
  int64_t source_ns=0;
  double roll_deg=0,pitch_deg=0,yaw_deg=0;
  double rollspeed=0,pitchspeed=0,yawspeed=0;
};

const char* phase12(double t){
  if(t<V12_STILL1_SEC)return "STILL_1";
  if(t<V12_STILL1_SEC+V12_YAW_SEC)return "YAW";
  return "STILL_2";
}

void cleanup12(int serial_fd,int camera_fd,bool streaming,
               std::vector<CameraBuffer>&buffers,uint8_t sys,uint8_t comp,bool rates){
  if(rates&&serial_fd>=0&&sys){
    try{
      requestRate(serial_fd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);
      requestRate(serial_fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);
    }catch(...){ }
  }
  if(streaming&&camera_fd>=0){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;ioctl(camera_fd,VIDIOC_STREAMOFF,&t);}
  for(auto&b:buffers)if(b.start&&b.length)munmap(b.start,b.length);
  if(camera_fd>=0)close(camera_fd);
  if(serial_fd>=0)close(serial_fd);
}
}

int main(){
  int serial_fd=-1,camera_fd=-1;bool streaming=false,rates=false;
  std::vector<CameraBuffer>buffers;uint8_t target_system=0,target_component=0;
  try{
    serial_fd=openSerial();
    std::cout<<"[MAV] waiting for HEARTBEAT...\n";
    mavlink_status_t ms{};mavlink_message_t mm{};
    int64_t deadline=monotonicNs()+10000000000LL;
    while(monotonicNs()<deadline&&target_system==0){
      pollfd p{serial_fd,POLLIN,0};
      if(poll(&p,1,100)>0){
        uint8_t b[2048];ssize_t n=read(serial_fd,b,sizeof(b));
        for(ssize_t i=0;i<n;++i){
          if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&mm,&ms)&&mm.msgid==MAVLINK_MSG_ID_HEARTBEAT){
            target_system=mm.sysid;target_component=mm.compid;break;
          }
        }
      }
    }
    if(!target_system)throw std::runtime_error("HEARTBEAT timeout");
    std::cout<<"[MAV] connected system="<<int(target_system)<<" component="<<int(target_component)<<"\n";
    requestRate(serial_fd,target_system,target_component,MAVLINK_MSG_ID_HIGHRES_IMU,IMU_RATE_HZ);
    requestRate(serial_fd,target_system,target_component,MAVLINK_MSG_ID_ATTITUDE,ATTITUDE_RATE_HZ);
    rates=true;

    camera_fd=open(CAMERA_DEVICE,O_RDWR|O_NONBLOCK);if(camera_fd==-1)fail("open camera");
    configureCamera(camera_fd);buffers=initCameraBuffers(camera_fd);
    v4l2_buf_type typ=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(xioctl(camera_fd,VIDIOC_STREAMON,&typ)==-1)fail("STREAMON");streaming=true;
    discardWarmup(camera_fd,CAMERA_WARMUP_FRAMES);

    tcflush(serial_fd,TCIFLUSH);std::memset(&ms,0,sizeof(ms));std::memset(&mm,0,sizeof(mm));
    std::vector<CameraSample>camera;std::vector<ImuSample>imu;std::vector<TimeSyncSample>ts;std::vector<AttitudeSample12>att;
    camera.reserve(10000);imu.reserve(10000);ts.reserve(500);att.reserve(3000);
    std::ofstream mjpeg(V12_MJPEG,std::ios::binary|std::ios::trunc);if(!mjpeg)throw std::runtime_error("Cannot create MJPEG output");

    uint64_t drops=0;bool have_seq=false;uint32_t prev_seq=0;int64_t pending=0,next_ts=monotonicNs();
    const int64_t start_ns=monotonicNs();
    int last_phase=-1,last_sec=-1;

    std::cout<<"\n================ YAW-ONLY RAW RECORDER V12 ================\n"
             <<"Protocol: 10s STILL -> 15s YAW ~+90 deg -> 10s STILL\n"
             <<"Keep height and roll/pitch as constant as practical.\n"
             <<"Do NOT translate intentionally.\n"
             <<"Recording starts NOW.\n\n";

    while(true){
      const int64_t now=monotonicNs();const double elapsed=(now-start_ns)*1e-9;
      if(elapsed>=V12_TOTAL_SEC)break;
      const int ph=elapsed<V12_STILL1_SEC?0:(elapsed<V12_STILL1_SEC+V12_YAW_SEC?1:2);
      if(ph!=last_phase){
        last_phase=ph;
        if(ph==0)std::cout<<">>> STILL: do not move <<<\n";
        else if(ph==1)std::cout<<"\n>>> YAW NOW: rotate smoothly about vertical axis ~+90 deg during 15 s <<<\n";
        else std::cout<<"\n>>> STOP: hold completely still <<<\n";
      }
      const int sec=(int)elapsed;
      if(sec!=last_sec){last_sec=sec;std::cout<<"["<<phase12(elapsed)<<"] t="<<std::fixed<<std::setprecision(1)<<elapsed<<" s\n";}

      if(now>=next_ts&&pending==0){pending=now;sendTimesync(serial_fd,pending,target_system,target_component);next_ts=now+TIMESYNC_PERIOD_NS;}
      pollfd pf[2]={{camera_fd,POLLIN,0},{serial_fd,POLLIN,0}};
      int rc=poll(pf,2,2);if(rc<0){if(errno==EINTR)continue;fail("poll");}

      if(pf[0].revents&POLLIN){
        for(;;){
          v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;
          if(xioctl(camera_fd,VIDIOC_DQBUF,&b)==-1){if(errno==EAGAIN)break;fail("DQBUF");}
          const int64_t recv=monotonicNs(),v4=timevalToNs(b.timestamp),corr=jtzero::timesync::correctCameraTimestampNs(v4);
          if(have_seq){uint32_t ex=prev_seq+1;if(b.sequence!=ex)drops+=uint32_t(b.sequence-ex);}prev_seq=b.sequence;have_seq=true;
          const uint64_t off=(uint64_t)mjpeg.tellp();mjpeg.write((const char*)buffers[b.index].start,b.bytesused);
          camera.push_back({recv,v4,corr,b.sequence,b.flags,b.bytesused,off});
          if(xioctl(camera_fd,VIDIOC_QBUF,&b)==-1)fail("QBUF");
        }
      }

      if(pf[1].revents&POLLIN){
        uint8_t by[8192];
        for(;;){
          ssize_t n=read(serial_fd,by,sizeof(by));if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;
          for(ssize_t i=0;i<n;++i){
            if(!mavlink_parse_char(MAVLINK_COMM_0,by[i],&mm,&ms))continue;
            const int64_t recv=monotonicNs();
            if(mm.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){
              mavlink_highres_imu_t a{};mavlink_msg_highres_imu_decode(&mm,&a);ImuSample s;
              s.recv_ns=recv;s.fc_ns=int64_t(a.time_usec)*1000;s.xacc=a.xacc;s.yacc=a.yacc;s.zacc=a.zacc;
              s.xgyro=a.xgyro;s.ygyro=a.ygyro;s.zgyro=a.zgyro;s.temperature=a.temperature;s.fields_updated=a.fields_updated;
#ifdef MAVLINK_MSG_HIGHRES_IMU_FIELD_ID_LEN
              s.imu_id=a.id;
#endif
              imu.push_back(s);
            }else if(mm.msgid==MAVLINK_MSG_ID_ATTITUDE){
              mavlink_attitude_t a{};mavlink_msg_attitude_decode(&mm,&a);
              att.push_back({recv,(int64_t)a.time_boot_ms*1000000LL,rad2deg(a.roll),rad2deg(a.pitch),rad2deg(a.yaw),a.rollspeed,a.pitchspeed,a.yawspeed});
            }else if(mm.msgid==MAVLINK_MSG_ID_TIMESYNC){
              mavlink_timesync_t a{};mavlink_msg_timesync_decode(&mm,&a);
              if(a.tc1!=0&&pending!=0&&a.ts1==pending){TimeSyncSample s;s.t0_rpi_ns=pending;s.t1_rpi_ns=recv;s.fc_ns=a.tc1;s.rtt_ns=recv-pending;s.rpi_mid_ns=pending+s.rtt_ns/2;s.good=s.rtt_ns>0&&nsToMs(s.rtt_ns)<=MAX_TIMESYNC_RTT_MS;ts.push_back(s);pending=0;}
            }
          }
        }
      }
      if(pending&&monotonicNs()-pending>20000000LL)pending=0;
    }

    mjpeg.flush();mjpeg.close();
    ClockMapping map=estimateClockMapping(ts);if(!map.valid)throw std::runtime_error("Not enough valid TIMESYNC samples");

    std::ofstream ci(V12_CAM_INDEX);if(!ci)throw std::runtime_error("Cannot create camera index");
    ci<<"sequence,v4l2_timestamp_ns,camera_timestamp_corrected_ns,recv_rpi_ns,delivery_latency_ms,mjpeg_offset,bytes_used,flags\n"<<std::fixed<<std::setprecision(9);
    for(auto&s:camera)ci<<s.sequence<<','<<s.v4l2_ns<<','<<s.corrected_ns<<','<<s.recv_ns<<','<<nsToMs(s.recv_ns-s.v4l2_ns)<<','<<s.mjpeg_offset<<','<<s.bytes_used<<','<<cameraTimestampFlags(s.flags)<<'\n';
    ci.close();

    std::ofstream csv(V12_CSV);if(!csv)throw std::runtime_error("Cannot create combined CSV");
    csv<<"event,recv_rpi_ns,source_timestamp_ns,mapped_rpi_ns,transport_latency_ms,camera_sequence,camera_flags,camera_bytes,xacc_m_s2,yacc_m_s2,zacc_m_s2,xgyro_rad_s,ygyro_rad_s,zgyro_rad_s,temperature_c,imu_id,timesync_t0_rpi_ns,timesync_t1_rpi_ns,timesync_fc_ns,timesync_mid_rpi_ns,timesync_rtt_ms,timesync_good,map_a,map_drift_ppm,map_fc_ref_ns,map_rpi_ref_ns,fc_roll_deg,fc_pitch_deg,fc_yaw_deg,fc_rollspeed,fc_pitchspeed,fc_yawspeed\n"<<std::fixed<<std::setprecision(9);
    for(auto&s:camera)csv<<"CAMERA,"<<s.recv_ns<<','<<s.v4l2_ns<<','<<s.corrected_ns<<','<<nsToMs(s.recv_ns-s.v4l2_ns)<<','<<s.sequence<<','<<cameraTimestampFlags(s.flags)<<','<<s.bytes_used<<",,,,,,,,,,,,,,,"<<double(map.a)<<','<<map.drift_ppm<<','<<map.fc_ref_ns<<','<<int64_t(std::llround(map.rpi_ref_ns))<<",,,,,,\n";
    for(auto&s:imu){const int64_t mn=map.map(s.fc_ns);csv<<"IMU,"<<s.recv_ns<<','<<s.fc_ns<<','<<mn<<','<<nsToMs(s.recv_ns-mn)<<",,,,"<<s.xacc<<','<<s.yacc<<','<<s.zacc<<','<<s.xgyro<<','<<s.ygyro<<','<<s.zgyro<<','<<s.temperature<<','<<int(s.imu_id)<<",,,,,,"<<double(map.a)<<','<<map.drift_ppm<<','<<map.fc_ref_ns<<','<<int64_t(std::llround(map.rpi_ref_ns))<<",,,,,,\n";}
    for(auto&s:ts)csv<<"TIMESYNC,"<<s.t1_rpi_ns<<",,,,,,,,,,,,,,,,"<<s.t0_rpi_ns<<','<<s.t1_rpi_ns<<','<<s.fc_ns<<','<<s.rpi_mid_ns<<','<<nsToMs(s.rtt_ns)<<','<<(s.good?1:0)<<','<<double(map.a)<<','<<map.drift_ppm<<','<<map.fc_ref_ns<<','<<int64_t(std::llround(map.rpi_ref_ns))<<",,,,,,\n";
    for(auto&s:att){const int64_t mn=map.map(s.source_ns);csv<<"ATTITUDE,"<<s.recv_ns<<','<<s.source_ns<<','<<mn<<','<<nsToMs(s.recv_ns-mn)<<",,,,,,,,,,,,,,,,,,"<<double(map.a)<<','<<map.drift_ppm<<','<<map.fc_ref_ns<<','<<int64_t(std::llround(map.rpi_ref_ns))<<','<<s.roll_deg<<','<<s.pitch_deg<<','<<s.yaw_deg<<','<<s.rollspeed<<','<<s.pitchspeed<<','<<s.yawspeed<<'\n';}
    csv.close();

    size_t good=0;for(auto&s:ts)if(s.good)++good;
    std::cout<<"\n================ YAW-ONLY V12 RESULT ================\n"
             <<"camera frames: "<<camera.size()<<"\n"
             <<"camera source drops: "<<drops<<"\n"
             <<"IMU rows: "<<imu.size()<<"\n"
             <<"ATTITUDE rows: "<<att.size()<<"\n"
             <<"TIMESYNC good: "<<good<<'/'<<ts.size()<<"\n"
             <<"clock drift ppm: "<<std::fixed<<std::setprecision(3)<<map.drift_ppm<<"\n"
             <<"combined CSV: "<<V12_CSV<<"\n"
             <<"camera index: "<<V12_CAM_INDEX<<"\n"
             <<"MJPEG: "<<V12_MJPEG<<"\n"
             <<"RECORDER RESULT: PASS\n";

    cleanup12(serial_fd,camera_fd,streaming,buffers,target_system,target_component,rates);
    return 0;
  }catch(const std::exception&e){
    std::cerr<<"[FATAL] "<<e.what()<<"\n";
    cleanup12(serial_fd,camera_fd,streaming,buffers,target_system,target_component,rates);
    return 1;
  }
}
