// JT-ZERO v21: короткий стендовый YAW +30 -> 0 -> -30 -> 0.
// Работает поверх v20 и предназначен для TRUE IMU-only режима.
// GUI русский, переходы между этапами автоматические после устойчивой фиксации.

#define main jtzero_v20_embedded_main
#include "live_mono_imu_optimizer_update_v20.cpp"
#undef main

namespace jtzero_v21 {
using namespace jtzero_v20;
using namespace jtzero_v10;

constexpr const char* kCsv21 = "/home/vio/jtzero_live_yaw_short_v21.csv";
constexpr const char* kWindow21 = "JT-ZERO: КОРОТКИЙ YAW v21";
constexpr double kTargets21[4] = {30.0, 0.0, -30.0, 0.0};
constexpr const char* kPhaseName21[4] = {"+30", "0_after_plus", "-30", "0_final"};
constexpr int kStable21 = 20;
constexpr double kYawTol21 = 2.0;
constexpr double kGyroStill21 = 0.08;
constexpr double kAccTol21 = 0.35;

struct Seg21 {
  size_t rows = 0;
  double max_pim_vxy = 0.0;
  double max_opt_vxy = 0.0;
  double max_fc_pred_grav = 0.0;
  double max_pred_opt_grav = 0.0;
  double max_dv = 0.0;
};

static void updateSeg21(Seg21& s, const Row20& r) {
  ++s.rows;
  s.max_pim_vxy = std::max(s.max_pim_vxy, r.pim_vxy);
  s.max_opt_vxy = std::max(s.max_opt_vxy, r.opt_vxy);
  s.max_fc_pred_grav = std::max(s.max_fc_pred_grav, r.fc_pred_grav);
  s.max_pred_opt_grav = std::max(s.max_pred_opt_grav, r.pred_opt_grav);
  s.max_dv = std::max(s.max_dv, r.dv);
}

static void save21(const Log20& log, const std::vector<int>& phases) {
  std::ofstream f(kCsv21, std::ios::trunc);
  f << std::fixed << std::setprecision(9);
  f << "phase,phase_name,target_yaw_deg,imu_us,imu_rpi_ns,keyframe,prev_keyframe,backend_ns,backend_age_ms,gyro_norm,fc_pred_gravity_deg,fc_opt_gravity_deg,pred_opt_rotation_deg,pred_opt_gravity_deg,dV_mps,dBA_mps2,dBG_rps,pim_vxy,opt_vxy,";
  f << "vpred_x,vpred_y,vpred_z,vopt_x,vopt_y,vopt_z,baprev_x,baprev_y,baprev_z,baopt_x,baopt_y,baopt_z,bgprev_x,bgprev_y,bgprev_z,bgopt_x,bgopt_y,bgopt_z\n";
  for (size_t i = 0; i < log.rows.size(); ++i) {
    const auto& r = log.rows[i];
    int p = i < phases.size() ? phases[i] : -1;
    const char* name = (p >= 0 && p < 4) ? kPhaseName21[p] : "unknown";
    double target = (p >= 0 && p < 4) ? kTargets21[p] : 0.0;
    f << p << ',' << name << ',' << target << ','
      << r.imu_us << ',' << r.imu_rpi_ns << ',' << r.keyframe << ',' << r.prev_keyframe << ','
      << r.backend_ns << ',' << r.backend_age_ms << ',' << r.gyro_norm << ','
      << r.fc_pred_grav << ',' << r.fc_opt_grav << ',' << r.pred_opt_rot << ',' << r.pred_opt_grav << ','
      << r.dv << ',' << r.dba << ',' << r.dbg << ',' << r.pim_vxy << ',' << r.opt_vxy << ','
      << r.Vpred.x() << ',' << r.Vpred.y() << ',' << r.Vpred.z() << ','
      << r.Vopt.x() << ',' << r.Vopt.y() << ',' << r.Vopt.z() << ','
      << r.BAprev.x() << ',' << r.BAprev.y() << ',' << r.BAprev.z() << ','
      << r.BAopt.x() << ',' << r.BAopt.y() << ',' << r.BAopt.z() << ','
      << r.BGprev.x() << ',' << r.BGprev.y() << ',' << r.BGprev.z() << ','
      << r.BGopt.x() << ',' << r.BGopt.y() << ',' << r.BGopt.z() << '\n';
  }
}

static void hud21(const cv::Mat& gray, const Telemetry& tel, bool ready, bool done,
                  int phase, int stable, const Log20& log, double ref_accum) {
  const cv::Scalar bg(15,15,15), white(235,235,235), green(80,220,80), red(40,40,245), yellow(0,220,255), muted(150,150,150);
  cv::Mat c(940,1440,CV_8UC3,bg), video;
  if(gray.empty()) video=cv::Mat(kHeight,kWidth,CV_8UC3,cv::Scalar(0)); else cv::cvtColor(gray,video,cv::COLOR_GRAY2BGR);
  cv::resize(video,video,{820,615},0,0,cv::INTER_NEAREST); video.copyTo(c(cv::Rect(20,80,820,615)));
  uiText(c,"JT-ZERO: КОРОТКИЙ YAW ±30° v21",20,48,.72,white,2);
  double yy=0,gn=0,an=0; {
    std::lock_guard<std::mutex> l(tel.mutex);
    yy=tel.fc_accum_yaw-ref_accum;
    gn=std::sqrt(tel.gx*tel.gx+tel.gy*tel.gy+tel.gz*tel.gz);
    an=std::sqrt(tel.ax*tel.ax+tel.ay*tel.ay+tel.az*tel.az);
  }
  double target=(phase>=0&&phase<4)?kTargets21[phase]:0.0;
  if(!ready){
    uiText(c,"СТЕНД НЕПОДВИЖЕН",225,285,.90,white,2);
    uiText(c,"SPACE — ЗАФИКСИРОВАТЬ НОЛЬ",190,355,.70,yellow,2);
  } else if(done) {
    uiText(c,"ТЕСТ ЗАВЕРШЁН",250,300,1.05,green,3);
  } else {
    char t[128]; std::snprintf(t,sizeof(t),"ПОВЕРНИТЕ YAW К %+.0f° И ОСТАНОВИТЕСЬ",target);
    uiText(c,t,120,300,.70,yellow,3);
    char p[128]; std::snprintf(p,sizeof(p),"Этап %d/4",phase+1); uiText(c,p,300,365,.52,white,2);
  }
  cv::Mat s=c(cv::Rect(860,70,560,850)); cv::rectangle(s,{0,0},{559,849},cv::Scalar(45,45,45),1);
  drawBar(s,"YAW",yy,-60,60,target,kYawTol21,145);
  char b[256];
  std::snprintf(b,sizeof(b),"YAW: %+.2f°   цель: %+.0f°",yy,target); uiText(s,b,18,205,.44,white,1);
  std::snprintf(b,sizeof(b),"Стабильность %d/%d",stable,kStable21); uiText(s,b,18,245,.44,white,1);
  std::snprintf(b,sizeof(b),"|gyro| %.3f rad/s",gn); uiText(s,b,18,285,.44,gn<=kGyroStill21?green:red,1);
  std::snprintf(b,sizeof(b),"|acc| %.3f m/s²",an); uiText(s,b,18,325,.44,std::abs(an-9.81)<=kAccTol21?green:red,1);
  if(!log.rows.empty()){
    const auto&r=log.rows.back();
    std::snprintf(b,sizeof(b),"PIM Vxy: %.3f м/с",r.pim_vxy); uiText(s,b,18,395,.52,r.pim_vxy>.5?red:green,2);
    std::snprintf(b,sizeof(b),"OPT Vxy: %.3f м/с",r.opt_vxy); uiText(s,b,18,440,.48,r.opt_vxy>.5?red:green,2);
    std::snprintf(b,sizeof(b),"FC→PRED gravity: %.3f°",r.fc_pred_grav); uiText(s,b,18,485,.46,r.fc_pred_grav>1.0?red:green,2);
    std::snprintf(b,sizeof(b),"PRED→OPT gravity: %.6f°",r.pred_opt_grav); uiText(s,b,18,530,.42,white,1);
    std::snprintf(b,sizeof(b),"ΔV optimizer: %.6f м/с",r.dv); uiText(s,b,18,570,.42,white,1);
    std::snprintf(b,sizeof(b),"KF: %lld",(long long)r.keyframe); uiText(s,b,18,615,.44,muted,1);
  }
  uiText(s,"Маршрут: +30 → 0 → -30 → 0",18,710,.42,muted,1);
  uiText(s,"TRUE IMU-only должен быть включён",18,750,.40,muted,1);
  uiText(c,"ESC / Q — прервать   SPACE — старт/выход",25,910,.42,muted,1);
  uiText(c,std::string("CSV: ")+kCsv21,760,910,.38,muted,1);
  cv::imshow(kWindow21,c);
}

} // namespace jtzero_v21

int main(int argc,char**argv){
  using namespace jtzero_v21; using namespace jtzero_v20; using namespace jtzero_v10;
  google::InitGoogleLogging(argv[0]);
  FLAGS_no_incremental_pose=true; FLAGS_visualize=false; FLAGS_viz_type=2; FLAGS_use_lcd=false; FLAGS_log_output=false; FLAGS_extract_planes_from_the_scene=false;
  if(!std::getenv("JTZERO_DIAG_IMU_ONLY")){std::cerr<<"[FATAL] JTZERO_DIAG_IMU_ONLY is not set\n";return 1;}
  const std::string params=argc>1?argv[1]:"params/JTZeroMonoFLUZeroLever";
  int cfd=-1,sfd=-1; bool streaming=false,pipeline_started=false,aborted=false,imu_req=false,att_req=false; uint8_t sys=0,comp=0;
  std::vector<CameraBuffer> bufs; std::shared_ptr<Pipeline20> pipe; std::thread pipe_thread; Telemetry tel;
  try{
    VIO::VioParams vp(params); pipe=std::make_shared<Pipeline20>(vp); pipe->installCallback(); pipe_thread=std::thread([pipe](){pipe->spin();}); pipeline_started=true;
    sfd=openSerial(); mavlink_status_t mst{}; mavlink_message_t msg{}; std::cout<<"[MAV] waiting for HEARTBEAT...\n"; int64_t dl=monotonicNs()+10000000000LL;
    while(monotonicNs()<dl&&!sys){pollfd p{sfd,POLLIN,0}; if(poll(&p,1,100)<=0)continue; uint8_t b[2048]; ssize_t n=read(sfd,b,sizeof(b)); if(n<=0)continue; for(ssize_t i=0;i<n;++i) if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst)&&msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=msg.sysid;comp=msg.compid;break;}}
    if(!sys)throw std::runtime_error("HEARTBEAT timeout");
    requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,kImuRateHz); imu_req=true; requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,kAttRateHz); att_req=true;
    cfd=open(kCameraDevice,O_RDWR|O_NONBLOCK); if(cfd==-1)fail("open camera"); configureCamera(cfd); bufs=initCameraBuffers(cfd); v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE; if(xioctl(cfd,VIDIOC_STREAMON,&type)==-1)fail("STREAMON"); streaming=true; discardWarmup(cfd);
    cv::setNumThreads(1); cv::namedWindow(kWindow21,cv::WINDOW_NORMAL); cv::resizeWindow(kWindow21,1440,940);
    std::vector<TimeSyncSample> sync; ClockMapping mapping; int64_t pending_sync=0,next_sync=monotonicNs(),next_hud=monotonicNs(); uint32_t prev_seq=0; int64_t prev_ts=0,last_sel=0; bool have_prev=false; VIO::FrameId fid=0; cv::Mat last_gray(kHeight,kWidth,CV_8UC1,cv::Scalar(0));
    std::deque<ImuSample10> iq; AttSample10 prev_att{},cur_att{}; bool have_att=false; Log20 log; std::vector<int> row_phases; Seg21 seg[4];
    bool ready=false,done=false; int phase=0,stable=0; double ref_accum=0;
    std::cout<<"\nJT-ZERO SHORT YAW v21\nSPACE fixes zero. Test: +30 -> 0 -> -30 -> 0.\n";
    while(true){
      const int64_t now=monotonicNs(); if(now>=next_sync&&pending_sync==0){pending_sync=now;sendTimesync(sfd,pending_sync,sys,comp);next_sync=now+kTimesyncPeriodNs;}
      pollfd pf[2]={{cfd,POLLIN,0},{sfd,POLLIN,0}}; int rc=poll(pf,2,2); if(rc<0){if(errno==EINTR)continue;fail("poll");}
      if(pf[1].revents&POLLIN){uint8_t b[8192];for(;;){ssize_t n=read(sfd,b,sizeof(b));if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;for(ssize_t i=0;i<n;++i){if(!mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst))continue;const int64_t rx=monotonicNs();
        if(msg.msgid==MAVLINK_MSG_ID_TIMESYNC){mavlink_timesync_t ts{};mavlink_msg_timesync_decode(&msg,&ts);if(ts.tc1!=0&&pending_sync&&ts.ts1==pending_sync){TimeSyncSample q;q.t0_rpi_ns=pending_sync;q.t1_rpi_ns=rx;q.fc_ns=ts.tc1;q.rtt_ns=rx-pending_sync;q.rpi_mid_ns=pending_sync+q.rtt_ns/2;q.good=q.rtt_ns>0&&nsToMs(q.rtt_ns)<=kMaxTimesyncRttMs;sync.push_back(q);pending_sync=0;mapping=estimateClockMapping(sync);}}
        else if(msg.msgid==MAVLINK_MSG_ID_ATTITUDE){mavlink_attitude_t a{};mavlink_msg_attitude_decode(&msg,&a);AttSample10 na{(uint64_t)a.time_boot_ms*1000ULL,a.roll*180.0/kPi,a.pitch*180.0/kPi,a.yaw*180.0/kPi};{std::lock_guard<std::mutex>l(tel.mutex);if(tel.have_prev_yaw)tel.fc_accum_yaw+=wrapDeg(na.yaw-tel.prev_yaw);tel.prev_yaw=na.yaw;tel.have_prev_yaw=true;tel.fc_roll=na.roll;tel.fc_pitch=na.pitch;tel.fc_yaw=na.yaw;tel.fc_valid=true;}
          if(have_att){prev_att=cur_att;cur_att=na;while(!iq.empty()&&iq.front().us<=cur_att.us){auto im=iq.front();if(im.us>=prev_att.us){size_t before=log.rows.size();process20(log,im,interpR(prev_att,cur_att,im.us),*pipe,mapping);if(log.rows.size()>before){row_phases.push_back(phase);updateSeg21(seg[phase],log.rows.back());}}iq.pop_front();}}else{cur_att=na;have_att=true;}
          if(ready&&!done){double yy,gn,an;{std::lock_guard<std::mutex>l(tel.mutex);yy=tel.fc_accum_yaw-ref_accum;gn=std::sqrt(tel.gx*tel.gx+tel.gy*tel.gy+tel.gz*tel.gz);an=std::sqrt(tel.ax*tel.ax+tel.ay*tel.ay+tel.az*tel.az);}bool ok=std::abs(yy-kTargets21[phase])<=kYawTol21&&gn<=kGyroStill21&&std::abs(an-9.81)<=kAccTol21;if(ok)++stable;else stable=0;if(stable>=kStable21){stable=0;std::cout<<"[STEP] "<<kPhaseName21[phase]<<" fixed.\n";if(phase==3){done=true;std::cout<<"[TEST] completed.\n";}else{++phase;std::cout<<"[NEXT] target "<<kTargets21[phase]<<" deg.\n";}}}}
        else if(msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){mavlink_highres_imu_t h{};mavlink_msg_highres_imu_decode(&msg,&h);ImuSample10 im{h.time_usec,h.xacc,-h.yacc,-h.zacc,h.xgyro,-h.ygyro,-h.zgyro};{std::lock_guard<std::mutex>l(tel.mutex);tel.ax=im.ax;tel.ay=im.ay;tel.az=im.az;tel.gx=im.gx;tel.gy=im.gy;tel.gz=im.gz;}iq.push_back(im);while(iq.size()>300)iq.pop_front();if(mapping.valid&&rx-mapping.last_update_ns<=kMappingStaleNs){VIO::ImuAccGyr d;d<<im.ax,im.ay,im.az,im.gx,im.gy,im.gz;pipe->fillSingleImuQueue(VIO::ImuMeasurement(mapping.map((int64_t)h.time_usec*1000LL),d));}}
      }}}
      if(pending_sync&&monotonicNs()-pending_sync>20000000LL)pending_sync=0;
      if(pf[0].revents&POLLIN){for(;;){v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;if(xioctl(cfd,VIDIOC_DQBUF,&b)==-1){if(errno==EAGAIN)break;fail("DQBUF");}const int64_t ts=jtzero::timesync::correctCameraTimestampNs(timevalToNs(b.timestamp));bool ok=true;if(have_prev){int64_t d=ts-prev_ts;ok=b.sequence==prev_seq+1U&&d>0&&d<=20000000LL;}prev_seq=b.sequence;prev_ts=ts;have_prev=true;bool due=last_sel==0||ts-last_sel>=30000000LL;if(ok&&due&&mapping.valid){std::vector<unsigned char>jpeg(b.bytesused);std::memcpy(jpeg.data(),bufs[b.index].start,b.bytesused);cv::Mat gray=cv::imdecode(jpeg,cv::IMREAD_GRAYSCALE);if(!gray.empty()){last_gray=gray;pipe->fillLeftFrameQueue(std::make_unique<VIO::Frame>(fid++,ts,vp.camera_params_.at(0),gray.clone()));last_sel=ts;}}if(xioctl(cfd,VIDIOC_QBUF,&b)==-1)fail("QBUF");}}
      if(now>=next_hud){hud21(last_gray,tel,ready,done,phase,stable,log,ref_accum);next_hud=now+kHudPeriodNs;}
      int key=cv::waitKey(1)&0xff; if(key==27||key=='q'||key=='Q'){if(!done)aborted=true;break;}
      if(key==' '&&!ready){Backend20 bb;bool bv=pipe->latest(&bb),fv=false;Eigen::Matrix3d Rfc0=Eigen::Matrix3d::Identity();{std::lock_guard<std::mutex>l(tel.mutex);fv=tel.fc_valid;if(fv){ref_accum=tel.fc_accum_yaw;Rfc0=fcRnedFlu(tel.fc_roll,tel.fc_pitch,tel.fc_yaw);}}if(bv&&fv&&have_att){log.world_map=bb.Ropt*Rfc0.transpose();log.armed=true;log.last_logged_kf=-1;ready=true;phase=0;std::cout<<"[ZERO] KF "<<bb.keyframe<<". Target +30 deg.\n";}}
      else if(key==' '&&done)break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(250)); pipe->shutdown(); if(pipe_thread.joinable())pipe_thread.join(); pipeline_started=false; save21(log,row_phases); cv::destroyAllWindows();
    if(imu_req)requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0); if(att_req)requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0); if(streaming){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);} for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length); if(cfd>=0)close(cfd); if(sfd>=0)close(sfd);
    std::cout<<"\n============================================================\nJT-ZERO SHORT YAW v21 RESULT\n============================================================\n";
    std::cout<<"aborted: "<<(aborted?"yes":"no")<<"\ncompleted: "<<(done?"yes":"no")<<"\nkeyframes logged: "<<log.rows.size()<<"\n";
    for(int p=0;p<4;++p){std::cout<<kPhaseName21[p]<<": rows="<<seg[p].rows<<" max PIM_Vxy="<<seg[p].max_pim_vxy<<" max OPT_Vxy="<<seg[p].max_opt_vxy<<" max FCgrav="<<seg[p].max_fc_pred_grav<<" max PRED->OPTgrav="<<seg[p].max_pred_opt_grav<<" max dVopt="<<seg[p].max_dv<<"\n";}
    std::cout<<"CSV: "<<kCsv21<<"\nOpen CSV:\n  code "<<kCsv21<<"\n"; return aborted?2:0;
  }catch(const std::exception&e){if(pipe)pipe->shutdown();if(pipeline_started&&pipe_thread.joinable())pipe_thread.join();if(streaming&&cfd>=0){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);}for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);cv::destroyAllWindows();std::cerr<<"[FATAL] "<<e.what()<<"\n";return 1;}
}
