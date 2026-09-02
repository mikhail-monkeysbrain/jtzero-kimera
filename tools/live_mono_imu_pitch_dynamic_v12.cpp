// JT-ZERO v12: dynamic pitch slow/fast diagnostic.
// Sequence after SPACE: slow 0->+10->0, then fast 0->+10->0.
// Uses the same sample-aligned ATTITUDE interpolation and HIGHRES_IMU integration as v10.

#define JTZERO_V10_NO_MAIN
#include "live_mono_imu_integrator_v10.cpp"
#undef JTZERO_V10_NO_MAIN

namespace jtzero_v12 {
using namespace jtzero_v10;

constexpr const char* kCsv12 = "/home/vio/jtzero_live_pitch_dynamic_v12.csv";
constexpr const char* kWindow12 = "JT-ZERO: DYNAMIC PITCH v12";
constexpr int kStable12 = 20;
constexpr double kGyroStill12 = 0.08;
constexpr double kAccTol12 = 0.35;
constexpr double kTargetTol12 = 0.8;
constexpr double kRollTol12 = 2.0;
constexpr double kYawTol12 = 3.0;
constexpr double kStartDelta12 = 1.0;

struct Target12 {
  double from_pitch;
  double to_pitch;
  bool slow;
  const char* label;
};

const Target12 kTargets12[] = {
  {0.0, 10.0, true,  "МЕДЛЕННО: PITCH 0 -> +10"},
  {10.0, 0.0, true,  "МЕДЛЕННО: PITCH +10 -> 0"},
  {0.0, 10.0, false, "БЫСТРО: PITCH 0 -> +10"},
  {10.0, 0.0, false, "БЫСТРО: PITCH +10 -> 0"}
};
constexpr int kTargetCount12 = sizeof(kTargets12)/sizeof(kTargets12[0]);

struct Meta12 {
  uint64_t imu_us=0;
  double fc_roll=0,fc_pitch=0,fc_yaw=0;
  int step=-1;
  int moving=0;
};

struct Segment12 {
  int step=0;
  bool slow=false;
  double from_pitch=0,to_pitch=0;
  size_t start_row=0,end_row=0,rows=0;
  double duration=0,avg_pitch_rate=0,max_abs_gyro_y=0;
  double dv_fc_x=0,dv_fc_y=0,dv_fc_xy=0;
  double dv_gyro_x=0,dv_gyro_y=0,dv_gyro_xy=0;
  double max_fc_axy=0,max_gyro_axy=0;
  double rms_fc_axy=0,rms_gyro_axy=0;
  double max_backend_vxy=0,max_pim_vxy=0;
  double end_backend_vxy=0,end_pim_vxy=0;
};

AttSample10 interpAtt12(const AttSample10& a,const AttSample10& b,uint64_t us){
  if(b.us<=a.us)return b;
  double t=(double)(us-a.us)/(double)(b.us-a.us);
  t=std::max(0.0,std::min(1.0,t));
  AttSample10 o;o.us=us;
  o.roll=a.roll+(b.roll-a.roll)*t;
  o.pitch=a.pitch+(b.pitch-a.pitch)*t;
  o.yaw=a.yaw+wrapDeg(b.yaw-a.yaw)*t;
  return o;
}

Segment12 summarizeSegment12(const Integrator10& in,int step,size_t start_row,size_t end_row){
  Segment12 s;s.step=step+1;s.slow=kTargets12[step].slow;s.from_pitch=kTargets12[step].from_pitch;s.to_pitch=kTargets12[step].to_pitch;
  if(in.rows.empty())return s;
  start_row=std::min(start_row,in.rows.size()-1);end_row=std::min(end_row,in.rows.size()-1);if(end_row<start_row)std::swap(start_row,end_row);
  s.start_row=start_row;s.end_row=end_row;s.rows=end_row-start_row+1;
  const auto& a=in.rows[start_row];const auto& b=in.rows[end_row];
  if(b.imu_us>a.imu_us)s.duration=(b.imu_us-a.imu_us)*1e-6;
  if(s.duration>1e-6)s.avg_pitch_rate=std::abs(s.to_pitch-s.from_pitch)/s.duration;
  s.dv_fc_x=b.vfx-a.vfx;s.dv_fc_y=b.vfy-a.vfy;s.dv_fc_xy=std::hypot(s.dv_fc_x,s.dv_fc_y);
  s.dv_gyro_x=b.vgx-a.vgx;s.dv_gyro_y=b.vgy-a.vgy;s.dv_gyro_xy=std::hypot(s.dv_gyro_x,s.dv_gyro_y);
  double qf=0,qg=0;
  for(size_t i=start_row;i<=end_row;++i){const auto&r=in.rows[i];const double af=std::hypot(r.afx,r.afy),ag=std::hypot(r.agx,r.agy);s.max_fc_axy=std::max(s.max_fc_axy,af);s.max_gyro_axy=std::max(s.max_gyro_axy,ag);qf+=af*af;qg+=ag*ag;s.max_abs_gyro_y=std::max(s.max_abs_gyro_y,std::abs(r.gy));s.max_backend_vxy=std::max(s.max_backend_vxy,r.backend_vxy);s.max_pim_vxy=std::max(s.max_pim_vxy,r.pim_vxy);}
  if(s.rows){s.rms_fc_axy=std::sqrt(qf/(double)s.rows);s.rms_gyro_axy=std::sqrt(qg/(double)s.rows);}
  s.end_backend_vxy=b.backend_vxy;s.end_pim_vxy=b.pim_vxy;return s;
}

void saveCsv12(const Integrator10& in,const std::vector<Meta12>& meta,const std::vector<Segment12>& segs){
  std::ofstream f(kCsv12,std::ios::trunc);f<<std::fixed<<std::setprecision(9);
  f<<"record,step,moving,imu_us,dt,fc_roll_rel,fc_pitch_rel,fc_yaw_rel,ax,ay,az,gx,gy,gz,a_fc_x,a_fc_y,a_fc_z,a_gyro_x,a_gyro_y,a_gyro_z,v_fc_x,v_fc_y,v_fc_z,v_gyro_x,v_gyro_y,v_gyro_z,backend_vxy,pim_vxy,slow,from_pitch,to_pitch,start_row,end_row,rows,duration_s,avg_pitch_rate_deg_s,max_abs_gyro_y,dv_fc_x,dv_fc_y,dv_fc_xy,dv_gyro_x,dv_gyro_y,dv_gyro_xy,max_fc_axy,max_gyro_axy,rms_fc_axy,rms_gyro_axy,max_backend_vxy,max_pim_vxy,end_backend_vxy,end_pim_vxy\n";
  const size_t n=std::min(in.rows.size(),meta.size());
  for(size_t i=0;i<n;++i){const auto&r=in.rows[i];const auto&m=meta[i];f<<"imu,"<<m.step<<','<<m.moving<<','<<r.imu_us<<','<<r.dt<<','<<m.fc_roll<<','<<m.fc_pitch<<','<<m.fc_yaw<<','<<r.ax<<','<<r.ay<<','<<r.az<<','<<r.gx<<','<<r.gy<<','<<r.gz<<','<<r.afx<<','<<r.afy<<','<<r.afz<<','<<r.agx<<','<<r.agy<<','<<r.agz<<','<<r.vfx<<','<<r.vfy<<','<<r.vfz<<','<<r.vgx<<','<<r.vgy<<','<<r.vgz<<','<<r.backend_vxy<<','<<r.pim_vxy<<",0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n";}
  for(const auto&s:segs)f<<"segment,"<<s.step<<",1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,"<<s.end_backend_vxy<<','<<s.end_pim_vxy<<','<<(s.slow?1:0)<<','<<s.from_pitch<<','<<s.to_pitch<<','<<s.start_row<<','<<s.end_row<<','<<s.rows<<','<<s.duration<<','<<s.avg_pitch_rate<<','<<s.max_abs_gyro_y<<','<<s.dv_fc_x<<','<<s.dv_fc_y<<','<<s.dv_fc_xy<<','<<s.dv_gyro_x<<','<<s.dv_gyro_y<<','<<s.dv_gyro_xy<<','<<s.max_fc_axy<<','<<s.max_gyro_axy<<','<<s.rms_fc_axy<<','<<s.rms_gyro_axy<<','<<s.max_backend_vxy<<','<<s.max_pim_vxy<<','<<s.end_backend_vxy<<','<<s.end_pim_vxy<<'\n';
}

void drawHud12(const cv::Mat& gray,const Telemetry& tel,bool ready,bool done,int step,int stable,bool moving,const Integrator10& in,const std::vector<Segment12>& segs,double ref_roll,double ref_pitch,double ref_accum,uint64_t move_start_us){
  const cv::Scalar bg(15,15,15),white(235,235,235),green(80,220,80),red(40,40,245),yellow(0,220,255),muted(150,150,150);
  cv::Mat c(940,1440,CV_8UC3,bg),video;if(gray.empty())video=cv::Mat(kHeight,kWidth,CV_8UC3,cv::Scalar(0));else cv::cvtColor(gray,video,cv::COLOR_GRAY2BGR);cv::resize(video,video,{820,615},0,0,cv::INTER_NEAREST);video.copyTo(c(cv::Rect(20,80,820,615)));
  uiText(c,"JT-ZERO: ДИНАМИЧЕСКИЙ PITCH v12",20,48,.72,white,2);
  double rr=0,pp=0,yy=0,gn=0,an=0;{std::lock_guard<std::mutex>l(tel.mutex);rr=wrapDeg(tel.fc_roll-ref_roll);pp=wrapDeg(tel.fc_pitch-ref_pitch);yy=tel.fc_accum_yaw-ref_accum;gn=std::sqrt(tel.gx*tel.gx+tel.gy*tel.gy+tel.gz*tel.gz);an=std::sqrt(tel.ax*tel.ax+tel.ay*tel.ay+tel.az*tel.az);}
  const double tp=(ready&&!done)?kTargets12[step].to_pitch:0.0;bool target=ready&&!done&&std::abs(pp-tp)<=kTargetTol12&&std::abs(rr)<=kRollTol12&&std::abs(yy)<=kYawTol12;
  if(!ready){uiText(c,"УСТАНОВИТЕ АППАРАТ В НОЛЬ И НЕ ДВИГАЙТЕ",75,285,.72,white,2);uiText(c,"SPACE — ЗАФИКСИРОВАТЬ НОЛЬ И BIAS",135,355,.70,yellow,2);}
  else if(done){uiText(c,"ТЕСТ ЗАВЕРШЁН",250,300,1.05,green,3);uiText(c,"SPACE — ВЫХОД",305,365,.68,white,2);}
  else if(target){uiText(c,"СТОП! НЕ ДВИГАТЬ",215,300,1.05,green,3);}
  else if(kTargets12[step].slow){uiText(c,"ДВИГАЙТЕ ОЧЕНЬ МЕДЛЕННО",150,270,.86,yellow,3);uiText(c,kTargets12[step].label,170,335,.66,white,2);uiText(c,"ЦЕЛЬ: примерно 3–5 секунд на 10°",175,395,.52,muted,1);}
  else {uiText(c,"ДВИГАЙТЕ БЫСТРО, НО ПЛАВНО",125,270,.82,red,3);uiText(c,kTargets12[step].label,170,335,.66,white,2);uiText(c,"ЦЕЛЬ: примерно 0.5–1.5 секунды на 10°",145,395,.52,muted,1);}
  cv::Mat s=c(cv::Rect(860,70,560,850));cv::rectangle(s,{0,0},{559,849},cv::Scalar(45,45,45),1);drawBar(s,"PITCH",pp,-20,20,tp,kTargetTol12,145);drawBar(s,"ROLL",rr,-20,20,0,kRollTol12,230);drawBar(s,"YAW",yy,-30,30,0,kYawTol12,315);
  char b[256];std::snprintf(b,sizeof(b),"Шаг %d/%d",done?kTargetCount12:step+1,kTargetCount12);uiText(s,b,18,380,.48,white,2);std::snprintf(b,sizeof(b),"Стабильность %d/%d",stable,kStable12);uiText(s,b,18,420,.44,white,1);std::snprintf(b,sizeof(b),"|gyro| %.3f rad/s",gn);uiText(s,b,18,460,.44,gn<=kGyroStill12?green:red,1);std::snprintf(b,sizeof(b),"|acc| %.3f m/s²",an);uiText(s,b,18,495,.44,std::abs(an-9.81)<=kAccTol12?green:red,1);
  if(moving&&move_start_us&&!in.rows.empty()&&in.rows.back().imu_us>move_start_us){double d=(in.rows.back().imu_us-move_start_us)*1e-6;std::snprintf(b,sizeof(b),"Время движения: %.2f s",d);uiText(s,b,18,545,.46,yellow,2);}
  if(!in.rows.empty()){const auto&r=in.rows.back();std::snprintf(b,sizeof(b),"FC int Vxy %.3f m/s",std::hypot(r.vfx,r.vfy));uiText(s,b,18,590,.44,green,1);std::snprintf(b,sizeof(b),"GYRO int Vxy %.3f m/s",std::hypot(r.vgx,r.vgy));uiText(s,b,18,625,.44,yellow,1);std::snprintf(b,sizeof(b),"Backend %.3f / PIM %.3f",r.backend_vxy,r.pim_vxy);uiText(s,b,18,660,.42,white,1);}
  if(!segs.empty()){const auto&q=segs.back();std::snprintf(b,sizeof(b),"Последний: %.2fs  %.2f°/s",q.duration,q.avg_pitch_rate);uiText(s,b,18,710,.40,white,1);std::snprintf(b,sizeof(b),"dV FC %.3f / GYRO %.3f",q.dv_fc_xy,q.dv_gyro_xy);uiText(s,b,18,745,.42,white,1);}
  uiText(s,"Сравниваем медленный/быстрый переход",18,795,.36,muted,1);uiText(c,"SPACE — ноль/выход   ESC / Q — прервать",25,910,.42,muted,1);uiText(c,std::string("CSV: ")+kCsv12,720,910,.40,muted,1);cv::imshow(kWindow12,c);
}

} // namespace jtzero_v12

#ifndef JTZERO_V12_NO_MAIN
int main(int argc,char**argv){
  using namespace jtzero_v12;using namespace jtzero_v10;
  google::InitGoogleLogging(argv[0]);FLAGS_visualize=false;FLAGS_viz_type=2;FLAGS_use_lcd=false;FLAGS_log_output=false;FLAGS_extract_planes_from_the_scene=false;
  if(!std::getenv("JTZERO_DIAG_IMU_ONLY")){std::cerr<<"[FATAL] JTZERO_DIAG_IMU_ONLY is not set\n";return 1;}const std::string params=argc>1?argv[1]:"params/JTZeroMonoFLUZeroLever";
  int cfd=-1,sfd=-1;bool streaming=false,pipeline_started=false,aborted=false,imu_req=false,att_req=false;uint8_t sys=0,comp=0;std::vector<CameraBuffer> bufs;std::shared_ptr<Pipeline10> pipe;std::thread pipe_thread;Telemetry tel;
  try{
    VIO::VioParams vp(params);pipe=std::make_shared<Pipeline10>(vp);pipe->installCallback();pipe_thread=std::thread([pipe](){pipe->spin();});pipeline_started=true;sfd=openSerial();mavlink_status_t mst{};mavlink_message_t msg{};std::cout<<"[MAV] waiting for HEARTBEAT...\n";int64_t dl=monotonicNs()+10000000000LL;while(monotonicNs()<dl&&!sys){pollfd p{sfd,POLLIN,0};if(poll(&p,1,100)<=0)continue;uint8_t b[2048];ssize_t n=read(sfd,b,sizeof(b));if(n<=0)continue;for(ssize_t i=0;i<n;++i)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst)&&msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=msg.sysid;comp=msg.compid;break;}}if(!sys)throw std::runtime_error("HEARTBEAT timeout");requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,kImuRateHz);imu_req=true;requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,kAttRateHz);att_req=true;
    cfd=open(kCameraDevice,O_RDWR|O_NONBLOCK);if(cfd==-1)fail("open camera");configureCamera(cfd);bufs=initCameraBuffers(cfd);v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;if(xioctl(cfd,VIDIOC_STREAMON,&type)==-1)fail("STREAMON");streaming=true;discardWarmup(cfd);cv::setNumThreads(1);cv::namedWindow(kWindow12,cv::WINDOW_NORMAL);cv::resizeWindow(kWindow12,1440,940);
    std::vector<TimeSyncSample> sync;ClockMapping mapping;int64_t pending_sync=0,next_sync=monotonicNs(),next_hud=monotonicNs();uint32_t prev_seq=0;int64_t prev_ts=0,last_sel=0;bool have_prev=false;VIO::FrameId fid=0;cv::Mat last_gray(kHeight,kWidth,CV_8UC1,cv::Scalar(0));std::deque<ImuSample10> iq;AttSample10 prev_att{},cur_att{};bool have_att=false;Integrator10 integ;std::vector<Meta12> meta;std::vector<Segment12> segs;bool ready=false,done=false,moving=false;int step=0,stable=0;size_t move_start_row=0;uint64_t move_start_us=0;double ref_roll=0,ref_pitch=0,ref_yaw_abs=0,ref_accum=0;
    std::cout<<"\nJT-ZERO DYNAMIC PITCH v12\nSPACE fixes zero, BA and BG. Sequence: SLOW +10->0, FAST +10->0.\n";
    while(true){const int64_t now=monotonicNs();if(now>=next_sync&&pending_sync==0){pending_sync=now;sendTimesync(sfd,pending_sync,sys,comp);next_sync=now+kTimesyncPeriodNs;}pollfd pf[2]={{cfd,POLLIN,0},{sfd,POLLIN,0}};int rc=poll(pf,2,2);if(rc<0){if(errno==EINTR)continue;fail("poll");}
      if(pf[1].revents&POLLIN){uint8_t b[8192];for(;;){ssize_t n=read(sfd,b,sizeof(b));if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;for(ssize_t i=0;i<n;++i){if(!mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst))continue;const int64_t rx=monotonicNs();if(msg.msgid==MAVLINK_MSG_ID_TIMESYNC){mavlink_timesync_t ts{};mavlink_msg_timesync_decode(&msg,&ts);if(ts.tc1!=0&&pending_sync&&ts.ts1==pending_sync){TimeSyncSample q;q.t0_rpi_ns=pending_sync;q.t1_rpi_ns=rx;q.fc_ns=ts.tc1;q.rtt_ns=rx-pending_sync;q.rpi_mid_ns=pending_sync+q.rtt_ns/2;q.good=q.rtt_ns>0&&nsToMs(q.rtt_ns)<=kMaxTimesyncRttMs;sync.push_back(q);pending_sync=0;mapping=estimateClockMapping(sync);}}
        else if(msg.msgid==MAVLINK_MSG_ID_ATTITUDE){mavlink_attitude_t a{};mavlink_msg_attitude_decode(&msg,&a);AttSample10 na{(uint64_t)a.time_boot_ms*1000ULL,a.roll*180.0/kPi,a.pitch*180.0/kPi,a.yaw*180.0/kPi};{std::lock_guard<std::mutex>l(tel.mutex);if(tel.have_prev_yaw)tel.fc_accum_yaw+=wrapDeg(na.yaw-tel.prev_yaw);tel.prev_yaw=na.yaw;tel.have_prev_yaw=true;tel.fc_roll=na.roll;tel.fc_pitch=na.pitch;tel.fc_yaw=na.yaw;tel.fc_valid=true;}if(have_att){prev_att=cur_att;cur_att=na;while(!iq.empty()&&iq.front().us<=cur_att.us){auto im=iq.front();if(im.us>=prev_att.us){const auto ai=interpAtt12(prev_att,cur_att,im.us);const size_t before=integ.rows.size();processOne(integ,im,interpR(prev_att,cur_att,im.us),*pipe);if(integ.rows.size()>before){Meta12 m;m.imu_us=im.us;m.fc_roll=wrapDeg(ai.roll-ref_roll);m.fc_pitch=wrapDeg(ai.pitch-ref_pitch);m.fc_yaw=wrapDeg(ai.yaw-ref_yaw_abs);m.step=ready&&!done?step:-1;m.moving=moving?1:0;meta.push_back(m);}iq.pop_front();}else iq.pop_front();}}else{cur_att=na;have_att=true;}
          if(ready&&!done){double rr,pp,yy,gn,an;{std::lock_guard<std::mutex>l(tel.mutex);rr=wrapDeg(tel.fc_roll-ref_roll);pp=wrapDeg(tel.fc_pitch-ref_pitch);yy=tel.fc_accum_yaw-ref_accum;gn=std::sqrt(tel.gx*tel.gx+tel.gy*tel.gy+tel.gz*tel.gz);an=std::sqrt(tel.ax*tel.ax+tel.ay*tel.ay+tel.az*tel.az);}const auto&t=kTargets12[step];if(!moving&&std::abs(pp-t.from_pitch)>=kStartDelta12){moving=true;stable=0;move_start_row=integ.rows.empty()?0:integ.rows.size()-1;move_start_us=integ.rows.empty()?0:integ.rows.back().imu_us;std::cout<<"[MOVE] step="<<(step+1)<<" "<<(t.slow?"SLOW":"FAST")<<" "<<t.from_pitch<<" -> "<<t.to_pitch<<" deg\n";}const bool at=std::abs(pp-t.to_pitch)<=kTargetTol12&&std::abs(rr)<=kRollTol12&&std::abs(yy)<=kYawTol12;const bool still=gn<=kGyroStill12&&std::abs(an-9.81)<=kAccTol12;if(moving&&at&&still)++stable;else if(!(moving&&at&&still))stable=0;if(moving&&stable>=kStable12){const size_t end_row=integ.rows.empty()?0:integ.rows.size()-1;auto s=summarizeSegment12(integ,step,move_start_row,end_row);segs.push_back(s);std::cout<<"[SEGMENT] "<<(s.slow?"SLOW":"FAST")<<" "<<s.from_pitch<<"->"<<s.to_pitch<<" duration="<<s.duration<<"s rate="<<s.avg_pitch_rate<<"deg/s dV_FC="<<s.dv_fc_xy<<" dV_GYRO="<<s.dv_gyro_xy<<" maxA_FC="<<s.max_fc_axy<<" maxA_GYRO="<<s.max_gyro_axy<<" maxPIM="<<s.max_pim_vxy<<"\n";moving=false;stable=0;move_start_us=0;++step;if(step>=kTargetCount12){done=true;std::cout<<"[TEST] completed.\n";}}}}
        else if(msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){mavlink_highres_imu_t h{};mavlink_msg_highres_imu_decode(&msg,&h);ImuSample10 im{h.time_usec,h.xacc,-h.yacc,-h.zacc,h.xgyro,-h.ygyro,-h.zgyro};{std::lock_guard<std::mutex>l(tel.mutex);tel.ax=im.ax;tel.ay=im.ay;tel.az=im.az;tel.gx=im.gx;tel.gy=im.gy;tel.gz=im.gz;}iq.push_back(im);while(iq.size()>200)iq.pop_front();if(mapping.valid&&rx-mapping.last_update_ns<=kMappingStaleNs){VIO::ImuAccGyr d;d<<im.ax,im.ay,im.az,im.gx,im.gy,im.gz;pipe->fillSingleImuQueue(VIO::ImuMeasurement(mapping.map((int64_t)h.time_usec*1000LL),d));}}}}}
      if(pending_sync&&monotonicNs()-pending_sync>20000000LL)pending_sync=0;if(pf[0].revents&POLLIN){for(;;){v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;if(xioctl(cfd,VIDIOC_DQBUF,&b)==-1){if(errno==EAGAIN)break;fail("DQBUF");}const int64_t ts=jtzero::timesync::correctCameraTimestampNs(timevalToNs(b.timestamp));bool ok=true;if(have_prev){int64_t dt=ts-prev_ts;ok=b.sequence==prev_seq+1U&&dt>0&&dt<=20000000LL;}prev_seq=b.sequence;prev_ts=ts;have_prev=true;bool due=last_sel==0||ts-last_sel>=30000000LL;if(ok&&due&&mapping.valid){std::vector<unsigned char> jpeg(b.bytesused);std::memcpy(jpeg.data(),bufs[b.index].start,b.bytesused);cv::Mat gray=cv::imdecode(jpeg,cv::IMREAD_GRAYSCALE);if(!gray.empty()){last_gray=gray;pipe->fillLeftFrameQueue(std::make_unique<VIO::Frame>(fid++,ts,vp.camera_params_.at(0),gray.clone()));last_sel=ts;}}if(xioctl(cfd,VIDIOC_QBUF,&b)==-1)fail("QBUF");}}
      if(now>=next_hud){drawHud12(last_gray,tel,ready,done,step,stable,moving,integ,segs,ref_roll,ref_pitch,ref_accum,move_start_us);next_hud=now+kHudPeriodNs;}int key=cv::waitKey(1)&0xff;if(key==27||key=='q'||key=='Q'){if(!done)aborted=true;break;}if(key==' '&&!ready){Backend10 b;bool bv=pipe->latest(&b),fv=false;{std::lock_guard<std::mutex>l(tel.mutex);fv=tel.fc_valid;if(fv){ref_roll=tel.fc_roll;ref_pitch=tel.fc_pitch;ref_yaw_abs=tel.fc_yaw;ref_accum=tel.fc_accum_yaw;}}if(bv&&fv&&have_att){integ.fixed_ba=Eigen::Vector3d(b.bax,b.bay,b.baz);integ.fixed_bg=Eigen::Vector3d(b.bgx,b.bgy,b.bgz);integ.Rgyro=fcRnedFlu(cur_att.roll,cur_att.pitch,cur_att.yaw);integ.vfc.setZero();integ.vgyro.setZero();integ.last_us=0;integ.armed=true;ready=true;std::cout<<"[ZERO] fixed BA ["<<b.bax<<", "<<b.bay<<", "<<b.baz<<"] BG ["<<b.bgx<<", "<<b.bgy<<", "<<b.bgz<<"]\n";}}else if(key==' '&&done)break;}
    std::this_thread::sleep_for(std::chrono::milliseconds(250));pipe->shutdown();if(pipe_thread.joinable())pipe_thread.join();pipeline_started=false;saveCsv12(integ,meta,segs);cv::destroyAllWindows();if(imu_req)requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);if(att_req)requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);if(streaming){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);}for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);
    std::cout<<"\n============================================================\nJT-ZERO DYNAMIC PITCH v12 RESULT\n============================================================\n"<<"aborted: "<<(aborted?"yes":"no")<<"\ncompleted: "<<(done?"yes":"no")<<"\nrows: "<<integ.rows.size()<<"\n";for(const auto&s:segs)std::cout<<(s.slow?"SLOW":"FAST")<<" "<<s.from_pitch<<"->"<<s.to_pitch<<" deg: duration="<<s.duration<<" s, rate="<<s.avg_pitch_rate<<" deg/s, dV_FC="<<s.dv_fc_xy<<" m/s, dV_GYRO="<<s.dv_gyro_xy<<" m/s, maxA_FC="<<s.max_fc_axy<<" m/s2, maxA_GYRO="<<s.max_gyro_axy<<" m/s2, maxPIM="<<s.max_pim_vxy<<" m/s\n";std::cout<<"CSV: "<<kCsv12<<"\nOpen CSV:\n  code "<<kCsv12<<"\n";return aborted?2:0;
  }catch(const std::exception&e){if(pipe)pipe->shutdown();if(pipeline_started&&pipe_thread.joinable())pipe_thread.join();if(streaming&&cfd>=0){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);}for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);cv::destroyAllWindows();std::cerr<<"[FATAL] "<<e.what()<<"\n";return 1;}
}
#endif // JTZERO_V12_NO_MAIN
