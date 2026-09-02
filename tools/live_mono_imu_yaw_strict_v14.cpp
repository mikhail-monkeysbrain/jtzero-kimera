// JT-ZERO v14: strict YAW diagnostic with automatic rejection of contaminated runs.
// Valid segment requires |ROLL| <= 1.5 deg and |PITCH| <= 1.5 deg during the whole motion.
// Sequence after SPACE: +30 -> 0 -> -30 -> 0. Invalid segment is rejected and must be retried.

#define JTZERO_V10_NO_MAIN
#include "live_mono_imu_integrator_v10.cpp"
#undef JTZERO_V10_NO_MAIN

namespace jtzero_v14 {
using namespace jtzero_v10;

constexpr const char* kCsv14 = "/home/vio/jtzero_live_yaw_strict_v14.csv";
constexpr const char* kWindow14 = "JT-ZERO: STRICT YAW v14";
constexpr int kStable14 = 20;
constexpr double kGyroStill14 = 0.08;
constexpr double kAccTol14 = 0.35;
constexpr double kYawTol14 = 1.5;
constexpr double kRpLimit14 = 1.5;
constexpr double kStartDelta14 = 2.0;

struct Target14 { double from_yaw; double to_yaw; const char* label; };
const Target14 kTargets14[] = {
  {  0.0, +30.0, "YAW 0 -> +30"},
  {+30.0,   0.0, "YAW +30 -> 0"},
  {  0.0, -30.0, "YAW 0 -> -30"},
  {-30.0,   0.0, "YAW -30 -> 0"}
};
constexpr int kTargetCount14 = sizeof(kTargets14)/sizeof(kTargets14[0]);

struct Meta14 {
  uint64_t imu_us=0;
  double roll=0,pitch=0,yaw=0;
  int step=-1;
  int moving=0;
  int rejected=0;
};

struct Segment14 {
  int step=0;
  double from_yaw=0,to_yaw=0;
  size_t start_row=0,end_row=0,rows=0;
  double duration=0,avg_yaw_rate=0;
  double max_abs_roll=0,max_abs_pitch=0,max_abs_gyro_z=0;
  double dv_fc_x=0,dv_fc_y=0,dv_fc_xy=0;
  double dv_gyro_x=0,dv_gyro_y=0,dv_gyro_xy=0;
  double mean_fc_ax=0,mean_fc_ay=0,mean_fc_axy=0;
  double rms_fc_axy=0,max_fc_axy=0;
  double rms_gyro_axy=0,max_gyro_axy=0;
  double max_backend_vxy=0,max_pim_vxy=0;
  double end_backend_vxy=0,end_pim_vxy=0;
};

Segment14 summarize14(const Integrator10& in,const std::vector<Meta14>& meta,int step,size_t start_row,size_t end_row){
  Segment14 s;s.step=step+1;s.from_yaw=kTargets14[step].from_yaw;s.to_yaw=kTargets14[step].to_yaw;
  if(in.rows.empty())return s;
  start_row=std::min(start_row,in.rows.size()-1);end_row=std::min(end_row,in.rows.size()-1);if(end_row<start_row)std::swap(start_row,end_row);
  s.start_row=start_row;s.end_row=end_row;s.rows=end_row-start_row+1;
  const auto&a=in.rows[start_row];const auto&b=in.rows[end_row];
  if(b.imu_us>a.imu_us)s.duration=(b.imu_us-a.imu_us)*1e-6;
  if(s.duration>1e-9)s.avg_yaw_rate=std::abs(s.to_yaw-s.from_yaw)/s.duration;
  s.dv_fc_x=b.vfx-a.vfx;s.dv_fc_y=b.vfy-a.vfy;s.dv_fc_xy=std::hypot(s.dv_fc_x,s.dv_fc_y);
  s.dv_gyro_x=b.vgx-a.vgx;s.dv_gyro_y=b.vgy-a.vgy;s.dv_gyro_xy=std::hypot(s.dv_gyro_x,s.dv_gyro_y);
  double qf=0,qg=0;
  const size_t nmeta=meta.size();
  for(size_t i=start_row;i<=end_row;++i){
    const auto&r=in.rows[i];const double af=std::hypot(r.afx,r.afy),ag=std::hypot(r.agx,r.agy);
    s.mean_fc_ax+=r.afx;s.mean_fc_ay+=r.afy;qf+=af*af;qg+=ag*ag;
    s.max_fc_axy=std::max(s.max_fc_axy,af);s.max_gyro_axy=std::max(s.max_gyro_axy,ag);
    s.max_abs_gyro_z=std::max(s.max_abs_gyro_z,std::abs(r.gz));
    s.max_backend_vxy=std::max(s.max_backend_vxy,r.backend_vxy);s.max_pim_vxy=std::max(s.max_pim_vxy,r.pim_vxy);
    if(i<nmeta){s.max_abs_roll=std::max(s.max_abs_roll,std::abs(meta[i].roll));s.max_abs_pitch=std::max(s.max_abs_pitch,std::abs(meta[i].pitch));}
  }
  if(s.rows){double d=1.0/(double)s.rows;s.mean_fc_ax*=d;s.mean_fc_ay*=d;s.mean_fc_axy=std::hypot(s.mean_fc_ax,s.mean_fc_ay);s.rms_fc_axy=std::sqrt(qf*d);s.rms_gyro_axy=std::sqrt(qg*d);}
  s.end_backend_vxy=b.backend_vxy;s.end_pim_vxy=b.pim_vxy;
  return s;
}

void saveCsv14(const Integrator10& in,const std::vector<Meta14>& meta,const std::vector<Segment14>& segs,int rejected_count){
  std::ofstream f(kCsv14,std::ios::trunc);f<<std::fixed<<std::setprecision(9);
  f<<"record,step,moving,rejected,imu_us,dt,fc_roll_rel,fc_pitch_rel,fc_yaw_rel,ax,ay,az,gx,gy,gz,a_fc_x,a_fc_y,a_fc_z,a_gyro_x,a_gyro_y,a_gyro_z,v_fc_x,v_fc_y,v_fc_z,v_gyro_x,v_gyro_y,v_gyro_z,backend_vxy,pim_vxy,from_yaw,to_yaw,start_row,end_row,rows,duration_s,avg_yaw_rate_deg_s,max_abs_roll,max_abs_pitch,max_abs_gyro_z,dv_fc_x,dv_fc_y,dv_fc_xy,dv_gyro_x,dv_gyro_y,dv_gyro_xy,mean_fc_ax,mean_fc_ay,mean_fc_axy,rms_fc_axy,max_fc_axy,rms_gyro_axy,max_gyro_axy,max_backend_vxy,max_pim_vxy,end_backend_vxy,end_pim_vxy,rejected_count\n";
  const size_t n=std::min(in.rows.size(),meta.size());
  for(size_t i=0;i<n;++i){const auto&r=in.rows[i];const auto&m=meta[i];f<<"imu,"<<m.step<<','<<m.moving<<','<<m.rejected<<','<<r.imu_us<<','<<r.dt<<','<<m.roll<<','<<m.pitch<<','<<m.yaw<<','<<r.ax<<','<<r.ay<<','<<r.az<<','<<r.gx<<','<<r.gy<<','<<r.gz<<','<<r.afx<<','<<r.afy<<','<<r.afz<<','<<r.agx<<','<<r.agy<<','<<r.agz<<','<<r.vfx<<','<<r.vfy<<','<<r.vfz<<','<<r.vgx<<','<<r.vgy<<','<<r.vgz<<','<<r.backend_vxy<<','<<r.pim_vxy<<",0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,"<<rejected_count<<'\n';}
  for(const auto&s:segs)f<<"segment,"<<s.step<<",1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,"<<s.end_backend_vxy<<','<<s.end_pim_vxy<<','<<s.from_yaw<<','<<s.to_yaw<<','<<s.start_row<<','<<s.end_row<<','<<s.rows<<','<<s.duration<<','<<s.avg_yaw_rate<<','<<s.max_abs_roll<<','<<s.max_abs_pitch<<','<<s.max_abs_gyro_z<<','<<s.dv_fc_x<<','<<s.dv_fc_y<<','<<s.dv_fc_xy<<','<<s.dv_gyro_x<<','<<s.dv_gyro_y<<','<<s.dv_gyro_xy<<','<<s.mean_fc_ax<<','<<s.mean_fc_ay<<','<<s.mean_fc_axy<<','<<s.rms_fc_axy<<','<<s.max_fc_axy<<','<<s.rms_gyro_axy<<','<<s.max_gyro_axy<<','<<s.max_backend_vxy<<','<<s.max_pim_vxy<<','<<s.end_backend_vxy<<','<<s.end_pim_vxy<<','<<rejected_count<<'\n';
}

void drawHud14(const cv::Mat& gray,const Telemetry& tel,bool ready,bool done,int step,int stable,bool moving,bool rejected,int rejected_count,const Integrator10& in,const std::vector<Segment14>& segs,double ref_roll,double ref_pitch,double ref_accum){
  const cv::Scalar bg(15,15,15),white(235,235,235),green(80,220,80),red(40,40,245),yellow(0,220,255),muted(150,150,150);
  cv::Mat c(940,1440,CV_8UC3,bg),video;if(gray.empty())video=cv::Mat(kHeight,kWidth,CV_8UC3,cv::Scalar(0));else cv::cvtColor(gray,video,cv::COLOR_GRAY2BGR);cv::resize(video,video,{820,615},0,0,cv::INTER_NEAREST);video.copyTo(c(cv::Rect(20,80,820,615)));
  uiText(c,"JT-ZERO: СТРОГИЙ YAW v14",20,48,.72,white,2);
  double rr=0,pp=0,yy=0,gn=0,an=0;{std::lock_guard<std::mutex>l(tel.mutex);rr=wrapDeg(tel.fc_roll-ref_roll);pp=wrapDeg(tel.fc_pitch-ref_pitch);yy=tel.fc_accum_yaw-ref_accum;gn=std::sqrt(tel.gx*tel.gx+tel.gy*tel.gy+tel.gz*tel.gz);an=std::sqrt(tel.ax*tel.ax+tel.ay*tel.ay+tel.az*tel.az);}
  const double ty=(ready&&!done)?kTargets14[step].to_yaw:0.0;
  if(!ready){uiText(c,"УСТАНОВИТЕ АППАРАТ В НОЛЬ И НЕ ДВИГАЙТЕ",75,285,.72,white,2);uiText(c,"SPACE — ЗАФИКСИРОВАТЬ НОЛЬ И BIAS",135,355,.70,yellow,2);}
  else if(done){uiText(c,"ТЕСТ ЗАВЕРШЁН",250,300,1.05,green,3);uiText(c,"SPACE — ВЫХОД",305,365,.68,white,2);}
  else if(rejected){uiText(c,"НЕВАЛИДНО — ВЕРНИТЕСЬ В НАЧАЛО",85,270,.86,red,3);char t[160];std::snprintf(t,sizeof(t),"Верните YAW к %.0f°, ROLL/PITCH к 0°",kTargets14[step].from_yaw);uiText(c,t,150,340,.62,white,2);}
  else if(moving){uiText(c,"ПОВОРАЧИВАЙТЕ ТОЛЬКО ПО YAW",145,275,.82,yellow,3);uiText(c,kTargets14[step].label,260,345,.66,white,2);uiText(c,"ROLL/PITCH должны оставаться в пределах ±1.5°",105,405,.50,muted,1);}
  else {char t[180];std::snprintf(t,sizeof(t),"ГОТОВО К ШАГУ: %s",kTargets14[step].label);uiText(c,t,155,300,.66,white,2);uiText(c,"Начинайте плавный поворот",245,365,.60,yellow,2);}
  cv::Mat s=c(cv::Rect(860,70,560,850));cv::rectangle(s,{0,0},{559,849},cv::Scalar(45,45,45),1);drawBar(s,"YAW",yy,-40,40,ty,kYawTol14,145);drawBar(s,"PITCH",pp,-6,6,0,kRpLimit14,230);drawBar(s,"ROLL",rr,-6,6,0,kRpLimit14,315);
  char b[256];std::snprintf(b,sizeof(b),"Шаг %d/%d",done?kTargetCount14:step+1,kTargetCount14);uiText(s,b,18,380,.48,white,2);std::snprintf(b,sizeof(b),"Отклонено проходов: %d",rejected_count);uiText(s,b,18,420,.44,rejected_count?yellow:white,1);std::snprintf(b,sizeof(b),"|gyro| %.3f rad/s",gn);uiText(s,b,18,460,.44,gn<=kGyroStill14?green:red,1);std::snprintf(b,sizeof(b),"|acc| %.3f m/s²",an);uiText(s,b,18,495,.44,std::abs(an-9.81)<=kAccTol14?green:red,1);
  std::snprintf(b,sizeof(b),"ROLL %.2f° / PITCH %.2f°",rr,pp);uiText(s,b,18,545,.46,(std::abs(rr)<=kRpLimit14&&std::abs(pp)<=kRpLimit14)?green:red,2);
  if(!in.rows.empty()){const auto&r=in.rows.back();std::snprintf(b,sizeof(b),"FC int Vxy %.3f m/s",std::hypot(r.vfx,r.vfy));uiText(s,b,18,595,.44,green,1);std::snprintf(b,sizeof(b),"GYRO int Vxy %.3f m/s",std::hypot(r.vgx,r.vgy));uiText(s,b,18,630,.44,yellow,1);std::snprintf(b,sizeof(b),"Backend %.3f / PIM %.3f",r.backend_vxy,r.pim_vxy);uiText(s,b,18,665,.42,white,1);}
  if(!segs.empty()){const auto&q=segs.back();std::snprintf(b,sizeof(b),"Последний ΔV FC %.3f / PIM max %.3f",q.dv_fc_xy,q.max_pim_vxy);uiText(s,b,18,720,.40,white,1);std::snprintf(b,sizeof(b),"max RP %.2f° / %.2f°",q.max_abs_roll,q.max_abs_pitch);uiText(s,b,18,755,.40,white,1);}
  uiText(s,"Невалидный проход не засчитывается",18,805,.36,muted,1);uiText(c,"SPACE — ноль/выход   ESC / Q — прервать",25,910,.42,muted,1);uiText(c,std::string("CSV: ")+kCsv14,720,910,.40,muted,1);cv::imshow(kWindow14,c);
}

} // namespace jtzero_v14

int main(int argc,char**argv){
  using namespace jtzero_v14;using namespace jtzero_v10;
  google::InitGoogleLogging(argv[0]);FLAGS_visualize=false;FLAGS_viz_type=2;FLAGS_use_lcd=false;FLAGS_log_output=false;FLAGS_extract_planes_from_the_scene=false;
  if(!std::getenv("JTZERO_DIAG_IMU_ONLY")){std::cerr<<"[FATAL] JTZERO_DIAG_IMU_ONLY is not set\n";return 1;}
  const std::string params=argc>1?argv[1]:"params/JTZeroMonoFLUZeroLever";
  int cfd=-1,sfd=-1;bool streaming=false,pipeline_started=false,aborted=false,imu_req=false,att_req=false;uint8_t sys=0,comp=0;std::vector<CameraBuffer> bufs;std::shared_ptr<Pipeline10> pipe;std::thread pipe_thread;Telemetry tel;
  try{
    VIO::VioParams vp(params);pipe=std::make_shared<Pipeline10>(vp);pipe->installCallback();pipe_thread=std::thread([pipe](){pipe->spin();});pipeline_started=true;
    sfd=openSerial();mavlink_status_t mst{};mavlink_message_t msg{};std::cout<<"[MAV] waiting for HEARTBEAT...\n";int64_t dl=monotonicNs()+10000000000LL;while(monotonicNs()<dl&&!sys){pollfd p{sfd,POLLIN,0};if(poll(&p,1,100)<=0)continue;uint8_t b[2048];ssize_t n=read(sfd,b,sizeof(b));if(n<=0)continue;for(ssize_t i=0;i<n;++i)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst)&&msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=msg.sysid;comp=msg.compid;break;}}if(!sys)throw std::runtime_error("HEARTBEAT timeout");requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,kImuRateHz);imu_req=true;requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,kAttRateHz);att_req=true;
    cfd=open(kCameraDevice,O_RDWR|O_NONBLOCK);if(cfd==-1)fail("open camera");configureCamera(cfd);bufs=initCameraBuffers(cfd);v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;if(xioctl(cfd,VIDIOC_STREAMON,&type)==-1)fail("STREAMON");streaming=true;discardWarmup(cfd);cv::setNumThreads(1);cv::namedWindow(kWindow14,cv::WINDOW_NORMAL);cv::resizeWindow(kWindow14,1440,940);
    std::vector<TimeSyncSample> sync;ClockMapping mapping;int64_t pending_sync=0,next_sync=monotonicNs(),next_hud=monotonicNs();uint32_t prev_seq=0;int64_t prev_ts=0,last_sel=0;bool have_prev=false;VIO::FrameId fid=0;cv::Mat last_gray(kHeight,kWidth,CV_8UC1,cv::Scalar(0));
    std::deque<ImuSample10> iq;AttSample10 prev_att{},cur_att{};bool have_att=false;Integrator10 integ;std::vector<Meta14> meta;std::vector<Segment14> segs;
    bool ready=false,done=false,moving=false,rejected=false;int step=0,stable=0,rejected_count=0;size_t move_start_row=0;double ref_roll=0,ref_pitch=0,ref_yaw_abs=0,ref_accum=0;
    std::cout<<"\nJT-ZERO STRICT YAW v14\nSPACE fixes zero, BA and BG. Sequence: +30 -> 0 -> -30 -> 0.\n";
    while(true){const int64_t now=monotonicNs();if(now>=next_sync&&pending_sync==0){pending_sync=now;sendTimesync(sfd,pending_sync,sys,comp);next_sync=now+kTimesyncPeriodNs;}pollfd pf[2]={{cfd,POLLIN,0},{sfd,POLLIN,0}};int rc=poll(pf,2,2);if(rc<0){if(errno==EINTR)continue;fail("poll");}
      if(pf[1].revents&POLLIN){uint8_t b[8192];for(;;){ssize_t n=read(sfd,b,sizeof(b));if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;for(ssize_t i=0;i<n;++i){if(!mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst))continue;const int64_t rx=monotonicNs();
        if(msg.msgid==MAVLINK_MSG_ID_TIMESYNC){mavlink_timesync_t ts{};mavlink_msg_timesync_decode(&msg,&ts);if(ts.tc1!=0&&pending_sync&&ts.ts1==pending_sync){TimeSyncSample q;q.t0_rpi_ns=pending_sync;q.t1_rpi_ns=rx;q.fc_ns=ts.tc1;q.rtt_ns=rx-pending_sync;q.rpi_mid_ns=pending_sync+q.rtt_ns/2;q.good=q.rtt_ns>0&&nsToMs(q.rtt_ns)<=kMaxTimesyncRttMs;sync.push_back(q);pending_sync=0;mapping=estimateClockMapping(sync);}}
        else if(msg.msgid==MAVLINK_MSG_ID_ATTITUDE){mavlink_attitude_t a{};mavlink_msg_attitude_decode(&msg,&a);AttSample10 na{(uint64_t)a.time_boot_ms*1000ULL,a.roll*180.0/kPi,a.pitch*180.0/kPi,a.yaw*180.0/kPi};{std::lock_guard<std::mutex>l(tel.mutex);if(tel.have_prev_yaw)tel.fc_accum_yaw+=wrapDeg(na.yaw-tel.prev_yaw);tel.prev_yaw=na.yaw;tel.have_prev_yaw=true;tel.fc_roll=na.roll;tel.fc_pitch=na.pitch;tel.fc_yaw=na.yaw;tel.fc_valid=true;}
          if(have_att){prev_att=cur_att;cur_att=na;while(!iq.empty()&&iq.front().us<=cur_att.us){auto im=iq.front();if(im.us>=prev_att.us){double t=(cur_att.us>prev_att.us)?(double)(im.us-prev_att.us)/(double)(cur_att.us-prev_att.us):1.0;t=std::max(0.0,std::min(1.0,t));AttSample10 ai;ai.us=im.us;ai.roll=prev_att.roll+(cur_att.roll-prev_att.roll)*t;ai.pitch=prev_att.pitch+(cur_att.pitch-prev_att.pitch)*t;ai.yaw=prev_att.yaw+wrapDeg(cur_att.yaw-prev_att.yaw)*t;const size_t before=integ.rows.size();processOne(integ,im,interpR(prev_att,cur_att,im.us),*pipe);if(integ.rows.size()>before){Meta14 m;m.imu_us=im.us;m.roll=wrapDeg(ai.roll-ref_roll);m.pitch=wrapDeg(ai.pitch-ref_pitch);m.yaw=wrapDeg(ai.yaw-ref_yaw_abs);m.step=ready&&!done?step:-1;m.moving=moving?1:0;m.rejected=rejected?1:0;meta.push_back(m);}iq.pop_front();}else iq.pop_front();}}else{cur_att=na;have_att=true;}
          if(ready&&!done){double rr,pp,yy,gn,an;{std::lock_guard<std::mutex>l(tel.mutex);rr=wrapDeg(tel.fc_roll-ref_roll);pp=wrapDeg(tel.fc_pitch-ref_pitch);yy=tel.fc_accum_yaw-ref_accum;gn=std::sqrt(tel.gx*tel.gx+tel.gy*tel.gy+tel.gz*tel.gz);an=std::sqrt(tel.ax*tel.ax+tel.ay*tel.ay+tel.az*tel.az);}const auto&tgt=kTargets14[step];
            if(rejected){bool back=std::abs(yy-tgt.from_yaw)<=kYawTol14&&std::abs(rr)<=kRpLimit14&&std::abs(pp)<=kRpLimit14&&gn<=kGyroStill14&&std::abs(an-9.81)<=kAccTol14;if(back)++stable;else stable=0;if(stable>=kStable14){rejected=false;stable=0;std::cout<<"[RETRY] step="<<(step+1)<<" ready again\n";}}
            else if(!moving){if(std::abs(yy-tgt.from_yaw)>=kStartDelta14){moving=true;stable=0;move_start_row=integ.rows.empty()?0:integ.rows.size()-1;std::cout<<"[MOVE] step="<<(step+1)<<" "<<tgt.from_yaw<<" -> "<<tgt.to_yaw<<" deg\n";}}
            else {if(std::abs(rr)>kRpLimit14||std::abs(pp)>kRpLimit14){moving=false;rejected=true;stable=0;++rejected_count;std::cout<<"[REJECT] step="<<(step+1)<<" roll="<<rr<<" pitch="<<pp<<". Return to start.\n";}
              else {bool at=std::abs(yy-tgt.to_yaw)<=kYawTol14;bool still=gn<=kGyroStill14&&std::abs(an-9.81)<=kAccTol14;if(at&&still)++stable;else stable=0;if(stable>=kStable14){size_t end_row=integ.rows.empty()?0:integ.rows.size()-1;auto s=summarize14(integ,meta,step,move_start_row,end_row);segs.push_back(s);std::cout<<"[SEGMENT] "<<s.from_yaw<<"->"<<s.to_yaw<<" duration="<<s.duration<<"s rate="<<s.avg_yaw_rate<<"deg/s maxRP="<<s.max_abs_roll<<"/"<<s.max_abs_pitch<<" dV_FC="<<s.dv_fc_xy<<" dV_GYRO="<<s.dv_gyro_xy<<" meanA_FC="<<s.mean_fc_axy<<" maxPIM="<<s.max_pim_vxy<<"\n";moving=false;stable=0;++step;if(step>=kTargetCount14){done=true;std::cout<<"[TEST] completed.\n";}}}}
          }}
        else if(msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){mavlink_highres_imu_t h{};mavlink_msg_highres_imu_decode(&msg,&h);ImuSample10 im{h.time_usec,h.xacc,-h.yacc,-h.zacc,h.xgyro,-h.ygyro,-h.zgyro};{std::lock_guard<std::mutex>l(tel.mutex);tel.ax=im.ax;tel.ay=im.ay;tel.az=im.az;tel.gx=im.gx;tel.gy=im.gy;tel.gz=im.gz;}iq.push_back(im);while(iq.size()>200)iq.pop_front();if(mapping.valid&&rx-mapping.last_update_ns<=kMappingStaleNs){VIO::ImuAccGyr d;d<<im.ax,im.ay,im.az,im.gx,im.gy,im.gz;pipe->fillSingleImuQueue(VIO::ImuMeasurement(mapping.map((int64_t)h.time_usec*1000LL),d));}}
      }}}
      if(pending_sync&&monotonicNs()-pending_sync>20000000LL)pending_sync=0;
      if(pf[0].revents&POLLIN){for(;;){v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;if(xioctl(cfd,VIDIOC_DQBUF,&b)==-1){if(errno==EAGAIN)break;fail("DQBUF");}const int64_t ts=jtzero::timesync::correctCameraTimestampNs(timevalToNs(b.timestamp));bool ok=true;if(have_prev){int64_t dt=ts-prev_ts;ok=b.sequence==prev_seq+1U&&dt>0&&dt<=20000000LL;}prev_seq=b.sequence;prev_ts=ts;have_prev=true;bool due=last_sel==0||ts-last_sel>=30000000LL;if(ok&&due&&mapping.valid){std::vector<unsigned char> jpeg(b.bytesused);std::memcpy(jpeg.data(),bufs[b.index].start,b.bytesused);cv::Mat gray=cv::imdecode(jpeg,cv::IMREAD_GRAYSCALE);if(!gray.empty()){last_gray=gray;pipe->fillLeftFrameQueue(std::make_unique<VIO::Frame>(fid++,ts,vp.camera_params_.at(0),gray.clone()));last_sel=ts;}}if(xioctl(cfd,VIDIOC_QBUF,&b)==-1)fail("QBUF");}}
      if(now>=next_hud){drawHud14(last_gray,tel,ready,done,step,stable,moving,rejected,rejected_count,integ,segs,ref_roll,ref_pitch,ref_accum);next_hud=now+kHudPeriodNs;}
      int key=cv::waitKey(1)&0xff;if(key==27||key=='q'||key=='Q'){if(!done)aborted=true;break;}if(key==' '&&!ready){Backend10 b;bool bv=pipe->latest(&b),fv=false;{std::lock_guard<std::mutex>l(tel.mutex);fv=tel.fc_valid;if(fv){ref_roll=tel.fc_roll;ref_pitch=tel.fc_pitch;ref_yaw_abs=tel.fc_yaw;ref_accum=tel.fc_accum_yaw;}}if(bv&&fv&&have_att){integ.fixed_ba=Eigen::Vector3d(b.bax,b.bay,b.baz);integ.fixed_bg=Eigen::Vector3d(b.bgx,b.bgy,b.bgz);integ.Rgyro=fcRnedFlu(cur_att.roll,cur_att.pitch,cur_att.yaw);integ.vfc.setZero();integ.vgyro.setZero();integ.last_us=0;integ.armed=true;ready=true;std::cout<<"[ZERO] fixed BA ["<<b.bax<<", "<<b.bay<<", "<<b.baz<<"] BG ["<<b.bgx<<", "<<b.bgy<<", "<<b.bgz<<"]\n";}}else if(key==' '&&done)break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(250));pipe->shutdown();if(pipe_thread.joinable())pipe_thread.join();pipeline_started=false;saveCsv14(integ,meta,segs,rejected_count);cv::destroyAllWindows();if(imu_req)requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);if(att_req)requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);if(streaming){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);}for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);
    std::cout<<"\n============================================================\nJT-ZERO STRICT YAW v14 RESULT\n============================================================\n"<<"aborted: "<<(aborted?"yes":"no")<<"\ncompleted: "<<(done?"yes":"no")<<"\nrejected runs: "<<rejected_count<<"\nrows: "<<integ.rows.size()<<"\n";for(const auto&s:segs)std::cout<<s.from_yaw<<"->"<<s.to_yaw<<" deg: duration="<<s.duration<<" s, rate="<<s.avg_yaw_rate<<" deg/s, maxRP="<<s.max_abs_roll<<"/"<<s.max_abs_pitch<<" deg, dV_FC="<<s.dv_fc_xy<<" m/s, dV_GYRO="<<s.dv_gyro_xy<<" m/s, meanA_FC="<<s.mean_fc_axy<<" m/s2, maxPIM="<<s.max_pim_vxy<<" m/s\n";std::cout<<"CSV: "<<kCsv14<<"\nOpen CSV:\n  code "<<kCsv14<<"\n";return aborted?2:0;
  }catch(const std::exception&e){if(pipe)pipe->shutdown();if(pipeline_started&&pipe_thread.joinable())pipe_thread.join();if(streaming&&cfd>=0){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);}for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);cv::destroyAllWindows();std::cerr<<"[FATAL] "<<e.what()<<"\n";return 1;}
}
