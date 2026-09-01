// JT-ZERO v11: static pitch gravity diagnostic.
// Sequence: 0 -> +10 pitch -> 0 -> -10 pitch -> 0.
// Uses the v10 sample-aligned FC/GYRO integrations and measures mean world acceleration
// only after each target has become stationary.

#define JTZERO_V10_NO_MAIN
#include "live_mono_imu_integrator_v10.cpp"
#undef JTZERO_V10_NO_MAIN

namespace jtzero_v11 {
using namespace jtzero_v10;

constexpr const char* kCsv11 = "/home/vio/jtzero_live_pitch_static_v11.csv";
constexpr const char* kWindow11 = "JT-ZERO: STATIC PITCH v11";
constexpr int kStable11 = 20;
constexpr int kMeanRows11 = 80;  // ~0.4 s at 200 Hz, entirely inside the stable plateau.
constexpr double kGyroStill11 = 0.08;
constexpr double kAccTol11 = 0.35;

struct Target11 { double pitch; const char* name; };
const Target11 kTargets11[] = {
  {+10.0, "+10 PITCH"},
  {  0.0, "0 PITCH"},
  {-10.0, "-10 PITCH"},
  {  0.0, "0 PITCH / FINISH"}
};
constexpr int kTargetCount11 = sizeof(kTargets11)/sizeof(kTargets11[0]);

struct Plateau11 {
  int step=0;
  double target_pitch=0;
  size_t rows=0;
  double fc_ax=0,fc_ay=0,fc_az=0,fc_axy=0;
  double gyro_ax=0,gyro_ay=0,gyro_az=0,gyro_axy=0;
  double fc_vxy=0,gyro_vxy=0,backend_vxy=0,pim_vxy=0;
};

Plateau11 measurePlateau(const Integrator10& in,int step,double target){
  Plateau11 p;p.step=step;p.target_pitch=target;
  if(in.rows.empty())return p;
  const size_t n=std::min<size_t>(kMeanRows11,in.rows.size());
  const size_t begin=in.rows.size()-n;
  for(size_t i=begin;i<in.rows.size();++i){const auto&r=in.rows[i];p.fc_ax+=r.afx;p.fc_ay+=r.afy;p.fc_az+=r.afz;p.gyro_ax+=r.agx;p.gyro_ay+=r.agy;p.gyro_az+=r.agz;}
  p.rows=n;const double d=1.0/(double)n;p.fc_ax*=d;p.fc_ay*=d;p.fc_az*=d;p.gyro_ax*=d;p.gyro_ay*=d;p.gyro_az*=d;
  p.fc_axy=std::hypot(p.fc_ax,p.fc_ay);p.gyro_axy=std::hypot(p.gyro_ax,p.gyro_ay);
  const auto&r=in.rows.back();p.fc_vxy=std::hypot(r.vfx,r.vfy);p.gyro_vxy=std::hypot(r.vgx,r.vgy);p.backend_vxy=r.backend_vxy;p.pim_vxy=r.pim_vxy;return p;
}

void saveCsv11(const Integrator10& in,const std::vector<Plateau11>& ps){
  std::ofstream f(kCsv11,std::ios::trunc);f<<std::fixed<<std::setprecision(9);
  f<<"record,step,target_pitch,imu_us,dt,ax,ay,az,gx,gy,gz,a_fc_x,a_fc_y,a_fc_z,a_gyro_x,a_gyro_y,a_gyro_z,v_fc_x,v_fc_y,v_fc_z,v_gyro_x,v_gyro_y,v_gyro_z,backend_vxy,pim_vxy,mean_rows,mean_fc_ax,mean_fc_ay,mean_fc_az,mean_fc_axy,mean_gyro_ax,mean_gyro_ay,mean_gyro_az,mean_gyro_axy\n";
  for(const auto&r:in.rows)f<<"imu,-1,0,"<<r.imu_us<<','<<r.dt<<','<<r.ax<<','<<r.ay<<','<<r.az<<','<<r.gx<<','<<r.gy<<','<<r.gz<<','<<r.afx<<','<<r.afy<<','<<r.afz<<','<<r.agx<<','<<r.agy<<','<<r.agz<<','<<r.vfx<<','<<r.vfy<<','<<r.vfz<<','<<r.vgx<<','<<r.vgy<<','<<r.vgz<<','<<r.backend_vxy<<','<<r.pim_vxy<<",0,0,0,0,0,0,0,0,0\n";
  for(const auto&p:ps)f<<"plateau,"<<p.step<<','<<p.target_pitch<<",0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,"<<p.backend_vxy<<','<<p.pim_vxy<<','<<p.rows<<','<<p.fc_ax<<','<<p.fc_ay<<','<<p.fc_az<<','<<p.fc_axy<<','<<p.gyro_ax<<','<<p.gyro_ay<<','<<p.gyro_az<<','<<p.gyro_axy<<'\n';
}

void drawHud11(const cv::Mat& gray,const Telemetry& tel,bool ready,bool done,int step,int stable,const Integrator10& in,const std::vector<Plateau11>& ps,double ref_roll,double ref_pitch,double ref_accum){
  const cv::Scalar bg(15,15,15),white(235,235,235),green(80,220,80),red(40,40,245),yellow(0,220,255),muted(150,150,150);
  cv::Mat c(940,1440,CV_8UC3,bg),video;if(gray.empty())video=cv::Mat(kHeight,kWidth,CV_8UC3,cv::Scalar(0));else cv::cvtColor(gray,video,cv::COLOR_GRAY2BGR);cv::resize(video,video,{820,615},0,0,cv::INTER_NEAREST);video.copyTo(c(cv::Rect(20,80,820,615)));
  uiText(c,"JT-ZERO: СТАТИЧЕСКИЙ PITCH v11",20,48,.72,white,2);
  double rr=0,pp=0,yy=0,gn=0,an=0;{std::lock_guard<std::mutex>l(tel.mutex);rr=wrapDeg(tel.fc_roll-ref_roll);pp=wrapDeg(tel.fc_pitch-ref_pitch);yy=tel.fc_accum_yaw-ref_accum;gn=std::sqrt(tel.gx*tel.gx+tel.gy*tel.gy+tel.gz*tel.gz);an=std::sqrt(tel.ax*tel.ax+tel.ay*tel.ay+tel.az*tel.az);}
  const double tp=(ready&&!done)?kTargets11[step].pitch:0.0;bool target=ready&&!done&&std::abs(pp-tp)<=1.0&&std::abs(rr)<=2.0&&std::abs(yy)<=3.0;
  if(!ready){uiText(c,"УСТАНОВИТЕ АППАРАТ В НОЛЬ И НЕ ДВИГАЙТЕ",75,285,.72,white,2);uiText(c,"SPACE — ЗАФИКСИРОВАТЬ НОЛЬ И BIAS",135,355,.70,yellow,2);}
  else if(done){uiText(c,"ТЕСТ ЗАВЕРШЁН",250,300,1.05,green,3);uiText(c,"SPACE — ВЫХОД",305,365,.68,white,2);}
  else if(target){uiText(c,"СТОП! НЕ ДВИГАТЬ",215,300,1.05,green,3);}
  else {char t[160];std::snprintf(t,sizeof(t),"УСТАНОВИТЕ PITCH %.0f°, YAW/ROLL ДЕРЖАТЬ 0°",tp);uiText(c,t,105,315,.64,white,2);}
  cv::Mat s=c(cv::Rect(860,70,560,850));cv::rectangle(s,{0,0},{559,849},cv::Scalar(45,45,45),1);drawBar(s,"PITCH",pp,-25,25,tp,1,145);drawBar(s,"ROLL",rr,-25,25,0,2,230);drawBar(s,"YAW",yy,-45,45,0,3,315);
  char b[256];std::snprintf(b,sizeof(b),"Шаг %d/%d: PITCH %.0f°",done?kTargetCount11:step+1,kTargetCount11,tp);uiText(s,b,18,385,.48,white,2);std::snprintf(b,sizeof(b),"Стабильность %d/%d",stable,kStable11);uiText(s,b,18,425,.44,white,1);std::snprintf(b,sizeof(b),"|gyro| %.3f rad/s",gn);uiText(s,b,18,465,.44,gn<=kGyroStill11?green:red,1);std::snprintf(b,sizeof(b),"|acc| %.3f m/s²",an);uiText(s,b,18,500,.44,std::abs(an-9.81)<=kAccTol11?green:red,1);
  if(!ps.empty()){const auto&p=ps.back();std::snprintf(b,sizeof(b),"Последняя площадка %.0f°",p.target_pitch);uiText(s,b,18,555,.44,white,1);std::snprintf(b,sizeof(b),"FC mean |aXY| %.4f m/s²",p.fc_axy);uiText(s,b,18,595,.46,green,2);std::snprintf(b,sizeof(b),"GYRO mean |aXY| %.4f m/s²",p.gyro_axy);uiText(s,b,18,635,.46,yellow,2);}
  if(!in.rows.empty()){const auto&r=in.rows.back();std::snprintf(b,sizeof(b),"Backend Vxy %.3f / PIM %.3f",r.backend_vxy,r.pim_vxy);uiText(s,b,18,690,.42,white,1);}
  uiText(s,"Сравниваем остаток gravity",18,745,.40,muted,1);uiText(s,"только на неподвижных площадках",18,775,.40,muted,1);uiText(c,"SPACE — ноль/выход   ESC / Q — прервать",25,910,.42,muted,1);uiText(c,std::string("CSV: ")+kCsv11,720,910,.40,muted,1);cv::imshow(kWindow11,c);
}
}

int main(int argc,char**argv){
  using namespace jtzero_v11;using namespace jtzero_v10;
  google::InitGoogleLogging(argv[0]);FLAGS_visualize=false;FLAGS_viz_type=2;FLAGS_use_lcd=false;FLAGS_log_output=false;FLAGS_extract_planes_from_the_scene=false;
  if(!std::getenv("JTZERO_DIAG_IMU_ONLY")){std::cerr<<"[FATAL] JTZERO_DIAG_IMU_ONLY is not set\n";return 1;}const std::string params=argc>1?argv[1]:"params/JTZeroMonoFLUZeroLever";
  int cfd=-1,sfd=-1;bool streaming=false,pipeline_started=false,aborted=false,imu_req=false,att_req=false;uint8_t sys=0,comp=0;std::vector<CameraBuffer> bufs;std::shared_ptr<Pipeline10> pipe;std::thread pipe_thread;Telemetry tel;
  try{
    VIO::VioParams vp(params);pipe=std::make_shared<Pipeline10>(vp);pipe->installCallback();pipe_thread=std::thread([pipe](){pipe->spin();});pipeline_started=true;sfd=openSerial();mavlink_status_t mst{};mavlink_message_t msg{};std::cout<<"[MAV] waiting for HEARTBEAT...\n";int64_t dl=monotonicNs()+10000000000LL;while(monotonicNs()<dl&&!sys){pollfd p{sfd,POLLIN,0};if(poll(&p,1,100)<=0)continue;uint8_t b[2048];ssize_t n=read(sfd,b,sizeof(b));if(n<=0)continue;for(ssize_t i=0;i<n;++i)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst)&&msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=msg.sysid;comp=msg.compid;break;}}if(!sys)throw std::runtime_error("HEARTBEAT timeout");requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,kImuRateHz);imu_req=true;requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,kAttRateHz);att_req=true;
    cfd=open(kCameraDevice,O_RDWR|O_NONBLOCK);if(cfd==-1)fail("open camera");configureCamera(cfd);bufs=initCameraBuffers(cfd);v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;if(xioctl(cfd,VIDIOC_STREAMON,&type)==-1)fail("STREAMON");streaming=true;discardWarmup(cfd);cv::setNumThreads(1);cv::namedWindow(kWindow11,cv::WINDOW_NORMAL);cv::resizeWindow(kWindow11,1440,940);
    std::vector<TimeSyncSample> sync;ClockMapping mapping;int64_t pending_sync=0,next_sync=monotonicNs(),next_hud=monotonicNs();uint32_t prev_seq=0;int64_t prev_ts=0,last_sel=0;bool have_prev=false;VIO::FrameId fid=0;cv::Mat last_gray(kHeight,kWidth,CV_8UC1,cv::Scalar(0));std::deque<ImuSample10> iq;AttSample10 prev_att{},cur_att{};bool have_att=false;Integrator10 integ;std::vector<Plateau11> plateaus;bool ready=false,done=false;int step=0,stable=0;double ref_roll=0,ref_pitch=0,ref_accum=0;
    std::cout<<"\nJT-ZERO STATIC PITCH v11\nSPACE fixes zero, BA and BG. Sequence: +10 -> 0 -> -10 -> 0 pitch.\n";
    while(true){const int64_t now=monotonicNs();if(now>=next_sync&&pending_sync==0){pending_sync=now;sendTimesync(sfd,pending_sync,sys,comp);next_sync=now+kTimesyncPeriodNs;}pollfd pf[2]={{cfd,POLLIN,0},{sfd,POLLIN,0}};int rc=poll(pf,2,2);if(rc<0){if(errno==EINTR)continue;fail("poll");}
      if(pf[1].revents&POLLIN){uint8_t b[8192];for(;;){ssize_t n=read(sfd,b,sizeof(b));if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;for(ssize_t i=0;i<n;++i){if(!mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst))continue;const int64_t rx=monotonicNs();if(msg.msgid==MAVLINK_MSG_ID_TIMESYNC){mavlink_timesync_t ts{};mavlink_msg_timesync_decode(&msg,&ts);if(ts.tc1!=0&&pending_sync&&ts.ts1==pending_sync){TimeSyncSample q;q.t0_rpi_ns=pending_sync;q.t1_rpi_ns=rx;q.fc_ns=ts.tc1;q.rtt_ns=rx-pending_sync;q.rpi_mid_ns=pending_sync+q.rtt_ns/2;q.good=q.rtt_ns>0&&nsToMs(q.rtt_ns)<=kMaxTimesyncRttMs;sync.push_back(q);pending_sync=0;mapping=estimateClockMapping(sync);}}
        else if(msg.msgid==MAVLINK_MSG_ID_ATTITUDE){mavlink_attitude_t a{};mavlink_msg_attitude_decode(&msg,&a);AttSample10 na{(uint64_t)a.time_boot_ms*1000ULL,a.roll*180.0/kPi,a.pitch*180.0/kPi,a.yaw*180.0/kPi};{std::lock_guard<std::mutex>l(tel.mutex);if(tel.have_prev_yaw)tel.fc_accum_yaw+=wrapDeg(na.yaw-tel.prev_yaw);tel.prev_yaw=na.yaw;tel.have_prev_yaw=true;tel.fc_roll=na.roll;tel.fc_pitch=na.pitch;tel.fc_yaw=na.yaw;tel.fc_valid=true;}if(have_att){prev_att=cur_att;cur_att=na;while(!iq.empty()&&iq.front().us<=cur_att.us){auto im=iq.front();if(im.us>=prev_att.us){processOne(integ,im,interpR(prev_att,cur_att,im.us),*pipe);iq.pop_front();}else iq.pop_front();}}else{cur_att=na;have_att=true;}
          if(ready&&!done){double rr,pp,yy,gn,an;{std::lock_guard<std::mutex>l(tel.mutex);rr=wrapDeg(tel.fc_roll-ref_roll);pp=wrapDeg(tel.fc_pitch-ref_pitch);yy=tel.fc_accum_yaw-ref_accum;gn=std::sqrt(tel.gx*tel.gx+tel.gy*tel.gy+tel.gz*tel.gz);an=std::sqrt(tel.ax*tel.ax+tel.ay*tel.ay+tel.az*tel.az);}const double tp=kTargets11[step].pitch;bool ok=std::abs(pp-tp)<=1.0&&std::abs(rr)<=2.0&&std::abs(yy)<=3.0&&gn<=kGyroStill11&&std::abs(an-9.81)<=kAccTol11;if(ok)++stable;else stable=0;if(stable>=kStable11){auto p=measurePlateau(integ,step+1,tp);plateaus.push_back(p);std::cout<<"[PLATEAU] pitch="<<tp<<" FC aXY="<<p.fc_axy<<" GYRO aXY="<<p.gyro_axy<<" FCint Vxy="<<p.fc_vxy<<" GYROint Vxy="<<p.gyro_vxy<<" PIM="<<p.pim_vxy<<"\n";stable=0;++step;if(step>=kTargetCount11){done=true;std::cout<<"[TEST] completed.\n";}}}}
        else if(msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){mavlink_highres_imu_t h{};mavlink_msg_highres_imu_decode(&msg,&h);ImuSample10 im{h.time_usec,h.xacc,-h.yacc,-h.zacc,h.xgyro,-h.ygyro,-h.zgyro};{std::lock_guard<std::mutex>l(tel.mutex);tel.ax=im.ax;tel.ay=im.ay;tel.az=im.az;tel.gx=im.gx;tel.gy=im.gy;tel.gz=im.gz;}iq.push_back(im);while(iq.size()>200)iq.pop_front();if(mapping.valid&&rx-mapping.last_update_ns<=kMappingStaleNs){VIO::ImuAccGyr d;d<<im.ax,im.ay,im.az,im.gx,im.gy,im.gz;pipe->fillSingleImuQueue(VIO::ImuMeasurement(mapping.map((int64_t)h.time_usec*1000LL),d));}}}}}
      if(pending_sync&&monotonicNs()-pending_sync>20000000LL)pending_sync=0;if(pf[0].revents&POLLIN){for(;;){v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;if(xioctl(cfd,VIDIOC_DQBUF,&b)==-1){if(errno==EAGAIN)break;fail("DQBUF");}const int64_t ts=jtzero::timesync::correctCameraTimestampNs(timevalToNs(b.timestamp));bool ok=true;if(have_prev){int64_t dt=ts-prev_ts;ok=b.sequence==prev_seq+1U&&dt>0&&dt<=20000000LL;}prev_seq=b.sequence;prev_ts=ts;have_prev=true;bool due=last_sel==0||ts-last_sel>=30000000LL;if(ok&&due&&mapping.valid){std::vector<unsigned char> jpeg(b.bytesused);std::memcpy(jpeg.data(),bufs[b.index].start,b.bytesused);cv::Mat gray=cv::imdecode(jpeg,cv::IMREAD_GRAYSCALE);if(!gray.empty()){last_gray=gray;pipe->fillLeftFrameQueue(std::make_unique<VIO::Frame>(fid++,ts,vp.camera_params_.at(0),gray.clone()));last_sel=ts;}}if(xioctl(cfd,VIDIOC_QBUF,&b)==-1)fail("QBUF");}}
      if(now>=next_hud){drawHud11(last_gray,tel,ready,done,step,stable,integ,plateaus,ref_roll,ref_pitch,ref_accum);next_hud=now+kHudPeriodNs;}int key=cv::waitKey(1)&0xff;if(key==27||key=='q'||key=='Q'){if(!done)aborted=true;break;}if(key==' '&&!ready){Backend10 b;bool bv=pipe->latest(&b),fv=false;{std::lock_guard<std::mutex>l(tel.mutex);fv=tel.fc_valid;if(fv){ref_roll=tel.fc_roll;ref_pitch=tel.fc_pitch;ref_accum=tel.fc_accum_yaw;}}if(bv&&fv&&have_att){integ.fixed_ba=Eigen::Vector3d(b.bax,b.bay,b.baz);integ.fixed_bg=Eigen::Vector3d(b.bgx,b.bgy,b.bgz);integ.Rgyro=fcRnedFlu(cur_att.roll,cur_att.pitch,cur_att.yaw);integ.vfc.setZero();integ.vgyro.setZero();integ.last_us=0;integ.armed=true;ready=true;std::cout<<"[ZERO] fixed BA ["<<b.bax<<", "<<b.bay<<", "<<b.baz<<"] BG ["<<b.bgx<<", "<<b.bgy<<", "<<b.bgz<<"]\n";}}else if(key==' '&&done)break;}
    std::this_thread::sleep_for(std::chrono::milliseconds(250));pipe->shutdown();if(pipe_thread.joinable())pipe_thread.join();pipeline_started=false;saveCsv11(integ,plateaus);cv::destroyAllWindows();if(imu_req)requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);if(att_req)requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);if(streaming){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);}for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);
    std::cout<<"\n============================================================\nJT-ZERO STATIC PITCH v11 RESULT\n============================================================\n"<<"aborted: "<<(aborted?"yes":"no")<<"\ncompleted: "<<(done?"yes":"no")<<"\nrows: "<<integ.rows.size()<<"\n";for(const auto&p:plateaus)std::cout<<"pitch "<<p.target_pitch<<" deg: FC mean aXY="<<p.fc_axy<<" m/s2, GYRO mean aXY="<<p.gyro_axy<<" m/s2, PIM="<<p.pim_vxy<<" m/s\n";std::cout<<"CSV: "<<kCsv11<<"\nOpen CSV:\n  code "<<kCsv11<<"\n";return aborted?2:0;
  }catch(const std::exception&e){if(pipe)pipe->shutdown();if(pipeline_started&&pipe_thread.joinable())pipe_thread.join();if(streaming&&cfd>=0){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);}for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);cv::destroyAllWindows();std::cerr<<"[FATAL] "<<e.what()<<"\n";return 1;}
}
