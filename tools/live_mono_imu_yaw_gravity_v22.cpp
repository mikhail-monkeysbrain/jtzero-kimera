// JT-ZERO v22: короткий YAW +30 -> 0 -> -30 -> 0.
// TRUE IMU-only. Диагностика направления ошибки gravity и ее связи с dV PIM.

#define JTZERO_V20_NO_MAIN
#include "live_mono_imu_optimizer_update_v20.cpp"
#undef JTZERO_V20_NO_MAIN

namespace jtzero_v22 {
using namespace jtzero_v20;
using namespace jtzero_v10;

constexpr const char* kCsv22 = "/home/vio/jtzero_live_yaw_gravity_v22.csv";
constexpr const char* kWindow22 = "JT-ZERO: YAW И GRAVITY v22";
constexpr double kTargets[4] = {30.0, 0.0, -30.0, 0.0};
constexpr const char* kNames[4] = {"+30", "0_after_plus", "-30", "0_final"};
constexpr int kStable = 20;
constexpr double kYawTol22 = 2.0, kGyroStill22 = 0.08, kAccTol22 = 0.35;

struct Row22 {
  int phase=0; int64_t kf=0, backend_ns=0;
  double fc_roll=0,fc_pitch=0,fc_yaw=0,pim_roll=0,pim_pitch=0,pim_yaw=0;
  double err_roll=0,err_pitch=0,grav_err_deg=0;
  double agr_x=0,agr_y=0,agr_z=0;
  double vp_x=0,vp_y=0,vp_z=0,dv_x=0,dv_y=0,dv_z=0;
  double intg_x=0,intg_y=0,intg_z=0,pim_vxy=0;
};

static Eigen::Vector3d rpy(const Eigen::Matrix3d& R){
  Eigen::Vector3d e=R.eulerAngles(2,1,0);
  return {e[2]*180.0/kPi,e[1]*180.0/kPi,e[0]*180.0/kPi};
}
static double wrap180(double x){while(x>180)x-=360;while(x<-180)x+=360;return x;}

struct Log22 { std::vector<Row22> rows; Eigen::Vector3d intg=Eigen::Vector3d::Zero(); bool have=false; int64_t last_kf=-1,last_ns=0; Eigen::Vector3d last_v=Eigen::Vector3d::Zero(); };

static bool process22(Log22& l,int phase,const Eigen::Matrix3d& Rfc,const Pipeline20& pipe,const Log20& base){
  Backend20 b; if(!pipe.latest(&b)||!b.have_prev||!base.armed||b.keyframe==l.last_kf)return false;
  const Eigen::Matrix3d F=base.world_map*Rfc, P=b.Rpred;
  const Eigen::Vector3d ef=rpy(F),ep=rpy(P);
  const Eigen::Vector3d z(0,0,1),g(0,0,-9.81);
  const Eigen::Vector3d fgrav_body=F.transpose()*Eigen::Vector3d(0,0,9.81);
  const Eigen::Vector3d aerr=P*fgrav_body+g;
  Row22 r; r.phase=phase;r.kf=b.keyframe;r.backend_ns=b.timestamp_ns;
  r.fc_roll=ef.x();r.fc_pitch=ef.y();r.fc_yaw=ef.z();r.pim_roll=ep.x();r.pim_pitch=ep.y();r.pim_yaw=ep.z();
  r.err_roll=wrap180(r.pim_roll-r.fc_roll);r.err_pitch=wrap180(r.pim_pitch-r.fc_pitch);
  r.grav_err_deg=std::acos(clamp20((F.transpose()*z).normalized().dot((P.transpose()*z).normalized())))*180.0/kPi;
  r.agr_x=aerr.x();r.agr_y=aerr.y();r.agr_z=aerr.z();r.vp_x=b.Vpred.x();r.vp_y=b.Vpred.y();r.vp_z=b.Vpred.z();r.pim_vxy=std::hypot(r.vp_x,r.vp_y);
  if(l.have){double dt=(b.timestamp_ns-l.last_ns)*1e-9;if(dt>0&&dt<1.0)l.intg+=aerr*dt;r.dv_x=b.Vpred.x()-l.last_v.x();r.dv_y=b.Vpred.y()-l.last_v.y();r.dv_z=b.Vpred.z()-l.last_v.z();}
  r.intg_x=l.intg.x();r.intg_y=l.intg.y();r.intg_z=l.intg.z();l.rows.push_back(r);l.have=true;l.last_kf=b.keyframe;l.last_ns=b.timestamp_ns;l.last_v=b.Vpred;return true;
}

static void save22(const Log22& l){std::ofstream f(kCsv22,std::ios::trunc);f<<std::fixed<<std::setprecision(9);f<<"phase,phase_name,keyframe,backend_ns,fc_roll_deg,fc_pitch_deg,fc_yaw_deg,pim_roll_deg,pim_pitch_deg,pim_yaw_deg,err_roll_deg,err_pitch_deg,gravity_err_deg,agrav_x,agrav_y,agrav_z,vpred_x,vpred_y,vpred_z,dvpred_x,dvpred_y,dvpred_z,int_agrav_x,int_agrav_y,int_agrav_z,pim_vxy\n";for(const auto&r:l.rows)f<<r.phase<<','<<kNames[r.phase]<<','<<r.kf<<','<<r.backend_ns<<','<<r.fc_roll<<','<<r.fc_pitch<<','<<r.fc_yaw<<','<<r.pim_roll<<','<<r.pim_pitch<<','<<r.pim_yaw<<','<<r.err_roll<<','<<r.err_pitch<<','<<r.grav_err_deg<<','<<r.agr_x<<','<<r.agr_y<<','<<r.agr_z<<','<<r.vp_x<<','<<r.vp_y<<','<<r.vp_z<<','<<r.dv_x<<','<<r.dv_y<<','<<r.dv_z<<','<<r.intg_x<<','<<r.intg_y<<','<<r.intg_z<<','<<r.pim_vxy<<'\n';}

static void hud(const cv::Mat& gray,const Telemetry& tel,bool ready,bool done,int phase,int stable,const Log22& log,double ref){
  const cv::Scalar bg(15,15,15),white(235,235,235),green(80,220,80),red(40,40,245),yellow(0,220,255),muted(150,150,150);cv::Mat c(940,1440,CV_8UC3,bg),v;if(gray.empty())v=cv::Mat(kHeight,kWidth,CV_8UC3,cv::Scalar(0));else cv::cvtColor(gray,v,cv::COLOR_GRAY2BGR);cv::resize(v,v,{820,615},0,0,cv::INTER_NEAREST);v.copyTo(c(cv::Rect(20,80,820,615)));uiText(c,"JT-ZERO: YAW / ОШИБКА GRAVITY v22",20,48,.72,white,2);
  double yy=0,gn=0,an=0;{std::lock_guard<std::mutex>q(tel.mutex);yy=tel.fc_accum_yaw-ref;gn=std::sqrt(tel.gx*tel.gx+tel.gy*tel.gy+tel.gz*tel.gz);an=std::sqrt(tel.ax*tel.ax+tel.ay*tel.ay+tel.az*tel.az);}double target=kTargets[std::min(phase,3)];
  if(!ready){uiText(c,"СТЕНД НЕПОДВИЖЕН",225,285,.90,white,2);uiText(c,"SPACE — ЗАФИКСИРОВАТЬ НОЛЬ",190,355,.70,yellow,2);}else if(done)uiText(c,"ТЕСТ ЗАВЕРШЁН",250,300,1.05,green,3);else{char t[128];snprintf(t,sizeof(t),"ПОВЕРНИТЕ YAW К %+.0f° И ОСТАНОВИТЕСЬ",target);uiText(c,t,120,300,.70,yellow,3);}
  cv::Mat s=c(cv::Rect(860,70,560,850));drawBar(s,"YAW",yy,-60,60,target,kYawTol22,145);char b[256];snprintf(b,sizeof(b),"Этап %d/4   стабильность %d/%d",phase+1,stable,kStable);uiText(s,b,18,210,.42,white,1);snprintf(b,sizeof(b),"|gyro| %.3f   |acc| %.3f",gn,an);uiText(s,b,18,250,.42,white,1);
  if(!log.rows.empty()){const auto&r=log.rows.back();snprintf(b,sizeof(b),"Ошибка roll: %+.3f°",r.err_roll);uiText(s,b,18,330,.48,std::abs(r.err_roll)>1?red:green,2);snprintf(b,sizeof(b),"Ошибка pitch: %+.3f°",r.err_pitch);uiText(s,b,18,375,.48,std::abs(r.err_pitch)>1?red:green,2);snprintf(b,sizeof(b),"Ошибка gravity: %.3f°",r.grav_err_deg);uiText(s,b,18,420,.48,r.grav_err_deg>1?red:green,2);snprintf(b,sizeof(b),"aG XY: %+.3f  %+.3f м/с²",r.agr_x,r.agr_y);uiText(s,b,18,475,.44,white,1);snprintf(b,sizeof(b),"PIM Vxy: %.3f м/с",r.pim_vxy);uiText(s,b,18,520,.48,r.pim_vxy>.5?red:green,2);snprintf(b,sizeof(b),"∫aG XY: %+.3f  %+.3f м/с",r.intg_x,r.intg_y);uiText(s,b,18,565,.42,white,1);snprintf(b,sizeof(b),"KF: %lld",(long long)r.kf);uiText(s,b,18,610,.42,muted,1);}
  uiText(s,"Маршрут: +30 → 0 → -30 → 0",18,710,.42,muted,1);uiText(s,"TRUE IMU-only",18,750,.40,muted,1);uiText(c,"ESC / Q — прервать   SPACE — старт/выход",25,910,.42,muted,1);cv::imshow(kWindow22,c);
}
}

int main(int argc,char**argv){using namespace jtzero_v22;using namespace jtzero_v20;using namespace jtzero_v10;google::InitGoogleLogging(argv[0]);FLAGS_no_incremental_pose=true;FLAGS_visualize=false;FLAGS_viz_type=2;FLAGS_use_lcd=false;FLAGS_log_output=false;FLAGS_extract_planes_from_the_scene=false;if(!std::getenv("JTZERO_DIAG_IMU_ONLY")){std::cerr<<"[FATAL] JTZERO_DIAG_IMU_ONLY is not set\n";return 1;}const std::string params=argc>1?argv[1]:"params/JTZeroMonoFLUZeroLever";int cfd=-1,sfd=-1;bool streaming=false,pipeline_started=false,aborted=false,imu_req=false,att_req=false;uint8_t sys=0,comp=0;std::vector<CameraBuffer>bufs;std::shared_ptr<Pipeline20>pipe;std::thread pipe_thread;Telemetry tel;
try{VIO::VioParams vp(params);pipe=std::make_shared<Pipeline20>(vp);pipe->installCallback();pipe_thread=std::thread([pipe](){pipe->spin();});pipeline_started=true;sfd=openSerial();mavlink_status_t mst{};mavlink_message_t msg{};std::cout<<"[MAV] waiting for HEARTBEAT...\n";int64_t dl=monotonicNs()+10000000000LL;while(monotonicNs()<dl&&!sys){pollfd p{sfd,POLLIN,0};if(poll(&p,1,100)<=0)continue;uint8_t b[2048];ssize_t n=read(sfd,b,sizeof(b));if(n<=0)continue;for(ssize_t i=0;i<n;++i)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst)&&msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=msg.sysid;comp=msg.compid;break;}}if(!sys)throw std::runtime_error("HEARTBEAT timeout");requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,kImuRateHz);imu_req=true;requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,kAttRateHz);att_req=true;cfd=open(kCameraDevice,O_RDWR|O_NONBLOCK);if(cfd==-1)fail("open camera");configureCamera(cfd);bufs=initCameraBuffers(cfd);v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;if(xioctl(cfd,VIDIOC_STREAMON,&type)==-1)fail("STREAMON");streaming=true;discardWarmup(cfd);cv::setNumThreads(1);cv::namedWindow(kWindow22,cv::WINDOW_NORMAL);cv::resizeWindow(kWindow22,1440,940);
std::vector<TimeSyncSample>sync;ClockMapping mapping;int64_t pending_sync=0,next_sync=monotonicNs(),next_hud=monotonicNs();uint32_t prev_seq=0;int64_t prev_ts=0,last_sel=0;bool have_prev=false;VIO::FrameId fid=0;cv::Mat last_gray(kHeight,kWidth,CV_8UC1,cv::Scalar(0));std::deque<ImuSample10>iq;AttSample10 prev_att{},cur_att{};bool have_att=false;Log20 base;Log22 log;bool ready=false,done=false;int phase=0,stable=0;double ref=0;std::cout<<"\nJT-ZERO YAW GRAVITY v22\nSPACE fixes zero. Test: +30 -> 0 -> -30 -> 0.\n";
while(true){const int64_t now=monotonicNs();if(now>=next_sync&&pending_sync==0){pending_sync=now;sendTimesync(sfd,pending_sync,sys,comp);next_sync=now+kTimesyncPeriodNs;}pollfd pf[2]={{cfd,POLLIN,0},{sfd,POLLIN,0}};int rc=poll(pf,2,2);if(rc<0){if(errno==EINTR)continue;fail("poll");}if(pf[1].revents&POLLIN){uint8_t b[8192];for(;;){ssize_t n=read(sfd,b,sizeof(b));if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;for(ssize_t i=0;i<n;++i){if(!mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst))continue;const int64_t rx=monotonicNs();if(msg.msgid==MAVLINK_MSG_ID_TIMESYNC){mavlink_timesync_t ts{};mavlink_msg_timesync_decode(&msg,&ts);if(ts.tc1!=0&&pending_sync&&ts.ts1==pending_sync){TimeSyncSample q;q.t0_rpi_ns=pending_sync;q.t1_rpi_ns=rx;q.fc_ns=ts.tc1;q.rtt_ns=rx-pending_sync;q.rpi_mid_ns=pending_sync+q.rtt_ns/2;q.good=q.rtt_ns>0&&nsToMs(q.rtt_ns)<=kMaxTimesyncRttMs;sync.push_back(q);pending_sync=0;mapping=estimateClockMapping(sync);}}
else if(msg.msgid==MAVLINK_MSG_ID_ATTITUDE){mavlink_attitude_t a{};mavlink_msg_attitude_decode(&msg,&a);AttSample10 na{(uint64_t)a.time_boot_ms*1000ULL,a.roll*180.0/kPi,a.pitch*180.0/kPi,a.yaw*180.0/kPi};{std::lock_guard<std::mutex>q(tel.mutex);if(tel.have_prev_yaw)tel.fc_accum_yaw+=wrapDeg(na.yaw-tel.prev_yaw);tel.prev_yaw=na.yaw;tel.have_prev_yaw=true;tel.fc_roll=na.roll;tel.fc_pitch=na.pitch;tel.fc_yaw=na.yaw;tel.fc_valid=true;}if(have_att){prev_att=cur_att;cur_att=na;while(!iq.empty()&&iq.front().us<=cur_att.us){auto im=iq.front();if(im.us>=prev_att.us&&ready)process22(log,phase,interpR(prev_att,cur_att,im.us),*pipe,base);iq.pop_front();}}else{cur_att=na;have_att=true;}if(ready&&!done){double yy,gn,an;{std::lock_guard<std::mutex>q(tel.mutex);yy=tel.fc_accum_yaw-ref;gn=std::sqrt(tel.gx*tel.gx+tel.gy*tel.gy+tel.gz*tel.gz);an=std::sqrt(tel.ax*tel.ax+tel.ay*tel.ay+tel.az*tel.az);}bool ok=std::abs(yy-kTargets[phase])<=kYawTol22&&gn<=kGyroStill22&&std::abs(an-9.81)<=kAccTol22;if(ok)++stable;else stable=0;if(stable>=kStable){std::cout<<"[STEP] "<<kNames[phase]<<" fixed.\n";stable=0;if(++phase>=4){phase=3;done=true;std::cout<<"[TEST] completed.\n";}else std::cout<<"[NEXT] target "<<kTargets[phase]<<" deg.\n";}}}
else if(msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){mavlink_highres_imu_t h{};mavlink_msg_highres_imu_decode(&msg,&h);ImuSample10 im{h.time_usec,h.xacc,-h.yacc,-h.zacc,h.xgyro,-h.ygyro,-h.zgyro};{std::lock_guard<std::mutex>q(tel.mutex);tel.ax=im.ax;tel.ay=im.ay;tel.az=im.az;tel.gx=im.gx;tel.gy=im.gy;tel.gz=im.gz;}iq.push_back(im);while(iq.size()>300)iq.pop_front();if(mapping.valid&&rx-mapping.last_update_ns<=kMappingStaleNs){VIO::ImuAccGyr d;d<<im.ax,im.ay,im.az,im.gx,im.gy,im.gz;pipe->fillSingleImuQueue(VIO::ImuMeasurement(mapping.map((int64_t)h.time_usec*1000LL),d));}}}}}if(pending_sync&&monotonicNs()-pending_sync>20000000LL)pending_sync=0;
if(pf[0].revents&POLLIN){for(;;){v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;if(xioctl(cfd,VIDIOC_DQBUF,&b)==-1){if(errno==EAGAIN)break;fail("DQBUF");}const int64_t ts=jtzero::timesync::correctCameraTimestampNs(timevalToNs(b.timestamp));bool ok=true;if(have_prev){int64_t d=ts-prev_ts;ok=b.sequence==prev_seq+1U&&d>0&&d<=20000000LL;}prev_seq=b.sequence;prev_ts=ts;have_prev=true;bool due=last_sel==0||ts-last_sel>=30000000LL;if(ok&&due&&mapping.valid){std::vector<unsigned char>jpeg(b.bytesused);std::memcpy(jpeg.data(),bufs[b.index].start,b.bytesused);cv::Mat gray=cv::imdecode(jpeg,cv::IMREAD_GRAYSCALE);if(!gray.empty()){last_gray=gray;pipe->fillLeftFrameQueue(std::make_unique<VIO::Frame>(fid++,ts,vp.camera_params_.at(0),gray.clone()));last_sel=ts;}}if(xioctl(cfd,VIDIOC_QBUF,&b)==-1)fail("QBUF");}}
if(now>=next_hud){hud(last_gray,tel,ready,done,phase,stable,log,ref);next_hud=now+kHudPeriodNs;}int key=cv::waitKey(1)&0xff;if(key==27||key=='q'||key=='Q'){if(!done)aborted=true;break;}if(key==' '&&!ready){Backend20 bb;bool bv=pipe->latest(&bb),fv=false;Eigen::Matrix3d Rfc0=Eigen::Matrix3d::Identity();{std::lock_guard<std::mutex>q(tel.mutex);fv=tel.fc_valid;if(fv){ref=tel.fc_accum_yaw;Rfc0=fcRnedFlu(tel.fc_roll,tel.fc_pitch,tel.fc_yaw);}}if(bv&&fv&&have_att){base.world_map=bb.Ropt*Rfc0.transpose();base.armed=true;ready=true;std::cout<<"[ZERO] KF "<<bb.keyframe<<". Target +30 deg.\n";}}else if(key==' '&&done)break;}
std::this_thread::sleep_for(std::chrono::milliseconds(250));pipe->shutdown();if(pipe_thread.joinable())pipe_thread.join();pipeline_started=false;save22(log);cv::destroyAllWindows();if(imu_req)requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);if(att_req)requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);if(streaming){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);}for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);
std::cout<<"\n============================================================\nJT-ZERO YAW GRAVITY v22 RESULT\n============================================================\naborted: "<<(aborted?"yes":"no")<<"\ncompleted: "<<(done?"yes":"no")<<"\nkeyframes logged: "<<log.rows.size()<<"\n";for(int p=0;p<4;++p){bool first=true;Row22 a,z;double mg=0;for(const auto&r:log.rows)if(r.phase==p){if(first){a=r;first=false;}z=r;mg=std::max(mg,r.grav_err_deg);}if(!first)std::cout<<kNames[p]<<": KF "<<a.kf<<" -> "<<z.kf<<" dVxy=["<<(z.vp_x-a.vp_x)<<", "<<(z.vp_y-a.vp_y)<<"] int_aG_xy=["<<(z.intg_x-a.intg_x)<<", "<<(z.intg_y-a.intg_y)<<"] max_gravity_err="<<mg<<" deg\n";}std::cout<<"CSV: "<<kCsv22<<"\nOpen CSV:\n  code "<<kCsv22<<"\n";return aborted?2:0;
}catch(const std::exception&e){if(pipe)pipe->shutdown();if(pipeline_started&&pipe_thread.joinable())pipe_thread.join();if(streaming&&cfd>=0){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);}for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);cv::destroyAllWindows();std::cerr<<"[FATAL] "<<e.what()<<"\n";return 1;}}
