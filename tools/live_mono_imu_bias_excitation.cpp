// JT-ZERO live Mono+IMU bias/excitation diagnostic.
// Phases: 10 s INIT still, 10 s EXCITE small +/-50..100 mm translations, 10 s SETTLE still.
#define main jtzero_standstill_unused_main
#include "live_mono_imu_standstill.cpp"
#undef main

namespace {
constexpr const char* kBiasCsv = "/home/vio/jtzero_live_bias_excitation.csv";
constexpr double kInitSec=10.0, kExciteSec=10.0, kSettleSec=10.0;
struct BiasState {
  int64_t ts=0,kf=0; double px=0,py=0,pz=0,vx=0,vy=0,vz=0;
  double bax=0,bay=0,baz=0,bgx=0,bgy=0,bgz=0;
};
class BiasPipeline final : public VIO::MonoImuPipeline {
 public:
  explicit BiasPipeline(const VIO::VioParams&p):VIO::MonoImuPipeline(p){}
  void install(){registerBackendOutputCallback([this](const std::shared_ptr<VIO::BackendOutput>&out){if(!out)return;const auto&s=out->W_State_Blkf_;const auto p=s.pose_.translation();const auto&v=s.velocity_;const auto ba=s.imu_bias_.accelerometer();const auto bg=s.imu_bias_.gyroscope();BiasState z;z.ts=s.timestamp_;z.kf=out->cur_kf_id_;z.px=p.x();z.py=p.y();z.pz=p.z();z.vx=v.x();z.vy=v.y();z.vz=v.z();z.bax=ba.x();z.bay=ba.y();z.baz=ba.z();z.bgx=bg.x();z.bgy=bg.y();z.bgz=bg.z();{std::lock_guard<std::mutex>l(m_);s_.push_back(z);}if((z.kf%10)==0)std::cout<<std::fixed<<std::setprecision(6)<<"[BIAS] kf="<<z.kf<<" P=["<<z.px<<','<<z.py<<','<<z.pz<<"] V=["<<z.vx<<','<<z.vy<<','<<z.vz<<"] BA=["<<z.bax<<','<<z.bay<<','<<z.baz<<"] BG=["<<z.bgx<<','<<z.bgy<<','<<z.bgz<<"]\n";});}
  std::vector<BiasState> states()const{std::lock_guard<std::mutex>l(m_);return s_;}
 private: mutable std::mutex m_; std::vector<BiasState>s_;
};
void save(const std::vector<BiasState>&s,int64_t t0,int64_t t1,int64_t t2){std::ofstream f(kBiasCsv,std::ios::trunc);f<<"phase,keyframe,timestamp_ns,px,py,pz,vx,vy,vz,bax,bay,baz,bgx,bgy,bgz\n";f<<std::fixed<<std::setprecision(9);for(const auto&z:s){const char*ph=z.ts<t1?"INIT":(z.ts<t2?"EXCITE":"SETTLE");f<<ph<<','<<z.kf<<','<<z.ts<<','<<z.px<<','<<z.py<<','<<z.pz<<','<<z.vx<<','<<z.vy<<','<<z.vz<<','<<z.bax<<','<<z.bay<<','<<z.baz<<','<<z.bgx<<','<<z.bgy<<','<<z.bgz<<'\n';}}
}

int main(int argc,char**argv){
 google::InitGoogleLogging(argv[0]);FLAGS_visualize=false;FLAGS_viz_type=2;FLAGS_use_lcd=false;FLAGS_log_output=false;FLAGS_extract_planes_from_the_scene=false;const std::string params=argc>1?argv[1]:"params/JTZeroMono";
 int cfd=-1,sfd=-1;bool streaming=false,req=false;uint8_t sys=0,comp=0;std::vector<CameraBuffer>bufs;std::shared_ptr<BiasPipeline>pipe;std::future<bool>worker;
 try{
  VIO::VioParams vp(params);if(vp.camera_params_.empty())throw std::runtime_error("No camera params loaded");pipe=std::make_shared<BiasPipeline>(vp);pipe->install();worker=std::async(std::launch::async,[pipe](){return pipe->spin();});
  sfd=openSerial();mavlink_status_t mst{};mavlink_message_t msg{};std::cout<<"[MAV] waiting for HEARTBEAT...\n";int64_t hd=monotonicNs()+10000000000LL;while(monotonicNs()<hd&&!sys){pollfd q{sfd,POLLIN,0};if(poll(&q,1,100)<=0)continue;uint8_t b[2048];ssize_t n=read(sfd,b,sizeof(b));if(n<=0)continue;for(ssize_t i=0;i<n;i++)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst)&&msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=msg.sysid;comp=msg.compid;break;}}if(!sys)throw std::runtime_error("HEARTBEAT timeout");requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,kImuRateHz);req=true;
  cfd=open(kCameraDevice,O_RDWR|O_NONBLOCK);if(cfd==-1)fail("open camera");configureCamera(cfd);bufs=initCameraBuffers(cfd);v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;if(xioctl(cfd,VIDIOC_STREAMON,&type)==-1)fail("STREAMON");streaming=true;discardWarmup(cfd);
  std::vector<TimeSyncSample>sync;ClockMapping mapping;int64_t pending=0,nextsync=monotonicNs();size_t raw=0,rej=0,sel=0,dec=0,irx=0,ifed=0,iskip=0;uint32_t prevseq=0;int64_t prevts=0,lastsel=0;bool haveprev=false;VIO::FrameId fid=0;
  const int64_t t0=monotonicNs(),t1=t0+(int64_t)(kInitSec*1e9),t2=t1+(int64_t)(kExciteSec*1e9),t3=t2+(int64_t)(kSettleSec*1e9);int phase=-1;
  std::cout<<"\nJT-ZERO BIAS + EXCITATION DIAGNOSTIC\n10 s INIT still -> 10 s EXCITE +/-50..100 mm -> 10 s SETTLE still.\nKeep roll/pitch/yaw as small as practical.\n";
  while(monotonicNs()<t3){int64_t now=monotonicNs();int np=now<t1?0:(now<t2?1:2);if(np!=phase){phase=np;if(phase==0)std::cout<<"\n>>>>>>>> INIT: DO NOT MOVE <<<<<<<<\n";if(phase==1)std::cout<<"\n>>>>>>>> EXCITE: small repeated +/-50..100 mm translations <<<<<<<<\n";if(phase==2)std::cout<<"\n>>>>>>>> SETTLE: STOP, DO NOT MOVE <<<<<<<<\n";}
   if(now>=nextsync&&pending==0){pending=now;sendTimesync(sfd,pending,sys,comp);nextsync=now+kTimesyncPeriodNs;}
   pollfd pf[2]={{cfd,POLLIN,0},{sfd,POLLIN,0}};int rc=poll(pf,2,2);if(rc<0){if(errno==EINTR)continue;fail("poll");}
   if(pf[1].revents&POLLIN){uint8_t b[8192];for(;;){ssize_t n=read(sfd,b,sizeof(b));if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;for(ssize_t i=0;i<n;i++)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst)){int64_t rx=monotonicNs();if(msg.msgid==MAVLINK_MSG_ID_TIMESYNC){mavlink_timesync_t ts{};mavlink_msg_timesync_decode(&msg,&ts);if(ts.tc1!=0&&pending!=0&&ts.ts1==pending){TimeSyncSample s;s.t0_rpi_ns=pending;s.t1_rpi_ns=rx;s.fc_ns=ts.tc1;s.rtt_ns=rx-pending;s.rpi_mid_ns=pending+s.rtt_ns/2;s.good=s.rtt_ns>0&&nsToMs(s.rtt_ns)<=kMaxTimesyncRttMs;sync.push_back(s);pending=0;mapping=estimateClockMapping(sync);}}else if(msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){++irx;mavlink_highres_imu_t h{};mavlink_msg_highres_imu_decode(&msg,&h);if(!mapping.valid||rx-mapping.last_update_ns>kMappingStaleNs){++iskip;continue;}VIO::ImuAccGyr d;d<<h.xacc,h.yacc,h.zacc,h.xgyro,h.ygyro,h.zgyro;pipe->fillSingleImuQueue(VIO::ImuMeasurement(mapping.map((int64_t)h.time_usec*1000LL),d));++ifed;}}}}
   if(pending&&monotonicNs()-pending>20000000LL)pending=0;
   if(pf[0].revents&POLLIN){for(;;){v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;if(xioctl(cfd,VIDIOC_DQBUF,&b)==-1){if(errno==EAGAIN)break;fail("DQBUF");}++raw;int64_t ts=jtzero::timesync::correctCameraTimestampNs(timevalToNs(b.timestamp));bool ok=true;if(haveprev){int64_t dt=ts-prevts;ok=b.sequence==prevseq+1U&&dt>0&&dt<=20000000LL;if(!ok)++rej;}prevseq=b.sequence;prevts=ts;haveprev=true;bool due=lastsel==0||ts-lastsel>=30000000LL;if(ok&&due&&mapping.valid){std::vector<unsigned char>jpeg(b.bytesused);std::memcpy(jpeg.data(),bufs[b.index].start,b.bytesused);cv::Mat gray=cv::imdecode(jpeg,cv::IMREAD_GRAYSCALE);if(!gray.empty()){pipe->fillLeftFrameQueue(std::make_unique<VIO::Frame>(fid++,ts,vp.camera_params_.at(0),gray.clone()));lastsel=ts;++sel;++dec;}}if(xioctl(cfd,VIDIOC_QBUF,&b)==-1)fail("QBUF");}}
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(500));pipe->shutdown();worker.get();auto states=pipe->states();save(states,t0,t1,t2);
  if(req){requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);req=false;}if(streaming){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);streaming=false;}for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);
  std::cout<<"\n============================================================\nJT-ZERO BIAS + EXCITATION RESULT\n============================================================\nraw camera frames: "<<raw<<"\nrejected raw pairs: "<<rej<<"\nselected frames: "<<sel<<"\nIMU received: "<<irx<<"\nIMU fed: "<<ifed<<"\nIMU skipped mapping: "<<iskip<<"\nTIMESYNC samples: "<<sync.size()<<"\nmapping valid: "<<(mapping.valid?"yes":"no")<<"\nmapping drift ppm: "<<mapping.drift_ppm<<"\nbackend states: "<<states.size()<<"\nCSV: "<<kBiasCsv<<"\n";if(!states.empty()){const auto&a=states.front(),&b=states.back();std::cout<<std::fixed<<std::setprecision(6)<<"FIRST BA=["<<a.bax<<','<<a.bay<<','<<a.baz<<"] BG=["<<a.bgx<<','<<a.bgy<<','<<a.bgz<<"]\nLAST  BA=["<<b.bax<<','<<b.bay<<','<<b.baz<<"] BG=["<<b.bgx<<','<<b.bgy<<','<<b.bgz<<"]\nLAST P=["<<b.px<<','<<b.py<<','<<b.pz<<"] V=["<<b.vx<<','<<b.vy<<','<<b.vz<<"]\n";}return 0;
 }catch(const std::exception&e){if(pipe)pipe->shutdown();if(sfd>=0&&req&&sys){try{requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);}catch(...){}}if(streaming&&cfd>=0){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(cfd,VIDIOC_STREAMOFF,&t);}for(auto&b:bufs)if(b.start&&b.start!=MAP_FAILED)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);std::cerr<<"[FATAL] "<<e.what()<<"\n";return 1;}
}
