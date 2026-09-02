// JT-ZERO repeated 300 mm camera/VIO distance validation v5.
// Six operator-confirmed legs: A->B, B->A repeated three times.
// Russian event-driven HUD. SPACE/ENTER always registers operator action.

#define main jtzero_v2_unused_main
#include "live_mono_imu_500mm_hud_v2.cpp"
#undef main

#include "jtzero_imu_correction.h"

namespace {
constexpr double kTruthM = 0.300;
constexpr int kLegCount = 6;
constexpr const char* kRepeatCsv = "/home/vio/jtzero_live_300mm_repeat_v5.csv";
constexpr const char* kRepeatWindow = "JT-ZERO 300 мм x6 v5";
constexpr int64_t kPreviewPeriodNs = 33333333LL;

struct Mark { VioState s{}; bool valid=false; };
struct Leg { int index=0; std::string direction; Mark a,b; double horizontal=0,dz=0,d3=0,error=0,scale=0; };

void ru(cv::Mat& img,const std::string&s,cv::Point p,int size,cv::Scalar color,int weight=cv::QT_FONT_NORMAL){
  cv::addText(img,s,p,"DejaVu Sans",size,color,weight,cv::QT_STYLE_NORMAL,0);
}

bool getState(const HudPipeline& pipeline,VioState* s){ return pipeline.latest(s); }

void drawRepeatHud(const cv::Mat&gray,const HudPipeline&pipeline,int leg,bool have_start,const std::string&notice,const Leg*last){
  cv::Mat bgr,video; cv::cvtColor(gray,bgr,cv::COLOR_GRAY2BGR); cv::resize(bgr,video,{900,675},0,0,cv::INTER_NEAREST);
  cv::Mat panel(900,380,CV_8UC3,{24,24,24}),canvas(900,1280,CV_8UC3,{8,8,8}); video.copyTo(canvas(cv::Rect(0,105,900,675)));
  ru(canvas,"JT-ZERO — измерение перемещения 300 мм",{28,52},27,{245,245,245},cv::QT_FONT_BOLD);
  ru(canvas,"Эталон — физические метки A и B",{28,84},17,{190,190,190});
  char buf[192]; std::snprintf(buf,sizeof(buf),"ПРОХОД %d / %d",std::min(leg+1,kLegCount),kLegCount); ru(panel,buf,{18,55},25,{245,245,245},cv::QT_FONT_BOLD);
  ru(panel,(leg%2==0)?"A  →  B":"B  →  A",{18,105},30,{0,230,255},cv::QT_FONT_BOLD);
  VioState s; const bool state=getState(pipeline,&s);
  ru(panel,state?"КООРДИНАТЫ: ЕСТЬ":"КООРДИНАТЫ: ЖДЁМ ПЕРВУЮ ОЦЕНКУ",{18,165},state?18:15,state?cv::Scalar(90,220,90):cv::Scalar(0,210,255),cv::QT_FONT_BOLD);
  if(state){std::snprintf(buf,sizeof(buf),"P [%.3f  %.3f  %.3f] м",s.px,s.py,s.pz);ru(panel,buf,{18,205},16,{220,220,220});}
  if(!have_start){ru(panel,"1. Поставьте камеру на текущую метку",{18,285},15,{220,220,220});ru(panel,"2. Нажмите ПРОБЕЛ — зафиксировать СТАРТ",{18,320},14,{220,220,220});}
  else{ru(panel,"Переместите камеру ровно на 300 мм",{18,285},16,{220,220,220});ru(panel,"Остановитесь и нажмите ПРОБЕЛ — КОНЕЦ",{18,320},14,{220,220,220});}
  cv::rectangle(panel,{18,385,344,92},cv::Scalar(55,145,55),-1,cv::LINE_AA);cv::rectangle(panel,{18,385,344,92},cv::Scalar(220,220,220),2,cv::LINE_AA);
  ru(panel,have_start?"ПРОБЕЛ / ENTER: КОНЕЦ":"ПРОБЕЛ / ENTER: СТАРТ",{35,442},18,{255,255,255},cv::QT_FONT_BOLD);
  if(last){std::snprintf(buf,sizeof(buf),"Последний: %.1f мм",last->horizontal*1000);ru(panel,buf,{18,545},20,{245,245,245},cv::QT_FONT_BOLD);std::snprintf(buf,sizeof(buf),"Ошибка: %+.1f мм  (%+.2f%%)",last->error*1000,last->error/kTruthM*100);ru(panel,buf,{18,585},17,{220,220,220});std::snprintf(buf,sizeof(buf),"ΔZ: %+.1f мм",last->dz*1000);ru(panel,buf,{18,620},16,{190,190,190});}
  ru(panel,"Измерение = sqrt(dx² + dy²)",{18,690},15,{185,185,185});ru(panel,"Q / ESC — выход",{18,730},15,{215,215,215});
  if(!notice.empty()){cv::rectangle(panel,{10,790,360,70},cv::Scalar(35,35,125),-1,cv::LINE_AA);ru(panel,notice,{20,835},14,{255,255,255},cv::QT_FONT_BOLD);}
  panel.copyTo(canvas(cv::Rect(900,0,380,900)));cv::imshow(kRepeatWindow,canvas);
}

Leg makeLeg(int idx,const Mark&a,const Mark&b){Leg r;r.index=idx;r.direction=(idx%2==0)?"A->B":"B->A";r.a=a;r.b=b;double dx=b.s.px-a.s.px,dy=b.s.py-a.s.py;r.dz=b.s.pz-a.s.pz;r.horizontal=std::sqrt(dx*dx+dy*dy);r.d3=std::sqrt(r.horizontal*r.horizontal+r.dz*r.dz);r.error=r.horizontal-kTruthM;r.scale=r.horizontal/kTruthM;return r;}
void saveLegs(const std::vector<Leg>&legs){std::ofstream f(kRepeatCsv,std::ios::trunc);f<<"leg,direction,dx_m,dy_m,dz_m,horizontal_m,distance3d_m,error_horizontal_m,scale_horizontal\n"<<std::fixed<<std::setprecision(9);for(const auto&r:legs)f<<r.index+1<<','<<r.direction<<','<<r.b.s.px-r.a.s.px<<','<<r.b.s.py-r.a.s.py<<','<<r.dz<<','<<r.horizontal<<','<<r.d3<<','<<r.error<<','<<r.scale<<'\n';}
void printSummary(const std::vector<Leg>&legs){if(legs.empty())return;double sum=0,sum2=0,minv=1e9,maxv=-1e9,ab=0,ba=0;int nab=0,nba=0;std::cout<<"\n================ 300 MM x6 RESULT ================\n";for(const auto&r:legs){std::cout<<"LEG "<<r.index+1<<' '<<r.direction<<": horizontal "<<std::fixed<<std::setprecision(2)<<r.horizontal*1000<<" mm  error "<<r.error*1000<<" mm  dz "<<r.dz*1000<<" mm  scale "<<std::setprecision(5)<<r.scale<<"\n";sum+=r.horizontal;sum2+=r.horizontal*r.horizontal;minv=std::min(minv,r.horizontal);maxv=std::max(maxv,r.horizontal);if(r.index%2==0){ab+=r.horizontal;++nab;}else{ba+=r.horizontal;++nba;}}double mean=sum/legs.size(),var=std::max(0.0,sum2/legs.size()-mean*mean),sd=std::sqrt(var),scale=mean/kTruthM;std::cout<<std::setprecision(2)<<"MEAN HORIZONTAL: "<<mean*1000<<" mm\nSTD: "<<sd*1000<<" mm\nMIN: "<<minv*1000<<" mm\nMAX: "<<maxv*1000<<" mm\nA->B mean: "<<(nab?ab/nab*1000:0)<<" mm\nB->A mean: "<<(nba?ba/nba*1000:0)<<" mm\n"<<std::setprecision(6)<<"MEAN SCALE measured/true: "<<scale<<"\nCORRECTION true/measured: "<<1.0/scale<<"\nCSV: "<<kRepeatCsv<<"\n";}
}

int main(int argc,char**argv){
  google::InitGoogleLogging(argv[0]);FLAGS_visualize=false;FLAGS_viz_type=2;FLAGS_use_lcd=false;FLAGS_log_output=false;FLAGS_extract_planes_from_the_scene=false;const std::string params=argc>1?argv[1]:"params/JTZeroMonoFLU";
  int camera_fd=-1,serial_fd=-1;bool streaming=false,pipeline_started=false,aborted=false;uint8_t sys=0,comp=0;std::vector<CameraBuffer>buffers;std::shared_ptr<HudPipeline>pipeline;std::thread pipeline_thread;
  try{
    VIO::VioParams vp(params);if(vp.camera_params_.empty())throw std::runtime_error("No camera params loaded");pipeline=std::make_shared<HudPipeline>(vp);pipeline->installBackendCallback();pipeline_thread=std::thread([pipeline](){pipeline->spin();});pipeline_started=true;
    serial_fd=openSerial();mavlink_status_t mst{};mavlink_message_t msg{};std::cout<<"[MAV] waiting for HEARTBEAT...\n";int64_t dl=monotonicNs()+10000000000LL;while(monotonicNs()<dl&&!sys){pollfd p{serial_fd,POLLIN,0};if(poll(&p,1,100)<=0)continue;uint8_t b[2048];ssize_t n=read(serial_fd,b,sizeof(b));if(n<=0)continue;for(ssize_t i=0;i<n;++i)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst)&&msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=msg.sysid;comp=msg.compid;break;}}if(!sys)throw std::runtime_error("HEARTBEAT timeout");requestRate(serial_fd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,kImuRateHz);
    camera_fd=open(kCameraDevice,O_RDWR|O_NONBLOCK);if(camera_fd==-1)fail("open camera");configureCamera(camera_fd);buffers=initCameraBuffers(camera_fd);v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;if(xioctl(camera_fd,VIDIOC_STREAMON,&type)==-1)fail("STREAMON");streaming=true;discardWarmup(camera_fd);cv::setNumThreads(1);cv::namedWindow(kRepeatWindow,cv::WINDOW_NORMAL);cv::resizeWindow(kRepeatWindow,1280,900);
    std::vector<TimeSyncSample>sync;sync.reserve(500);ClockMapping mapping;int64_t pending=0,next_sync=monotonicNs(),last_sel=0,last_preview=0,prev_ts=0;uint32_t prev_seq=0;bool have_prev=false;VIO::FrameId fid=0;cv::Mat gray(kHeight,kWidth,CV_8UC1,cv::Scalar(0));jtzero::ImuCorrection imu_correction;int leg=0;bool have_start=false;Mark start_mark;std::vector<Leg>legs;std::string notice;int64_t notice_until=0;
    while(leg<kLegCount){int64_t now=monotonicNs();if(now>=next_sync&&pending==0){pending=now;sendTimesync(serial_fd,pending,sys,comp);next_sync=now+kTimesyncPeriodNs;}pollfd pf[2]={{camera_fd,POLLIN,0},{serial_fd,POLLIN,0}};int rc=poll(pf,2,2);if(rc<0){if(errno==EINTR)continue;fail("poll");}
      if(pf[1].revents&POLLIN){uint8_t b[8192];for(;;){ssize_t n=read(serial_fd,b,sizeof(b));if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;for(ssize_t i=0;i<n;++i){if(!mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst))continue;int64_t recv=monotonicNs();if(msg.msgid==MAVLINK_MSG_ID_TIMESYNC){mavlink_timesync_t ts{};mavlink_msg_timesync_decode(&msg,&ts);if(ts.tc1!=0&&pending&&ts.ts1==pending){TimeSyncSample s;s.t0_rpi_ns=pending;s.t1_rpi_ns=recv;s.fc_ns=ts.tc1;s.rtt_ns=recv-pending;s.rpi_mid_ns=pending+s.rtt_ns/2;s.good=s.rtt_ns>0&&nsToMs(s.rtt_ns)<=kMaxTimesyncRttMs;sync.push_back(s);pending=0;mapping=estimateClockMapping(sync);}}else if(msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU&&mapping.valid){mavlink_highres_imu_t h{};mavlink_msg_highres_imu_decode(&msg,&h);Eigen::Vector3d acc=jtzero::ImuCorrection::accelFrdToFlu(h.xacc,h.yacc,h.zacc),gyro=jtzero::ImuCorrection::gyroFrdToFlu(h.xgyro,h.ygyro,h.zgyro),w=imu_correction.correctGyro(h.time_usec,acc,gyro);VIO::ImuAccGyr data;data<<acc.x(),acc.y(),acc.z(),w.x(),w.y(),w.z();pipeline->fillSingleImuQueue(VIO::ImuMeasurement(mapping.map((int64_t)h.time_usec*1000LL),data));}}}}
      if(pending&&monotonicNs()-pending>20000000LL)pending=0;
      if(pf[0].revents&POLLIN){for(;;){v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;if(xioctl(camera_fd,VIDIOC_DQBUF,&b)==-1){if(errno==EAGAIN)break;fail("DQBUF");}int64_t corrected=jtzero::timesync::correctCameraTimestampNs(timevalToNs(b.timestamp));bool ok=true;if(have_prev){int64_t dt=corrected-prev_ts;ok=b.sequence==prev_seq+1U&&dt>0&&dt<=20000000LL;}prev_seq=b.sequence;prev_ts=corrected;have_prev=true;bool feed_due=last_sel==0||corrected-last_sel>=30000000LL,preview_due=last_preview==0||corrected-last_preview>=kPreviewPeriodNs;if(preview_due||(ok&&feed_due&&mapping.valid)){std::vector<unsigned char>jpg(b.bytesused);std::memcpy(jpg.data(),buffers[b.index].start,b.bytesused);cv::Mat g=cv::imdecode(jpg,cv::IMREAD_GRAYSCALE);if(!g.empty()){if(preview_due){gray=g;last_preview=corrected;}if(ok&&feed_due&&mapping.valid){pipeline->fillLeftFrameQueue(std::make_unique<VIO::Frame>(fid++,corrected,vp.camera_params_.at(0),g.clone()));last_sel=corrected;}}}if(xioctl(camera_fd,VIDIOC_QBUF,&b)==-1)fail("QBUF");}}
      if(monotonicNs()>notice_until)notice.clear();const Leg*last=legs.empty()?nullptr:&legs.back();drawRepeatHud(gray,*pipeline,leg,have_start,notice,last);int key=cv::waitKeyEx(5);if(key==27||key=='q'||key=='Q'){aborted=true;break;}bool capture=(key==32||key==13||key==10||key==16777220||key==16777221);if(capture){VioState s;if(!getState(*pipeline,&s)){notice="НЕТ КООРДИНАТ: ПОДВИГАЙТЕ КАМЕРУ 2–3 СМ";notice_until=monotonicNs()+2500000000LL;std::cout<<"[HUD] key accepted, waiting for first coordinate estimate\n";continue;}if(!have_start){start_mark.s=s;start_mark.valid=true;have_start=true;notice="СТАРТ ЗАФИКСИРОВАН";notice_until=monotonicNs()+1500000000LL;std::cout<<"LEG "<<leg+1<<" START captured at KF "<<s.keyframe<<"\n";}else{Mark end;end.s=s;end.valid=true;Leg r=makeLeg(leg,start_mark,end);legs.push_back(r);std::cout<<"LEG "<<leg+1<<" "<<r.direction<<" horizontal="<<std::fixed<<std::setprecision(2)<<r.horizontal*1000<<" mm error="<<r.error*1000<<" mm dz="<<r.dz*1000<<" mm\n";++leg;have_start=false;notice="КОНЕЦ ЗАФИКСИРОВАН";notice_until=monotonicNs()+1500000000LL;}}
    }
    pipeline->shutdown();if(pipeline_thread.joinable())pipeline_thread.join();pipeline_started=false;cv::destroyAllWindows();saveLegs(legs);printSummary(legs);requestRate(serial_fd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);if(streaming){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(camera_fd,VIDIOC_STREAMOFF,&t);}for(auto&bf:buffers)if(bf.start&&bf.start!=MAP_FAILED)munmap(bf.start,bf.length);if(camera_fd!=-1)close(camera_fd);if(serial_fd!=-1)close(serial_fd);std::cout<<"RESULT: "<<(!aborted&&legs.size()==kLegCount?"PASS":"FAIL")<<"\n";return !aborted&&legs.size()==kLegCount?0:1;
  }catch(const std::exception&e){std::cerr<<"[FATAL] "<<e.what()<<"\n";if(pipeline)pipeline->shutdown();if(pipeline_started&&pipeline_thread.joinable())pipeline_thread.join();try{cv::destroyAllWindows();}catch(...){}if(streaming&&camera_fd!=-1){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;xioctl(camera_fd,VIDIOC_STREAMOFF,&t);}for(auto&bf:buffers)if(bf.start&&bf.start!=MAP_FAILED)munmap(bf.start,bf.length);if(camera_fd!=-1)close(camera_fd);if(serial_fd!=-1)close(serial_fd);return 1;}
}
