// JT-ZERO Stage 11 final diagnostic recorder v15.29.
// Based on record_mono_imu_yaw_only_gui_v13.cpp and preserves its raw dataset format.
// Protocol after 2 s zero acquire:
// 10 s STILL0
// 15 s YAW_A 0 -> +90
// 5 s STILL90_A
// 15 s YAW_RETURN_A +90 -> 0
// 10 s STILL_HOME
// 20 s MOVE_YAW_500: move exactly 500 mm by mechanical marks while yawing 0 -> +90
// 10 s STILL_500_YAW90
// 15 s YAW_RETURN_500 +90 -> 0
// 10 s STILL_500_HOME
// 20 s MOVE_BACK_500: return exactly 500 mm to start, keep yaw ~0
// 10 s STILL_END
// Outputs are directly compatible with replay_mono_imu_zxy_ab_v11.cpp.

#define main jtzero_camera_imu_logger_unused_main_v1529
#include "camera_imu_extrinsics_logger.cpp"
#undef main

namespace {

constexpr const char* OUT_CSV="/home/vio/jtzero_final_suite_v15_29.csv";
constexpr const char* OUT_CAM="/home/vio/jtzero_final_suite_v15_29_camera.csv";
constexpr const char* OUT_MJPEG="/home/vio/jtzero_final_suite_v15_29.mjpg";
constexpr const char* OUT_ATT="/home/vio/jtzero_final_suite_v15_29_attitude.csv";
constexpr const char* OUT_PHASES="/home/vio/jtzero_final_suite_v15_29_phases.csv";
constexpr const char* WIN="JT-Zero: финальный тест пункта 11";
constexpr double ZERO_SEC=2.0;
constexpr double RP_OK_DEG=4.0;
constexpr double YAW_OK_DEG=8.0;

struct Phase1529 {
  const char* name;
  double duration;
  double target_yaw;
  double expected_xy_mm;
  const char* instruction;
  bool moving;
};

constexpr Phase1529 PHASES[] = {
  {"STILL0",10.0,0.0,0.0,"НЕ ДВИГАТЬ",false},
  {"YAW_A",15.0,90.0,0.0,"ПЛАВНО ПОВЕРНИ YAW ДО +90°",true},
  {"STILL90_A",5.0,90.0,0.0,"СТОП. НЕ ДВИГАТЬ",false},
  {"YAW_RETURN_A",15.0,0.0,0.0,"ПЛАВНО ВЕРНИ YAW В 0°",true},
  {"STILL_HOME",10.0,0.0,0.0,"НЕ ДВИГАТЬ",false},
  {"MOVE_YAW_500",20.0,90.0,500.0,"ПЕРЕМЕСТИ РОВНО 500 ММ И ОДНОВРЕМЕННО YAW ДО +90°",true},
  {"STILL_500_YAW90",10.0,90.0,0.0,"СТОП НА ОТМЕТКЕ 500 ММ. НЕ ДВИГАТЬ",false},
  {"YAW_RETURN_500",15.0,0.0,0.0,"НА ОТМЕТКЕ 500 ММ ВЕРНИ YAW В 0°",true},
  {"STILL_500_HOME",10.0,0.0,0.0,"НЕ ДВИГАТЬ НА ОТМЕТКЕ 500 ММ",false},
  {"MOVE_BACK_500",20.0,0.0,500.0,"ВЕРНИ БПЛА РОВНО 500 ММ НА СТАРТОВУЮ ОТМЕТКУ. YAW ДЕРЖИ 0°",true},
  {"STILL_END",10.0,0.0,0.0,"ФИНАЛ: НЕ ДВИГАТЬ",false},
};
constexpr size_t NPH = sizeof(PHASES)/sizeof(PHASES[0]);

struct Att1529 {int64_t recv_ns=0,src_ns=0;double r=0,p=0,y=0,rs=0,ps=0,ys=0;};

double relDeg1529(double a,double z){return wrap180(a-z);}

double totalSec1529(){double s=0;for(const auto&p:PHASES)s+=p.duration;return s;}

size_t phaseIndex1529(double elapsed,double* local=nullptr,double* remain=nullptr){
  double t=0;
  for(size_t i=0;i<NPH;++i){
    const double e=t+PHASES[i].duration;
    if(elapsed<e){if(local)*local=elapsed-t;if(remain)*remain=e-elapsed;return i;}
    t=e;
  }
  if(local)*local=0;if(remain)*remain=0;return NPH-1;
}

void text1529(cv::Mat&im,const std::string&s,cv::Point p,cv::Scalar c=cv::Scalar(255,255,255),int h=22){
  cv::addText(im,s,p,"DejaVu Sans",h,c,cv::QT_FONT_NORMAL,cv::QT_STYLE_NORMAL,0);
}
int gaugeX1529(double v,double minv,double maxv,int x,int w){double u=(v-minv)/(maxv-minv);u=std::clamp(u,0.0,1.0);return x+(int)std::lround(u*w);}
void gauge1529(cv::Mat&im,int x,int y,int w,const std::string&name,double value,double target,double tol,double minv,double maxv){
  cv::rectangle(im,{x,y,w,28},{70,70,70},cv::FILLED);
  int g0=gaugeX1529(target-tol,minv,maxv,x,w),g1=gaugeX1529(target+tol,minv,maxv,x,w);if(g1<g0)std::swap(g0,g1);
  cv::rectangle(im,{g0,y,std::max(1,g1-g0),28},{0,140,0},cv::FILLED);
  int z=gaugeX1529(0,minv,maxv,x,w),v=gaugeX1529(value,minv,maxv,x,w),t=gaugeX1529(target,minv,maxv,x,w);
  cv::line(im,{z,y-5},{z,y+33},{160,160,160},1);cv::line(im,{t,y-5},{t,y+33},{0,255,255},2);
  bool ok=std::abs(wrap180(value-target))<=tol;cv::line(im,{v,y-8},{v,y+36},ok?cv::Scalar(0,255,0):cv::Scalar(0,0,255),4,cv::LINE_AA);
  std::ostringstream ss;ss<<name<<": "<<std::fixed<<std::setprecision(1)<<value<<"°";text1529(im,ss.str(),{x,y-12});
}

void drawGui1529(cv::Mat&screen,const cv::Mat&gray,double elapsed,double rr,double rp,double ry,bool have_att,bool zero_ready){
  screen.setTo(cv::Scalar(18,18,18));
  if(!gray.empty()){
    cv::Mat bgr;cv::cvtColor(gray,bgr,cv::COLOR_GRAY2BGR);cv::resize(bgr,bgr,{760,570});bgr.copyTo(screen(cv::Rect(20,120,760,570)));
    cv::rectangle(screen,{20,120,760,570},{180,180,180},1);cv::line(screen,{400,120},{400,690},{100,100,100},1);cv::line(screen,{20,405},{780,405},{100,100,100},1);
  }
  text1529(screen,"JT-ZERO — ФИНАЛЬНЫЙ ДИАГНОСТИЧЕСКИЙ ТЕСТ ПУНКТА 11",{30,42},{255,255,255},22);
  if(!zero_ready){
    text1529(screen,"КАЛИБРОВКА НУЛЯ: БПЛА НЕПОДВИЖЕН",{30,82},{0,220,255},22);
  }else{
    double local=0,remain=0;size_t pi=phaseIndex1529(elapsed,&local,&remain);const auto&p=PHASES[pi];
    std::ostringstream ph;ph<<"ФАЗА "<<(pi+1)<<"/"<<NPH<<": "<<p.name;text1529(screen,ph.str(),{30,82},{0,220,255},22);
    text1529(screen,p.instruction,{810,125},p.moving?cv::Scalar(0,220,255):cv::Scalar(0,255,0),20);
    std::ostringstream ts;ts<<"Осталось: "<<std::fixed<<std::setprecision(1)<<std::max(0.0,remain)<<" с";text1529(screen,ts.str(),{850,82});
    gauge1529(screen,840,220,380,"ROLL",rr,0,RP_OK_DEG,-20,20);
    gauge1529(screen,840,320,380,"PITCH",rp,0,RP_OK_DEG,-20,20);
    gauge1529(screen,840,420,380,"YAW",ry,p.target_yaw,YAW_OK_DEG,-120,120);
    bool rp_ok=std::abs(rr)<=RP_OK_DEG&&std::abs(rp)<=RP_OK_DEG;
    text1529(screen,rp_ok?"ROLL/PITCH В ЗЕЛЁНОЙ ЗОНЕ":"ВНИМАНИЕ: НЕ НАКЛОНЯЙ БПЛА",{825,515},rp_ok?cv::Scalar(0,255,0):cv::Scalar(0,0,255));
    if(std::string(p.name)=="MOVE_YAW_500") text1529(screen,"Используй физические метки 0 → 500 мм",{825,555},{255,255,255});
    if(std::string(p.name)=="MOVE_BACK_500") text1529(screen,"Используй физические метки 500 → 0 мм",{825,555},{255,255,255});
    std::ostringstream all;all<<"Общее время: "<<std::fixed<<std::setprecision(1)<<elapsed<<" / "<<totalSec1529()<<" с";text1529(screen,all.str(),{825,600},{180,180,180});
  }
  if(!have_att)text1529(screen,"НЕТ ATTITUDE ОТ FC",{825,640},{0,0,255});
  text1529(screen,"ESC — аварийно завершить тест",{825,680},{190,190,190});
}

void cleanup1529(int sfd,int cfd,bool streaming,std::vector<CameraBuffer>&buf,uint8_t sys,uint8_t comp,bool rates){
  if(rates&&sfd>=0&&sys){try{requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);}catch(...){}}
  if(streaming&&cfd>=0){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;ioctl(cfd,VIDIOC_STREAMOFF,&t);}for(auto&b:buf)if(b.start&&b.length)munmap(b.start,b.length);
  if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);cv::destroyAllWindows();
}

} // namespace

int main(){
  int sfd=-1,cfd=-1;bool streaming=false,rates=false;std::vector<CameraBuffer>buf;uint8_t sys=0,comp=0;
  try{
    sfd=openSerial();std::cout<<"[MAV] waiting for HEARTBEAT...\n";mavlink_status_t ms{};mavlink_message_t mm{};int64_t dl=monotonicNs()+10000000000LL;
    while(monotonicNs()<dl&&!sys){pollfd p{sfd,POLLIN,0};if(poll(&p,1,100)>0){uint8_t b[2048];ssize_t n=read(sfd,b,sizeof(b));for(ssize_t i=0;i<n;++i)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&mm,&ms)&&mm.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=mm.sysid;comp=mm.compid;break;}}}
    if(!sys)throw std::runtime_error("HEARTBEAT timeout");requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,IMU_RATE_HZ);requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,ATTITUDE_RATE_HZ);rates=true;
    cfd=open(CAMERA_DEVICE,O_RDWR|O_NONBLOCK);if(cfd<0)fail("open camera");configureCamera(cfd);buf=initCameraBuffers(cfd);v4l2_buf_type typ=V4L2_BUF_TYPE_VIDEO_CAPTURE;if(xioctl(cfd,VIDIOC_STREAMON,&typ)==-1)fail("STREAMON");streaming=true;discardWarmup(cfd,CAMERA_WARMUP_FRAMES);
    cv::namedWindow(WIN,cv::WINDOW_NORMAL);cv::setWindowProperty(WIN,cv::WND_PROP_FULLSCREEN,cv::WINDOW_FULLSCREEN);cv::Mat screen(720,1280,CV_8UC3),latest_gray;
    std::vector<CameraSample>cams;std::vector<ImuSample>imus;std::vector<TimeSyncSample>syncs;std::vector<Att1529>atts;
    cams.reserve(25000);imus.reserve(40000);syncs.reserve(1000);atts.reserve(10000);
    std::ofstream mj(OUT_MJPEG,std::ios::binary|std::ios::trunc);if(!mj)throw std::runtime_error("Cannot create MJPEG");
    bool have_seq=false;uint32_t prev_seq=0;uint64_t drops=0;int64_t pending=0,next_ts=monotonicNs();bool have_att=false,zero_ready=false;
    double ar=0,ap=0,ay=0,sum_r=0,sum_p=0,sum_sy=0,sum_cy=0;size_t zn=0;int64_t zero_start=0,test_start=0,last_gui=0;uint32_t preview_div=0;
    tcflush(sfd,TCIFLUSH);std::memset(&ms,0,sizeof(ms));std::memset(&mm,0,sizeof(mm));std::cout<<"[GUI] Финальный all-in-one recorder v15.29 запущен.\n";
    bool abort=false;
    while(!abort){
      int64_t now=monotonicNs();double elapsed=zero_ready?(now-test_start)*1e-9:0.0;if(zero_ready&&elapsed>=totalSec1529())break;
      if(now>=next_ts&&pending==0){pending=now;sendTimesync(sfd,pending,sys,comp);next_ts=now+TIMESYNC_PERIOD_NS;}
      pollfd pf[2]={{cfd,POLLIN,0},{sfd,POLLIN,0}};int rc=poll(pf,2,2);if(rc<0){if(errno==EINTR)continue;fail("poll");}
      if(pf[0].revents&POLLIN){for(;;){v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;if(xioctl(cfd,VIDIOC_DQBUF,&b)==-1){if(errno==EAGAIN)break;fail("DQBUF");}int64_t recv=monotonicNs(),v4=timevalToNs(b.timestamp),corr=jtzero::timesync::correctCameraTimestampNs(v4);if(have_seq){uint32_t ex=prev_seq+1;if(b.sequence!=ex)drops+=uint32_t(b.sequence-ex);}prev_seq=b.sequence;have_seq=true;uint64_t off=(uint64_t)mj.tellp();mj.write((const char*)buf[b.index].start,b.bytesused);cams.push_back({recv,v4,corr,b.sequence,b.flags,b.bytesused,off});if((preview_div++%4)==0){std::vector<unsigned char>j((unsigned char*)buf[b.index].start,(unsigned char*)buf[b.index].start+b.bytesused);latest_gray=cv::imdecode(j,cv::IMREAD_GRAYSCALE);}if(xioctl(cfd,VIDIOC_QBUF,&b)==-1)fail("QBUF");}}
      if(pf[1].revents&POLLIN){uint8_t by[8192];for(;;){ssize_t n=read(sfd,by,sizeof(by));if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;for(ssize_t i=0;i<n;++i){if(!mavlink_parse_char(MAVLINK_COMM_0,by[i],&mm,&ms))continue;int64_t recv=monotonicNs();if(mm.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){mavlink_highres_imu_t a{};mavlink_msg_highres_imu_decode(&mm,&a);ImuSample s;s.recv_ns=recv;s.fc_ns=(int64_t)a.time_usec*1000;s.xacc=a.xacc;s.yacc=a.yacc;s.zacc=a.zacc;s.xgyro=a.xgyro;s.ygyro=a.ygyro;s.zgyro=a.zgyro;s.temperature=a.temperature;s.fields_updated=a.fields_updated;
#ifdef MAVLINK_MSG_HIGHRES_IMU_FIELD_ID_LEN
s.imu_id=a.id;
#endif
imus.push_back(s);}else if(mm.msgid==MAVLINK_MSG_ID_ATTITUDE){mavlink_attitude_t a{};mavlink_msg_attitude_decode(&mm,&a);double r=rad2deg(a.roll),p=rad2deg(a.pitch),y=rad2deg(a.yaw);atts.push_back({recv,(int64_t)a.time_boot_ms*1000000LL,r,p,y,a.rollspeed,a.pitchspeed,a.yawspeed});have_att=true;if(!zero_ready){if(zero_start==0)zero_start=recv;sum_r+=r;sum_p+=p;sum_sy+=std::sin(a.yaw);sum_cy+=std::cos(a.yaw);++zn;if((recv-zero_start)*1e-9>=ZERO_SEC&&zn>20){ar=sum_r/zn;ap=sum_p/zn;ay=rad2deg(std::atan2(sum_sy,sum_cy));zero_ready=true;test_start=recv;std::cout<<"[ZERO] roll="<<ar<<" pitch="<<ap<<" yaw="<<ay<<"\n";}}}else if(mm.msgid==MAVLINK_MSG_ID_TIMESYNC){mavlink_timesync_t a{};mavlink_msg_timesync_decode(&mm,&a);if(a.tc1!=0&&pending!=0&&a.ts1==pending){TimeSyncSample s;s.t0_rpi_ns=pending;s.t1_rpi_ns=recv;s.fc_ns=a.tc1;s.rtt_ns=recv-pending;s.rpi_mid_ns=pending+s.rtt_ns/2;s.good=s.rtt_ns>0&&nsToMs(s.rtt_ns)<=MAX_TIMESYNC_RTT_MS;syncs.push_back(s);pending=0;}}}}}
      if(pending&&monotonicNs()-pending>20000000LL)pending=0;
      now=monotonicNs();if(now-last_gui>33000000LL){last_gui=now;double rr=0,rp=0,ry=0;if(have_att&&!atts.empty()&&zero_ready){rr=relDeg1529(atts.back().r,ar);rp=relDeg1529(atts.back().p,ap);ry=relDeg1529(atts.back().y,ay);}drawGui1529(screen,latest_gray,zero_ready?(now-test_start)*1e-9:0.0,rr,rp,ry,have_att,zero_ready);cv::imshow(WIN,screen);int k=cv::waitKeyEx(1);if(k==27)abort=true;}
    }
    mj.flush();mj.close();ClockMapping map=estimateClockMapping(syncs);if(!map.valid)throw std::runtime_error("Not enough valid TIMESYNC samples");
    std::ofstream ci(OUT_CAM);ci<<"sequence,v4l2_timestamp_ns,camera_timestamp_corrected_ns,recv_rpi_ns,delivery_latency_ms,mjpeg_offset,bytes_used,flags\n"<<std::fixed<<std::setprecision(9);for(auto&s:cams)ci<<s.sequence<<','<<s.v4l2_ns<<','<<s.corrected_ns<<','<<s.recv_ns<<','<<nsToMs(s.recv_ns-s.v4l2_ns)<<','<<s.mjpeg_offset<<','<<s.bytes_used<<','<<cameraTimestampFlags(s.flags)<<'\n';
    std::ofstream csv(OUT_CSV);csv<<"event,recv_rpi_ns,source_timestamp_ns,mapped_rpi_ns,transport_latency_ms,c5,c6,c7,xacc_m_s2,yacc_m_s2,zacc_m_s2,xgyro_rad_s,ygyro_rad_s,zgyro_rad_s\n"<<std::fixed<<std::setprecision(9);for(auto&s:imus){int64_t mn=map.map(s.fc_ns);csv<<"IMU,"<<s.recv_ns<<','<<s.fc_ns<<','<<mn<<','<<nsToMs(s.recv_ns-mn)<<",,,,"<<s.xacc<<','<<s.yacc<<','<<s.zacc<<','<<s.xgyro<<','<<s.ygyro<<','<<s.zgyro<<'\n';}
    std::ofstream af(OUT_ATT);af<<"recv_rpi_ns,source_timestamp_ns,mapped_rpi_ns,roll_deg,pitch_deg,yaw_deg,rel_roll_deg,rel_pitch_deg,rel_yaw_deg,rollspeed,pitchspeed,yawspeed\n"<<std::fixed<<std::setprecision(9);for(auto&s:atts){int64_t mn=map.map(s.src_ns);af<<s.recv_ns<<','<<s.src_ns<<','<<mn<<','<<s.r<<','<<s.p<<','<<s.y<<','<<relDeg1529(s.r,ar)<<','<<relDeg1529(s.p,ap)<<','<<relDeg1529(s.y,ay)<<','<<s.rs<<','<<s.ps<<','<<s.ys<<'\n';}
    std::ofstream pf(OUT_PHASES);pf<<"phase,start_rpi_ns,end_rpi_ns,target_yaw_deg,expected_xy_mm\n";double t=0;for(const auto&p:PHASES){int64_t a=test_start+(int64_t)std::llround(t*1e9);t+=p.duration;int64_t b=test_start+(int64_t)std::llround(t*1e9);pf<<p.name<<','<<a<<','<<b<<','<<p.target_yaw<<','<<p.expected_xy_mm<<'\n';}
    size_t good=0;for(auto&s:syncs)if(s.good)++good;std::cout<<"\n================ FINAL SUITE RECORDER V15.29 ================\n"<<"camera frames: "<<cams.size()<<"\nsource drops: "<<drops<<"\nIMU rows: "<<imus.size()<<"\nATTITUDE rows: "<<atts.size()<<"\nTIMESYNC good: "<<good<<'/'<<syncs.size()<<"\nCSV: "<<OUT_CSV<<"\nCAM: "<<OUT_CAM<<"\nMJPEG: "<<OUT_MJPEG<<"\nATTITUDE: "<<OUT_ATT<<"\nPHASES: "<<OUT_PHASES<<"\nRESULT: "<<(abort?"ABORTED":"PASS")<<"\n";
    cleanup1529(sfd,cfd,streaming,buf,sys,comp,rates);return abort?2:0;
  }catch(const std::exception&e){std::cerr<<"[FATAL] "<<e.what()<<"\n";cleanup1529(sfd,cfd,streaming,buf,sys,comp,rates);return 1;}
}
