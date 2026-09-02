// JT-ZERO Stage 11.8 v14: GUI yaw return diagnostic.
// Protocol: 2 s zero -> 10 s STILL @0 -> 15 s YAW to +90 -> 10 s STILL @+90
//           -> 15 s YAW back to 0 -> 10 s STILL @0.
// Russian fullscreen GUI + raw Camera/IMU/ATTITUDE recording + automatic state comparison.

#define main jtzero_camera_imu_logger_unused_main_v14
#include "camera_imu_extrinsics_logger.cpp"
#undef main

namespace {

constexpr const char* OUT_CSV="/home/vio/jtzero_yaw_return_v14.csv";
constexpr const char* OUT_CAM="/home/vio/jtzero_yaw_return_v14_camera.csv";
constexpr const char* OUT_MJPEG="/home/vio/jtzero_yaw_return_v14.mjpg";
constexpr const char* OUT_ATT="/home/vio/jtzero_yaw_return_v14_attitude.csv";
constexpr const char* WIN="JT-Zero: YAW туда и обратно";

constexpr double ZERO_SEC=2.0;
constexpr double STILL0_A=10.0;
constexpr double YAW_OUT=15.0;
constexpr double STILL90=10.0;
constexpr double YAW_BACK=15.0;
constexpr double STILL0_B=10.0;
constexpr double T1=STILL0_A;
constexpr double T2=T1+YAW_OUT;
constexpr double T3=T2+STILL90;
constexpr double T4=T3+YAW_BACK;
constexpr double TOTAL=T4+STILL0_B;
constexpr double RP_OK_DEG=4.0;
constexpr double YAW_OK_DEG=8.0;

struct Att14 {int64_t recv_ns=0,src_ns=0;double r=0,p=0,y=0,rs=0,ps=0,ys=0;};
struct ImuTimed14 {double t=0;double ax=0,ay=0,az=0,gx=0,gy=0,gz=0;};
struct AttTimed14 {double t=0,r=0,p=0,y=0;};
struct Avg14 {size_t n=0;double ax=0,ay=0,az=0,gx=0,gy=0,gz=0,r=0,p=0,sy=0,cy=0;};

double rel14(double a,double z){return wrap180(a-z);}

void txt14(cv::Mat&im,const std::string&s,cv::Point p,cv::Scalar c=cv::Scalar(255,255,255),int px=22){
  cv::addText(im,s,p,"DejaVu Sans",px,c,cv::QT_FONT_NORMAL,cv::QT_STYLE_NORMAL,0);
}
int gx14(double v,double lo,double hi,int x,int w){double u=(v-lo)/(hi-lo);u=std::clamp(u,0.0,1.0);return x+(int)std::lround(u*w);}
void gauge14(cv::Mat&im,int x,int y,int w,const std::string&name,double v,double target,double tol,double lo,double hi){
  cv::rectangle(im,{x,y,w,28},{70,70,70},cv::FILLED);
  int a=gx14(target-tol,lo,hi,x,w),b=gx14(target+tol,lo,hi,x,w);if(b<a)std::swap(a,b);
  cv::rectangle(im,{a,y,std::max(1,b-a),28},{0,140,0},cv::FILLED);
  int z=gx14(0,lo,hi,x,w),q=gx14(v,lo,hi,x,w),t=gx14(target,lo,hi,x,w);
  cv::line(im,{z,y-5},{z,y+33},{160,160,160},1);cv::line(im,{t,y-5},{t,y+33},{0,255,255},2);
  bool ok=std::abs(wrap180(v-target))<=tol;cv::line(im,{q,y-8},{q,y+36},ok?cv::Scalar(0,255,0):cv::Scalar(0,0,255),4,cv::LINE_AA);
  std::ostringstream ss;ss<<name<<": "<<std::fixed<<std::setprecision(1)<<v<<"°";txt14(im,ss.str(),{x,y-12});
}

void phase14(double t,std::string&label,double&remain,double&target,bool&moving){
  moving=false;
  if(t<T1){label="ФАЗА 1: ПОКОЙ В ИСХОДНОМ ПОЛОЖЕНИИ";remain=T1-t;target=0;}
  else if(t<T2){label="ФАЗА 2: ПЛАВНО ПОВЕРНИ YAW ДО +90°";remain=T2-t;target=90;moving=true;}
  else if(t<T3){label="ФАЗА 3: СТОП НА +90°, НЕ ДВИГАТЬ";remain=T3-t;target=90;}
  else if(t<T4){label="ФАЗА 4: ПЛАВНО ВЕРНИ YAW К 0°";remain=T4-t;target=0;moving=true;}
  else {label="ФАЗА 5: ПОКОЙ ПОСЛЕ ВОЗВРАТА";remain=TOTAL-t;target=0;}
}

void gui14(cv::Mat&screen,const cv::Mat&gray,double elapsed,double rr,double rp,double ry,bool have_att,bool zero_ready){
  screen.setTo(cv::Scalar(18,18,18));
  if(!gray.empty()){cv::Mat b;cv::cvtColor(gray,b,cv::COLOR_GRAY2BGR);cv::resize(b,b,{760,570});b.copyTo(screen(cv::Rect(20,120,760,570)));cv::rectangle(screen,{20,120,760,570},{180,180,180},1);cv::line(screen,{400,120},{400,690},{100,100,100},1);cv::line(screen,{20,405},{780,405},{100,100,100},1);}
  txt14(screen,"JT-ZERO — YAW +90° И ВОЗВРАТ В 0°",{35,45});
  if(!zero_ready){txt14(screen,"КАЛИБРОВКА НУЛЯ: стенд не трогать",{35,82},{0,220,255});}
  else{
    std::string ph;double rem=0,target=0;bool moving=false;phase14(elapsed,ph,rem,target,moving);
    txt14(screen,ph,{35,82},{0,220,255});std::ostringstream os;os<<"Осталось: "<<std::fixed<<std::setprecision(1)<<std::max(0.0,rem)<<" с";txt14(screen,os.str(),{850,82});
    gauge14(screen,840,185,380,"ROLL",rr,0,RP_OK_DEG,-20,20);gauge14(screen,840,285,380,"PITCH",rp,0,RP_OK_DEG,-20,20);gauge14(screen,840,385,380,"YAW",ry,target,YAW_OK_DEG,-120,120);
    bool rpok=std::abs(rr)<=RP_OK_DEG&&std::abs(rp)<=RP_OK_DEG;
    txt14(screen,rpok?"ROLL/PITCH В ЗЕЛЁНОЙ ЗОНЕ":"ВНИМАНИЕ: ROLL/PITCH ВНЕ ЗОНЫ",{840,475},rpok?cv::Scalar(0,255,0):cv::Scalar(0,0,255));
    txt14(screen,moving?"Вращай только вокруг оси YAW":"Стенд не трогать",{840,525});
    txt14(screen,"Цель: проверить возврат ACC и PITCH после YAW",{840,575},{210,210,210},18);
  }
  if(!have_att)txt14(screen,"НЕТ ATTITUDE ОТ FC",{840,620},{0,0,255});txt14(screen,"ESC — аварийно завершить",{840,665});
}

void cleanup14(int sfd,int cfd,bool streaming,std::vector<CameraBuffer>&buf,uint8_t sys,uint8_t comp,bool rates){
  if(rates&&sfd>=0&&sys){try{requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);}catch(...){}}
  if(streaming&&cfd>=0){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;ioctl(cfd,VIDIOC_STREAMOFF,&t);}for(auto&b:buf)if(b.start&&b.length)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);cv::destroyAllWindows();
}

Avg14 avgWindow14(const std::vector<ImuTimed14>&iv,const std::vector<AttTimed14>&av,double a,double b){
  Avg14 o;for(auto&s:iv)if(s.t>=a&&s.t<=b){o.ax+=s.ax;o.ay+=s.ay;o.az+=s.az;o.gx+=s.gx;o.gy+=s.gy;o.gz+=s.gz;++o.n;}size_t ni=o.n;if(ni){o.ax/=ni;o.ay/=ni;o.az/=ni;o.gx/=ni;o.gy/=ni;o.gz/=ni;}
  size_t na=0;for(auto&s:av)if(s.t>=a&&s.t<=b){o.r+=s.r;o.p+=s.p;o.sy+=std::sin(s.y*PI/180.0);o.cy+=std::cos(s.y*PI/180.0);++na;}if(na){o.r/=na;o.p/=na;o.sy=rad2deg(std::atan2(o.sy,o.cy));}return o;
}
double gravAngle14(const Avg14&a,const Avg14&b){double na=std::sqrt(a.ax*a.ax+a.ay*a.ay+a.az*a.az),nb=std::sqrt(b.ax*b.ax+b.ay*b.ay+b.az*b.az);double c=(a.ax*b.ax+a.ay*b.ay+a.az*b.az)/(na*nb);c=std::clamp(c,-1.0,1.0);return std::acos(c)*180.0/PI;}
void printAvg14(const char*name,const Avg14&s){std::cout<<name<<"\n  ACC FRD: ["<<std::fixed<<std::setprecision(6)<<s.ax<<" "<<s.ay<<" "<<s.az<<"] m/s^2\n  GYRO FRD: ["<<std::setprecision(8)<<s.gx<<" "<<s.gy<<" "<<s.gz<<"] rad/s\n  ATT RPY: ["<<std::setprecision(3)<<s.r<<" "<<s.p<<" "<<s.sy<<"] deg\n";}

} // namespace

int main(){
  int sfd=-1,cfd=-1;bool streaming=false,rates=false;std::vector<CameraBuffer>buf;uint8_t sys=0,comp=0;
  try{
    sfd=openSerial();std::cout<<"[MAV] waiting for HEARTBEAT...\n";mavlink_status_t ms{};mavlink_message_t mm{};int64_t dl=monotonicNs()+10000000000LL;
    while(monotonicNs()<dl&&!sys){pollfd p{sfd,POLLIN,0};if(poll(&p,1,100)>0){uint8_t b[2048];ssize_t n=read(sfd,b,sizeof(b));for(ssize_t i=0;i<n;++i)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&mm,&ms)&&mm.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=mm.sysid;comp=mm.compid;break;}}}if(!sys)throw std::runtime_error("HEARTBEAT timeout");
    requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,IMU_RATE_HZ);requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,ATTITUDE_RATE_HZ);rates=true;
    cfd=open(CAMERA_DEVICE,O_RDWR|O_NONBLOCK);if(cfd<0)fail("open camera");configureCamera(cfd);buf=initCameraBuffers(cfd);v4l2_buf_type typ=V4L2_BUF_TYPE_VIDEO_CAPTURE;if(xioctl(cfd,VIDIOC_STREAMON,&typ)==-1)fail("STREAMON");streaming=true;discardWarmup(cfd,CAMERA_WARMUP_FRAMES);
    cv::namedWindow(WIN,cv::WINDOW_NORMAL);cv::setWindowProperty(WIN,cv::WND_PROP_FULLSCREEN,cv::WINDOW_FULLSCREEN);cv::Mat screen(720,1280,CV_8UC3),latest_gray;
    std::vector<CameraSample>cams;std::vector<ImuSample>imus;std::vector<TimeSyncSample>syncs;std::vector<Att14>atts;std::vector<ImuTimed14>it;std::vector<AttTimed14>at;
    cams.reserve(10000);imus.reserve(15000);syncs.reserve(800);atts.reserve(4000);it.reserve(15000);at.reserve(4000);std::ofstream mj(OUT_MJPEG,std::ios::binary|std::ios::trunc);if(!mj)throw std::runtime_error("Cannot create MJPEG");
    bool have_seq=false;uint32_t prev_seq=0;uint64_t drops=0;int64_t pending=0,next_ts=monotonicNs();bool have_att=false,zero_ready=false;double ar=0,ap=0,ay=0,sum_r=0,sum_p=0,sum_sy=0,sum_cy=0;size_t zn=0;int64_t zero_start=0,test_start=0,last_gui=0;uint32_t preview_div=0;bool abort=false;
    tcflush(sfd,TCIFLUSH);std::memset(&ms,0,sizeof(ms));std::memset(&mm,0,sizeof(mm));std::cout<<"[GUI] YAW return v14 запущен.\n";
    while(!abort){int64_t now=monotonicNs();double elapsed=zero_ready?(now-test_start)*1e-9:0;if(zero_ready&&elapsed>=TOTAL)break;if(now>=next_ts&&pending==0){pending=now;sendTimesync(sfd,pending,sys,comp);next_ts=now+TIMESYNC_PERIOD_NS;}
      pollfd pf[2]={{cfd,POLLIN,0},{sfd,POLLIN,0}};int rc=poll(pf,2,2);if(rc<0){if(errno==EINTR)continue;fail("poll");}
      if(pf[0].revents&POLLIN){for(;;){v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;if(xioctl(cfd,VIDIOC_DQBUF,&b)==-1){if(errno==EAGAIN)break;fail("DQBUF");}int64_t recv=monotonicNs(),v4=timevalToNs(b.timestamp),corr=jtzero::timesync::correctCameraTimestampNs(v4);if(have_seq){uint32_t ex=prev_seq+1;if(b.sequence!=ex)drops+=uint32_t(b.sequence-ex);}prev_seq=b.sequence;have_seq=true;uint64_t off=(uint64_t)mj.tellp();mj.write((const char*)buf[b.index].start,b.bytesused);cams.push_back({recv,v4,corr,b.sequence,b.flags,b.bytesused,off});if((preview_div++%4)==0){std::vector<unsigned char>j((unsigned char*)buf[b.index].start,(unsigned char*)buf[b.index].start+b.bytesused);latest_gray=cv::imdecode(j,cv::IMREAD_GRAYSCALE);}if(xioctl(cfd,VIDIOC_QBUF,&b)==-1)fail("QBUF");}}
      if(pf[1].revents&POLLIN){uint8_t by[8192];for(;;){ssize_t n=read(sfd,by,sizeof(by));if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;for(ssize_t i=0;i<n;++i){if(!mavlink_parse_char(MAVLINK_COMM_0,by[i],&mm,&ms))continue;int64_t recv=monotonicNs();if(mm.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){mavlink_highres_imu_t a{};mavlink_msg_highres_imu_decode(&mm,&a);ImuSample s;s.recv_ns=recv;s.fc_ns=(int64_t)a.time_usec*1000;s.xacc=a.xacc;s.yacc=a.yacc;s.zacc=a.zacc;s.xgyro=a.xgyro;s.ygyro=a.ygyro;s.zgyro=a.zgyro;s.temperature=a.temperature;s.fields_updated=a.fields_updated;
#ifdef MAVLINK_MSG_HIGHRES_IMU_FIELD_ID_LEN
s.imu_id=a.id;
#endif
imus.push_back(s);if(zero_ready)it.push_back({(recv-test_start)*1e-9,a.xacc,a.yacc,a.zacc,a.xgyro,a.ygyro,a.zgyro});}
        else if(mm.msgid==MAVLINK_MSG_ID_ATTITUDE){mavlink_attitude_t a{};mavlink_msg_attitude_decode(&mm,&a);double r=rad2deg(a.roll),p=rad2deg(a.pitch),y=rad2deg(a.yaw);atts.push_back({recv,(int64_t)a.time_boot_ms*1000000LL,r,p,y,a.rollspeed,a.pitchspeed,a.yawspeed});have_att=true;if(!zero_ready){if(!zero_start)zero_start=recv;sum_r+=r;sum_p+=p;sum_sy+=std::sin(a.yaw);sum_cy+=std::cos(a.yaw);++zn;if((recv-zero_start)*1e-9>=ZERO_SEC&&zn>20){ar=sum_r/zn;ap=sum_p/zn;ay=rad2deg(std::atan2(sum_sy,sum_cy));zero_ready=true;test_start=recv;std::cout<<"[ZERO] roll="<<ar<<" pitch="<<ap<<" yaw="<<ay<<"\n";}}else at.push_back({(recv-test_start)*1e-9,r,p,y});}
        else if(mm.msgid==MAVLINK_MSG_ID_TIMESYNC){mavlink_timesync_t a{};mavlink_msg_timesync_decode(&mm,&a);if(a.tc1!=0&&pending!=0&&a.ts1==pending){TimeSyncSample s;s.t0_rpi_ns=pending;s.t1_rpi_ns=recv;s.fc_ns=a.tc1;s.rtt_ns=recv-pending;s.rpi_mid_ns=pending+s.rtt_ns/2;s.good=s.rtt_ns>0&&nsToMs(s.rtt_ns)<=MAX_TIMESYNC_RTT_MS;syncs.push_back(s);pending=0;}}}}}
      if(pending&&monotonicNs()-pending>20000000LL)pending=0;now=monotonicNs();if(now-last_gui>33000000LL){last_gui=now;double rr=0,rp=0,ry=0;if(have_att&&!atts.empty()&&zero_ready){rr=rel14(atts.back().r,ar);rp=rel14(atts.back().p,ap);ry=rel14(atts.back().y,ay);}gui14(screen,latest_gray,zero_ready?(now-test_start)*1e-9:0,rr,rp,ry,have_att,zero_ready);cv::imshow(WIN,screen);if(cv::waitKeyEx(1)==27)abort=true;}
    }
    mj.flush();mj.close();ClockMapping map=estimateClockMapping(syncs);if(!map.valid)throw std::runtime_error("Not enough valid TIMESYNC samples");
    std::ofstream ci(OUT_CAM);ci<<"sequence,v4l2_timestamp_ns,camera_timestamp_corrected_ns,recv_rpi_ns,delivery_latency_ms,mjpeg_offset,bytes_used,flags\n"<<std::fixed<<std::setprecision(9);for(auto&s:cams)ci<<s.sequence<<','<<s.v4l2_ns<<','<<s.corrected_ns<<','<<s.recv_ns<<','<<nsToMs(s.recv_ns-s.v4l2_ns)<<','<<s.mjpeg_offset<<','<<s.bytes_used<<','<<cameraTimestampFlags(s.flags)<<'\n';
    std::ofstream csv(OUT_CSV);csv<<"event,recv_rpi_ns,source_timestamp_ns,mapped_rpi_ns,transport_latency_ms,c5,c6,c7,xacc_m_s2,yacc_m_s2,zacc_m_s2,xgyro_rad_s,ygyro_rad_s,zgyro_rad_s\n"<<std::fixed<<std::setprecision(9);for(auto&s:imus){int64_t mn=map.map(s.fc_ns);csv<<"IMU,"<<s.recv_ns<<','<<s.fc_ns<<','<<mn<<','<<nsToMs(s.recv_ns-mn)<<",,,,"<<s.xacc<<','<<s.yacc<<','<<s.zacc<<','<<s.xgyro<<','<<s.ygyro<<','<<s.zgyro<<'\n';}
    std::ofstream af(OUT_ATT);af<<"recv_rpi_ns,source_timestamp_ns,mapped_rpi_ns,roll_deg,pitch_deg,yaw_deg,rel_roll_deg,rel_pitch_deg,rel_yaw_deg,rollspeed,pitchspeed,yawspeed\n"<<std::fixed<<std::setprecision(9);for(auto&s:atts){int64_t mn=map.map(s.src_ns);af<<s.recv_ns<<','<<s.src_ns<<','<<mn<<','<<s.r<<','<<s.p<<','<<s.y<<','<<rel14(s.r,ar)<<','<<rel14(s.p,ap)<<','<<rel14(s.y,ay)<<','<<s.rs<<','<<s.ps<<','<<s.ys<<'\n';}
    Avg14 A=avgWindow14(it,at,2,8),B=avgWindow14(it,at,T2+2,T3-2),C=avgWindow14(it,at,T4+2,TOTAL-2);size_t good=0;for(auto&s:syncs)if(s.good)++good;
    std::cout<<"\n================ YAW RETURN GUI V14 RESULT ================\n";printAvg14("START @0°",A);printAvg14("YAW @+90°",B);printAvg14("RETURN @0°",C);
    std::cout<<"\nGRAVITY ANGLE START -> +90: "<<std::fixed<<std::setprecision(3)<<gravAngle14(A,B)<<" deg\nGRAVITY ANGLE START -> RETURN: "<<gravAngle14(A,C)<<" deg\n"
             <<"PITCH START -> +90: "<<rel14(B.p,A.p)<<" deg\nPITCH START -> RETURN: "<<rel14(C.p,A.p)<<" deg\nYAW START -> +90: "<<rel14(B.sy,A.sy)<<" deg\nYAW START -> RETURN: "<<rel14(C.sy,A.sy)<<" deg\n"
             <<"camera frames: "<<cams.size()<<"  source drops: "<<drops<<"\nIMU rows: "<<imus.size()<<"  ATTITUDE rows: "<<atts.size()<<"\nTIMESYNC good: "<<good<<'/'<<syncs.size()<<"\nCSV: "<<OUT_CSV<<"\nATTITUDE: "<<OUT_ATT<<"\nRESULT: "<<(abort?"ABORTED":"PASS")<<"\n";
    cleanup14(sfd,cfd,streaming,buf,sys,comp,rates);return abort?2:0;
  }catch(const std::exception&e){std::cerr<<"[FATAL] "<<e.what()<<"\n";cleanup14(sfd,cfd,streaming,buf,sys,comp,rates);return 1;}
}
