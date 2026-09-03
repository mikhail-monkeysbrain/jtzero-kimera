// JT-ZERO Stage 11 diagnostic v15.32
// Short synchronized yaw test: camera + ATTITUDE + ATTITUDE_QUATERNION + HIGHRES_IMU.
// Purpose: distinguish a real physical tilt from an FC attitude-estimation tilt during yaw.

#define main jtzero_camera_imu_logger_unused_main_v1532
#include "camera_imu_extrinsics_logger.cpp"
#undef main

#include <array>
#include <sys/stat.h>

namespace {

constexpr const char* WIN1532 = "JT-Zero: yaw / gravity / camera v15.32";
constexpr const char* OUT_EULER1532 = "/home/vio/jtzero_attitude_v15_32.csv";
constexpr const char* OUT_QUAT1532 = "/home/vio/jtzero_quaternion_v15_32.csv";
constexpr const char* OUT_IMU1532 = "/home/vio/jtzero_imu_v15_32.csv";
constexpr const char* OUT_CAM1532 = "/home/vio/jtzero_camera_v15_32.csv";
constexpr const char* OUT_MJPEG1532 = "/home/vio/jtzero_camera_v15_32.mjpg";
constexpr const char* OUT_PHASES1532 = "/home/vio/jtzero_phases_v15_32.csv";
constexpr double ZERO_SEC1532 = 2.0;
constexpr double RP_TOL1532 = 2.0;
constexpr double TILT_TOL1532 = 2.0;
constexpr double YAW_TOL1532 = 8.0;
constexpr double ACC_LP_TAU1532 = 0.25;

struct Phase1532 { const char* name; double duration; double target_yaw; const char* instruction; };
constexpr Phase1532 PHASES1532[] = {
  {"STILL0",5.0,0.0,"НЕ ДВИГАТЬ"},
  {"YAW_90",15.0,90.0,"ПЛАВНО ПОВЕРНИ YAW ДО +90°"},
  {"STILL90",5.0,90.0,"НЕ ДВИГАТЬ"},
  {"YAW_RETURN",15.0,0.0,"ПЛАВНО ВЕРНИ YAW В 0°"},
  {"STILL_END",5.0,0.0,"НЕ ДВИГАТЬ"},
};
constexpr size_t NPH1532=sizeof(PHASES1532)/sizeof(PHASES1532[0]);

struct Euler1532 {int64_t recv_ns=0,src_ns=0;double r=0,p=0,y=0,rs=0,ps=0,ys=0;};
struct Quat1532 {int64_t recv_ns=0,src_ns=0;double w=1,x=0,y=0,z=0,rs=0,ps=0,ys=0;};
struct Imu1532 {int64_t recv_ns=0,src_ns=0;double ax=0,ay=0,az=0,gx=0,gy=0,gz=0;};
struct Cam1532 {int64_t recv_ns=0,v4l2_ns=0,corr_ns=0;uint32_t seq=0,flags=0,bytes=0;uint64_t off=0;};
struct Q1532 {double w=1,x=0,y=0,z=0;};

Q1532 qnorm1532(Q1532 q){double n=std::sqrt(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z);if(n<=1e-12)return{};q.w/=n;q.x/=n;q.y/=n;q.z/=n;return q;}
Q1532 qconj1532(Q1532 q){return{q.w,-q.x,-q.y,-q.z};}
Q1532 qmul1532(Q1532 a,Q1532 b){return{a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z,a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w};}
std::array<double,3> bodyZ1532(Q1532 q){q=qnorm1532(q);return{2*(q.x*q.z+q.w*q.y),2*(q.y*q.z-q.w*q.x),1-2*(q.x*q.x+q.y*q.y)};}
double bodyZTilt1532(Q1532 q0,Q1532 q){auto a=bodyZ1532(q0),b=bodyZ1532(q);double d=std::clamp(a[0]*b[0]+a[1]*b[1]+a[2]*b[2],-1.0,1.0);return rad2deg(std::acos(d));}
void qEuler1532(Q1532 q,double&r,double&p,double&y){q=qnorm1532(q);r=rad2deg(std::atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y)));double s=std::clamp(2*(q.w*q.y-q.z*q.x),-1.0,1.0);p=rad2deg(std::asin(s));y=rad2deg(std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)));}

double vecTilt1532(const std::array<double,3>&a,const std::array<double,3>&b){double na=std::sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]),nb=std::sqrt(b[0]*b[0]+b[1]*b[1]+b[2]*b[2]);if(na<1e-9||nb<1e-9)return 0;double d=(a[0]*b[0]+a[1]*b[1]+a[2]*b[2])/(na*nb);return rad2deg(std::acos(std::clamp(d,-1.0,1.0)));}

double totalSec1532(){double s=0;for(auto&p:PHASES1532)s+=p.duration;return s;}
size_t phaseIdx1532(double e,double*rem=nullptr){double t=0;for(size_t i=0;i<NPH1532;++i){double z=t+PHASES1532[i].duration;if(e<z){if(rem)*rem=z-e;return i;}t=z;}if(rem)*rem=0;return NPH1532-1;}
void text1532(cv::Mat&i,const std::string&s,cv::Point p,cv::Scalar c=cv::Scalar(255,255,255),int h=20){cv::addText(i,s,p,"DejaVu Sans",h,c,cv::QT_FONT_NORMAL,cv::QT_STYLE_NORMAL,0);}
int gx1532(double v,double lo,double hi,int x,int w){double u=std::clamp((v-lo)/(hi-lo),0.0,1.0);return x+(int)std::lround(u*w);}
void gauge1532(cv::Mat&i,int x,int y,int w,const std::string&n,double v,double target,double tol,double lo,double hi){cv::rectangle(i,{x,y,w,24},{65,65,65},cv::FILLED);int a=gx1532(target-tol,lo,hi,x,w),b=gx1532(target+tol,lo,hi,x,w);if(b<a)std::swap(a,b);cv::rectangle(i,{a,y,std::max(1,b-a),24},{0,130,0},cv::FILLED);int pv=gx1532(v,lo,hi,x,w),pt=gx1532(target,lo,hi,x,w);cv::line(i,{pt,y-4},{pt,y+28},{0,255,255},2);bool ok=std::abs(wrap180(v-target))<=tol;cv::line(i,{pv,y-7},{pv,y+31},ok?cv::Scalar(0,255,0):cv::Scalar(0,0,255),4,cv::LINE_AA);std::ostringstream ss;ss<<n<<": "<<std::fixed<<std::setprecision(2)<<v<<"°";text1532(i,ss.str(),{x,y-8},ok?cv::Scalar(0,255,0):cv::Scalar(0,0,255),18);}

void draw1532(cv::Mat&im,const cv::Mat&gray,double elapsed,bool zero,bool he,bool hq,bool hi,
              double rr,double rp,double ry,double qtilt,double atilt,double gx,double gy,double gz){
  im.setTo(cv::Scalar(18,18,18));
  text1532(im,"JT-ZERO — YAW / GRAVITY / CAMERA v15.32",{25,35},{255,255,255},22);
  if(!zero){text1532(im,"КАЛИБРОВКА НУЛЯ 2 С: БПЛА НЕПОДВИЖЕН",{25,72},{0,220,255},21);}
  else{double rem=0;size_t pi=phaseIdx1532(elapsed,&rem);const auto&ph=PHASES1532[pi];std::ostringstream ss;ss<<"ФАЗА "<<(pi+1)<<"/"<<NPH1532<<": "<<ph.name<<"   Осталось: "<<std::fixed<<std::setprecision(1)<<rem<<" с";text1532(im,ss.str(),{25,72},{0,220,255},20);text1532(im,ph.instruction,{25,103},(std::string(ph.name).find("STILL")==0)?cv::Scalar(0,255,0):cv::Scalar(0,220,255),20);
    if(!gray.empty()){cv::Mat b;cv::cvtColor(gray,b,cv::COLOR_GRAY2BGR);cv::resize(b,b,{720,540});b.copyTo(im(cv::Rect(20,130,720,540)));cv::rectangle(im,{20,130,720,540},{160,160,160},1);cv::line(im,{380,130},{380,670},{90,90,90},1);cv::line(im,{20,400},{740,400},{90,90,90},1);}
    int x=790,w=430;
    text1532(im,"ATTITUDE Euler",{x,145},{255,255,255},19);gauge1532(im,x,185,w,"ROLL",rr,0,RP_TOL1532,-12,12);gauge1532(im,x,245,w,"PITCH",rp,0,RP_TOL1532,-12,12);gauge1532(im,x,305,w,"YAW",ry,ph.target_yaw,YAW_TOL1532,-120,120);
    text1532(im,"Наклон относительно старта",{x,365},{255,255,255},19);gauge1532(im,x,405,w,"QUAT BODY-Z",qtilt,0,TILT_TOL1532,0,15);gauge1532(im,x,465,w,"ACC GRAVITY",atilt,0,TILT_TOL1532,0,15);
    std::ostringstream gs;gs<<"GYRO rad/s   X="<<std::fixed<<std::setprecision(3)<<gx<<"   Y="<<gy<<"   Z="<<gz;text1532(im,gs.str(),{x,545},{220,220,220},18);
    bool agree=std::abs(qtilt-atilt)<=2.0;std::string verdict;if(qtilt>2&&atilt<=2)verdict="FC TILT, НО GRAVITY СТАБИЛЬНА";else if(qtilt>2&&atilt>2)verdict="BODY-Z И GRAVITY НАКЛОНЕНЫ";else verdict="НАКЛОН НЕ ОБНАРУЖЕН";text1532(im,verdict,{x,600},agree?cv::Scalar(0,255,0):cv::Scalar(0,220,255),19);
    std::ostringstream tt;tt<<"Время: "<<std::fixed<<std::setprecision(1)<<elapsed<<" / "<<totalSec1532()<<" с";text1532(im,tt.str(),{x,650},{180,180,180},17);
  }
  if(!he)text1532(im,"НЕТ ATTITUDE",{820,700},{0,0,255},18);if(!hq)text1532(im,"НЕТ QUATERNION",{980,700},{0,0,255},18);if(!hi)text1532(im,"НЕТ HIGHRES_IMU",{820,735},{0,0,255},18);text1532(im,"ESC — аварийно завершить",{1010,35},{180,180,180},16);
}

uint64_t fsize1532(const char*p){struct stat s{};return stat(p,&s)==0?(uint64_t)s.st_size:0;}

} // namespace

int main(){
  int sfd=-1,cfd=-1;bool streaming=false,rates=false;std::vector<CameraBuffer>buf;uint8_t sys=0,comp=0;
  try{
    sfd=openSerial();std::cout<<"[MAV] waiting for HEARTBEAT...\n";mavlink_status_t ms{};mavlink_message_t mm{};int64_t dl=monotonicNs()+10000000000LL;
    while(monotonicNs()<dl&&!sys){pollfd p{sfd,POLLIN,0};if(poll(&p,1,100)>0){uint8_t b[2048];ssize_t n=read(sfd,b,sizeof(b));for(ssize_t i=0;i<n;++i)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&mm,&ms)&&mm.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=mm.sysid;comp=mm.compid;break;}}}
    if(!sys)throw std::runtime_error("HEARTBEAT timeout");
    requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,50);requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE_QUATERNION,50);requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,200);rates=true;

    cfd=open(CAMERA_DEVICE,O_RDWR|O_NONBLOCK);if(cfd<0)fail("open camera");configureCamera(cfd);buf=initCameraBuffers(cfd);v4l2_buf_type typ=V4L2_BUF_TYPE_VIDEO_CAPTURE;if(xioctl(cfd,VIDIOC_STREAMON,&typ)==-1)fail("STREAMON");streaming=true;discardWarmup(cfd,CAMERA_WARMUP_FRAMES);

    std::ofstream mj(OUT_MJPEG1532,std::ios::binary|std::ios::trunc);if(!mj)throw std::runtime_error("Cannot create MJPEG");
    std::vector<Euler1532> es;std::vector<Quat1532> qs;std::vector<Imu1532> is;std::vector<Cam1532> cs;es.reserve(4000);qs.reserve(4000);is.reserve(15000);cs.reserve(8000);
    bool he=false,hq=false,hi=false,zero=false,abort=false,have_seq=false;uint32_t prev_seq=0;uint64_t drops=0;
    double er0=0,ep0=0,ey0=0,se_r=0,se_p=0,se_sy=0,se_cy=0;size_t ne0=0;double sqw=0,sqx=0,sqy=0,sqz=0;size_t nq0=0;Q1532 q0{};std::array<double,3>a0{0,0,0};size_t na0=0;
    std::array<double,3>alp{0,0,0};bool alp_init=false;int64_t alp_last=0;
    int64_t zero_start=0,test_start=0,last_gui=0;cv::Mat latest_gray,screen(800,1280,CV_8UC3);uint32_t preview_div=0;
    cv::namedWindow(WIN1532,cv::WINDOW_NORMAL);cv::setWindowProperty(WIN1532,cv::WND_PROP_FULLSCREEN,cv::WINDOW_FULLSCREEN);tcflush(sfd,TCIFLUSH);std::memset(&ms,0,sizeof(ms));std::memset(&mm,0,sizeof(mm));

    while(!abort){
      int64_t now=monotonicNs();double elapsed=zero?(now-test_start)*1e-9:0;if(zero&&elapsed>=totalSec1532())break;
      pollfd pf[2]={{cfd,POLLIN,0},{sfd,POLLIN,0}};int rc=poll(pf,2,3);if(rc<0){if(errno==EINTR)continue;fail("poll");}
      if(pf[0].revents&POLLIN){for(;;){v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;if(xioctl(cfd,VIDIOC_DQBUF,&b)==-1){if(errno==EAGAIN)break;fail("DQBUF");}int64_t recv=monotonicNs(),v4=timevalToNs(b.timestamp),corr=jtzero::timesync::correctCameraTimestampNs(v4);if(have_seq){uint32_t ex=prev_seq+1;if(b.sequence!=ex)drops+=uint32_t(b.sequence-ex);}prev_seq=b.sequence;have_seq=true;uint64_t off=(uint64_t)mj.tellp();mj.write((const char*)buf[b.index].start,b.bytesused);cs.push_back({recv,v4,corr,b.sequence,b.flags,b.bytesused,off});if((preview_div++%4)==0){std::vector<unsigned char>j((unsigned char*)buf[b.index].start,(unsigned char*)buf[b.index].start+b.bytesused);latest_gray=cv::imdecode(j,cv::IMREAD_GRAYSCALE);}if(xioctl(cfd,VIDIOC_QBUF,&b)==-1)fail("QBUF");}}
      if(pf[1].revents&POLLIN){uint8_t b[8192];for(;;){ssize_t n=read(sfd,b,sizeof(b));if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;for(ssize_t k=0;k<n;++k){if(!mavlink_parse_char(MAVLINK_COMM_0,b[k],&mm,&ms))continue;int64_t recv=monotonicNs();
        if(mm.msgid==MAVLINK_MSG_ID_ATTITUDE){mavlink_attitude_t a{};mavlink_msg_attitude_decode(&mm,&a);double r=rad2deg(a.roll),p=rad2deg(a.pitch),y=rad2deg(a.yaw);es.push_back({recv,(int64_t)a.time_boot_ms*1000000LL,r,p,y,a.rollspeed,a.pitchspeed,a.yawspeed});he=true;if(!zero){if(!zero_start)zero_start=recv;se_r+=r;se_p+=p;se_sy+=std::sin(a.yaw);se_cy+=std::cos(a.yaw);++ne0;}}
        else if(mm.msgid==MAVLINK_MSG_ID_ATTITUDE_QUATERNION){mavlink_attitude_quaternion_t a{};mavlink_msg_attitude_quaternion_decode(&mm,&a);Q1532 q=qnorm1532({a.q1,a.q2,a.q3,a.q4});qs.push_back({recv,(int64_t)a.time_boot_ms*1000000LL,q.w,q.x,q.y,q.z,a.rollspeed,a.pitchspeed,a.yawspeed});hq=true;if(!zero){sqw+=q.w;sqx+=q.x;sqy+=q.y;sqz+=q.z;++nq0;}}
        else if(mm.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){mavlink_highres_imu_t a{};mavlink_msg_highres_imu_decode(&mm,&a);is.push_back({recv,(int64_t)a.time_usec*1000LL,a.xacc,a.yacc,a.zacc,a.xgyro,a.ygyro,a.zgyro});hi=true;std::array<double,3>cur{a.xacc,a.yacc,a.zacc};if(!alp_init){alp=cur;alp_init=true;alp_last=recv;}else{double dt=std::max(0.0,(recv-alp_last)*1e-9),alpha=dt/(ACC_LP_TAU1532+dt);for(int j=0;j<3;++j)alp[j]+=alpha*(cur[j]-alp[j]);alp_last=recv;}if(!zero){for(int j=0;j<3;++j)a0[j]+=a.xacc*(j==0)+a.yacc*(j==1)+a.zacc*(j==2);++na0;}}
      }}}
      if(!zero&&zero_start&&he&&hq&&hi&&(monotonicNs()-zero_start)*1e-9>=ZERO_SEC1532&&ne0>20&&nq0>20&&na0>50){er0=se_r/ne0;ep0=se_p/ne0;ey0=rad2deg(std::atan2(se_sy,se_cy));q0=qnorm1532({sqw/nq0,sqx/nq0,sqy/nq0,sqz/nq0});for(double&v:a0)v/=na0;zero=true;test_start=monotonicNs();std::cout<<"[ZERO] Euler="<<er0<<","<<ep0<<","<<ey0<<" acc="<<a0[0]<<","<<a0[1]<<","<<a0[2]<<"\n";}
      now=monotonicNs();if(now-last_gui>33000000LL){last_gui=now;double rr=0,rp=0,ry=0,qt=0,at=0,gx=0,gy=0,gz=0;if(zero&&he&&!es.empty()){rr=wrap180(es.back().r-er0);rp=wrap180(es.back().p-ep0);ry=wrap180(es.back().y-ey0);}if(zero&&hq&&!qs.empty())qt=bodyZTilt1532(q0,{qs.back().w,qs.back().x,qs.back().y,qs.back().z});if(zero&&hi&&!is.empty()){at=vecTilt1532(a0,alp);gx=is.back().gx;gy=is.back().gy;gz=is.back().gz;}draw1532(screen,latest_gray,zero?(now-test_start)*1e-9:0,zero,he,hq,hi,rr,rp,ry,qt,at,gx,gy,gz);cv::imshow(WIN1532,screen);if(cv::waitKeyEx(1)==27)abort=true;}
    }

    mj.flush();mj.close();if(!mj.good())throw std::runtime_error("MJPEG write failed");
    std::ofstream ef(OUT_EULER1532,std::ios::trunc);ef<<"recv_rpi_ns,source_timestamp_ns,roll_deg,pitch_deg,yaw_deg,rel_roll_deg,rel_pitch_deg,rel_yaw_deg,rollspeed,pitchspeed,yawspeed\n"<<std::fixed<<std::setprecision(9);for(auto&s:es)ef<<s.recv_ns<<','<<s.src_ns<<','<<s.r<<','<<s.p<<','<<s.y<<','<<wrap180(s.r-er0)<<','<<wrap180(s.p-ep0)<<','<<wrap180(s.y-ey0)<<','<<s.rs<<','<<s.ps<<','<<s.ys<<'\n';ef.close();
    std::ofstream qf(OUT_QUAT1532,std::ios::trunc);qf<<"recv_rpi_ns,source_timestamp_ns,q1_w,q2_x,q3_y,q4_z,body_z_tilt_deg,rel_roll_deg,rel_pitch_deg,rel_yaw_deg,rollspeed,pitchspeed,yawspeed\n"<<std::fixed<<std::setprecision(9);for(auto&s:qs){Q1532 q{s.w,s.x,s.y,s.z},qr=qnorm1532(qmul1532(qconj1532(q0),q));double r,p,y;qEuler1532(qr,r,p,y);qf<<s.recv_ns<<','<<s.src_ns<<','<<s.w<<','<<s.x<<','<<s.y<<','<<s.z<<','<<bodyZTilt1532(q0,q)<<','<<r<<','<<p<<','<<y<<','<<s.rs<<','<<s.ps<<','<<s.ys<<'\n';}qf.close();
    std::ofstream imf(OUT_IMU1532,std::ios::trunc);imf<<"recv_rpi_ns,source_timestamp_ns,xacc_m_s2,yacc_m_s2,zacc_m_s2,xgyro_rad_s,ygyro_rad_s,zgyro_rad_s\n"<<std::fixed<<std::setprecision(9);for(auto&s:is)imf<<s.recv_ns<<','<<s.src_ns<<','<<s.ax<<','<<s.ay<<','<<s.az<<','<<s.gx<<','<<s.gy<<','<<s.gz<<'\n';imf.close();
    std::ofstream cf(OUT_CAM1532,std::ios::trunc);cf<<"sequence,v4l2_timestamp_ns,camera_timestamp_corrected_ns,recv_rpi_ns,mjpeg_offset,bytes_used,flags\n";for(auto&s:cs)cf<<s.seq<<','<<s.v4l2_ns<<','<<s.corr_ns<<','<<s.recv_ns<<','<<s.off<<','<<s.bytes<<','<<cameraTimestampFlags(s.flags)<<'\n';cf.close();
    std::ofstream pf(OUT_PHASES1532,std::ios::trunc);pf<<"phase,start_rpi_ns,end_rpi_ns,target_yaw_deg\n";double t=0;for(auto&p:PHASES1532){int64_t a=test_start+(int64_t)std::llround(t*1e9);t+=p.duration;int64_t b=test_start+(int64_t)std::llround(t*1e9);pf<<p.name<<','<<a<<','<<b<<','<<p.target_yaw<<'\n';}pf.close();

    if(rates&&sfd>=0&&sys){requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE_QUATERNION,0);requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);}if(streaming&&cfd>=0){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;ioctl(cfd,VIDIOC_STREAMOFF,&t);}for(auto&b:buf)if(b.start&&b.length)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);cv::destroyAllWindows();
    std::cout<<"\n================ V15.32 RESULT ================\nEuler rows: "<<es.size()<<"\nQuaternion rows: "<<qs.size()<<"\nIMU rows: "<<is.size()<<"\nCamera frames: "<<cs.size()<<"  source drops: "<<drops<<"\nMJPEG bytes: "<<fsize1532(OUT_MJPEG1532)<<"\nEuler CSV: "<<OUT_EULER1532<<"\nQuaternion CSV: "<<OUT_QUAT1532<<"\nIMU CSV: "<<OUT_IMU1532<<"\nCamera CSV: "<<OUT_CAM1532<<"\nMJPEG: "<<OUT_MJPEG1532<<"\nPhases: "<<OUT_PHASES1532<<"\nRESULT: "<<(abort?"ABORTED":"PASS")<<"\n";return abort?2:0;
  }catch(const std::exception&e){std::cerr<<"[FATAL] "<<e.what()<<"\n";if(rates&&sfd>=0&&sys){try{requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE_QUATERNION,0);requestRate(sfd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);}catch(...){}}if(streaming&&cfd>=0){v4l2_buf_type t=V4L2_BUF_TYPE_VIDEO_CAPTURE;ioctl(cfd,VIDIOC_STREAMOFF,&t);}for(auto&b:buf)if(b.start&&b.length)munmap(b.start,b.length);if(cfd>=0)close(cfd);if(sfd>=0)close(sfd);cv::destroyAllWindows();return 1;}
}
