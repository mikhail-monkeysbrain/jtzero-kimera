// JT-ZERO Stage 11 diagnostic v15.31
// Short GUI test to separate Euler R/P coupling from real FC attitude tilt.
// Records MAVLink ATTITUDE and ATTITUDE_QUATERNION simultaneously.

#define main jtzero_camera_imu_logger_unused_main_v1531
#include "camera_imu_extrinsics_logger.cpp"
#undef main

#include <array>

namespace {

constexpr const char* WIN1531 = "JT-Zero: диагностика ориентации v15.31";
constexpr const char* OUT_EULER1531 = "/home/vio/jtzero_attitude_v15_31.csv";
constexpr const char* OUT_QUAT1531 = "/home/vio/jtzero_quaternion_v15_31.csv";
constexpr const char* OUT_PHASES1531 = "/home/vio/jtzero_attitude_phases_v15_31.csv";
constexpr double ZERO_SEC1531 = 2.0;
constexpr double RP_TOL1531 = 2.0;
constexpr double TILT_TOL1531 = 2.0;
constexpr double YAW_TOL1531 = 8.0;

struct Phase1531 {
  const char* name;
  double duration;
  double target_yaw;
  const char* instruction;
};

constexpr Phase1531 PHASES1531[] = {
  {"STILL0", 5.0, 0.0, "НЕ ДВИГАТЬ"},
  {"YAW_90", 15.0, 90.0, "ПЛАВНО ПОВЕРНИ YAW ДО +90°"},
  {"STILL90", 5.0, 90.0, "НЕ ДВИГАТЬ"},
  {"YAW_RETURN", 15.0, 0.0, "ПЛАВНО ВЕРНИ YAW В 0°"},
  {"STILL_END", 5.0, 0.0, "НЕ ДВИГАТЬ"},
};
constexpr size_t NPH1531 = sizeof(PHASES1531)/sizeof(PHASES1531[0]);

struct Euler1531 {
  int64_t recv_ns=0, src_ns=0;
  double r=0,p=0,y=0,rs=0,ps=0,ys=0;
};
struct Quat1531 {
  int64_t recv_ns=0, src_ns=0;
  double w=1,x=0,y=0,z=0,rs=0,ps=0,ys=0;
};

struct Q1531 { double w=1,x=0,y=0,z=0; };
Q1531 qnorm1531(Q1531 q){
  double n=std::sqrt(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z);
  if(n<=1e-12) return {1,0,0,0};
  q.w/=n;q.x/=n;q.y/=n;q.z/=n;return q;
}
Q1531 qconj1531(Q1531 q){return {q.w,-q.x,-q.y,-q.z};}
Q1531 qmul1531(Q1531 a,Q1531 b){
  return {a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z,
          a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,
          a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,
          a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w};
}
std::array<double,3> bodyZ1531(Q1531 q){
  q=qnorm1531(q);
  // Third column of body->earth rotation matrix.
  return {2.0*(q.x*q.z+q.w*q.y),
          2.0*(q.y*q.z-q.w*q.x),
          1.0-2.0*(q.x*q.x+q.y*q.y)};
}
double tiltBetween1531(Q1531 a,Q1531 b){
  auto za=bodyZ1531(a), zb=bodyZ1531(b);
  double d=za[0]*zb[0]+za[1]*zb[1]+za[2]*zb[2];
  d=std::clamp(d,-1.0,1.0);
  return rad2deg(std::acos(d));
}
void quatEuler1531(Q1531 q,double& roll,double& pitch,double& yaw){
  q=qnorm1531(q);
  const double sr=2*(q.w*q.x+q.y*q.z);
  const double cr=1-2*(q.x*q.x+q.y*q.y);
  roll=rad2deg(std::atan2(sr,cr));
  double sp=2*(q.w*q.y-q.z*q.x); sp=std::clamp(sp,-1.0,1.0);
  pitch=rad2deg(std::asin(sp));
  const double sy=2*(q.w*q.z+q.x*q.y);
  const double cy=1-2*(q.y*q.y+q.z*q.z);
  yaw=rad2deg(std::atan2(sy,cy));
}

double totalSec1531(){double s=0;for(auto&p:PHASES1531)s+=p.duration;return s;}
size_t phaseIdx1531(double elapsed,double* remain=nullptr){
  double t=0;
  for(size_t i=0;i<NPH1531;++i){double e=t+PHASES1531[i].duration;if(elapsed<e){if(remain)*remain=e-elapsed;return i;}t=e;}
  if(remain)*remain=0;return NPH1531-1;
}
void text1531(cv::Mat& im,const std::string& s,cv::Point p,cv::Scalar c=cv::Scalar(255,255,255),int h=22){
  cv::addText(im,s,p,"DejaVu Sans",h,c,cv::QT_FONT_NORMAL,cv::QT_STYLE_NORMAL,0);
}
int gx1531(double v,double lo,double hi,int x,int w){double u=(v-lo)/(hi-lo);u=std::clamp(u,0.0,1.0);return x+(int)std::lround(u*w);}
void gauge1531(cv::Mat& im,int x,int y,int w,const std::string& name,double v,double target,double tol,double lo,double hi){
  cv::rectangle(im,{x,y,w,28},{65,65,65},cv::FILLED);
  int a=gx1531(target-tol,lo,hi,x,w),b=gx1531(target+tol,lo,hi,x,w);if(b<a)std::swap(a,b);
  cv::rectangle(im,{a,y,std::max(1,b-a),28},{0,140,0},cv::FILLED);
  int px=gx1531(v,lo,hi,x,w),pt=gx1531(target,lo,hi,x,w);
  cv::line(im,{pt,y-5},{pt,y+33},{0,255,255},2);
  bool ok=std::abs(wrap180(v-target))<=tol;
  cv::line(im,{px,y-8},{px,y+36},ok?cv::Scalar(0,255,0):cv::Scalar(0,0,255),4,cv::LINE_AA);
  std::ostringstream ss;ss<<name<<": "<<std::fixed<<std::setprecision(2)<<v<<"°";text1531(im,ss.str(),{x,y-12},ok?cv::Scalar(0,255,0):cv::Scalar(0,0,255),20);
}

void draw1531(cv::Mat& im,double elapsed,bool zero_ready,bool have_e,bool have_q,
              double rr,double rp,double ry,double qrr,double qrp,double qry,double qtilt){
  im.setTo(cv::Scalar(18,18,18));
  text1531(im,"JT-ZERO — ДИАГНОСТИКА ATTITUDE vs QUATERNION v15.31",{35,45},{255,255,255},24);
  if(!zero_ready){
    text1531(im,"КАЛИБРОВКА НУЛЯ 2 С: БПЛА НЕПОДВИЖЕН",{35,90},{0,220,255},22);
  }else{
    double rem=0;size_t pi=phaseIdx1531(elapsed,&rem);const auto& ph=PHASES1531[pi];
    std::ostringstream a;a<<"ФАЗА "<<(pi+1)<<"/"<<NPH1531<<": "<<ph.name<<"   Осталось: "<<std::fixed<<std::setprecision(1)<<rem<<" с";
    text1531(im,a.str(),{35,90},{0,220,255},22);
    text1531(im,ph.instruction,{35,130},ph.name==std::string("STILL0")||ph.name==std::string("STILL90")||ph.name==std::string("STILL_END")?cv::Scalar(0,255,0):cv::Scalar(0,220,255),22);

    text1531(im,"MAVLink ATTITUDE (Euler)",{50,190},{255,255,255},21);
    gauge1531(im,50,240,520,"ROLL",rr,0,RP_TOL1531,-12,12);
    gauge1531(im,50,330,520,"PITCH",rp,0,RP_TOL1531,-12,12);
    gauge1531(im,50,420,520,"YAW",ry,ph.target_yaw,YAW_TOL1531,-120,120);

    text1531(im,"ATTITUDE_QUATERNION",{690,190},{255,255,255},21);
    gauge1531(im,690,240,520,"Q-ROLL",qrr,0,RP_TOL1531,-12,12);
    gauge1531(im,690,330,520,"Q-PITCH",qrp,0,RP_TOL1531,-12,12);
    gauge1531(im,690,420,520,"Q-YAW",qry,ph.target_yaw,YAW_TOL1531,-120,120);
    gauge1531(im,690,520,520,"BODY-Z TILT",qtilt,0,TILT_TOL1531,0,12);

    bool e_ok=std::abs(rr)<=RP_TOL1531&&std::abs(rp)<=RP_TOL1531;
    bool q_ok=qtilt<=TILT_TOL1531;
    text1531(im,e_ok?"Euler R/P: В НОРМЕ":"Euler R/P: ИЗМЕНЯЮТСЯ",{50,560},e_ok?cv::Scalar(0,255,0):cv::Scalar(0,0,255),21);
    text1531(im,q_ok?"Body-Z: НАКЛОНА НЕТ":"Body-Z: FC ВИДИТ НАКЛОН",{690,610},q_ok?cv::Scalar(0,255,0):cv::Scalar(0,0,255),21);
    std::ostringstream tt;tt<<"Время: "<<std::fixed<<std::setprecision(1)<<elapsed<<" / "<<totalSec1531()<<" с";text1531(im,tt.str(),{50,650},{180,180,180},19);
  }
  if(!have_e)text1531(im,"НЕТ ATTITUDE",{50,700},{0,0,255},20);
  if(!have_q)text1531(im,"НЕТ ATTITUDE_QUATERNION",{690,700},{0,0,255},20);
  text1531(im,"ESC — аварийно завершить",{940,45},{180,180,180},18);
}

} // namespace

int main(){
  int sfd=-1;uint8_t sys=0,comp=0;bool rates=false;
  try{
    sfd=openSerial();
    std::cout<<"[MAV] waiting for HEARTBEAT...\n";
    mavlink_status_t ms{};mavlink_message_t mm{};int64_t deadline=monotonicNs()+10000000000LL;
    while(monotonicNs()<deadline&&!sys){pollfd p{sfd,POLLIN,0};if(poll(&p,1,100)>0){uint8_t b[2048];ssize_t n=read(sfd,b,sizeof(b));for(ssize_t i=0;i<n;++i)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&mm,&ms)&&mm.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=mm.sysid;comp=mm.compid;break;}}}
    if(!sys)throw std::runtime_error("HEARTBEAT timeout");
    requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,50);
    requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE_QUATERNION,50);
    rates=true;

    std::vector<Euler1531> es;std::vector<Quat1531> qs;es.reserve(5000);qs.reserve(5000);
    bool have_e=false,have_q=false,zero_ready=false,abort=false;
    double er0=0,ep0=0,ey0=0,sum_er=0,sum_ep=0,sum_sy=0,sum_cy=0;size_t en0=0;
    Q1531 q0{};double sum_qw=0,sum_qx=0,sum_qy=0,sum_qz=0;size_t qn0=0;
    int64_t zero_start=0,test_start=0,last_gui=0;
    cv::namedWindow(WIN1531,cv::WINDOW_NORMAL);cv::setWindowProperty(WIN1531,cv::WND_PROP_FULLSCREEN,cv::WINDOW_FULLSCREEN);cv::Mat screen(720,1280,CV_8UC3);
    tcflush(sfd,TCIFLUSH);std::memset(&ms,0,sizeof(ms));std::memset(&mm,0,sizeof(mm));

    while(!abort){
      int64_t now=monotonicNs();double elapsed=zero_ready?(now-test_start)*1e-9:0.0;if(zero_ready&&elapsed>=totalSec1531())break;
      pollfd p{sfd,POLLIN,0};int rc=poll(&p,1,5);if(rc<0){if(errno==EINTR)continue;fail("poll");}
      if(p.revents&POLLIN){uint8_t b[8192];for(;;){ssize_t n=read(sfd,b,sizeof(b));if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;for(ssize_t i=0;i<n;++i){if(!mavlink_parse_char(MAVLINK_COMM_0,b[i],&mm,&ms))continue;int64_t recv=monotonicNs();
        if(mm.msgid==MAVLINK_MSG_ID_ATTITUDE){mavlink_attitude_t a{};mavlink_msg_attitude_decode(&mm,&a);double r=rad2deg(a.roll),pp=rad2deg(a.pitch),y=rad2deg(a.yaw);es.push_back({recv,(int64_t)a.time_boot_ms*1000000LL,r,pp,y,a.rollspeed,a.pitchspeed,a.yawspeed});have_e=true;if(!zero_ready){if(!zero_start)zero_start=recv;sum_er+=r;sum_ep+=pp;sum_sy+=std::sin(a.yaw);sum_cy+=std::cos(a.yaw);++en0;}}
        else if(mm.msgid==MAVLINK_MSG_ID_ATTITUDE_QUATERNION){mavlink_attitude_quaternion_t a{};mavlink_msg_attitude_quaternion_decode(&mm,&a);Q1531 q=qnorm1531({a.q1,a.q2,a.q3,a.q4});qs.push_back({recv,(int64_t)a.time_boot_ms*1000000LL,q.w,q.x,q.y,q.z,a.rollspeed,a.pitchspeed,a.yawspeed});have_q=true;if(!zero_ready){sum_qw+=q.w;sum_qx+=q.x;sum_qy+=q.y;sum_qz+=q.z;++qn0;}}
      }}}
      if(!zero_ready&&zero_start&&have_e&&have_q&&(monotonicNs()-zero_start)*1e-9>=ZERO_SEC1531&&en0>20&&qn0>20){
        er0=sum_er/en0;ep0=sum_ep/en0;ey0=rad2deg(std::atan2(sum_sy,sum_cy));q0=qnorm1531({sum_qw/qn0,sum_qx/qn0,sum_qy/qn0,sum_qz/qn0});zero_ready=true;test_start=monotonicNs();std::cout<<"[ZERO] Euler="<<er0<<","<<ep0<<","<<ey0<<" q="<<q0.w<<","<<q0.x<<","<<q0.y<<","<<q0.z<<"\n";
      }
      now=monotonicNs();if(now-last_gui>33000000LL){last_gui=now;double rr=0,rp=0,ry=0,qrr=0,qrp=0,qry=0,tilt=0;if(zero_ready&&have_e&&!es.empty()){rr=wrap180(es.back().r-er0);rp=wrap180(es.back().p-ep0);ry=wrap180(es.back().y-ey0);}if(zero_ready&&have_q&&!qs.empty()){Q1531 q{qs.back().w,qs.back().x,qs.back().y,qs.back().z};Q1531 qr=qnorm1531(qmul1531(qconj1531(q0),q));quatEuler1531(qr,qrr,qrp,qry);tilt=tiltBetween1531(q0,q);}draw1531(screen,zero_ready?(now-test_start)*1e-9:0.0,zero_ready,have_e,have_q,rr,rp,ry,qrr,qrp,qry,tilt);cv::imshow(WIN1531,screen);if(cv::waitKeyEx(1)==27)abort=true;}
    }

    std::ofstream ef(OUT_EULER1531,std::ios::trunc);ef<<"recv_rpi_ns,source_timestamp_ns,roll_deg,pitch_deg,yaw_deg,rel_roll_deg,rel_pitch_deg,rel_yaw_deg,rollspeed,pitchspeed,yawspeed\n"<<std::fixed<<std::setprecision(9);for(auto&s:es)ef<<s.recv_ns<<','<<s.src_ns<<','<<s.r<<','<<s.p<<','<<s.y<<','<<wrap180(s.r-er0)<<','<<wrap180(s.p-ep0)<<','<<wrap180(s.y-ey0)<<','<<s.rs<<','<<s.ps<<','<<s.ys<<'\n';ef.flush();ef.close();
    std::ofstream qf(OUT_QUAT1531,std::ios::trunc);qf<<"recv_rpi_ns,source_timestamp_ns,q1_w,q2_x,q3_y,q4_z,rel_roll_deg,rel_pitch_deg,rel_yaw_deg,body_z_tilt_deg,rollspeed,pitchspeed,yawspeed\n"<<std::fixed<<std::setprecision(9);for(auto&s:qs){Q1531 q{s.w,s.x,s.y,s.z};Q1531 qr=qnorm1531(qmul1531(qconj1531(q0),q));double r,p,y;quatEuler1531(qr,r,p,y);qf<<s.recv_ns<<','<<s.src_ns<<','<<s.w<<','<<s.x<<','<<s.y<<','<<s.z<<','<<r<<','<<p<<','<<y<<','<<tiltBetween1531(q0,q)<<','<<s.rs<<','<<s.ps<<','<<s.ys<<'\n';}qf.flush();qf.close();
    std::ofstream pf(OUT_PHASES1531,std::ios::trunc);pf<<"phase,start_rpi_ns,end_rpi_ns,target_yaw_deg\n";double t=0;for(auto&p:PHASES1531){int64_t a=test_start+(int64_t)std::llround(t*1e9);t+=p.duration;int64_t b=test_start+(int64_t)std::llround(t*1e9);pf<<p.name<<','<<a<<','<<b<<','<<p.target_yaw<<'\n';}pf.flush();pf.close();

    if(rates&&sfd>=0&&sys){requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE_QUATERNION,0);}if(sfd>=0)close(sfd);cv::destroyAllWindows();
    std::cout<<"\n================ V15.31 RESULT ================\nEuler rows: "<<es.size()<<"\nQuaternion rows: "<<qs.size()<<"\nEuler CSV: "<<OUT_EULER1531<<"\nQuaternion CSV: "<<OUT_QUAT1531<<"\nPhases: "<<OUT_PHASES1531<<"\nRESULT: "<<(abort?"ABORTED":"PASS")<<"\n";
    return abort?2:0;
  }catch(const std::exception& e){std::cerr<<"[FATAL] "<<e.what()<<"\n";if(rates&&sfd>=0&&sys){try{requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);requestRate(sfd,sys,comp,MAVLINK_MSG_ID_ATTITUDE_QUATERNION,0);}catch(...){}}if(sfd>=0)close(sfd);cv::destroyAllWindows();return 1;}
}
