// JT-ZERO v39: comprehensive root-cause sweep for yaw-induced false horizontal velocity.
// No Kimera backend. Uses HIGHRES_IMU + ATTITUDE only.
// Event-driven mechanical route: 0 -> right -> 0 -> left -> 0.
// Tests: gyro cross-axis/rate coupling, bias changes, dt/gaps, accel orientation dependence,
// FC-vs-ACC tilt, Z gyro scale proxy, dynamic lever/sculling signature, and integration-method sensitivity.
#define JTZERO_V10_NO_MAIN
#include "live_mono_imu_integrator_v10.cpp"
#undef JTZERO_V10_NO_MAIN

#include <array>
#include <fstream>
#include <iomanip>
#include <numeric>

namespace jtzero_v39 {
using namespace jtzero_v10;

constexpr const char* kCsv39 = "/home/vio/jtzero_live_imu_root_cause_sweep_v39.csv";
constexpr const char* kWindow39 = "JT-ZERO: ПОИСК ПРИЧИНЫ IMU v39";
constexpr double kG = 9.81;
constexpr double kGyroStill = 0.035;
constexpr double kAccTol = 0.45;
constexpr int kCalN = 600;
constexpr int kStableN = 120;
constexpr int kStaticN = 1000;

enum class Phase {
  WAIT_ZERO=0, CALIBRATE,
  STATIC_0_A, ROT_R, STATIC_R,
  RET_0_A, STATIC_0_B,
  ROT_L, STATIC_L,
  RET_0_B, STATIC_0_C,
  DONE
};

constexpr int kM = 9;
constexpr const char* kNames[kM] = {
  "STATIC_0_A","ROT_R","STATIC_R","RET_0_A","STATIC_0_B",
  "ROT_L","STATIC_L","RET_0_B","STATIC_0_C"
};

struct Att {
  bool valid=false;
  double roll=0,pitch=0,yaw=0;
  double rs=0,ps=0,ys=0;
};

struct LinReg {
  int n=0;
  double sx=0,sy=0,sxx=0,syy=0,sxy=0;
  void add(double x,double y){++n;sx+=x;sy+=y;sxx+=x*x;syy+=y*y;sxy+=x*y;}
  double slope()const{double d=n*sxx-sx*sx;return std::abs(d)>1e-18?(n*sxy-sx*sy)/d:0;}
  double intercept()const{return n?(sy-slope()*sx)/n:0;}
  double corr()const{double dx=n*sxx-sx*sx,dy=n*syy-sy*sy;double d=dx*dy;return d>1e-24?(n*sxy-sx*sy)/std::sqrt(d):0;}
};

struct Stats {
  int n=0; double dt=0;
  Eigen::Vector3d sum_acc=Eigen::Vector3d::Zero();
  Eigen::Vector3d sum_gyr=Eigen::Vector3d::Zero();
  Eigen::Vector3d sum_fc_rates=Eigen::Vector3d::Zero();
  Eigen::Vector3d dv_euler=Eigen::Vector3d::Zero();
  Eigen::Vector3d dv_mid=Eigen::Vector3d::Zero();
  double sum_axy_euler=0, sum_axy_mid=0, max_axy_euler=0, max_axy_mid=0;
  double max_dt=0; int bad_dt=0;
  LinReg gx_gz, gy_gz, gx_gz2, gy_gz2;
  LinReg gz_fcys;
  LinReg gx_ax, gx_ay, gy_ax, gy_ay;
};

struct Row {
  uint64_t us=0; int phase=-1; double dt=0;
  Eigen::Vector3d acc=Eigen::Vector3d::Zero();
  Eigen::Vector3d gyr=Eigen::Vector3d::Zero();
  Eigen::Vector3d fc_rates=Eigen::Vector3d::Zero();
  double fc_roll=0,fc_pitch=0,fc_yaw=0;
  double acc_roll=0,acc_pitch=0;
  Eigen::Vector3d a_euler=Eigen::Vector3d::Zero();
  Eigen::Vector3d a_mid=Eigen::Vector3d::Zero();
  Eigen::Vector3d v_euler=Eigen::Vector3d::Zero();
  Eigen::Vector3d v_mid=Eigen::Vector3d::Zero();
  double grav_err_euler=0,grav_err_mid=0,fc_acc_tilt_err=0;
};

struct State {
  Phase phase=Phase::WAIT_ZERO;
  uint64_t last_us=0;
  int cal_n=0,stable_n=0,static_n=0;
  Eigen::Vector3d cal_as=Eigen::Vector3d::Zero(),cal_gs=Eigen::Vector3d::Zero();
  Eigen::Vector3d BA=Eigen::Vector3d::Zero(),BG=Eigen::Vector3d::Zero();
  Eigen::Matrix3d R_euler=Eigen::Matrix3d::Identity();
  Eigen::Matrix3d R_mid=Eigen::Matrix3d::Identity();
  Eigen::Vector3d V_euler=Eigen::Vector3d::Zero(),V_mid=Eigen::Vector3d::Zero();
  Eigen::Vector3d prev_wc=Eigen::Vector3d::Zero(),prev_ac=Eigen::Vector3d::Zero();
  bool have_prev=false;
  std::array<Stats,kM> st;
  std::vector<Row> rows;
};

static double clamp1(double x){return std::max(-1.0,std::min(1.0,x));}
static bool still(const Eigen::Vector3d&a,const Eigen::Vector3d&w){return w.norm()<=kGyroStill&&std::abs(a.norm()-kG)<=kAccTol;}
static Eigen::Matrix3d expR(const Eigen::Vector3d&th){double q=th.norm();return q>1e-12?Eigen::AngleAxisd(q,th/q).toRotationMatrix():Eigen::Matrix3d::Identity();}
static void accelTilt(const Eigen::Vector3d&a,double&r,double&p){Eigen::Vector3d u=a.normalized();r=std::atan2(u.y(),u.z())*180.0/kPi;double pr=std::atan2(-u.x(),std::hypot(u.y(),u.z()))*180.0/kPi;p=-pr;}
static Eigen::Matrix3d Rfc(double r,double p,double y){return fcRnedFlu(r,p,y);}
static double gravityErr(const Eigen::Matrix3d&R,const Eigen::Vector3d&a){Eigen::Vector3d ua=a.normalized();Eigen::Vector3d pred=R.transpose()*Eigen::Vector3d(0,0,-1);return std::acos(clamp1(ua.dot(pred)))*180.0/kPi;}
static int idx(Phase p){switch(p){case Phase::STATIC_0_A:return 0;case Phase::ROT_R:return 1;case Phase::STATIC_R:return 2;case Phase::RET_0_A:return 3;case Phase::STATIC_0_B:return 4;case Phase::ROT_L:return 5;case Phase::STATIC_L:return 6;case Phase::RET_0_B:return 7;case Phase::STATIC_0_C:return 8;default:return -1;}}
static const char* label(Phase p){switch(p){case Phase::WAIT_ZERO:return "ПОСТАВЬТЕ СТЕНД В ФИЗИЧЕСКИЙ НОЛЬ";case Phase::CALIBRATE:return "КАЛИБРОВКА — НЕ ДВИГАТЬ";case Phase::STATIC_0_A:return "ИСХОДНАЯ СТАТИКА — НЕ ДВИГАТЬ";case Phase::ROT_R:return "ПОВЕРНИТЕ ВПРАВО НА 60–100°, ОСТАНОВИТЕСЬ";case Phase::STATIC_R:return "СТАТИКА СПРАВА — НЕ ДВИГАТЬ";case Phase::RET_0_A:return "ВЕРНИТЕ ТОЧНО В ФИЗИЧЕСКИЙ НОЛЬ";case Phase::STATIC_0_B:return "СТАТИКА В НУЛЕ — НЕ ДВИГАТЬ";case Phase::ROT_L:return "ПОВЕРНИТЕ ВЛЕВО НА 60–100°, ОСТАНОВИТЕСЬ";case Phase::STATIC_L:return "СТАТИКА СЛЕВА — НЕ ДВИГАТЬ";case Phase::RET_0_B:return "ВЕРНИТЕ ТОЧНО В ФИЗИЧЕСКИЙ НОЛЬ";case Phase::STATIC_0_C:return "ФИНАЛЬНАЯ СТАТИКА — НЕ ДВИГАТЬ";case Phase::DONE:return "ТЕСТ ЗАВЕРШЁН";}return "";}

static void beginAfterCal(State&s,const Att&a){
  Eigen::Vector3d ma=s.cal_as/double(s.cal_n);s.BG=s.cal_gs/double(s.cal_n);
  Eigen::Matrix3d R0=Rfc(a.roll,a.pitch,a.yaw);
  s.BA=ma-R0.transpose()*Eigen::Vector3d(0,0,-kG);
  s.R_euler=R0;s.R_mid=R0;s.V_euler.setZero();s.V_mid.setZero();s.have_prev=false;s.last_us=0;s.static_n=0;s.stable_n=0;s.phase=Phase::STATIC_0_A;
  std::cout<<"[CAL] BA=["<<s.BA.transpose()<<"] BG=["<<s.BG.transpose()<<"]\n";
}

static void addStats(Stats&z,double dt,const Eigen::Vector3d&a,const Eigen::Vector3d&w,const Eigen::Vector3d&fr,const Eigen::Vector3d&ae,const Eigen::Vector3d&am){
  ++z.n;z.dt+=dt;z.sum_acc+=a;z.sum_gyr+=w;z.sum_fc_rates+=fr;z.dv_euler+=ae*dt;z.dv_mid+=am*dt;
  double xe=std::hypot(ae.x(),ae.y()),xm=std::hypot(am.x(),am.y());z.sum_axy_euler+=xe;z.sum_axy_mid+=xm;z.max_axy_euler=std::max(z.max_axy_euler,xe);z.max_axy_mid=std::max(z.max_axy_mid,xm);z.max_dt=std::max(z.max_dt,dt);if(dt>0.008||dt<0.002)++z.bad_dt;
  double gz=w.z();z.gx_gz.add(gz,w.x());z.gy_gz.add(gz,w.y());z.gx_gz2.add(gz*std::abs(gz),w.x());z.gy_gz2.add(gz*std::abs(gz),w.y());z.gz_fcys.add(fr.z(),gz);
  z.gx_ax.add(a.x(),w.x());z.gx_ay.add(a.y(),w.x());z.gy_ax.add(a.x(),w.y());z.gy_ay.add(a.y(),w.y());
}

static void process(State&s,const Att&att,uint64_t us,const Eigen::Vector3d&rawa,const Eigen::Vector3d&raww){
  bool isstill=still(rawa,raww);
  if(s.phase==Phase::CALIBRATE){if(isstill&&att.valid){s.cal_as+=rawa;s.cal_gs+=raww;if(++s.cal_n>=kCalN)beginAfterCal(s,att);}else{s.cal_n=0;s.cal_as.setZero();s.cal_gs.setZero();}return;}
  if(s.phase==Phase::WAIT_ZERO||s.phase==Phase::DONE||!att.valid)return;
  double dt=0;if(s.last_us&&us>s.last_us)dt=(us-s.last_us)*1e-6;s.last_us=us;if(dt<=0||dt>.03)return;
  Eigen::Vector3d wc=raww-s.BG,ac=rawa-s.BA;
  Eigen::Matrix3d Rprev=s.R_euler;
  s.R_euler=s.R_euler*expR(wc*dt);
  Eigen::Vector3d wmid=s.have_prev?0.5*(s.prev_wc+wc):wc;
  Eigen::Vector3d amid=s.have_prev?0.5*(s.prev_ac+ac):ac;
  Eigen::Matrix3d Rmid0=s.R_mid;
  s.R_mid=s.R_mid*expR(wmid*dt);
  Eigen::Matrix3d ReHalf=Rprev*expR(wc*(0.5*dt));
  Eigen::Matrix3d RmHalf=Rmid0*expR(wmid*(0.5*dt));
  Eigen::Vector3d ae=ReHalf*ac+Eigen::Vector3d(0,0,kG);
  Eigen::Vector3d am=RmHalf*amid+Eigen::Vector3d(0,0,kG);
  s.V_euler+=ae*dt;s.V_mid+=am*dt;
  Eigen::Vector3d fr(att.rs,-att.ps,-att.ys);
  int ii=idx(s.phase);if(ii>=0)addStats(s.st[ii],dt,rawa,wc,fr,ae,am);
  double ar=0,ap=0;accelTilt(rawa,ar,ap);double fcerr=std::hypot(att.roll-ar,att.pitch-ap);
  Row r;r.us=us;r.phase=ii;r.dt=dt;r.acc=rawa;r.gyr=wc;r.fc_rates=fr;r.fc_roll=att.roll;r.fc_pitch=att.pitch;r.fc_yaw=att.yaw;r.acc_roll=ar;r.acc_pitch=ap;r.a_euler=ae;r.a_mid=am;r.v_euler=s.V_euler;r.v_mid=s.V_mid;r.grav_err_euler=gravityErr(s.R_euler,rawa);r.grav_err_mid=gravityErr(s.R_mid,rawa);r.fc_acc_tilt_err=fcerr;s.rows.push_back(r);
  s.prev_wc=wc;s.prev_ac=ac;s.have_prev=true;
  if(isstill)++s.stable_n;else s.stable_n=0;
  if(s.phase==Phase::STATIC_0_A||s.phase==Phase::STATIC_R||s.phase==Phase::STATIC_0_B||s.phase==Phase::STATIC_L||s.phase==Phase::STATIC_0_C){
    if(isstill)++s.static_n;else s.static_n=0;
    if(s.static_n>=kStaticN){s.static_n=0;s.stable_n=0;if(s.phase==Phase::STATIC_0_A){s.phase=Phase::ROT_R;std::cout<<"[STEP] baseline done. Rotate right, stop, SPACE.\n";}else if(s.phase==Phase::STATIC_R){s.phase=Phase::RET_0_A;std::cout<<"[STEP] right static done. Return to zero, stop, SPACE.\n";}else if(s.phase==Phase::STATIC_0_B){s.phase=Phase::ROT_L;std::cout<<"[STEP] zero static done. Rotate left, stop, SPACE.\n";}else if(s.phase==Phase::STATIC_L){s.phase=Phase::RET_0_B;std::cout<<"[STEP] left static done. Return to zero, stop, SPACE.\n";}else{s.phase=Phase::DONE;std::cout<<"[TEST] final static done.\n";}}}
}

static void save(const State&s){std::ofstream f(kCsv39);f<<std::fixed<<std::setprecision(9)<<"imu_us,phase,phase_name,dt,ax,ay,az,gx_corr,gy_corr,gz_corr,fc_rollspeed_flu,fc_pitchspeed_flu,fc_yawspeed_flu,fc_roll,fc_pitch,fc_yaw,acc_roll,acc_pitch,a_euler_x,a_euler_y,a_euler_z,a_mid_x,a_mid_y,a_mid_z,v_euler_x,v_euler_y,v_euler_z,v_mid_x,v_mid_y,v_mid_z,gravity_err_euler_deg,gravity_err_mid_deg,fc_acc_tilt_err_deg\n";for(const auto&r:s.rows){const char*n=r.phase>=0&&r.phase<kM?kNames[r.phase]:"OTHER";f<<r.us<<','<<r.phase<<','<<n<<','<<r.dt<<','<<r.acc.x()<<','<<r.acc.y()<<','<<r.acc.z()<<','<<r.gyr.x()<<','<<r.gyr.y()<<','<<r.gyr.z()<<','<<r.fc_rates.x()<<','<<r.fc_rates.y()<<','<<r.fc_rates.z()<<','<<r.fc_roll<<','<<r.fc_pitch<<','<<r.fc_yaw<<','<<r.acc_roll<<','<<r.acc_pitch<<','<<r.a_euler.x()<<','<<r.a_euler.y()<<','<<r.a_euler.z()<<','<<r.a_mid.x()<<','<<r.a_mid.y()<<','<<r.a_mid.z()<<','<<r.v_euler.x()<<','<<r.v_euler.y()<<','<<r.v_euler.z()<<','<<r.v_mid.x()<<','<<r.v_mid.y()<<','<<r.v_mid.z()<<','<<r.grav_err_euler<<','<<r.grav_err_mid<<','<<r.fc_acc_tilt_err<<'\n';}}

static Eigen::Vector3d mean(const Eigen::Vector3d&s,int n){return n?s/double(n):Eigen::Vector3d::Zero();}
static double tiltDelta(const Eigen::Vector3d&a,const Eigen::Vector3d&b){Eigen::Vector3d ua=a.normalized(),ub=b.normalized();return std::acos(clamp1(ua.dot(ub)))*180.0/kPi;}

static void summary(const State&s){
  std::cout<<std::fixed<<std::setprecision(6)<<"\n============================================================\nJT-ZERO IMU ROOT-CAUSE SWEEP v39 RESULT\n============================================================\nBA=["<<s.BA.transpose()<<"] m/s^2\nBG=["<<s.BG.transpose()<<"] rad/s\n";
  for(int i=0;i<kM;i++){const Stats&z=s.st[i];Eigen::Vector3d ma=mean(z.sum_acc,z.n),mw=mean(z.sum_gyr,z.n);std::cout<<"\n"<<kNames[i]<<" n="<<z.n<<" dt="<<z.dt<<" s max_dt="<<z.max_dt<<" bad_dt="<<z.bad_dt<<"\n mean acc=["<<ma.transpose()<<"] mean gyro_corr=["<<mw.transpose()<<"]\n Euler mean/max axy="<<(z.n?z.sum_axy_euler/z.n:0)<<"/"<<z.max_axy_euler<<" dVxy="<<std::hypot(z.dv_euler.x(),z.dv_euler.y())<<"\n Midpt mean/max axy="<<(z.n?z.sum_axy_mid/z.n:0)<<"/"<<z.max_axy_mid<<" dVxy="<<std::hypot(z.dv_mid.x(),z.dv_mid.y())<<"\n gx~gz K="<<z.gx_gz.slope()<<" corr="<<z.gx_gz.corr()<<"  gy~gz K="<<z.gy_gz.slope()<<" corr="<<z.gy_gz.corr()<<"\n gx~gz|gz| K="<<z.gx_gz2.slope()<<" corr="<<z.gx_gz2.corr()<<"  gy~gz|gz| K="<<z.gy_gz2.slope()<<" corr="<<z.gy_gz2.corr()<<"\n gz vs FC yawspeed: scale="<<z.gz_fcys.slope()<<" corr="<<z.gz_fcys.corr()<<"\n";}
  Eigen::Vector3d a0=mean(s.st[0].sum_acc,s.st[0].n),ar=mean(s.st[2].sum_acc,s.st[2].n),a0b=mean(s.st[4].sum_acc,s.st[4].n),al=mean(s.st[6].sum_acc,s.st[6].n),a0c=mean(s.st[8].sum_acc,s.st[8].n);
  Eigen::Vector3d g0=mean(s.st[0].sum_gyr,s.st[0].n),gr=mean(s.st[2].sum_gyr,s.st[2].n),g0b=mean(s.st[4].sum_gyr,s.st[4].n),gl=mean(s.st[6].sum_gyr,s.st[6].n),g0c=mean(s.st[8].sum_gyr,s.st[8].n);
  std::cout<<"\n---------------- CANDIDATE CHECKS ----------------\n";
  std::cout<<"Accel gravity direction change 0A->R="<<tiltDelta(a0,ar)<<" deg  0A->L="<<tiltDelta(a0,al)<<" deg  0A->0C="<<tiltDelta(a0,a0c)<<" deg\n";
  std::cout<<"Static gyro bias shift |R-0A|="<<(gr-g0).norm()<<" rad/s  |L-0A|="<<(gl-g0).norm()<<"  |0C-0A|="<<(g0c-g0).norm()<<"\n";
  double kxr=s.st[1].gx_gz.slope(),kyr=s.st[1].gy_gz.slope(),kxl=s.st[5].gx_gz.slope(),kyl=s.st[5].gy_gz.slope();
  std::cout<<"Cross-axis signed: ROT_R Kxz="<<kxr<<" Kyz="<<kyr<<" ; ROT_L Kxz="<<kxl<<" Kyz="<<kyl<<"\n";
  double cross=std::max({std::abs(kxr),std::abs(kyr),std::abs(kxl),std::abs(kyl)});
  double eul=std::hypot(s.st[1].dv_euler.x()+s.st[3].dv_euler.x()+s.st[5].dv_euler.x()+s.st[7].dv_euler.x(),s.st[1].dv_euler.y()+s.st[3].dv_euler.y()+s.st[5].dv_euler.y()+s.st[7].dv_euler.y());
  double mid=std::hypot(s.st[1].dv_mid.x()+s.st[3].dv_mid.x()+s.st[5].dv_mid.x()+s.st[7].dv_mid.x(),s.st[1].dv_mid.y()+s.st[3].dv_mid.y()+s.st[5].dv_mid.y()+s.st[7].dv_mid.y());
  std::cout<<"Dynamic-only combined dVxy Euler="<<eul<<" Midpoint="<<mid<<" ratio="<<(eul>1e-9?mid/eul:0)<<"\n";
  std::cout<<"\nRANKING HELPER:\n";
  if(cross>0.01)std::cout<<" HIGH: gyro cross-axis/rate coupling is large enough to be a primary suspect.\n";else std::cout<<" LOW/MED: linear gyro cross-axis coefficient is small.\n";
  if((gr-g0).norm()>0.002||(gl-g0).norm()>0.002)std::cout<<" HIGH: orientation-dependent/static gyro bias shift detected.\n";else std::cout<<" LOW: no large static gyro bias shift by yaw position.\n";
  if(mid<0.7*eul)std::cout<<" HIGH/MED: integration discretization/coning materially changes dynamic error.\n";else std::cout<<" LOW: midpoint integration does not materially cure dynamic error.\n";
  if(tiltDelta(a0,a0c)>0.5)std::cout<<" MED/HIGH: accelerometer gravity direction failed to return; mechanical/accel repeatability issue.\n";else std::cout<<" LOW: accelerometer gravity direction returns well at physical zero.\n";
  int bad=0;for(const auto&z:s.st)bad+=z.bad_dt;if(bad>20)std::cout<<" MED/HIGH: many abnormal IMU dt samples; inspect transport/timestamps.\n";else std::cout<<" LOW: IMU dt continuity is not a major suspect.\n";
  std::cout<<"CSV: "<<kCsv39<<"\nOpen CSV:\n  code "<<kCsv39<<"\n";
}

static void hud(const State&s,const Att&a,const Eigen::Vector3d&acc,const Eigen::Vector3d&gyr){cv::Mat c(860,1320,CV_8UC3,cv::Scalar(15,15,15));cv::Scalar w(235,235,235),gr(80,220,80),ye(0,220,255),rd(80,80,255);uiText(c,"JT-ZERO: КОМПЛЕКСНЫЙ ПОИСК ПРИЧИНЫ v39",35,52,.68,w,2);uiText(c,label(s.phase),45,125,.60,(s.phase==Phase::DONE?gr:ye),2);if(s.phase==Phase::WAIT_ZERO)uiText(c,"SPACE — ПОДТВЕРДИТЬ ФИЗИЧЕСКИЙ НОЛЬ",45,180,.52,w,2);else if(s.phase==Phase::ROT_R||s.phase==Phase::RET_0_A||s.phase==Phase::ROT_L||s.phase==Phase::RET_0_B)uiText(c,"После остановки и стабильности нажмите SPACE",45,180,.50,w,2);char b[256];snprintf(b,sizeof(b),"Стабильность %d/%d   |gyro| %.4f   |acc| %.4f",s.stable_n,kStableN,gyr.norm(),acc.norm());uiText(c,b,45,245,.46,w,1);snprintf(b,sizeof(b),"FC R/P/Y: %+.2f  %+.2f  %+.2f",a.roll,a.pitch,a.yaw);uiText(c,b,45,300,.46,w,1);snprintf(b,sizeof(b),"Vxy Euler %.3f м/с   Midpoint %.3f м/с",std::hypot(s.V_euler.x(),s.V_euler.y()),std::hypot(s.V_mid.x(),s.V_mid.y()));uiText(c,b,45,360,.52,rd,2);uiText(c,"Проверяется за один прогон:",45,445,.50,gr,2);uiText(c,"cross-axis gyro, bias drift, dt/gaps, gyro scale proxy,",65,500,.43,w,1);uiText(c,"accel orientation, FC-vs-ACC tilt, coning/midpoint, dynamic signature",65,545,.43,w,1);uiText(c,"Маршрут: 0 → вправо → 0 → влево → 0",45,655,.48,ye,2);uiText(c,"ESC/Q — прервать",45,805,.40,w,1);cv::imshow(kWindow39,c);}

} // namespace jtzero_v39

int main(int argc,char**argv){
  google::InitGoogleLogging(argv[0]);using namespace jtzero_v39;int fd=-1;uint8_t sys=0,comp=0;mavlink_status_t mst{};mavlink_message_t msg{};Att att;State s;Eigen::Vector3d acc=Eigen::Vector3d::Zero(),gyr=Eigen::Vector3d::Zero();bool aborted=false;uint64_t last=0;
  try{fd=openSerial();std::cout<<"[MAV] waiting for HEARTBEAT...\n";int64_t dl=monotonicNs()+10000000000LL;while(monotonicNs()<dl&&!sys){pollfd p{fd,POLLIN,0};if(poll(&p,1,100)<=0)continue;uint8_t b[2048];ssize_t n=read(fd,b,sizeof(b));if(n<=0)continue;for(ssize_t i=0;i<n;i++)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst)&&msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=msg.sysid;comp=msg.compid;break;}}if(!sys)throw std::runtime_error("HEARTBEAT timeout");requestRate(fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,100);requestRate(fd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,200);cv::namedWindow(kWindow39);std::cout<<"\nJT-ZERO IMU ROOT-CAUSE SWEEP v39\nFollow Russian GUI. Mechanical positions, no exact yaw angle required.\n";int64_t nh=0;
    while(true){pollfd p{fd,POLLIN,0};poll(&p,1,5);if(p.revents&POLLIN){uint8_t b[8192];ssize_t n=read(fd,b,sizeof(b));if(n>0)for(ssize_t i=0;i<n;i++)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&mst)){if(msg.msgid==MAVLINK_MSG_ID_ATTITUDE){mavlink_attitude_t a{};mavlink_msg_attitude_decode(&msg,&a);att.valid=true;att.roll=a.roll*180.0/kPi;att.pitch=a.pitch*180.0/kPi;att.yaw=a.yaw*180.0/kPi;att.rs=a.rollspeed;att.ps=a.pitchspeed;att.ys=a.yawspeed;}else if(msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){mavlink_highres_imu_t h{};mavlink_msg_highres_imu_decode(&msg,&h);if(h.time_usec==last)continue;last=h.time_usec;acc={h.xacc,-h.yacc,-h.zacc};gyr={h.xgyro,-h.ygyro,-h.zgyro};process(s,att,h.time_usec,acc,gyr);}}}
      if(monotonicNs()>=nh){hud(s,att,acc,gyr);nh=monotonicNs()+33333333LL;}int k=cv::waitKey(1)&255;if(k==27||k=='q'||k=='Q'){aborted=true;break;}if(k==' '&&s.phase==Phase::WAIT_ZERO&&att.valid){s.phase=Phase::CALIBRATE;s.cal_n=0;s.cal_as.setZero();s.cal_gs.setZero();std::cout<<"[ZERO] physical zero confirmed. Calibrating.\n";}else if(k==' '&&s.stable_n>=kStableN){if(s.phase==Phase::ROT_R){s.phase=Phase::STATIC_R;s.static_n=0;s.stable_n=0;std::cout<<"[CONFIRM] right position. Static collection.\n";}else if(s.phase==Phase::RET_0_A){s.phase=Phase::STATIC_0_B;s.static_n=0;s.stable_n=0;std::cout<<"[CONFIRM] first zero return. Static collection.\n";}else if(s.phase==Phase::ROT_L){s.phase=Phase::STATIC_L;s.static_n=0;s.stable_n=0;std::cout<<"[CONFIRM] left position. Static collection.\n";}else if(s.phase==Phase::RET_0_B){s.phase=Phase::STATIC_0_C;s.static_n=0;s.stable_n=0;std::cout<<"[CONFIRM] final zero return. Static collection.\n";}}else if(k==' '&&s.phase==Phase::DONE)break;
      if(s.phase==Phase::DONE){hud(s,att,acc,gyr);int kk=cv::waitKey(1)&255;if(kk==' '||kk==27||kk=='q'||kk=='Q')break;}
    }
    save(s);summary(s);std::cout<<"aborted: "<<(aborted?"yes":"no")<<"\n";requestRate(fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);requestRate(fd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);close(fd);cv::destroyAllWindows();return aborted?2:0;
  }catch(const std::exception&e){if(fd>=0)close(fd);cv::destroyAllWindows();std::cerr<<"[FATAL] "<<e.what()<<"\n";return 1;}
}
