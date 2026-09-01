// JT-ZERO static HIGHRES_IMU + ATTITUDE frame diagnostic.
// Three 8 s phases: LEVEL, RIGHT-DOWN, NOSE-DOWN. No Kimera pipeline.
#define main jtzero_standstill_unused_main
#include "live_mono_imu_standstill.cpp"
#undef main

namespace {
struct Mean {
  uint64_t ni=0,na=0; double ax=0,ay=0,az=0,gx=0,gy=0,gz=0,r=0,p=0; double sy=0,cy=0;
  void imu(const mavlink_highres_imu_t& h){++ni;ax+=h.xacc;ay+=h.yacc;az+=h.zacc;gx+=h.xgyro;gy+=h.ygyro;gz+=h.zgyro;}
  void att(const mavlink_attitude_t& a){++na;r+=a.roll;p+=a.pitch;sy+=std::sin(a.yaw);cy+=std::cos(a.yaw);}
  void print(const char* n) const { const double di=std::max<uint64_t>(1,ni), da=std::max<uint64_t>(1,na); const double mx=ax/di,my=ay/di,mz=az/di; std::cout<<"\n"<<n<<"\nIMU samples: "<<ni<<"  ATTITUDE samples: "<<na<<"\nacc [m/s^2]: ["<<mx<<","<<my<<","<<mz<<"]  |acc|="<<std::sqrt(mx*mx+my*my+mz*mz)<<"\ngyro [rad/s]: ["<<gx/di<<","<<gy/di<<","<<gz/di<<"]\nATTITUDE deg: roll="<<(r/da)*180.0/kPi<<" pitch="<<(p/da)*180.0/kPi<<" yaw="<<std::atan2(sy/da,cy/da)*180.0/kPi<<"\n"; }
};
}

int main(int argc,char**argv){
 google::InitGoogleLogging(argv[0]); int fd=-1; uint8_t sys=0,comp=0; bool reqi=false,reqa=false;
 try{
  fd=openSerial(); mavlink_status_t st{}; mavlink_message_t msg{}; std::cout<<"[MAV] waiting for HEARTBEAT...\n"; int64_t dl=monotonicNs()+10000000000LL;
  while(monotonicNs()<dl&&!sys){pollfd q{fd,POLLIN,0};if(poll(&q,1,100)<=0)continue;uint8_t b[1024];ssize_t n=read(fd,b,sizeof(b));if(n<=0)continue;for(ssize_t i=0;i<n;i++)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&st)&&msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){sys=msg.sysid;comp=msg.compid;break;}}
  if(!sys)throw std::runtime_error("HEARTBEAT timeout"); requestRate(fd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,200);reqi=true;requestRate(fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,50);reqa=true;
  const char* names[3]={"LEVEL","RIGHT SIDE DOWN","NOSE DOWN"}; Mean m[3];
  for(int phase=0;phase<3;phase++){
   std::cout<<"\n============================================================\nPHASE "<<(phase+1)<<"/3: "<<names[phase]<<"\n";
   if(phase==0)std::cout<<"Place rig in normal level test position.\n"; else if(phase==1)std::cout<<"Tilt RIGHT side down about 15-25 deg and HOLD.\n"; else std::cout<<"Tilt NOSE down about 15-25 deg and HOLD.\n";
   std::cout<<"You have 5 seconds to position it...\n"; std::this_thread::sleep_for(std::chrono::seconds(5)); std::cout<<"HOLD STILL: collecting 8 seconds...\n"; int64_t end=monotonicNs()+8000000000LL;
   while(monotonicNs()<end){pollfd q{fd,POLLIN,0};if(poll(&q,1,20)<=0)continue;uint8_t b[4096];for(;;){ssize_t n=read(fd,b,sizeof(b));if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK))break;if(n<=0)break;for(ssize_t i=0;i<n;i++)if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&st)){if(msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){mavlink_highres_imu_t h{};mavlink_msg_highres_imu_decode(&msg,&h);m[phase].imu(h);}else if(msg.msgid==MAVLINK_MSG_ID_ATTITUDE){mavlink_attitude_t a{};mavlink_msg_attitude_decode(&msg,&a);m[phase].att(a);}}}}
   m[phase].print(names[phase]);
  }
  requestRate(fd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);reqi=false;requestRate(fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);reqa=false;close(fd);fd=-1;
  std::cout<<"\n============================================================\nJT-ZERO STATIC IMU/ATTITUDE FRAME RESULT\n============================================================\n";for(int i=0;i<3;i++)m[i].print(names[i]);
  std::cout<<"Expected FRD qualitative response: right-side-down gives positive roll and a gravity-vector change mainly on Y; nose-down gives negative pitch and a gravity-vector change mainly on X. Exact accelerometer sign will be determined from these measurements, not assumed.\n";return 0;
 }catch(const std::exception&e){if(fd>=0&&sys){try{if(reqi)requestRate(fd,sys,comp,MAVLINK_MSG_ID_HIGHRES_IMU,0);if(reqa)requestRate(fd,sys,comp,MAVLINK_MSG_ID_ATTITUDE,0);}catch(...){}}if(fd>=0)close(fd);std::cerr<<"[FATAL] "<<e.what()<<"\n";return 1;}
}
