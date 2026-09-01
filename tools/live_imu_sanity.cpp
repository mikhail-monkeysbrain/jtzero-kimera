// JT-ZERO live IMU sanity diagnostic: inspect exactly the HIGHRES_IMU values
// and mapped timestamps used by the Kimera live runners, without starting VIO.
#define main jtzero_standstill_unused_main
#include "live_mono_imu_standstill.cpp"
#undef main

namespace {
struct Stats {
  uint64_t n = 0;
  double sx=0, sy=0, sz=0, sgx=0, sgy=0, sgz=0;
  double snorm=0, sgnorm=0;
  double min_dt_ms=1e9, max_dt_ms=0, sum_dt_ms=0;
  uint64_t dt_n=0, nonmono=0, gap20=0;
  int64_t prev_ns=0;
  void add(int64_t t, double ax,double ay,double az,double gx,double gy,double gz) {
    ++n; sx+=ax; sy+=ay; sz+=az; sgx+=gx; sgy+=gy; sgz+=gz;
    snorm += std::sqrt(ax*ax+ay*ay+az*az);
    sgnorm += std::sqrt(gx*gx+gy*gy+gz*gz);
    if(prev_ns){
      double dt=(t-prev_ns)/1e6;
      if(dt<=0) ++nonmono;
      else { ++dt_n; sum_dt_ms+=dt; min_dt_ms=std::min(min_dt_ms,dt); max_dt_ms=std::max(max_dt_ms,dt); if(dt>20)++gap20; }
    }
    prev_ns=t;
  }
};
}

int main(int argc,char**argv){
  google::InitGoogleLogging(argv[0]);
  const double seconds = argc>1 ? std::max(5.0,std::atof(argv[1])) : 30.0;
  int fd=-1; uint8_t tsys=0,tcomp=0; bool imu_req=false;
  try {
    fd=openSerial();
    std::cout<<"[IMU] waiting for HEARTBEAT...\n";
    mavlink_status_t st{}; mavlink_message_t msg{};
    int64_t deadline=monotonicNs()+10000000000LL;
    while(monotonicNs()<deadline&&!tsys){
      pollfd p{fd,POLLIN,0}; if(poll(&p,1,100)<=0)continue;
      uint8_t b[512]; ssize_t n=read(fd,b,sizeof(b));
      for(ssize_t i=0;i<n;i++) if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&st)&&msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){tsys=msg.sysid;tcomp=msg.compid;break;}
    }
    if(!tsys) throw std::runtime_error("No HEARTBEAT");
    requestMessageInterval(fd,tsys,tcomp,MAVLINK_MSG_ID_HIGHRES_IMU,200); imu_req=true;
    TimesyncAffineMapper mapper;
    Stats raw,mapped;
    uint64_t imu_rx=0, skipped=0, ts_tx=0;
    int64_t start=monotonicNs(), next_ts=start;
    std::cout<<"[IMU] Keep rig motionless for "<<seconds<<" s.\n";
    while(monotonicNs()-start < (int64_t)(seconds*1e9)){
      int64_t now=monotonicNs();
      if(now>=next_ts){ sendTimesyncRequest(fd,now); ++ts_tx; next_ts=now+100000000LL; }
      pollfd p{fd,POLLIN,0}; if(poll(&p,1,20)<=0)continue;
      uint8_t b[1024]; ssize_t n=read(fd,b,sizeof(b)); if(n<=0)continue;
      for(ssize_t i=0;i<n;i++) if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&st)){
        int64_t rx=monotonicNs();
        if(msg.msgid==MAVLINK_MSG_ID_TIMESYNC){ mavlink_timesync_t t{}; mavlink_msg_timesync_decode(&msg,&t); handleTimesyncMessage(fd,t,rx,&mapper); }
        else if(msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){
          mavlink_highres_imu_t h{}; mavlink_msg_highres_imu_decode(&msg,&h); ++imu_rx;
          raw.add((int64_t)h.time_usec*1000LL,h.xacc,h.yacc,h.zacc,h.xgyro,h.ygyro,h.zgyro);
          int64_t mt=0; if(!mapper.mapFcUsecToRpiNs(h.time_usec,rx,&mt)){++skipped;continue;}
          mapped.add(mt,h.xacc,h.yacc,h.zacc,h.xgyro,h.ygyro,h.zgyro);
        }
      }
    }
    if(imu_req) requestMessageInterval(fd,tsys,tcomp,MAVLINK_MSG_ID_HIGHRES_IMU,0);
    if(fd>=0) close(fd);
    auto print=[&](const char*name,const Stats&s){
      double d=std::max<uint64_t>(1,s.n);
      std::cout<<"\n"<<name<<" samples: "<<s.n
               <<"\nmean acc [m/s^2]: ["<<s.sx/d<<","<<s.sy/d<<","<<s.sz/d<<"]"
               <<"\nmean |acc| [m/s^2]: "<<s.snorm/d
               <<"\nmean gyro [rad/s]: ["<<s.sgx/d<<","<<s.sgy/d<<","<<s.sgz/d<<"]"
               <<"\nmean |gyro| [rad/s]: "<<s.sgnorm/d;
      if(s.dt_n) std::cout<<"\ndt ms min/mean/max: "<<s.min_dt_ms<<" / "<<s.sum_dt_ms/s.dt_n<<" / "<<s.max_dt_ms
                         <<"\nnonmonotonic: "<<s.nonmono<<"  gaps>20ms: "<<s.gap20;
      std::cout<<"\n";
    };
    std::cout<<"\n============================================================\nJT-ZERO LIVE IMU SANITY RESULT\n============================================================\n";
    std::cout<<"duration: "<<seconds<<" s\nIMU received: "<<imu_rx<<"\nIMU skipped before mapping valid: "<<skipped
             <<"\nTIMESYNC requests: "<<ts_tx<<"\nmapping valid: "<<(mapper.valid()?"yes":"no")
             <<"\nmapping drift ppm: "<<mapper.driftPpm()<<"\n";
    print("RAW FC",raw); print("MAPPED TO RPI",mapped);
    std::cout<<"Expected stationary acceleration magnitude: approximately 9.81 m/s^2.\n";
    std::cout<<"Do not infer axis correctness from this test alone; use the existing roll/pitch/yaw sign test for axes.\n";
    return mapper.valid()&&mapped.n>1000?0:2;
  } catch(const std::exception&e){ if(fd>=0)close(fd); std::cerr<<"[FATAL] "<<e.what()<<"\n"; return 1; }
}
