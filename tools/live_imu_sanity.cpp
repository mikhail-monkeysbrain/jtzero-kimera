// JT-ZERO live IMU sanity diagnostic: inspect exactly the HIGHRES_IMU values
// and mapped timestamps used by the verified Kimera live runners, without VIO.
#define main jtzero_standstill_unused_main
#include "live_mono_imu_standstill.cpp"
#undef main

namespace {
struct Stats {
  uint64_t n=0, dt_n=0, nonmono=0, gap20=0;
  double sx=0,sy=0,sz=0,sgx=0,sgy=0,sgz=0,snorm=0,sgnorm=0;
  double min_dt_ms=1e9,max_dt_ms=0,sum_dt_ms=0;
  int64_t prev_ns=0;
  void add(int64_t t,double ax,double ay,double az,double gx,double gy,double gz){
    ++n; sx+=ax;sy+=ay;sz+=az;sgx+=gx;sgy+=gy;sgz+=gz;
    snorm+=std::sqrt(ax*ax+ay*ay+az*az); sgnorm+=std::sqrt(gx*gx+gy*gy+gz*gz);
    if(prev_ns){ double dt=(t-prev_ns)/1e6; if(dt<=0)++nonmono; else{++dt_n;sum_dt_ms+=dt;min_dt_ms=std::min(min_dt_ms,dt);max_dt_ms=std::max(max_dt_ms,dt);if(dt>20)++gap20;} }
    prev_ns=t;
  }
};
}

int main(int argc,char**argv){
  google::InitGoogleLogging(argv[0]);
  const double seconds=argc>1?std::max(5.0,std::atof(argv[1])):30.0;
  int fd=-1; uint8_t tsys=0,tcomp=0; bool imu_req=false;
  try{
    fd=openSerial();
    std::cout<<"[IMU] waiting for HEARTBEAT...\n";
    mavlink_status_t st{}; mavlink_message_t msg{};
    int64_t deadline=monotonicNs()+10000000000LL;
    while(monotonicNs()<deadline&&!tsys){
      pollfd p{fd,POLLIN,0}; if(poll(&p,1,100)<=0)continue;
      uint8_t b[512]; ssize_t n=read(fd,b,sizeof(b)); if(n<=0)continue;
      for(ssize_t i=0;i<n;i++) if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&st)&&msg.msgid==MAVLINK_MSG_ID_HEARTBEAT){tsys=msg.sysid;tcomp=msg.compid;break;}
    }
    if(!tsys)throw std::runtime_error("No HEARTBEAT");
    requestRate(fd,tsys,tcomp,MAVLINK_MSG_ID_HIGHRES_IMU,kImuRateHz); imu_req=true;

    std::vector<TimeSyncSample> timesync; timesync.reserve((size_t)(seconds*12));
    ClockMapping mapping; int64_t pending=0,next_ts=monotonicNs();
    Stats raw,mapped; uint64_t imu_rx=0,skipped=0;
    int64_t start=monotonicNs();
    std::cout<<"[IMU] Keep rig motionless for "<<seconds<<" s.\n";
    while(monotonicNs()-start<(int64_t)(seconds*1e9)){
      int64_t now=monotonicNs();
      if(now>=next_ts&&pending==0){pending=now;sendTimesync(fd,pending,tsys,tcomp);next_ts=now+kTimesyncPeriodNs;}
      pollfd p{fd,POLLIN,0}; if(poll(&p,1,20)<=0)continue;
      uint8_t b[4096];
      for(;;){
        ssize_t n=read(fd,b,sizeof(b)); if(n==-1&&(errno==EAGAIN||errno==EWOULDBLOCK))break; if(n<=0)break;
        for(ssize_t i=0;i<n;i++) if(mavlink_parse_char(MAVLINK_COMM_0,b[i],&msg,&st)){
          int64_t rx=monotonicNs();
          if(msg.msgid==MAVLINK_MSG_ID_TIMESYNC){
            mavlink_timesync_t ts{};mavlink_msg_timesync_decode(&msg,&ts);
            if(ts.tc1!=0&&pending!=0&&ts.ts1==pending){TimeSyncSample s;s.t0_rpi_ns=pending;s.t1_rpi_ns=rx;s.fc_ns=ts.tc1;s.rtt_ns=rx-pending;s.rpi_mid_ns=pending+s.rtt_ns/2;s.good=s.rtt_ns>0&&nsToMs(s.rtt_ns)<=kMaxTimesyncRttMs;timesync.push_back(s);pending=0;mapping=estimateClockMapping(timesync);}
          }else if(msg.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){
            mavlink_highres_imu_t h{};mavlink_msg_highres_imu_decode(&msg,&h);++imu_rx;
            int64_t fc_ns=(int64_t)h.time_usec*1000LL;
            raw.add(fc_ns,h.xacc,h.yacc,h.zacc,h.xgyro,h.ygyro,h.zgyro);
            if(!mapping.valid||rx-mapping.last_update_ns>kMappingStaleNs){++skipped;continue;}
            mapped.add(mapping.map(fc_ns),h.xacc,h.yacc,h.zacc,h.xgyro,h.ygyro,h.zgyro);
          }
        }
      }
      if(pending&&monotonicNs()-pending>20000000LL)pending=0;
    }
    if(imu_req){requestRate(fd,tsys,tcomp,MAVLINK_MSG_ID_HIGHRES_IMU,0);imu_req=false;} close(fd);fd=-1;
    auto print=[](const char*name,const Stats&s){double d=std::max<uint64_t>(1,s.n);std::cout<<"\n"<<name<<" samples: "<<s.n<<"\nmean acc [m/s^2]: ["<<s.sx/d<<","<<s.sy/d<<","<<s.sz/d<<"]\nmean |acc| [m/s^2]: "<<s.snorm/d<<"\nmean gyro [rad/s]: ["<<s.sgx/d<<","<<s.sgy/d<<","<<s.sgz/d<<"]\nmean |gyro| [rad/s]: "<<s.sgnorm/d; if(s.dt_n)std::cout<<"\ndt ms min/mean/max: "<<s.min_dt_ms<<" / "<<s.sum_dt_ms/s.dt_n<<" / "<<s.max_dt_ms<<"\nnonmonotonic: "<<s.nonmono<<"  gaps>20ms: "<<s.gap20;std::cout<<"\n";};
    std::cout<<"\n============================================================\nJT-ZERO LIVE IMU SANITY RESULT\n============================================================\n"<<"duration: "<<seconds<<" s\nIMU received: "<<imu_rx<<"\nIMU skipped before mapping valid: "<<skipped<<"\nTIMESYNC samples: "<<timesync.size()<<"\nmapping valid: "<<(mapping.valid?"yes":"no")<<"\nmapping drift ppm: "<<mapping.drift_ppm<<"\n";
    print("RAW FC",raw);print("MAPPED TO RPI",mapped);
    std::cout<<"Expected stationary acceleration magnitude: approximately 9.81 m/s^2.\n";
    return mapping.valid&&mapped.n>1000?0:2;
  }catch(const std::exception&e){if(fd>=0&&imu_req&&tsys){try{requestRate(fd,tsys,tcomp,MAVLINK_MSG_ID_HIGHRES_IMU,0);}catch(...){}}if(fd>=0)close(fd);std::cerr<<"[FATAL] "<<e.what()<<"\n";return 1;}
}
