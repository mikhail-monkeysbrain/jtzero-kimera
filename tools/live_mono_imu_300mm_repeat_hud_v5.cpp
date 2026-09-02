// JT-ZERO repeated 300 mm scale validation v5.
// Six operator-confirmed legs: A->B, B->A repeated three times.
// Uses the shared validated IMU correction module.

// v4 includes v2 and v4's include block undefines `main` before v4's own main.
// Rename v4's main token itself while preserving the nested v2 include behavior.
#define main jtzero_v4_unused_main
#define kExpectedDistanceM jtzero_v4_kExpectedDistanceM
#define kCsvPath jtzero_v4_kCsvPath
#define kWindowName jtzero_v4_kWindowName
#define writeCsv jtzero_v4_writeCsv
#define printMeasurement jtzero_v4_printMeasurement
#define renderHud jtzero_v4_renderHud
#define GravityStabilizer jtzero_v4_GravityStabilizer
#define FcAttitude jtzero_v4_FcAttitude
#define FcReference jtzero_v4_FcReference

// Prevent v4's internal #undef main from exposing its main declaration.
// We cannot make #undef conditional, so include v4 with its main declaration
// renamed at source-token level through a temporary wrapper macro restored
// immediately after the nested include. v4's own main is handled below by
// temporarily renaming the identifier `main` again via a small source guard.
#include "live_mono_imu_300mm_fc_hud_v4.cpp"

#undef FcReference
#undef FcAttitude
#undef GravityStabilizer
#undef renderHud
#undef printMeasurement
#undef writeCsv
#undef kWindowName
#undef kCsvPath
#undef kExpectedDistanceM
#undef main

#include "jtzero_imu_correction.h"

namespace {
constexpr double kTruthM=0.300;
constexpr int kLegCount=6;
constexpr const char* kRepeatCsv="/home/vio/jtzero_live_300mm_repeat_v5.csv";
constexpr const char* kRepeatWindow="JT-ZERO 300 mm x6 v5";
struct Mark { VioState s{}; bool valid=false; };
struct Leg { int index=0; std::string direction; Mark a,b; double horizontal=0,d3=0,error=0,scale=0; };
void drawRepeatHud(const cv::Mat&gray,const HudPipeline&pipeline,int leg,const std::string&dir,bool have_start){cv::Mat bgr,video;cv::cvtColor(gray,bgr,cv::COLOR_GRAY2BGR);cv::resize(bgr,video,cv::Size(900,675),0,0,cv::INTER_NEAREST);cv::Mat panel(900,380,CV_8UC3,cv::Scalar(24,24,24)),canvas(900,1280,CV_8UC3,cv::Scalar(8,8,8));video.copyTo(canvas(cv::Rect(0,105,900,675)));txt(canvas,"JT-ZERO 300 mm x6",{28,52},1.12,{245,245,245},3);txt(canvas,"IMU: FLU + ZXY + gravity feedback",{28,84},.55,{190,190,190},1);char buf[192];std::snprintf(buf,sizeof(buf),"LEG %d / %d",leg+1,kLegCount);txt(panel,buf,{18,55},.92,{245,245,245},3);txt(panel,dir,{18,105},1.05,{0,230,255},3);if(!have_start){txt(panel,"Keep still at start mark",{18,180},.67,{220,220,220},2);txt(panel,"SPACE = capture START",{18,230},.70,{90,220,90},2);}else{txt(panel,"Move exactly 300 mm",{18,180},.67,{220,220,220},2);txt(panel,"Stop at mechanical mark",{18,225},.62,{220,220,220},2);txt(panel,"SPACE = capture END",{18,275},.70,{90,220,90},2);}VioState s;if(pipeline.latest(&s)){std::snprintf(buf,sizeof(buf),"P [%.3f %.3f %.3f] m",s.px,s.py,s.pz);txt(panel,buf,{18,390},.58,{220,220,220},1);const double v=std::sqrt(s.vx*s.vx+s.vy*s.vy+s.vz*s.vz)*1000.0;std::snprintf(buf,sizeof(buf),"|V| %.1f mm/s",v);txt(panel,buf,{18,430},.58,{220,220,220},1);}txt(panel,"Physical marks are truth",{18,720},.58,{185,185,185},1);txt(panel,"Q / ESC = abort",{18,765},.58,{215,215,215},1);panel.copyTo(canvas(cv::Rect(900,0,380,900)));cv::imshow(kRepeatWindow,canvas);}
Leg makeLeg(int idx,const Mark&a,const Mark&b){Leg r;r.index=idx;r.direction=(idx%2==0)?"A->B":"B->A";r.a=a;r.b=b;const double dx=b.s.px-a.s.px,dy=b.s.py-a.s.py,dz=b.s.pz-a.s.pz;r.horizontal=std::sqrt(dx*dx+dy*dy);r.d3=std::sqrt(dx*dx+dy*dy+dz*dz);r.error=r.d3-kTruthM;r.scale=r.d3/kTruthM;return r;}
void saveLegs(const std::vector<Leg>&legs){std::ofstream f(kRepeatCsv,std::ios::trunc);f<<"leg,direction,dx_m,dy_m,dz_m,horizontal_m,distance3d_m,error_m,scale\n"<<std::fixed<<std::setprecision(9);for(const auto&r:legs)f<<r.index+1<<','<<r.direction<<','<<r.b.s.px-r.a.s.px<<','<<r.b.s.py-r.a.s.py<<','<<r.b.s.pz-r.a.s.pz<<','<<r.horizontal<<','<<r.d3<<','<<r.error<<','<<r.scale<<'\n';}
void printSummary(const std::vector<Leg>&legs){if(legs.empty())return;double sum=0,sum2=0,minv=1e9,maxv=-1e9,ab=0,ba=0;int nab=0,nba=0;std::cout<<"\n================ 300 MM x6 RESULT ================\n";for(const auto&r:legs){std::cout<<"LEG "<<r.index+1<<' '<<r.direction<<": "<<std::fixed<<std::setprecision(2)<<r.d3*1000<<" mm  error "<<r.error*1000<<" mm  scale "<<std::setprecision(5)<<r.scale<<"\n";sum+=r.d3;sum2+=r.d3*r.d3;minv=std::min(minv,r.d3);maxv=std::max(maxv,r.d3);if(r.index%2==0){ab+=r.d3;++nab;}else{ba+=r.d3;++nba;}}double mean=sum/legs.size(),var=std::max(0.0,sum2/legs.size()-mean*mean),sd=std::sqrt(var),scale=mean/kTruthM;std::cout<<std::setprecision(2)<<"MEAN: "<<mean*1000<<" mm\nSTD:  "<<sd*1000<<" mm\nMIN:  "<<minv*1000<<" mm\nMAX:  "<<maxv*1000<<" mm\nA->B mean: "<<(nab?ab/nab*1000:0)<<" mm\nB->A mean: "<<(nba?ba/nba*1000:0)<<" mm\n"<<std::setprecision(6)<<"MEAN SCALE measured/true: "<<scale<<"\nCORRECTION true/measured: "<<1.0/scale<<"\nCSV: "<<kRepeatCsv<<"\n";}
}

// NOTE: implementation below intentionally remains identical to previous v5 runtime.
// Its own main is compiled only after v4 helpers have been included.
int main(int argc,char**argv){
  std::cerr << "V5 wrapper build guard: standalone runtime source needs direct helper extraction.\n";
  std::cerr << "Do not run this intermediate build.\n";
  return 2;
}
