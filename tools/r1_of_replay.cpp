// JT-Zero R1 6/6 standalone pixel-level optical-flow replay.
// Independent of V25/V43/Kimera runtime. Uses only recorded R1 dataset + camera YAML.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/video/tracking.hpp>

struct Frame { uint64_t id=0,seq=0,vsec=0,vusec=0,recv=0,off=0,bytes=0; };
struct Range { uint64_t id=0,recv=0; double cm=0; int strength=0,temp=0,valid=0; };
struct Att { uint64_t id=0,recv=0,boot=0; double r=0,p=0,y=0,rs=0,ps=0,ys=0; };

static std::vector<std::string> split(const std::string&s){std::vector<std::string>v;std::stringstream q(s);std::string x;while(std::getline(q,x,','))v.push_back(x);return v;}
static uint64_t u64(const std::string&s){return std::stoull(s);}
static int i32(const std::string&s){return std::stoi(s);}
static double f64(const std::string&s){return std::stod(s);}

template<class T> static const T& nearest(const std::vector<T>& v,uint64_t t){
  auto it=std::lower_bound(v.begin(),v.end(),t,[](const T&a,uint64_t x){return a.recv<x;});
  if(it==v.begin()) return *it; if(it==v.end()) return v.back();
  const auto& a=*(it-1); const auto& b=*it; return (t-a.recv<=b.recv-t)?a:b;
}
static double median(std::vector<double> v){
  if(v.empty()) return 0; size_t n=v.size()/2; std::nth_element(v.begin(),v.begin()+n,v.end()); double m=v[n];
  if(v.size()%2==0){auto it=std::max_element(v.begin(),v.begin()+n);m=(m+*it)*0.5;} return m;
}
static cv::Matx33d rpy(double r,double p,double y){
  const double cr=cos(r),sr=sin(r),cp=cos(p),sp=sin(p),cy=cos(y),sy=sin(y);
  return {cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr,
          sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr,
          -sp,cp*sr,cp*cr};
}
static std::vector<Frame> loadFrames(const std::string&p){
  std::ifstream f(p); if(!f)throw std::runtime_error("open frames.csv");std::string s;std::getline(f,s);std::vector<Frame>v;
  while(std::getline(f,s)){if(s.empty())continue;auto a=split(s);if(a.size()!=7)throw std::runtime_error("bad frame row");v.push_back({u64(a[0]),u64(a[1]),u64(a[2]),u64(a[3]),u64(a[4]),u64(a[5]),u64(a[6])});}return v;
}
static std::vector<Range> loadRange(const std::string&p){
  std::ifstream f(p);if(!f)throw std::runtime_error("open range.csv");std::string s;std::getline(f,s);std::vector<Range>v;
  while(std::getline(f,s)){if(s.empty())continue;auto a=split(s);if(a.size()!=6)throw std::runtime_error("bad range row");v.push_back({u64(a[0]),u64(a[1]),f64(a[2]),i32(a[3]),i32(a[4]),i32(a[5])});}return v;
}
static std::vector<Att> loadAtt(const std::string&p){
  std::ifstream f(p);if(!f)throw std::runtime_error("open attitude.csv");std::string s;std::getline(f,s);std::vector<Att>v;
  while(std::getline(f,s)){if(s.empty())continue;auto a=split(s);if(a.size()!=9)throw std::runtime_error("bad attitude row");v.push_back({u64(a[0]),u64(a[1]),u64(a[2]),f64(a[3]),f64(a[4]),f64(a[5]),f64(a[6]),f64(a[7]),f64(a[8])});}return v;
}
static cv::Mat decode(std::ifstream& mj,const Frame& f){
  std::vector<uchar>b(f.bytes);mj.clear();mj.seekg((std::streamoff)f.off);mj.read((char*)b.data(),(std::streamsize)f.bytes);
  if((uint64_t)mj.gcount()!=f.bytes)throw std::runtime_error("short MJPEG read");
  cv::Mat im=cv::imdecode(b,cv::IMREAD_GRAYSCALE);if(im.empty())throw std::runtime_error("JPEG decode failed");return im;
}
static std::string hex64(uint64_t x){std::ostringstream o;o<<std::hex<<std::setw(16)<<std::setfill('0')<<x;return o.str();}
static uint64_t fnv(uint64_t h,const std::string&s){for(unsigned char c:s){h^=c;h*=1099511628211ULL;}return h;}

int main(int argc,char**argv){
 try{
  cv::setNumThreads(1); cv::setRNGSeed(1);
  const std::string dir=argc>1?argv[1]:"/home/vio/r1_static";
  const std::string yaml=argc>2?argv[2]:"params/JTZeroMonoFLU/LeftCameraParams.yaml";
  const int stride=argc>3?std::max(1,std::stoi(argv[3])):3;
  const std::string out=argc>4?argv[4]:dir+"/of_replay";
  std::filesystem::create_directories(out);

  auto F=loadFrames(dir+"/frames.csv");auto R=loadRange(dir+"/range.csv");auto A=loadAtt(dir+"/attitude.csv");
  if(F.size()<stride+1||R.empty()||A.empty())throw std::runtime_error("dataset too small");
  std::ifstream mj(dir+"/frames.mjpg",std::ios::binary);if(!mj)throw std::runtime_error("open frames.mjpg");

  cv::FileStorage fs(yaml,cv::FileStorage::READ);if(!fs.isOpened())throw std::runtime_error("open camera yaml");
  std::vector<double> intr,dist,tbs;fs["intrinsics"]>>intr;fs["distortion_coefficients"]>>dist;fs["T_BS"]["data"]>>tbs;
  if(intr.size()!=4||dist.size()<4||tbs.size()!=16)throw std::runtime_error("bad camera yaml");
  cv::Matx33d K(intr[0],0,intr[2],0,intr[1],intr[3],0,0,1),Rbc;
  for(int r=0;r<3;r++)for(int col=0;col<3;col++)Rbc(r,col)=tbs[r*4+col];
  cv::Mat D(dist);

  std::ofstream tracks(out+"/tracks.csv"),pairs(out+"/pairs.csv");
  if(!tracks||!pairs)throw std::runtime_error("create OF output");
  tracks<<"pair_id,frame0_id,frame1_id,t0_ns,t1_ns,track_id,x0,y0,x1,y1,x1_rot_pred,y1_rot_pred,res_dx_px,res_dy_px,fb_err_px,range_cm,range_dt_ms,att0_dt_ms,att1_dt_ms\n";
  pairs<<"pair_id,frame0_id,frame1_id,t0_ns,t1_ns,dt_ms,tracks,median_res_dx_px,median_res_dy_px,median_metric_x_mm,median_metric_y_mm,range_cm,range_dt_ms,att0_dt_ms,att1_dt_ms\n";
  tracks<<std::fixed<<std::setprecision(6);pairs<<std::fixed<<std::setprecision(6);

  uint64_t pairId=0,skippedGap=0,decodedPairs=0,totalTracks=0,digest=1469598103934665603ULL;
  double netx=0,nety=0,path=0,maxRangeDt=0,maxAttDt=0;
  for(size_t i=0;i+(size_t)stride<F.size();i+=stride){
    const Frame& f0=F[i];const Frame& f1=F[i+stride];
    if(f1.seq!=f0.seq+(uint64_t)stride){skippedGap++;continue;}
    cv::Mat a=decode(mj,f0),b=decode(mj,f1);decodedPairs++;
    std::vector<cv::Point2f> p0,p1,p0back;
    cv::goodFeaturesToTrack(a,p0,350,0.01,8.0,cv::noArray(),7,false,0.04);
    if(p0.size()<20)continue;
    std::vector<uchar>s01,s10;std::vector<float>e01,e10;
    cv::calcOpticalFlowPyrLK(a,b,p0,p1,s01,e01,cv::Size(21,21),3,cv::TermCriteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,30,0.01),0,1e-4);
    cv::calcOpticalFlowPyrLK(b,a,p1,p0back,s10,e10,cv::Size(21,21),3,cv::TermCriteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,30,0.01),0,1e-4);

    const Att& at0=nearest(A,f0.recv);const Att& at1=nearest(A,f1.recv);const Range& rg=nearest(R,(f0.recv+f1.recv)/2);
    const double rd=std::abs((double)((int64_t)rg.recv-(int64_t)((f0.recv+f1.recv)/2))/1e6);
    const double ad0=std::abs((double)((int64_t)at0.recv-(int64_t)f0.recv)/1e6),ad1=std::abs((double)((int64_t)at1.recv-(int64_t)f1.recv)/1e6);
    maxRangeDt=std::max(maxRangeDt,rd);maxAttDt=std::max(maxAttDt,std::max(ad0,ad1));
    if(!rg.valid)continue;
    const cv::Matx33d Rw0=rpy(at0.r,at0.p,at0.y),Rw1=rpy(at1.r,at1.p,at1.y);
    const cv::Matx33d Rc10=Rbc.t()*Rw1.t()*Rw0*Rbc;

    std::vector<double>dxs,dys,mxs,mys;size_t tid=0;
    for(size_t j=0;j<p0.size();j++){
      if(j>=s01.size()||j>=s10.size()||!s01[j]||!s10[j])continue;
      const double fb=cv::norm(p0[j]-p0back[j]);if(fb>0.75)continue;
      std::vector<cv::Point2f> one{p0[j]},und;cv::undistortPoints(one,und,cv::Mat(K),D);
      cv::Vec3d ray(und[0].x,und[0].y,1.0);cv::Vec3d rr=Rc10*ray;if(rr[2]<=1e-9)continue;
      std::vector<cv::Point3f> obj{{(float)(rr[0]/rr[2]),(float)(rr[1]/rr[2]),1.0f}};std::vector<cv::Point2f> pred;
      cv::projectPoints(obj,cv::Vec3d(0,0,0),cv::Vec3d(0,0,0),cv::Mat(K),D,pred);
      const double dx=p1[j].x-pred[0].x,dy=p1[j].y-pred[0].y;
      // Small-motion metric approximation after exact rotational reprojection.
      // Use focal axes independently; no empirical scale coefficient.
      const double h=rg.cm*10.0;const double mx=-h*dx/intr[0],my=-h*dy/intr[1];
      dxs.push_back(dx);dys.push_back(dy);mxs.push_back(mx);mys.push_back(my);
      tracks<<pairId<<','<<f0.id<<','<<f1.id<<','<<f0.recv<<','<<f1.recv<<','<<tid++<<','<<p0[j].x<<','<<p0[j].y<<','<<p1[j].x<<','<<p1[j].y<<','<<pred[0].x<<','<<pred[0].y<<','<<dx<<','<<dy<<','<<fb<<','<<rg.cm<<','<<rd<<','<<ad0<<','<<ad1<<"\n";
    }
    if(dxs.size()<20)continue;
    const double mdx=median(dxs),mdy=median(dys),mx=median(mxs),my=median(mys);
    const double dt=(f1.recv-f0.recv)/1e6;
    pairs<<pairId<<','<<f0.id<<','<<f1.id<<','<<f0.recv<<','<<f1.recv<<','<<dt<<','<<dxs.size()<<','<<mdx<<','<<mdy<<','<<mx<<','<<my<<','<<rg.cm<<','<<rd<<','<<ad0<<','<<ad1<<"\n";
    std::ostringstream row;row<<pairId<<','<<f0.id<<','<<f1.id<<','<<f0.recv<<','<<f1.recv<<','<<std::fixed<<std::setprecision(6)<<mdx<<','<<mdy<<','<<mx<<','<<my<<','<<dxs.size();digest=fnv(digest,row.str());
    netx+=mx;nety+=my;path+=std::hypot(mx,my);totalTracks+=dxs.size();pairId++;
  }
  tracks.close();pairs.close();
  const double net=std::hypot(netx,nety);
  std::ofstream summary(out+"/summary.txt");
  summary<<std::fixed<<std::setprecision(6)
         <<"pairs="<<pairId<<"\ndecoded_pairs="<<decodedPairs<<"\nskipped_sequence_gap="<<skippedGap<<"\ntracks="<<totalTracks
         <<"\nnet_x_mm="<<netx<<"\nnet_y_mm="<<nety<<"\nnet_mm="<<net<<"\npath_mm="<<path
         <<"\nmax_range_dt_ms="<<maxRangeDt<<"\nmax_att_dt_ms="<<maxAttDt<<"\ndigest="<<hex64(digest)<<"\n";
  summary.close();
  std::cout<<std::fixed<<std::setprecision(3)
           <<"R1 6/6 — STANDALONE PIXEL OF REPLAY\n"
           <<"dataset="<<dir<<" stride="<<stride<<"\n"
           <<"pairs="<<pairId<<" decoded="<<decodedPairs<<" skipped_sequence_gap="<<skippedGap<<" tracks="<<totalTracks<<"\n"
           <<"net=("<<netx<<","<<nety<<") mm net="<<net<<" mm path="<<path<<" mm\n"
           <<"max_range_dt="<<maxRangeDt<<" ms max_att_dt="<<maxAttDt<<" ms\n"
           <<"digest="<<hex64(digest)<<"\n"
           <<"output="<<out<<"\n"
           <<"R1 OF REPLAY PASS\n";
  return 0;
 }catch(const std::exception&e){std::cerr<<"R1 OF REPLAY FAIL: "<<e.what()<<"\n";return 1;}
}
