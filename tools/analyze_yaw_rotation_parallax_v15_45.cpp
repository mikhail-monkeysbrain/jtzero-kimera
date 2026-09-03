// JT-ZERO v15.45 offline visual rotation/parallax diagnostic.
// Uses the existing v15.42 camera MJPEG + camera index + FC ATTITUDE only.
// Production Kimera sources/params are not modified.

#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr int64_t kReplayCameraPeriodNs = 30000000LL;
constexpr int64_t kReplayRawMaxDtNs = 20000000LL;
constexpr double kPi = 3.14159265358979323846;

struct CamRow { uint32_t seq=0; int64_t ts=0; uint64_t off=0; size_t bytes=0; };
struct AttRow { int64_t ts=0; double r=0,p=0,y=0, rs=0,ps=0,ys=0; };
struct PairMetric {
  int64_t t0=0,t1=0; int tracks=0; double dt=0;
  double fc_rot_deg=0, yaw_rate_deg_s=0;
  double raw_med=0, rot_med=0, residual_med=0, residual_p90=0, ratio=0;
  double h_inlier=0, h_med=0;
  bool rotating=false;
};

std::vector<std::string> split(const std::string& s){
  std::vector<std::string> o; std::string c;
  for(char ch:s){ if(ch==','){o.push_back(c);c.clear();} else c.push_back(ch); }
  o.push_back(c); return o;
}
long long i64v(const std::string&s){return s.empty()?0:std::stoll(s);} 
double dv(const std::string&s){return s.empty()?0.0:std::stod(s);} 

double median(std::vector<double> v){
  if(v.empty()) return std::numeric_limits<double>::quiet_NaN();
  const size_t n=v.size(),m=n/2; std::nth_element(v.begin(),v.begin()+m,v.end()); double a=v[m];
  if(n%2) return a; std::nth_element(v.begin(),v.begin()+m-1,v.end()); return 0.5*(a+v[m-1]);
}
double percentile(std::vector<double> v,double q){
  if(v.empty()) return std::numeric_limits<double>::quiet_NaN();
  size_t k=(size_t)std::llround(q*(v.size()-1)); std::nth_element(v.begin(),v.begin()+k,v.end()); return v[k];
}

cv::Matx33d rpy(double r,double p,double y){
  r*=kPi/180.0;p*=kPi/180.0;y*=kPi/180.0;
  const double cr=cos(r),sr=sin(r),cp=cos(p),sp=sin(p),cy=cos(y),sy=sin(y);
  return cv::Matx33d(cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr,
                     sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr,
                     -sp,   cp*sr,          cp*cr);
}
double rotAngleDeg(const cv::Matx33d&R){
  double c=(R(0,0)+R(1,1)+R(2,2)-1.0)*0.5; c=std::max(-1.0,std::min(1.0,c)); return acos(c)*180.0/kPi;
}

std::vector<CamRow> loadCam(const std::string& path){
  std::ifstream f(path); if(!f) throw std::runtime_error("Cannot open camera CSV: "+path);
  std::string line; std::getline(f,line); std::vector<CamRow> out;
  bool have=false; uint32_t prev_seq=0; int64_t prev_ts=0,last_sel=0;
  while(std::getline(f,line)){
    auto c=split(line); if(c.size()<7) continue;
    CamRow s; s.seq=(uint32_t)std::stoul(c[0]); s.ts=i64v(c[2]); s.off=(uint64_t)std::stoull(c[5]); s.bytes=(size_t)std::stoull(c[6]);
    bool ok=true; if(have){ const int64_t dt=s.ts-prev_ts; ok=s.seq==prev_seq+1U && dt>0 && dt<=kReplayRawMaxDtNs; }
    prev_seq=s.seq;prev_ts=s.ts;have=true;
    const bool due=last_sel==0 || s.ts-last_sel>=kReplayCameraPeriodNs;
    if(ok&&due&&s.bytes>0){out.push_back(s);last_sel=s.ts;}
  }
  if(out.size()<2) throw std::runtime_error("Too few selected camera rows"); return out;
}
std::vector<AttRow> loadAtt(const std::string& path){
  std::ifstream f(path); if(!f) throw std::runtime_error("Cannot open ATTITUDE CSV: "+path);
  std::string line; std::getline(f,line); std::vector<AttRow> out;
  while(std::getline(f,line)){
    auto c=split(line); if(c.size()<12) continue;
    AttRow a; a.ts=i64v(c[2]); // source_timestamp_ns is FC boot clock; camera uses mapped RPi clock, use mapped below.
    a.ts=i64v(c[2]);
    // Column 2 is source; column 3 is mapped_rpi_ns. Use mapped time for camera correlation.
    a.ts=i64v(c[2]);
    if(c.size()>3) a.ts=i64v(c[2]);
    // Correct final assignment explicitly:
    a.ts=i64v(c[2]);
    out.push_back(a);
  }
  // Reload using header names to avoid column-order ambiguity.
  f.close(); std::ifstream g(path); std::getline(g,line); auto h=split(line);
  auto idx=[&](const std::string&n){auto it=std::find(h.begin(),h.end(),n); if(it==h.end()) throw std::runtime_error("Missing ATT column: "+n); return (size_t)(it-h.begin());};
  size_t it=idx("mapped_rpi_ns"),ir=idx("roll_deg"),ip=idx("pitch_deg"),iy=idx("yaw_deg"),irs=idx("rollspeed"),ips=idx("pitchspeed"),iys=idx("yawspeed");
  out.clear(); while(std::getline(g,line)){auto c=split(line); if(c.size()<=std::max({it,ir,ip,iy,irs,ips,iys})) continue; AttRow a; a.ts=i64v(c[it]);a.r=dv(c[ir]);a.p=dv(c[ip]);a.y=dv(c[iy]);a.rs=dv(c[irs]);a.ps=dv(c[ips]);a.ys=dv(c[iys]); if(a.ts>0) out.push_back(a);} 
  if(out.empty()) throw std::runtime_error("No ATTITUDE rows"); return out;
}
const AttRow& nearestAtt(const std::vector<AttRow>&a,int64_t t,size_t&hint){
  while(hint+1<a.size() && std::llabs(a[hint+1].ts-t)<=std::llabs(a[hint].ts-t)) ++hint; return a[hint];
}
cv::Mat decode(std::ifstream& jf,const CamRow&s){
  std::vector<uchar>b(s.bytes); jf.clear(); jf.seekg((std::streamoff)s.off,std::ios::beg); jf.read((char*)b.data(),(std::streamsize)s.bytes);
  if((size_t)jf.gcount()!=s.bytes) return {}; return cv::imdecode(b,cv::IMREAD_GRAYSCALE);
}

void summaryPhase(const std::string&name,const std::vector<PairMetric>&v){
  std::vector<double> raw,rot,res,p90,rat,hin,hm,ang; int tracks=0;
  for(const auto&m:v){raw.push_back(m.raw_med);rot.push_back(m.rot_med);res.push_back(m.residual_med);p90.push_back(m.residual_p90);rat.push_back(m.ratio);hin.push_back(m.h_inlier);hm.push_back(m.h_med);ang.push_back(m.fc_rot_deg);tracks+=m.tracks;}
  std::cout<<name<<" pairs="<<v.size(); if(v.empty()){std::cout<<"\n";return;}
  std::cout<<std::fixed<<std::setprecision(3)
           <<" median_tracks="<<(tracks/(double)v.size())
           <<" raw_flow="<<median(raw)<<"px rot_pred="<<median(rot)<<"px"
           <<" residual="<<median(res)<<"px p90="<<median(p90)<<"px"
           <<" residual/raw="<<median(rat)
           <<" H_inlier="<<median(hin)
           <<" H_err="<<median(hm)<<"px"
           <<" FC_pair_rot="<<median(ang)<<"deg\n";
}
}

int main(int argc,char**argv){
  const std::string cam=argc>1?argv[1]:"/home/vio/jtzero_yaw_only_v15_42_camera.csv";
  const std::string mjpg=argc>2?argv[2]:"/home/vio/jtzero_yaw_only_v15_42.mjpg";
  const std::string att=argc>3?argv[3]:"/home/vio/jtzero_yaw_only_v15_42_attitude.csv";
  const std::string out=argc>4?argv[4]:"/home/vio/jtzero_rotation_parallax_v15_45.csv";
  try{
    auto C=loadCam(cam); auto A=loadAtt(att); std::ifstream jf(mjpg,std::ios::binary); if(!jf) throw std::runtime_error("Cannot open MJPEG");
    const cv::Mat K=(cv::Mat_<double>(3,3)<<568.53170752165227,0,315.98271077441063,0,569.68005562865858,239.88148589100641,0,0,1);
    const cv::Mat D=(cv::Mat_<double>(1,5)<<0.073569192194028493,-0.095253893789117,-0.010810530757187299,-0.0022843373576970235,0.082177400802757483);
    const cv::Matx33d RBC(0.009367371,0.999954838,-0.001604448, 0.999326771,-0.009418381,-0.035458408, -0.035471918,-0.001271215,-0.999369865);
    std::ofstream csv(out); csv<<"t0_ns,t1_ns,dt_s,tracks,rotating,fc_pair_rot_deg,yaw_rate_deg_s,raw_flow_med_px,rot_pred_med_px,residual_med_px,residual_p90_px,residual_raw_ratio,homography_inlier_ratio,homography_med_err_px\n";
    std::vector<PairMetric> all,still,turn; size_t ah0=0,ah1=0;
    cv::Mat prev=decode(jf,C[0]); if(prev.empty()) throw std::runtime_error("First frame decode failed");
    for(size_t i=1;i<C.size();++i){
      cv::Mat cur=decode(jf,C[i]); if(cur.empty()){prev=cur;continue;} if(prev.empty()){prev=cur;continue;}
      std::vector<cv::Point2f> p0,p1; cv::goodFeaturesToTrack(prev,p0,350,0.01,7.0,cv::noArray(),7,false,0.04);
      if(p0.size()<20){prev=cur;continue;}
      std::vector<uchar>st;std::vector<float>err;cv::calcOpticalFlowPyrLK(prev,cur,p0,p1,st,err,cv::Size(21,21),3,cv::TermCriteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,30,0.01));
      std::vector<cv::Point2f> q0,q1; for(size_t k=0;k<p0.size();++k) if(st[k]&&p1[k].x>=0&&p1[k].x<cur.cols&&p1[k].y>=0&&p1[k].y<cur.rows){q0.push_back(p0[k]);q1.push_back(p1[k]);}
      if(q0.size()<20){prev=cur;continue;}
      std::vector<cv::Point2f> u0,u1; cv::undistortPoints(q0,u0,K,D,cv::noArray(),K); cv::undistortPoints(q1,u1,K,D,cv::noArray(),K);
      const auto&a0=nearestAtt(A,C[i-1].ts,ah0); const auto&a1=nearestAtt(A,C[i].ts,ah1);
      cv::Matx33d RWB0=rpy(a0.r,a0.p,a0.y),RWB1=rpy(a1.r,a1.p,a1.y); cv::Matx33d RC1C0=RBC.t()*RWB1.t()*RWB0*RBC;
      std::vector<double> rawf,rotf,res; rawf.reserve(u0.size());rotf.reserve(u0.size());res.reserve(u0.size());
      for(size_t k=0;k<u0.size();++k){
        cv::Vec3d x((u0[k].x-315.98271077441063)/568.53170752165227,(u0[k].y-239.88148589100641)/569.68005562865858,1.0); cv::Vec3d z=RC1C0*x; if(z[2]<=1e-6) continue;
        cv::Point2f pr((float)(568.53170752165227*z[0]/z[2]+315.98271077441063),(float)(569.68005562865858*z[1]/z[2]+239.88148589100641));
        rawf.push_back(cv::norm(u1[k]-u0[k])); rotf.push_back(cv::norm(pr-u0[k])); res.push_back(cv::norm(u1[k]-pr));
      }
      if(res.size()<20){prev=cur;continue;}
      cv::Mat mask,H=cv::findHomography(u0,u1,cv::RANSAC,1.5,mask,2000,0.995); std::vector<double> herr; int nin=0;
      if(!H.empty()){
        std::vector<cv::Point2f> hp; cv::perspectiveTransform(u0,hp,H);
        for(size_t k=0;k<hp.size();++k) if(mask.at<uchar>((int)k)){++nin;herr.push_back(cv::norm(hp[k]-u1[k]));}
      }
      PairMetric m; m.t0=C[i-1].ts;m.t1=C[i].ts;m.dt=(m.t1-m.t0)*1e-9;m.tracks=(int)res.size();m.fc_rot_deg=rotAngleDeg(RC1C0);m.yaw_rate_deg_s=0.5*(a0.ys+a1.ys)*180.0/kPi;m.rotating=std::abs(m.yaw_rate_deg_s)>2.0 || m.fc_rot_deg>0.08;m.raw_med=median(rawf);m.rot_med=median(rotf);m.residual_med=median(res);m.residual_p90=percentile(res,0.90);m.ratio=m.raw_med>0.05?m.residual_med/m.raw_med:0;m.h_inlier=u0.empty()?0:nin/(double)u0.size();m.h_med=median(herr);all.push_back(m);(m.rotating?turn:still).push_back(m);
      csv<<m.t0<<','<<m.t1<<','<<m.dt<<','<<m.tracks<<','<<(m.rotating?1:0)<<','<<m.fc_rot_deg<<','<<m.yaw_rate_deg_s<<','<<m.raw_med<<','<<m.rot_med<<','<<m.residual_med<<','<<m.residual_p90<<','<<m.ratio<<','<<m.h_inlier<<','<<m.h_med<<'\n';
      prev=cur;
    }
    std::cout<<"============================================================\nJT-ZERO v15.45 ROTATION / PARALLAX DIAGNOSTIC\n============================================================\n";
    std::cout<<"selected camera frames="<<C.size()<<" analyzed pairs="<<all.size()<<"\n"; summaryPhase("STILL",still);summaryPhase("ROTATION",turn);summaryPhase("ALL",all);
    std::vector<double> rr,hr;for(const auto&m:turn){if(m.raw_med>0.5)rr.push_back(m.ratio);hr.push_back(m.h_inlier);} double ratio=median(rr),hin=median(hr);
    std::string verdict="INSUFFICIENT_ROTATION_DATA"; if(turn.size()>=30){if(ratio<=0.35&&hin>=0.75)verdict="STRONGLY_ROTATION_DOMINATED_LOW_PARALLAX";else if(ratio<=0.60&&hin>=0.60)verdict="ROTATION_DOMINATED_LOW_PARALLAX_SUPPORTED";else verdict="SIGNIFICANT_NON_ROTATIONAL_PARALLAX_PRESENT";}
    std::cout<<"ROTATION_MEDIAN_RESIDUAL_RAW_RATIO="<<std::fixed<<std::setprecision(4)<<ratio<<"\nROTATION_MEDIAN_H_INLIER="<<hin<<"\nV15_45_VERDICT="<<verdict<<"\nCSV="<<out<<"\nRESULT: COMPLETE\n";return 0;
  }catch(const std::exception&e){std::cerr<<"[FATAL] "<<e.what()<<"\nRESULT: FAIL\n";return 1;}
}
