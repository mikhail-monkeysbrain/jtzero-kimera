// JT-ZERO v15.48 offline projective-rank diagnostic for yaw dataset v15.42.
// Tests whether the small non-pure-rotation homography component is approximately
// rank-1, as expected for a planar translation term H ~ R + t*n^T/d.
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
constexpr int64_t kReplayCameraPeriodNs=30000000LL;
constexpr int64_t kReplayRawMaxDtNs=20000000LL;
constexpr double kPi=3.14159265358979323846;
struct CamRow{uint32_t seq=0;int64_t ts=0;uint64_t off=0;size_t bytes=0;};
struct AttRow{int64_t ts=0;double r=0,p=0,y=0,ys=0;};
struct M{double raw=0,herr=0,hin=0,fcres=0,bestres=0,s1=0,s2=0,s3=0,r21=0,r31=0,fcrot=0,bestrot=0;bool rotating=false;};
std::vector<std::string> split(const std::string&s){std::vector<std::string>o;std::string c;for(char ch:s){if(ch==','){o.push_back(c);c.clear();}else c.push_back(ch);}o.push_back(c);return o;}
long long i64v(const std::string&s){return s.empty()?0:std::stoll(s);} double dv(const std::string&s){return s.empty()?0:std::stod(s);} 
double median(std::vector<double>v){if(v.empty())return std::numeric_limits<double>::quiet_NaN();size_t n=v.size(),m=n/2;std::nth_element(v.begin(),v.begin()+m,v.end());double a=v[m];if(n%2)return a;std::nth_element(v.begin(),v.begin()+m-1,v.end());return .5*(a+v[m-1]);}
cv::Matx33d rpy(double r,double p,double y){r*=kPi/180;p*=kPi/180;y*=kPi/180;double cr=cos(r),sr=sin(r),cp=cos(p),sp=sin(p),cy=cos(y),sy=sin(y);return {cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr,sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr,-sp,cp*sr,cp*cr};}
double rotang(const cv::Mat&R){double c=(cv::trace(R)[0]-1.0)*.5;c=std::max(-1.0,std::min(1.0,c));return acos(c)*180/kPi;}
std::vector<CamRow> loadCam(const std::string&p){std::ifstream f(p);if(!f)throw std::runtime_error("Cannot open camera CSV");std::string l;std::getline(f,l);std::vector<CamRow>o;bool have=false;uint32_t ps=0;int64_t pt=0,last=0;while(std::getline(f,l)){auto c=split(l);if(c.size()<7)continue;CamRow s;s.seq=std::stoul(c[0]);s.ts=i64v(c[2]);s.off=std::stoull(c[5]);s.bytes=std::stoull(c[6]);bool ok=true;if(have){auto dt=s.ts-pt;ok=s.seq==ps+1U&&dt>0&&dt<=kReplayRawMaxDtNs;}ps=s.seq;pt=s.ts;have=true;bool due=last==0||s.ts-last>=kReplayCameraPeriodNs;if(ok&&due&&s.bytes){o.push_back(s);last=s.ts;}}return o;}
std::vector<AttRow> loadAtt(const std::string&p){std::ifstream f(p);if(!f)throw std::runtime_error("Cannot open attitude CSV");std::string l;std::getline(f,l);auto h=split(l);auto ix=[&](const std::string&n){auto it=std::find(h.begin(),h.end(),n);if(it==h.end())throw std::runtime_error("Missing ATT column "+n);return size_t(it-h.begin());};size_t it=ix("mapped_rpi_ns"),ir=ix("roll_deg"),ip=ix("pitch_deg"),iy=ix("yaw_deg"),iys=ix("yawspeed");std::vector<AttRow>o;while(std::getline(f,l)){auto c=split(l);if(c.size()<=std::max({it,ir,ip,iy,iys}))continue;o.push_back({i64v(c[it]),dv(c[ir]),dv(c[ip]),dv(c[iy]),dv(c[iys])});}return o;}
const AttRow& nearA(const std::vector<AttRow>&a,int64_t t,size_t&h){while(h+1<a.size()&&std::llabs(a[h+1].ts-t)<=std::llabs(a[h].ts-t))++h;return a[h];}
cv::Mat decode(std::ifstream&f,const CamRow&s){std::vector<uchar>b(s.bytes);f.clear();f.seekg((std::streamoff)s.off);f.read((char*)b.data(),s.bytes);if((size_t)f.gcount()!=s.bytes)return{};return cv::imdecode(b,cv::IMREAD_GRAYSCALE);}
cv::Mat nearestRotation(const cv::Mat&A){cv::SVD s(A,cv::SVD::FULL_UV);cv::Mat R=s.u*s.vt;if(cv::determinant(R)<0){cv::Mat U=s.u.clone();U.col(2)*=-1;R=U*s.vt;}return R;}
double pointResidual(const std::vector<cv::Point2f>&a,const std::vector<cv::Point2f>&b,const cv::Mat&H){std::vector<cv::Point2f>p;cv::perspectiveTransform(a,p,H);std::vector<double>e;for(size_t i=0;i<p.size();++i)e.push_back(cv::norm(p[i]-b[i]));return median(e);}
void sum(const char*n,const std::vector<M>&v){std::vector<double>raw,he,hi,fr,br,s1,s2,s3,r21,r31,fa,ba;for(auto&m:v){raw.push_back(m.raw);he.push_back(m.herr);hi.push_back(m.hin);fr.push_back(m.fcres);br.push_back(m.bestres);s1.push_back(m.s1);s2.push_back(m.s2);s3.push_back(m.s3);r21.push_back(m.r21);r31.push_back(m.r31);fa.push_back(m.fcrot);ba.push_back(m.bestrot);}std::cout<<n<<" pairs="<<v.size();if(v.empty()){std::cout<<"\n";return;}std::cout<<std::fixed<<std::setprecision(4)<<" raw="<<median(raw)<<"px FCres="<<median(fr)<<"px BESTres="<<median(br)<<"px H_err="<<median(he)<<"px H_inlier="<<median(hi)<<" sigma=["<<median(s1)<<","<<median(s2)<<","<<median(s3)<<"] s2/s1="<<median(r21)<<" s3/s1="<<median(r31)<<" FCrot="<<median(fa)<<"deg BESTrot="<<median(ba)<<"deg\n";}
}
int main(int argc,char**argv){
  std::string cam=argc>1?argv[1]:"/home/vio/jtzero_yaw_only_v15_42_camera.csv",mjpg=argc>2?argv[2]:"/home/vio/jtzero_yaw_only_v15_42.mjpg",att=argc>3?argv[3]:"/home/vio/jtzero_yaw_only_v15_42_attitude.csv",out=argc>4?argv[4]:"/home/vio/jtzero_projective_rank_v15_48.csv";
  try{
    auto C=loadCam(cam);auto A=loadAtt(att);if(C.size()<2||A.empty())throw std::runtime_error("insufficient data");std::ifstream jf(mjpg,std::ios::binary);if(!jf)throw std::runtime_error("Cannot open MJPEG");
    cv::Mat K=(cv::Mat_<double>(3,3)<<568.53170752165227,0,315.98271077441063,0,569.68005562865858,239.88148589100641,0,0,1),Ki=K.inv();
    cv::Mat D=(cv::Mat_<double>(1,5)<<0.073569192194028493,-0.095253893789117,-0.010810530757187299,-0.0022843373576970235,0.082177400802757483);
    cv::Matx33d RBCx(0.009367371,0.999954838,-0.001604448,0.999326771,-0.009418381,-0.035458408,-0.035471918,-0.001271215,-0.999369865);cv::Mat RBC(RBCx);
    std::ofstream csv(out);csv<<"t0_ns,t1_ns,rotating,raw_px,fc_res_px,best_rot_res_px,h_err_px,h_inlier,sigma1,sigma2,sigma3,s2_s1,s3_s1,fc_rot_deg,best_rot_deg\n";
    cv::Mat prev=decode(jf,C[0]);size_t ah0=0,ah1=0;std::vector<M>all,turn,still;
    for(size_t i=1;i<C.size();++i){cv::Mat cur=decode(jf,C[i]);if(prev.empty()||cur.empty()){prev=cur;continue;}std::vector<cv::Point2f>p0,p1;cv::goodFeaturesToTrack(prev,p0,350,.01,7);if(p0.size()<30){prev=cur;continue;}std::vector<uchar>st;std::vector<float>er;cv::calcOpticalFlowPyrLK(prev,cur,p0,p1,st,er,cv::Size(21,21),3,cv::TermCriteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,30,.01));std::vector<cv::Point2f>q0,q1;for(size_t k=0;k<p0.size();++k)if(st[k]&&p1[k].x>=0&&p1[k].x<cur.cols&&p1[k].y>=0&&p1[k].y<cur.rows){q0.push_back(p0[k]);q1.push_back(p1[k]);}if(q0.size()<30){prev=cur;continue;}std::vector<cv::Point2f>u0,u1;cv::undistortPoints(q0,u0,K,D,cv::noArray(),K);cv::undistortPoints(q1,u1,K,D,cv::noArray(),K);
      std::vector<double>rf;for(size_t k=0;k<u0.size();++k)rf.push_back(cv::norm(u1[k]-u0[k]));double raw=median(rf);cv::Mat mask;cv::Mat H=cv::findHomography(u0,u1,cv::RANSAC,1.5,mask,2000,.995);if(H.empty()){prev=cur;continue;}int nin=cv::countNonZero(mask);std::vector<cv::Point2f>hp;cv::perspectiveTransform(u0,hp,H);std::vector<double>he;for(size_t k=0;k<hp.size();++k)if(mask.at<uchar>((int)k))he.push_back(cv::norm(hp[k]-u1[k]));
      auto&a0=nearA(A,C[i-1].ts,ah0);auto&a1=nearA(A,C[i].ts,ah1);cv::Matx33d RWB0=rpy(a0.r,a0.p,a0.y),RWB1=rpy(a1.r,a1.p,a1.y);cv::Matx33d wrong=RBCx.t()*RWB1.t()*RWB0*RBCx;cv::Mat Rfc(cv::Matx33d(wrong.t()));
      cv::Mat N=Ki*H*K;double alpha=cv::trace(N.t()*Rfc)[0]/cv::sum(N.mul(N))[0];if(alpha<0)alpha=-alpha;cv::Mat As=alpha*N;cv::Mat Rbest=nearestRotation(As);cv::Mat E=As-Rbest;cv::SVD sv(E,cv::SVD::NO_UV);double s1=sv.w.at<double>(0),s2=sv.w.at<double>(1),s3=sv.w.at<double>(2);
      cv::Mat Hfc=K*Rfc*Ki,Hbest=K*Rbest*Ki;double fcres=pointResidual(u0,u1,Hfc),bestres=pointResidual(u0,u1,Hbest);double fcrot=rotang(Rfc),bestrot=rotang(Rbest);bool rotating=std::abs(.5*(a0.ys+a1.ys))*180/kPi>2.0||fcrot>.08;
      M m{raw,median(he),nin/(double)u0.size(),fcres,bestres,s1,s2,s3,s1>1e-12?s2/s1:0,s1>1e-12?s3/s1:0,fcrot,bestrot,rotating};all.push_back(m);(rotating?turn:still).push_back(m);csv<<C[i-1].ts<<','<<C[i].ts<<','<<(rotating?1:0)<<','<<raw<<','<<fcres<<','<<bestres<<','<<median(he)<<','<<nin/(double)u0.size()<<','<<s1<<','<<s2<<','<<s3<<','<<m.r21<<','<<m.r31<<','<<fcrot<<','<<bestrot<<'\n';prev=cur;
    }
    std::cout<<"============================================================\nJT-ZERO v15.48 PROJECTIVE RANK TEST\n============================================================\n";sum("STILL",still);sum("ROTATION",turn);sum("ALL",all);
    std::vector<double>r21,r31,hi,br,he;for(auto&m:turn){r21.push_back(m.r21);r31.push_back(m.r31);hi.push_back(m.hin);br.push_back(m.bestres);he.push_back(m.herr);}double a=median(r21),b=median(r31),h=median(hi),bp=median(br),hep=median(he);std::cout<<std::fixed<<std::setprecision(4)<<"ROTATION_MEDIAN_S2_S1="<<a<<"\nROTATION_MEDIAN_S3_S1="<<b<<"\nROTATION_MEDIAN_H_INLIER="<<h<<"\nROTATION_BESTROT_RESIDUAL_PX="<<bp<<"\nROTATION_H_RESIDUAL_PX="<<hep<<"\n";std::string v="PROJECTIVE_COMPONENT_NOT_RANK1";if(turn.size()<30)v="INSUFFICIENT_ROTATION_DATA";else if(h>.90&&a<.35&&b<.15)v="PROJECTIVE_COMPONENT_STRONGLY_RANK1_PLANE_LIKE";else if(h>.90&&a<.60)v="PROJECTIVE_COMPONENT_APPROX_RANK1";std::cout<<"V15_48_VERDICT="<<v<<"\nCSV="<<out<<"\nRESULT: COMPLETE\n";
  }catch(const std::exception&e){std::cerr<<"[FATAL] "<<e.what()<<"\n";return 1;}return 0;
}
