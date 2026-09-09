// JT-Zero R1 deterministic replay validator.
// Standalone: no V25/V43/Kimera dependencies.
#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

struct Frame {
  uint64_t id=0, seq=0, vsec=0, vusec=0, recv=0, off=0, bytes=0;
};
struct Range {
  uint64_t id=0, recv=0;
  int dist=0, strength=0, temp=0, valid=0;
};

static std::vector<std::string> split(const std::string& s) {
  std::vector<std::string> v; std::stringstream ss(s); std::string x;
  while(std::getline(ss,x,',')) v.push_back(x);
  return v;
}
static uint64_t u64(const std::string& s){ return std::stoull(s); }
static int i32(const std::string& s){ return std::stoi(s); }

static uint64_t fnv1a(uint64_t h,const void* p,size_t n){
  const auto* b=static_cast<const unsigned char*>(p);
  for(size_t i=0;i<n;i++){ h^=b[i]; h*=1099511628211ULL; }
  return h;
}
template<class T> static uint64_t hashv(uint64_t h,const T& v){ return fnv1a(h,&v,sizeof(v)); }

int main(int argc,char** argv){
  try{
    const std::string dir=argc>1?argv[1]:"/home/vio/r1_smoke";
    const std::string fp=dir+"/frames.csv", mp=dir+"/frames.mjpg", rp=dir+"/range.csv";
    std::ifstream fc(fp), rc(rp), mj(mp,std::ios::binary);
    if(!fc||!rc||!mj) throw std::runtime_error("missing frames.csv / frames.mjpg / range.csv");

    std::string line; std::getline(fc,line);
    if(line!="frame_id,sequence,v4l2_sec,v4l2_usec,recv_mono_ns,offset,bytes")
      throw std::runtime_error("unexpected frames.csv schema");
    std::vector<Frame> frames;
    while(std::getline(fc,line)){
      if(line.empty()) continue; auto a=split(line); if(a.size()!=7) throw std::runtime_error("bad frame row");
      frames.push_back({u64(a[0]),u64(a[1]),u64(a[2]),u64(a[3]),u64(a[4]),u64(a[5]),u64(a[6])});
    }

    std::getline(rc,line);
    if(line!="sample_id,recv_mono_ns,distance_cm,strength,temp_raw,valid")
      throw std::runtime_error("unexpected range.csv schema");
    std::vector<Range> ranges;
    while(std::getline(rc,line)){
      if(line.empty()) continue; auto a=split(line); if(a.size()!=6) throw std::runtime_error("bad range row");
      ranges.push_back({u64(a[0]),u64(a[1]),i32(a[2]),i32(a[3]),i32(a[4]),i32(a[5])});
    }
    if(frames.empty()||ranges.empty()) throw std::runtime_error("empty dataset");

    const uint64_t mj_size=std::filesystem::file_size(mp);
    uint64_t expected=0, broken=0, seq_breaks=0, ts_back=0;
    uint64_t h=1469598103934665603ULL;
    std::vector<char> payload;
    for(size_t k=0;k<frames.size();k++){
      const auto& f=frames[k];
      if(f.off!=expected) broken++;
      if(k && f.seq!=frames[k-1].seq+1) seq_breaks++;
      if(k && f.recv<=frames[k-1].recv) ts_back++;
      if(f.off+f.bytes>mj_size) throw std::runtime_error("frame payload outside MJPEG file");
      payload.resize(f.bytes); mj.seekg(static_cast<std::streamoff>(f.off)); mj.read(payload.data(),static_cast<std::streamsize>(f.bytes));
      if(static_cast<uint64_t>(mj.gcount())!=f.bytes) throw std::runtime_error("short MJPEG read");
      if(f.bytes<4 || (unsigned char)payload[0]!=0xff || (unsigned char)payload[1]!=0xd8)
        throw std::runtime_error("frame does not start with JPEG SOI");
      h=hashv(h,f.id); h=hashv(h,f.seq); h=hashv(h,f.recv); h=hashv(h,f.bytes); h=fnv1a(h,payload.data(),payload.size());
      expected=f.off+f.bytes;
    }
    if(expected!=mj_size) throw std::runtime_error("indexed MJPEG end != file size");
    if(broken) throw std::runtime_error("broken MJPEG offset chain");

    uint64_t range_back=0, valid=0; long double sum=0;
    for(size_t k=0;k<ranges.size();k++){
      const auto& r=ranges[k]; if(k && r.recv<=ranges[k-1].recv) range_back++;
      if(r.valid){ valid++; sum+=r.dist; }
      h=hashv(h,r.id); h=hashv(h,r.recv); h=hashv(h,r.dist); h=hashv(h,r.strength); h=hashv(h,r.temp); h=hashv(h,r.valid);
    }
    if(ts_back||range_back) throw std::runtime_error("non-monotonic receive timestamp");
    if(!valid) throw std::runtime_error("no valid TF-Luna samples");

    std::cout<<"R1 4/6 — STANDALONE DETERMINISTIC REPLAY\n";
    std::cout<<"dataset="<<dir<<"\n";
    std::cout<<"frames="<<frames.size()<<" mjpeg_bytes="<<mj_size
             <<" seq_breaks="<<seq_breaks<<" frame_ts_back="<<ts_back<<"\n";
    std::cout<<"range_samples="<<ranges.size()<<" valid_range="<<valid
             <<" mean_distance_cm="<<std::fixed<<std::setprecision(6)<<(double)(sum/valid)
             <<" range_ts_back="<<range_back<<"\n";
    std::cout<<std::hex<<std::setfill('0')<<"replay_digest="<<std::setw(16)<<h<<std::dec<<"\n";
    std::cout<<"R1 REPLAY PASS\n";
    return 0;
  }catch(const std::exception& e){
    std::cerr<<"R1 REPLAY FAIL: "<<e.what()<<"\n"; return 1;
  }
}
