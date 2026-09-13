// JT-Zero — camera-only recorder for deterministic OpticalFlow replay.
// Records every OV9281 MJPEG frame and its V4L2 timestamp. No FC/TF-Luna.
#define JTZERO_OPTFLOW_LIBRARY
#include "optical_flow_mavlink_mvp_v2.cpp"
#include <filesystem>
#include <fstream>

int main(int argc,char**argv){
  if(argc<3){
    std::cerr<<"Использование: "<<argv[0]<<" <camera> <out_dir> [seconds]\n";
    return 2;
  }
  const std::string camdev=argv[1];
  const std::filesystem::path out=argv[2];
  const double seconds=(argc>3)?std::stod(argv[3]):20.0;
  if(!(seconds>=5.0&&seconds<=120.0)){std::cerr<<"seconds должен быть 5..120\n";return 2;}
  try{
    std::filesystem::create_directories(out);
    std::ofstream bin(out/"frames.mjpgbin",std::ios::binary|std::ios::trunc);
    std::ofstream csv(out/"frames.csv",std::ios::trunc);
    if(!bin||!csv) throw std::runtime_error("не удалось создать dataset");
    csv<<"frame,camera_ts_ns,offset_bytes,size_bytes\n";

    Camera cam; cam.openDev(camdev);
    std::signal(SIGINT,onSignal); std::signal(SIGTERM,onSignal);
    const int64_t start=monoNs();
    uint64_t frame=0,bytes=0;
    while(g_running && (monoNs()-start)*1e-9<seconds){
      pollfd p{cam.fd,POLLIN,0};
      int pr=poll(&p,1,100);
      if(pr<0){if(errno==EINTR)continue;fail("camera poll");}
      if(pr<=0)continue;
      for(;;){
        v4l2_buffer b{}; b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;
        if(xioctl(cam.fd,VIDIOC_DQBUF,&b)<0){
          if(errno==EAGAIN)break;
          fail("VIDIOC_DQBUF");
        }
        const int64_t ts=(int64_t)b.timestamp.tv_sec*1000000000LL+(int64_t)b.timestamp.tv_usec*1000LL;
        const uint64_t off=(uint64_t)bin.tellp();
        bin.write(reinterpret_cast<const char*>(cam.bufs[b.index].p),(std::streamsize)b.bytesused);
        if(!bin) throw std::runtime_error("ошибка записи frames.mjpgbin");
        csv<<frame<<','<<ts<<','<<off<<','<<b.bytesused<<'\n';
        bytes+=b.bytesused; ++frame;
        if(xioctl(cam.fd,VIDIOC_QBUF,&b)<0)fail("VIDIOC_QBUF");
      }
      if(frame%200==0) std::cerr<<"REC frames="<<frame<<" bytes="<<bytes<<"\r"<<std::flush;
    }
    csv.flush();bin.flush();
    std::cerr<<"\nЗапись завершена: "<<out<<" frames="<<frame<<" bytes="<<bytes<<"\n";
    return 0;
  }catch(const std::exception&e){
    std::cerr<<"ОШИБКА: "<<e.what()<<"\n";return 1;
  }
}
