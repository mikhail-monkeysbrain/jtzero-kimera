#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "common/mavlink.h"

namespace fs=std::filesystem;
static constexpr double G0=9.80665;
static int64_t ns(){timespec t{};clock_gettime(CLOCK_MONOTONIC,&t);return int64_t(t.tv_sec)*1000000000LL+t.tv_nsec;}
static int serial(){int f=open("/dev/ttyAMA0",O_RDWR|O_NOCTTY|O_NONBLOCK);if(f<0){perror("serial");exit(2);}termios t{};tcgetattr(f,&t);cfmakeraw(&t);cfsetispeed(&t,B460800);cfsetospeed(&t,B460800);t.c_cflag|=CLOCAL|CREAD;t.c_cflag&=~CRTSCTS;tcsetattr(f,TCSANOW,&t);return f;}
static void sendMsg(int f,const mavlink_message_t&m){uint8_t b[MAVLINK_MAX_PACKET_LEN];auto n=mavlink_msg_to_send_buffer(b,&m);write(f,b,n);}
static void rate(int f,uint8_t s,uint8_t c,uint32_t id,int hz){mavlink_message_t m{};mavlink_msg_command_long_pack(255,190,&m,s,c,MAV_CMD_SET_MESSAGE_INTERVAL,0,id,1e6f/hz,0,0,0,0,0);sendMsg(f,m);}
static std::string stamp(){auto n=std::chrono::system_clock::now();auto tt=std::chrono::system_clock::to_time_t(n);std::tm t{};localtime_r(&tt,&t);char b[32];strftime(b,sizeof(b),"%Y%m%d_%H%M%S",&t);return b;}
static void txt(cv::Mat&i,const std::string&s,int x,int y,int px=28,const cv::Scalar& col=cv::Scalar(235,235,235),int th=1){cv::addText(i,s,{x,y},"DejaVu Sans",px,col,th,cv::LINE_AA,false);}
static void panel(cv::Mat&i,int x,int y,int w,int h,const cv::Scalar&fill,const cv::Scalar&border=cv::Scalar(90,90,90)){cv::rectangle(i,{x,y},{x+w,y+h},fill,cv::FILLED);cv::rectangle(i,{x,y},{x+w,y+h},border,2);}

int main(){
 const char* L[]={"A1","B1","A2","B2","A3","B3","A4"}; const char* P[]={"A","B","A","B","A","B","A"};
 constexpr int STAGES=7; const double SEC=10.0;
 std::string dir="/home/vio/jtzero_runs/"+stamp()+"_P11_IMU_PATH_AB_GUI"; fs::create_directories(dir);
 std::ofstream o(dir+"/p11_imu_path.csv"),e(dir+"/p11_events.csv");
 o<<"recv_ns,fc_time,stage,position,recording,type,ax_si,ay_si,az_si,acc_norm_si,temperature\n";
 e<<"event_ns,event,stage,position\n";
 int f=serial(); mavlink_message_t m{}; mavlink_status_t st{}; uint8_t sy=0,co=0;
 std::cout<<"[MAV] ожидание HEARTBEAT...\n";
 while(!sy){uint8_t b[2048];int n=read(f,b,sizeof(b));for(int j=0;j<n;j++)if(mavlink_parse_char(MAVLINK_COMM_0,b[j],&m,&st)&&m.msgid==MAVLINK_MSG_ID_HEARTBEAT){sy=m.sysid;co=m.compid;break;}usleep(10000);}
 rate(f,sy,co,MAVLINK_MSG_ID_RAW_IMU,50);
 rate(f,sy,co,MAVLINK_MSG_ID_SCALED_IMU,50);
 rate(f,sy,co,MAVLINK_MSG_ID_SCALED_IMU2,50);
 rate(f,sy,co,MAVLINK_MSG_ID_SCALED_IMU3,50);
 rate(f,sy,co,MAVLINK_MSG_ID_HIGHRES_IMU,50);

 const std::string win="P11: сравнение потоков IMU в точках A/B";
 cv::namedWindow(win,cv::WINDOW_NORMAL); cv::setWindowProperty(win,cv::WND_PROP_FULLSCREEN,cv::WINDOW_FULLSCREEN);
 int s=0; bool rec=false,finished=false; int64_t t0=0; long count=0;
 long cRaw=0,cS1=0,cS2=0,cS3=0,cHi=0;
 double lastNorm=0,lastTemp=0;

 auto write=[&](const char* typ,uint64_t fc,double ax,double ay,double az,double temp){
   double an=sqrt(ax*ax+ay*ay+az*az); lastNorm=an; if(std::isfinite(temp))lastTemp=temp;
   o<<ns()<<','<<fc<<','<<L[s]<<','<<P[s]<<','<<(rec?1:0)<<','<<typ<<','<<std::setprecision(10)<<ax<<','<<ay<<','<<az<<','<<an<<','<<temp<<"\n"; count++;
 };

 while(!finished){
  pollfd q{f,POLLIN,0};poll(&q,1,5);uint8_t b[8192];int n=read(f,b,sizeof(b));
  for(int j=0;j<n;j++) if(mavlink_parse_char(MAVLINK_COMM_0,b[j],&m,&st)){
    if(m.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){mavlink_highres_imu_t x{};mavlink_msg_highres_imu_decode(&m,&x);write("HIGHRES_IMU",x.time_usec,x.xacc,x.yacc,x.zacc,x.temperature);cHi++;}
    else if(m.msgid==MAVLINK_MSG_ID_RAW_IMU){mavlink_raw_imu_t x{};mavlink_msg_raw_imu_decode(&m,&x);double k=G0/1000.0;write("RAW_IMU",x.time_usec,x.xacc*k,x.yacc*k,x.zacc*k,NAN);cRaw++;}
    else if(m.msgid==MAVLINK_MSG_ID_SCALED_IMU){mavlink_scaled_imu_t x{};mavlink_msg_scaled_imu_decode(&m,&x);double k=G0/1000.0;write("SCALED_IMU",x.time_boot_ms,x.xacc*k,x.yacc*k,x.zacc*k,x.temperature/100.0);cS1++;}
    else if(m.msgid==MAVLINK_MSG_ID_SCALED_IMU2){mavlink_scaled_imu2_t x{};mavlink_msg_scaled_imu2_decode(&m,&x);double k=G0/1000.0;write("SCALED_IMU2",x.time_boot_ms,x.xacc*k,x.yacc*k,x.zacc*k,x.temperature/100.0);cS2++;}
    else if(m.msgid==MAVLINK_MSG_ID_SCALED_IMU3){mavlink_scaled_imu3_t x{};mavlink_msg_scaled_imu3_decode(&m,&x);double k=G0/1000.0;write("SCALED_IMU3",x.time_boot_ms,x.xacc*k,x.yacc*k,x.zacc*k,x.temperature/100.0);cS3++;}
  }

  double el=rec?(ns()-t0)/1e9:0;
  if(rec&&el>=SEC){rec=false;e<<ns()<<",PLATEAU_END,"<<L[s]<<','<<P[s]<<"\n";e.flush();o.flush();if(s==STAGES-1)finished=true;else{s++;t0=0;}}

  const int SW=1280,SH=720; cv::Mat im(SH,SW,CV_8UC3,cv::Scalar(20,22,26));
  panel(im,16,14,SW-32,82,cv::Scalar(31,35,42),cv::Scalar(65,70,80));
  txt(im,"P11 — сравнение MAVLink-потоков IMU",34,50,28);
  txt(im,"Проверяем, на каком уровне появляется разница A/B",34,78,18,cv::Scalar(180,185,195));
  for(int k=0;k<STAGES;k++){int x=610+k*86;cv::Scalar fill=(k<s)?cv::Scalar(55,115,70):(k==s?cv::Scalar(60,95,170):cv::Scalar(48,52,60));panel(im,x,31,68,38,fill,cv::Scalar(90,100,115));txt(im,L[k],x+18,58,21);}

  panel(im,24,116,800,392,cv::Scalar(27,30,36),cv::Scalar(75,82,95));
  txt(im,"СЕЙЧАС",48,154,20,cv::Scalar(160,170,185));txt(im,std::string("ТОЧКА ")+P[s],48,214,52,cv::Scalar(250,250,250),2);
  if(rec){txt(im,"ИДЁТ ЗАПИСЬ",48,270,26,cv::Scalar(100,215,255),2);txt(im,"НЕ ТРОГАЙТЕ СИСТЕМУ",48,316,34,cv::Scalar(90,90,255),2);int rem=std::max(0,(int)ceil(SEC-el));txt(im,"Осталось: "+std::to_string(rem)+" с",48,382,46,cv::Scalar(100,225,255),2);}
  else{if(s==0)txt(im,"1. Установите систему в точку A.",48,276,29);else txt(im,std::string("1. Переместите систему в точку ")+P[s]+".",48,276,29);txt(im,"2. Уберите руки.",48,326,29);txt(im,"3. Дождитесь полной неподвижности.",48,376,29);panel(im,48,410,700,64,cv::Scalar(45,105,70),cv::Scalar(75,145,95));txt(im,"4. Нажмите ПРОБЕЛ — запись 10 с.",70,451,25,cv::Scalar(245,245,245),2);}

  panel(im,24,528,800,104,cv::Scalar(31,35,42),cv::Scalar(65,70,80));txt(im,"ДАЛЬШЕ",48,560,18,cv::Scalar(160,170,185));if(s<STAGES-1)txt(im,std::string("После записи: ")+P[s]+" → "+P[s+1],48,598,26,cv::Scalar(235,235,240),2);else txt(im,"После этой записи тест завершится.",48,598,26,cv::Scalar(235,235,240),2);

  panel(im,846,116,410,516,cv::Scalar(27,30,36),cv::Scalar(75,82,95));txt(im,"ПОТОКИ IMU",870,154,22,cv::Scalar(190,200,215));
  txt(im,"RAW_IMU:      "+std::to_string(cRaw),870,208,20);txt(im,"SCALED_IMU:   "+std::to_string(cS1),870,246,20);txt(im,"SCALED_IMU2:  "+std::to_string(cS2),870,284,20);txt(im,"SCALED_IMU3:  "+std::to_string(cS3),870,322,20);txt(im,"HIGHRES_IMU:  "+std::to_string(cHi),870,360,20);
  std::ostringstream nn,tt;nn<<std::fixed<<std::setprecision(4)<<"Последний |a|: "<<lastNorm<<" м/с²";tt<<std::fixed<<std::setprecision(2)<<"Температура: "<<lastTemp<<" °C";txt(im,nn.str(),870,420,22,cv::Scalar(120,220,250),2);txt(im,tt.str(),870,458,20,cv::Scalar(195,200,210));
  txt(im,"Это НЕ прямой ADC-тест.",870,520,19,cv::Scalar(185,190,200),2);txt(im,"Сравниваются MAVLink-потоки",870,552,19,cv::Scalar(165,170,180));txt(im,"и доступные IMU-инстансы.",870,584,19,cv::Scalar(165,170,180));

  panel(im,24,650,1232,52,cv::Scalar(31,35,42),cv::Scalar(65,70,80));txt(im,"ПРОБЕЛ — запись",48,684,20,cv::Scalar(120,235,160),2);txt(im,"ESC — выход",340,684,20,cv::Scalar(120,160,245),2);txt(im,"A1 → B1 → A2 → B2 → A3 → B3 → A4",620,684,18,cv::Scalar(175,180,190));
  cv::imshow(win,im); int k=cv::waitKey(1)&255; if(k==27){e<<ns()<<",ABORT,"<<L[s]<<','<<P[s]<<"\n";break;} if(k==' '&&!rec){rec=true;t0=ns();e<<t0<<",PLATEAU_START,"<<L[s]<<','<<P[s]<<"\n";e.flush();}
 }
 o.flush();e.flush();close(f);cv::destroyAllWindows();std::ofstream l("/home/vio/jtzero_p11_latest_imu_path_run.txt");l<<dir<<"\n";
 std::cout<<"[ГОТОВО] "<<dir<<"\n[ПОТОКИ] RAW="<<cRaw<<" S1="<<cS1<<" S2="<<cS2<<" S3="<<cS3<<" HIGHRES="<<cHi<<"\n";return 0;
}