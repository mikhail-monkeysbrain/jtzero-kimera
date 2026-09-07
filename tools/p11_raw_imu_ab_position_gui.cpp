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
static int64_t ns(){timespec t{};clock_gettime(CLOCK_MONOTONIC,&t);return int64_t(t.tv_sec)*1000000000LL+t.tv_nsec;}
static int serial(){int f=open("/dev/ttyAMA0",O_RDWR|O_NOCTTY|O_NONBLOCK);if(f<0){perror("serial");exit(2);}termios t{};tcgetattr(f,&t);cfmakeraw(&t);cfsetispeed(&t,B460800);cfsetospeed(&t,B460800);t.c_cflag|=CLOCAL|CREAD;t.c_cflag&=~CRTSCTS;tcsetattr(f,TCSANOW,&t);return f;}
static void rate(int f,uint8_t s,uint8_t c){mavlink_message_t m{};mavlink_msg_command_long_pack(255,190,&m,s,c,MAV_CMD_SET_MESSAGE_INTERVAL,0,MAVLINK_MSG_ID_HIGHRES_IMU,5000,0,0,0,0,0);uint8_t b[MAVLINK_MAX_PACKET_LEN];auto n=mavlink_msg_to_send_buffer(b,&m);write(f,b,n);}
static std::string stamp(){auto n=std::chrono::system_clock::now();auto tt=std::chrono::system_clock::to_time_t(n);std::tm t{};localtime_r(&tt,&t);char b[32];strftime(b,sizeof(b),"%Y%m%d_%H%M%S",&t);return b;}
static void txt(cv::Mat& i, const std::string& s, int x, int y, int px=28, const cv::Scalar& col=cv::Scalar(235,235,235), int th=1){
  cv::addText(i,s,{x,y},"DejaVu Sans",px,col,th,cv::LINE_AA,false);
}
static void panel(cv::Mat& i, int x, int y, int w, int h, const cv::Scalar& fill, const cv::Scalar& border=cv::Scalar(90,90,90)){
  cv::rectangle(i,{x,y},{x+w,y+h},fill,cv::FILLED);
  cv::rectangle(i,{x,y},{x+w,y+h},border,2);
}
static void centerTxt(cv::Mat& i, const std::string& s, int cx, int y, int px, const cv::Scalar& col=cv::Scalar(245,245,245), int th=1){
  // Qt text extent is not exposed consistently; this approximation is good enough for centered HUD labels.
  int approx=static_cast<int>(s.size()*px*0.54);
  txt(i,s,std::max(20,cx-approx/2),y,px,col,th);
}
int main(){const char* L[]={"A1","B1","A2","B2","A3","B3","A4"};const char* P[]={"A","B","A","B","A","B","A"};constexpr int STAGES=7;const double SEC=10;
 std::string dir="/home/vio/jtzero_runs/"+stamp()+"_P11_RAW_IMU_AB_POSITION_GUI";fs::create_directories(dir);std::ofstream o(dir+"/p11_raw_imu.csv"),e(dir+"/p11_events.csv");
 o<<"recv_ns,time_usec,stage,position,recording,ax_frd,ay_frd,az_frd,ax_flu,ay_flu,az_flu,acc_norm,gx_frd,gy_frd,gz_frd,gx_flu,gy_flu,gz_flu,temperature\n";e<<"event_ns,event,stage,position\n";
 int f=serial();mavlink_message_t m{};mavlink_status_t st{};uint8_t sy=0,co=0;std::cout<<"[MAV] ожидание HEARTBEAT...\n";while(!sy){uint8_t b[1024];int n=read(f,b,sizeof(b));for(int j=0;j<n;j++)if(mavlink_parse_char(MAVLINK_COMM_0,b[j],&m,&st)&&m.msgid==MAVLINK_MSG_ID_HEARTBEAT){sy=m.sysid;co=m.compid;break;}usleep(10000);}rate(f,sy,co);
 cv::namedWindow("P11: проверка IMU в точках A/B",cv::WINDOW_NORMAL);
cv::setWindowProperty("P11: проверка IMU в точках A/B",cv::WND_PROP_FULLSCREEN,cv::WINDOW_FULLSCREEN);int s=0;bool rec=false,finished=false;int64_t t0=0;double ax=0,ay=0,az=0,gx=0,gy=0,gz=0,tp=0;uint64_t tu=0;long count=0;
 while(!finished){pollfd q{f,POLLIN,0};poll(&q,1,5);uint8_t b[4096];int n=read(f,b,sizeof(b));for(int j=0;j<n;j++)if(mavlink_parse_char(MAVLINK_COMM_0,b[j],&m,&st)&&m.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){mavlink_highres_imu_t x{};mavlink_msg_highres_imu_decode(&m,&x);ax=x.xacc;ay=x.yacc;az=x.zacc;gx=x.xgyro;gy=x.ygyro;gz=x.zgyro;tp=x.temperature;tu=x.time_usec;count++;double an=sqrt(ax*ax+ay*ay+az*az);o<<ns()<<','<<tu<<','<<L[s]<<','<<P[s]<<','<<(rec?1:0)<<','<<std::setprecision(9)<<ax<<','<<ay<<','<<az<<','<<ax<<','<<-ay<<','<<-az<<','<<an<<','<<gx<<','<<gy<<','<<gz<<','<<gx<<','<<-gy<<','<<-gz<<','<<tp<<"\n";}
  double el=rec?(ns()-t0)/1e9:0;if(rec&&el>=SEC){rec=false;e<<ns()<<",PLATEAU_END,"<<L[s]<<','<<P[s]<<"\n";e.flush();o.flush();if(s==6)finished=true;else{s++;t0=0;}}
  const int SW=1280, SH=720;
  cv::Mat im(SH,SW,CV_8UC3,cv::Scalar(20,22,26));

  // Верхняя панель.
  panel(im,16,14,SW-32,82,cv::Scalar(31,35,42),cv::Scalar(65,70,80));
  txt(im,"P11 — проверка IMU в точках A/B",34,50,28);
  txt(im,"Этап "+std::to_string(s+1)+" из 7",34,78,18,cv::Scalar(180,185,195));
  for(int k=0;k<STAGES;k++){
    int x=610+k*86;
    cv::Scalar fill=(k<s)?cv::Scalar(55,115,70):(k==s?cv::Scalar(60,95,170):cv::Scalar(48,52,60));
    panel(im,x,31,68,38,fill,cv::Scalar(90,100,115));
    txt(im,L[k],x+18,58,21);
  }

  // Главная инструкция.
  panel(im,24,116,800,392,cv::Scalar(27,30,36),cv::Scalar(75,82,95));
  txt(im,"СЕЙЧАС",48,154,20,cv::Scalar(160,170,185));
  txt(im,std::string("ТОЧКА ")+P[s],48,214,52,cv::Scalar(250,250,250),2);

  if(rec){
    txt(im,"ИДЁТ ЗАПИСЬ",48,270,26,cv::Scalar(100,215,255),2);
    txt(im,"НЕ ТРОГАЙТЕ БПЛА",48,316,34,cv::Scalar(90,90,255),2);
    int remain=std::max(0,(int)ceil(SEC-el));
    txt(im,"Осталось: "+std::to_string(remain)+" с",48,382,46,cv::Scalar(100,225,255),2);
    txt(im,"После записи переместите систему в следующую точку.",48,442,22,cv::Scalar(190,195,205));
  }else{
    if(s==0){
      txt(im,"1. Установите БПЛА в точку A.",48,276,29);
    }else{
      txt(im,std::string("1. Переместите систему в точку ")+P[s]+".",48,276,29);
    }
    txt(im,"2. Уберите руки.",48,326,29);
    txt(im,"3. Дождитесь полной неподвижности.",48,376,29);
    panel(im,48,410,700,64,cv::Scalar(45,105,70),cv::Scalar(75,145,95));
    txt(im,"4. Нажмите ПРОБЕЛ — начнётся запись 10 с.",70,451,25,cv::Scalar(245,245,245),2);
  }

  // Что будет дальше.
  panel(im,24,528,800,104,cv::Scalar(31,35,42),cv::Scalar(65,70,80));
  txt(im,"ДАЛЬШЕ",48,560,18,cv::Scalar(160,170,185));
  if(s<STAGES-1){
    txt(im,std::string("После этой записи: ")+P[s]+" → "+P[s+1],48,598,26,cv::Scalar(235,235,240),2);
  }else{
    txt(im,"После этой записи тест завершится.",48,598,26,cv::Scalar(235,235,240),2);
  }

  // Правая диагностическая панель.
  panel(im,846,116,410,516,cv::Scalar(27,30,36),cv::Scalar(75,82,95));
  txt(im,"ДАННЫЕ IMU",870,154,22,cv::Scalar(190,200,215));
  std::ostringstream a1,a2,a3,a4,gt;
  a1<<std::fixed<<std::setprecision(4)<<"ax  "<<ax<<" м/с²";
  a2<<std::fixed<<std::setprecision(4)<<"ay  "<<ay<<" м/с²";
  a3<<std::fixed<<std::setprecision(4)<<"az  "<<az<<" м/с²";
  a4<<std::fixed<<std::setprecision(4)<<"|a| "<<sqrt(ax*ax+ay*ay+az*az)<<" м/с²";
  gt<<std::fixed<<std::setprecision(2)<<"Температура  "<<tp<<" °C";
  txt(im,a1.str(),870,214,24);
  txt(im,a2.str(),870,258,24);
  txt(im,a3.str(),870,302,24);
  txt(im,a4.str(),870,356,27,cv::Scalar(120,220,250),2);
  txt(im,gt.str(),870,410,22,cv::Scalar(195,200,210));
  cv::line(im,{870,442},{1230,442},cv::Scalar(70,75,85),1);
  txt(im,"Источник: FC HIGHRES_IMU",870,480,19,cv::Scalar(165,170,180));
  txt(im,"Kimera и камера выключены",870,512,19,cv::Scalar(165,170,180));
  if(rec){
    txt(im,"Во время записи:",870,558,20,cv::Scalar(205,210,220));
    txt(im,"не касайтесь системы.",870,590,22,cv::Scalar(225,225,230),2);
  }else{
    txt(im,"Сейчас систему можно",870,558,20,cv::Scalar(205,210,220));
    txt(im,"перемещать.",870,590,22,cv::Scalar(120,220,250),2);
  }

  // Нижняя строка управления.
  panel(im,24,650,1232,52,cv::Scalar(31,35,42),cv::Scalar(65,70,80));
  txt(im,"ПРОБЕЛ — запись",48,684,20,cv::Scalar(120,235,160),2);
  txt(im,"ESC — выход",340,684,20,cv::Scalar(120,160,245),2);
  txt(im,"A1 → B1 → A2 → B2 → A3 → B3 → A4",620,684,18,cv::Scalar(175,180,190));

  cv::imshow("P11: проверка IMU в точках A/B",im);int k=cv::waitKey(1)&255;if(k==27){e<<ns()<<",ABORT,"<<L[s]<<','<<P[s]<<"\n";break;}if(k==' '&&!rec){rec=true;t0=ns();e<<t0<<",PLATEAU_START,"<<L[s]<<','<<P[s]<<"\n";e.flush();}}
 o.flush();e.flush();close(f);cv::destroyAllWindows();std::ofstream l("/home/vio/jtzero_p11_latest_run.txt");l<<dir<<"\n";std::cout<<"[ГОТОВО] "<<dir<<"\n[ГОТОВО] сэмплов="<<count<<"\n";}