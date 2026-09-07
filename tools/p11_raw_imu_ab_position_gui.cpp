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
static void txt(cv::Mat&i,std::string s,int y,double z=.75,int th=2){cv::putText(i,s,{30,y},cv::FONT_HERSHEY_SIMPLEX,z,{235,235,235},th,cv::LINE_AA);}
int main(){const char* L[]={"A1","B1","A2","B2","A3","B3","A4"};const char* P[]={"A","B","A","B","A","B","A"};const double SEC=10;
 std::string dir="/home/vio/jtzero_runs/"+stamp()+"_P11_RAW_IMU_AB_POSITION_GUI";fs::create_directories(dir);std::ofstream o(dir+"/p11_raw_imu.csv"),e(dir+"/p11_events.csv");
 o<<"recv_ns,time_usec,stage,position,recording,ax_frd,ay_frd,az_frd,ax_flu,ay_flu,az_flu,acc_norm,gx_frd,gy_frd,gz_frd,gx_flu,gy_flu,gz_flu,temperature\n";e<<"event_ns,event,stage,position\n";
 int f=serial();mavlink_message_t m{};mavlink_status_t st{};uint8_t sy=0,co=0;std::cout<<"[MAV] waiting for HEARTBEAT...\n";while(!sy){uint8_t b[1024];int n=read(f,b,sizeof(b));for(int j=0;j<n;j++)if(mavlink_parse_char(MAVLINK_COMM_0,b[j],&m,&st)&&m.msgid==MAVLINK_MSG_ID_HEARTBEAT){sy=m.sysid;co=m.compid;break;}usleep(10000);}rate(f,sy,co);
 cv::namedWindow("P11 RAW IMU A/B",cv::WINDOW_NORMAL);cv::resizeWindow("P11 RAW IMU A/B",1000,650);int s=0;bool rec=false,finished=false;int64_t t0=0;double ax=0,ay=0,az=0,gx=0,gy=0,gz=0,tp=0;uint64_t tu=0;long count=0;
 while(!finished){pollfd q{f,POLLIN,0};poll(&q,1,5);uint8_t b[4096];int n=read(f,b,sizeof(b));for(int j=0;j<n;j++)if(mavlink_parse_char(MAVLINK_COMM_0,b[j],&m,&st)&&m.msgid==MAVLINK_MSG_ID_HIGHRES_IMU){mavlink_highres_imu_t x{};mavlink_msg_highres_imu_decode(&m,&x);ax=x.xacc;ay=x.yacc;az=x.zacc;gx=x.xgyro;gy=x.ygyro;gz=x.zgyro;tp=x.temperature;tu=x.time_usec;count++;double an=sqrt(ax*ax+ay*ay+az*az);o<<ns()<<','<<tu<<','<<L[s]<<','<<P[s]<<','<<(rec?1:0)<<','<<std::setprecision(9)<<ax<<','<<ay<<','<<az<<','<<ax<<','<<-ay<<','<<-az<<','<<an<<','<<gx<<','<<gy<<','<<gz<<','<<gx<<','<<-gy<<','<<-gz<<','<<tp<<"\n";}
  double el=rec?(ns()-t0)/1e9:0;if(rec&&el>=SEC){rec=false;e<<ns()<<",PLATEAU_END,"<<L[s]<<','<<P[s]<<"\n";e.flush();o.flush();if(s==6)finished=true;else{s++;t0=0;}}
  cv::Mat im(650,1000,CV_8UC3,cv::Scalar(22,22,22));txt(im,"P11 RAW IMU: A/B POSITION CONTROL",55,.95);txt(im,"Stage "+std::to_string(s+1)+"/7   "+L[s]+"   POSITION "+P[s],115,1.0);
  if(rec){txt(im,"DO NOT TOUCH THE UAV",190,1.0,3);txt(im,"Recording: "+std::to_string(std::max(0,(int)ceil(SEC-el)))+" s remaining",240,.9);}
  else{txt(im,s==0?"Place UAV at A and leave it stationary.":"Movement finished. Leave UAV untouched.",190,.82);txt(im,"When fully stationary press SPACE.",240,.82);}
  std::ostringstream z;z<<std::fixed<<std::setprecision(4)<<"FRD acc=["<<ax<<", "<<ay<<", "<<az<<"]  |a|="<<sqrt(ax*ax+ay*ay+az*az)<<"  T="<<tp;txt(im,z.str(),330,.65,1);
  txt(im,"Protocol: A1 -> B1 -> A2 -> B2 -> A3 -> B3 -> A4",400,.7);if(!rec&&s<6)txt(im,std::string("After recording: move ")+P[s]+" -> "+P[s+1]+", then stop touching it.",450,.65,1);
  txt(im,"SPACE = record stationary plateau     ESC = abort",570,.68);cv::imshow("P11 RAW IMU A/B",im);int k=cv::waitKey(1)&255;if(k==27){e<<ns()<<",ABORT,"<<L[s]<<','<<P[s]<<"\n";break;}if(k==' '&&!rec){rec=true;t0=ns();e<<t0<<",PLATEAU_START,"<<L[s]<<','<<P[s]<<"\n";e.flush();}}
 o.flush();e.flush();close(f);cv::destroyAllWindows();std::ofstream l("/home/vio/jtzero_p11_latest_run.txt");l<<dir<<"\n";std::cout<<"[DONE] "<<dir<<"\n[DONE] samples="<<count<<"\n";}