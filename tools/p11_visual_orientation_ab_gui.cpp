#include <opencv2/opencv.hpp>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
#include <chrono>

namespace fs = std::filesystem;

static std::string nowStamp() {
    auto t=std::time(nullptr); std::tm tm{}; localtime_r(&t,&tm);
    char b[32]; std::strftime(b,sizeof(b),"%Y%m%d_%H%M%S",&tm); return b;
}

int main(int argc,char**argv){
    const int cam = argc>1 ? std::atoi(argv[1]) : 0;
    cv::VideoCapture cap(cam, cv::CAP_V4L2);
    cap.set(cv::CAP_PROP_FRAME_WIDTH,640); cap.set(cv::CAP_PROP_FRAME_HEIGHT,480);
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));\n    cap.set(cv::CAP_PROP_FPS,100);\n    cap.set(cv::CAP_PROP_BUFFERSIZE,1);
    if(!cap.isOpened()){ std::cerr<<"[ОШИБКА] OV9281 не открыта.\n"; return 1; }

    const std::vector<std::string> stages={"A1","B1","A2","B2","A3","B3","A4"};
    size_t s=0; bool recording=false; int saved=0;
    const fs::path out=fs::path("/home/vio/jtzero_runs")/(nowStamp()+"_P11_VISUAL_ORIENTATION_AB_GUI");
    fs::create_directories(out);
    std::ofstream ev(out/"p11_visual_events.csv"); ev<<"event,stage,file\n";

    cv::namedWindow("P11: независимая визуальная проверка A/B",cv::WINDOW_NORMAL);
    cv::setWindowProperty("P11: независимая визуальная проверка A/B",cv::WND_PROP_FULLSCREEN,cv::WINDOW_FULLSCREEN);

    for(;;){
        cv::Mat frame; cap>>frame; if(frame.empty()) continue;
        cv::Mat view=frame.clone();
        cv::rectangle(view,{0,0},{view.cols,145},cv::Scalar(0,0,0),cv::FILLED);
        const std::string st=stages[s];
        cv::putText(view,"P11 — ВНЕШНЯЯ ПРОВЕРКА ОРИЕНТАЦИИ",{20,35},cv::FONT_HERSHEY_SIMPLEX,.72,cv::Scalar(255,255,255),2);
        cv::putText(view,"Позиция: "+st+"   Кадров: "+std::to_string(saved)+"/20",{20,72},cv::FONT_HERSHEY_SIMPLEX,.68,cv::Scalar(255,255,255),2);
        cv::putText(view,recording?"ИДЕТ ЗАПИСЬ — не перемещайте БПЛА":"Установите БПЛА в "+st+" и нажмите ПРОБЕЛ",{20,108},cv::FONT_HERSHEY_SIMPLEX,.62,recording?cv::Scalar(80,220,80):cv::Scalar(0,220,255),2);
        cv::putText(view,"ПРОБЕЛ — запись 20 кадров   Q/ESC — выход",{20,137},cv::FONT_HERSHEY_SIMPLEX,.50,cv::Scalar(210,210,210),1);
        cv::imshow("P11: независимая визуальная проверка A/B",view);

        int k=cv::waitKey(1);
        if(k==27||k=='q'||k=='Q') break;
        if(k==' '&&!recording){ recording=true; saved=0; ev<<"START,"<<st<<",\n"; }

        if(recording && saved<20){
            const auto fn=st+"_"+cv::format("%02d",saved)+".png";
            cv::imwrite((out/fn).string(),frame);
            ev<<"FRAME,"<<st<<","<<fn<<"\n"; ++saved;
            cv::waitKey(45);
            if(saved==20){
                ev<<"END,"<<st<<",\n"; ev.flush(); recording=false;
                if(s+1<stages.size()) ++s;
                else {
                    cv::Mat done(480,800,CV_8UC3,cv::Scalar(0,0,0));
                    cv::putText(done,"ТЕСТ ЗАВЕРШЕН",{120,180},cv::FONT_HERSHEY_SIMPLEX,1.3,cv::Scalar(80,220,80),3);
                    cv::putText(done,"Все 7 позиций записаны. Нажмите любую клавишу.",{45,250},cv::FONT_HERSHEY_SIMPLEX,.65,cv::Scalar(255,255,255),2);
                    cv::imshow("P11: независимая визуальная проверка A/B",done); cv::waitKey(0); break;
                }
            }
        }
    }
    std::cout<<"[ГОТОВО] "<<out<<"\n";
}
