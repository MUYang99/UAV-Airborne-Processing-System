#ifndef _CONTROL3_H_
#define _CONTROL3_H_

#include<opencv2/opencv.hpp>
#include<iomanip>
using namespace cv;
using namespace std;

class Control3
{
public:
    Control3();
    Control3(double cameraAngle,double normalHeight,double distance,double normalV,double normalV_,double normalHeightV,double normalW,double squareSize);
    ~Control3();

    double cameraAngle;   //摄像头纵向视角
    double normalHeight;  //正常飞行高度
    double distance;      //绕障碍圈旁矩形做圆周运动的半径
    double normalV,normalV_,normalHeightV;//粗糙移动时前后或左右的最大速度
    double normalW;         //穿障碍圈时寻找障碍圈的最大角速度
    double forwardV,leftrV,heightV; //粗糙移动时和停机坪降落时的速度
    double forwardV_,leftrV_,yawV_,heightV_; //穿障碍圈时的速度
    int count1,count2,count_,counti; //计数器
    vector<Rect> squares;  //穿障碍圈时检测到的矩形
    Rect square; //矩形框
    bool isFirstTakeoff;
    double squareSize;

    int roughMove(double dx,double dy); //粗糙移动时计算速度的函数，返回值为50Hz时的发送次数
    void getVelocity();                 //停机坪降落时计算速度的函数guidance
    void getVelocity1();
    double centerX;
    double centerY;
    void getVelocityo();                //gopro
    void getVelocity_(bool isClockwise); //穿障碍圈时计算速度的函数
};

#endif
