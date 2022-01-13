#include"Control3.h"

Control3::Control3()
{
}
Control3::Control3(double cameraAngle,double normalHeight,
                   double distance,double normalV,
                   double normalV_,double normalHeightV,
                   double normalW,double squareSize)
{
    this->cameraAngle = cameraAngle;
    this->normalHeight = normalHeight;
    this->distance = distance;
    this->normalV = normalV;
    this->normalV_ = normalV_;
    this->normalHeightV = normalHeightV;
    this->normalW = normalW;
    this->squareSize = squareSize;
    forwardV = leftrV = heightV = 0;
    forwardV_ = leftrV_ = yawV_ = heightV_ = 0;
    count1 = 1; 
    count2 = 1;
    count_ = 1;
    counti = 0;
    isFirstTakeoff = true;
}
Control3::~Control3()
{
}
int Control3::roughMove(double dx,double dy)
{
    if(dx> abs(dy))
    {
        forwardV = normalV;
        if(dy>0)
                leftrV = abs(dy)/dx*forwardV;
        if(dy<=0)
                leftrV = -abs(dy)/dx*forwardV;
    }
    if(dx<= abs(dy))
    {
        if(dy>0)
                leftrV = normalV;
        if(dy<=0)
                leftrV = -normalV;
        forwardV = dx/abs(dy)*normalV;
    }
    int circleTimes = dx/forwardV/0.02/2;
    return circleTimes;
}
void Control3::getVelocity()
{
    double centerX = square.x+square.width/2;
    double centerY = square.y+square.height/2;
    if(centerX-160<=50&&centerX-160>=-50)
        leftrV = (centerX - 160)/50.0*normalV_;
    if(centerX-160<-50)
        leftrV = -normalV_;
    if(centerX-160>50)
        leftrV = normalV_;
    if(centerY-120<=50&&centerY-120>=-50)
        forwardV = -(centerY - 120)/50.0*normalV_;
    if(centerY-120<-50)
        forwardV = normalV_;
    if(centerY-120>50)
        forwardV = -normalV_;
}
//guidance 320 240
void Control3::getVelocity1()
{
    if(abs(centerX-340) <= 50)//180 -160 = 20 offset left camera
        leftrV = (centerX - 340)/50.0*normalV_;
    if(centerX-340<-50)
        leftrV = -normalV_;
    if(centerX-340>50) 
        leftrV = normalV_;

    if(centerY-240<=50 && centerY-240>=-50)
        forwardV = -(centerY - 240)/50.0*normalV_;
    if(centerY-240<-50)
        forwardV = normalV_; 
    if(centerY-240>50)
        forwardV = -normalV_;
}
//gopro 640 480
void Control3::getVelocityo()
{
    double centerX = square.x + square.width/2;
    double centerY = square.y + square.height/2;
    if(abs(centerX - 290) <= 50)// guidance image is left align:240 = 320 -80
        leftrV = (centerX - 290)/50.0*normalV_;
    else if(centerX-290 <- 50)
        leftrV = -normalV_;
    else if(centerX-290 > 50)
        leftrV = normalV_;

    if(centerY-240<=50&&centerY-240>=-50)
        forwardV = -(centerY - 240)/50.0*normalV_;
    else if(centerY-240<-50)
        forwardV = normalV_;
    else if(centerY-240>50)
        forwardV = -normalV_;
}
void Control3::getVelocity_(bool isClockwise)
{
    forwardV_ = leftrV_ = yawV_ = heightV_ = 0;
    /*-----摄像头对准矩形-----*/
    if(count_ == 1)
    {
        if(abs(square.x+square.width/2-320)<=15&&abs(square.y+square.height/2-240)<=10)
            count_=2;
        else
        {
            if(abs(square.x+square.width/2-320)>200)
                yawV_ = (square.x+square.width/2-320)/abs(square.x+square.width/2-320)*normalW;
            else
                yawV_ = (square.x+square.width/2-320)/200.0*normalW;
            heightV_ = -(square.y+square.height/2-240)/100.0*0.2;
        }
    }
    /*-----调整飞机到矩形的距离为distance-----*/
    if(count_ == 2)
    {
        double rectHeight = squareSize*240.0/(tan(cameraAngle/2)*distance);
        if(abs(square.x+square.width/2-320)<=20&&abs(square.height -rectHeight)<=10&&abs(square.y+square.height/2-240)<=10)
            count_ = 3;
        else
        {
            leftrV_ = (square.x+square.width/2-320)/100.0*0.2;
            forwardV_ = (rectHeight - square.height)/100.0*0.3;
            heightV_ = -(square.y+square.height/2-240)/100.0*0.2;
        }
    }
    /*-----以distance为半径做圆周运动-----*/
    if(count_ ==3)
    {
        if(abs(square.width-square.height*360.0/480)<=3&&abs(square.y+square.height/2-240)<=20)
            count_ = 4;
        else
        {
            heightV_ = -(square.y+square.height/2-240)/100.0*0.2;
            if(isClockwise)
            {
                leftrV_ = -0.12-(1-square.width/square.height)/10.0;
                yawV_ = -leftrV_*180.0/3.1415926/distance;
            }
            else
            {
                 leftrV_ = 0.12+(1-square.width/square.height)/10.0;
                yawV_ = -leftrV_*180.0/3.1415926/distance;
            }
        }
    }
    /*-----调整飞机对准矩形-----*/
    if(count_ ==4)
    {
        if(abs(square.y+square.height/2-240)<=20&&abs(square.x+square.width/2-320)<=15)
            count_ = 5;
        else
        {
            leftrV_ = (square.x+square.width/2-320)/100.0*0.2;
            heightV_ = -(square.y+square.height/2-240)/100.0*0.2;
        }
    }
    /*-----调整飞机离矩形的距离-----*/
    if(count_ == 5)
    {
        double rectHeight_ = squareSize*240.0/(tan(cameraAngle/2)*2/3*distance);
        if(abs(square.height -rectHeight_)<=10&&abs(square.y+square.height/2-240)<=10)
            count_ = 6;
        else
        {
            forwardV_ = (rectHeight_ - square.height)/100.0*0.3;
            heightV_ = -(square.y+square.height/2-240)/100.0*0.2;
        }
    }
}
