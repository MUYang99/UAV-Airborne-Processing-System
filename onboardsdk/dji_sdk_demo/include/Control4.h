#ifndef _CONTROL4_H_
#define _CONTROL4_H_
#include<opencv2/opencv.hpp>
#include<iostream>
using namespace cv;
using namespace std;

class Control4
{
public:
    Control4();
    Control4(double height,int secondsNumber);
    ~Control4();

    double distanceForward;
    double distanceLeft;
    double distanceRight;
    double height;
    Mat forwardImage;
    Mat rightImage;
    Mat leftImage;
    int secondsNumber;
};

#endif
