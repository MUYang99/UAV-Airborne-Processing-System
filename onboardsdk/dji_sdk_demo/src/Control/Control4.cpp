#include"Control4.h"

Control4::Control4()
{
}
Control4::Control4(double height, int secondsNumber)
{
    this->height = height;
    this->secondsNumber = secondsNumber;
    distanceForward = distanceLeft = distanceRight = 0;
}
Control4::~Control4()
{
}
