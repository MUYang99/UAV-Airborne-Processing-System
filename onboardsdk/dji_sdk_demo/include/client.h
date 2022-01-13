#ifndef _CLIENT_H_
#define _CLIENT_H_

/** @file client.cpp
 *  @version 3.1.8
 *  @date July 29th, 2016
 *
 *  @brief
 *  All the exampls for ROS are implemented here.
 *
 *  @copyright 2016 DJI. All rights reserved.
 *
 */
#include <ros/ros.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <stdlib.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>
#include <cv_bridge/cv_bridge.h>
#include<signal.h>
#include "std_msgs/Float64.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include"ImageProc.h"
#include"PIDController.h"
#include"Control3.h"
#include"Control4.h"
#include"NumRecog.h"

//darknet_ros
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <dji_sdk/Compass.h>

//QR
#include <image_transport/image_transport.h>
#include <aruco_msgs/MarkerArray.h>

using namespace DJI::onboardSDK;
namespace enc = sensor_msgs::image_encodings;

typedef actionlib::SimpleActionClient<darknet_ros_msgs::CheckForObjectsAction> YoloClient;
typedef std::shared_ptr<YoloClient> YoloClientPtr;

#define PI 3.1415926  //圆周率常量


/*飞机姿态参数*/
DJIDrone* drone;

//drone data
ros::Subscriber attitude_quaternion_subscriber;
ros::Subscriber acceleration_subscriber;
ros::Subscriber global_position_subscriber;
ros::Subscriber local_position_subscriber;
ros::Subscriber odometry_subscriber;
ros::Subscriber compass_subscriber;
ros::Subscriber velocity_subscriber;

ros::Subscriber time_stamp_subscriber;
ros::Subscriber mission_status_subscriber;
ros::Subscriber mission_event_subscriber;
ros::Subscriber mobile_data_subscriber;


//guidance
ros::Subscriber ultrasonic_sub;
ros::Subscriber belowimage_sub;
ros::Subscriber forwardimageleft_sub;
ros::Subscriber rightImage_sub;
ros::Subscriber leftImage_sub;
ros::Subscriber depthImage_sub;
ros::Subscriber motion_sub;
ros::Subscriber odomCombine_sub;

image_transport::Publisher QRimage_pub;
ros::Subscriber markers_sub;
void markers_sub_callback(aruco_msgs::MarkerArray markersMsg);

//pose ekf
ros::Publisher imu_pub;
ros::Publisher vo_pub;



/*自定义手机回调函数*/
void StartMission1Callback(DJIDrone *drone);//101
void StartMission2Callback(DJIDrone *drone);//102
void StartMission3Callback(DJIDrone *drone);//103
void StartMission4Callback(DJIDrone *drone);//104
void StopMissionCallback(DJIDrone *drone);//107

void attitude_quaternion_subscriber_callback(const dji_sdk::AttitudeQuaternion attitude_quaternion);
void global_position_subscriber_callback(const dji_sdk::GlobalPosition global_position);
void local_position_subscriber_callback(const dji_sdk::LocalPosition local_position);
void accelerationCallback(dji_sdk::Acceleration acceleration);
void compassCallback(const dji_sdk::Compass compass);
void velocityCallback(dji_sdk::Velocity velocity);
void odomCombineCallback(const geometry_msgs::PoseWithCovarianceStamped& poseMsg);

/*guidnace消息回调函数*/
void ultrasonic_callback(const sensor_msgs::LaserScan& g_ul);
void belowImage_callback(const sensor_msgs::Image& belowImageMsg);
void forwardimageleft_callback(const sensor_msgs::Image& forwardImageleft);
void rightImage_callback(const sensor_msgs::Image& rightImage);
void leftImage_callback(const sensor_msgs::Image& leftImage);
void depthImgCallback(const sensor_msgs::Image& depthImage); 
void motionCallback(const geometry_msgs::Point& point);



int getDepth(Rect& rect);
void onMouse(int event,int x,int y,int flags,void* param);//click to see depth data

/*darknet*/
void receiveTargetsCallback(const darknet_ros_msgs::BoundingBoxes& targetBBoxes);

/*odometry*/
void odometryCallback(const nav_msgs::Odometry& odometry);

/*compass*/
void compassCallback(const dji_sdk::Compass compass);

void savePicture(const cv::Mat& picture, const uint32_t number);


double yawSum = 0;
int receiveCount=0;
float drone_pitch, drone_roll, drone_yaw;//俯仰角 横滚角 偏航角
float localPos_x, localPos_y, localPos_z ;//相对起飞位置
double realHeight = 0;//通过guidance获得飞机高度

float xo, yo, zo;//odometry
float px, py, pz;//guidance motion

/*对象声明*/
VideoCapture capture(0);
VideoCapture capture_video;
double imgWidth , imgHeight;//图像宽高
ImageProc imageProc;
//目标识别
darknet_ros_msgs::CheckForObjectsGoal yoloGoal;
darknet_ros_msgs::BoundingBoxes yoloResult;
YoloClientPtr yoloClient;
void doneCB(const actionlib::SimpleClientGoalState& state,
            const darknet_ros_msgs::CheckForObjectsResultConstPtr& result);

//number 识别 
vector<Mat> rectmat;
vector<Point> resultCenter;
NumRecog numRecog;

const char* modelDir = "/home/nvidia/catkin_ws/src/onboardsdk/dji_sdk_demo/src/NumRecog/model/numberRecong_8_13.model";


PIDController pidVX, pidVY, pidVYAW;//前后、上下、偏航PID控制
Control3 control3(0.5236,
                  1.0,
                  2.0,
                  0.4,
                  0.15,
                  0.3,
                  15.0,
                  0.5);
/*
cameraAngle;
normalHeight;
distance;
normalV;
normalV_; 
normalHeightV;
normalW;
squareSize;
*/

/*调试用*/
bool DEBUG_svm;
bool DEBUG_yolo;

struct OrderValue
{
    double value; 
    int num;
};
void Sort(OrderValue *TemOrderValue, int a);
//bool FindRealRoute(CvPoint * allPts, int NUM_RED, vector<CvPoint> &RoutePoints, DoublePoint ScreenPoint, double heading, double d);
vector<Point> trackPointsVector;
double  targetyaw , lastTargetYaw;
double forwardb, leftr, height, yaw;//0X5B 前后v(-10 ~10m/s)、左右v、上下position、偏航v控制量
double lastLeftr;
int lastFrameState;//1 line;2 slope
int counter = 0;//hesder
 int blueDetectedFrame;

/*任务2*/
const double dis = 1.7;  //人离摄像头的距离
const double rectHeight  = 0.198;  //矩形的实际高度
const double rectWidth = 0.27;    //矩形的实际高度
const double angle = 15.5/180.0*3.1415926;   //摄像头数值视角宽度一半
const double vMax = 1.0;   //最大速度
const double r_dis = 0.1*dis;   //速度为零的范围半径
double lastForwardV;
double forwardV,yawV,heightV;
Rect rect;
Point center(320,240);

/*任务3*/
void searchQRCode(Mat &fimage);
bool isLastTargetCircle = false;
float right_limit, left_limit;
int recycleTimes;
float offset_kp;
float normalHeightV;
float landMissionH, acrossMissionH, QRCodeMissionH;

float initYaw, rotateYaw;
float yawRate = 0;
bool ifStartMission = false;
bool findTarget_yolo = false;
bool findTarget_ground = false;


int landPatNumber;
int circleNumber;
int numberFlag[10] = {0};
void getNumberFlag(int number,const int flag);
int targetNumber = 1;
std::string targetName[11] = {"","one", "two" ,"three", "four", "five" ,"six", "seven","eight","nine","ten"};
void getHeightV(float height);
bool findRightSquare(vector<Rect>& rects,Rect& rect);
void findRightSquare_(vector<Rect>&  rects,Rect& rect);
void hardstandLanding(Mat& srcImage,double dx,double dy,int nextMission);
void acrossHinderCircle(Mat& srcImage,double height1,double height2,bool isClockwise1,bool isClockwise2,int nextMission);
void yolo(const Mat& srcImage);
void doneCB(const actionlib::SimpleClientGoalState& state,
            const darknet_ros_msgs::CheckForObjectsResultConstPtr& result);

const char *QRSaveDir = "/home/nvidia/Pictures/Result/";
double curHeight =0;//获取离地高度
Mat belowImage;
Mat forwardleftImage;
Mat leftImage;
Mat depthImage;
int t;

cv::VideoWriter writer; 

float distanceForward = 0;
Mat forwardImage;


string type2str(int type);



/*全局函数声明*/
void imageReceived(Mat &srcImage);
void land(Mat &fimage,Mat &bimage);
void acrossCircle(Mat &fimage);
void getNumberFlag(int number, int flag);
void Stop(int signo)
{
    capture.release();
    drone->landing();
    cout<<"landing";
    exit(0);
}

#endif
