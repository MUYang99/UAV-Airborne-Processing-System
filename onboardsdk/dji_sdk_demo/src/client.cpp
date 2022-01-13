#include<client.h>
int initnumber;

float drift_start_x, drift_start_y;
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sdk_client");
    ROS_INFO("sdk_service_client_test");
    ros::NodeHandle nh;
    drone = new DJIDrone(nh);

    //virtual RC test data
    uint32_t virtual_rc_data[6];

    //set frequency test data
    uint8_t msg_frequency_data[16] = {1,2,3,4,3,2,1,2,3,4,3,2,1,2,3,4};
    //waypoint action test data
    dji_sdk::WaypointList newWaypointList; 
    dji_sdk::Waypoint waypoint0; 
    dji_sdk::Waypoint waypoint1;
    dji_sdk::Waypoint waypoint2;
    dji_sdk::Waypoint waypoint3; 
    dji_sdk::Waypoint waypoint4;

    //groundstation test data
    dji_sdk::MissionWaypointTask waypoint_task;
    dji_sdk::MissionWaypoint 	 waypoint;
    dji_sdk::MissionHotpointTask hotpoint_task;
    dji_sdk::MissionFollowmeTask followme_task;
    dji_sdk::MissionFollowmeTarget followme_target;
    uint8_t userData = 0;


    //mission code
    ros::param::get("~imgWidth",imgWidth);
    ros::param::get("~imgHeight",imgHeight);

    ros::param::get("~targetNumber",targetNumber);
    initnumber=targetNumber;
    ros::param::get("landpat_circle/circle",circleNumber);
    getNumberFlag(circleNumber, 1);
 
    
    ros::param::get("~DEBUG_svm",DEBUG_svm);
    ros::param::get("~DEBUG_yolo",DEBUG_yolo);

    ros::param::get("~initYaw", initYaw);
    //rotateYaw = -initYaw;
    rotateYaw = initYaw/180.0 * PI;
    ros::param::get("~right_limit",right_limit);
    ros::param::get("~left_limit", left_limit);

    ros::param::get("~offset_kp", offset_kp);
    ros::param::get("~normalHeightV", normalHeightV);

ros::param::get("~landMissionH", landMissionH);
ros::param::get("~acrossMissionH", acrossMissionH);
ros::param::get("~QRCodeMissionH", QRCodeMissionH);


//QR
image_transport::ImageTransport it(nh);
QRimage_pub = it.advertise("/cameras/left_hand_camera/image", 1);
markers_sub = nh.subscribe("/aruco_marker_publisher/markers", 1 ,markers_sub_callback);

     drone_pitch=0;//俯仰角
     drone_roll=0;//横滚角
     drone_yaw=0;//偏航角
     localPos_x=0; 
     localPos_y=0;
     localPos_z=0;
     xo=0;yo=0;zo=0;
     px=0;py=0;pz=0;
     
 
    //subsciber
    attitude_quaternion_subscriber = nh.subscribe("dji_sdk/attitude_quaternion",10,attitude_quaternion_subscriber_callback);
    global_position_subscriber = nh.subscribe("dji_sdk/global_position", 10,global_position_subscriber_callback);
    local_position_subscriber = nh.subscribe("dji_sdk/local_position", 10,local_position_subscriber_callback);
    odometry_subscriber = nh.subscribe("dji_sdk/odometry", 10, odometryCallback);
    compass_subscriber = nh.subscribe("dji_sdk/compass", 10, compassCallback);
    acceleration_subscriber = nh.subscribe("dji_sdk/acceleration", 10, accelerationCallback);
    velocity_subscriber = nh.subscribe("dji_sdk/velocity", 10, velocityCallback);
 
    odomCombine_sub = nh.subscribe("/robot_pose_ekf/odom_combined", 10, odomCombineCallback);
    
    //guidance
    ultrasonic_sub = nh.subscribe("/guidance/ultrasonic", 1, ultrasonic_callback);
    belowimage_sub = nh.subscribe("/guidance/below_left_image", 1, belowImage_callback);
    forwardimageleft_sub = nh.subscribe("/guidance/forward_left_image", 1, forwardimageleft_callback);
    rightImage_sub = nh.subscribe("/guidance/right_left_image", 1, rightImage_callback);
    leftImage_sub = nh.subscribe("/guidance/left_left_image", 1, leftImage_callback);
    //depthImage_sub = nh.subscribe("/guidance/depth_image", 1, depthImgCallback);
    motion_sub = nh.subscribe("/guidance/position",1, motionCallback);
    //capture_video = VideoCapture("/home/nvidia/Videos/M100_test.mp4");


    int t = cv::getTickCount();
    std::string fileName_="/home/nvidia/Videos/" + std::to_string(t);
    std::string fileName=fileName_+".avi";
    writer=cv::VideoWriter(fileName,CV_FOURCC('F','L','V','1'),25.0,cv::Size(640,480));


    //poseekf
    imu_pub = nh.advertise<sensor_msgs::Imu>("/imu_data", 10);
    vo_pub = nh.advertise<nav_msgs::Odometry>("/vo", 10);

    yoloClient.reset(new YoloClient("/darknet_ros/check_for_objects", true));
    numRecog.loadModel(modelDir);

     

    ros::spinOnce();
    {
        //Mobile callback
	drone->setStartMission1Callback(StartMission1Callback, &userData);//mission1
	drone->setStartMission2Callback(StartMission2Callback, &userData);//mission2
        drone->setStartMission3Callback(StartMission3Callback, &userData);//mission3
	drone->setStartMission4Callback(StartMission4Callback, &userData);//mission4
        drone->setStopMissionCallback(StopMissionCallback, &userData);//shutdown
    }
    signal(SIGINT,Stop);

    if(!capture.isOpened()) 
    {
        cout<<"camera open failed!"<<endl;
        ros::shutdown();
    }
   

     yoloClient->waitForServer();

    ros::Rate rate(25);
    while(ros::ok())
    {
        ros::spinOnce(); 
        //yolo(forwardImage);
        capture >> forwardImage;
        writer<<forwardImage;
        //savePicture(forwardImage, 2);

        //Mat groundRGB;
        //capture_video >> groundRGB;

        printf("\033[2J");
        printf("\033[1;1H");
       
       //if(targetNumber == 10 && !forwardImage.empty())
       //{
          //sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", forwardImage).toImageMsg();
          //QRimage_pub.publish(msg);
          //cout<<" send img msg to QR"<<endl;
         // imshow("forward", forwardImage);
          //waitKey(1);
       //}
        

        //printf("px %f\npy %f\nH %f\n",px, py,curHeight);
        /*if(DEBUG_svm){
           if(!belowImage.empty())
           { 
             Mat fakeRGB;
	     cvtColor(belowImage , fakeRGB, CV_GRAY2RGB);
	     if(!fakeRGB.empty())
	        yolo(fakeRGB);
	   }else
             printf("below img is empty\n"); 
        }else if(DEBUG_yolo){
           if(!forwardImage.empty())
     	       yolo(forwardImage);
        }else */
          if(!forwardImage.empty() && !belowImage.empty()){
             namedWindow("guidance",WINDOW_NORMAL);
              printf("px %f\npy %f\nH %f\n",px, py,curHeight);
             if(ifStartMission) 
             {
	       printf("target: %d  State: %d  H: %f CurYaw: %f\n",targetNumber, control3.count2, curHeight, drone_yaw);
               yawRate = - 0.06*(drone_yaw/PI*180 - initYaw);
	       //yawRate =0;
               if(targetNumber == 10)
               {
                  printf("targety %f\n", drift_start_y + 2);
                  searchQRCode(forwardImage);
                  imshow("gopro", forwardImage);
                  waitKey(2);
               }else if(numberFlag[targetNumber] == 0)// landpat 
	       {
                  printf("targety %f\n", drift_start_y + 0.3);
		  land(forwardImage,belowImage); 
	       }else{
                  printf("targety %f distanceForward %f\n",drift_start_y + 0.5, distanceForward);
		  acrossCircle(forwardImage);
	       } 
             }else {
                yolo(forwardImage); 
	        imshow("guidance",belowImage);
                waitKey(1);
             } 
        }/*else if(forwardImage.empty()){
          printf("gopro empty\n");
        }else{
          printf("guidance empty\n"); 
        }//gopro guidance */
     rate.sleep();
    }//while









     ros::AsyncSpinner spinner(4);
     spinner.start();
     ros::waitForShutdown();
     return 0;
}


int frontTarget_pixloc_flag;
int groundTarget_pixloc_flag_x;
int groundTarget_pixloc_flag_y;

void land(Mat &fimage,Mat &bimage)
{
    findTarget_yolo = false;

    //move up
    if(control3.count2 == 1)
    { 
        if(control3.isFirstTakeoff && curHeight < 0.2)
	{
	       cout<<"take off..."<<endl; 
               control3.isFirstTakeoff = false;
               drift_start_x = px;
               drift_start_y = py;
               printf("drift_start_x %f  drift_start_y %f\n",drift_start_x, drift_start_y );
	       drone->takeoff(); 
	       sleep(7);

               ros::spinOnce();
               return;
	}else 
        
        yolo(fimage);
        if(findTarget_yolo)
        {
           control3.count2 = 6;
           frontTarget_pixloc_flag = control3.square.x + control3.square.width/2 - 320;
        }/*else if(isLastTargetCircle)
              control3.count2 = 4;*/
         else if(fabs(curHeight -landMissionH ) > 0.2)
        {
            getHeightV(landMissionH); 
            drone->attitude_control(0x4B, 0, 0, control3.heightV, yawRate);
        }else
           control3.count2 = 2; 
        return; 
    }


    //move right if not find
    if(control3.count2 == 2)
    {
        yolo(fimage);
        if(findTarget_yolo && abs(control3.square.x + control3.square.width/2 - 320) <= 290)
        {
             drone->attitude_control(0x4B, 0, -0.1, 0, yawRate);//offset inertia
             control3.count2 = 6;
        }else{
            if(px < right_limit){ 
               control3.leftrV = 0.35;
              if(!isLastTargetCircle && (drift_start_y + 0.3 >= py))
                  control3.forwardV =  0.15;
               else
                  control3.forwardV = 0;//(drift_start_y + 0.3 - py) * 0.2;

               getHeightV(landMissionH);
               drone->attitude_control(0x4B, control3.forwardV, control3.leftrV, control3.heightV, yawRate);
            }else
               control3.count2 = 3;
        }
        return;
    }


    //move left if not find
    if(control3.count2 == 3)
    {
        yolo(fimage);
        if(findTarget_yolo && abs(control3.square.x + control3.square.width/2 - 320) <= 290)
        {
              drone->attitude_control(0x4B, 0, 0.1, 0, yawRate);//offset inertia
              control3.count2 = 6;
        }else{ 
            
            if(px > left_limit)
            {
               if(px > 0)
                  control3.leftrV = -0.45;
               else
                  control3.leftrV = -0.3;

              if(!isLastTargetCircle && (drift_start_y + 0.3 >= py))
                  control3.forwardV =  0.15;
               else
                  control3.forwardV = 0;//(drift_start_y + 0.3 - py) * 0.2;
               getHeightV(landMissionH);
               drone->attitude_control(0x4B, control3.forwardV, control3.leftrV, control3.heightV, yawRate);
            }else
               control3.count2 = 4;       
        }
      return;
    } 


    //move forward  up and right if not find
    if(control3.count2 == 4)
    {
      Mat gray2RGB;
      cvtColor(belowImage , gray2RGB, CV_GRAY2RGB);
      //type2str(gray2RGB.type());


      //imwrite("/home/nvidia/Pictures/M100/guidance.jpg",gray2RGB); 
      resize(gray2RGB,gray2RGB,cv::Size(imgWidth,imgHeight));
      if(!gray2RGB.empty())
      {
         yolo(gray2RGB);
         if(findTarget_yolo) 
         { 
            printf("state 4 find target\n"); 
            control3.count2 = 6;
         }else{
            if(px < right_limit)
            {
	            //move forward alittle
		    printf("state 4 moveforward up right\n");
		    control3.forwardV =  0;
		    control3.leftrV = 0;

		    if(drift_start_y + 1.0 > py)
		       control3.forwardV =  0.15;//(drift_start_y + 0.8 - py) * 0.2;
		    else if(px < right_limit)
		       control3.leftrV = 0.3;
		    
		    getHeightV(2.6);

		    drone->attitude_control(0x4B, control3.forwardV, control3.leftrV, control3.heightV, yawRate);
            }else
               control3.count2 = 5; 
            
            
        }//find
      }//!fakeRGB.empty()
      else
         printf("fakergb emptyy\n"); 
      return;
    } 



    //move forward  up and left if not find
    if(control3.count2 == 5)
    {
      Mat gray2RGB;
      cvtColor(belowImage , gray2RGB, CV_GRAY2RGB);
      //type2str(gray2RGB.type());


      //imwrite("/home/nvidia/Pictures/M100/guidance.jpg",gray2RGB); 
      resize(gray2RGB,gray2RGB,cv::Size(imgWidth,imgHeight));
      if(!gray2RGB.empty())
      {
         yolo(gray2RGB);
         if(findTarget_yolo) 
         { 
            printf("state 5 find target\n"); 
            control3.count2 = 6;
         }else{
            if(px > left_limit)
            {
	            //move forward alittle
		    printf("state 4 moveforward up right\n");
		    control3.forwardV =  0;
		    control3.leftrV = 0;

		    if(drift_start_y + 1.0 > py)
		       control3.forwardV =  0.15;//(drift_start_y + 0.8 - py) * 0.2;
		    else if(px > left_limit)
		       control3.leftrV = -0.3;
		    
		    getHeightV(2.6);

		    drone->attitude_control(0x4B, control3.forwardV, control3.leftrV, control3.heightV, yawRate);
            }else{
               control3.count2 = 4;
               drift_start_y = drift_start_y + 0.5;
            } 
            
            
        }//find
      }//!fakeRGB.empty()
      else
         printf("fakergb emptyy\n"); 
      return;
    } 



    //move precisely to align front target
    /*if(control3.count2 == 4)
    {
        yolo(fimage);
        if(findTarget_yolo)
        {
            if(abs(control3.square.x + control3.square.width/2 - 290)<=20)// guidance image is left scope:240 = 320 -60
               control3.count2 = 5;
            else
            { 
                //getHeightV(control3.normalHeight);
                control3.getVelocityo();
                drone->attitude_control(0x4B, 0, control3.leftrV,0, yawRate);
            }
            frontTarget_pixloc_flag = control3.square.x + control3.square.width/2 - 290;
        }else{
            //last find but lost now and last target is left to center && so move left
            control3.leftrV = 1.0 * (frontTarget_pixloc_flag + 0.0001)/fabs(frontTarget_pixloc_flag + 0.0001) * 0.08;
            drone->attitude_control(0x4B, 0, control3.leftrV, 0, yawRate);
         }
       return;
    }*/



    //move towards
    if(control3.count2 == 6)
    {
      Mat gray2RGB;
      cvtColor(belowImage , gray2RGB, CV_GRAY2RGB);
      //type2str(gray2RGB.type());


      //imwrite("/home/nvidia/Pictures/M100/guidance.jpg",gray2RGB); 
      resize(gray2RGB,gray2RGB,cv::Size(imgWidth,imgHeight));
      if(!gray2RGB.empty())
      {
         yolo(gray2RGB);
         if(findTarget_yolo) 
         { 
            printf("state 6 find target\n"); 
            if(abs(control3.square.x + control3.square.width/2 - 340)<=20 &&//offset left camera 
               abs(control3.square.y + control3.square.height/2 - 240)<=20)// guidance image 320 240 
               control3.count2 = 7;
            else{
                control3.centerX = control3.square.x + control3.square.width/2;
		control3.centerY = control3.square.y + control3.square.height/2;
                control3.getVelocity1();
                drone->attitude_control(0x4B, control3.forwardV, control3.leftrV,0, yawRate);/////////////
            }
            findTarget_ground = true;
            groundTarget_pixloc_flag_x = control3.square.x + control3.square.width/2 - 320;
            groundTarget_pixloc_flag_y = control3.square.y + control3.square.height/2 - 240;
         }/*else if(findTarget_ground){
            printf("state 5 lose target\n");
            //last find but lost now
            control3.forwardV = -1.0 * (groundTarget_pixloc_flag_y + 0.0001)/fabs(groundTarget_pixloc_flag_y+ 0.0001) * 0.1;
            control3.leftrV = 1.0 * groundTarget_pixloc_flag_x/fabs(groundTarget_pixloc_flag_x) * 0.1;
            drone->attitude_control(0x4B, control3.forwardV, control3.leftrV, control3.heightV, 0);
         }*/else{
            printf("state 6 moveforward\n");
            getHeightV(2.6);
            control3.leftrV = 0.1 * (control3.square.x + control3.square.width/2 - 320)/150.0;//100 fx
            drone->attitude_control(0x4B,  0.18 , control3.leftrV, control3.heightV, yawRate);
        }//find
      }//!fakeRGB.empty()
      else
         printf("fakergb emptyy\n"); 
         

      /*if(imageProc.findSquares(belowImage, rectmat, resultCenter))
      {
        for(int i(0); i!= rectmat.size(); ++i)
        {
           if(numRecog.getNumber(rectmat[i]) == targetNumber)
           {
              if(abs(resultCenter[i].x-180)<=30 && abs(resultCenter[i].y-bimage.rows/2)<=30)
                 control3.count2 = 6;
              else{
                 control3.centerX = resultCenter[i].x;
		 control3.centerY = resultCenter[i].y;
                 control3.getVelocity1();
                 drone->attitude_control(0x4B, control3.forwardV, control3.leftrV,0, yawRate);
              }
              return;
          }//if find 
        }//for
      }*/
      return; 
    } 

    //land
    if(control3.count2 == 7)
    {
        if(curHeight <= 0.2)
        {
          ros::spinOnce();
          control3.count2 = 1;
          ++targetNumber;
          control3.isFirstTakeoff=true;
          findTarget_ground = false;
          isLastTargetCircle = false;
          drift_start_x = px;
          drift_start_y = py;
        }else{
          //control3.heightV = -0.25; 
          //drone->attitude_control(0x4B, 0, 0,control3.heightV, 0);
	  drone->landing();
          sleep(8);
        }
    }
}

int findCount = 0;
int crossflag = 0;
double heighto;
void acrossCircle(Mat &fimage)
{
    findTarget_yolo = false;
    //move up
    if(control3.count2 == 1)
    {
        if(control3.isFirstTakeoff && curHeight < 0.2)
        {
               cout<<"take off..."<<endl;
               control3.isFirstTakeoff = false;
               drift_start_x = px;
               drift_start_y = py;
               printf("drift_start_x %f  drift_start_y %f",drift_start_x, drift_start_y );
               drone->takeoff();
               sleep(6);

               ros::spinOnce();
               return;
        }

	yolo(fimage);
        if(findTarget_yolo)
        {
          control3.count2 = 4;
          frontTarget_pixloc_flag = control3.square.x + control3.square.width/2 - 320;
        }else if(fabs(curHeight -acrossMissionH ) > 0.2)
        {
            getHeightV(acrossMissionH);
            drone->attitude_control(0x4B, 0, 0, control3.heightV, yawRate);
        }else
            control3.count2 = 2;

        return;
    }
    

    //move right if not find 
    if(control3.count2 == 2)
    {
        yolo(fimage);
        if(findTarget_yolo) 
        {
              drone->attitude_control(0x4B, 0, - 0.1, 0, yawRate);//offset inertia
              control3.count2 = 4;
              frontTarget_pixloc_flag = control3.square.x + control3.square.width/2 - 320;
        }else{
            if(px < right_limit)
            {
               control3.leftrV = 0.25;
               
              if(!isLastTargetCircle && (drift_start_y + 0.3 >= py))
              {
                 control3.forwardV =  0.15;
                 cout<<"control3.forwardV =  0.15 "<<endl;
              }else{
                 control3.forwardV =  0;
                 cout<<"control3.forwardV =  0 "<<endl;
              }
 
               getHeightV(acrossMissionH);
               drone->attitude_control(0x4B, control3.forwardV, control3.leftrV, control3.heightV, yawRate);
            } 
            else
               control3.count2 = 3;  
        }
        return;
    }


    //move left if not find
    if(control3.count2 == 3)
    {
        yolo(fimage);
        if(findTarget_yolo)
        {
              drone->attitude_control(0x4B, 0, 0.1, 0, yawRate);//offset inertia
              control3.count2 = 4;
              frontTarget_pixloc_flag = control3.square.x + control3.square.width/2 - 320;
        }else{
            if(px > left_limit)
            {
               if(px > 0)
                  control3.leftrV = -0.4;
               else
                  control3.leftrV = -0.25;

               
               if(!isLastTargetCircle && (drift_start_y + 0.3 >= py))
                  control3.forwardV =  0.15;
               else
                  control3.forwardV = 0;
               getHeightV(acrossMissionH);
               drone->attitude_control(0x4B, control3.forwardV, control3.leftrV, control3.heightV, yawRate);
            } 
            else
               control3.count2 = 2;  
                  
        }  
      return;
    } 

    //move precisely
    if(control3.count2 == 4)
    {
        if(crossflag >= 6)
        {
           heighto = curHeight + 0.68;
           control3.count2 = 5;
           crossflag = 0;
           printf("ready to across circle....\n"); 
        }else{
           yolo(fimage);
           if(findTarget_yolo){
             if(abs(control3.square.x + control3.square.width/2 - 320) <=20 &&
               abs(control3.square.y + control3.square.height/2 - 250) <=20 && distanceForward < 1.5)
                ++crossflag;//offset the instability of distanceForward
             else{
               if(abs(control3.square.x + control3.square.width/2 - 320) <=20 &&
                  abs(control3.square.y + control3.square.height/2 - 250) <=20)
                    control3.forwardV = 0.1;//distanceForward > 1.0 ? distanceForward * 0.06 : 0;
               else{
                    if(!isLastTargetCircle && (drift_start_y + 0.3 >= py))
                      control3.forwardV =  0.15;
                    else
                      control3.forwardV = 0.01;
	       }
                int errorx = control3.square.x + control3.square.width/2 - 320;
		int errory = control3.square.y + control3.square.height/2 - 250;
		control3.leftrV = errorx/100.0 * 0.15;
		control3.heightV = -errory/100.0 * 0.15;
                drone->attitude_control(0x4B, control3.forwardV, control3.leftrV,control3.heightV, yawRate);
                crossflag = 0;                  
              }//adjust 
               frontTarget_pixloc_flag = control3.square.x + control3.square.width/2 - 320;
            }else{//last find but not find now
              control3.leftrV = frontTarget_pixloc_flag < 0? -0.15 : 0.15;
		
               if(!isLastTargetCircle && (drift_start_y + 0.3 >= py))
                  control3.forwardV =  0.15; 
               else
                  control3.forwardV = 0;

              getHeightV(acrossMissionH);
              drone->attitude_control(0x4B, control3.forwardV, control3.leftrV, control3.heightV , yawRate);
            }
        }
        return; 
    }  

    //across 
    if(control3.count2 == 5)
    {
          //distanceForward
        if(fabs(curHeight - heighto) > 0.1)
        {
            printf("move up to align circle\n");
            getHeightV(heighto);
            drone->attitude_control(0x4B, 0, 0, control3.heightV, 0);
        }else{
            printf("across circle....\n");
            for(int i(0); i <400; ++i)
            {
               drone->attitude_control(0x4B, 0.3, 0, 0, 0);
               usleep(10000);
            }
            ros::spinOnce();
            control3.count2 = 1;
            ++targetNumber;
            control3.isFirstTakeoff=true;
            isLastTargetCircle = true;
            drift_start_x = px;
            drift_start_y = py; 
        }
    }
}//acrossCircle

 
void yolo(const Mat& srcImage)
{
    sensor_msgs::Image msg;
    cv_bridge::CvImage(std_msgs::Header(), "bgr8", srcImage).toImageMsg(msg);
    yoloGoal.image = msg;
    yoloClient->sendGoal(yoloGoal,&doneCB);
    bool finishBeforeTimeOut = yoloClient->waitForResult(ros::Duration(0.5));
    if(!finishBeforeTimeOut)
        ROS_INFO("Action did not finish before ddl.");  
       //actionlib::SimpleClientGoalState state = yoloClient->getState();
       //ROS_INFO("Action finished: %s",state.toString().c_str());
}

//darknet_ros result callback
void doneCB(const actionlib::SimpleClientGoalState& state,
            const darknet_ros_msgs::CheckForObjectsResultConstPtr& result)
{
  yoloResult.bounding_boxes.clear();
  yoloResult = result->bounding_boxes;
  for(int i(0);i != yoloResult.bounding_boxes.size(); ++i)
  {
    if(targetNumber == 1 && (yoloResult.bounding_boxes[i].Class == "one" || yoloResult.bounding_boxes[i].Class == "seven"))
    {
       findTarget_yolo = true;
	 //printf("\n------find Target-----------\n");
	 control3.square.x = yoloResult.bounding_boxes[i].xmin; 
	 control3.square.y = yoloResult.bounding_boxes[i].ymin;
	 control3.square.width = yoloResult.bounding_boxes[i].xmax - yoloResult.bounding_boxes[i].xmin + 1;
	 control3.square.height = yoloResult.bounding_boxes[i].ymax - yoloResult.bounding_boxes[i].ymin + 1;
    }else if(targetNumber == 7 && (yoloResult.bounding_boxes[i].Class == "one" || yoloResult.bounding_boxes[i].Class == "seven"))
    {
       findTarget_yolo = true;
	 //printf("\n------find Target-----------\n");
	 control3.square.x = yoloResult.bounding_boxes[i].xmin; 
	 control3.square.y = yoloResult.bounding_boxes[i].ymin;
	 control3.square.width = yoloResult.bounding_boxes[i].xmax - yoloResult.bounding_boxes[i].xmin + 1;
	 control3.square.height = yoloResult.bounding_boxes[i].ymax - yoloResult.bounding_boxes[i].ymin + 1;
    }else if(yoloResult.bounding_boxes[i].Class.compare(targetName[targetNumber]) == 0)
      {
	 findTarget_yolo = true;
	 //printf("\n------find Target-----------\n");
	 control3.square.x = yoloResult.bounding_boxes[i].xmin; 
	 control3.square.y = yoloResult.bounding_boxes[i].ymin;
	 control3.square.width = yoloResult.bounding_boxes[i].xmax - yoloResult.bounding_boxes[i].xmin + 1;
	 control3.square.height = yoloResult.bounding_boxes[i].ymax - yoloResult.bounding_boxes[i].ymin + 1;
      } 
         /*if(!depthImage.empty())
         { 
             int depth = getDepth(control3.square);
         }*/

  }
}  

const float threshHeight = 0.3;
void getHeightV(float targetHeight) 
{
    float errorHeight = targetHeight - curHeight;
    if(fabs(errorHeight) <= threshHeight)
        control3.heightV = errorHeight/threshHeight * normalHeightV;
    else
        control3.heightV = errorHeight/fabs(errorHeight) * normalHeightV;
}

int lastCount2;
bool ifFirstFind = true;
bool findQRCode = false; 
std::vector<int> QRcodeNumber;


uint32_t ID;
void searchQRCode(Mat &fimage)
{
   sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", fimage).toImageMsg();
   QRimage_pub.publish(msg);
    //move up and forward
    if(control3.count2 == 1)
    {
        if(control3.isFirstTakeoff && curHeight < 0.2)
        {
               cout<<"take off..."<<endl;
               control3.isFirstTakeoff = false;
               drift_start_x = px;
               drift_start_y = py;
               printf("drift_start_x %f  drift_start_y %f",drift_start_x, drift_start_y );
               drone->takeoff();
               sleep(6);

               ros::spinOnce();
               return;
        }

        if(findQRCode)
        {
          lastCount2 = 1;
          control3.count2 = 5;
          frontTarget_pixloc_flag = control3.square.x + control3.square.width/2 - 320;
        }else if(fabs(curHeight -QRCodeMissionH ) > 0.2 || drift_start_y + 2.5 > py)
        {
            getHeightV(QRCodeMissionH);
            if(drift_start_y + 2.5 <= py)
               control3.forwardV = 0;
            else 
               control3.forwardV = 0.2;
            drone->attitude_control(0x4B, control3.forwardV, 0, control3.heightV, yawRate);
        }else
            control3.count2 = 2;

        return;
    }
    

    //move right if not find 
    if(control3.count2 == 2)
    {
        if(findQRCode) 
        {
              lastCount2 = 2;
              drone->attitude_control(0x4B, 0, - 0.15, 0, yawRate);//offset inertia
              control3.count2 = 5;
              sleep(1);
        }else{
            if(px < right_limit)
            {
               control3.leftrV = 0.15;
               
               if(drift_start_y + 2.5 <= py)
                  control3.forwardV = 0;
               else /*if(fabs(drift_start_y + 1.5 - py) > 0.2 && distanceForward > 2.0)
                  control3.forwardV =  (drift_start_y + 1.5 - py) * 0.5;
               else if(distanceForward > 2.0)
                  control3.forwardV =  (drift_start_y + 1.5 - py) * 0.1;
               else*/
                  control3.forwardV = 0.2;

               getHeightV(QRCodeMissionH);
               drone->attitude_control(0x4B, control3.forwardV, control3.leftrV, control3.heightV, yawRate);
            } 
            else
               control3.count2 = 3; 
        }
        return;
    }


    //move left if not find
    if(control3.count2 == 3)
    {
        if(findQRCode)
        {
              lastCount2 = 3;
              drone->attitude_control(0x4B, 0, 0.15, 0, yawRate);//offset inertia
              sleep(1);
              control3.count2 = 5;
              frontTarget_pixloc_flag = control3.square.x + control3.square.width/2 - 320;
        }else{
            if(px > left_limit)
            {
               if(px > 0)
                  control3.leftrV = -0.3;
               else
                  control3.leftrV = -0.15;

               
               if(drift_start_y + 2.5 <= py)
                  control3.forwardV = 0;
               else 
                  control3.forwardV =  0.2;
               getHeightV(QRCodeMissionH);
               drone->attitude_control(0x4B, control3.forwardV, control3.leftrV, control3.heightV, yawRate);
            } 
            else{
               control3.count2 = 4;
               drift_start_y = drift_start_y + 2.5;
            }
               
                  
        }  
      return;
    } 



//move forward
    if(control3.count2 == 4)
    {
       if(drift_start_y + 2.5 <= py)
       { 
          control3.forwardV = 0;
          control3.count2 = 2;
       }else 
          control3.forwardV =  0.2;
       getHeightV(QRCodeMissionH);
       drone->attitude_control(0x4B, control3.forwardV, control3.leftrV, control3.heightV, yawRate);
      return;
    } 


    //move precisely
    if(control3.count2 == 5)
    {
        /*if(crossflag >= 6)
        {
           heighto = curHeight + 0.75;
           control3.count2 = 5;
           crossflag = 0;
           printf("ready to across circle....\n"); 
        }else{
           if(findQRCode){
             if(abs(control3.square.x + control3.square.width/2 - 320) <=20 &&
               abs(control3.square.y + control3.square.height/2 - 240) <=20 && distanceForward < 1.5)
                ++crossflag;//offset the instability of distanceForward
             else{
             if(abs(control3.square.x + control3.square.width/2 - 320) <=20 &&
               abs(control3.square.y + control3.square.height/2 - 240) <=20)
                  control3.forwardV = distanceForward > 1.0 ? distanceForward * 0.06 : 0;
               else
                  control3.forwardV = 0.03;

	int errorx = control3.square.x + control3.square.width/2 - 320;
	int errory = control3.square.y + control3.square.height/2 - 240;
	control3.leftrV = errorx/100.0 * 0.15;
	control3.heightV = -errory/100.0 * 0.15;


                drone->attitude_control(0x4B, control3.forwardV, control3.leftrV,control3.heightV, yawRate);
                crossflag = 0;                  
              }//adjust 
               frontTarget_pixloc_flag = control3.square.x + control3.square.width/2 - 320;
            }else{
              //last find but not find now
              control3.leftrV = frontTarget_pixloc_flag < 0? -0.1 : 0.1;
               if(fabs(drift_start_y + 0.5 - py) > 0.2)
                  control3.forwardV =  (drift_start_y + 0.5 - py) * 0.2;
               else
                  control3.forwardV =  (drift_start_y + 0.5 - py) * 0.1;
              getHeightV(QRCodeMissionH);
              drone->attitude_control(0x4B, control3.forwardV, control3.leftrV, control3.heightV , yawRate);
            }
        }*/

         if(drift_start_y + 2.5 < py)
             control3.count2 = 6;
         else
             drone->attitude_control(0x4B, 0.2, 0, 0 , yawRate);
        return; 
    }  

    //save picture
    if(control3.count2 == 6)
    {
            
            printf("save picture....\n");
            control3.count2 = lastCount2;
            savePicture(fimage, ID);
    }
}



 std::vector<uint32_t> QResult;

void markers_sub_callback(aruco_msgs::MarkerArray markersMsg)
{
   findQRCode = false;
   for(size_t i=0; i < markersMsg.markers.size(); ++i)
   {
      std::vector<uint32_t>::iterator ret = std::find(QResult.begin(), QResult.end(), markersMsg.markers[i].id);
      if(ret == QResult.end())
      {
         findQRCode = true;
         QResult.push_back(markersMsg.markers[i].id);
         //QRCenter_x = markersMsg.markers[i].pose.pose.position x y z
         //geometry_msgs/PoseWithCovariance pose
         cout<<"new QR:"<<markersMsg.markers[i].id <<endl;
         ID = markersMsg.markers[i].id;
      }else{
          cout<<"old QR:"<<endl;
      }
   }
       
}

void savePicture(const cv::Mat& picture, const uint32_t number)
{
    char postfix[]=".png";
    char path[255];
    memset(path,'\0',sizeof(char)*255);//尽量使用memset，例如数组的多次初始化
    snprintf(path,sizeof(path),"%s%ld%s",QRSaveDir,number,postfix);
    imwrite(path,picture);
}


//callback
float qx,qy,qz,qw;
void attitude_quaternion_subscriber_callback(const dji_sdk::AttitudeQuaternion attitude_quaternion)
{
     qx = attitude_quaternion.q0;
     qy = attitude_quaternion.q0; 
     qz = attitude_quaternion.q0;
     qw = attitude_quaternion.q0;
     double q2sqr = attitude_quaternion.q2 * attitude_quaternion.q2;
     double t0 = -2.0 * (q2sqr + attitude_quaternion.q3 * attitude_quaternion.q3) + 1.0;
     double t1 = +2.0 * (attitude_quaternion.q1 * attitude_quaternion.q2 + attitude_quaternion.q0 * attitude_quaternion.q3);
     double t2 = -2.0 * (attitude_quaternion.q1 * attitude_quaternion.q3 - attitude_quaternion.q0 * attitude_quaternion.q2);
     double t3 = +2.0 * (attitude_quaternion.q2 * attitude_quaternion.q3 + attitude_quaternion.q0 * attitude_quaternion.q1);
     double t4 = -2.0 * (attitude_quaternion.q1 * attitude_quaternion.q1 + q2sqr) + 1.0;

     t2 = t2 > 1.0 ? 1.0 : t2;
     t2 = t2 < -1.0 ? -1.0 : t2;
    drone_pitch = asin(t2);
    drone_roll = atan2(t3, t4);
    drone_yaw = atan2(t1, t0); 
    //yawSum += drone_yaw; 
    //++receiveCount; 
    //cout<<"drone_yaw "<< drone_yaw<<endl;

    ros::Time timestamp = ros::Time::now();
    std_msgs::Header header;
    header.stamp = timestamp;
    header.frame_id = "/base_footprint";

    sensor_msgs::Imu imuMsg;
    imuMsg.header = header;
    float square = sqrt(pow(qx,2) + pow(qy,2) + pow(qz,2) + pow(qw,2));
    imuMsg.orientation.x = qx/square;
    imuMsg.orientation.y = qy/square;
    imuMsg.orientation.z = qz/square;
    imuMsg.orientation.w = qw/square;
    imu_pub.publish(imuMsg);
}


void global_position_subscriber_callback(const dji_sdk::GlobalPosition global_position)
{
      float latitude,longitude,altitude,height;
      int health;
      latitude=global_position.latitude;
      longitude=global_position.longitude;
      altitude=global_position.altitude;
      height=global_position.height;
      health=global_position.health;
      //cout<<"latitude: "<<latitude<<" longitude:"<<longitude<<" altitude:"<<altitude<<endl;
}

void local_position_subscriber_callback(const dji_sdk::LocalPosition local_position)
{
    localPos_x=local_position.x;
    localPos_y=local_position.y;
    localPos_z=local_position.z;
    //cout<<"localPos_x: "<<localPos_x<<" localPos_y:"<<localPos_y<<" localPos_z:"<<localPos_z<<endl;
}

//odometry need gps
void odometryCallback(const nav_msgs::Odometry& odometry)
{
   xo = odometry.pose.pose.position.x; 
   yo = odometry.pose.pose.position.y;
   zo = odometry.pose.pose.position.z;
   //(ros::Time::now() - startTime).toSec()
   //cout<<"ox: "<<xo<<" oy:"<<yo<<" oz:"<<zo<<endl;
}

void odomCombineCallback(const geometry_msgs::PoseWithCovarianceStamped& poseMsg)
{
   //printf("ox: %f oy: %f oz: %f\n", poseMsg.pose.pose.position.x, poseMsg.pose.pose.position.y, poseMsg.pose.pose.position.z );
}

void accelerationCallback(dji_sdk::Acceleration acceleration)
{
   //printf("acceleration x: %d y: %d z: %d\n", acceleration.ax, acceleration.ay, acceleration.az);
}

void compassCallback(const dji_sdk::Compass compass)
{
  //printf("compass x: %d y: %d z: %d\n", compass.x, compass.y, compass.z);
}

void velocityCallback(dji_sdk::Velocity velocity)
{ 
  //printf(" velocity x: %d y: %d z: %d\n", velocity.vx, velocity.vy, velocity.vz);
}

void motionCallback(const geometry_msgs::Point& point)
{
   //rotate to contest coordinate
   px = point.y * cos(rotateYaw) - point.x * sin(rotateYaw); 
   py = point.y * sin(rotateYaw) + point.x * cos(rotateYaw);
   pz = point.z;

   //printf("0 %f %f\n",point.x, point.y);
   //printf("1 %f %f\n",px, py);

    /*ros::Time timestamp = ros::Time::now();
    std_msgs::Header header;
    header.stamp = timestamp;
    header.frame_id = "/odom_combined";

    nav_msgs::Odometry voMsg; 
    voMsg.header = header;
    voMsg.pose.pose.position.x = point.x; 
    voMsg.pose.pose.position.y = point.y;
    voMsg.pose.pose.position.z = point.z;
    voMsg.pose.pose.orientation.x = qx;
    voMsg.pose.pose.orientation.y = qy;
    voMsg.pose.pose.orientation.z = qz;
    voMsg.pose.pose.orientation.w = qw;
    for(int i(0); i<36; i += 7)
        voMsg.pose.covariance[i] = 0.001;
    vo_pub.publish(voMsg);*/
}

//guidance
/*ultrasonic*/
void ultrasonic_callback(const sensor_msgs::LaserScan& g_ul)
{
    curHeight = g_ul.ranges[0] < 0 ? curHeight : g_ul.ranges[0];
    distanceForward = g_ul.ranges[1] < 0 ? distanceForward : g_ul.ranges[1];
    //control4.distanceRight =  g_ul.ranges[2] < 0 ? control4.distanceRight : g_ul.ranges[2];
    //control4.distanceLeft =  g_ul.ranges[4] < 0 ? control4.distanceLeft : g_ul.ranges[4];
    //cout<<control4.distanceForward<<"   "<<control4.distanceRight<<"   "<<control4.distanceLeft<<"   "<<curHeight<<endl;
}
 
/*belowimageleft*/
void belowImage_callback(const sensor_msgs::Image& belowImageleftMsg)
{
    cv_bridge::CvImagePtr image;
    image = cv_bridge::toCvCopy(belowImageleftMsg, sensor_msgs::image_encodings::MONO8);
    
    image->image.copyTo(belowImage);
    //imshow("belowleft",belowImage);
    //waitKey(1);
}
void forwardimageleft_callback(const sensor_msgs::Image& forwardImageleftMsg)
{
    cv_bridge::CvImagePtr image;
    image = cv_bridge::toCvCopy(forwardImageleftMsg, sensor_msgs::image_encodings::MONO8);
    
    image->image.copyTo(forwardleftImage);
    //imshow("forwardleft",forwardleftImage);
    //waitKey(1);
}
void rightImage_callback(const sensor_msgs::Image& rightImageMsg)
{
    cv_bridge::CvImagePtr image;
    image = cv_bridge::toCvCopy(rightImageMsg, sensor_msgs::image_encodings::MONO8);
    //image->image.copyTo(control4.rightImage);
    //imshow("右图像",control4.rightImage);
    //waitKey(1);
}
void leftImage_callback(const sensor_msgs::Image& leftImageMsg)
{
    cv_bridge::CvImagePtr image;
    image = cv_bridge::toCvCopy(leftImageMsg, sensor_msgs::image_encodings::MONO8);
   // image->image.copyTo(control4.leftImage);
    //imshow("左图像",control4.leftImage);
   //waitKey(1);
}

void depthImgCallback(const sensor_msgs::Image& depthImageMsg)
{
    cv_bridge::CvImagePtr image = cv_bridge::toCvCopy(depthImageMsg, sensor_msgs::image_encodings::TYPE_16SC1);
    image->image.copyTo(depthImage);
}



//get depth of specified rect
int getDepth(Rect& rect)
{
    int num=0; 
    int length=0;
    vector<int> array;

    for(int i= rect.y ; i < rect.y + rect.height; ++i)
        for(int j = rect.x; j < rect.x + rect.width; ++j)
        {

            int depth = static_cast<int>(depthImage.at<unsigned short>(cv::Point(i,j)));
            //printf("depth %d\n", depth);
            if(depth > 10 && depth < 1300)
               array.push_back(depth);
        }

    //insert sort
    for(int i=0; i < array.size() ; i++)
    {
        int j = i;
        while(j>0 && array[j] < array[j-1])
        {
            swap(array[j],array[j-1]);
            j--;
        }
    }

    int n=0; 
    for(int k=0;k<array.size();k++)
    {

        //n++;
        //if(n>length*0.25&&n<length*0.5)
         printf("%d ",array[k]);
    }
printf("\n\n");

Mat depth_show;
depthImage.convertTo(depth_show, CV_8UC1);
rectangle( depth_show, rect.tl(), rect.br(), cv::Scalar( 200,200,200), 2, 8, 0 );//画矩形

namedWindow("depth",CV_WINDOW_AUTOSIZE);
setMouseCallback("depth",onMouse,&depthImage);
imshow("depth",depth_show);
waitKey(1);
//printf("------------------\n");
array.clear();
return 0;
}

int  time_click=0;
void onMouse(int event,int x,int y,int flags,void* param)
{
    Mat *img=reinterpret_cast<Mat*>(param);
    if(event==CV_EVENT_LBUTTONDOWN)//left_click
    {
        time_click++;
        cout<<time_click<<endl;
        cout<<x<<" "<<y<<":"<<static_cast<int>(img->at<unsigned short>(cv::Point(x,y)))<<endl<<endl;
    }
}

void getNumberFlag(int number,const int flag)
{
  while(number > 0)
  {
    int lastNum = number % 10;
    numberFlag[lastNum] = flag;
    number = number/10;
  }
}

/*自定义手机回调函数*/
void StartMission3Callback(DJIDrone *drone)
{
  drone->request_sdk_permission_control();
  sleep(1);
  ros::spinOnce();
  //drone->drone_arm();
  //sleep(8);
  ros::spinOnce();
  ifStartMission = true;
control3.isFirstTakeoff=true;
control3.count2 = 1;
targetNumber = initnumber;
drift_start_x = px;
drift_start_y = py;
findTarget_ground =false;

}

void StartMission1Callback(DJIDrone *drone)
{
  drone->request_sdk_permission_control();
  sleep(1);
  ros::spinOnce();
  //drone->drone_arm();
  //sleep(8);
  //ros::spinOnce();
  ifStartMission = true;
control3.isFirstTakeoff=true;
control3.count2 = 1;
targetNumber = initnumber;
drift_start_x =px;
drift_start_y =py;
findTarget_ground =false;

}

void StartMission2Callback(DJIDrone *drone) 
{
  drone->request_sdk_permission_control();
  sleep(1);
  ros::spinOnce();
  //drone->drone_arm();
  //sleep(8); 
  ifStartMission = true;
control3.isFirstTakeoff=true;
control3.count2 = 1;
targetNumber = initnumber;
drift_start_x =px;
drift_start_y =py;

}
void StartMission4Callback(DJIDrone *drone)
{ 
  drone->request_sdk_permission_control();
  sleep(1);
   ros::spinOnce();
  //drone->drone_arm();
  //sleep(8);
  ifStartMission = true;
control3.isFirstTakeoff=true;
control3.count2 = 1;
targetNumber = initnumber;
drift_start_x =px;
drift_start_y =py;
findTarget_ground =false;

}

void StopMissionCallback(DJIDrone *drone)
{
  //close computer
  //system("sudo poweroff");
  drone->request_sdk_permission_control();
  sleep(1);
  ros::spinOnce();
  //drone->drone_arm();
  //sleep(1);
//ros::spinOnce();
  ifStartMission = true;
control3.isFirstTakeoff=true;
control3.count2 = 1;
targetNumber = initnumber;
drift_start_x =px;
drift_start_y =py;
findTarget_ground =false;

}

string type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');
cout<<r<<endl;
  return r;
}
