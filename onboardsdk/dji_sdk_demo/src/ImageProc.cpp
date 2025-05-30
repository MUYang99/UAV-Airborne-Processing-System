#include "ImageProc.h"

#define USEPID 1
int ImageProc::framenum=1;
//constructor
ImageProc::ImageProc()
{
    size_width=640;
    size_height=480;
    altitude = 1300;// mm
    set_alt = 1300.0;
    scale=0.6;

    //目标HSV阀值
      green_lower = cvScalar(35, 43,46,0);
      green_upper = cvScalar(77, 255, 255,0);
      thread_red = cvScalar(4, 100, 40,0);//4,
      thread_blue = cvScalar(108, 130, 40 ,0);

//      const Scalar hsvRedLo( 0,  40,  40);
//      const Scalar hsvRedHi(40, 255, 255);

//      const Scalar hsvGreenLo(41,  40,  40);
//      const Scalar hsvGreenHi(90, 255, 255);

//      const Scalar hsvBlueLo(100,  40,  40);
//      const Scalar hsvBlueHi(140, 255, 255);

     imgThresh_red = cvCreateImage( cvSize(size_width,size_height), IPL_DEPTH_8U , 1 );
     imgThresh_blue = cvCreateImage( cvSize(size_width,size_height), IPL_DEPTH_8U , 1 );
     imgThresh_green= cvCreateImage( cvSize(size_width,size_height), IPL_DEPTH_8U , 1 );
     //cvInitFont( &font, CV_FONT_HERSHEY_SIMPLEX,0.3, 1, 0, 2, 1);
     alt_ind=0;
     w =0;
     frame_count = 0;
     lastheadx=600;
     lastheady=180;

     //detector rectangle
     thresh = 50;//Canny算子边缘检测的参数

     FeaturesSize = 6400;//
     PcaSize = 200;
     //detect num
     input1 = fopen("/home/nvidia/catkin_ws/src/onboardsdk/dji_sdk_demo/src/svmdata/mA.txt","r");//均值矩阵
     input2 = fopen("/home/nvidia/catkin_ws/src/onboardsdk/dji_sdk_demo/src/svmdata/v.txt", "r");//投影矩阵
     mA.create(1, FeaturesSize, CV_32FC1);
     v.create(FeaturesSize, PcaSize,CV_32FC1);

     //均值矩阵input1
     if (input1 == NULL)
         fprintf(stderr, "can't open the mA file");
     else
         OpenPCAData(input1, mA, FeaturesSize, 1);
     fclose(input1);

     //投影矩阵input2
     if (input2 == NULL)
         fprintf(stderr, "can't open the V file");
     else
         OpenPCAData(input2, v, PcaSize, FeaturesSize);
     fclose(input2);

//     float *pr = (float *)v.data;
//     for (int i = 0; i < v.rows; i++)
//         for (int j = 0; j < v.cols; j++)
//                 printf("%lf\n", pr[i*v.cols + j]);


     predict_probability = 0;
     max_nr_attr = 200;
     tem_line = NULL;//定义txt文件的每一行
}

int ImageProc::IsSizeEqual( CvSize x, CvSize y )
{
    if( x.width == y.width && x.height == y.height )
        return(1);
    return(0);
}


/*void ImageProc::MakeControlZero(control_data_t *l_control)
{
    l_control->roll = 0;
    l_control->pitch = 0;
    l_control->yaw = 0;
}*/

int ImageProc::my_round(float t)
{
    int i;
    if(t<0)
    {
       i =  (int)(t-0.5);
    }
    else i= (int)(t+0.5);
    if(i>6) i=6;
    if(i<-6) i=-6;

    return i;
}


/*void ImageProc::limit_control(control_data_t *l_control)
{
    if(l_control->pitch > 1.0) l_control->pitch= 1.0 ;
    else if(l_control->pitch < -1.0) l_control->pitch= -1.0 ;

    if(l_control->roll > 1.0 ) l_control->roll = 1.0;
    else if(l_control->roll < -1.0) l_control->roll = -1.0;

    if(l_control->gaz > 1.0 ) l_control->gaz= 1.0;
    else if(l_control->gaz < -1.0) l_control->gaz = -1.0;

    if(l_control->yaw > 1.0 ) l_control->yaw= 1.0;
    else if(l_control->yaw < -1.0) l_control->yaw = -1.0;
}*/


float ImageProc::absfl( float num )
{
    if( num < 0 )
        return( (float)-1*num );
    return( num );
}

/*void ImageProc::printControlData(control_data_t src)
{
    //PRINT( "ROLL:%f , PITCH:%f\n\n", src.roll,src.pitch );
}*/




void ImageProc::debugDumpMatrix (CvMat *A,const char *Title)
{
    int i =0,j=0;
   fprintf(stdout,"%s\n",Title);
   for (  i=0; i < A->rows;i++ )
   {
      for ( j=0; j < A->cols;j++ )
      {
         fprintf(stdout,"%5.3f  ",cvmGet(A,i,j));
      }
      fprintf(stdout,"\n");
   }
}

float ImageProc::calcDistHSI( CvScalar pix,CvScalar sclr) //Calculate the Distance between two Scalars
{
    int x;
    x = pix.val[0]-sclr.val[0];
    if( abs(x) > 90 )
        x = 180-abs(x);
    if(abs(x)< 6 && (pix.val[1]-sclr.val[1])>0 )//&&(pix.val[2]-sclr.val[2])>0
        return 1;
    else return 0;
    //return( ( abs(x)+abs(pix.val[1]-sclr.val[1]) )/2 );  // abs( h1-h2 ) + abs( (s1-s2)/3 ) -- More dependent on Colour which is in Hue Space
 }

int ImageProc::BackProjectHSI( IplImage *src,IplImage *dst, CvScalar sclr ) // Get a Image corresponding to Specific Image
{
    int i,j;
    CvScalar pix;

    for( i=0;i<src->width;i++ )
        for( j=0;j<src->height;j++ )
        {
                uchar *ptr_src,*ptr_dst;
                ptr_dst = (uchar *) dst->imageData + j * dst->widthStep + i * dst->nChannels;
                ptr_src = (uchar *) src->imageData + j * src->widthStep + i * src->nChannels;
//                if(ptr_src[0] > 170)
//                    ptr_src[0] = 180 - ptr_src[0] ;//H_red 0~15 || 170 ~180thread_red
                pix = cvScalar( ptr_src[0],ptr_src[1],ptr_src[2],0 );
//                ptr_dst[0] = ptr_src[1];

               if( calcDistHSI(pix,sclr) )
                    ptr_dst[0] = 255;
                else
                    ptr_dst[0] = 0;
        }
    return(1);
}

void ImageProc::itoa_ht(float data, char s[], int pos)
{
    char i = pos;
    char m;
    float f;
    f = data;
    if (f > 0)
        s[pos] = '+';
    else s[pos] = '-';

    i = i + 1;
    f = (f>0) ? f : -f;
    m = f;
    s[i++] = m + '0';
    s[i++] = '.';
    f = f*10 - m*10;
    s[i++] = (char)f + '0';

    m = f;
    f = f*10 - m*10;
    s[i++] = (char)f + '0';
    s[i] = '\0';

}

//计算x平均值
int ImageProc::CalAvgX(CvPoint * points ,int count)
{
    int result = 0,i =0;
    for(i=0; i<count;i++)
        result += points[i].x;
    return result/count ;
}

//计算y平均值
int ImageProc::CalAvgY(CvPoint * points ,int count)
{
    int result = 0,i =0;
    for(i=0; i<count;i++)
        result += points[i].y;
    return result/count ;
}

//点集排序，选择排序 bit->small
void ImageProc::SortPoints(CvPoint * points, int count )
{
    int i,j,w;
    CvPoint tmp;
    for(i = 0;i<count;i++)
    {
        w = i;
        for(j=i+1;j<count;j++)
        {
            if(points[j].x > points[w].x)  w=j;
        }
        tmp = points[i];
        points[i] = points[w] ;
        points[w] = tmp;
    }
}


void ImageProc::clip(int n1,int & n,int   n2)
 {
    if (n < n1)
        n=n1;
    if (n > n2)
        n=n2;
}

//参数：单通道灰度图,和连通区域的中心坐标
bool ImageProc::isCircle(Mat & image_H , CvRect & courtouRect)
{
    //判断外界矩形内255像素是否大于一定比例
    Mat roImg(image_H, courtouRect);
    int count = 0;
    for (int i = 0; i<roImg.rows; i++)
    {
        uchar* data = roImg.ptr<uchar>(i);
        for (int j = 0; j<roImg.cols; j++)
            if (data[j] == 255)
                count += 1;
    }
    imshow("iscircle",image_H);
    waitKey(20);
    cout<<"count"<<count<<endl;
    if( count > 0.7*(courtouRect.height*courtouRect.width))
        return true;
    return false;
}

//void Dilation(Mat  )
//{
//  int dilation_type;
//  if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
//  else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
//  else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }

//  Mat element = getStructuringElement( dilation_type,
//                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
//                                       Point( dilation_size, dilation_size ) );
//  ///膨胀操作
//  dilate( src, dilation_dst, element );
//  imshow( "Dilation Demo", dilation_dst );
//}

/*------目标检测-------*/
//棋盘检测
/*
Rect ImageProc::cheesePlaneDetector(Mat &imgRGB, bool &isFindChessboard)
{
    isFindChessboard = false;
    int corner_row = 3;
    int corner_col = 3;

    Mat imgGray;
    cvtColor(imgRGB, imgGray, CV_BGR2GRAY);

    CvSize pattern_size = cvSize(corner_row, corner_col);
    vector<Point2f>corners;

    findChessboardCorners(imgGray, pattern_size, corners, 3 );

    int maxX, minX, maxY, minY;

    if (corners.size() > 0)
    {
        maxX = 0, minX = corners[0].x, maxY = 0, minY = corners[0].y;
        for (int i = 0; i < corners.size(); i++)
        {
            maxX = maxX > corners[i].x ? maxX : corners[i].x;
            minX = minX < corners[i].x ? minX : corners[i].x;
            maxY = maxY > corners[i].y ? maxY : corners[i].y;
            minY = minY < corners[i].y ? minY : corners[i].y;
        }
        Point point1(minX,minY), point2(maxX,maxY);
        rectangle(imgRGB, point1, point2, (0, 0, 255), 2, 8, 0);
        isFindChessboard = true;
    }
    //Point centerPoint((minX+maxX)/2,(minY+maxY)/2);
    return Rect(minX, minY,maxX-minX, maxY-minY);
}*/

//squaresDetect

void ImageProc::getMatrix(Mat& intrinsic_matrix, Vec4d& distortion_coeffs)
{
    fstream file1("/home/exbot/catkin_ws/src/onboardsdk/dji_sdk_demo/src/fisheye/int_file.txt", ios::in);
    fstream file2("/home/exbot/catkin_ws/src/onboardsdk/dji_sdk_demo/src/fisheye/dis_file.txt", ios::in);
    if (!file1 || !file2)
    {
        cout << "open error!" << endl;
    }
    else
    {
        double a;
        for (int i = 0; i < 3; i++)
        {
            double* data = intrinsic_matrix.ptr<double>(i);
            for (int j = 0; j < 3; j++)
            {
                file1 >> a;
                data[j] = a;
            }
        }
        for (int i = 0; i < 4;i++)
        {
            file2 >> a;
            distortion_coeffs[i] = a;
        }
    }
}
void ImageProc::correctImg(Mat& image, Mat intrinsic_matrix, Vec4d distortion_coeffs)
{
    Mat mapx = Mat(image.size(), CV_32FC1);
    Mat mapy = Mat(image.size(), CV_32FC1);
    Mat R = Mat::eye(3, 3, CV_32F);
    initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, R, intrinsic_matrix, image.size(), CV_32FC1, mapx, mapy);
    remap(image, image, mapx, mapy, INTER_LINEAR);
}




/*-------矩形检测---------*/
// from pt0->pt1 and from pt0->pt2
bool ImageProc::findSingleSquares_(Mat& image, Rect& rect,vector<Mat>& rectmat)
{
    vector<vector<Point> > squares;
    if (getSquarePts_(image, squares))
    {
        cutImg(image,squares,rectmat);
        /*int maxX, minX, maxY, minY;
        maxX = maxY = 0;
        minX = squares[0][0].x;  minY = squares[0][0].y;
        for (int j = 0; j < 4; j++)
        {
            maxX = maxX > squares[0][j].x ? maxX : squares[0][j].x;
            minX = minX < squares[0][j].x ? minX : squares[0][j].x;
            maxY = maxY > squares[0][j].y ? maxY : squares[0][j].y;
            minY = minY < squares[0][j].y ? minY : squares[0][j].y;
        }
        rect.x = minX; rect.y = minY;
        rect.height = maxY - minY + 1;
        rect.width = maxX - minX + 1;
        rectangle(image, Point(minX,minY), Point(maxX,maxY), (0, 0, 255), 2, 8, 0);*/
        return true;
    }
    else
        return false;
}

void ImageProc::cutImg(Mat image, vector<vector<Point>> squares, vector<Mat>& rectResult)
{
        for(int i_=0;i_<squares.size();i_++)
        {
        /*vector<Point>longside;   //存储矩形长边的两个顶点
        longside.push_back(squares[i_][1]);
        longside.push_back(squares[i_][2]);

        Mat rot(2, 3, CV_32FC1);   //旋转矩阵
        Mat rotMat = Mat::zeros(image.size(), image.type());   //定义旋转后的矩阵

        Point center = Point(rotMat.cols / 2, rotMat.rows / 2);   //旋转中心
        double angle = atan((longside[0].y - longside[1].y) / ((longside[0].x - longside[1].x)+0.00000001))*180/pi;   //旋转角度
        scale = image.rows/sqrt(image.cols*image.cols+image.rows*image.rows);   //缩放参数

        rot = getRotationMatrix2D(center, angle, scale); //获得旋转矩阵
        warpAffine(image, rotMat, rot, image.size());    //旋转并缩放图像

        vector<Point>resultPoint;   //存储旋转后矩形的4个顶点
        Point point;
        for (int i = 0; i < 4; i++)
        {
            point = getPointAffinedPos(squares[i_][i], Point(image.cols/2, image.rows/2), angle*pi/180);
            resultPoint.push_back(point);   //存储顶点
        }
        //获得矩形的bounding box(近似于矩形本身)
        int maxX, minX, maxY, minY;
        maxX = maxY = 0;
        minX = resultPoint[0].x;  minY = resultPoint[0].y;
        for (int i = 0; i < resultPoint.size(); i++)
        {
            maxX = resultPoint[i].x > maxX ?  resultPoint[i].x :maxX;
            minX =  resultPoint[i].x < minX ? resultPoint[i].x : minX;
            maxY = resultPoint[i].y > maxY ?  resultPoint[i].y : maxY;
            minY =  resultPoint[i].y	 < minY ? resultPoint[i].y : minY;
        }*/
        //裁剪矩形并存储到新图像中
	int maxX, minX, maxY, minY;
        maxX = maxY = 0;
        minX = squares[i_][0].x;  minY = squares[i_][0].y;
        for (int i = 0; i < 4; i++)
        {
            maxX = squares[i_][i].x > maxX ? squares[i_][i].x :maxX;
            minX = squares[i_][i].x < minX ? squares[i_][i].x : minX;
            maxY = squares[i_][i].y > maxY ? squares[i_][i].y : maxY;
            minY = squares[i_][i].y < minY ? squares[i_][i].y : minY;
        }
        Rect rect(minX, minY, maxX - minX + 1, maxY - minY + 1);
        //rectangle(image, Point(minX,minY), Point(maxX,maxY), (0, 0, 255), 2, 8, 0);
	Mat temp(image, rect);
        rectResult.push_back(temp);
        imshow("cutimg",temp);
	waitKey(1);
    }
}
int th1=50;
int th2=220;
int length=40; 

bool ImageProc::getSquarePts_(Mat& image, vector<vector<Point> >& squares)
{
    bool isGetSquares = false;
    squares.clear();   //数组清空

    Mat pyrDown_result, pyrUp_result, edge;
    pyrDown(image, pyrDown_result, Size(image.cols / 2, image.rows / 2));
    pyrUp(pyrDown_result, pyrUp_result, image.size());
    vector<vector<Point> > contours;   //保存检测到的矩形边缘点的数组
    namedWindow("edge",WINDOW_NORMAL);
    createTrackbar("th1","edge",&th1,500,0);
    createTrackbar("th2","edge",&th2,500,0);
    createTrackbar("length","edge",&length,100,0);

     Canny(pyrUp_result, edge,th1,th2, 3);         //Canny算子边缘处理

     dilate(edge, edge, Mat(), Point(-1, -1)); //对边缘处理得结果做膨胀处理
     
     findContours(edge, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);   //对edge进行处理,将检测出的每个轮廓（相互独立）以点集（拐点）的形式存储到contours中
     vector<Point> approx;   //存储矩形4个顶点的坐标的数组
     for (size_t i = 0; i < contours.size(); i++)
     {
         approxPolyDP(Mat(contours[i]), approx, length, true);//输出封闭的多边形顶点点集
         //判断是否为四边形等条件
         if (approx.size() == 4 && fabs(contourArea(Mat(approx))) > 1000 && isContourConvex(Mat(approx)))
         {
             double maxCosine = 0;

             for (int j = 2; j < 5; j++)
             {
                 double cosine = fabs(getAngle(approx[j % 4], approx[j - 2], approx[j - 1]));   //计算夹角余弦值
                 maxCosine = MAX(maxCosine, cosine);
             }
             //如果三个夹角余弦值都小于0.3则说明是矩形，并存储矩形
             if (maxCosine < 0.3)
             {
                 squares.push_back(approx);
                 isGetSquares=true; 
             }
         }else{
            int cotourArea = fabs(contourArea(contours[i]));
            int blackPixCount = getblackPixNum(image, approx);
            cout<<" area:  "<<cotourArea<<endl;

	    if(cotourArea > 500)
	    { 
                   Rect  boundRect = boundingRect( Mat(approx));
		   putRectNumber(image,boundRect,cotourArea);
		   //squares.push_back(approx);
                    //isGetSquares=true;
	    }
        cout<<"end"<<endl;
           
         }
     }
     imshow("edge",edge);

     imshow("guidance",image);
     waitKey(1);
    return isGetSquares;
}


int ImageProc::getblackPixNum(Mat& image, vector<Point>& approx)
{
    if(approx.size() < 4)
        return false;

    int blackPixCount = 0;

    Rect  boundRect = boundingRect( Mat(approx));

    int minx = boundRect.x;
    int miny = boundRect.y;
    int maxx = minx + boundRect.width - 2;
    int maxy = miny + boundRect.height - 2;

  for (int i = miny; i != maxy; ++i)
  {
	  uchar* pData = image.ptr<uchar>(i);
	  for (int j = minx ; j != maxx; ++j)        
	    if (pData[j] < 50)
	       ++blackPixCount;
  }
             
    return blackPixCount;

}

  void ImageProc::putRectNumber(cv::Mat& image, const cv::Rect& rect,
                                const int area)
  {
    cv::rectangle( image, rect.tl(), rect.br(), cv::Scalar( 50,50,50), 2, 8, 0 );//画矩形
    //设置绘制文本的相关参数
    std::string text = std::to_string(area);

    int font_face = cv::FONT_HERSHEY_COMPLEX; 
    double font_scale = 0.8;
    int thickness = 1;
    int baseline;
    //获取文本框的长宽
    cv::Size text_size = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);
  
    //将文本框居中绘制
    cv::Point origin(rect.tl().x + 10, rect.tl().y+10);
    cv::putText(image, text, origin, font_face, font_scale, cv::Scalar(255, 255, 255), thickness, 8, 0);
  }


bool ImageProc::findBristlingSquares(Mat& image,vector<Rect>&rects)
{
    vector<vector<Point> > squares;
    if (getSquarePtsByThreshold(image, squares))
    { 
        int maxX, minX, maxY, minY;
        maxX = maxY = 0;
        for(int i=0;i<squares.size();i++)
        {
            minX = squares[i][0].x;
            minY = squares[i][0].y;
            for (int j = 0; j < 4; j++)
            {
                maxX = maxX > squares[i][j].x ? maxX : squares[i][j].x;
                minX = minX < squares[i][j].x ? minX : squares[i][j].x;
                maxY = maxY > squares[i][j].y ? maxY : squares[i][j].y;
                minY = minY < squares[i][j].y ? minY : squares[i][j].y;
            }
            Rect rect;
            rect.x = minX; rect.y = minY;
            rect.height = maxY - minY + 1;
            rect.width = maxX - minX + 1;
            rects.push_back(rect);
            rectangle(image, Point(minX,minY), Point(maxX,maxY), (0, 0, 255), 2, 8, 0);
        }
        return true;
    }
    else
           return false;
}

bool ImageProc::getSquarePtsByThreshold(Mat image, vector<vector<Point> >& squares)
{
    bool isGetSquares = false;
    squares.clear();   //数组清空

    Mat pyr, timg, gray;
    pyrDown(image, pyr, Size(image.cols / 2, image.rows / 2));
    pyrUp(pyr, timg, image.size());
    Mat imageHSV,image_H;
    cvtColor(timg, imageHSV, CV_BGR2HSV);     //将image转到HSV空间
    for(int i = 0;i<imageHSV.rows;i++)
    {
        uchar* data = imageHSV.ptr<uchar>(i);
        for(int j=0;j<imageHSV.cols;j++)
        {
            if(data[j*imageHSV.channels()]>12&&data[j*imageHSV.channels()]<30&&data[j*imageHSV.channels()+1]>100&&data[j*imageHSV.channels()+2]>80)
                data[j*imageHSV.channels()] = 0;
            else
                data[j*imageHSV.channels()] = 255;
        }
    }
    image_H.create(imageHSV.size(), imageHSV.depth());   //定义与imageHSV同尺寸和深度的图像image_H

    int ch1[] = { 0, 0 };
    mixChannels(&imageHSV, 1, &image_H, 1, ch1, 1);   //将imageHSV的H层复制到image_H
    imshow("image_H",image_H);

    vector<vector<Point> > contours;   //保存检测到的矩形边缘点的数组

     Canny(image_H, image_H, 0, 50, 5);         //Canny算子边缘处理
     dilate(image_H, gray, Mat(), Point(-1, -1)); //对边缘处理得结果做膨胀处理
     imshow("G",gray);

     findContours(gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);   //对gray进行处理,将检测出的每个轮廓（相互独立）以点集（拐点）的形式存储到contours中
     vector<Point> approx;   //存储矩形4个顶点的坐标的数组

     for (size_t i = 0; i < contours.size(); i++)
     {
         approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);//输出封闭的多边形顶点点集
         //判断是否为四边形等条件
         if (approx.size() == 4 && fabs(contourArea(Mat(approx))) > 2000 && isContourConvex(Mat(approx)))
         {
             double maxCosine = 0;

             for (int j = 2; j < 5; j++)
             {
                 double cosine = fabs(getAngle(approx[j % 4], approx[j - 2], approx[j - 1]));   //计算夹角余弦值
                 maxCosine = MAX(maxCosine, cosine);
             }
             //如果三个夹角余弦值都小于0.3则说明是矩形，并存储矩形
             if (maxCosine < 0.5)
             {
                 squares.push_back(approx);
                 isGetSquares=true;
             }
         }
     }
    return isGetSquares;
}






double ImageProc::getAngle(Point pt1, Point pt2, Point pt0)
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2) / sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

bool ImageProc::isYellowSquare(Mat image_H, vector<Point> square)
{
    int pointX[4];
    int pointY[4];
    for (int i = 0; i < 4; i++)
    {
        pointX[i] = square[i].x;
        pointY[i] = square[i].y;
    }
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 3 - i; j++)
        {
            if (pointX[j] > pointX[j + 1])
            {
                int tx = pointX[j];
                pointX[j] = pointX[j + 1];
                pointX[j + 1] = tx;
            }
            if (pointY[j] > pointY[j + 1])
            {
                int ty = pointY[j];
                pointY[j] = pointY[j + 1];
                pointY[j + 1] = ty;
            }
        }
    }
    Rect rect(pointX[0], pointY[0], pointX[3] - pointX[0] + 1, pointY[3] - pointY[0] + 1);
    Mat roImg(image_H, rect);
    int count = 0;
    for (int i = 0; i<roImg.rows; i++)
    {
        uchar* data = roImg.ptr<uchar>(i);
        for (int j = 0; j<roImg.cols; j++)
        {
            if (data[j] == 0)
                count += 1;
        }
    }
    if (count>0.8*((roImg.cols*roImg.rows)/2 + (pointX[2]-pointX[1]+1)*(pointY[2]-pointY[1]+1)/2))
        return true;
    else
        return false;
}

bool ImageProc::getSquarePts(Mat image, vector<vector<Point> >& squares)
{
    bool isGetSquares = false;
    squares.clear();   //数组清空

    Mat pyr, timg, gray0(image.size(), CV_8U), gray,grayImg;
    cvtColor(image, grayImg, CV_RGB2GRAY);
    Mat imageHSV,image_H;
    cvtColor(image, imageHSV, CV_BGR2HSV);     //将image转到HSV空间
    for(int i = 0;i<imageHSV.rows;i++)
    {
        uchar* data = imageHSV.ptr<uchar>(i);
        for(int j=0;j<imageHSV.cols;j++)
        {
            if(data[j*imageHSV.channels()]>12&&data[j*imageHSV.channels()]<30&&data[j*imageHSV.channels()+1]>100&&data[j*imageHSV.channels()+2]>80)
                data[j*imageHSV.channels()] = 0;
            else
                data[j*imageHSV.channels()] = 255;
        }
    }
    image_H.create(imageHSV.size(), imageHSV.depth());   //定义与imageHSV同尺寸和深度的图像image_H

    int ch1[] = { 0, 0 };
    mixChannels(&imageHSV, 1, &image_H, 1, ch1, 1);   //将imageHSV的H层复制到image_H
    //imshow("image_H",image_H);
    pyrDown(grayImg, pyr, Size(image.cols / 2, image.rows / 2));
    pyrUp(pyr, timg, image.size());
    vector<vector<Point> > contours;   //保存检测到的矩形边缘点的数组

    int ch[] = { 0, 0 };
    mixChannels(&timg, 1, &gray0, 1, ch, 1);  //将去燥后的image复制到gray0
    //对gray0进行处理

     Canny(gray0, gray, 0, 50, 3);         //Canny算子边缘处理
     dilate(gray, gray, Mat(), Point(-1, -1)); //对边缘处理得结果做膨胀处理

     imshow("G",gray);
     findContours(gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);   //对gray进行处理,将检测出的每个轮廓（相互独立）以点集（拐点）的形式存储到contours中
     vector<Point> approx;   //存储矩形4个顶点的坐标的数组

     for (size_t i = 0; i < contours.size(); i++)
     {
         approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);//输出封闭的多边形顶点点集
         //判断是否为四边形等条件
         if (approx.size() == 4 && fabs(contourArea(Mat(approx))) > 1000 && isContourConvex(Mat(approx)))
         {
             double maxCosine = 0;

             for (int j = 2; j < 5; j++)
             {
                 double cosine = fabs(getAngle(approx[j % 4], approx[j - 2], approx[j - 1]));   //计算夹角余弦值
                 maxCosine = MAX(maxCosine, cosine);
             }
             //如果三个夹角余弦值都小于0.3则说明是矩形，并存储矩形
             if (maxCosine < 0.3 && isYellowSquare(image_H,approx))
             {
                 squares.push_back(approx);
                 isGetSquares=true;
             }
         }
     }
    return isGetSquares;
}



//旋转图像中点的函数
Point ImageProc::getPointAffinedPos(const Point src, const Point center, double angle)
{
    Point dst;
    int x = src.x - center.x;
    int y = src.y - center.y;

    dst.x = cvRound(x * cos(angle) + y * sin(angle) + center.x);
    dst.y = cvRound(-x * sin(angle) + y * cos(angle) + center.y);

    dst.x = (dst.x - center.x)*scale + center.x;
    dst.y = (dst.y - center.y)*scale + center.y;

    return dst;
}

//裁切


// the function draws all the squares in the image
void ImageProc::drawSquares(Mat& image, vector<vector<Point> > squares)
{
    int maxX, minX, maxY, minY;
    for (int i = 0; i < squares.size(); i++)
    {
        maxX = maxY = 0;
        minX = squares[i][0].x;  minY = squares[i][0].y;
        for (int j = 0; j < 4; j++)
        {
            maxX = maxX > squares[i][j].x ? maxX : squares[i][j].x;
            minX = minX < squares[i][j].x ? minX : squares[i][j].x;
            maxY = maxY > squares[i][j].y ? maxY : squares[i][j].y;
            minY = minY < squares[i][j].y ? minY : squares[i][j].y;
        }
        Point point1, point2;
        point1.x = minX; point1.y = minY;
        point2.x = maxX; point2.y = maxY;

        rectangle(image, point1, point2, (0, 0, 255), 2, 8, 0);   //画矩形
    }
    //imshow("矩形检测结果", image);
}


bool ImageProc::findSquares(Mat image, vector<Mat>& resultImage,vector<Point>& center)
{
    resultImage.clear();
    center.clear();
    
    vector<vector<Point> > squarePts;
    if (getSquarePts_(image, squarePts))
    {
      cutImg(image, squarePts, resultImage);
      drawSquares(image, squarePts);
      for (int i = 0; i < squarePts.size(); i++)
      {
        Point point;
        point.x = (squarePts[i][0].x + squarePts[i][2].x) / 2;
        point.y = (squarePts[i][0].y + squarePts[i][2].y) / 2;
        center.push_back(point);
      }
      return true;
    }
         
    return false;
}


void ImageProc::regionGrow(Mat& grayImg)
{
   /*Mat srcImg = imread("raw.jpg",1);
       Mat grayImg;

       cvtColor(srcImg,grayImg,CV_BGR2GRAY);*/

 
/*
       Mat binaryImg = Mat::zeros(grayImg.rows,grayImg.cols,CV_8UC1);

       adaptiveThreshold(grayImg,binaryImg,255,ADAPTIVE_THRESH_GAUSSIAN_C,THRESH_BINARY_INV,5,10);

       namedWindow("threshold", CV_WINDOW_NORMAL); 

       imshow("threshold",binaryImg); 

 



       findContours(binaryImg,vContours,vHierarchy,CV_RETR_LIST,CV_CHAIN_APPROX_NONE);*/

 

    Mat pyr, timg, gray0(grayImg.size(), CV_8U), edge;
    //imshow("image_H",image_H);
    pyrDown(grayImg, pyr, Size(grayImg.cols / 2, grayImg.rows / 2));
    pyrUp(pyr, timg, grayImg.size());
    vector<vector<Point> > vContours;   //保存检测到的矩形边缘点的数组
    vector< Vec4i > vHierarchy;

    int ch[] = { 0, 0 };
    mixChannels(&timg, 1, &gray0, 1, ch, 1);  //将去燥后的image复制到gray0
    //对gray0进行处理

     Canny(gray0, edge, 50, 220, 3);         //Canny算子边缘处理
     dilate(edge, edge, Mat(), Point(-1, -1)); //对边缘处理得结果做膨胀处理
findContours(edge, vContours, vHierarchy,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);   //对edge进行处理,将检测出的每个轮廓（相互独立）以点集（拐点）的形式存储到contours中







       Mat markersImg = Mat::zeros(grayImg.rows,grayImg.cols,CV_8UC1);

       //for (int idx = 0; idx >= 0;idx=vHierarchy[idx][0])

       //{

              Scalar color(rand()&255,rand()&255,rand()&255);

              drawContours(markersImg,vContours,0,color,CV_FILLED,8,vHierarchy);

       //}

       namedWindow("contours", CV_WINDOW_NORMAL); 
       imshow("contours",markersImg); 

 

       Mat markers;
       markersImg.convertTo(markers, CV_32S); 
 //cout<<markers.channels<<" "<<grayImg.channels<<endl;

       Mat srcImg;
       cvtColor(grayImg , srcImg, CV_GRAY2BGR);

       watershed(srcImg, markers);

       Mat segmentImg;

       markers.convertTo(segmentImg,CV_8U);

 

       namedWindow("segmentation_result", CV_WINDOW_NORMAL); 

       imshow("segmentation_result", segmentImg); 

       waitKey(1);
}

//for mission 2
bool ImageProc::getSquaresByContours(Mat image, Point& point, Rect rect)
{
    Mat imageHSV, image_H;
    cvtColor(image, imageHSV, CV_BGR2HSV);
    image_H.create(imageHSV.size(), imageHSV.depth());

    int ch1[] = { 0, 0 };
    mixChannels(&imageHSV, 1, &image_H, 1, ch1, 1);

    for (int i = 0; i < image_H.rows; i++)
    {
        uchar* data = image_H.ptr<uchar>(i);
        for (int j = 0; j < image_H.cols; j++)
        {
            if (data[j] >= 17 && data[j] <= 32)
                data[j] = 0;
            else
                data[j] = 255;
        }
    }
    medianBlur(image_H, image_H, 5);
    vector<vector<Point> > contours;
    vector<vector<Point> >::iterator i;
    vector<Point>::iterator j;
    findContours(image_H, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
    if (contours.size() > 0)
    {
        for (i = contours.begin(); i != contours.end();)
        {
            int area = contourArea(Mat(*i));
            if (area < 8000 || area > 60000)
                i = contours.erase(i);
            else
                ++i;
        }
        if (contours.size() > 0)
        {
            int centerX = 0, centerY = 0;
            for (i = contours.begin(); i != contours.end();)
            {
                for (j = (*i).begin(); j != (*i).end();j++)
                {
                    centerX += (*j).x;
                    centerY += (*j).y;
                }
                centerX = centerX / (*i).size();
                centerY = centerY / (*i).size();
                if ((centerX - point.x)*(centerX - point.x) + (centerY - point.y)*(centerY - point.y) > 1000)
                    i = contours.erase(i);
                else
                    ++i;
            }
            if (contours.size() > 0)
            {
                for (int i = 0; i < contours[0].size(); i++)
                {
                    point.x += contours[0][i].x;
                    point.y += contours[0][i].y;
                }
                point.x = point.x / contours[0].size();
                point.y = point.y / contours[0].size();
                rectangle(image, Point(point.x - rect.width / 2, point.y - rect.height / 2),
                                                                        Point(point.x + rect.width / 2, point.y + rect.height / 2), (0, 0, 255), 2, 8, 0);
                return true;
            }
            else
                return false;
        }
    }
    else
        return false;
}


bool ImageProc::findSingleSquares(Mat& image, Rect& rect)
{
    vector<vector<Point> > squares;
    if (getSquarePts(image, squares))
    {
        int maxX, minX, maxY, minY;
        maxX = maxY = 0;
        minX = squares[0][0].x;  minY = squares[0][0].y;
        for (int j = 0; j < 4; j++)
        {
            maxX = maxX > squares[0][j].x ? maxX : squares[0][j].x;
            minX = minX < squares[0][j].x ? minX : squares[0][j].x;
            maxY = maxY > squares[0][j].y ? maxY : squares[0][j].y;
            minY = minY < squares[0][j].y ? minY : squares[0][j].y;
        }
        rect.x = minX; rect.y = minY;
        rect.height = maxY - minY + 1;
        rect.width = maxX - minX + 1;
        rectangle(image, Point(minX,minY), Point(maxX,maxY), (0, 0, 255), 2, 8, 0);
        return true;
    }
    else
           return false;
}

/*------数字识别-------*/
int ImageProc::getNumber(Mat &image)
{
    int predict_num=-2;
    Mat  gray_img1, gray_img2, gray_img3;//img原始图片，gray_img1灰度化图片，gray_img2归一化后的灰度图片
    imshow("numberimage",image);
    waitKey(20);
    if(true)
   {
       Mat tem_feattureVector, featureVector;//tem_feattureVector为得到的LBP特征向量，featureVector为降维后的LBP特征向量


       //读取图片、灰度化、归一化
       //img = imread("C:\\Users\\Administrator.PC-20161112GTCE\\Desktop\\c++\\lbp\\7_46.jpg", CV_LOAD_IMAGE_COLOR);
       cvtColor(image, gray_img1, CV_BGR2GRAY);
       my_imresize(gray_img1, gray_img2, 140, 140);//归一化为[100,100]
       gray_img3 = gray_img2(Range(20,120), Range(20,120));
       //printf("%d\t%d\n",gray_img3.rows,gray_img3.cols);
//       uchar *pr;
//       for (int i = 0; i < gray_img3.rows; i++)
//       {
//           pr = gray_img3.ptr<uchar>(i);
//           for (int j = 0; j < gray_img3.cols; j++)
//           {
//               pr[ j ] = uchar( 1/(1+pow(80/double(pr[ j ]),4.5))*255);
//           }
//       }

//       imshow("gray3",gray_img3);
//       waitKey(10);

       //定义统计特征的窗口大小，窗口为25*25时，LBP特征维数为4*4*256
       Size cellSize;
       cellSize.width = 20; cellSize.height = 20;

       ComputeLBPFeatureVector_256(gray_img3, cellSize, tem_feattureVector);
       fastPCA(tem_feattureVector, featureVector);
//              float *pr;
//              for (int i = 0; i < featureVector.rows; i++)
//              {
//                  pr = featureVector.ptr<float>(i);
//                  for (int j = 0; j < featureVector.cols; j++)
//                  {
//                      cout<<pr[ j ]<<"\t";
//                  }
//              }
       //printf("%d\t%d\n",featureVector.rows,featureVector.cols);
       predict_num = svmpredict(featureVector);//预测的结果

//       waitKey(0);
//       free(tem_line);
//       getchar();

   }
   return predict_num;
}


void ImageProc::ComputeLBPFeatureVector_256(const Mat &srcImage, Size cellSize, Mat &featureVector)
{
    // 参数检查，内存分配
    CV_Assert(srcImage.depth() == CV_8U&&srcImage.channels() == 1);

    Mat LBPImage;
    ComputeLBPImage_256(srcImage, LBPImage);
    imshow("LBPimage.jpg", LBPImage);
    waitKey(10);

    // 计算cell个数
    int widthOfCell = cellSize.width;
    int heightOfCell = cellSize.height;
    int numberOfCell_X = srcImage.cols / widthOfCell;// X方向cell的个数
    int numberOfCell_Y = srcImage.rows / heightOfCell;

    // 特征向量的个数
    int numberOfDimension = 256 * numberOfCell_X*numberOfCell_Y;
    featureVector.create(1, numberOfDimension, CV_32FC1);
    featureVector.setTo(Scalar(0));

    // 计算LBP特征向量
    int stepOfCell = srcImage.cols;
    int pixelCount = cellSize.width*cellSize.height;
    float *dataOfFeatureVector = (float *)featureVector.data;

    // cell的特征向量在最终特征向量中的起始位置
    int index = -256;
    for (int y = 0; y <= numberOfCell_Y - 1; ++y)
    {
        for (int x = 0; x <= numberOfCell_X - 1; ++x)
        {
            index += 256;

            // 计算每个cell的LBP直方图
            Mat cell = LBPImage(Rect(x * widthOfCell, y * heightOfCell, widthOfCell, heightOfCell));
            uchar *rowOfCell = cell.data;
            for (int y_Cell = 0; y_Cell <= cell.rows - 1; ++y_Cell, rowOfCell += stepOfCell)
            {
                uchar *colOfCell = rowOfCell;
                for (int x_Cell = 0; x_Cell <= cell.cols - 1; ++x_Cell, ++colOfCell)
                {
                    ++dataOfFeatureVector[index + colOfCell[0]];
                }
            }

            // 一定要归一化！否则分类器计算误差很大
            for (int i = 0; i <= 255; ++i)
            {
                dataOfFeatureVector[index + i] /= pixelCount;
                //printf("%f\n", dataOfFeatureVector[index + i]);
            }
        }
    }

}
//srcImage:灰度图
//LBPImage:LBP图
void ImageProc::ComputeLBPImage_256(const Mat &srcImage, Mat &LBPImage)
{
    // �������飬�ڴ�����
    CV_Assert(srcImage.depth() == CV_8U&&srcImage.channels() == 1);
    LBPImage.create(srcImage.size(), srcImage.type());

    // ���ԭͼ���߽磬���ڱ߽紦��
    Mat extendedImage, tem_vec1, tem_vec2;
    copyMakeBorder(srcImage, extendedImage, 1, 1, 1, 1, BORDER_CONSTANT, Scalar::all(255));
    tem_vec1.create(1, LBPImage.cols + 2, CV_32FC1);
    tem_vec2.create(LBPImage.rows + 2, 1, CV_32FC1);
    tem_vec1.setTo(Scalar(255));
    tem_vec2.setTo(Scalar(255));
    tem_vec1.row(0).copyTo(extendedImage.row(0));
    tem_vec1.row(0).copyTo(extendedImage.row(LBPImage.rows + 1));
    tem_vec2.col(0).copyTo(extendedImage.col(0));
    tem_vec2.col(0).copyTo(extendedImage.col(LBPImage.cols + 1));

    // ����LBP����ͼ
    int heightOfExtendedImage = extendedImage.rows;
    int widthOfExtendedImage = extendedImage.cols;
    int widthOfLBP = LBPImage.cols;
    uchar *rowOfExtendedImage = extendedImage.data + widthOfExtendedImage + 1;
    uchar *rowOfLBPImage = LBPImage.data;
    for (int y = 1; y <= heightOfExtendedImage - 2; ++y, rowOfExtendedImage += widthOfExtendedImage, rowOfLBPImage += widthOfLBP)
    {
        // ��
        uchar *colOfExtendedImage = rowOfExtendedImage;
        uchar *colOfLBPImage = rowOfLBPImage;
        for (int x = 1; x <= widthOfExtendedImage - 2; ++x, ++colOfExtendedImage, ++colOfLBPImage)
        {
            // ����LBPֵ
            int LBPValue = 0;
            /*int temp;
            temp = colOfExtendedImage[0 - widthOfExtendedImage];
            cout << temp << endl;*/
            if (colOfExtendedImage[0 - widthOfExtendedImage - 1] > colOfExtendedImage[0])
                LBPValue += 64;
            if (colOfExtendedImage[0 - widthOfExtendedImage] > colOfExtendedImage[0])
                LBPValue += 32;
            if (colOfExtendedImage[0 - widthOfExtendedImage + 1] > colOfExtendedImage[0])
                LBPValue += 16;
            if (colOfExtendedImage[0 + 1] > colOfExtendedImage[0])
                LBPValue += 8;
            if (colOfExtendedImage[0 + widthOfExtendedImage + 1] > colOfExtendedImage[0])
                LBPValue += 4;
            if (colOfExtendedImage[0 + widthOfExtendedImage] > colOfExtendedImage[0])
                LBPValue += 2;
            if (colOfExtendedImage[0 + widthOfExtendedImage - 1] > colOfExtendedImage[0])
                LBPValue += 1;
            if (colOfExtendedImage[0 - 1] > colOfExtendedImage[0])
                LBPValue += 128;

            colOfLBPImage[0] = LBPValue;
            //printf("%d\n", LBPValue);
        }  // x
    }// y
}

void ImageProc::fastPCA(Mat tem_feattureVector, Mat &featureVector)
{
    Mat tem_result;
    tem_result.create(1, FeaturesSize, CV_32FC1);

    //进行降维变换
    tem_result = tem_feattureVector - mA;
    featureVector = tem_result * v;
//    float *pr = (float *)featureVector.data;
//         for (int i = 0; i < featureVector.rows; i++)
//             for (int j = 0; j < featureVector.cols; j++)
//                   printf("%lf\n", pr[i*featureVector.cols + j]);
//    float *pr = (float *)mA.data;
//    for (int i = 0; i < mA.rows; i++)
//        for (int j = 0; j < mA.cols; j++)
//            printf("%lf\n", pr[i*mA.cols + j]);
}

void ImageProc::OpenPCAData(FILE *input, Mat &result, int width, int height)
{
    int i=0, j=0;
    char *value, *pEnd;
    result.create(height, width, CV_32FC1);
    result.setTo(Scalar(0));

    float *DataResult = (float *)result.data;
    max_line_len = 6400;
    tem_line = (char *)malloc(max_line_len*sizeof(char));

    while (readline(input)!=NULL)
    {
        j = 0;
        while (1)
        {
            if (j==0)
                value = strtok(tem_line, "\t");
            else
                value = strtok(NULL, "\t");
            if (value == NULL)
                break;
            DataResult[i*width+j] = strtod(value, &pEnd);
            j++;
        }
        i++;
    }
}

char* ImageProc::readline(FILE *input)
{
    int len;

    if (fgets(tem_line, max_line_len, input) == NULL)
        return NULL;

    while (strrchr(tem_line, '\n') == NULL)
    {
        max_line_len *= 2;
        tem_line = (char *)realloc(tem_line, max_line_len);
        len = (int)strlen(tem_line);
        if (fgets(tem_line + len, max_line_len - len, input) == NULL)
            break;
    }
    return tem_line;
}

double ImageProc::svmpredict(Mat &featureVector)
{
    //printf("run!");
    double predict_label = -2;

    //读取model文件  /home/exbot/catkin_ws/src/sendbb/src/svmdata
    if((model = svm_load_model("/home/exbot/catkin_ws/src/onboardsdk/dji_sdk_demo/src/svmdata/train_data.model"))!=0)
    {
        //printf("model run!\n");
    }

    float *pr_featureVector = (float *)featureVector.data;
    int svm_type = svm_get_svm_type(model);
    int nr_class = svm_get_nr_class(model);
    double *prob_estimates = NULL;

    int i,j;
    if (predict_probability)
    {
        if (svm_type == NU_SVR || svm_type == EPSILON_SVR)
            printf("Prob. model for test data: target value = predicted value + z,\nz: Laplace distribution e^(-|z|/sigma)/(2sigma),sigma=%g\n", svm_get_svr_probability(model));
        else
        {
            int *labels = (int *)malloc(nr_class*sizeof(int));
            svm_get_labels(model, labels);
            prob_estimates = (double *)malloc(nr_class*sizeof(double));
            free(labels);
        }
    }


    x = (struct svm_node *) realloc(x, max_nr_attr*sizeof(struct svm_node));
    for (i = 0; i < featureVector.cols;i++)
    {
        x[i].index = i + 1;
        x[i].value = pr_featureVector[i];
        //printf("%d\t", i);
        //printf("%d\t", x[i].index);
        //printf("%f\t", x[i].value);
    }
    x[i].index = -1;
    //printf("%d\t", x[200].index);
    if (predict_probability && (svm_type == C_SVC || svm_type == NU_SVC))
    {
        predict_label = svm_predict_probability(model, x, prob_estimates);
    }
    else
    {
        predict_label = svm_predict(model, x);
    }
    return predict_label;
}

void ImageProc::my_imresize(Mat matSrc, Mat &matDst1, int height, int width)
{
    matDst1 = Mat(Size(height, width), matSrc.type(), Scalar::all(0));

    double scale_x = (double)matSrc.cols / matDst1.cols;
    double scale_y = (double)matSrc.rows / matDst1.rows;

    int iscale_x = cv::saturate_cast<int>(scale_x);
    int iscale_y = cv::saturate_cast<int>(scale_y);

    for (int j = 0; j < matDst1.rows; ++j)
    {
        float fy = (float)((j + 0.5) * scale_y - 0.5);
        int sy = cvFloor(fy);
        fy -= sy;
        sy = min(sy, matSrc.rows - 3);
        sy = max(1, sy);

        const float A = -0.75f;

        float coeffsY[4];
        coeffsY[0] = ((A*(fy + 1) - 5 * A)*(fy + 1) + 8 * A)*(fy + 1) - 4 * A;
        coeffsY[1] = ((A + 2)*fy - (A + 3))*fy*fy + 1;
        coeffsY[2] = ((A + 2)*(1 - fy) - (A + 3))*(1 - fy)*(1 - fy) + 1;
        coeffsY[3] = 1.f - coeffsY[0] - coeffsY[1] - coeffsY[2];


        short cbufY[4];
        cbufY[0] = cv::saturate_cast<short>(coeffsY[0] * 2048);
        cbufY[1] = cv::saturate_cast<short>(coeffsY[1] * 2048);
        cbufY[2] = cv::saturate_cast<short>(coeffsY[2] * 2048);
        cbufY[3] = cv::saturate_cast<short>(coeffsY[3] * 2048);


        for (int i = 0; i < matDst1.cols; ++i)
        {
            float fx = (float)((i + 0.5) * scale_x - 0.5);
            int sx = cvFloor(fx);
            fx -= sx;

            if (sx < 1) {
                fx = 0, sx = 1;
            }
            if (sx >= matSrc.cols - 3) {
                fx = 0, sx = matSrc.cols - 3;
            }

            float coeffsX[4];
            coeffsX[0] = ((A*(fx + 1) - 5 * A)*(fx + 1) + 8 * A)*(fx + 1) - 4 * A;
            coeffsX[1] = ((A + 2)*fx - (A + 3))*fx*fx + 1;
            coeffsX[2] = ((A + 2)*(1 - fx) - (A + 3))*(1 - fx)*(1 - fx) + 1;
            coeffsX[3] = 1.f - coeffsX[0] - coeffsX[1] - coeffsX[2];

            short cbufX[4];
            cbufX[0] = cv::saturate_cast<short>(coeffsX[0] * 2048);
            cbufX[1] = cv::saturate_cast<short>(coeffsX[1] * 2048);
            cbufX[2] = cv::saturate_cast<short>(coeffsX[2] * 2048);
            cbufX[3] = cv::saturate_cast<short>(coeffsX[3] * 2048);

            float temp_value;
            temp_value = abs((
                (int)matSrc.at<uchar>(sy - 1, sx - 1) * cbufX[0] * cbufY[0] +
                (int)matSrc.at<uchar>(sy, sx - 1) * cbufX[0] * cbufY[1] +
                (int)matSrc.at<uchar>(sy + 1, sx - 1) * cbufX[0] * cbufY[2] +
                (int)matSrc.at<uchar>(sy + 2, sx - 1) * cbufX[0] * cbufY[3] +
                (int)matSrc.at<uchar>(sy - 1, sx) * cbufX[1] * cbufY[0] +
                (int)matSrc.at<uchar>(sy, sx) * cbufX[1] * cbufY[1] +
                (int)matSrc.at<uchar>(sy + 1, sx) * cbufX[1] * cbufY[2] +
                (int)matSrc.at<uchar>(sy + 2, sx) * cbufX[1] * cbufY[3] +
                (int)matSrc.at<uchar>(sy - 1, sx + 1) * cbufX[2] * cbufY[0] +
                (int)matSrc.at<uchar>(sy, sx + 1) * cbufX[2] * cbufY[1] +
                (int)matSrc.at<uchar>(sy + 1, sx + 1) * cbufX[2] * cbufY[2] +
                (int)matSrc.at<uchar>(sy + 2, sx + 1) * cbufX[2] * cbufY[3] +
                (int)matSrc.at<uchar>(sy - 1, sx + 2) * cbufX[3] * cbufY[0] +
                (int)matSrc.at<uchar>(sy, sx + 2) * cbufX[3] * cbufY[1] +
                (int)matSrc.at<uchar>(sy + 1, sx + 2) * cbufX[3] * cbufY[2] +
                (int)matSrc.at<uchar>(sy + 2, sx + 2) * cbufX[3] * cbufY[3]) / pow(2, 22));
            //printf("%d\t%f\n", j*matDst1.rows + i + 1, temp_value);
            matDst1.at<uchar>(j, i) = (int)(temp_value + 0.5);
        }
    }
}

ImageProc::~ImageProc()
{
    //释放指针
    cvReleaseImage( &imgThresh_green );
    cvReleaseImage( &imgThresh_red );
    cvReleaseImage( &imgThresh_blue);
   //free(tem_line);
}
