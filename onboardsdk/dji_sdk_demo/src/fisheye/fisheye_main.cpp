#include<opencv2/opencv.hpp>
#include <fstream>
using namespace cv;
using namespace std;

void getMatrix(Mat& intrinsic_matrix, Vec4d& distortion_coeffs,string filename_int,string filename_dis)
{
	fstream file1(filename_int, ios::in);
	fstream file2(filename_dis, ios::in);
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
void correctImg(Mat& image, Mat intrinsic_matrix, Vec4d distortion_coeffs)
{
	Mat mapx = Mat(image.size(), CV_32FC1);
	Mat mapy = Mat(image.size(), CV_32FC1);
	Mat R = Mat::eye(3, 3, CV_32F);
	fisheye::initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, R, intrinsic_matrix, image.size(), CV_32FC1, mapx, mapy);
	remap(image, image, mapx, mapy, INTER_LINEAR);
}

void main()
{
	Mat intrinsic_matrix(Size(3,3), CV_64FC1, Scalar::all(0));
	Vec4d distortion_coeffs;
	getMatrix(intrinsic_matrix, distortion_coeffs, "int_file.txt", "dis_file.txt");
	VideoCapture capture(0);
	Mat image;
	while (true)
	{
		capture >> image;
		correctImg(image, intrinsic_matrix, distortion_coeffs);
		imshow("correct image", image);
		waitKey(30);
	}
}
