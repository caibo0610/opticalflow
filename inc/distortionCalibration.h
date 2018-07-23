#ifndef _DISTORTIONCALIBRATION_H
#define _DISTORTIONCALIBRATION_H


#include "opencv2/opencv.hpp"
#include <iostream>
using namespace std;
using namespace cv;

class DistortionCalibrator
{
	
public:
	DistortionCalibrator(){height = 240; width=320;};
	DistortionCalibrator(int _h, int _w);
	~DistortionCalibrator();

	
	int height;
	int width;
	Size image_size;
	Size cutImage_size;
	
	Matx33f intrinsic_matrix;
	Vec4f distortion_coeffs;
	
	Mat originalImg;
	Mat undistortedImg;
	Mat equalizeHistImg;
	Mat cutImg;
	Mat borderImg;


	Mat intrinsic_mat;
	Mat new_intrinsic_mat;
	

	void initializeDistortionParameters();
	

	void eliminateDistortion(Mat src, Mat dst);
	void CutImag_1_2(Mat src, Mat dst, int w,int h, int offset_x, int offset_y);
	void equalizeHistFunc(Mat src, Mat dst);
	void borderDetection(Mat src, Mat dst);
	

};



#endif //_DISTORTIONCALIBRATION_H
