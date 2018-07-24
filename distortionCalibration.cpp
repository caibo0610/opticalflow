#include "distortionCalibration.h"

float intrinsic_vec[9] = { 273.99497952772407f, 0.0f, 143.51565451116193f, 0.0f, 273.42858716863560f, 110.48756341112096f, 0.0f, 0.0f, 1.0f };
float distortion_coeffs_vec[4] = {0.053806942095731204f,-0.088103822806196011f,-2.2990819704397905f,9.0720659923083584f};
//float intrinsic_vec[9] = { 141.988f,0.0,156.722,0.0,142.495,120.713,0.0,0.0,1.0 };
//float distortion_coeffs_vec[4] = {0.00832156,-0.0161815,-0.0111732,0.0234787};

DistortionCalibrator::DistortionCalibrator(int _w, int _h)
{
	
	height = _h;
	width = _w;
	
	image_size.height =_h;
	image_size.width = _w;

	cutImage_size.height = _h/2;
	cutImage_size.width = _w/2;

	originalImg = Mat(image_size,CV_8UC1);
	undistortedImg = Mat(image_size,CV_8UC1);
	
	cutImg = Mat(cutImage_size,CV_8UC1);
	equalizeHistImg = Mat(cutImage_size,CV_8UC1);
	borderImg = Mat(cutImage_size,CV_8UC1);
	
}

DistortionCalibrator::~DistortionCalibrator()
{
}


void DistortionCalibrator::initializeDistortionParameters()
{
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			intrinsic_matrix(i, j) = intrinsic_vec[i*3+j];
		}
	}

	for(int i=0; i<4; i++)
	{
		distortion_coeffs[i] = distortion_coeffs_vec[i];

		cout << distortion_coeffs[i] << endl;

	}

	intrinsic_mat = (cv::Mat_<float>(3,3) << 273.99497952772407f, 0.0f, 143.51565451116193f, 0.0f, 273.42858716863560f, 110.48756341112096f, 0.0f, 0.0f, 1.0f);
		
	intrinsic_mat.copyTo(new_intrinsic_mat);
	
	//调节视场大小,乘的系数越小视场越大
	new_intrinsic_mat.at<float>(0, 0) *= 0.7;
	new_intrinsic_mat.at<float>(1, 1) *= 0.7;
	//调节校正图中心，建议置于校正图中心
	new_intrinsic_mat.at<float>(0, 2) = 0.5 * width;
	new_intrinsic_mat.at<float>(1, 2) = 0.5 * height;

	
	return;
}


void DistortionCalibrator::eliminateDistortion(Mat src, Mat dst)
{
	fisheye::undistortImage(src, dst, intrinsic_matrix, distortion_coeffs, new_intrinsic_mat);
	return;
}



void DistortionCalibrator::CutImag_1_2( Mat src, Mat dst, int w,int h, int offset_x, int offset_y)
{
	int new_w = w/2;
	int new_h = h/2;

	int start_x = w/4+offset_x;
	int start_y = h/4+offset_y;

//	cvSetImageROI( (IplImage*)undistortedImg,cvRect(start_x, start_y, new_w, new_h) );
//	cvCopy((CvArr*)cutImg, (CvArr*)undistortedImg);
//	cvResetImageROI((IplImage*)undistortedImg);

	int i=0;

	for( i=0;i<new_h;i++ )
	{
		memcpy( (dst.data+i*new_w), (src.data+(start_y+i)*w+start_x), new_w );
		
	}
	
}

void DistortionCalibrator::equalizeHistFunc(Mat src, Mat dst)
{
	//blur( src, src, Size(3,3) );
	equalizeHist(  src, dst );
	
}

void DistortionCalibrator::borderDetection(Mat src, Mat dst)
{

#if 0
	Mat ax, ay;
	
    Sobel(src, ax, CV_8UC1, 1, 0,-1);       
    Sobel(src, ay, CV_8UC1, 0, 1,-1);

    addWeighted(ax, 0.5, ay, 0.5, 0, dst);
#else
	Canny(src, dst, 100, 300, 3);
#endif

}

