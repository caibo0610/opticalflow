/********************************************************************
	created:	2017/5/25
	filename: 	flow_algo.h
	file ext:	h
	author:  	cyj
	purpose:  
*********************************************************************/

#ifndef FLOW_ALGO_H
#define FLOW_ALGO_H

#include "opencv2/opencv.hpp"
#include <iostream>
#include "type.h"
#include "lkp.h"

//#define OPENCV_FLOW

#define XFREE(x) ({\
	if(x != NULL) {\
		free(x);\
		x = NULL;\
	}\
})

void Initial_flow(ParamA* _PA, ParamB* _PB);
void img_rotation_coord_opti_comp(imgtype1 *img_int, imgtype1 *img_out, float rotationangle, float height1);
void XFreeSpace(ParamA* _PA,ParamB* _PB);
void compute_flow_opencv(cv::Mat pre_img, cv::Mat cur_img, ParamA* _PA, ParamB* _PB);
float GetRightPoint1(ParamB* _PB, std::vector<cv::Point2f> features, std::vector<cv::Point2f> features_after, std::vector<uchar> status);

#endif //FLOW_ALGO_H

