/********************************************************************
	created:	27:7:2015
	filename: 	ggf
	file ext:	h
	author:  	Fu Chongyang
	purpose:  
*********************************************************************/

#include "type.h"
#include <stdint.h>

#include "lkp.h"
#include "opticflow.h"

//#define epsilon  0.000001
#define RAD_RECORD_MAX  12
#define VELOCITY_FILTER 0.3f

#define FILTERIMG_COEFFICIENT 0.2f

//#define USE_NEON

#ifdef __cplusplus
extern "C" {
#endif
typedef struct Interface_Flightcontrol_Opticalflow
{
    int status;  //0: unvailable, 1:vailable, 2:heightisn't available
    float height;
    float vx;
    float vy;
}Interface_Flightcontrol_Opticalflow;

//void CutImgOffset(imgtype1* src, imgtype1* dst,int w,int h, int offset_x, int offset_y);
int savePicture(imgtype1* src, int w, int h, const char* fileName);

#ifdef __cplusplus
}
#endif



