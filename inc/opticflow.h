/********************************************************************
	created:	2015/8/17
	filename: 	oav_camera.h
	file ext:	h
	author:  	guosc
	purpose:  
*********************************************************************/

#ifndef C_OPTIC_FLOW_H
#define C_OPTIC_FLOW_H

#include <stdio.h>
#include "type.h"

#define OV7251
#define FC_INTEGRATION

#define USB_PREVIEW_W 320
#define USB_PREVIEW_H 240

#define OPTIC_VERSION  21
#define CORRECT_COEF   1.0
#define REL_DATE       "20171103"

#define TRACK_PRINT
/*
#define XFREE(x) ({\
	if(x != NULL) {\
		free(x);\
		x = NULL;\
	}\
})
*/

typedef struct OF_params
{
    U16 of_status;
    U16 w;
    U16 h;
    U16 reserve;
    U32 bufsize;
    U8 *pre_image_buf;
    U8 *cur_image_buf;
}OF_params;

typedef enum
{
    OF_SUCCESS = 0,
    OF_CONFIG_FILE_NONEXIST,
    OF_MEM_ALLOC_FAILURE,
    OF_IMU_INTIAL_FAILURE
}ERROR_CODE;

typedef enum
{
    OF_INVALID = 0,
    OF_CONFIG_OK,
    OF_CONFIG_FAIL
}OF_STATUS;

#define flowBufferSize (24 + USB_PREVIEW_W*USB_PREVIEW_H)
struct Flow_buffer
{
	unsigned int nframe;
	int frame_interval;
	int height;
	int radvx;
	int radvy;
	int radvz;
	U8 data[USB_PREVIEW_W*USB_PREVIEW_H];
};

enum global_param_id_t
{
	PARAM_CAM_CENTER_OFFSETX = 0, //flow image center offset in x direction
	PARAM_CAM_CENTER_OFFSETY, //flow image center offset in y direction
	PARAM_CAM_FOCAL_LENGTH,   //flow camera's focal length
	PARAM_CAM_PIXEL_SIZE,     //flow sensor's pixel size
	PARAM_CAM_TO_CENTER_DISX, //x-direction distance between flow camera and flight center
	PARAM_CAM_TO_CENTER_DISY, //y-direction distance between flow camera and flight center
	PARAM_CAM_TO_CENTER_DISX_PIXEL, //x-direction distance(pixel) between flow camera and flight center
	PARAM_CAM_TO_CENTER_DISY_PIXEL, //y-direction distance(pixel) between flow camera and flight center

	PARAM_MAX_COUNT
};

enum global_ctrl_id_t
{
	CTRL_MODE = 0,   //flow ctrl mode, = 0 -- use pz ctrl, = 1 -- use zero tech ctrl
	CTRL_SAVE_LOG,   // = 0  -- not save rotate log, = 1 -- save rotate log to "media/internal/data/video/flow_rotate_log_#.txt"
	CTRL_SAVE_VIDEO, // = 0  -- not save video, = 1 -- save video to "media/internal/data/video/preImg_#.yuv, nextImg_#.yuv"

	CTRL_MAX_COUNT
};

struct global_struct
{
	int ctrl[CTRL_MAX_COUNT];
	float params[PARAM_MAX_COUNT];
};


#ifdef DEBUG
#define DBG(fmt, args...) printf("%s@%d " fmt ,__FILE__,__LINE__, ##args)
#else
#define DBG(fmt, args...)
#endif

#ifdef TRACK_PRINT
#define TRACK(fmt, args...) printf(fmt , ##args)
#else
#define TRACK(fmt, args...)
#endif

#ifdef FC_INTEGRATION
#define FCINTEG(fmt, args...) printf(fmt , ##args)
#else
#define FCINTEG(fmt, args...)
#endif


#ifdef VECTOR_VERIFY
#define VECTOR(fmt, args...) printf(fmt , ##args)
#else
#define VECTOR
#endif

#define ERROR(fmt, args...) printf("%s@%d>>>" fmt ,__FILE__,__LINE__, ##args)

#define CRASH(fmt, args...)

#endif //C_OPTIC_FLOW_H

