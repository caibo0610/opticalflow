/*==============================================================================
  Copyright (c) 2014-2015 ZeroTech
  Author: 郑艺�?
  Date: 2015.09.18
  Email: ZhengYiqiang@zerouav.com
  All rights reserved. ZeroTech Proprietary and Confidential.
==============================================================================*/

#include "inc/flight_control.h"
#include "inc/adspmsgd.h"
#include "inc/rpcmem.h"
#include "opticflow.h"
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>
#define SENSOR_QUEUE_SIZE 20
static float *TruePosDataPtr = NULL;
static int FlagDataInit = FALSE;
extern S32 exposure_value;

#define   OPTIFLOW    6

volatile int LoopFlag = 1;



void rpcmem_init()
{
}

void rpcmem_deinit()
{
}

void* rpcmem_alloc(int heapid, uint32 flags, int size)
{
   (void)heapid;
   (void)flags;

   return malloc(size);
}

void rpcmem_free(void* po)
{
   free(po);
}

int get_adsp_srf_distance(float *distance)
{
	flight_control_get_srf_distance(distance);

	return 0;
}

int deinit_imu_data()
{
	if (TruePosDataPtr) {
		rpcmem_free(TruePosDataPtr);
		TruePosDataPtr = NULL;
	}
  	rpcmem_deinit();

	FlagDataInit = FALSE;
  
	return 0;
}

int init_imu_data()
{
	int ret = 0;
	int i=0;

	if (FlagDataInit) {
		ERROR("Warning: data already inited\n");
		return ret;
	}

	rpcmem_init();

	TRACK("- allocate %d bytes from ION heap for imu sensor.\n", sizeof(float) * SENSOR_QUEUE_SIZE);
	if (0 == (TruePosDataPtr = (float*)rpcmem_alloc(0, RPCMEM_HEAP_DEFAULT, sizeof(float) * SENSOR_QUEUE_SIZE))) {
		ERROR("Error: alloc failed for imu data 1\n");
		ret = -1;
		goto bail;
	}
    
	for(i=0; i<SENSOR_QUEUE_SIZE; i++) {
		TruePosDataPtr[i] = 0.0;
	}

	if (0 != ret) {  									  
		ERROR("Error: init shared mem failed\n");
		ret = -1;
		goto bail;
	}

    flight_control_ap_set_version(OPTIFLOW,OPTIC_VERSION);

	FlagDataInit = TRUE;
	return ret;
bail:
	deinit_imu_data();
	return ret;
}


int get_position(float *pitch, float *yaw, float *radx, float *rady, uint64_t *radtime,float *radz)
{
#if 0

#else
	flight_control_optiflow_imu_newdata_t  optiflow_imu_data;
	flight_control_ap_get_optiflow_imu_newdata(&optiflow_imu_data);

    *pitch = optiflow_imu_data.pitch_angle;
    *yaw = optiflow_imu_data.yaw_angle;
    *radx = optiflow_imu_data.fix_gyro_x;
    *rady = optiflow_imu_data.fix_gyro_y;
    *radz = optiflow_imu_data.fix_gyro_z;
	
	*radtime = optiflow_imu_data.timestamp;


#endif
	return 0;
}

int get_running_flag()
{
    flight_control_optiflow_user_data_t optiflow_user_data;
	flight_control_ap_get_optiflow_user_data(&optiflow_user_data);


    int running_flag = optiflow_user_data.flag;

    if (running_flag > 0)
        return 1;
    else
        return 0;
}

int get_camera_type()
{
    uint8_t flag = 0;
    flight_control_ap_get_optiflow_camera_type(&flag);

    if (flag == OPTIC_CAMERA_TYPE_OV7251_57D)
        return 57;
    else if (flag == OPTIC_CAMERA_TYPE_OV7251_65D)
        return 65;
    else
    {
        ERROR("unknown camera type %d, and set it default FOV to 57\n",flag);
        return 57;
    }
}

int send_optiflow_data(int32_t flag,float vx,float vy, float height, float orgvx, float orgvy, int confidence)
{
#if 0
#else
    flight_control_optiflow_data_t optiflow_data;

    optiflow_data.vx = vx;
    optiflow_data.vy = vy;
    optiflow_data.height = height;
    optiflow_data.ori_vx = orgvx;
    optiflow_data.ori_vy = orgvy;
    //optiflow_data.exposure_time = exposure_value;
    optiflow_data.texture = confidence;
    optiflow_data.quality = flag;

    int ret = flight_control_ap_send_optiflow_result(&optiflow_data);
#endif
	return ret;
}


