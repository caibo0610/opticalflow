#ifndef _FLIGHT_CONTROL_H
#define _FLIGHT_CONTROL_H
/// @file flight_control.idl
///
#include "AEEStdDef.h"
#ifndef __QAIC_HEADER
#define __QAIC_HEADER(ff) ff
#endif //__QAIC_HEADER

#ifndef __QAIC_HEADER_EXPORT
#define __QAIC_HEADER_EXPORT
#endif // __QAIC_HEADER_EXPORT

#ifndef __QAIC_HEADER_ATTRIBUTE
#define __QAIC_HEADER_ATTRIBUTE
#endif // __QAIC_HEADER_ATTRIBUTE


//Data of Optical Flow:	
typedef struct flight_control_optiflow_user_data_t flight_control_optiflow_user_data_t;
struct flight_control_optiflow_user_data_t {
		uint8 flag;				//0-close,  1->open
		uint8 reseved[8];
};
typedef struct flight_control_optiflow_data_t flight_control_optiflow_data_t;
struct flight_control_optiflow_data_t {
		uint64 time_stamp;		
		float vx;				
		float vy;				
		float height;			
		float ori_vx;			
		float ori_vy;			
		uint32 exposure_time;	
        uint32 texture;       
		uint8 quality;		
};
typedef struct flight_control_optiflow_imu_newdata_t flight_control_optiflow_imu_newdata_t;
struct flight_control_optiflow_imu_newdata_t {
   uint32 frame_cnt;
   uint64 timestamp;
   float yaw_angle;
   float pitch_angle;
   float roll_angle;
   float fix_gyro_x;
   float fix_gyro_y;
   float fix_gyro_z;
   uint32 reserved;
};

#define OPTIC_CAMERA_TYPE_OV7251_57D    0
#define OPTIC_CAMERA_TYPE_OV7251_65D    1

#ifdef __cplusplus
extern "C" {
#endif

__QAIC_HEADER_EXPORT int __QAIC_HEADER(flight_control_get_srf_distance)(float* distance) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(flight_control_get_adsp_time)(float* timestamp) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(flight_control_ap_set_version)(uint8 type, uint8 version) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(flight_control_ap_get_optiflow_imu_newdata)(flight_control_optiflow_imu_newdata_t* optiflow_imu_data) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(flight_control_ap_get_optiflow_user_data)(flight_control_optiflow_user_data_t* optiflow_user_data) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(flight_control_ap_send_optiflow_result)(flight_control_optiflow_data_t* optiflow_data) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(flight_control_ap_get_optiflow_camera_type)(uint8* type) __QAIC_HEADER_ATTRIBUTE;

#ifdef __cplusplus
}
#endif
#endif //_FLIGHT_CONTROL_H
