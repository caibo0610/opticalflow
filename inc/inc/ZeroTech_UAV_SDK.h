/******************************************************************************
* Copyright (c) 2007-2016, ZeroTech Co., Ltd.
* All rights reserved.
*******************************************************************************
* File name     : ZeroTech_UAV_SDK.h
* Description   : CONFIDENTIAL. The SDK is only for internal testing. 
				  There will be some big changes in the last couple of days.
* Version       : v0.3
* Create Time   : 2015/11/18
* Last Modified : 2016/06/13
* Author   		: ֣��ǿ(zhengyiqiang@zerotech.com)
* Modify history:
*******************************************************************************
* Modify Time   Modify person  Modification
* ------------------------------------------------------------------------------
*
*******************************************************************************/
#ifndef _ZeroTech_UAV_SDK_H_
#define _ZeroTech_UAV_SDK_H_

#ifdef __cplusplus
extern "C" {
#endif

#define COORDINATE_GEODETIC			0	//����/�������ϵ(����, GPS��γ��)
#define COORDINATE_PROJECT_COMPASS	1	//ͶӰ����ϵ1-�����ϱ������������ϵ(ƽ��, (�����ϱ�)-->(x+,x-,y-,y+))
#define COORDINATE_PROJECT_CONTROL	2	//ͶӰ����ϵ2-ǰ�����ҷ����������ϵ(ƽ��, (ǰ������)-->(y+,y-,x-,x+))
#define COORDINATE_UAV_BODY			3	//��������ϵ(������ģ������ϵ���ڴ�����ϵ��ת)


//������٣�
#define SDK_MAX_VX 800		//�������/�������ٶ�: ��λ��cm/s
#define SDK_MAX_VY 800		//����ϱ���/ǰ�����ٶ�: ��λ��cm/s
#define SDK_MAX_VZ 800		//��������ٶȣ�		 ��λ��cm/s
#define SDK_MAX_VO 150		//�������ת�ٶ�:		 ��λ���Ƕ�/s

#define RETURN_SUCCESS	0
#define RETURN_FAIL 	(-1)

//------------------------------------------------------------------------------------------

typedef struct _sdk_takeoff_cmd
{
	uint32_t target_height;
	
}sdk_takeoff_cmd_t;

typedef struct _sdk_autoland_cmd
{
	float reserved[4];
	
}sdk_autoland_cmd_t;


//------------------------------------------------------------------------------------------

typedef struct _sdk_curr_location
{//��ǰλ��:
	int satellites;				//��ǰ����: [<6]-GPS����λ; [<20]-GPS��λ; [=98]-GPS����λ���й���; [=99]-GPS��λ����Ҳ�й���
	uint64_t gps_timestamp; 	//GPSʱ���
	float gps_longitude;		//GPS����
	float gps_latitude;			//GPSγ��
	float gps_height;			//GPS���θ߶ȣ���λ��m
	float air_height;			//��ѹ�߶�(�������ɵ�)����λ��m
	float flow_longitude;		//��������
	float flow_latitude;		//����γ��
	float ultrasonic_height;	//�������߶ȣ���λ��m
}sdk_curr_location_t;

typedef struct _sdk_curr_speed
{//��ǰ�ٶȣ�
	int16_t vx;			//����/�����ٶ�:	����/�һ�����Ϊ��, 	��λ��cm/s
	int16_t vy;			//����/ǰ���ٶ�:	����/��ͷ����Ϊ��, 		��λ��cm/s
	int16_t vz;			//�����ٶȣ�		��������Ϊ��,			��λ��cm/s
	uint8_t coordinate;	//�ο�����ϵ: 1-COORDINATE_PROJECT_COMPASS(�����ϱ�); 2-COORDINATE_PROJECT_CONTROL(ǰ������)
}sdk_curr_speed_t;

typedef struct _sdk_gesture_quarternion
{//��̬��Ԫ����
	float qw;
	float qx;
	float qy;
	float qz;
}sdk_gesture_quarternion_t;

typedef struct _sdk_fly_status
{//����״̬��
	uint64_t timestamp;
	int motor_status;	//�ɻ��Ƿ����(����Ƿ���ת):	0-δ����,�����ת; 1-�Ѿ�����,���ת����
	int fly_status;		//����״̬��1-���棬2-����У�3-�����У�4-����ģʽ��5-�������䣬6-��ͣ�У�7-�����У��Ǻ���ģʽ�ͷ���ģʽ����255-����쳣ģʽ
	int MotionStatus;   // 1-�˶�ģʽ��0-��׼ģʽ
	int satellites;		//��ǰ����:  [<6]-GPS����λ; [<20]-GPS��λ; [=98]-GPS����λ���й���; [=99]-GPS��λ����Ҳ�й���
	int longitude;		//����*1e7������10000000Ϊʵ�ʾ���
	int latitude;		//γ��*1e7������10000000Ϊʵ��γ��
	float height;		//�߶�: 	��λ��	m
	float pitch_angle;	//�����ǣ�	��λ�� �Ƕ�
	float roll_angle;	//����ǣ�	��λ�� �Ƕ�
	float yaw_angle;	//����ǣ�	��λ�� �Ƕ�
}sdk_fly_status_t;

typedef struct _sdk_ptz_position
{//��̨λ�ã�
	float pitch_angle;	//��̨������(����ڴ��):		��λ�� �Ƕ�
	float yaw_angle;	//��̨�����(����ڻ���) :		��λ�� �Ƕ�
}sdk_ptz_position_t;
	
typedef struct _sdk_target_speed
{//Ŀ���ٶ�:
	int16_t vx;			//����/�����ٶ�:	����/�һ�����Ϊ��, 	��λ��cm/s
	int16_t vy;			//����/ǰ���ٶ�:	����/��ͷ����Ϊ��, 		��λ��cm/s
	int16_t vz;			//�����ٶȣ�		��������Ϊ��,			��λ��cm/s
	int16_t vo;			//������ת�ٶ�:		˳ʱ�뷽��Ϊ��, 		��λ���Ƕ�/s
	uint8_t coordinate;	//�ο�����ϵ: 1-COORDINATE_PROJECT_COMPASS(�����ϱ�); 2-COORDINATE_PROJECT_CONTROL(ǰ������)
}sdk_target_speed_t;

typedef struct _sdk_target_acc
{//Ŀ����ٶȣ�
	int16_t accx_right;	//������ٶȣ�	�һ�����Ϊ��,		��λ��cm/s^2
	int16_t accy_head;	//ǰ����ٶȣ�	��ͷ����Ϊ��,		��λ��cm/s^2
	int16_t accz_up;	//������ٶȣ�	��������Ϊ��,	��λ��cm/s^2
}sdk_target_acc_t;

typedef struct _sdk_target_gesture
{//Ŀ����̬��
	float pitch_angle;	//Ŀ�긩����:	��λ�� �Ƕ�
	float roll_angle;	//Ŀ������:	��λ�� �Ƕ�
	float yaw_angle;	//Ŀ�꺽���:	��λ�� �Ƕ�
}sdk_target_gesture_t;

typedef struct _sdk_target_gesture_speed
{//Ŀ����̬�������ٶȣ�
	float v_pitch;		//Ŀ�긩�����ٶ�: ��λ�� �Ƕ�/s
	float v_roll;		//Ŀ�������ٶ�: ��λ�� �Ƕ�/s
	float v_yaw;		//Ŀ�꺽����ٶ�: ��λ�� �Ƕ�/s
}sdk_target_gesture_speed_t;

typedef struct _sdk_ptz_speed
{//Ŀ����̨�ٶȣ�
	float v_pitch;		//��̨������(����ڵ�������ϵ):	��λ�� �Ƕ�/s
	float v_yaw;		//��̨�����(����ڻ���):		��λ�� �Ƕ�/s
}sdk_ptz_speed_t;	
	
//------------------------------------------------------------------------------------------
/*
Ŀ����ٽӿڲ���˵����
	flag: 0-��, 1-��, 2-���³�ʼ��
	mode: 0-����ģʽ, 1-����ģʽ
	x0: ���Ͻ�X����
	y0: ���Ͻ�Y����
	x1: ���½�X����
	y1: ���½�Y����
	quality: ���ٽ������(���Ŷ�), 0~255, 0-������(����), ��ֵԽ����Ŷ�Խ��
*/
typedef struct _sdk_target_tracking_init_data
{
   //��һ��������
   float x0;
   float y0;
   float x1;
   float y1;
   uint8_t flag;	//0-��, 1-��, 2-���³�ʼ��
   uint8_t mode;	//0-����ģʽ, 1-����ģʽ
}sdk_target_tracking_init_data_t;

typedef struct _sdk_target_tracking_result_data
{	
   //��һ��������
   float x0;
   float y0;
   float x1;
   float y1;
   uint8_t quality;	//���ٽ������(���Ŷ�), 0~255, 0-������(����), ��ֵԽ����Ŷ�Խ��
}sdk_target_tracking_result_data_t;

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//about Pudrone Middleware david
typedef struct _sdk_disable_rc_cmd
{//we can set the para with 1, to enable the rc channel,and set para with 0, to disable the rc channel
	uint8_t xplus;
	uint8_t xminus;
	
	uint8_t yplus;
	uint8_t yminus;
	
	uint8_t zplus;
	uint8_t zminus;
	
	uint8_t yawplus;
	uint8_t yawminus;
} sdk_disable_rc_cmd_t;

typedef struct _sdk_set_max_vel 
{//
	float xplus_maxvel;
	float xminus_maxvel;

	float yplus_maxvel;
	float yminus_maxvel;
	
	float zplus_maxvel;
	float zminus_maxvel;
	
	float yawplus_maxvel;
	float yawminus_maxvel;
} sdk_set_max_vel_t;
//david add 2017.7.31 for pudrone middleware
extern int disable_rc_Cmd_from_ZeroTech_UAV_SDK(sdk_disable_rc_cmd_t * disable_rc_cmd);
extern int set_max_vel_from_ZeroTech_UAV_SDK(sdk_set_max_vel_t * sdk_set_max_vel);
//------------------------------------------------------------------------------------------
extern int init_ZeroTech_UAV_SDK(void);
extern int exit_ZeroTech_UAV_SDK(void);

extern int enable_action_control_cmd_to_ZeroTech_UAV_SDK(void);
extern int disable_action_control_cmd_to_ZeroTech_UAV_SDK(void);

extern int unlock_cmd_to_ZeroTech_UAV_SDK(void);
extern int takeoff_cmd_to_ZeroTech_UAV_SDK(sdk_takeoff_cmd_t *takeoff_cmd);
extern int autoland_cmd_to_ZeroTech_UAV_SDK(sdk_autoland_cmd_t *autoland_cmd);
extern int emergency_stop_cmd_to_ZeroTech_UAV_SDK(void);
extern int lock_cmd_to_ZeroTech_UAV_SDK(void);
extern int go_home_cmd_to_ZeroTech_UAV_SDK(void);

extern int get_fly_status_from_ZeroTech_UAV_SDK(sdk_fly_status_t *fly_status);
extern int get_curr_location_from_ZeroTech_UAV_SDK(sdk_curr_location_t *curr_location);
extern int get_curr_speed_from_ZeroTech_UAV_SDK(sdk_curr_speed_t *curr_speed);
extern int set_target_speed_to_ZeroTech_UAV_SDK(sdk_target_speed_t *target_speed);
extern int get_target_speed_from_ZeroTech_UAV_SDK(sdk_target_speed_t *target_speed);
extern int set_target_acc_to_ZeroTech_UAV_SDK(sdk_target_acc_t *target_acc);
extern int get_target_acc_from_ZeroTech_UAV_SDK(sdk_target_acc_t *target_acc);
extern int get_gesture_quarternion_from_ZeroTech_UAV_SDK(sdk_gesture_quarternion_t *gesture_quarternion);
extern int set_target_gesture_to_ZeroTech_UAV_SDK(sdk_target_gesture_t *target_gesture);
extern int get_target_gesture_from_ZeroTech_UAV_SDK(sdk_target_gesture_t *target_gesture);
extern int set_target_gesture_speed_to_ZeroTech_UAV_SDK(sdk_target_gesture_speed_t *target_gesture_speed);
extern int get_target_gesture_speed_from_ZeroTech_UAV_SDK(sdk_target_gesture_speed_t *target_gesture_speed);	
extern int set_ptz_position_to_ZeroTech_UAV_SDK(sdk_ptz_position_t *ptz_position);
extern int get_ptz_position_from_ZeroTech_UAV_SDK(sdk_ptz_position_t *ptz_position);
extern int set_ptz_speed_to_ZeroTech_UAV_SDK(sdk_ptz_speed_t *ptz_speed);
extern int get_ptz_speed_from_ZeroTech_UAV_SDK(sdk_ptz_speed_t *ptz_speed);	

extern int get_target_tracking_init_data_from_ZeroTech_UAV_SDK(sdk_target_tracking_init_data_t *init_data);
extern int send_target_tracking_result_data_to_ZeroTech_UAV_SDK(sdk_target_tracking_result_data_t *target_data);
extern int get_target_tracking_result_data_from_ZeroTech_UAV_SDK(sdk_target_tracking_result_data_t *target_data);

extern void coordinate_spin_func_in_ZeroTech_UAV_SDK(float x, float y, float z, float ax, float ay, float az, float *xt, float *yt, float *zt);
extern int spin_yaw_angle_cmd_to_ZeroTech_UAV_SDK(int16_t spin_angle, int16_t vo_yaw);//����ʽ����(��ͷ)��ת�Ƕ�����ת�ٶȿ���, �з���, ˳ʱ�뷽��Ϊ��.


#ifdef __cplusplus
}
#endif


#endif //_ZeroTech_UAV_SDK_H_
