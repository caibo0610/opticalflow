/********************************************************************
  created:  2015/07/27
  filename:   ggf
  file ext:  c
  author:  Fu Chongyang
  purpose:  使用光流算法跟踪图像移动
*********************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <math.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <assert.h>
#include <unistd.h>
#include "uav_of.h"
#include <sys/socket.h>  
#include <netinet/in.h>  
#include <arpa/inet.h>  
#include <unistd.h>  
#include <string.h>  
#include <arm_neon.h> 
#define PORT 1111  
#include <fstream>
#include "inc/flight_control.h"
#include "inc/ZeroTech_UAV_SDK.h"
#include <time.h>
#include <iostream>
#include <iomanip>
#include "flow_algo.h"

//#include "pz_ae.h"

std::fstream g_optic_all_log;

//#define SAVEPICTURE
#ifdef SAVEPICTURE
const char* originalPic = "originalImg.bmp";

char originalName[50];
char borderName[50];
uint32_t picCounter = 0;
#endif

FILE *file_prev, *file_next;//
char fname_prev[60], fname_next[60];//

uint8_t dataLogSave = 0;
char datalog[100];

struct Flow_buffer *flow_buffer = NULL;
struct global_struct g_data = {0};

#define CV_DESCALE(x,n) (((x) + (1 << ((n)-1))) >> (n))
//#define FLT_EPSILON 1.192092896e-07F
#define max(a,b) (((a) > (b)) ? (a) : (b))
#define min(a,b) (((a) < (b)) ? (a) : (b))
#define N_LVBO_flow 3
//#define IMG_ROTATION_TEST 
OF_params optical_flow_params;
ParamA PA;
ParamB PB;
U8 *ping_buffer = NULL;
U8 *pang_buffer = NULL;

U32 pingpangflag = 0;
U32 frame_counter = 0;
//extern pthread_mutex_t mutex;
//extern Ae_t ae_ctrl;
//extern Chromatix_ae_type ae_params;
extern S32 frame_interval;
extern S32 algo_frame_interval;
extern S32 exposure_value;
extern S32 exposure_max;
extern S32 exposure_set_flag; 
extern S32 gain_value;

extern U32 latest_index;
extern uint64_t timestamp[2];

static void initGloabData();
static void fillFlowBuffer(U8 *imageData, ParamB* _PB);
static void parseCalibFile();
static int  OpticalFlow_Confidence_Filter(int _input);
static void FTrack(U8 *pre, U8 *cur,ParamA* PA,ParamB* PB);
extern int get_adsp_srf_distance(float *distance);
extern int send_optiflow_data(int32_t flag,float vx,float vy, float height, float orgvx, float orgvy, int confidence);
extern int init_imu_data();
extern int deinit_imu_data();
extern int get_position(float *pitch, float *yaw, float *radx, float *rady, uint64_t *timeStamp,float *radz);
extern int get_camera_type();

U32 rad_frame_index = 4;//choosed IMU rad velocity index

/*******************************************************************************/
// Method:  initOpticalFlow
// Date:    2015/8/5
// Returns:   S32
// Parameter: OF_params *params
// Function:  Optical Flow main proc, it is for thread entry function
/******************************************************************************/
S32 initialzeOpticalFlow()
{
    OF_params *params = &optical_flow_params;

    if(init_imu_data() != 0)
	{
		ERROR("init_imu_data() failed!\n");
		return OF_IMU_INTIAL_FAILURE;
	}
    PA.maxCorners = 32;
    PA.minCorners = 32;
    PA.minDist = 20;
    PA.maxlevel = 2;
    PA.w = USB_PREVIEW_W;
    PA.h = USB_PREVIEW_H;
    PA.maxcount = 30;
    PA.winsize = 21;
    //PA.ip = "192.168.1.19";
    sprintf(PA.ip, "%s","192.168.1.142");

	int field_angle = 153;//
	//float focal_len = 0.83;//1.657;//1.64

    /* 建立socket*/
    if((PA.sock = socket(AF_INET,SOCK_DGRAM,0))<0)
    {  
        perror("socket"); 
        ERROR("socket open error\n");
        return OF_CONFIG_FILE_NONEXIST;
        exit(1);  
    }  

    DBG("server ip: %s\n",PA.ip);

    PA.th=(double)GGP_THRESHOLD;

    if (PA.w != USB_PREVIEW_W || PA.h != USB_PREVIEW_H)
    {
        ERROR("can't support pixels configuration, PA.w=%d,PA.h=%d",PA.w,PA.h);
        return -1;
    }

    params->w = (U16)PA.w;
    params->h = (U16)PA.h;
    params->bufsize = PA.w * PA.h;

    //initialize pingpang buffer
    ping_buffer = (U8 *)malloc(params->bufsize);
    pang_buffer = (U8 *)malloc(params->bufsize);

#ifdef OPENCV_FLOW
	Initial_flow(&PA,&PB);
#else
    //initialize PB
	PB.m_bsetparam=0;
    PB.nframe=1;
    PB.flagPyrmd=1;
    PB.spaceFlag=1;
    Initial(&PA,&PB);
#endif

	initGloabData();
	parseCalibFile();

	update_camera_fov(field_angle,g_data.params[PARAM_CAM_FOCAL_LENGTH],1,&PB);
    params->of_status = OF_CONFIG_OK;

	//init_ZeroTech_UAV_SDK();
	//enable_action_control_cmd_to_ZeroTech_UAV_SDK();

	if(flow_buffer == NULL)
		flow_buffer = (struct Flow_buffer *)malloc(sizeof(struct Flow_buffer));

	flow_buffer->nframe = 0;
	flow_buffer->frame_interval = 0;
	flow_buffer->height = 0;
	flow_buffer->radvx = 0;
	flow_buffer->radvy = 0;
	flow_buffer->radvz = 0;
	memset(flow_buffer->data, 0, USB_PREVIEW_W*USB_PREVIEW_H);

    return OF_SUCCESS;

}


void releaseOpticalFlow()
{
	XFREE(ping_buffer);
	XFREE(pang_buffer);
#ifdef OPENCV_FLOW
    XFreeSpace(&PA,&PB);
#else
	FreeSpace(&PA,&PB);
#endif
	XFREE(flow_buffer);
    deinit_imu_data();
}

#if 1
/*purpose:select the minimum dT between timestamps and camerats*/
U32 select_matched_data(void)
{	
	int i = 0; 
	U32 match_num = 0;
	int64_t com_dt = 0;
	uint64_t min_com_dt = 0;
	uint64_t abs_com_dt = 0;
	
	for(i=0;i<8;i++){
		com_dt = PB.fcpara[i].timestamps - PB.fcpara[0].camerats;
		
		abs_com_dt = fabs(com_dt);
		//printf("data_dt = %llu\n",abs_com_dt);
		if(i == 0){
			min_com_dt = abs_com_dt;
		}
		if(abs_com_dt < min_com_dt){
			min_com_dt = abs_com_dt;
			match_num = i;
		}
	}
	return match_num;
}
#endif

/**/
void velocity_comp(float *comp_x,float *comp_y,float *dz)
{
	//基于旋转中心进行补偿
	float compx = *comp_x;
	float compy = *comp_y;
	float radvz = *dz;
	float ori_camy = *comp_y;
	float fuselage_avertence_angle = 34.79f*0.0174f;
	float fDisCam2IMU = 0.0213 * cos(fuselage_avertence_angle);
	float fVelocityCam = 0;
	float fR_Cam2O = 0;
	float fSita = 0;
	float fR_IMU2O = 0;

	if((compx != 0)&&(radvz != 0))
	{
		fVelocityCam = sqrt(compx * compx + compy * compy);
		fR_Cam2O = fabs(fVelocityCam / radvz);
		fSita = atan(fabs(compy / compx));
		
		if ((radvz > 0 && compy < 0 && compx < 0) || (radvz > 0 && compy > 0 && compx < 0)
			|| (radvz < 0 && compy > 0 && compx > 0) || (radvz < 0 && compy < 0 && compx > 0)){
			fSita = 3.1415926f - fSita;
		}

		fR_IMU2O = sqrt(fDisCam2IMU * fDisCam2IMU + fR_Cam2O * fR_Cam2O - 2 * fDisCam2IMU * fR_Cam2O * cos(fSita));
		if (fR_IMU2O < 0.01 || fR_Cam2O < 0.01 || fabs(compx) / (fabs(compy) + 0.00001f) >= 10)
		{
			compx -= fDisCam2IMU * radvz;
		}
		
		else
		{
			float fVelocityIMU = fabs(fR_IMU2O * radvz);
			compx = (fR_IMU2O * fR_IMU2O + fDisCam2IMU * fDisCam2IMU - fR_Cam2O * fR_Cam2O) / (2 * fR_IMU2O * fDisCam2IMU) * fVelocityIMU;
			compy = fR_Cam2O * sin(fSita) / fR_IMU2O * fVelocityIMU;
		
			if (radvz > 0)
			{
				*comp_x = -compx;
			}
			else
			{
				*comp_x = compx;
			}
			if(ori_camy < 0)
			{
				*comp_y = -compy;
			}
			else
			{
				*comp_y = compy;
			}

		}
	}
	else
	{
		compx -= fDisCam2IMU * radvz;
	}
}

/*

*/
void sendResult(float pixelx, float pixely, unsigned long tvdiff,unsigned long tvdiff2)
{
    int ret = 0;
    float height;

    int printf_flag = 0;
	
    //float dx,dy;
    float vx = 0.0,vy = 0.0;
    float radvx,radvy,radvz;
    float compx = 0.0,compy = 0.0;
	float vx_pitch_comp = 0.0, vy_roll_comp = 0.0;
    float tmp,tmp1;
	
    int32_t flag = 1;
    int32_t filter_flag = 0;
    const float radfactor = (3.141593)/180;
	const float COE = 0.017453;
    float dis_per_pixel_per_height = PB.dis_per_pixel_per_height;
    U32 current_rad_index;

    current_rad_index = rad_frame_index;

    height = PB.height1;

	float dis_th = 0.5;
	float variance_th = 0.6;
	float fluct_th = 0.7;
	
    if ((height < 0.4 )||(height > 100.0))
    {
        height = 0.5;
        PB.height1 = height;
    }

    if (PB.xgd > 0)
    {
    	flag = (int)round(PB.xgd * 255);
	}
    else
        flag = 0;

    //if we get the IMU result from starting of algo
    tmp = height * radfactor;
    radvx = tmp * PB.fcpara[current_rad_index].radvx;
    radvy = tmp * PB.fcpara[current_rad_index].radvy;
	radvz =  COE * PB.fcpara[current_rad_index].radvz;

    if ((pixelx<60)&&(pixelx>-60)&&(pixely<60)&&(pixely>-60)&&(frame_interval>0)&&(frame_interval<1000*1000))
    {
	   flag=flag;
    }
    else
    {
	   flag=0;
    }
 
	if ((flag != 0) && (tvdiff > 0))
	{
        tmp1 = (dis_per_pixel_per_height * height) /(frame_interval/1000.0);
        vx = tmp1 * pixelx;   //unit m/s
        vy = tmp1 * pixely;   //unit: m/s

	#if 1
        compx = -(vx - radvy);
        compy = vy - radvx;	
		vx_pitch_comp = compx;
		vy_roll_comp = compy;
	
	#endif

		if(PB.rotflag)
		{
		//基于旋转中心进行速度补偿
			if(g_data.ctrl[CTRL_MODE] == 1)
       			velocity_comp(&compx,&compy,&radvz);
		}

    	if (update_confidence_level(compx,compy,height,dis_th,variance_th,fluct_th,&PB))
    	{
        	flag = 0;
    	}

    	printf_flag = flag;

    	filter_flag = OpticalFlow_Confidence_Filter(flag);

    	flag = filter_flag;

        PB.v_comp.x = compx;
        PB.v_comp.y = compy;

        ret = send_optiflow_data(flag,compx,compy,height,-vy,vx,PB.maxeig);

        PB.v_opti_pre.x = vx;
        PB.v_opti_pre.y = vy;

        TRACK("[vx vy] = [%10.6f|%10.6f] [radvx radvy] =[%10.6f|%10.6f] [x y] = [%10.6f|%10.6f]\n",vx,vy,radvx,radvy,compx,compy);
	}
    else
    {
		compx=0;
		compy=0;
        vx = 0;   //unit m/s
        vy = 0;   //unit: m/s
        flag = 0;

        printf_flag = flag;
        filter_flag = OpticalFlow_Confidence_Filter(flag);
        flag = filter_flag;

        ret = send_optiflow_data(flag,vx,vy,height,radvx,radvy,PB.maxeig);
        if (ret)
        {
           //ERROR("send data to flight control error, return value: %d\n",ret);
        }
        
        TRACK("[vx vy] = [%10.6f|%10.6f]\n",vx,vy);

     }
    //record result for debugging
    PB.h_record = height;
    PB.v_record.x = vx;
    PB.v_record.y = vy;
    PB.rad_record.x = radvx;
    PB.rad_record.y = radvy;
    PB.com_record.x = compx;
    PB.com_record.y = compy;
    PB.timediff = tvdiff;
    PB.v_gyro_pre.x = radvx;
    PB.v_gyro_pre.y = radvy;

	if(1 == g_data.ctrl[CTRL_SAVE_LOG]) {
		//add all log information into one log
		static unsigned int count_log_all= 0;
		static int fileIndex_txt = 0;
		static int fileIsExis_txt = 0;
		//sdk_curr_speed_t curr_speed = {0};
		//float vx_flight = 0.0, vy_flight = 0.0, vz_flight = 0.0;

		//curr_speed.coordinate = COORDINATE_PROJECT_CONTROL;
		//get_curr_speed_from_ZeroTech_UAV_SDK(&curr_speed);
		//vx_flight = curr_speed.vx / 100.0;
		//vy_flight = curr_speed.vy / 100.0;
		//vz_flight = curr_speed.vz / 100.0;

		if (0 == count_log_all) {
			if(0 == dataLogSave)
			{		
				do
				{
					fileIndex_txt ++;
					sprintf(datalog, "/media/internal/data/video/flow_rotate_log_%d.txt", fileIndex_txt);
			
					fileIsExis_txt = access(datalog, 0 );
				}
				while(-1 != fileIsExis_txt);

				g_optic_all_log.open(datalog,std::ios::out);
				dataLogSave = 1;
			}

			if (g_optic_all_log.is_open()) {
				g_optic_all_log << "use_zero_tech" << "\t" << "nframe" << "\t" << "height" << "\t" 
							<< "frame_interval" << "\t" << "rotation_angle" << "\t"
				            << "pixel_x" << "\t" << "pixel_y" << "\t" << "vx" << "\t" << "vy" << "\t" 
				            <<"radvx" << "\t" << "radvy" <<"\t" << "radvz" << "\t"
				            << "vx_pitch_comp" << "\t" << "vy_roll_comp" << "\t" << "compx" << "\t" << "compy" << "\t"
				            //<< "credibility_flag" << "\t" << "flow_vx_filtered" << "\t" << "flow_vy_filtered" << "\t"
				            //<< "vx_flight" << "\t" << "vy_flight" << "\t" << "vz_flight" << "\t"
							<< "dis_per_pixel_per_height" << "\t"
							<< "feature_point" << "\t" << "xgd" << "\t" << "PB.maxeig" << "\t"
							<< "flag" << "\t" << "filter_flag" << "\t"
							<< std::endl;
		
				std::cout << "Open the log file successfully!" << std::endl 
					  << "Please waiting for the end of the algorithm..." << std::endl;

			} else		
				std::cout << "Error opening file" << std::endl; 	
		}
		if(999999 > count_log_all) {
			if(g_optic_all_log.is_open()) {
				g_optic_all_log << g_data.ctrl[CTRL_MODE] << "\t" << PB.nframe << "\t" << height << "\t"
								<< frame_interval << "\t" << PB.rotation_angle << "\t"
								<< pixelx << "\t" << pixely << "\t" << vx << "\t" << vy << "\t"
								<< radvx << "\t"  << radvy << "\t"  << radvz <<"\t"
								<< vx_pitch_comp << "\t" << vy_roll_comp << "\t" << compx << "\t" << compy << "\t"
								//<< PB.credibility_flag << "\t" << flow_vx_filtered << "\t" << flow_vy_filtered << "\t"
								//<< vx_flight << "\t" << vy_flight << "\t" << vz_flight << "\t"
								<< dis_per_pixel_per_height << "\t"
								<< *(PB.nump) << "\t" << PB.xgd << "\t" << PB.maxeig << "\t"
								<< printf_flag << "\t" << filter_flag << "\t"
								<< std::endl;      
			} else {
				std::cout << "Error opening file" << std::endl; 	
				return;
			}
		} else {
			g_optic_all_log.close();
			std::cout << "closed file!" << std::endl;
		}
       count_log_all++;
	}
}

/*


*/
void FTrack(U8 *pre, U8 *cur,ParamA* PA,ParamB* PB)
{
    PB->nframe++;
    //S32 i;
	//struct timeval t1, t2, t7, t8;

	if(flow_buffer != NULL)
		fillFlowBuffer(pre, PB);
    //if (PB->m_bspeed)
    //{
        PA->w = USB_PREVIEW_W / 2;
        PA->h = USB_PREVIEW_H / 2;
        PA->winsize=11;
        PA->minDist = 10;
		cv::Mat pre_img(cv::Size(PA->w, PA->h), CV_8U);
		cv::Mat cur_img(cv::Size(PA->w, PA->h), CV_8U);

        PB->prevImg = pre_img.data;//PB->tailorprevImg;
        PB->nextImg = cur_img.data;//PB->tailornextImg;

        PB->rotation_angle = PB->fcpara[rad_frame_index].radvz * (frame_interval/1000000.0);
        static S32 rotation_plus = 0;
        static S32 rotation_minus = 0;

        if (PB->rotation_angle > 0.3)//0.1
        {
            rotation_plus++;
            rotation_minus = 0;
        }
        else if (PB->rotation_angle < -0.3)//0.1
        {
            rotation_minus++;
            rotation_plus = 0;
        }
        else
        {
            rotation_minus = 0;
            rotation_plus = 0;
        }

        if (rotation_plus >= 1 || rotation_minus >= 1)
        {
        	const float COE = 0.017453; // = 2pi/360 = 0.017453
        	//gettimeofday(&t1,NULL);
			img_rotation_coord_opti_comp(pre, (imgtype1 *)PA->mineig, COE * PB->rotation_angle,PB->height1);
			//gettimeofday(&t2,NULL);
			//ERROR("chenyijun:img_rotation_coord_opti_comp cost time = %ld us\n", 1000000 * (t2.tv_sec - t1.tv_sec) + (t2.tv_usec - t1.tv_usec));

            CutImg((imgtype1 *)PA->mineig,PB->prevImg,USB_PREVIEW_W,USB_PREVIEW_H);
            PB->rotflag = 1;//for log
        } else {
            PB->rotflag = 0;//for log
            CutImg((imgtype1 *)pre,PB->prevImg,USB_PREVIEW_W,USB_PREVIEW_H);
			
        }
		//CutImg((imgtype1 *)pre,PB->prevImg,320,240);
		CutImg(cur,PB->nextImg,USB_PREVIEW_W,USB_PREVIEW_H);
    //}

#ifdef SAVEPICTURE
	picCounter ++;

	if( (picCounter > 1) && (picCounter < 3000) )
	{
		sprintf(originalName, "/media/internal/data/video/originalImg_%d.bmp", picCounter);

		cv::imwrite(originalName, optDistortionCalibrator.originalImg);
	}
#endif

#if 1
	if (((PB->nframe & 0x3) == 0x3) && (PB->height1 > 0.4))
    {
		//ERROR("nframe = %d: ", PB->nframe);
		baoguang(&exposure_value,&exposure_set_flag,exposure_max,PA,PB);
    }
#endif

#ifdef OPENCV_FLOW
	compute_flow_opencv(pre_img, cur_img, PA, PB);
#else
	//struct timeval t3, t4, t5, t6;
	//gettimeofday(&t3, NULL);
	GetGoodfeature(PA, PB); //find feature point
	//gettimeofday(&t4, NULL);
	//ERROR("chenyijun:GetGoodfeature cost time = %ld us, point num = %d\n", 
		//1000000 * (t4.tv_sec - t3.tv_sec) + (t4.tv_usec - t3.tv_usec), *(PB->nump));

    for (int i = 0; i < (*(PB->nump)); i++)
    {
        PB->prevPts[2*i]=PB->fpts[2*i];
        PB->prevPts[2*i+1]=PB->fpts[2*i+1];
        PB->nextPts[2*i]=PB->prevPts[2*i];
        PB->nextPts[2*i+1]=PB->prevPts[2*i+1];
        PB->status[i]=1;
    }

	//LK pyramid algo
	//gettimeofday(&t5,NULL);
    UseLKP(PA,PB);
	//gettimeofday(&t6,NULL);

	//ERROR("chenyijun:UseLKP cost time = %ld us\n", 
		//1000000 * (t6.tv_sec - t5.tv_sec) + (t6.tv_usec - t5.tv_usec));
#endif

	//filter feature point
	//gettimeofday(&t7,NULL);
	GetRightPoint3(PB);
	//gettimeofday(&t8,NULL);
	//ERROR("chenyijun:nframe = %d, GetRightPoint3 cost time = %ld us, after filter, point num = %d\n", PB->nframe,
		//1000000 * (t8.tv_sec - t7.tv_sec) + (t8.tv_usec - t7.tv_usec), *(PB->nump));

    //get the confidence level
    getzxd(PA,PB);

	if((1 == g_data.ctrl[CTRL_SAVE_VIDEO])
		&& (PB->nframe < 60000)){
//************************ print feature point onto image **************************
/*		cv::Mat preImg(cv::Size(USB_PREVIEW_W/2, USB_PREVIEW_H/2), CV_8U, PB->prevImg);
		cv::Mat curImg(cv::Size(USB_PREVIEW_W/2, USB_PREVIEW_H/2), CV_8U, PB->nextImg);
		cv::Point2f detect_point;
		cv::Point2f match_point;

		for (int i = 0; i < (*(PB->nump)); i++) {
			detect_point.x = PB->prevPts[2 * i];
			detect_point.y = PB->prevPts[2 * i + 1];
			match_point.x = PB->nextPts[2 * i];
			match_point.y = PB->nextPts[2 * i + 1];

			if (PB->status[i]) {
				cv::circle(preImg, detect_point, 2, cv::Scalar(0, 0, 255));
				cv::circle(curImg, match_point, 2, cv::Scalar(0, 0, 255));
			} else {
				cv::circle(preImg, detect_point, 2, cv::Scalar(255, 255, 255));
				cv::circle(curImg, match_point, 2, cv::Scalar(255, 255, 255));
			}
		}*/
//*********************** print feature point onto image ***************************/
		static uint8_t frameSaveFlag = 0;
		static int fileIndex = 0;
		static int fileIsExis = 0;

		if(0==frameSaveFlag) {
			do {
				fileIndex ++;
			
				sprintf(fname_prev, "/media/internal/data/video/prevImg_%d.yuv", fileIndex);
				sprintf(fname_next, "/media/internal/data/video/nextImg_%d.yuv", fileIndex);
			
				fileIsExis = access( fname_prev, 0 );
				ERROR("-------------save video:%s  %s-------------\n", fname_prev, fname_next);
			}
			while(-1 != fileIsExis);

			file_prev = fopen(fname_prev, "wb");
			file_next = fopen(fname_next, "wb");
			if(file_prev < 0)// || file_next < 0)
			{
				ERROR("open file failed!\n");
			}
			frameSaveFlag = 1;
		}
		if(access( fname_prev, 0 ) == 0) {
			fwrite(PB->prevImg, sizeof(imgtype1)*USB_PREVIEW_W*USB_PREVIEW_H/4, 1, file_prev);
			fflush(file_prev);
		}
		if(access( fname_next, 0 ) == 0){
			fwrite(PB->nextImg, sizeof(imgtype1)*USB_PREVIEW_W*USB_PREVIEW_H/4, 1, file_next);
			fflush(file_next);
		}
	}

}

Point2f_z IMUradxy[RAD_RECORD_MAX];
float radtimestap[RAD_RECORD_MAX];
float frame_height;
para_from_fc fcrealtimepara[RAD_RECORD_MAX];

/*************Z.B.C. 20160628 TEST**********************/
int64 fir_dt = 0;
U8 fir_flag = 0;
uint64 cam_err_dt = 0;
uint64 lin_sys_tim_sum = 0;
uint64 IMU_sys_tim_sum = 0;

void get_rad_velocity(float *r_vx, float *r_vy, float *r_vz, uint64_t *rad_ts, uint64_t *cam_ts)
{
    float yaw_x,yaw_y,radx,rady,radz;
    uint64_t radtime;
	
	struct timespec cur;
	float IMU_sys_tim = 0;
    get_position(&yaw_x,&yaw_y, &radx, &rady,&radtime,&radz);

    int i;
    for (i = RAD_RECORD_MAX - 1; i > 0; i--)
    {
        fcrealtimepara[i] = fcrealtimepara[i-1];
    }
    fcrealtimepara[0].radvx = radx;
    fcrealtimepara[0].radvy = rady;
    fcrealtimepara[0].radvz = radz;
    fcrealtimepara[0].timestamps = radtime;
    fcrealtimepara[0].camerats = timestamp[latest_index];
	
	if(r_vx)
	{
		*r_vx = radx;
		*r_vy = rady;
		*r_vz = radz;
		*rad_ts = radtime;
		*cam_ts = timestamp[latest_index];
	}

#if 1	
	if(fir_flag == 0)
	{
		clock_gettime(CLOCK_MONOTONIC, &cur);
		lin_sys_tim_sum = (uint64)cur.tv_sec * 1000000 + cur.tv_nsec/1000;//linux sys clk,us
		flight_control_get_adsp_time(&IMU_sys_tim);
		IMU_sys_tim_sum = (uint64)(IMU_sys_tim * 1000000.0);//us
		fir_dt = lin_sys_tim_sum - IMU_sys_tim_sum;//IMU_sys_tim;
		fir_flag++;
	}
	
	/*value below are imu timestamp and camera timestamp */
	fcrealtimepara[0].timestamps = radtime * 1000+ fir_dt*1000;
#endif

}

void get_height()
{
    get_adsp_srf_distance(&frame_height);
}

void get_optiflow_next_buf(U8 **cur, U8 **pre, float *curr_radvx, float *curr_radvy,float *curr_radvz)
{
    OF_params *params = &optical_flow_params;
    if (pingpangflag == 0)
    {
        params->cur_image_buf = ping_buffer;
        params->pre_image_buf = pang_buffer;
    }
    else
    {
        params->cur_image_buf = pang_buffer;
        params->pre_image_buf = ping_buffer;
    }
    pingpangflag = (pingpangflag + 1) % 2;

    memcpy(&PB.fcpara[0], &fcrealtimepara[0],sizeof(para_from_fc) * RAD_RECORD_MAX);

	if(curr_radvx)
	{
		*curr_radvx = fcrealtimepara[0].radvx;
	}
	if(curr_radvy)
	{
		*curr_radvy = fcrealtimepara[0].radvy;
	}
	if(curr_radvz)
	{
		*curr_radvz = fcrealtimepara[0].radvz;
	}
    PB.height1 = frame_height;

    *cur = params->cur_image_buf;
    *pre = params->pre_image_buf;
}

/*******************************************************************************/
// Method:  procOpticalFlow
// Function:  Optical Flow main proc, it is for thread entry function
// Parameter: OF_params *params
// Returns:   void
// Date:    2015/8/5
// author:  zero-tech
/******************************************************************************/
void processOpticalFlow()
{
    struct timeval start,end;

    OF_params *params = &optical_flow_params;

    gettimeofday(&start,NULL);

    DBG("start processOpticalFlow\n");

    if (frame_counter > 0)
    {
        static struct timeval tval1,tval2;
        
        unsigned long diffcost,diffinter;
        gettimeofday(&tval1,NULL);
        diffinter = 1000000 * (tval1.tv_sec - tval2.tv_sec) + (tval1.tv_usec - tval2.tv_usec);
        
        tval2 = tval1;

		rad_frame_index = select_matched_data();
		
        FTrack(params->pre_image_buf, params->cur_image_buf, &PA, &PB);

        diffinter = diffinter;
		
		gettimeofday(&end,NULL);
		
        diffcost = 1000000 * (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec);
        
		sendResult(PB.xyv.x, PB.xyv.y,diffinter,diffcost);
		
        //TRACK("tcost:%6ld ",diffcost);
        diffcost = diffcost;

    }
    frame_counter++;
}

/**********************************************************************
// Method:  initGloabData
// Function:  init gloab data that camera params and control flag included
// Parameter: None
// returns: void
// Date:    2018/4/10
// author:  cyj
**********************************************************************/
void initGloabData()
{
	g_data.ctrl[CTRL_MODE] = 1;
	g_data.ctrl[CTRL_SAVE_LOG] = 0;
	g_data.ctrl[CTRL_SAVE_VIDEO] = 0;
	g_data.params[PARAM_CAM_CENTER_OFFSETX] = 0;
	g_data.params[PARAM_CAM_CENTER_OFFSETY] = 0;
	g_data.params[PARAM_CAM_FOCAL_LENGTH] = 0.83;
	g_data.params[PARAM_CAM_TO_CENTER_DISX] = -16;
	g_data.params[PARAM_CAM_TO_CENTER_DISY] = 24.5;
	g_data.params[PARAM_CAM_PIXEL_SIZE] = 3;
	g_data.params[PARAM_CAM_TO_CENTER_DISX_PIXEL] = g_data.params[PARAM_CAM_TO_CENTER_DISX] * g_data.params[PARAM_CAM_FOCAL_LENGTH] / g_data.params[PARAM_CAM_PIXEL_SIZE];
	g_data.params[PARAM_CAM_TO_CENTER_DISY_PIXEL] = g_data.params[PARAM_CAM_TO_CENTER_DISY] * g_data.params[PARAM_CAM_FOCAL_LENGTH] / g_data.params[PARAM_CAM_PIXEL_SIZE];
}

/**********************************************************************
// Method:  parseCalibFile
// Function:  parse the camere calib file
// Parameter: None
// returns: void
// Date:    2018/4/10
// author:  cyj
**********************************************************************/
void parseCalibFile()
{
	std::ifstream fin("/media/internal/calib/flow_camera_center_calib.txt", std::ios::in); 
	if (!fin) {
		ERROR("----------------optical_flow camera center calib file read fail--------------\n");
	}
	else {
		std::string  str;  
		while (getline(fin,str))
		{    
			while((int)(str.find(" ")) != -1)
				str = str.replace(str.find(" "), 1, "");
			while((int)(str.find("\r")) != -1)
				str = str.replace(str.find("\r"), 1, "");
			while((int)(str.find("\n")) != -1)
				str = str.replace(str.find("\n"), 1, "");
			printf("Read from file: %s\n", str.c_str());

			int pos = -1;
			std::string strpos;
			if((pos = str.find("USE_ZEROTECH:")) != -1) {
				strpos = str.substr(pos + strlen("USE_ZEROTECH:"));
				g_data.ctrl[CTRL_MODE] = atoi(strpos.c_str());
			}
			if((pos = str.find("SAVE_VIDEO:")) != -1) {
				strpos = str.substr(pos + strlen("SAVE_VIDEO:"));
				g_data.ctrl[CTRL_SAVE_VIDEO] = atoi(strpos.c_str());
			}
			if((pos = str.find("SAVE_LOG:")) != -1) {
				strpos = str.substr(pos + strlen("SAVE_LOG:"));
				g_data.ctrl[CTRL_SAVE_LOG] = atoi(strpos.c_str());
			}
			if((pos = str.find("x:")) != -1) {
				strpos = str.substr(pos + strlen("x:"));
				g_data.params[PARAM_CAM_CENTER_OFFSETX] = atof(strpos.c_str());
			}
			if((pos = str.find("y:")) != -1) {
				strpos = str.substr(pos + strlen("y:"));
				g_data.params[PARAM_CAM_CENTER_OFFSETY] = atof(strpos.c_str());
			}
			if((pos = str.find("f:")) != -1) {
				strpos = str.substr(pos + strlen("f:"));
				g_data.params[PARAM_CAM_FOCAL_LENGTH] = atof(strpos.c_str());
			}
			if((pos = str.find("camToIMU_x_mm:")) != -1) {
				strpos = str.substr(pos + strlen("camToIMU_x_mm:"));
				g_data.params[PARAM_CAM_TO_CENTER_DISX] = atof(strpos.c_str());
			}
			if((pos = str.find("camToIMU_y_mm:")) != -1) {
				strpos = str.substr(pos + strlen("camToIMU_y_mm:"));
				g_data.params[PARAM_CAM_TO_CENTER_DISY] = atof(strpos.c_str());
			}

		}
		fin.close();

		g_data.params[PARAM_CAM_TO_CENTER_DISX_PIXEL] = g_data.params[PARAM_CAM_TO_CENTER_DISX] * g_data.params[PARAM_CAM_FOCAL_LENGTH] / g_data.params[PARAM_CAM_PIXEL_SIZE];
		g_data.params[PARAM_CAM_TO_CENTER_DISY_PIXEL] = g_data.params[PARAM_CAM_TO_CENTER_DISY] * g_data.params[PARAM_CAM_FOCAL_LENGTH] / g_data.params[PARAM_CAM_PIXEL_SIZE];
	}
	ERROR("%s use_zero_ctrl: %d, save_log: %d, save_video: %d, x_offset: %f, y_offset: %f, focal_len: %f, camToIMU_x_pixel: %f, camToIMU_y_pixel: %f\n", 
		__func__, g_data.ctrl[CTRL_MODE], g_data.ctrl[CTRL_SAVE_LOG], g_data.ctrl[CTRL_SAVE_VIDEO],
		g_data.params[PARAM_CAM_CENTER_OFFSETX], g_data.params[PARAM_CAM_CENTER_OFFSETY],
		g_data.params[PARAM_CAM_FOCAL_LENGTH], g_data.params[PARAM_CAM_TO_CENTER_DISX_PIXEL], g_data.params[PARAM_CAM_TO_CENTER_DISY_PIXEL]);
}


/**********************************************************************
// Method:  fillFlowBuffer
// Function:  fill the required data for camera calib including image data and IMU data
// Parameter: 
//			U8 * imageData  -- pre image data
//			ParamB* _PB -- a struct containing IMU data
// returns: void
// Date:    2018/4/10
// author:  cyj
**********************************************************************/
void fillFlowBuffer(U8 *imageData, ParamB* _PB)
{
	const float COE = 0.017453; // = 2pi/360 = 0.017453

	flow_buffer->nframe = _PB->nframe;
	flow_buffer->height = _PB->height1 * 100;
	if(flow_buffer->height < 40)
		flow_buffer->height = 50;
	flow_buffer->frame_interval = frame_interval;
	flow_buffer->radvx = flow_buffer->height * COE * _PB->fcpara[rad_frame_index].radvx * 100;
	flow_buffer->radvy = flow_buffer->height * COE * _PB->fcpara[rad_frame_index].radvy * 100;
	flow_buffer->radvz = COE * _PB->fcpara[rad_frame_index].radvz * 10000;

	memcpy((U8*)flow_buffer->data, (U8*)imageData, USB_PREVIEW_W*USB_PREVIEW_H);
}


/**********************************************************************
// Method:  OpticalFlow_Confidence_Filter
// Function:  Filtering the confidence in current opticalflow result for flight control 
// Parameter: 
//			int _input  -- raw confidence
// returns: int         -- processed confidence
// Date:    2018/7/6
// author:  caibo
**********************************************************************/
#ifdef BLOWPASS
int OpticalFlow_Confidence_Filter(int _input)
{
	static int flag_continue_cnt = 0;
	if(flag_continue_cnt > 1000)
	{
		flag_continue_cnt = 30;
	}

	if(_input > 200)
	{
		flag_continue_cnt ++;
	}
	else
	{
		flag_continue_cnt = 0;
	}

	if(flag_continue_cnt > 30)
	{
		return _input;
	}
	else
	{
		return 0;
	}
	
}
#else
int OpticalFlow_Confidence_Filter(int _input)
{
	static float output_pre = 0.0f;

	output_pre = (1 - FILTERIMG_COEFFICIENT) * output_pre + FILTERIMG_COEFFICIENT * _input;

	return output_pre;
	
}
#endif