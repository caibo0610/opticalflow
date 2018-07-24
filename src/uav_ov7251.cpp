/* Copyright (c) 2015, The Linux Foundataion. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <unistd.h>
#include <sys/time.h>
#include <signal.h>
#include <pthread.h>
#include <stdlib.h>
#include <syslog.h>
#include <stdio.h>
#include <sched.h>
#include <sys/shm.h>
#include <math.h>
#include <pthread.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <opencv2/opencv.hpp>

#include "camera.h"
#include "type.h"
#include "lkp.h"
//#include "camera_log.h"
//#include "camera_parameters.h"
#include "uav_ov7251.h"
#include "shmdata.h"

//#include "pz_ae.h"
#include "gduUdpClient.h"

//////socket//////
#define SERVER_PORT 7103
#define LENGTH_OF_LISTEN_QUEUE 20

char revData;
int client_socket;
int server_socket;
char frameHead[2] = {0x1b, 0x2b};
char frameTail[2] = {0x3b, 0x4b};
//pthread_t recv_id, send_id;
U8 *optbuf;
int size_w = USB_PREVIEW_W;
int size_h = USB_PREVIEW_H;
/////socket///////

extern ParamB PB;
extern OF_params optical_flow_params;
extern struct Flow_buffer *flow_buffer;

#define digital_image_processing 0

using namespace std;
using namespace camera;

U32 interrupt_flag = FALSE;
pthread_mutex_t mutex;
pthread_cond_t  cond;
U8 *bufPtr[2], *tmp_buf;
U32 latest_index = 0;
uint64_t timestamp[2] = {0};

S32 exposure_value = 300;//500;//250;//300;//
S32 exposure_max = 1000;//800;//1000;//1028;//500;
S32 exposure_adjust_step = 60;//40;//50;//80;//100;//
S32 gain_value = 50;
S32 exposure_set_flag = 0; //0 means don't change exposure value, 1 means increase, -1 means decrease
S32 frame_interval = 0; //unit us
S32 algo_frame_interval = 0;
S32 ov_running_flag = 0;

int camera_switch_peroid = 1;
int camera_switch_protect_flag = 0; //1 means camear stay on switch status, 0 means camear stay on normal status
/*
struct Chromatix_ae_type ae_params = 
{
	50, //default index
	120, //target luma
	10, //luma torance
	255, //bright shreshod
	2, //sync_exp_gain
	CONV_SLOW, //slow_ae_converance
	0.8, //converance
};

struct Ae_t ae_ctrl = {0};
*/
extern U8 fir_flag;

ImageSize QVGASize(320,240);
ImageSize VGASize(640,480);

struct CameraCaps
{
    vector<ImageSize> pSizes, vSizes;
    vector<string> focusModes, wbModes, isoModes;
    Range brightness, sharpness, contrast;
    vector<Range> previewFpsRanges;
    vector<VideoFPS> videoFpsValues;
};

enum OutputFormatType{
    YUV_FORMAT,
    RAW_FORMAT,
};

enum CamFunction {
    CAM_FUNC_HIRES = 0,
    CAM_FUNC_OPTIC_FLOW = 1,
};

struct TestConfig
{
    bool dumpFrames;
    bool infoMode;
    int runTime;
    CamFunction func;
    OutputFormatType outputFormat;
};

class CameraTest : ICameraListener
{
public:

    CameraTest();
    CameraTest(TestConfig config);
    ~CameraTest();
    int run();
	
    int initialize(int camId);

    /* listener methods */
    virtual void onError();
    virtual void onPreviewFrame(ICameraFrame* frame);
    virtual void onVideoFrame(ICameraFrame* frame);

private:
    ICameraDevice* camera_;
    CameraParams params_;
    ImageSize pSize_, vSize_;
    CameraCaps caps_;
    TestConfig config_;

    int printCapabilities();
    int setParameters();
	U32 raw10_to_320_240_raw8(U8 *input,U8 *output,U8 *tmp_output,uint32_t size);
};

CameraTest::CameraTest()
{
	
}

CameraTest::CameraTest(TestConfig config)
{
    config_ = config;
}

int CameraTest::initialize(int camId)
{
    int rc;

    rc = ICameraDevice::createInstance(camId, &camera_);
    if (rc != 0) {
        printf("could not open camera %d\n", camId);
        return rc;
    }
    camera_->addListener(this);

    rc = params_.init(camera_);
    if (rc != 0) {
        printf("failed to init parameters\n");
        ICameraDevice::deleteInstance(&camera_);
        return rc;
    }
    //printf("params = %s\n", params_.toString().c_str());
    /* query capabilities */
    caps_.pSizes = params_.getSupportedPreviewSizes();
    caps_.vSizes = params_.getSupportedVideoSizes();
    caps_.focusModes = params_.getSupportedFocusModes();
    caps_.wbModes = params_.getSupportedWhiteBalance();
    caps_.isoModes = params_.getSupportedISO();
    caps_.brightness = params_.getSupportedBrightness();
    caps_.sharpness = params_.getSupportedSharpness();
    caps_.contrast = params_.getSupportedContrast();
    caps_.previewFpsRanges = params_.getSupportedPreviewFpsRanges();
    caps_.videoFpsValues = params_.getSupportedVideoFps();

	//ae_init_simple(&ae_ctrl, &ae_params);

    return 0;
}

CameraTest::~CameraTest()
{
    /* release camera device */
    ICameraDevice::deleteInstance(&camera_);
}

U32 CameraTest::raw10_to_320_240_raw8(U8 *input,U8 *output,U8 *tmp_output,uint32_t size)
{
	
	U32 i = 0;
	U32 j = 0;
	uint32_t size_raw640480;
	
	size_raw640480 = size *1.25;
	for ( i = 0; i < size_raw640480;i+=5){
		memcpy(tmp_output + j,input + i,4);
		j+=4;
	}

	j = 0;
	i = 0;
	if((USB_PREVIEW_W == 320) && (USB_PREVIEW_H == 240))
		CutImg((imgtype1*) tmp_output,(imgtype1*)output,VGASize.width,VGASize.height);
	else
		memcpy(output, tmp_output, size);
	return j;
}

void CameraTest::onError()
{
    printf("camera error!, aborting\n");
    exit(EXIT_FAILURE);
}

int onPreviewFrame_DONE = 0;
void CameraTest::onPreviewFrame(ICameraFrame* frame)
{
	onPreviewFrame_DONE = 1;
	if (camera_switch_protect_flag)
		return;

    pthread_mutex_lock(&mutex);

    latest_index = (latest_index + 1) % 2;
    timestamp[latest_index] = frame->timeStamp;

    raw10_to_320_240_raw8(frame->data, bufPtr[latest_index], tmp_buf, 640*480);

	get_rad_velocity();

    get_height();

	pthread_mutex_unlock(&mutex);

    pthread_cond_signal(&cond);

    usleep(6000);
    get_rad_velocity();

    usleep(6000);
    get_rad_velocity();

	onPreviewFrame_DONE = 0;

}

void CameraTest::onVideoFrame(ICameraFrame* frame)
{

}

int CameraTest::printCapabilities()
{
    printf("Camera capabilities\n");

    printf("available preview sizes:\n");
    for (int i = 0; i < (int)caps_.pSizes.size(); i++) {
        printf("%d: %d x %d\n", i, caps_.pSizes[i].width, caps_.pSizes[i].height);
    }
    printf("available video sizes:\n");
    for (int i = 0; i < (int)caps_.vSizes.size(); i++) {
        printf("%d: %d x %d\n", i, caps_.vSizes[i].width, caps_.vSizes[i].height);
    }
    printf("available focus modes:\n");
    for (int i = 0; i < (int)caps_.focusModes.size(); i++) {
        printf("%d: %s\n", i, caps_.focusModes[i].c_str());
    }
    printf("available whitebalance modes:\n");
    for (int i = 0; i < (int)caps_.wbModes.size(); i++) {
        printf("%d: %s\n", i, caps_.wbModes[i].c_str());
    }
    printf("available ISO modes:\n");
    for (int i = 0; i < (int)caps_.isoModes.size(); i++) {
        printf("%d: %s\n", i, caps_.isoModes[i].c_str());
    }
    printf("available brightness values:\n");
    printf("min=%d, max=%d, step=%d\n", caps_.brightness.min,
           caps_.brightness.max, caps_.brightness.step);
    printf("available sharpness values:\n");
    printf("min=%d, max=%d, step=%d\n", caps_.sharpness.min,
           caps_.sharpness.max, caps_.sharpness.step);
    printf("available contrast values:\n");
    printf("min=%d, max=%d, step=%d\n", caps_.contrast.min,
           caps_.contrast.max, caps_.contrast.step);

    printf("available preview fps ranges:\n");
    for (int i = 0; i < (int)caps_.previewFpsRanges.size(); i++) {
        printf("%d: [%d, %d]\n", i, caps_.previewFpsRanges[i].min,
               caps_.previewFpsRanges[i].max);
    }
    printf("available video fps values:\n");
    for (int i = 0; i < (int)caps_.videoFpsValues.size(); i++) {
        printf("%d: %d\n", i, caps_.videoFpsValues[i]);
    }
    return 0;
}


int CameraTest::setParameters()
{
    int pSizeIdx = 0;
    int vSizeIdx = 0;
    int focusModeIdx = 0;
    int wbModeIdx = 0;
    int isoModeIdx = 0;
    int pFpsIdx = 2;
    int vFpsIdx = 0;
	int FPS_ov7251_puzhou = 50;

    pSize_ = caps_.pSizes[pSizeIdx];
    vSize_ = caps_.pSizes[vSizeIdx];

	//pSize_ = QVGASize;
	//vSize_ = QVGASize;
	pSize_ = VGASize;
	vSize_ = VGASize;

	printf("set parameter pSize.w=%d, vSize.w=%d\n",pSize_.width,vSize_.width);

    if ( config_.func == CAM_FUNC_OPTIC_FLOW ){
	pSize_ = VGASize;
	vSize_ = VGASize;
    }

    printf("setting preview size: %dx%d\n", pSize_.width, pSize_.height);
    params_.setPreviewSize(pSize_);
    printf("setting video size: %dx%d\n", vSize_.width, vSize_.height);
    params_.setVideoSize(vSize_);

    if ( config_.func != CAM_FUNC_OPTIC_FLOW ){
      printf("setting focus mode: %s\n", caps_.focusModes[focusModeIdx].c_str());
      params_.setFocusMode(caps_.focusModes[focusModeIdx]);
      printf("setting WB mode: %s\n", caps_.wbModes[wbModeIdx].c_str());
      params_.setWhiteBalance(caps_.wbModes[wbModeIdx]);
      printf("setting ISO mode: %s\n", caps_.isoModes[isoModeIdx].c_str());
      params_.setISO(caps_.isoModes[isoModeIdx]);

      printf("setting preview fps range: %d, %d\n",
           caps_.previewFpsRanges[pFpsIdx].min,
           caps_.previewFpsRanges[pFpsIdx].max);
      params_.setPreviewFpsRange(caps_.previewFpsRanges[pFpsIdx]);

      printf("setting video fps: %d\n", caps_.videoFpsValues[vFpsIdx]);
      params_.setVideoFPS(caps_.videoFpsValues[vFpsIdx]);
    }
    else
    {
       // params_.setPreviewFpsRange(caps_.previewFpsRanges[pFpsIdx]);
		Range preview_Fps_ranges_TEST;
    	preview_Fps_ranges_TEST.max = FPS_ov7251_puzhou*1000;
    	preview_Fps_ranges_TEST.min = FPS_ov7251_puzhou*1000;

    	params_.setPreviewFpsRange(preview_Fps_ranges_TEST);
		printf("UAV FPS : %dfps \n",FPS_ov7251_puzhou);
        printf("ISO mode: %s\n", caps_.isoModes[isoModeIdx].c_str());

        char str[32];
        sprintf(str,"%u",exposure_value);
		//sprintf(str,"%u",ae_ctrl.cur_linecnt);
        params_.set( "qc-exposure-manual", str);
        sprintf(str,"%u",gain_value);
		//sprintf(str,"%u",ae_ctrl.cur_gain);
        params_.set( "qc-gain-manual", str);
        params_.commit();
    }

    if (config_.outputFormat == RAW_FORMAT)
    {
        params_.set("preview-format", "bayer-rggb");
        params_.set("picture-format", "bayer-mipi-10gbrg");
        params_.set("raw-size", "640x480");
    }
    return params_.commit();
}
//--->begin libo 20180420
int send_data_to_CE = 0;
void *CE_control(void *data)
{
    gduUdpClient my_udp;
    while(1)
    {
        if(1==send_data_to_CE)
        {
            my_udp.initalParameter();//只会初始化一次
            my_udp.gduSetUAV();
        }
        usleep(500000);
    }
    
    return ((void *)0);
}
//--->end libo 20180420

void *init_func(void *p)
{
    //--->begin libo 20180420

    pthread_t send_date_to_CE_thread;
    pthread_create(&send_date_to_CE_thread, NULL, &CE_control, NULL);
    pthread_detach(send_date_to_CE_thread);

    //--->end libo 20180420
    // socket
    int server_socket;
    int client_socket;
    //pthread_t recv_id, send_id;
    struct sockaddr_in server_addr;
    int opt = 1;
    struct sockaddr_in client_addr;
    socklen_t length;
    bzero(&server_addr, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    // server_addr.sin_addr.s_addr = htons(INADDR_ANY);
    inet_aton("127.0.0.1", (struct in_addr *)&server_addr.sin_addr);
    server_addr.sin_port = htons(SERVER_PORT);
    server_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (server_socket < 0)
    {
        printf("Create Socket Failed!\n");
        exit(1);
    }
    //int flags = fcntl(client_socket,F_GETFL,0);
    //fcntl(client_socket,F_SETFL,flags|O_NONBLOCK);
    // bind a socket
    setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    if (bind(server_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)))
    {
        printf("Server Bind Port: %d Failed!\n", SERVER_PORT);
        exit(1);
    }
    if (listen(server_socket, LENGTH_OF_LISTEN_QUEUE))
    {
        printf("Server Listen Failed!\n");
        exit(1);
    }
	while(1) {
    	printf("Socket Accepting... \n");
    	length = sizeof(client_addr);
    	client_socket = accept(server_socket, (struct sockaddr *)&client_addr, &length);
    	if (client_socket < 0)
    	{
        	printf("Server Accept Failed!\n");
        	return NULL;
    	}
    	int snd_size = 150 * 1024;
    	length = sizeof(snd_size);
    	int err = setsockopt(client_socket, SOL_SOCKET, SO_SNDBUF, &snd_size, length);
    	if (err < 0)
    	{
        	printf("set sock opt error!\n");
    	}

		//char ImageName[100] = "";
		unsigned int old_frame = 0;

        send_data_to_CE = 1;  //libo 20180420

		unsigned char sendData[flowBufferSize + 4] = " ";
		while(1) {
    		while(old_frame == flow_buffer->nframe) {
				//printf("%s: old_frame = %d, cur_frame = %d\n", __func__, old_frame, flow_buffer->nframe);
				usleep(1000);
    		}

    		old_frame = flow_buffer->nframe;
        	memcpy(sendData, &frameHead, 2);
        	memcpy(sendData + 2, flow_buffer, flowBufferSize);
        	memcpy(sendData + flowBufferSize + 2, &frameTail, 2);

			struct timeval start,end;
		
			gettimeofday(&start,NULL);
        	int n = send(client_socket, sendData, flowBufferSize + 4, MSG_WAITALL);
        	printf("%s: nframe = %d, send size is %d\n", __func__, flow_buffer->nframe, n);

        	int m = recv(client_socket, &revData, sizeof(revData), MSG_WAITALL);
        	printf("%s: recv size is %d, recv data is %c\n", __func__,  m, revData);

			gettimeofday(&end,NULL);
			printf("%s: send and receive buffer running time is: %ld us\n", __func__, (unsigned long)(1000000 * (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec)));
			if((n == -1) && (revData == ' '))
				break;
/*
        if(m == 1)
        {        
            if (revData == 's')
            {
    			static int j = 0;
            	if(access("/home/linaro/calib/opt/", 0) == 0) {
                	sprintf(ImageName,"/home/linaro/calib/opt/opt%d.bmp",++j);
					cv::Mat optGray(cv::Size(size_w,size_h),CV_8UC1);
					optGray.data = flow_buffer->data;
                	cv::imwrite(ImageName,optGray);
            	}
            }
            if (revData == 'c' || revData == 'a')
            {
                FILE *fp;
                if ((fp = fopen("opt_error.txt", "wb")) == NULL)
                {
                    printf("open log file error!!!\n");
                    exit(0);
                }
                usleep(30);
            }
            if (revData == 'e')
            {
                printf("stop!\n");
                system("reboot");
                break;
            }
        }
*/
    	}
	}
    return NULL;
}

int CameraTest::run()
{
    // socket connction with server//
    
    int client;
    pthread_t init_id;

    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

    if (pthread_create(&init_id, &attr, init_func, &client)) 
    {
        printf("pthread_create read_func err\n");
    }

    ////////socket connction with server/////////

    int rc = EXIT_SUCCESS;

	//share memory initialzie
	void *shm = NULL;
	struct shared_use_st *shared = NULL;
	int shmid;
		
	//get share memory
	shmid = shmget((key_t)6538, sizeof(struct shared_use_st), 0666|IPC_CREAT);
	if (shmid == -1)
	{
		fprintf(stderr,"shmget failed\n");
		exit(EXIT_FAILURE);
	}

	// map the share memory to local address
	shm = shmat(shmid,(void*)0,0);
	if (shm == (void*)-1)
	{
		fprintf(stderr, "shmat failed\n");
		exit(EXIT_FAILURE);
	}

	shared = (struct shared_use_st*)shm;

    ERROR("will call getNumberOfCameras\n");
    int n = getNumberOfCameras();

    ERROR("num_cameras = %d\n", n);
    if (n < 1) {
        printf("No cameras found.\n");
        return EXIT_FAILURE;
    }

    int camId=-1;

    ERROR("config_.func = %d\n",config_.func);
    /* find camera based on function */
    for (int i=0; i<n; i++) {
        CameraInfo info;
        getCameraInfo(i, info);
        ERROR("i = %d,info.func=%d,config_.func=%d\n",i,info.func,config_.func);
        if (info.func == config_.func) {
            camId = i;
        }
    }

    if (camId < 0)
    {
        ERROR("can't find optic flow camera\n");
        return EXIT_FAILURE;
    }

    ERROR("testing camera index=%d\n", camId);

    initialize(camId);
    ERROR("finished initialization\n");

    if (config_.infoMode) {
        printCapabilities();
        return rc;
    }

    int ret = initialzeOpticalFlow();

	
    if (ret != 0)
    {
        ERROR("optical flow configuration failure, error code %d\n",ret);
        exit(1);
    }
    ERROR("finished initialzeOpticalFlow\n");

    setParameters();
    ERROR("finished setParameters\n");

    bufPtr[0] = (U8*)malloc(USB_PREVIEW_W*USB_PREVIEW_H);//385024
    bufPtr[1] = (U8*)malloc(USB_PREVIEW_W*USB_PREVIEW_H);//385024
	tmp_buf = (U8*)malloc(640*480);

    pthread_mutex_init(&mutex,NULL);
    pthread_cond_init(&cond,NULL);

    static uint64_t old_timestamp = 0;
    U8 *curbuf;
	U8 *prebuf;
	float curr_radvx,curr_radvy,curr_radvz;
    S32 local_starup_flag;
    int algo_stop_counter = 0;
    int algo_running_counter = 0;
	
    ERROR("start loop\n");
    local_starup_flag = 0;
	int wait_pic_counter = 0;

    while (1)
    {
        DBG("run processing begin \n");
		shared->counter++;
        if (interrupt_flag == TRUE)
        {
            ERROR("receive SIGINT mesg, loop is over\n");
            break;
        }

        /******************************************************************/
        #if 1
		//get running flag from flight control
        local_starup_flag = get_running_flag();

        if (ov_running_flag != local_starup_flag)
        {
            if (local_starup_flag == 1)
            {
				ERROR("stop->start, stopcounter = %d\n", algo_stop_counter);
                if ((ov_running_flag < 1) && (algo_stop_counter > 2))
                {
                    ERROR("start preview\n");
					camera_switch_protect_flag = 1;
                    camera_->startPreview();
					camera_switch_protect_flag = 0;
                    ERROR("start preview ok\n");
					ov_running_flag = local_starup_flag;
					fir_flag = 0;
                }
            }
            else
            {
				ERROR("start->stop, runningcounter = %d, ov_running_flag = %d, flight_running_flag = %d\n", algo_running_counter, ov_running_flag, local_starup_flag);
                if ((ov_running_flag == 1) && (algo_running_counter > 80))
                {
                	while(onPreviewFrame_DONE == 1)
                	{
                		printf("wait onproview STOP !! \n");
            			usleep(1000);//
                	}
                    ERROR("stop preview\n");
					camera_switch_protect_flag = 1;
                    camera_->stopPreview();
					camera_switch_protect_flag = 0;
                    ERROR("stop preview ok\n");
					ov_running_flag = local_starup_flag;
                }
            }
        }

        if (ov_running_flag == 0)
        {
			//clear algo running_counter
			algo_running_counter = 0;

            algo_stop_counter++;
			if ((algo_stop_counter > 20) && (shared->counter > 200))
			{
				ERROR("algo stop, but stop opticflow because deamon procee hasn't clear the flag, counter is %d\n", shared->counter);
				break;
			}
            usleep(100000);
            continue;
        }

		//run opticflow manually or if ov-opticflow is dead, opticflow should stop
		if (shared->counter > 1200)
		{
			ERROR("algo runing, but stop opticflow because deamon procee hasn't clear the flag\n");
			break;
		}

        algo_stop_counter = 0;
		algo_running_counter++;

		if ((algo_running_counter & 0x1f) == 0x1f)
			{
			//ERROR("algo is running, running counter is %d\n",algo_running_counter);
			}
        #else
        camera_->startPreview();
        #endif

        /******************************************************************/

        while (timestamp[latest_index] == old_timestamp)
        {
			usleep(1000);
			wait_pic_counter++;
			if((wait_pic_counter > 10)&&(local_starup_flag == 1))
			//if(local_starup_flag == 1)
			{
				shared->wait_pic_renew++;
				wait_pic_counter = 0;
            }
			//printf("cyj:%s:wait_pic_renew = %d\n", __func__, shared->wait_pic_renew);
		}
		wait_pic_counter = 0;
		shared->wait_pic_renew = 0;

        pthread_mutex_lock(&mutex);
        frame_interval = (int32_t)((timestamp[latest_index] - timestamp[(latest_index+1) % 2])/1000);
        algo_frame_interval = (int32_t)((timestamp[latest_index] - old_timestamp)/1000);
      	old_timestamp = timestamp[latest_index];

		get_optiflow_next_buf(&curbuf, &prebuf,&curr_radvx,&curr_radvy,&curr_radvz);
		memcpy((void*)curbuf, (void*)bufPtr[latest_index], USB_PREVIEW_W*USB_PREVIEW_H);//385024
		memcpy((void*)prebuf, (void*)bufPtr[(latest_index + 1) % 2], USB_PREVIEW_W*USB_PREVIEW_H);//385024
	   	pthread_mutex_unlock(&mutex);

		struct timeval tt1, tt2;
		unsigned long cost_time = 0;
		gettimeofday(&tt1,NULL);
	   	processOpticalFlow();
		gettimeofday(&tt2,NULL);
		cost_time = 1000000 * (tt2.tv_sec - tt1.tv_sec) + (tt2.tv_usec - tt1.tv_usec);
		//ERROR("chenyijun:processOpticalFlow cost time = %ld us\n", cost_time);
		/*
		if(!ae_ctrl.ae_settled) {
			char str[32];
			int need_update_ae = 0;
			if(ae_ctrl.cur_linecnt != ae_ctrl.last_linecnt) {
				ae_ctrl.last_linecnt = ae_ctrl.cur_linecnt;
				sprintf(str,"%u", ae_ctrl.cur_linecnt);
            	params_.set( "qc-exposure-manual", str);
				need_update_ae = 1;
			}
			if(ae_ctrl.cur_gain != ae_ctrl.last_gain) {
				ae_ctrl.last_gain = ae_ctrl.cur_gain;
				sprintf(str,"%u", ae_ctrl.cur_gain);
				params_.set( "qc-gain-manual", str);
				need_update_ae = 1;
			}

			if(need_update_ae) {
				params_.commit();
				ae_ctrl.skip_frame = ae_params.sync_exp_gain;
			}
		}
		*/
        if (exposure_set_flag)
        {
            char str[32];
            int adjust_step;
            int exposure_min = 1;
            if (exposure_value <= 100 || exposure_value > 700)
                adjust_step = exposure_adjust_step / 5;
            else
                adjust_step = exposure_adjust_step;

			if (exposure_value < 20)
                adjust_step = 5;

            if (exposure_value < 10)
                adjust_step = 1;

            exposure_value += exposure_set_flag * adjust_step;
            exposure_value = exposure_value > exposure_max ? exposure_max : exposure_value;
            exposure_value = exposure_value < exposure_min ? exposure_min : exposure_value;

            sprintf(str,"%u",exposure_value);
            params_.set( "qc-exposure-manual", str);

            if (exposure_value < 20 && gain_value > 30)
            {
                params_.set( "qc-gain-manual", "0");
                gain_value = 0;
            }

            if (exposure_value == exposure_max && gain_value < 80)//50)
            {
                params_.set( "qc-gain-manual", "50");
                gain_value = 50;
            }
            params_.commit();
            exposure_set_flag = 0;
        }	
//#ifdef OPENCV_FLOW
		if(cost_time < 8000)
			usleep(10000);
		else if(cost_time < 12000)
			usleep(5000);
//#endif
    }

    pthread_mutex_destroy(&mutex);
    pthread_cond_destroy(&cond);

    free(bufPtr[0]);
    free(bufPtr[1]);
    free(tmp_buf);

    camera_->stopPreview();
    releaseOpticalFlow();

    return rc;
}

const char usageStr[] =
    "Camera API test application \n"
    "\n"
    "usage: camera-test [options]\n"
    "\n"
    "  -t <duration>   capture duration in seconds [10]\n"
    "  -d              dump frames\n"
    "  -i              info mode\n"
    "                    - print camera capabilities\n"
    "                    - streaming will not be started\n"
    "  -f <type>       camera type\n"
    "                    - hires\n"
    "                    - optic\n"
    "  -o Output format\n"
    "                   0 :YUV format (default)\n"
    "                   1 : RAW format \n"
    "  -h              print this message\n"
    "  -v              dispaly version info\n"
;

static inline void printUsageExit(int code)
{
    printf("%s", usageStr);
    exit(code);
}

/* parses commandline options and populates the config
   data structure */

static TestConfig parseCommandline(int argc, char* argv[])
{
    TestConfig cfg;
	memset(&cfg,0,sizeof(cfg));
    cfg.outputFormat = YUV_FORMAT;
    int outputFormat;
    /* default config */
    cfg.dumpFrames = false;
    cfg.runTime = 2;
    cfg.func = CAM_FUNC_HIRES;
    int c;
    while ((c = getopt(argc, argv, "hdt:if:o:v")) != -1) {
        switch (c) {;
          case 't':
              cfg.runTime = atoi(optarg);
              break;
          case 'f':
          {
                  string str(optarg);
                  if (str == "hires") {
                      cfg.func = CAM_FUNC_HIRES;
                  } else if (str == "optic") {
                      cfg.func = CAM_FUNC_OPTIC_FLOW;
                  }
                  break;
          }
          case 'd':
              cfg.dumpFrames = true;
              break;
          case 'i':
              cfg.infoMode = true;
              break;
         case 'o':
            outputFormat = atoi(optarg);
            switch ( outputFormat )
            {
                case 0: /* IMX135 , IMX214 */
                   cfg.outputFormat = YUV_FORMAT;
                   break;
                case 1: /* IMX214 */
                    cfg.outputFormat = RAW_FORMAT;
                    break;
                default:
                    printf("Invalid format. Setting to default YUV_FORMAT");
                    cfg.outputFormat = YUV_FORMAT;
                    break;
            }
	        break;
          case 'v':
          {
            int versionid = OPTIC_VERSION;
            char date[] = REL_DATE;
            int degree = get_camera_type();
            char version[256];
            sprintf(version,
                    "\n"
                    "  version. 1.9.%d  \n"
                    "  Release time: %s  \n"
                    "  get FOV from flightcontrol: %d\n"
                    "  camera switch off"
                    "  optic processing max frame rate: 40Hz  \n", versionid,date,degree);

            printf("optical flow version info:\n %s\n",version);
            exit(EXIT_SUCCESS);
            break;
          }
          case 'h':
          case '?':
              printUsageExit(0);
          default:
              abort();
        }
    }
    return cfg;
}

void interruptCapture()
{
    ERROR("receive SIGINT\n");
    interrupt_flag = TRUE;
    exit(0);
}

int main(int argc, char* argv[])
{
    ERROR("main start, will call sigemptyset\n");
    
#if 1
	setlogmask(LOG_UPTO(LOG_EMERG));
    openlog(NULL, LOG_NDELAY, LOG_DAEMON);
    sigset_t blockSet;
    sigfillset(&blockSet);
    pthread_sigmask(SIG_BLOCK,& blockSet, NULL);
    
#endif
    if (setvbuf(stdout,NULL,_IOLBF,0)){
        ERROR("set cache buf for stdout\n");
    }

    ERROR("will call parseCommandline\n");

    TestConfig config = parseCommandline(argc, argv);

    config.func = CAM_FUNC_OPTIC_FLOW;
    config.dumpFrames = false;
    config.outputFormat = RAW_FORMAT;

    ERROR("will call CameraTest()\n");

    CameraTest test(config);

    ERROR("will call test.run\n");

    test.run();

    return EXIT_SUCCESS;
}
