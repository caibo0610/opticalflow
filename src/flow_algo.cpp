/********************************************************************
  created:  2015/07/27
  filename:   ggf
  file ext:  c
  author:  Fu Chongyang
  purpose:  使用光流算法跟踪图像移动
*********************************************************************/
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <fstream>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <unistd.h>
#include <sys/time.h>
#include <arm_neon.h>
#include "opticflow.h"
#include "flow_algo.h"

using namespace cv;
using namespace std;
 
#define max(a,b) (((a) > (b)) ? (a) : (b))
#define min(a,b) (((a) < (b)) ? (a) : (b))

extern struct global_struct g_data;

struct greaterThanPtr :
        public std::binary_function<const float *, const float *, bool>
{
    bool operator () (const float * a, const float * b) const
    // Ensure a fully deterministic result of the sort
    { return (*a > *b) ? true : (*a < *b) ? false : (a > b); }
};

/**********************************************************************
// Method:  GetRightPoint1
// Function:  select matched corners,The corners' moving is similar to the more corners, the higher of its scores.choose the first ten corners of the score
// Parameter: 
//			ParamB* _PB  -- a struct containing feature point's cordinates/status etc.
//			vector<Point2f> features  -- coordinate of previou image's detected corners
//			vector<Point2f> features_after  -- coordinate of next image's matched corners
//			vector<uchar> status -- corners' status, 1 -- valid, 0 -- invalid, non participation in the final shift's calculation
// Date:    2018/5/24
// author:  cyj
**********************************************************************/

float GetRightPoint1(ParamB* _PB, vector<Point2f> features, vector<Point2f> features_after, vector<uchar> status)
{
	int i,j;
	double dx,dy,dx1,dy1,dx2,dy2;
	float nx[maxnum];
	float ny[maxnum];
	float sumi[maxnum];
	float sumx=0;
	float sumy=0;
	int upth;
	float norm[maxnum];

	int numpoint;//特征点个数
	int sumstatus_all,sumstatus;
	//Point2f_z* point;
	int * posi; 
	//int * status;
	//float* prevPts;
	//float *nextPts;
	int* pnump;
	int distflag;
	float th;
	float n2;

    //DBG("enter GetRightPoint3\n");

    //CRASH();
	posi = _PB->posi;
	//status = PB->status;
	//prevPts = PB->prevPts;
	//nextPts = PB->nextPts;
	pnump = _PB->nump; 
	sumstatus_all=0;
	sumstatus=0;//符合status的个数 前10精英的sumstatus
	numpoint=*pnump;
    _PB->percent = 0;
	
	if (numpoint>0)
	{
		for(i=0; i<numpoint; i++) 
		{
			//dx = nextPts[2*i]-prevPts[2*i];//
			//dy = nextPts[2*i+1]-prevPts[2*i+1];
			dx = features_after[i].x - features[i].x;//
			dy = features_after[i].y - features_after[i].y;

			norm[i] = sqrt(dx*dx+dy*dy);

			if (norm[i]>1e-6)
			{
				dx=dx/norm[i];
				dy=dy/norm[i];
			}
			else
			{
				dx=0;
				dy=0;
			}
			nx[i]=dx;//标准化速度向量
			ny[i]=dy;//标准化速度向量

		}
        // CRASH("");
		//计算得分
		for (i=0; i<numpoint; i++)
		{

			if (status[i]==0)
			{
				sumi[i]=0;       //对于status等于0的地方 得分为0
			}
			else
			{
				sumi[i]=0;
				//dx1 = nextPts[2*i]-prevPts[2*i];//
				//dy1 = nextPts[2*i+1]-prevPts[2*i+1];
				dx1 = features_after[i].x - features[i].x;
				dy1 = features_after[i].y - features_after[i].y;
				for (j=0;j<numpoint;j++)
				{
					distflag=1;
					//dx2 = nextPts[2*j]-prevPts[2*j];//
					//dy2 = nextPts[2*j+1]-prevPts[2*j+1];
					dx2 = features_after[j].x - features[j].x;
					dy2 = features_after[j].y - features_after[j].y;
					distflag=((dx1-dx2)*(dx1-dx2)+(dy1-dy2)*(dy1-dy2))>1?0:1;//距离判定 距离大分数为0;

					
					if ((norm[i]<GRP_POINT_INTERVAL)&&(norm[j]<GRP_POINT_INTERVAL))
						n2=1;
					else
						n2=nx[i]*nx[j]+ny[i]*ny[j];
					
					sumi[i]=sumi[i]+n2*status[j]*distflag;//统计得分i,j  距离大为0  status 0 为0  
				}

			}
			posi[i]=i;//posi 记录最初顺序

		}
		for (i=0; i<numpoint; i++)
		{
			_PB->sumnum[i]=sumi[i];
		}

        //CRASH
		Sort2(sumi, numpoint, posi);//posi记录排序后的顺序

		th= GRP_POINT_SCORE * sumi[0];
		sumstatus_all=0;
        
		//根据得分计算是否有效
		for (i=0; i<numpoint; i++)
		{
			if (_PB->sumnum[i]<th)
			{
				status[i]=0;
			}
			else
			{
				sumstatus_all=sumstatus_all+status[i];
			}
		}

		*_PB->nump=sumstatus_all;//更新特征点个数
	    _PB->percent = sumi[0]/numpoint;
		upth = sumstatus_all<10? sumstatus_all:10;//取10个特征点如果numpoint<10 取numpoint
		
		for (i=0;i<upth;i++)
		{
			j=posi[i];
			//sumx= sumx+(nextPts[j*2]-prevPts[j*2])*status[j];
			//sumy= sumy+(nextPts[j*2+1]-prevPts[j*2+1])*status[j];
			sumx= sumx+(features_after[j].x-features[j].x)*status[j];
			sumy= sumy+(features_after[j].y-features[j].y)*status[j];
			if (status[j])//如果排序后的posi[]中存在status[j]==0的数 不给以统计
			sumstatus++;
		}
        
        //CRASH();
		if (sumstatus)
		{
			_PB->pv->x = sumx/sumstatus;//upth; 
			_PB->pv->y = sumy/sumstatus;//upth;
		
		}
		else
		{
			_PB->pv->x = 0;
			_PB->pv->y = 0;
			_PB->percent = -1;
			//printf("sumstatus==0\n");
		}
	}
	else
	{
		_PB->pv->x = 0;
		_PB->pv->y = 0;
		_PB->percent = -1;
		//printf("nump1==0\n");
	}

	//if (PB->m_bspeed)
	//{
		//_PB->pv->x=_PB->pv->x;
		//_PB->pv->y=_PB->pv->y;
	//}

	_PB->v=sqrt((_PB->pv->x)*(_PB->pv->x)+(_PB->pv->y)*(_PB->pv->y));

	return _PB->percent;
}


/**********************************************************************
// Method:  shiTomasiFeaturesToTrack
// Function:  shi-Tomasi corner detect
// Parameter: 
//			InputArray _image  -- in image
//			OutputArray _corners  -- corner coordinate detected
//			int maxCorners  -- limit max number of corners
//			double qualityLevel -- Parameter characterizing the minimal accepted quality of image corners. The parameter value is multiplied by the best corner quality measure
//			double minDistance -- Minimum possible Euclidean distance between the returned corners
//			InputArray _mask -- Optional region of interest
//			int blockSize -- Size of an average block for computing a derivative covariation matrix over each pixel neighborhood
//			int gradientSize -- Aperture parameter for the Sobel() operator.
// Date:    2018/5/24
// author:  cyj
**********************************************************************/

int shiTomasiFeaturesToTrack(InputArray _image, OutputArray _corners,
                              int maxCorners, double qualityLevel, double minDistance,
                              InputArray _mask, int blockSize, int gradientSize)
{
	//double harrisK = 0.4;
	//bool useHarrisDetector = 0;
    //CV_INSTRUMENT_REGION()

    CV_Assert( qualityLevel > 0 && minDistance >= 0 && maxCorners >= 0 );
    CV_Assert( _mask.empty() || (_mask.type() == CV_8UC1 && _mask.sameSize(_image)) );
/*
    CV_OCL_RUN(_image.dims() <= 2 && _image.isUMat(),
               ocl_goodFeaturesToTrack(_image, _corners, maxCorners, qualityLevel, minDistance,
                                    _mask, blockSize, gradientSize, useHarrisDetector, harrisK))
*/
    Mat image = _image.getMat(), eig, tmp;
    if (image.empty())
    {
        _corners.release();
        return 0;
    }

    // Disabled due to bad accuracy
    /*
    CV_OVX_RUN(false && useHarrisDetector && _mask.empty() &&
               !ovx::skipSmallImages<VX_KERNEL_HARRIS_CORNERS>(image.cols, image.rows),
               openvx_harris(image, _corners, maxCorners, qualityLevel, minDistance, blockSize, gradientSize, harrisK))
	*/

    //if( useHarrisDetector )
        //cornerHarris( image, eig, blockSize, gradientSize, harrisK);
    //else
    cornerMinEigenVal( image, eig, blockSize, gradientSize);//compute Mat M's eig value

    double maxVal = 0;
    minMaxLoc( eig, 0, &maxVal, 0, 0, _mask);

	//为了跟零度源码中PB->maxeig的计算方法保持一致，Opencv在sobel求梯度时乘以了缩小因子1/scale，故此处应该乘以scale*scale
	int depth = _image.depth();
	double scale = (double)(1 << ((gradientSize > 0 ? gradientSize : 3) - 1)) * blockSize;
	if (gradientSize < 0)
		scale *= 2.0;
	if (depth == CV_8U)
		scale *= 255.0;
	int maxeig = int(maxVal * scale * scale);

    threshold( eig, eig, maxVal*qualityLevel, 0, THRESH_TOZERO);
    dilate( eig, tmp, Mat());

    Size imgsize = image.size();
    std::vector<const float*> tmpCorners;

    // collect list of pointers to features - put them into temporary image
    Mat mask = _mask.getMat();
    for( int y = 1; y < imgsize.height - 1; y++ )
    {
        const float* eig_data = (const float*)eig.ptr(y);
        const float* tmp_data = (const float*)tmp.ptr(y);
        const uchar* mask_data = mask.data ? mask.ptr(y) : 0;

        for( int x = 1; x < imgsize.width - 1; x++ )
        {
            float val = eig_data[x];
            if( val != 0 && val == tmp_data[x] && (!mask_data || mask_data[x]) )
                tmpCorners.push_back(eig_data + x);
        }
    }

    std::vector<Point2f> corners;
    size_t i, j, total = tmpCorners.size(), ncorners = 0;

    if (total == 0)
    {
        _corners.release();
        return 0;
    }

    std::sort( tmpCorners.begin(), tmpCorners.end(), greaterThanPtr());

	//collect features that the diatance between two features must be grater than minDistance
    if (minDistance >= 1)
    {
         // Partition the image into larger grids
        int w = image.cols;
        int h = image.rows;

        const int cell_size = cvRound(minDistance);
        const int grid_width = (w + cell_size - 1) / cell_size;
        const int grid_height = (h + cell_size - 1) / cell_size;

        std::vector<std::vector<Point2f> > grid(grid_width*grid_height);

        minDistance *= minDistance;

        for( i = 0; i < total; i++ )
        {
            int ofs = (int)((const uchar*)tmpCorners[i] - eig.ptr());
            int y = (int)(ofs / eig.step);
            int x = (int)((ofs - y*eig.step)/sizeof(float));

            bool good = true;

            int x_cell = x / cell_size;
            int y_cell = y / cell_size;

            int x1 = x_cell - 1;
            int y1 = y_cell - 1;
            int x2 = x_cell + 1;
            int y2 = y_cell + 1;

            // boundary check
            x1 = max(0, x1);
            y1 = max(0, y1);
            x2 = min(grid_width-1, x2);
            y2 = min(grid_height-1, y2);

            for( int yy = y1; yy <= y2; yy++ )
            {
                for( int xx = x1; xx <= x2; xx++ )
                {
                    std::vector <Point2f> &m = grid[yy*grid_width + xx];

                    if( m.size() )
                    {
                        for(j = 0; j < m.size(); j++)
                        {
                            float dx = x - m[j].x;
                            float dy = y - m[j].y;

                            if( dx*dx + dy*dy < minDistance )
                            {
                                good = false;
                                goto break_out;
                            }
                        }
                    }
                }
            }

            break_out:

            if (good)
            {
                grid[y_cell*grid_width + x_cell].push_back(Point2f((float)x, (float)y));

                corners.push_back(Point2f((float)x, (float)y));
                ++ncorners;

                if( maxCorners > 0 && (int)ncorners == maxCorners )
                    break;
            }
        }
    }
    else
    {
        for( i = 0; i < total; i++ )
        {
            int ofs = (int)((const uchar*)tmpCorners[i] - eig.ptr());
            int y = (int)(ofs / eig.step);
            int x = (int)((ofs - y*eig.step)/sizeof(float));

            corners.push_back(Point2f((float)x, (float)y));
            ++ncorners;
            if( maxCorners > 0 && (int)ncorners == maxCorners )
                break;
        }
    }

    Mat(corners).convertTo(_corners, _corners.fixedType() ? _corners.type() : CV_32F);
	return maxeig;
}

/**********************************************************************
// Method:  compute_flow_opencv
// Function:	compute flow using opencv calcOpticalFlowPyrLK methold
// Parameter: 
//		  Mat pre_img  -- pre image data
//		  Mat cur_img  -- next image data
//		  ParamA* _PA  -- a struct containing params for getting and tracking feature points
//		  ParamB* _PB -- a struct containing IMU data, feature point's cordinates/status etc.
// returns: void
// Date:	  2018/5/24
// author:  cyj
**********************************************************************/

void compute_flow_opencv(Mat pre_img, Mat cur_img, ParamA* _PA, ParamB* _PB)
{
	//struct timeval t1, t2, t3, t4;
	vector<Point2f> features_after, features;
	vector<uchar> status;
	vector<float> err;
	Size winSize = Size(11, 11);
	int maxLevel = 2;
	//int goodPoints = 0;

	//gettimeofday(&t1, NULL);
    _PB->maxeig = shiTomasiFeaturesToTrack(pre_img, features, _PA->maxCorners, _PA->th, _PA->minDist, Mat(), 3, 3);
    //goodFeaturesToTrack(pre_img, features, _PA->maxCorners, _PA->th, _PA->minDist, Mat(), 3, 3);
	//gettimeofday(&t2, NULL);
	//ERROR("chenyijun:%s:goodFeaturesToTrack cost time = %ld us, points num = %d, maxeig = %d\n", 
		//__func__, 
		//1000000 * (t2.tv_sec - t1.tv_sec) + (t2.tv_usec - t1.tv_usec), features.size(), _PB->maxeig);

	if(features.size() > 0) {
		//gettimeofday(&t3, NULL);
		calcOpticalFlowPyrLK(pre_img, cur_img, features, features_after, status, err, winSize, maxLevel);
		//gettimeofday(&t4, NULL);
		//ERROR("chenyijun:%s:calcOpticalFlowPyrLK cost time = %ld us\n", 
			//__func__, 
			//1000000 * (t4.tv_sec - t3.tv_sec) + (t4.tv_usec - t3.tv_usec));

		*(_PB->nump) = features.size();
    	for (int i=0;i<(*(_PB->nump));i++)
    	{
        	_PB->prevPts[2*i]=features[i].x;
        	_PB->prevPts[2*i+1]=features[i].y;
       		_PB->nextPts[2*i]=features_after[i].x;
        	_PB->nextPts[2*i+1]=features_after[i].y;
			if((fabs(features_after[i].x - features[i].x) <= 20) && (fabs(features_after[i].y - features[i].y) <= 20)) {
				//status[i] = status[i];
        		_PB->status[i]=status[i];
			} else {
				_PB->status[i]=0;
				//status[i] = 0;
				//ERROR("chenyijun:%s: unconfident tracked points, x_dis = %f pixel, y_dis = %f pixel\n", 
				//__func__,
				//features_after[i].x - features[i].x, features_after[i].y - features[i].y);
			}
			//if(_PB->status[i] == 1)
				//goodPoints++;
    	}

		//ERROR("chenyijun:%s:before filter, correctly tracked points num = %d\n", __func__, goodPoints);
		//GetRightPoint1(_PB, features, features_after, status);
		//ERROR("chenyijun:%s:nframe = %d, after filter, select points num = %d\n", __func__, _PB->nframe, *(_PB->nump));
	}else {
		ERROR("%s:There's no feature point detected\n", __func__);
		_PB->xyv.x = 0;
		_PB->xyv.y = 0;
		_PB->maxeig = 0;
	}
}

/**********************************************************************
// Method:  img_rotation_coord_opti_comp
// Function:  flow rotation compensation:preImg rotate follow flight 
// Parameter: 
			imgtype1 *img_int -- src img
			imgtype1 *img_out -- img after rotate
			float rotationangle -- rotate angle
			float height1 -- Ultrasonic height
// returns: void
// Date:    2018/5/24
// author:  cyj
**********************************************************************/

void img_rotation_coord_opti_comp(imgtype1 *img_int, imgtype1 *img_out, float rotationangle, float height1)
{
	int i, j, i1, j1, w,ix,iy;
	int x1, x2, y1, y2;
	float x,y,a, b;
	float iw00, iw01, iw10, iw11;
	float theta,cosx,sinx;
	float dst;
	int centerpointx = 0, centerpointy = 0;
	theta = rotationangle;//飞机逆时针旋转时，图像为顺时针旋转，旋转角度为正值，故旋转角度不需要反向处理
	//printf("rotation_angle = %10.5f\n",theta);
	float height = height1;
	float dis_x = 0;
	float dis_y = 0;

	if(g_data.ctrl[CTRL_MODE] == 1) {//零度原版旋转中心偏移值
		dis_y = 2.76 / height; 
		dis_x = 1.92 / height; 
		centerpointx = 160 - dis_x;
		centerpointy = 120 + dis_y;
	}else if(g_data.ctrl[CTRL_MODE] == 0) {//旋转中心为标定值
		dis_y = g_data.params[PARAM_CAM_TO_CENTER_DISY_PIXEL] / height;
		dis_x = g_data.params[PARAM_CAM_TO_CENTER_DISX_PIXEL] / height;
		centerpointx = 160 + dis_x + g_data.params[PARAM_CAM_CENTER_OFFSETX];
		centerpointy = 120 + dis_y + g_data.params[PARAM_CAM_CENTER_OFFSETY];
	}
	w = USB_PREVIEW_W; 
	int h = USB_PREVIEW_H;
	cosx = cos(theta);
	sinx = sin(theta);
	memcpy(img_out, img_int, w * h );
	for (i = 120 - 80;i < 120 + 80; i++)
	{
		for (j = 160-100; j < 160 + 100; j++)
		{
			//convert image coordinate to math coordinate:图像坐标中左上角为坐标原点;实际上图像中心才是坐标原点(原点上没有旋转速度)
			i1 = i - centerpointy;
			j1 = j - centerpointx;

			//i,j ??????img1????????????????? y,x
			x1 = j1;
			y1 = i1;

			/*should rotate at negative direction*/
			x = cosx*x1 + sinx*y1;
			y = -sinx*x1 + cosx*y1;

			//Get the integer part of the coordinate after rotation:决定2*2邻域的坐标位置
			ix = floor(x); iy = floor(y);
	
			//Get the decimal part of the coordinate after rotation:决定2*2邻域各个像素点的权重
			a = x - ix;
			b = y - iy;

			//The weight of four pixels in the 2*2 neighborhood 
			iw00 = (1.f - a)*(1.f - b);
			iw01 = a*(1.f - b);
			iw10 = ((1.f - a)*b);
			iw11 = 1 - iw00 - iw01 - iw10;

			//convert math coordinate to image coordinate
			//Bilinear difference in 2*2 neighborhood
			x2 = ix + centerpointx;
			y2 = iy + centerpointy;
			dst = img_int[y2*w + x2] * iw00 + img_int[y2*w + x2 + 1] * iw01 + img_int[(y2 + 1)*w + x2] * iw10 + img_int[(y2 + 1)*w + x2 + 1] * iw11;
				
			//dst = img[y2*w+x2];
			img_out[i*w + j] = dst;
		}
	}

}

/**********************************************************************
// Method:  Initial_flow
// Function:  initial flow params, malloc space for computing flow 
// Parameter: --
// returns: void
// Date:    2018/5/24
// author:  cyj
**********************************************************************/

void Initial_flow(ParamA* _PA, ParamB* _PB)
{
    S32 h,w;//,mwinsize;
    S32 i;
    U32 buftotal = 0;
    h=(_PA->h)>> 1;
    w=(_PA->w)>> 1;
	
		//分配空间A   16 bytes aligned
		//PA->pdydata=(derive_type *)aligned_alloc(16,sizeof(derive_type)*h*w);//short1 //存放每个像素点x方向的梯度Ix
		//PA->pdxdata=(derive_type *)aligned_alloc(16,sizeof(derive_type)*h*w);//short1 //存放每个像素点y方向的梯度Iy
		//PA->_cov1=(derive2_type*)aligned_alloc(16,sizeof(derive2_type)*h*w);//int1    //存放每个像素点的Ix2
		//PA->_cov2=(derive2_type*)aligned_alloc(16,sizeof(derive2_type)*h*w);//S32 1   //存放每个像素点的Iy2
		//PA->_cov3=(derive2_type*)aligned_alloc(16,sizeof(derive2_type)*h*w);//S32 1   //存放每个像素点的IxIy
		//PB->tailorprevImg = (imgtype1*)aligned_alloc(16,sizeof(imgtype1)*(PA->w*PA->h));
        //PB->tailornextImg = (imgtype1*)aligned_alloc(16,sizeof(imgtype1)*(PA->w*PA->h));

        //buftotal += sizeof(derive_type)*h*w*5;
        //buftotal += sizeof(imgtype1)*h*w*2;

        //PA->cov1=(derive2_type*)aligned_alloc(16,sizeof(derive2_type)*h*w);  //每个像素点在3*3窗口内的sum(Ix2)
		//PA->cov2=(derive2_type*)aligned_alloc(16,sizeof(derive2_type)*h*w);  //每个像素点在3*3窗口内的sum(Iy2)
		//PA->cov3=(derive2_type*)aligned_alloc(16,sizeof(derive2_type)*h*w);  //每个像素点在3*3窗口内的sum(IxIy)
        _PA->mineig=(S32*)aligned_alloc(16,sizeof(S32)*h*w);     //存放每个像素的灰度变换函数矩阵M的最小特征值
		//PA->mineigout = (S32*)aligned_alloc(16,sizeof(S32)*h*w);//存放特征值中3*3局域极大值结果
		//PA->posx=(S32*)aligned_alloc(16,sizeof(S32)*h*w);       //存放局部极大值坐标x
		//PA->posy=(S32*)aligned_alloc(16,sizeof(S32)*h*w);       //存放局部极大值坐标y
		//PA->mineig2 = (S32*)aligned_alloc(16,sizeof(S32)*h*w); //存放膨胀结果

        buftotal += sizeof(S32)*h*w;
		
		//_PA->buf1_x=(imgtype2*)aligned_alloc(16,sizeof(imgtype2)*(PD_SZ*w));
        //_PA->buf1_y =(imgtype2*)aligned_alloc(16,sizeof(imgtype2)*(PD_SZ*w));

        //buftotal += sizeof(imgtype2)*(PD_SZ*w)*2;

        _PA->hist=(int*)aligned_alloc(16,256*sizeof(S32));//统计图像灰度直方图，256 bins, 存放每个灰度等级的像素数，用于自动曝光
        buftotal += sizeof(S32)*(256);

		//分配空间B
		_PB->numf=0;
		_PB->nump=&(_PB->numf);//(S32*)malloc(sizeof(S32));
		_PB->pv=&_PB->xyv;
		//PB->goodnum=0;
		//PB->goodresult=1;
		_PB->nframe=0;
        //PB->m_bspeed = 0;
		//PB->m_setflag=1;//表示刚从另一模式进入
		//PB->m_cutpreimg = 0;
		
		_PB->status = (S32*)aligned_alloc(16,sizeof(S32)*maxnum);//特征点状态, 1 -- 特征点能正确跟踪        ,      0 -- 特征点未能正确跟踪或跟踪出错
		_PB->posi=(S32*)aligned_alloc(16,maxnum*sizeof(S32)*2);
		_PB->prevPts=(float*)aligned_alloc(16,maxnum*sizeof(float)*2);//前一帧中特征点的坐标位置, 2*i 存放x, 2*i+1存放y
		_PB->nextPts=(float*)aligned_alloc(16,maxnum*sizeof(float)*2);//前一帧中特征点的坐标位置, 2*i 存放x, 2*i+1存放y

		_PB->sumnum = (float*)aligned_alloc(16,maxnum*sizeof(float));//计算匹配相似度时,记录当前特征点的累计得分,超过一定阈值的筛选为良好特征点

		//_PB->spaceFlag=0;//空间分配标识 置零

        buftotal += sizeof(S32)*maxnum * 3;
        //buftotal += sizeof(float)*8;
        buftotal += maxnum*sizeof(float) * 5;
		
		_PB->v=0;

		for(i=255;i>-1;i--)
		{
			_PA->hist[i]=i;
		}

       // TRACK("static buffer total size: %d bytes\n",buftotal);
	   
	   printf("ok ------ 4\n");

}

/**********************************************************************
// Method:  XFreeSpace
// Function:  free space
// Parameter: --
// returns: void
// Date:    2018/5/24
// author:  cyj
**********************************************************************/
void XFreeSpace(ParamA* _PA,ParamB* _PB)
{
	XFREE(_PA->mineig);
    XFREE(_PA->hist);
	XFREE(_PB->status);
	XFREE(_PB->prevPts);
	XFREE(_PB->nextPts);
	XFREE(_PB->posi);
    XFREE(_PB->sumnum);
}

