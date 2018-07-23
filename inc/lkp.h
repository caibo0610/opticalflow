/********************************************************************
	created:	27:7:2015
	filename: 	ggf
	file ext:	h
	author:  	Fu Chongyang
	purpose:  
*********************************************************************/
#ifndef C_GGF_H
#define C_GGF_H

#include "type.h"
#include <stdint.h>

#define WIDTH  640
#define maxnum 128
//#define epsilon  0.000001
#define  PD_SZ  3
#define  PD_SZ5 5


//threshold definition
#define GGP_THRESHOLD 0.01f
#define GRP_POINT_INTERVAL 0.01f
#define GRP_POINT_SCORE  0.9f // 0.9f
#define FCOR_WINDOW_X 10
#define FCOR_WINDOW_Y 5
#define FCOR_THRESHOLD 0.2f  //more bigger more strict
#define DIS_COEFFCIENT 0.333333f  //assume the focus is 3.6mm, pixel size is 6um*6um for 640*480 pixels,unit cm
#define RAD_RECORD_MAX  12


/******************************************************************************************************************
currently, the pixel solution is 320*240, so the pixel size is 12um*12um for each pixel
 pixel * 12(um)      3.6(mm)              pixel * h * 12(um)      pixel * h(m) * 12      1200
---------------- = -----------   ==> d = -------------------- = --------------------- = ------ * pixel * h (cm), so
      d               h(m)                      3.6(mm)               3600               3600

 the coeffient is 0.333333
*******************************************************************************************************************/

#define VELOCITY_FILTER 0.3f

#define USE_NEON


#ifdef __cplusplus
extern "C" {
#endif

typedef	int		derive2_type;
typedef	unsigned char	imgtype1;
typedef	short	imgtype2;

typedef	short	derive_type;


typedef struct Point2f_z
{
    float x;
    float y;
} Point2f_z;

typedef struct para_from_fc
{
    float radvx;
    float radvy;
    float radvz;
    uint64_t timestamps;
    uint64_t camerats;
}para_from_fc;



typedef struct Point2i_z
{
    int x;
    int y;
} Point2i_z;

typedef struct Size_z
{
    int width;
    int height;
} Size_z;




//需要更改的参数
typedef struct LKP_Param
{
    int minDist;// 两特征点之间的最小距离
    int maxCorners;// 检测的最大特征数
    int minCorners;
    int maxcount;
    double th;//0.1*最大值作为阈值
    int w;    
    int h;
    int winsize;
    int maxlevel;
    derive_type  *pdydata;//=(derive_type *)malloc(sizeof(derive_type)*h*w);
    derive_type *pdxdata;//=(derive_type *)malloc(sizeof(derive_type)*h*w);
    derive2_type *_cov1;//=(derive2_type*)malloc(sizeof(derive2_type)*h*w);
    derive2_type *_cov2;//=(derive2_type*)malloc(sizeof(derive2_type)*h*w);
    derive2_type *_cov3;//=(derive2_type*)malloc(sizeof(derive2_type)*h*w);

    derive2_type *cov1;//=(derive2_type*)malloc(sizeof(derive2_type)*h*w);
    derive2_type *cov2;//=(derive2_type*)malloc(sizeof(derive2_type)*h*w);
    derive2_type *cov3;//=(derive2_type*)malloc(sizeof(derive2_type)*h*w);

    int *mineig;//=(int*)malloc(sizeof(int)*h*w);
    int *mineigout;// = (int*)malloc(sizeof(int)*h*w);//存放局域极大值结果
    int *posx;//=(int*)malloc(sizeof(int)*h*w);   //存放局部极大值坐标
    int *posy;//=(int*)malloc(sizeof(int)*h*w);
    int* mineig2;//存放膨胀结果
    int *mgrid;
    int *mgridsize;// 
    int *hIndex;
    int *wIndex;

    imgtype2 *buf1_x;//fundx fundy
    imgtype2 *buf1_y;
    derive2_type *buf2;//box
    imgtype2 *buf3;//subimg


    imgtype2* IWinBuf;//
    derive_type* derivIWinBuf;//
    char ip[20];
    int sock;
    unsigned long logtv;

    int light;
    int* hist;

    int gray_low;//?ò?è?±·?í????T;
    int gray_high;//?ò?è?±·?í?é??T;
    int pixnum_gray_high;//?ò?èé??T?a??êy;
    int pixnum_gray_low;//?ò?è???T?a??êy;

    int ci;
    int cj;//keypoint

} ParamA;//算法运行参数

typedef struct LKP_Space
{
    imgtype1* prevImg;//前一帧图片
    imgtype1* nextImg;//当前图片
    imgtype1* tailorprevImg;//前一帧图片
    imgtype1* tailornextImg;//当前图片
    imgtype1 * Pyrmd;//金字塔第二层空间首地址
    imgtype1 * Pyrmd1;//金字塔第二层空间首地址

    imgtype1* _J;
    imgtype1* _I;
    derive_type* _derivI;
    imgtype2 * trow1;
    imgtype2 * trow0;
    derive_type* Dprev;

    double*	windspace;//=(double*)malloc(s2*2*sizeof(double));
    float* prevPts;//当前帧中的位置
    float* nextPts;//下一帧中的位置

	float* rotePts;//为旋转时的特征点位置
	float rote_angle_base;
		
    float* sumnum;//积分
    int * status;//特征点状态保存
    int * fpts;//特征点初始位置
    int * nump;//查找特征点个数

    int * posi;//特征点积分排序的位置序号
    int numf0;
    int numf;//特征点个数

    int spaceFlag;//空间初始化标记
    int flagPyrmd;//金字塔首次初始化标记
    int m_bsetparam;//是否改变金字塔层数？ 1改变0不改变
    imgtype1* pyr_Ind0[8];//指向各层金字塔的指针 前一帧
    imgtype1* pyr_Ind1[8];//指向各层金字塔的指针 当前帧

    int m_setflag;
    U32 m_cutpreimg;
    int m_bspeed;
    float *pvs;
    float v;//帧中移动距离
    int goodresult;//0为不可信 1为可信
    int goodnum;//=0;//统计正确结果数量
    //int nframe;//=1;//帧数计数
    unsigned int nframe;
    float percent;
    int maxeig;
    
    float xgd;
    float sumwind1;
    float sumwind2;
    Point2f_z *pv;
    Point2f_z xyv;//移动距离
    Point2f_z v_opti_pre;//for linear filter from optical flow
    Point2f_z v_gyro_pre;// for linear filter from gyroscope sensor
    Point2f_z v_comp;// calibration result;
    float IMUtimestamp1;       //get this value when receive picture data from USB camera
    float height1;             //get this value when receive picture data from USB camera
    Point2f_z IMUpitchyaw1;    //get this value when receive picture data from USB camera
    Point2f_z NSvxy1;          //get this value when receive picture data from USB camera
    float dis_per_pixel_per_height;
    para_from_fc fcpara[RAD_RECORD_MAX];
    float h_record;
    Point2f_z v_record;
    Point2f_z rad_record;
    Point2f_z com_record;
    float rotation_angle;
    S32   rotflag;
    unsigned long timediff;
}ParamB;//算法运行空间指针

//************************************
// Method:    GetGoodfeature
// Date:      2015/7/27
// Returns:   void
// Parameter: ParamA * PA
// Parameter: ParamB * PB
// Function:  查找最好的特征点
//************************************
void GetGoodfeature(ParamA* PA,ParamB* PB);

//************************************
// Method:    FunDerivScharrDxy
// Date:      2015/7/27
// Returns:   void
// Parameter: imgtype1 * pdata  输入矩阵
// Parameter: derive_type * pdxydata 滤波结果
// Parameter: int w
// Parameter: int h
// Parameter: imgtype2 * trow0
// Parameter: imgtype2 * trow1
// Function:  对输入矩阵pdata进行Scharr滤波
//************************************
 void FunScharrDxy(imgtype1* pdata,derive_type* pdxydata, int w, int h, imgtype2* trow0 ,imgtype2 * trow1);

//************************************
// Method:    FunDy
// Date:      2015/7/27
// Returns:   void
// Parameter: imgtype1 * pdata
// Parameter: derive_type * pdydata
// Parameter: int w
// Parameter: int h
// Function:  x方向滤波
//************************************
 void FunDy(imgtype1* pdata,derive_type* pdydata, int w, int h,imgtype2* buf);//y方向滤波

//************************************
// Method:    FunDx
// Date:      2015/7/27
// Returns:   void
// Parameter: imgtype1 * pdata
// Parameter: derive_type * pdxdata
// Parameter: int w
// Parameter: int h
// Function:  y方向滤波
//************************************
 void FunDx(imgtype1* pdata,derive_type* pdxdata, int w, int h,imgtype2* buf);//x方向滤波
 void FunDxy(imgtype1* _src, ParamA *PA);

//************************************
// Method:    Boxf
// Date:      2015/7/27
// Returns:   void
// Parameter: derive2_type * _src
// Parameter: derive2_type * _dst
// Parameter: int width
// Parameter: int height
// Function:  boxfilter滤波
//************************************
// void Boxf(derive2_type* _src ,derive2_type* _dst,int width,int height,derive2_type* buf,ParamA *PA);// box滤波
void Boxf(derive2_type* _src , derive2_type* _dst, ParamA *PA);


//************************************
// Method:    Interp
// Date:      2015/7/27
// Returns:   int
// Parameter: int x
// Parameter: int width
// Function:  处理超出边界的部分
//************************************
int  Interp(int x,int width);//使用对称延拓的方法 处理超出边界的点


//************************************
// Method:    KeepLocalMax
// Returns:   void
// Parameter: int * mineig
// Parameter: int * posx   //保存局部极大值的坐标
// Parameter: int * posy
// Parameter: int * mineigout  输出局部极大值
// Parameter: ParamA * PA
// Parameter: ParamB * PB
// Function:  对给出的特征值矩阵进行处理，得到保存局部极大值的数组
//            并且保存了坐标posx,posy
//************************************
void KeepLocalMax(int* mineig, int* posx, int* posy,int *mineigout,ParamA* PA,ParamB* PB);

//************************************
// Method:    DelMin
// Returns:   void
// Parameter: int * mineig //特征值矩阵
// Parameter: int maxeig   //最大特征值
// Parameter: ParamA * PA
// Function:  去除小于给定阈值的特征值，并设为0
//************************************
 void DelMin(int* mineig, int maxeig, ParamA* PA);

//************************************
// Method:    MinDistSelect
// Returns:   void
// Parameter: int * mineig
// Parameter: int total
// Parameter: int * posx
// Parameter: int * posy
// Parameter: ParamA * PA
// Parameter: ParamB * PB
// Function:  排除距离小于参数的特征点
//************************************
void MinDistSelect(int* mineig,int total, int* posx, int*posy, ParamA* PA, ParamB* PB); 

//************************************
// Method:    Sort
// Returns:   void
// Parameter: int * dst
// Parameter: int total
// Parameter: int * px
// Parameter: int * py
// Function:  对给出的数组进行排序，并交换元素的坐标
//************************************
void Sort(int* dst,int total,int* px,int* py);//快速排序, 同时交换2维坐标

//************************************
// Method:    Swap
// Date:      2015/7/27
// Returns:   void
// Parameter: int * a
// Parameter: int * b
// Parameter: int low
// Parameter: int high
// Parameter: int * posx
// Parameter: int * posy
// Function:  交换同时交换x,y坐标
//************************************
void Swap(int* a, int* b,int low,int high,int* posx,int* posy);

//************************************
// Method:    Partition
// Date:      2015/7/27
// Returns:   int
// Parameter: int a[]
// Parameter: int low
// Parameter: int high
// Parameter: int * posx
// Parameter: int * posy
// Function:  分区，返回基准元素位置
//************************************
int  Partition(int a[], int low, int high,int* posx,int* posy);


//************************************
// Method:    QuickSort
// Date:      2015/7/27
// Returns:   void
// Parameter: int a[]
// Parameter: int low
// Parameter: int high
// Parameter: int * posx
// Parameter: int * posy
// Function:  快速排序
//************************************
void QuickSort(int a[], int low, int high, int* posx, int* posy);

//************************************
// Method:    Sort2
// Date:      2015/7/27
// Returns:   void
// Parameter: float * dst
// Parameter: int total
// Parameter: int * px
// Function:  快速排序，同时交换1维坐标
//************************************
void Sort2(float* dst,int total,int* px);

//************************************
// Method:    Swap2
// Date:      2015/7/27
// Returns:   void
// Parameter: float * a
// Parameter: float * b
// Parameter: int low
// Parameter: int high
// Parameter: int * posx
// Function:  交换，同时交换1维坐标
//************************************
void Swap2(float* a, float* b,int low,int high,int* posx);

//************************************
// Method:    Partition2
// Date:      2015/7/27
// Returns:   int 基本元素排序后位置
// Parameter: float * a
// Parameter: int low
// Parameter: int high
// Parameter: int * posx
// Function:  分块，同时返回基本元素排序后位置
//************************************
int  Partition2(float* a, int low, int high,int* posx);

//************************************
// Method:    QuickSort2
// Date:      2015/7/27
// Returns:   void
// Parameter: float * a
// Parameter: int low
// Parameter: int high
// Parameter: int * posx
// Function:  快速排序 同时交换1维坐标
//************************************
void QuickSort2(float* a, int low, int high,int* posx);

//************************************
// Method:    BuildPyrmd
// Returns:   void
// Parameter: ParamA * PA
// Parameter: ParamB * PB
// Function:  建立图像的金字塔，并使用指针指向各层金字塔
//            返回指向各层金字塔的指针数组PB->pind
//************************************
void BuildPyrmd(ParamA* PA, ParamB* PB);


//************************************
// Method:    LKPyramid
// Date:      2015/7/27
// Returns:   void
// Parameter: ParamA * PA
// Parameter: ParamB * PB
// Function:  调用LKP方法,计算目标位置PB->nextPts
//************************************
void LKPyramid(ParamA* PA, ParamB* PB);

//************************************
// Method:    SubImg
// Date:      2015/7/27
// Returns:   void
// Parameter: imgtype1 * pdata
// Parameter: imgtype1 * pdata2
// Parameter: int src_width
// Parameter: int src_height
// Function:  对pdata指向的数据减采样，并输出到pdata2指向的地址
//************************************
 void SubImg(imgtype1* pdata,imgtype1* pdata2,int src_width,int src_height,imgtype2* buf);//减采样 算子 [1 4 6 4 1]
void SubImg2(imgtype1* pdata,imgtype1* pdata2,int src_width,int src_height,imgtype2* buf);//减采样 算子 [1 1]
//void CutImg(imgtype1* src,imgtype1* dst,int w,int h);
//************************************
// Method:    GetBorder
// Date:      2015/7/27
// Returns:   int   返回处理后的坐标
// Parameter: int i     坐标
// Parameter: int height_pyr  源图像尺寸
// Parameter: int h1 扩展的窗口尺寸
// Function:  处理扩展扩展矩阵 超出边界部分的坐标
//************************************
int  GetBorder(int i,int height,int h1);

//************************************
// Method:    ExtentCopy
// Date:      2015/7/27
// Returns:   void
// Parameter: derive_type * _derivI 扩展后矩阵
// Parameter: imgtype1 * _I  扩展后矩阵
// Parameter: imgtype1 * _J  扩展后矩阵
// Parameter: derive_type * prevDeriv 扩展前
// Parameter: imgtype1 * I           扩展前
// Parameter: imgtype1 * J           扩展前 
// Parameter: int width_pyr
// Parameter: int height_pyr
// Parameter: int winsize
// Function:  对原矩阵进行扩展得到扩展后的,_I,_J,_derivI;
//************************************
// void ExtentCopy(derive_type* _derivI,imgtype1* _I, imgtype1* _J, derive_type* prevDeriv,imgtype1* I,imgtype1* J,int width_pyr, int height_pyr,int winsize);
void ExtentCopy(ParamA *PA, ParamB *PB, imgtype1* I, imgtype1* J, int width_pyr, int height_pyr);

//************************************
// Method:    LK
// Date:      2015/7/27
// Returns:   void
 // Parameter: imgtype1 * _I I扩展后的图像
 // Parameter: imgtype1 * _J J扩展后的图像
// Parameter: derive_type * _derivI DerivI扩展后的图像
// Parameter: int width   宽度
// Parameter: int height  高度
// Parameter: int level   金字塔第level层
// Parameter: ParamA * PA  算法参数
// Parameter: ParamB * PB  空间指针
// Function:  循环迭代计算目标点位置
//************************************
void LK(imgtype1* _I, imgtype1* _J, derive_type* _derivI, int width, int height, int level, ParamA* PA, ParamB* PB);



//************************************
// Method:    GetRightPoint
// Date:      2015/7/27
// Returns:   float
// Parameter: ParamB * PB  空间指针 包括特征点的坐标
// Function:  筛选已有特征点，积分排序，取前upth个点求平均
//************************************
float GetRightPoint(ParamB* PB);///筛选已有特征点
float GetRightPoint3(ParamB* PB);///筛选已有特征点2

//************************************
// Method:    ChangeParam
// Date:      2015/7/27
// Returns:   void
// Parameter: ParamA * PA
// Parameter: ParamB * PB
// Function:  改变参数
//************************************
void  SetParam(ParamA* PA,ParamB* PB);


//************************************
// Method:    FreeSpace
// Date:      2015/7/27
// Returns:   void
// Parameter: ParamB * PB
// Function:  释放空间
//************************************
 void  FreeSpace(ParamA* PA,ParamB* PB);

//************************************
// Method:    SwapPyr
// Date:      2015/7/27
// Returns:   void
// Parameter: ParamB * PB
// Function:  交换金字塔指针
//************************************
void  SwapPyr(ParamB* PB);//imgtype1* pyr,imgtype1* pyr1,imgtype1* *pyr_Ind0,imgtype1* *pyr_Ind1);

//************************************
// Method:    UseLKP
// Date:      2015/7/27
// Returns:   void
// Parameter: ParamA *
// Parameter: ParamB *
// Function:  使用LKP算法
//************************************
void  UseLKP(ParamA *, ParamB *);

//************************************
// Method:    UpdatePts
// Date:      2015/7/27
// Returns:   void
// Parameter: ParamB *
// Function:  更新特征点坐标prevPts,status,nump,goodresult
//************************************
void  UpdatePts(ParamB *);//

//************************************
// Method:    Init
// Date:      2015/7/27
// Returns:   void
// Parameter: ParamA * PA
// Parameter: ParamB * PB
// Function:  初始化数据空间
//************************************
void Initial(ParamA* PA, ParamB* PB);
//************************************
// fun:       CutImg
// Date:      2015/7/27
// Returns:   void
// Parameter: 原图  src
// Parameter: 目的图dst
// Parameter: 原图宽w
// Parameter: 原图高h
// Function:  裁剪中间1/4图片
//************************************
void CutImg(imgtype1* src,imgtype1* dst,int w,int h);
//************************************
// fun:       getzxd
// Date:      
// Returns:   void
// Parameter: ParamA*PA
// Parameter: ParamB* PB
// Function:  计算置信度
//************************************
void getzxd(ParamA*PA,ParamB*PB);
//************************************
// fun:    update_camera_fov
// Date:      
// Returns:   void
// Parameter: 视场角field_angle;
// Parameter: 焦距focal_len;
// Parameter: ParamB* PB
// Function:  设置相机像素参数到算法库
//************************************
void update_camera_fov(int field_angle,float focal_len,int pic_size_flg,ParamB* PB);
//************************************
// fun:    baoguang
// Date:      2016/7/17
// Returns:   void
// Parameter: 曝光值:exposure_value;
// Parameter: 调节步长:exposure_set_flag;
// Parameter: 曝光最大值:exposure_max
// Function:  调节图片曝光
//************************************
void baoguang(S32 *exposure_value,S32 *exposure_set_flag,S32 exposure_max,ParamA* PA,ParamB* PB);

//************************************
// fun:    img_rotation_coord_opti
// Date:      2016/7/17
// Returns:   void
// Parameter: 输入图片:img_int;
// Parameter: 输出图片:img_out;
// Parameter: 旋转角度:rotationangle
// Function:  对图片进行旋转
//************************************
void img_rotation_coord_opti(imgtype1 *img_int, imgtype1 *img_out, float rotationangle,float height1);

//************************************
// fun:    update_confidence_level
// Date:      2016/3/27
// Returns:   int
// Parameter: 光流速度:compx compy;高度:height
// Parameter: 开始判断异常速度门限:dis_th;
// Parameter: 方差门限:variance_th*height
// Parameter: 速度波动门限:fluct_th*height
// Parameter: ParamB * PB
// Function:  判断光流速度是否可信
//************************************
int update_confidence_level(float compx,float compy,float height,
										 float dis_th,float variance_th,float fluct_th,ParamB* PB);

#ifdef __cplusplus
}
#endif

#endif

