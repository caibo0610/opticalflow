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




//��Ҫ���ĵĲ���
typedef struct LKP_Param
{
    int minDist;// ��������֮�����С����
    int maxCorners;// �������������
    int minCorners;
    int maxcount;
    double th;//0.1*���ֵ��Ϊ��ֵ
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
    int *mineigout;// = (int*)malloc(sizeof(int)*h*w);//��ž��򼫴�ֵ���
    int *posx;//=(int*)malloc(sizeof(int)*h*w);   //��žֲ�����ֵ����
    int *posy;//=(int*)malloc(sizeof(int)*h*w);
    int* mineig2;//������ͽ��
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

    int gray_low;//?��?��?����?��????T;
    int gray_high;//?��?��?����?��?��??T;
    int pixnum_gray_high;//?��?����??T?a??��y;
    int pixnum_gray_low;//?��?��???T?a??��y;

    int ci;
    int cj;//keypoint

} ParamA;//�㷨���в���

typedef struct LKP_Space
{
    imgtype1* prevImg;//ǰһ֡ͼƬ
    imgtype1* nextImg;//��ǰͼƬ
    imgtype1* tailorprevImg;//ǰһ֡ͼƬ
    imgtype1* tailornextImg;//��ǰͼƬ
    imgtype1 * Pyrmd;//�������ڶ���ռ��׵�ַ
    imgtype1 * Pyrmd1;//�������ڶ���ռ��׵�ַ

    imgtype1* _J;
    imgtype1* _I;
    derive_type* _derivI;
    imgtype2 * trow1;
    imgtype2 * trow0;
    derive_type* Dprev;

    double*	windspace;//=(double*)malloc(s2*2*sizeof(double));
    float* prevPts;//��ǰ֡�е�λ��
    float* nextPts;//��һ֡�е�λ��

	float* rotePts;//Ϊ��תʱ��������λ��
	float rote_angle_base;
		
    float* sumnum;//����
    int * status;//������״̬����
    int * fpts;//�������ʼλ��
    int * nump;//�������������

    int * posi;//��������������λ�����
    int numf0;
    int numf;//���������

    int spaceFlag;//�ռ��ʼ�����
    int flagPyrmd;//�������״γ�ʼ�����
    int m_bsetparam;//�Ƿ�ı������������ 1�ı�0���ı�
    imgtype1* pyr_Ind0[8];//ָ������������ָ�� ǰһ֡
    imgtype1* pyr_Ind1[8];//ָ������������ָ�� ��ǰ֡

    int m_setflag;
    U32 m_cutpreimg;
    int m_bspeed;
    float *pvs;
    float v;//֡���ƶ�����
    int goodresult;//0Ϊ������ 1Ϊ����
    int goodnum;//=0;//ͳ����ȷ�������
    //int nframe;//=1;//֡������
    unsigned int nframe;
    float percent;
    int maxeig;
    
    float xgd;
    float sumwind1;
    float sumwind2;
    Point2f_z *pv;
    Point2f_z xyv;//�ƶ�����
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
}ParamB;//�㷨���пռ�ָ��

//************************************
// Method:    GetGoodfeature
// Date:      2015/7/27
// Returns:   void
// Parameter: ParamA * PA
// Parameter: ParamB * PB
// Function:  ������õ�������
//************************************
void GetGoodfeature(ParamA* PA,ParamB* PB);

//************************************
// Method:    FunDerivScharrDxy
// Date:      2015/7/27
// Returns:   void
// Parameter: imgtype1 * pdata  �������
// Parameter: derive_type * pdxydata �˲����
// Parameter: int w
// Parameter: int h
// Parameter: imgtype2 * trow0
// Parameter: imgtype2 * trow1
// Function:  ���������pdata����Scharr�˲�
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
// Function:  x�����˲�
//************************************
 void FunDy(imgtype1* pdata,derive_type* pdydata, int w, int h,imgtype2* buf);//y�����˲�

//************************************
// Method:    FunDx
// Date:      2015/7/27
// Returns:   void
// Parameter: imgtype1 * pdata
// Parameter: derive_type * pdxdata
// Parameter: int w
// Parameter: int h
// Function:  y�����˲�
//************************************
 void FunDx(imgtype1* pdata,derive_type* pdxdata, int w, int h,imgtype2* buf);//x�����˲�
 void FunDxy(imgtype1* _src, ParamA *PA);

//************************************
// Method:    Boxf
// Date:      2015/7/27
// Returns:   void
// Parameter: derive2_type * _src
// Parameter: derive2_type * _dst
// Parameter: int width
// Parameter: int height
// Function:  boxfilter�˲�
//************************************
// void Boxf(derive2_type* _src ,derive2_type* _dst,int width,int height,derive2_type* buf,ParamA *PA);// box�˲�
void Boxf(derive2_type* _src , derive2_type* _dst, ParamA *PA);


//************************************
// Method:    Interp
// Date:      2015/7/27
// Returns:   int
// Parameter: int x
// Parameter: int width
// Function:  �������߽�Ĳ���
//************************************
int  Interp(int x,int width);//ʹ�öԳ����صķ��� �������߽�ĵ�


//************************************
// Method:    KeepLocalMax
// Returns:   void
// Parameter: int * mineig
// Parameter: int * posx   //����ֲ�����ֵ������
// Parameter: int * posy
// Parameter: int * mineigout  ����ֲ�����ֵ
// Parameter: ParamA * PA
// Parameter: ParamB * PB
// Function:  �Ը���������ֵ������д����õ�����ֲ�����ֵ������
//            ���ұ���������posx,posy
//************************************
void KeepLocalMax(int* mineig, int* posx, int* posy,int *mineigout,ParamA* PA,ParamB* PB);

//************************************
// Method:    DelMin
// Returns:   void
// Parameter: int * mineig //����ֵ����
// Parameter: int maxeig   //�������ֵ
// Parameter: ParamA * PA
// Function:  ȥ��С�ڸ�����ֵ������ֵ������Ϊ0
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
// Function:  �ų�����С�ڲ�����������
//************************************
void MinDistSelect(int* mineig,int total, int* posx, int*posy, ParamA* PA, ParamB* PB); 

//************************************
// Method:    Sort
// Returns:   void
// Parameter: int * dst
// Parameter: int total
// Parameter: int * px
// Parameter: int * py
// Function:  �Ը���������������򣬲�����Ԫ�ص�����
//************************************
void Sort(int* dst,int total,int* px,int* py);//��������, ͬʱ����2ά����

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
// Function:  ����ͬʱ����x,y����
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
// Function:  ���������ػ�׼Ԫ��λ��
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
// Function:  ��������
//************************************
void QuickSort(int a[], int low, int high, int* posx, int* posy);

//************************************
// Method:    Sort2
// Date:      2015/7/27
// Returns:   void
// Parameter: float * dst
// Parameter: int total
// Parameter: int * px
// Function:  ��������ͬʱ����1ά����
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
// Function:  ������ͬʱ����1ά����
//************************************
void Swap2(float* a, float* b,int low,int high,int* posx);

//************************************
// Method:    Partition2
// Date:      2015/7/27
// Returns:   int ����Ԫ�������λ��
// Parameter: float * a
// Parameter: int low
// Parameter: int high
// Parameter: int * posx
// Function:  �ֿ飬ͬʱ���ػ���Ԫ�������λ��
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
// Function:  �������� ͬʱ����1ά����
//************************************
void QuickSort2(float* a, int low, int high,int* posx);

//************************************
// Method:    BuildPyrmd
// Returns:   void
// Parameter: ParamA * PA
// Parameter: ParamB * PB
// Function:  ����ͼ��Ľ���������ʹ��ָ��ָ����������
//            ����ָ������������ָ������PB->pind
//************************************
void BuildPyrmd(ParamA* PA, ParamB* PB);


//************************************
// Method:    LKPyramid
// Date:      2015/7/27
// Returns:   void
// Parameter: ParamA * PA
// Parameter: ParamB * PB
// Function:  ����LKP����,����Ŀ��λ��PB->nextPts
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
// Function:  ��pdataָ������ݼ��������������pdata2ָ��ĵ�ַ
//************************************
 void SubImg(imgtype1* pdata,imgtype1* pdata2,int src_width,int src_height,imgtype2* buf);//������ ���� [1 4 6 4 1]
void SubImg2(imgtype1* pdata,imgtype1* pdata2,int src_width,int src_height,imgtype2* buf);//������ ���� [1 1]
//void CutImg(imgtype1* src,imgtype1* dst,int w,int h);
//************************************
// Method:    GetBorder
// Date:      2015/7/27
// Returns:   int   ���ش���������
// Parameter: int i     ����
// Parameter: int height_pyr  Դͼ��ߴ�
// Parameter: int h1 ��չ�Ĵ��ڳߴ�
// Function:  ������չ��չ���� �����߽粿�ֵ�����
//************************************
int  GetBorder(int i,int height,int h1);

//************************************
// Method:    ExtentCopy
// Date:      2015/7/27
// Returns:   void
// Parameter: derive_type * _derivI ��չ�����
// Parameter: imgtype1 * _I  ��չ�����
// Parameter: imgtype1 * _J  ��չ�����
// Parameter: derive_type * prevDeriv ��չǰ
// Parameter: imgtype1 * I           ��չǰ
// Parameter: imgtype1 * J           ��չǰ 
// Parameter: int width_pyr
// Parameter: int height_pyr
// Parameter: int winsize
// Function:  ��ԭ���������չ�õ���չ���,_I,_J,_derivI;
//************************************
// void ExtentCopy(derive_type* _derivI,imgtype1* _I, imgtype1* _J, derive_type* prevDeriv,imgtype1* I,imgtype1* J,int width_pyr, int height_pyr,int winsize);
void ExtentCopy(ParamA *PA, ParamB *PB, imgtype1* I, imgtype1* J, int width_pyr, int height_pyr);

//************************************
// Method:    LK
// Date:      2015/7/27
// Returns:   void
 // Parameter: imgtype1 * _I I��չ���ͼ��
 // Parameter: imgtype1 * _J J��չ���ͼ��
// Parameter: derive_type * _derivI DerivI��չ���ͼ��
// Parameter: int width   ���
// Parameter: int height  �߶�
// Parameter: int level   ��������level��
// Parameter: ParamA * PA  �㷨����
// Parameter: ParamB * PB  �ռ�ָ��
// Function:  ѭ����������Ŀ���λ��
//************************************
void LK(imgtype1* _I, imgtype1* _J, derive_type* _derivI, int width, int height, int level, ParamA* PA, ParamB* PB);



//************************************
// Method:    GetRightPoint
// Date:      2015/7/27
// Returns:   float
// Parameter: ParamB * PB  �ռ�ָ�� ���������������
// Function:  ɸѡ���������㣬��������ȡǰupth������ƽ��
//************************************
float GetRightPoint(ParamB* PB);///ɸѡ����������
float GetRightPoint3(ParamB* PB);///ɸѡ����������2

//************************************
// Method:    ChangeParam
// Date:      2015/7/27
// Returns:   void
// Parameter: ParamA * PA
// Parameter: ParamB * PB
// Function:  �ı����
//************************************
void  SetParam(ParamA* PA,ParamB* PB);


//************************************
// Method:    FreeSpace
// Date:      2015/7/27
// Returns:   void
// Parameter: ParamB * PB
// Function:  �ͷſռ�
//************************************
 void  FreeSpace(ParamA* PA,ParamB* PB);

//************************************
// Method:    SwapPyr
// Date:      2015/7/27
// Returns:   void
// Parameter: ParamB * PB
// Function:  ����������ָ��
//************************************
void  SwapPyr(ParamB* PB);//imgtype1* pyr,imgtype1* pyr1,imgtype1* *pyr_Ind0,imgtype1* *pyr_Ind1);

//************************************
// Method:    UseLKP
// Date:      2015/7/27
// Returns:   void
// Parameter: ParamA *
// Parameter: ParamB *
// Function:  ʹ��LKP�㷨
//************************************
void  UseLKP(ParamA *, ParamB *);

//************************************
// Method:    UpdatePts
// Date:      2015/7/27
// Returns:   void
// Parameter: ParamB *
// Function:  ��������������prevPts,status,nump,goodresult
//************************************
void  UpdatePts(ParamB *);//

//************************************
// Method:    Init
// Date:      2015/7/27
// Returns:   void
// Parameter: ParamA * PA
// Parameter: ParamB * PB
// Function:  ��ʼ�����ݿռ�
//************************************
void Initial(ParamA* PA, ParamB* PB);
//************************************
// fun:       CutImg
// Date:      2015/7/27
// Returns:   void
// Parameter: ԭͼ  src
// Parameter: Ŀ��ͼdst
// Parameter: ԭͼ��w
// Parameter: ԭͼ��h
// Function:  �ü��м�1/4ͼƬ
//************************************
void CutImg(imgtype1* src,imgtype1* dst,int w,int h);
//************************************
// fun:       getzxd
// Date:      
// Returns:   void
// Parameter: ParamA*PA
// Parameter: ParamB* PB
// Function:  �������Ŷ�
//************************************
void getzxd(ParamA*PA,ParamB*PB);
//************************************
// fun:    update_camera_fov
// Date:      
// Returns:   void
// Parameter: �ӳ���field_angle;
// Parameter: ����focal_len;
// Parameter: ParamB* PB
// Function:  ����������ز������㷨��
//************************************
void update_camera_fov(int field_angle,float focal_len,int pic_size_flg,ParamB* PB);
//************************************
// fun:    baoguang
// Date:      2016/7/17
// Returns:   void
// Parameter: �ع�ֵ:exposure_value;
// Parameter: ���ڲ���:exposure_set_flag;
// Parameter: �ع����ֵ:exposure_max
// Function:  ����ͼƬ�ع�
//************************************
void baoguang(S32 *exposure_value,S32 *exposure_set_flag,S32 exposure_max,ParamA* PA,ParamB* PB);

//************************************
// fun:    img_rotation_coord_opti
// Date:      2016/7/17
// Returns:   void
// Parameter: ����ͼƬ:img_int;
// Parameter: ���ͼƬ:img_out;
// Parameter: ��ת�Ƕ�:rotationangle
// Function:  ��ͼƬ������ת
//************************************
void img_rotation_coord_opti(imgtype1 *img_int, imgtype1 *img_out, float rotationangle,float height1);

//************************************
// fun:    update_confidence_level
// Date:      2016/3/27
// Returns:   int
// Parameter: �����ٶ�:compx compy;�߶�:height
// Parameter: ��ʼ�ж��쳣�ٶ�����:dis_th;
// Parameter: ��������:variance_th*height
// Parameter: �ٶȲ�������:fluct_th*height
// Parameter: ParamB * PB
// Function:  �жϹ����ٶ��Ƿ����
//************************************
int update_confidence_level(float compx,float compy,float height,
										 float dis_th,float variance_th,float fluct_th,ParamB* PB);

#ifdef __cplusplus
}
#endif

#endif

