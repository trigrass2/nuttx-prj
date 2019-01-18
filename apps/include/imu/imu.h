#ifndef  __IMU_H_INCLUDED

#define  __IMU_H_INCLUDED

#include <DSP_Lib/arm_math.h>
#include "imu_matrix.h"

#define GRAVITY	  			9.80665
#define DEG2RAD				0.017453293
#define RAD2DEG				57.29578
#define KNOT_M				0.5144444

////0: 无RTK信号；  1：有RTK信号，滤掉前100个点；   2：融合开始，滤掉融合开始前1000个点；  3：数据正常
//#define NO_RTK 				0
//#define RTK_HEAD			1
//#define FUSION_HEAD			2
//#define	FUSION_OK			3

typedef enum fusion_mode_e
{
	CF_MODE,
	EKF_BASE_MODE,
	EKF_GPS_MODE,
	RAW_MODE
}fusion_mode;

typedef enum Yaw_Obs_Select_e
{
	OBS_YAW_MAG,
	OBS_YAW_RTK,
	OBS_YAW_MIX
}Yaw_Obs_Select;

typedef enum fusion_print_select_e
{
	FUSION_PRINT_NO,
	FUSION_PRINT_QNB,
	FUSION_PRINT_ENB,
	FUSION_PRINT_ATT,
	FUSION_PRINT_TIME,
}fusion_print_select;

typedef enum raw_print_select_e
{
	RAW_PRINT_NO,
	RAW_PRINT_ADIS16488a,
	RAW_PRINT_HMC6343,
	RAW_PRINT_GPS,
	RAW_PRINT_TIME,
	RAW_PRINT_GYRO
}raw_print_select;

typedef struct HMC6343_Data_s
{
	Vector3f bhead;
	Vector3f decli;
	Vector3f mag;
} HMC6343_Data;


typedef struct gps_data_s
{
	int      gpsValid;
    int      latSign;
    int      lonSign;

    double lat;
    double lon;
    double alt;


	uint32_t year;
	uint32_t month;
	uint32_t day;

	uint32_t Num;

	float speed;

	uint32_t             itow;

	bool  yaw_valid;
	float yaw;
	float yaw_filter;
	float yaw_last;

	float diff_rtk_mag;
	float diff_rtk_mag_all;
	uint32_t diff_rtk_mag_N;
} gps_data;


typedef struct Imu_Data_s
{
	/*************************************************/
	//姿态相关数据
	Quaternion   Qnb;		//四元数
	Quaternion   Qnb_pre;	//四元数预估值

	EulerAng     Enb;		//欧拉角，单位：rad
	EulerAng     Enb_pre;	//欧拉角预估值
	EulerAng     Enb_obs;	//欧拉角观测值

	Matrix3x3    Cbn;		//b->n旋转矩阵
	Matrix3x3    Cbn_pre;	//b->n旋转矩阵预估值

	Matrix3x3    Cnb;		//n->b旋转矩阵
	Matrix3x3    Cbd;		//b->d旋转矩阵

	Attitude     att;		//姿态，单位：度

	float        declination;		//磁偏角，用于磁北补偿

	/*************************************************/
	//融合相关数据
	fusion_mode     fusion_mode;	//融合模式
	float        dt;				//间隔时间
	float        dtover2;			//间隔时间1/2

	uint32_t   start_raw_i;			//IMU计数，去除前start_raw_N个数
    uint32_t   start_raw_N;			//IMU计数范围
	uint32_t   start_rtk_i;			//RTK计数，去除前start_rtk_N个数
	uint32_t   start_rtk_N;			//RTK计数范围
	uint32_t   miss_rtk_yaw_i;			//RTK yaw丢失计数

	uint32_t   start_fusion_i;		//融合计数，去除前start_fusion_rtk_N/start_fusion_rtk_N个数
	uint32_t   start_fusion_mag_N;	//融合计数范围（mag）
	uint32_t   start_fusion_rtk_N;	//融合计数（RTK）
	bool       fusion_out_flag;		//融合输出状态

	Yaw_Obs_Select  obs_yaw_select;	//观测数据选择（mag或者rtk）
	Yaw_Obs_Select  last_obs_yaw_select;

	/*************************************************/
	//IMU相关数据
	//acc
	Vector3f ba;					//ba raw
	Vector3f ba_bia;				//ba零偏
	Vector3f ba_noise;				//ba噪声
	Vector3f ba_filter;				//ba滤波
	bool     ba_filter_enable;		//ba滤波使能
	float    ba_norm;				//ba模长（未滤波）

	//gyro
	Vector3f bg;					//bg raw
	Vector3f bg_bia;				//bg零偏
	Vector3f bg_noise;				//bg噪声
	Vector3f bg_filter;				//bg滤波
	bool     bg_filter_enable;		//ba滤波使能
	Vector3f bg_sum;				//bg求和
	Vector3f bg_mean;				//bg求平均
	uint32_t bg_i;					//bg求平均计数

	Vector3f  delta;				//bg在dt时间内的角度
	float _2_delta0;
	float _4_delta0;
	float coeff_1;
	float coeff_2;
	Matrix4x4 Picard_MD0;
	Matrix4x4 Mone;

	Vector3f bm;					//bm raw
	Vector3f bm_s;					//bm软铁
	Vector3f bm_h;					//bm硬铁
	Vector3f bm_noise;				//bm噪声
	Vector3f bm_filter;				//bm滤波
	bool     bm_filter_enable;
	float    bm_norm;
	Vector3f dm;					//d系下mag值

	Vector3f nr;					//n系位置
	Vector3f nv;					//n系速度

	gps_data gps;					//gps数据
	HMC6343_Data hmc;				//HMC6343数据

	//IMU安装方式
	bool att_up;

	//参数
	float ARW;						//角度随机游走
	float Bs_gyro;					//陀螺零偏不稳定性
	float VRW;						//速度随机游走
	float Bs_acc;					//加速度零偏不稳定性

	float var_obs_roll;				//roll角方差
	float var_obs_pitch;			//pitch角方差
	float var_obs_mag_yaw;			//yaw角(mag)方差
	float var_obs_rtk_yaw;			//yaw角(rtk)方差

	//融合时间测量
	int time_pre;
	int time_now;

	bool initd;						//imu是否初始化
	bool task_created;				//imu task是否创建
	bool subsribe_initd;			//数据是否订阅
	bool first_aimed;				//是否初始化对准
	bool re_aimed;
	bool fusion_ok;					//融合状态
	bool Qnb_update;				//四元数是否更新
	bool Enb_update;				//欧拉角是否更新

	//adis16488a
	int  adis16488a_fd;
	int  orb_adis16488a_fd;
	bool adis16488a_enable;
	bool adis16488a_update;
	int  adis16488a_time;

	//hmc6343
	int  hmc6343_fd;
	int  orb_hmc6343_fd;
	bool hmc6343_enable;
	bool hmc6343_update;
	int  hmc6343_time;

	//gps
	int  orb_gps_fd;
	bool gps_enable;
	bool gps_update;
	bool gps_ok;
	float gps_yaw_correct;
	bool rtk_filter_enable;

	//静止判断
	float acc_switch_up;
	float acc_switch_low;
	int  static_i;
	int  static_N;
	bool static_flag;

	fusion_print_select fusion_print;	//打印数据选择
	raw_print_select    raw_print;	//打印数据选择

}Imu_Data;


extern void imu_init(Imu_Data *imu);
extern bool imu_Subsribe_init(Imu_Data *imu);
extern bool imu_Subsribe(Imu_Data *imu);
extern void imu_publish_init(void);
extern void imu_publish(Imu_Data *imu);

extern bool imu_rawdata(Imu_Data *imu);

extern void FirstOrderLowPass( float *output,  float input );
extern void gyro_filter( Vector3f *gyro , Vector3f *gyro_filter_outputer);
extern void acc_filter( Vector3f *acc , Vector3f *acc_filter_outputer);
extern void mag_filter( Vector3f *mag , Vector3f *mag_filter_outputer);
extern void rtk_filter( float *rtk_in, float *rtk_out);

extern void imu_fusion_print(Imu_Data *imu);
extern void imu_raw_print(Imu_Data *imu);

extern void imu_aim(Imu_Data *imu);

extern bool imu_fusion_cf(Imu_Data *imu);
extern bool imu_fusion_ekf_base(Imu_Data *imu);
extern bool imu_fusion(Imu_Data *imu);

extern float imu_mag_declination(float lat, float lon);

#endif
