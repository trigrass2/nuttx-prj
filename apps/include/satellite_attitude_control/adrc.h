#ifndef _ADRC_H_
#define _ADRC_H_

struct adrc_data
{
    /*****安排过度过程*******/
    float x1;//跟踪微分期状态量
    float x2;//跟踪微分期状态量微分项

    float r;//时间尺度
    float h;//ADRC系统积分时间
    uint16_t N0;//跟踪微分器解决速度超调h0=N*h
    float damp_c;//阻尼因子

    float h0;
    float fh;//最速微分加速度跟踪量
    /*****扩张状态观测器*******/
    /******已系统输出y和输入u来跟踪估计系统状态和扰动*****/
    float z1;
    float z2;
    float z3;//根据控制对象输入与输出，提取的扰动信息
    float e;//系统状态误差
    float y;//系统输出量
    float fe;
    float fe1;
    float beta_01;
    float beta_02;
    float beta_03;
    float b;
    /**********系统状态误差反馈率********************/
    float e0;//状态误差积分项
    float e1;//状态偏差
    float e2;//状态量微分项
    float u0;//非线性组合系统输出
    float u;//带扰动补偿后的输出
    float b0;//扰动补偿
    /*********第一种组合形式************************/
    float beta_0;//线性
    float beta_1;//非线性组合参数
    float beta_2;//u0=beta_1*e1+beta_2*e2+(beta_0*e0);
    /*********第二种组合形式************************/
    float alpha1;//u0=beta_1*fal(e1,alpha1,zeta)+beta_2*fal(e2,alpha2,zeta)
    float alpha2;//0<alpha1<1<alpha2
    float zeta;//线性段的区间长度
    /*********第三种组合形式************************/
    float h1;//u0=-fhan(e1,e2,r,h1);
    uint16_t N1;//跟踪微分器解决速度超调h0=N*h
    /*********第四种组合形式************************/
    float c;//u0=-fhan(e1,c*e2*e2,r,h1);
};

struct adrc_td
{
    float td_v1;//跟踪微分期状态量
    float td_v2;//跟踪微分期状态量微分项

    float td_r0;//时间尺度
    float td_h0;//ADRC系统积分时间
    uint16_t td_N0;//跟踪微分器解决速度超调h0=N*h
    float td_damp_c0;//阻尼因子
    float td_fh0;//最速微分加速度跟踪量

    float td_z1;//跟踪微分期状态量
    float td_z2;//跟踪微分期状态量微分项

    float td_r1;//时间尺度
    float td_h1;//ADRC系统积分时间
    uint16_t td_N1;//跟踪微分器解决速度超调h0=N*h
    float td_damp_c1;//阻尼因子
    float td_fh1;//最速微分加速度跟踪量
    /*********第一种组合形式************************/
    float con1_beta0;//线性
    float con1_beta1;//非线性组合参数
    float con1_beta2;//u0=beta1*e1+beta2*e2+(beta0*e0);
    /*********第二种组合形式************************/
    //u0=beta_0*fal(e0,alpha0,zeta)+beta_1*fal(e1,alpha1,zeta)+beta_2*fal(e2,alpha2,zeta)
    //0<alpha1<1<alpha2
    float con2_alpha0;
    float con2_alpha1;
    float con2_alpha2;
    float con2_zeta;//线性段的区间长度
    /*********第三种组合形式************************/
    //u0=con_beta0*e0 - fhan(e1,e2,r1,h1);   
    float con3_beta0;
    float con3_r1;
    float con3_h1;
    uint16_t con3_N1;//跟踪微分器解决速度超调h0=N*h
    float con3_damp_c1;//阻尼因子
    /*********第四种组合形式************************/
    //u0=-fhan(e1,c*e2,r2,h2);
    float con4_beta0;
    float con4_r2;
    float con4_h2;
    uint16_t con4_N2;//跟踪微分器解决速度超调h0=N*h
    float con4_damp_c2;//阻尼因子
    /**********系统状态误差反馈率********************/
    float e0;//状态误差积分项
    float e1;//状态偏差
    float e2;//状态量微分项
    float u0;//非线性组合系统输出
    /**********系统状态误差反馈率********************/
};

float iir_filter( float in ,float *a_Filt,float *b_Filt);
void  ADRC_Init(struct adrc_data *targ_par,float *init_par);
void td_Init(struct adrc_td *targ_par,float *init_par);
void  FhanTD_ADRC(struct adrc_data *fhan_Input,float expect_ADRC);//安排ADRC过度过程
float Fhan_ADRC(float x1,float x2,float r,float h);//安排ADRC过度过程
float Fal_ADRC(float e,float alpha,float zeta);
void  TD_ADRC(struct adrc_data *fhan_Input,float expect_ADRC);
void  ADRC_Control(struct adrc_data *fhan_Input,float expect_ADRC,float feedback);
void  Nolinear_Conbination_ADRC_MODE1(struct adrc_data *fhan_Input);

extern struct adrc_data ADRC_Pitch_Controller,ADRC_polar_Controller,ADRC_yaw_Controller;
extern struct adrc_td TD_Pitch_Controller,TD_polar_Controller,TD_yaw_Controller;

extern float adrc_pitch_par[17];
extern float adrc_yaw_par[17];
extern float adrc_polar_par[17];

extern float td_pitch_par[2][4];

extern float td_yaw_par[2][4];

extern float td_polar_par[2][4];



extern float pitch_bFilt[4];
extern float pitch_aFilt[4];

extern float yaw_bFilt[4];
extern float yaw_aFilt[4];

extern float polar_bFilt[4];
extern float polar_aFilt[4];

extern float signal_bFilt[4];
extern float signal_aFilt[4];

extern float com_bFilt[4];
extern float com_aFilt[4];

#endif

