#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/fs/fs.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdbool.h>
#include <poll.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <math.h>

#include <satellite_attitude_control/adrc.h>
#include <DSP_Lib/arm_math.h>

struct adrc_data ADRC_Pitch_Controller,ADRC_polar_Controller,ADRC_yaw_Controller;
struct adrc_td TD_Pitch_Controller,TD_polar_Controller,TD_yaw_Controller;

float adrc_pitch_par[17]=
{
    /*TD跟踪微分器   改进最速TD,h0=N*h    扩张状态观测器ESO                      扰动补偿       非线性组合*/
    /*  r     h      N0  c             beta_01   beta_02    beta_03         b0            beta_0  beta_1     beta_2     N1     C    alpha1  alpha2  zeta  b*/    
    300000 ,0.005 , 3, 1,              300,      4000,      10000,          0.001,        0.002,   2.0,      0.0010,    5,    5,    0.8,   1.5,    50,    0
};

float adrc_yaw_par[17]=
{
    /*TD跟踪微分器   改进最速TD,h0=N*h    扩张状态观测器ESO                      扰动补偿       非线性组合*/
    /*  r     h      N0  c             beta_01   beta_02    beta_03         b0            beta_0  beta_1     beta_2     N1     C    alpha1  alpha2  zeta  b*/    
    300000 ,0.005 , 3, 1,              300,      4000,      10000,          0.001,        0.002,   2.0,      0.0010,    5,    5,    0.8,   1.5,    50,    0
};
float adrc_polar_par[17]=
{
    /*TD跟踪微分器   改进最速TD,h0=N*h    扩张状态观测器ESO                      扰动补偿       非线性组合*/
    /*  r     h      N0  c             beta_01   beta_02    beta_03         b0            beta_0  beta_1     beta_2     N1     C    alpha1  alpha2  zeta  b*/   
    300000 ,0.005 , 3, 1,              300,      4000,      10000,          0.001,        0.002,   1.2,      0.0005,    5,    5,    0.8,   1.5,    50,    0
};

float td_pitch_par[2][4]=
{
    /*TD跟踪微分器   改进最速TD,h0=N*h */
    /*r     h      N0  c */
    {400000 ,0.005 , 1, 1},
    {400000 ,0.005 , 1, 1},
};

float td_yaw_par[2][4]=
{
    /*TD跟踪微分器   改进最速TD,h0=N*h */
    /*r     h      N0  c */
    {300000 ,0.005 , 3, 1},
    {300000 ,0.005 , 3, 1},
};

float td_polar_par[2][4]=
{
    /*TD跟踪微分器   改进最速TD,h0=N*h */
    /*r     h      N0  c */
    {300000 ,0.005 , 3, 1},
    {300000 ,0.005 , 3, 1},
};

/*
三阶IIR滤波器-80HZ相位滞后120°-40HZ相位滞后120°
IIR  1000-20  3½×µÍÍ¨
a =1.0000   -2.7488    2.5282   -0.7776
b =2.196e-04    6.588e-04    6.588e-04    2.196e-04
------------------------------------------------------
IIR  1000-40  3½×µÍÍ¨
a =1.0000   -2.4986    2.1153   -0.6041
b =0.0016    0.0047    0.0047    0.0016
------------------------------------------------------
IIR  1000-60  3½×µÍÍ¨
a =1.0000 ,  -2.2501 ,   1.7564 ,  -0.4683
b =0.0048 ,   0.0143 ,  0.0143  ,  0.0048
------------------------------------------------------
IIR  1000-80  3½×µÍÍ¨
a =1.0000   -2.0038    1.4471   -0.3618
b =0.0102    0.0305    0.0305    0.0102
------------------------------------------------------
IIR  1000-100  3½×µÍÍ¨
a =1.0000   -1.7600    1.1829   -0.2781
b =0.0181    0.0543    0.0543    0.0181
------------------------------------------------------
IIR  1000-120  3½×µÍÍ¨
a =1.0000   -1.5189    0.9600   -0.2120
b =0.0286    0.0859    0.0859    0.0286
------------------------------------------------------
IIR  1000-140  3½×µÍÍ¨
a =1.0000   -1.2803    0.7751   -0.1598
b =0.0419    0.1256    0.1256    0.0419
------------------------------------------------------
IIR  1000-160  3½×µÍÍ¨
a =1.0000 ,  -1.0441  ,  0.6252  , -0.1180
b =0.0579 ,   0.1737  ,  0.1737  ,  0.0579
------------------------------------------------------
IIR  1000-180  3½×µÍÍ¨
a =1.0000   -0.8098    0.5081   -0.0843
b =0.0767    0.2302    0.2302    0.0767
------------------------------------------------------
IIR  1000-200  3½×µÍÍ¨
a =1.0000   -0.5772    0.4218   -0.0563
b =0.0985    0.2956    0.2956    0.0985
------------------------------------------------------
IIR  1000-300  3½×µÍÍ¨
a =1.0000    0.5772    0.4218    0.0563
b =0.2569    0.7707    0.7707    0.2569
------------------------------------------------------
IIR  1000-400  3½×µÍÍ¨
a =1.0000    1.7600    1.1829    0.2781
b =0.5276    1.5829    1.5829    0.5276
*/

float pitch_bFilt[4] = {1.0000 ,  -2.2501 ,   1.7564 ,  -0.4683};
float pitch_aFilt[4] = {0.0048 ,   0.0143 ,  0.0143  ,  0.0048};

float yaw_bFilt[4] = {0.0016 , 0.0047 ,   0.0047 ,   0.0016};
float yaw_aFilt[4] = {1.0000 ,  -2.4986  ,  2.1153  , -0.6041};

float polar_bFilt[4] = {0.0016 , 0.0047 ,   0.0047 ,   0.0016};
float polar_aFilt[4] = {1.0000 ,  -2.4986  ,  2.1153  , -0.6041};

float signal_bFilt[4] = {0.0016 , 0.0047 ,   0.0047 ,   0.0016};
float signal_aFilt[4] = {1.0000 ,  -2.4986  ,  2.1153  , -0.6041};

float com_bFilt[4] = {0.0016 , 0.0047 ,   0.0047 ,   0.0016};
float com_aFilt[4] = {1.0000 ,  -2.4986  ,  2.1153  , -0.6041};

static float b_buffer[4];float a_buffer[4];
static bool filter_start_flag = false;//加速度滤波

// a[0]*y(k) + a[1]*y(k-1) + a[2]*y(k-2) + a[3]*y(k-3) = b[0]*x(k) + b[1]*x(k-1) + b[2]*x(k-2) + b[3]*x(k-3)
// a[0] = 1;y(k) = ( b[0]*x(k) + b[1]*x(k-1) + b[2]*x(k-2) + b[3]*x(k-3) ) - ( a[1]*y(k-1) + a[2]*y(k-2) + a[3]*y(k-3) )
float iir_filter( float in ,float *a_Filt,float *b_Filt)
{
	int i;float out;
    float temp_a,temp_b;

	if(!filter_start_flag)
	{
		filter_start_flag = true;

        for(i=0;i<4;i++)
        {
            b_buffer[i] = in;
            a_buffer[i] = in;
        }
        out = in;
	}
	else
	{
        b_buffer[0] = in;

        temp_a = a_Filt[1] * a_buffer[1] + a_Filt[2] * a_buffer[2] + a_Filt[3] * a_buffer[3];                
        temp_b = b_Filt[0] * b_buffer[0] + b_Filt[1] * b_buffer[1] + b_Filt[2] * b_buffer[2]+b_Filt[3] * b_buffer[3];

        out =  temp_b - temp_a;

        b_buffer[1] = b_buffer[0];
        b_buffer[2] = b_buffer[1];
        b_buffer[3] = b_buffer[2];
        
        a_buffer[1] = out;
        a_buffer[2] = a_buffer[1];
        a_buffer[3] = a_buffer[2];        
	}

    return out;
}

void ADRC_Init(struct adrc_data *targ_par,float *init_par)
{
    uint16_t i1 = 0;

    targ_par->r      = init_par[i1++];
    targ_par->h      = init_par[i1++];
    targ_par->N0     = (uint16_t)(init_par[i1++]);
    targ_par->damp_c = init_par[i1++];

    targ_par->beta_01= init_par[i1++];
    targ_par->beta_02= init_par[i1++];
    targ_par->beta_03= init_par[i1++];
    targ_par->b0     = init_par[i1++];

    targ_par->beta_0 = init_par[i1++];
    targ_par->beta_1 = init_par[i1++];
    targ_par->beta_2 = init_par[i1++];
    targ_par->N1     = (uint16_t)(init_par[i1++]);
    targ_par->c      = init_par[i1++];

    targ_par->alpha1 = init_par[i1++];
    targ_par->alpha2 = init_par[i1++];
    targ_par->zeta   = init_par[i1++];
    targ_par->b      = init_par[i1++];
    i1 = 0;

}
void td_Init(struct adrc_td *targ_par,float *init_par)
{
    uint16_t i1 = 0; 

    targ_par->td_r0 = init_par[i1++];
    targ_par->td_h0 = init_par[i1++];
    targ_par->td_N0 = init_par[i1++];
    targ_par->td_damp_c0 = init_par[i1++];

    targ_par->td_r1 = init_par[i1++];
    targ_par->td_h1 = init_par[i1++];
    targ_par->td_N1 = init_par[i1++];
    targ_par->td_damp_c1 = init_par[i1++];    

    i1 = 0;

}
float Constrain_Float(float amt, float low, float high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

float Sign_ADRC(float Input)
{
    float output=0;

    if(Input>0.0)
      output=1.0;
    else if(Input<-0.0)
      output=-1.0;
    else 
      output=0;
    return output;
}

float Fsg_ADRC(float x,float d)
{
    float output=0;
    output=(Sign_ADRC(x+d)-Sign_ADRC(x-d))/2;
    return output;
}
//ADRC最速跟踪微分器TD，改进的算法fhan
void FhanTD_ADRC(struct adrc_data *fhan_Input,float expect_ADRC)//安排ADRC过度过程
{
    float d=0,a0=0,y=0,a1=0,a2=0,a=0;
    float x1_delta=0;//ADRC状态跟踪误差项
    x1_delta=fhan_Input->x1-expect_ADRC;//用x1-v(k)替代x1得到离散更新公式
    fhan_Input->h0=fhan_Input->N0*fhan_Input->h;//用h0替代h，解决最速跟踪微分器速度超调问题

    //Fhan
    d=fhan_Input->r*fhan_Input->h0*fhan_Input->h0;//d=rh^2;
    a0=fhan_Input->h0*fhan_Input->x2;//a0=h*x2
    y=x1_delta+a0;//y=x1+a0
    a1=sqrt(d*(d+8*fabsf(y)));//a1=sqrt(d*(d+8*ABS(y))])
    a2=a0+Sign_ADRC(y)*(a1-d)/2;//a2=a0+sign(y)*(a1-d)/2;
    a=(a0+y)*Fsg_ADRC(y,d)+a2*(1-Fsg_ADRC(y,d));

    //TD
    fhan_Input->fh=-fhan_Input->r*(a/d)*Fsg_ADRC(a,d)-fhan_Input->r*Sign_ADRC(a)*(1-Fsg_ADRC(a,d));//得到最速微分加速度跟踪量
    fhan_Input->x1+=fhan_Input->h*fhan_Input->x2;//跟新最速跟踪状态量x1
    fhan_Input->x2+=fhan_Input->h*fhan_Input->fh;//跟新最速跟踪状态量微分x2
}
float Fhan_ADRC(float x1,float x2,float r,float h)//安排ADRC过度过程
{
    float d=0,a0=0,y=0,a1=0,a2=0,a=0;
    float fhan_output=0;

    d  = r*h*h;//d=rh^2;
    a0 = h*x2;//a0=h*x2
    y  = x1+a0;//y=x1+a0
    a1 = sqrt(d*(d+8*fabsf(y)));//a1=sqrt(d*(d+8*ABS(y))])
    a2 = a0+Sign_ADRC(y)*(a1-d)/2;//a2=a0+sign(y)*(a1-d)/2;
    a  = (a0+y)*Fsg_ADRC(y,d)+a2*(1-Fsg_ADRC(y,d));

    fhan_output = -r*(a/d)*Fsg_ADRC(a,d)-r*Sign_ADRC(a)*(1-Fsg_ADRC(a,d));//得到最速微分加速度跟踪量    

    return fhan_output;
}
//原点附近有连线性段的连续幂次函数
float Fal_ADRC(float e,float alpha,float zeta)
{
    int16_t s=0;float fal_output=0;

    s = (Sign_ADRC(e+zeta)-Sign_ADRC(e-zeta))/2;
    fal_output = e*s/(powf(zeta,1-alpha))+powf(fabsf(e),alpha)*Sign_ADRC(e)*(1-s);
    return fal_output;
}
/************TD微分跟踪器、TD微分信号提取********************/
void TD_ADRC(struct adrc_data *fhan_Input,float expect_ADRC)
{
    float x1_delta = 0;//ADRC状态跟踪误差项
    float h0 = 0;//ADRC状态跟踪误差项
    float x2_c = 0;

    x1_delta = fhan_Input->x1-expect_ADRC;//用x1-v(k)替代x1得到离散更新公式
    h0 = fhan_Input->N0*fhan_Input->h;//用h0替代h，解决最速跟踪微分器速度超调问题
    x2_c = fhan_Input->x2*fhan_Input->damp_c;//用cx2代替x2，可以调节“阻尼因子”，使系统避免高频颤震问题(fhan_Input->damp_c 为可调节‘阻尼因子’)

    fhan_Input->fh = Fhan_ADRC(x1_delta,x2_c,fhan_Input->r,h0);

    fhan_Input->x1 += fhan_Input->h*fhan_Input->x2;//跟新最速跟踪状态量x1
    fhan_Input->x2 += fhan_Input->h*fhan_Input->fh;//跟新最速跟踪状态量微分x2(fh <= r)
}
/************扩张状态观测器********************/
//状态观测器参数beta01=1/h  beta02=1/(3*h^2)  beta03=2/(8^2*h^3) ...
void ESO_ADRC(struct adrc_data *fhan_Input)
{
    fhan_Input->fe  = Fal_ADRC(fhan_Input->e,0.5,fhan_Input->h);//非线性函数，提取跟踪状态与当前状态误差
    fhan_Input->fe1 = Fal_ADRC(fhan_Input->e,0.25,fhan_Input->h);

    fhan_Input->e   = fhan_Input->z1-fhan_Input->y;//状态误差
    /*************扩展状态量更新**********/
    fhan_Input->z1 += fhan_Input->h*(fhan_Input->z2-fhan_Input->beta_01*fhan_Input->e);
    fhan_Input->z2 += fhan_Input->h*(fhan_Input->z3-fhan_Input->beta_02*fhan_Input->fe+fhan_Input->b*fhan_Input->u);
                                      
    //ESO估计状态加速度信号，进行扰动补偿，传统MEMS陀螺仪漂移较大，估计会产生漂移
    fhan_Input->z3 += fhan_Input->h*(-fhan_Input->beta_03*fhan_Input->fe1);
}
/************非线性组合****************/

void Nolinear_Conbination_ADRC_MODE1(struct adrc_data *fhan_Input)
{
    float d=0,a0=0,y=0,a1=0,a2=0,a=0;
    float Sy=0,Sa=0;//ADRC状态跟踪误差项

    fhan_Input->h1=fhan_Input->N1*fhan_Input->h;

    d = fhan_Input->r*fhan_Input->h1*fhan_Input->h1;
    a0 =fhan_Input->h1*fhan_Input->c*fhan_Input->e2;
    y = fhan_Input->e1+a0;
    a1 = sqrt(d*(d+8*fabsf(y)));
    a2 = a0+Sign_ADRC(y)*(a1-d)/2;

    Sy = Fsg_ADRC(y,d);
    a = (a0+y-a2)*Sy+a2;
    Sa = Fsg_ADRC(a,d);

    fhan_Input->u0 = -fhan_Input->r*((a/d)-Sign_ADRC(a))*Sa-fhan_Input->r*Sign_ADRC(a);

    //a=(a0+y)*Fsg_ADRC(y,d)+a2*(1-Fsg_ADRC(y,d));

    //fhan_Input->fh=-fhan_Input->r*(a/d)*Fsg_ADRC(a,d)
    //                -fhan_Input->r*Sign_ADRC(a)*(1-Fsg_ADRC(a,d));//得到最速微分加速度跟踪量
}

void Nolinear_Conbination_ADRC(struct adrc_data *fhan_Input)
{
    float temp_e2=0;

    temp_e2 = Constrain_Float(fhan_Input->e2,-3000,3000);

    fhan_Input->u0  =   fhan_Input->beta_1*Fal_ADRC(fhan_Input->e1,fhan_Input->alpha1,fhan_Input->zeta)
                        +fhan_Input->beta_2*Fal_ADRC(temp_e2,fhan_Input->alpha2,fhan_Input->zeta);
}

void ADRC_Control(struct adrc_data *fhan_Input,float expect_ADRC,float feedback_ADRC)
{
    /*自抗扰控制器第1步*/
    /*
    安排过度过程，输入为期望给定，
    由TD跟踪微分器得到：
    过度期望信号x1，过度期望微分信号x2
    */
    TD_ADRC(fhan_Input,expect_ADRC);

    /*自抗扰控制器第2步*/
    /************系统输出值为反馈量，状态反馈，ESO扩张状态观测器的输入*********/
    fhan_Input->y=feedback_ADRC;
    /*
    扩张状态观测器，得到反馈信号的扩张状态：
    1、状态信号z1；
    2、状态速度信号z2；
    3、状态加速度信号z3。
    其中z1、z2用于作为状态反馈与TD微分跟踪器得到的x1,x2做差后，
    经过非线性函数映射，乘以beta系数后，
    组合得到未加入状态加速度估计扰动补偿的原始控制量u
    */
    ESO_ADRC(fhan_Input);//低成本MEMS会产生漂移，扩展出来的z3此项会漂移，目前暂时未想到办法解决，未用到z3
    /*自抗扰控制器第3步*/
    /********状态误差反馈率***/
    fhan_Input->e0+=fhan_Input->e1*fhan_Input->h;//状态积分项
    fhan_Input->e1=fhan_Input->x1-fhan_Input->z1;//状态偏差项
    fhan_Input->e2=fhan_Input->x2-fhan_Input->z2;//状态微分项，
    /********线性组合*******/
    /*
    fhan_Input->u0=//fhan_Input->beta_0*fhan_Input->e0
                  +fhan_Input->beta_1*fhan_Input->e1
                  +fhan_Input->beta_2*fhan_Input->e2;
    */
    Nolinear_Conbination_ADRC(fhan_Input);
    /**********扰动补偿*******/
    //fhan_Input->u=fhan_Input->u0
    //             -fhan_Input->z3/fhan_Input->b0;
    //由于MEMS传感器漂移比较严重，当beta_03取值比较大时，长时间z3漂移比较大，目前不加入扰动补偿控制量
    fhan_Input->u=Constrain_Float(fhan_Input->u0,-200,200);
}

