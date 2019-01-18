#include <nuttx/config.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <nuttx/fs/fs.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include <nuttx/fs/fs.h>
#include <debug.h>
#include <syslog.h>
#include <imu/imu_matrix.h>
#include <DSP_Lib/arm_math.h>

/*------------------------------向量计算Vector3f------------------------------*/
/*-----向量相加-----*/
void  Vector3f_Add_Vector3f(Vector3f  *a , Vector3f  *b,  Vector3f  *c)
{
	c->x = a->x + b->x;
	c->y = a->y + b->y;
	c->z = a->z + b->z;
}

/*-----向量相减-----*/
void  Vector3f_Sub_Vector3f(Vector3f  *a , Vector3f  *b,  Vector3f  *c)
{
	c->x = a->x - b->x;
	c->y = a->y - b->y;
	c->z = a->z - b->z;
}

/*-----向量点乘-----*/
void  Vector3f_Mult_Vector3f(Vector3f  *a , Vector3f  *b,  Vector3f  *c)
{
	c->x = a->x * b->x;
	c->y = a->y * b->y;
	c->z = a->z * b->z;
}

/*-----向量乘以系数-----*/
void  Vector3f_Mult_Coeff(Vector3f  *a , float b,  Vector3f  *c)
{
	arm_matrix_instance_f32 ma;
	arm_matrix_instance_f32 mc;

	ma.numRows = 3;
	ma.numCols = 1;
	ma.pData = (float *)a->axis;

	mc.numRows = 3;
	mc.numCols = 1;
	mc.pData = (float *)c->axis;

	arm_mat_scale_f32(&ma, b, &mc);
}

/*-----向量归一化-----*/
bool  Vector3f_Normalize(Vector3f  *a)
{
	float tmp, norm;
	arm_power_f32(a->axis, 3, &tmp);
	arm_sqrt_f32(tmp, &norm);

	if(norm == 0)
		return false;

	Vector3f_Mult_Coeff(a, 1/norm, a);

	return true;
}

/*-----向量叉乘-----*/
void  Vector3f_Cross_Vector3f(Vector3f  *a , Vector3f  *b,  Vector3f  *c)
{
	c->x = a->y * b->z - a->z * b->y;
	c->y = a->z * b->x - a->x * b->z;
	c->z = a->x * b->y - a->y * b->x;
}

/*-----向量叉乘左半边矩阵-----*/
void  Vector3f_Cross(Vector3f  *a , Matrix3x3  *b)
{
	b->T11 =  0;
	b->T12 = -a->z;
	b->T13 =  a->y;

	b->T21 =  a->z;
	b->T22 =  0;
	b->T23 = -a->x;

	b->T31 = -a->y;
	b->T32 =  a->x;
	b->T33 =  0;
}


/*------------------------------向量计算Vector3d------------------------------*/
/*-----向量相加-----*/
void  Vector3d_Add_Vector3d(Vector3d  *a , Vector3d  *b,  Vector3d  *c)
{
	c->x = a->x + b->x;
	c->y = a->y + b->y;
	c->z = a->z + b->z;
}

/*-----向量相减-----*/
void  Vector3d_Sub_Vector3d(Vector3d  *a , Vector3d  *b,  Vector3d  *c)
{
	c->x = a->x - b->x;
	c->y = a->y - b->y;
	c->z = a->z - b->z;
}

/*-----向量点乘-----*/
void  Vector3d_Mult_Vector3d(Vector3d  *a , Vector3d  *b,  Vector3d  *c)
{
	c->x = a->x * b->x;
	c->y = a->y * b->y;
	c->z = a->z * b->z;
}

/*-----向量乘以系数-----*/
void  Vector3d_Mult_Coeff(Vector3d  *a , float b,  Vector3d  *c)
{
	int i;
	for(i=0; i<3; i++)
	{
		c->axis[i] = a->axis[i] * b;
	}
}

/*-----向量归一化-----*/
bool  Vector3d_Normalize(Vector3d  *a)
{
	float norm;
	norm = sqrt(a->x * a->x  + a->y * a->y + a->z * a->z);

	if(norm == 0)
			return false;

	int i;
	for(i=0; i<3; i++)
	{
		a->axis[i] = a->axis[i] / norm;
	}

	return true;
}

/*-----向量叉乘-----*/
void  Vector3d_Cross_Vector3d(Vector3d  *a , Vector3d  *b,  Vector3d  *c)
{
	c->x = a->y * b->z - a->z * b->y;
	c->y = a->z * b->x - a->x * b->z;
	c->z = a->x * b->y - a->y * b->x;
}

/*-----向量叉乘左半边矩阵-----*/
void  Vector3d_Cross(Vector3d  *a , Matrix3x3  *b)
{
	b->T11 =  0;
	b->T12 = -a->z;
	b->T13 =  a->y;

	b->T21 =  a->z;
	b->T22 =  0;
	b->T23 = -a->x;

	b->T31 = -a->y;
	b->T32 =  a->x;
	b->T33 =  0;
}


/*------------------------------四元数计算------------------------------*/
/*-----四元数乘以系数-----*/
void Quaternion_Mult_Coeff(Quaternion *Q_src, float b, Quaternion *Q_des)
{
	arm_matrix_instance_f32 ma;
	arm_matrix_instance_f32 mc;

	ma.numRows = 4;
	ma.numCols = 1;
	ma.pData = (float *)Q_src->axis;

	mc.numRows = 4;
	mc.numCols = 1;
	mc.pData = (float *)Q_des->axis;

	arm_mat_scale_f32(&ma, b, &mc);
}

/*-----四元数归一化-----*/
bool Quaternion_Normalize(Quaternion *Q)
{
	float tmp, norm;
	arm_power_f32(Q->axis, 4, &tmp);
	arm_sqrt_f32(tmp, &norm);

	if(norm == 0)
		return false;
	Quaternion_Mult_Coeff(Q, 1/norm, Q);

	return true;
}


/*------------------------------3维矩阵计算------------------------------*/

/*-----3阶矩阵乘以3维向量-----*/
void  Matrix3x3_Mult_Vector3f(Matrix3x3  *a , Vector3f  *b,  Vector3f  *c)
{
	arm_matrix_instance_f32 ma;
	arm_matrix_instance_f32 mb;
	arm_matrix_instance_f32 mc;

	ma.numRows = 3;
	ma.numCols = 3;
	ma.pData = (float *)a->axis;

	mb.numRows = 3;
	mb.numCols = 1;
	mb.pData = (float *)b->axis;

	mc.numRows = 3;
	mc.numCols = 1;
	mc.pData = (float *)c->axis;

	arm_mat_mult_f32(&ma, &mb, &mc);
}

/*-----3阶矩阵乘以3维矩阵-----*/
void  Matrix3x3_Mult_Matrix3x3(Matrix3x3  *a , Matrix3x3  *b,  Matrix3x3  *c)
{
	arm_matrix_instance_f32 ma;
	arm_matrix_instance_f32 mb;
	arm_matrix_instance_f32 mc;

	ma.numRows = 3;
	ma.numCols = 3;
	ma.pData = (float *)a->axis;

	mb.numRows = 3;
	mb.numCols = 3;
	mb.pData = (float *)b->axis;

	mc.numRows = 3;
	mc.numCols = 3;
	mc.pData = (float *)c->axis;

	arm_mat_mult_f32(&ma, &mb, &mc);
}

/*-----3阶矩阵转置-----*/
void  Matrix3x3_Trans(Matrix3x3 *a,  Matrix3x3 *b)
{
	arm_matrix_instance_f32 ma;
	arm_matrix_instance_f32 mb;

	ma.numRows = 3;
	ma.numCols = 3;
	ma.pData = (float *)a->axis;

	mb.numRows = 3;
	mb.numCols = 3;
	mb.pData = (float *)b->axis;


	arm_mat_trans_f32(&ma, &mb);
}

void  Matrix3x3_One(Matrix3x3 *a)
{
	a->T11 = 1;	  a->T12 = 0;  	a->T13 = 0;
	a->T21 = 0;	  a->T22 = 1;  	a->T23 = 0;
	a->T31 = 0;	  a->T32 = 0;  	a->T33 = 1;
}

/*------------------------------4维矩阵计算------------------------------*/
/*-----4阶矩阵 + 4维矩阵-----*/
void  Matrix4x4_Add_Matrix4x4(Matrix4x4  *a , Matrix4x4  *b,  Matrix4x4  *c)
{
	arm_matrix_instance_f32 ma;
	arm_matrix_instance_f32 mb;
	arm_matrix_instance_f32 mc;

	ma.numRows = 4;
	ma.numCols = 4;
	ma.pData = (float *)a->axis;

	mb.numRows = 4;
	mb.numCols = 4;
	mb.pData = (float *)b->axis;

	mc.numRows = 4;
	mc.numCols = 4;
	mc.pData = (float *)c->axis;

	arm_mat_add_f32(&ma, &mb, &mc);
}

/*-----4阶矩阵 - 4维矩阵-----*/
void  Matrix4x4_Sub_Matrix4x4(Matrix4x4  *a , Matrix4x4  *b,  Matrix4x4  *c)
{
	arm_matrix_instance_f32 ma;
	arm_matrix_instance_f32 mb;
	arm_matrix_instance_f32 mc;

	ma.numRows = 4;
	ma.numCols = 4;
	ma.pData = (float *)a->axis;

	mb.numRows = 4;
	mb.numCols = 4;
	mb.pData = (float *)b->axis;

	mc.numRows = 4;
	mc.numCols = 4;
	mc.pData = (float *)c->axis;

	arm_mat_sub_f32(&ma, &mb, &mc);
}

/*-----4阶矩阵 * 4维矩阵-----*/
void  Matrix4x4_Mult_Matrix4x4(Matrix4x4  *a , Matrix4x4  *b,  Matrix4x4  *c)
{
	arm_matrix_instance_f32 ma;
	arm_matrix_instance_f32 mb;
	arm_matrix_instance_f32 mc;

	ma.numRows = 4;
	ma.numCols = 4;
	ma.pData = (float *)a->axis;

	mb.numRows = 4;
	mb.numCols = 4;
	mb.pData = (float *)b->axis;

	mc.numRows = 4;
	mc.numCols = 4;
	mc.pData = (float *)c->axis;

	arm_mat_mult_f32(&ma, &mb, &mc);
}

/*-----4阶矩阵 * 4维矩阵-----*/
void  Matrix4x4_Mult_Coeff(Matrix4x4  *a , float  b,  Matrix4x4  *c)
{
	arm_matrix_instance_f32 ma;
	arm_matrix_instance_f32 mc;

	ma.numRows = 4;
	ma.numCols = 4;
	ma.pData = (float *)a->axis;

	mc.numRows = 4;
	mc.numCols = 4;
	mc.pData = (float *)c->axis;

	arm_mat_scale_f32(&ma, b, &mc);
}

/*-----4阶矩阵转置-----*/
void  Matrix4x4_Trans(Matrix4x4 *a,  Matrix4x4 *b)
{
	arm_matrix_instance_f32 ma;
	arm_matrix_instance_f32 mb;

	ma.numRows = 4;
	ma.numCols = 4;
	ma.pData = (float *)a->axis;

	mb.numRows = 4;
	mb.numCols = 4;
	mb.pData = (float *)b->axis;


	arm_mat_trans_f32(&ma, &mb);
}


void  Matrix4x4_One(Matrix4x4 *a)
{
	a->T11 = 1;	  a->T12 = 0;  	a->T13 = 0;    a->T14 = 0;
	a->T21 = 0;	  a->T22 = 1;  	a->T23 = 0;    a->T24 = 0;
	a->T31 = 0;	  a->T32 = 0;  	a->T33 = 1;    a->T34 = 0;
	a->T41 = 0;	  a->T42 = 0;  	a->T43 = 0;    a->T44 = 1;
}


/*------------------------------四元数、转换矩阵、欧拉角互转------------------------------*/
/*--------------------------旋转矩阵旋转顺序Cbn：roll-pitch-yaw-------------------------*/

/*-------------四元数转旋转矩阵------------*/
void Qnb_2_Cbn(Quaternion *Qnb,  Matrix3x3 *Cbn)
{
	float q0q0;
	float q0q1;
	float q0q2;
	float q0q3;

	float q1q1;
	float q1q2;
	float q1q3;

	float q2q2;
	float q2q3;

	float q3q3;

	q0q0 = Qnb->q0 * Qnb->q0;
	q0q1 = Qnb->q0 * Qnb->q1;
	q0q2 = Qnb->q0 * Qnb->q2;
	q0q3 = Qnb->q0 * Qnb->q3;

	q1q1 = Qnb->q1 * Qnb->q1;
	q1q2 = Qnb->q1 * Qnb->q2;
	q1q3 = Qnb->q1 * Qnb->q3;

	q2q2 = Qnb->q2 * Qnb->q2;
	q2q3 = Qnb->q2 * Qnb->q3;

	q3q3 = Qnb->q3 * Qnb->q3;

	Cbn->T11 = q0q0 + q1q1 - q2q2 - q3q3;
	Cbn->T12 = 2*(q1q2 - q0q3);
	Cbn->T13 = 2*(q1q3 + q0q2);

	Cbn->T21 = 2*(q0q3 + q1q2);
	Cbn->T22 = q0q0 - q1q1 + q2q2 - q3q3;
	Cbn->T23 = 2*(q2q3 - q0q1);

	Cbn->T31 = 2*(q1q3 - q0q2);
	Cbn->T32 = 2*(q0q1 + q2q3);
	Cbn->T33 = q0q0 - q1q1 - q2q2 + q3q3;
}

//*-----旋转矩阵转欧拉角-----*/
void Cbn_2_Enb(Matrix3x3 *Cbn, EulerAng *Enb)
{
	Enb->roll  = atan2(Cbn->T32, Cbn->T33);
	Enb->pitch = -1.0 * asin(Cbn->T31);
	Enb->yaw = atan2(Cbn->T21, Cbn->T11);
}

/*-----四元数转欧拉角-----*/
void Qnb_2_Enb(Quaternion *Qnb, EulerAng *Enb)
{
	Matrix3x3 Cbn;
	Qnb_2_Cbn(Qnb, &Cbn);
	Cbn_2_Enb(&Cbn, Enb);
}

/*-----欧拉角转旋转矩阵（Cbn roll-pitch-yaw）-----*/
void Enb_2_Cbn(EulerAng *Enb, Matrix3x3 *Cbn)
{
	float phi, theta, psi;
	phi = Enb->roll;
	theta = Enb->pitch;
	psi = Enb->yaw;

	Cbn->T11 =  cos(theta) * cos(psi);
	Cbn->T12 = -cos(phi) * sin(psi) + sin(phi) * sin(theta) * cos(psi);
	Cbn->T13 =  sin(phi) * sin(psi) + cos(phi) * sin(theta) * cos(psi);

	Cbn->T21 =  cos(theta) * sin(psi);
	Cbn->T22 =  cos(phi) * cos(psi) + sin(phi) * sin(theta) * sin(psi);
	Cbn->T23 = -sin(phi) * cos(psi) + cos(phi) * sin(theta) * sin(psi);

	Cbn->T31 = -sin(theta);
	Cbn->T32 =  sin(phi) * cos(theta);
	Cbn->T33 =  cos(phi) * cos(theta);
}

/*-----欧拉角转旋转矩阵（Cbd roll-pitch）-----*/
void Enb_2_Cbd(EulerAng *Enb, Matrix3x3 *Cbd)
{
	float phi, theta;
	phi = Enb->roll;
	theta = Enb->pitch;

	Cbd->T11 = cos(theta);
	Cbd->T12 = sin(theta) * sin(phi);
	Cbd->T13 = sin(theta) * cos(phi);

	Cbd->T21 = 0;
	Cbd->T22 = cos(phi);
	Cbd->T23 = sin(-phi);

	Cbd->T31 = sin(-theta);
	Cbd->T32 = cos(theta) * sin(phi);
	Cbd->T33 = cos(theta) * cos(phi);
}


/*-----旋转矩阵转四元数-----*/
void Cbn_2_Qnb(Matrix3x3 *Cbn, Quaternion *Qnb)
{
	float tr;
	float s;
	float s1;

	tr = Cbn->T11 + Cbn->T22 + Cbn->T33;
	s = sqrt(tr +1);
	s1 = 0.5 / s;

	Qnb->q0 = 0.5 *s;
	Qnb->q1 = (Cbn->T32 - Cbn->T23) * s1;
	Qnb->q1 = (Cbn->T13 - Cbn->T31) * s1;
	Qnb->q1 = (Cbn->T21 - Cbn->T12) * s1;

	Quaternion_Normalize(Qnb);
}

/*-----欧拉角转四元数-----*/
void Enb_Qnb(EulerAng *Enb, Quaternion *Qnb)
{
	Matrix3x3 Cbn;
	Enb_2_Cbn(Enb, &Cbn);
	Cbn_2_Qnb(&Cbn, Qnb);
}

/*------------------------------N维矩阵计算------------------------------*/

/*-----N维矩阵相加-----*/
void MatrixN_Add_MatrixN(float *src1, int rows1,  int cols1,
						 float *src2, int rows2, int cols2,
						 float *des, int rows, int cols)
{
	arm_matrix_instance_f32 ma;
	arm_matrix_instance_f32 mb;
	arm_matrix_instance_f32 mc;

	ma.numRows = rows1;
	ma.numCols = cols1;
	ma.pData = (float *)src1;

	mb.numRows = rows2;
	mb.numCols = cols2;
	mb.pData = (float *)src2;

	mc.numRows = rows;
	mc.numCols = cols;
	mc.pData = (float *)des;

	arm_mat_add_f32(&ma, &mb, &mc);
}

/*-----N维矩阵相减-----*/
void MatrixN_Sub_MatrixN(float *src1, int rows1,  int cols1,
						 float *src2, int rows2, int cols2,
						 float *des, int rows, int cols)
{
	arm_matrix_instance_f32 ma;
	arm_matrix_instance_f32 mb;
	arm_matrix_instance_f32 mc;

	ma.numRows = rows1;
	ma.numCols = cols1;
	ma.pData = (float *)src1;

	mb.numRows = rows2;
	mb.numCols = cols2;
	mb.pData = (float *)src2;

	mc.numRows = rows;
	mc.numCols = cols;
	mc.pData = (float *)des;

	arm_mat_sub_f32(&ma, &mb, &mc);
}

/*-----N维矩阵相乘-----*/
void MatrixN_Mult_MatrixN(float *src1, int rows1,  int cols1,
						  float *src2, int rows2, int cols2,
						  float *des, int rows, int cols)
{
	arm_matrix_instance_f32 ma;
	arm_matrix_instance_f32 mb;
	arm_matrix_instance_f32 mc;

	ma.numRows = rows1;
	ma.numCols = cols1;
	ma.pData = (float *)src1;

	mb.numRows = rows2;
	mb.numCols = cols2;
	mb.pData = (float *)src2;

	mc.numRows = rows;
	mc.numCols = cols;
	mc.pData = (float *)des;

	arm_mat_mult_f32(&ma, &mb, &mc);
}

/*-----N维矩阵乘以系数-----*/
void MatrixN_Mult_Coeff(float *src1, int rows1,  int cols1,
						float co,
						float *des, int rows, int cols)
{
	arm_matrix_instance_f32 ma;
	arm_matrix_instance_f32 mc;

	ma.numRows = rows1;
	ma.numCols = cols1;
	ma.pData = (float *)src1;

	mc.numRows = rows;
	mc.numCols = cols;
	mc.pData = (float *)des;

	arm_mat_scale_f32(&ma, co, &mc);
}

/*-----N维矩阵求逆-----*/
bool MatrixN_Inverse(float *src1, int rows1,  int cols1,
					 float *des, int rows, int cols)
{
	arm_matrix_instance_f32 ma;
	arm_matrix_instance_f32 mb;

	ma.numRows = rows1;
	ma.numCols = cols1;
	ma.pData = (float *)src1;

	mb.numRows = rows;
	mb.numCols = cols;
	mb.pData = (float *)des;

	arm_status sta;
	sta = arm_mat_inverse_f32(&ma, &mb);

	if(sta != ARM_MATH_SUCCESS)
	{
		syslog(LOG_ERR, "【imu】: ERROR Matrix inverse failed:  %d\n", sta);
		return false;
	}

	return true;
}

/*-----N维矩阵转置-----*/
void MatrixN_Trans(float *src1, int rows1,  int cols1,
				   float *des, int rows, int cols)
{
	arm_matrix_instance_f32 ma;
	arm_matrix_instance_f32 mb;

	ma.numRows = rows1;
	ma.numCols = cols1;
	ma.pData = (float *)src1;

	mb.numRows = rows;
	mb.numCols = cols;
	mb.pData = (float *)des;

	arm_mat_trans_f32(&ma, &mb);
}






