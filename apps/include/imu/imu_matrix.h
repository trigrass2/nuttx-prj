#ifndef   __IMU_MATRIX_H_INCLUDED

#define   __IMU_MATRIX_H_INCLUDED



typedef union
{
	struct
	{
		float x;
		float y;
		float z;
	};
	float axis[3];
} Vector3f;

typedef union
{
	struct
	{
		double x;
		double y;
		double z;
	};
	double axis[3];
} Vector3d;


typedef union
{
	struct
	{
		float q0;
		float q1;
		float q2;
		float q3;
	};

	float axis[4];
} Quaternion;


typedef union
{
	struct
	{
		float roll;
		float pitch;
		float yaw;
	};

	float axis[3];
} EulerAng;


typedef union
{
	struct
	{
		float T11, T12, T13;
		float T21, T22, T23;
		float T31, T32, T33;
	};
	float axis[3][3];
}Matrix3x3;

typedef union
{
	struct
	{
		float T11, T12, T13, T14;
		float T21, T22, T23, T24;
		float T31, T32, T33, T34;
		float T41, T42, T43, T44;
	};
	float axis[4][4];
}Matrix4x4;



typedef union
{
	struct
	{
		float roll;
		float pitch;
		float yaw;
	};

	float axis[3];
} Attitude;


extern void  Vector3f_Add_Vector3f(Vector3f  *a , Vector3f  *b,  Vector3f  *c);
extern void  Vector3f_Sub_Vector3f(Vector3f  *a , Vector3f  *b,  Vector3f  *c);
extern void  Vector3f_Mult_Vector3f(Vector3f  *a , Vector3f  *b,  Vector3f  *c);
extern void  Vector3f_Mult_Coeff(Vector3f  *a , float b,  Vector3f  *c);
extern bool  Vector3f_Normalize(Vector3f  *a);
extern void  Vector3f_Cross_Vector3f(Vector3f  *a , Vector3f  *b,  Vector3f  *c);
extern void  Vector3f_Cross(Vector3f  *a , Matrix3x3  *b);

extern void  Vector3d_Add_Vector3d(Vector3d  *a , Vector3d  *b,  Vector3d  *c);
extern void  Vector3d_Sub_Vector3d(Vector3d  *a , Vector3d  *b,  Vector3d  *c);
extern void  Vector3d_Mult_Vector3d(Vector3d  *a , Vector3d  *b,  Vector3d  *c);
extern void  Vector3d_Mult_Coeff(Vector3d  *a , float b,  Vector3d  *c);
extern bool  Vector3d_Normalize(Vector3d  *a);
extern void  Vector3d_Cross_Vector3d(Vector3d  *a , Vector3d  *b,  Vector3d  *c);
extern void  Vector3d_Cross(Vector3d  *a , Matrix3x3  *b);


extern void Quaternion_Mult_Coeff(Quaternion *Q_src, float b, Quaternion *Q_des);
extern bool Quaternion_Normalize(Quaternion *Q);

extern void  Matrix3x3_Mult_Vector3f(Matrix3x3  *a , Vector3f  *b,  Vector3f  *c);
extern void  Matrix3x3_Mult_Matrix3x3(Matrix3x3  *a , Matrix3x3  *b,  Matrix3x3  *c);
extern void  Matrix3x3_Trans(Matrix3x3 *a,  Matrix3x3 *b);
extern void  Matrix3x3_One(Matrix3x3 *a);

extern void  Matrix4x4_Add_Matrix4x4(Matrix4x4  *a , Matrix4x4  *b,  Matrix4x4  *c);
extern void  Matrix4x4_Sub_Matrix4x4(Matrix4x4  *a , Matrix4x4  *b,  Matrix4x4  *c);
extern void  Matrix4x4_Mult_Matrix4x4(Matrix4x4  *a , Matrix4x4  *b,  Matrix4x4  *c);
extern void  Matrix4x4_Mult_Coeff(Matrix4x4  *a , float  b,  Matrix4x4  *c);
extern void  Matrix4x4_Trans(Matrix4x4 *a,  Matrix4x4 *b);
extern void  Matrix4x4_One(Matrix4x4 *a);


extern void Qnb_2_Cbn(Quaternion *Qnb,  Matrix3x3 *Cbn);
extern void Cbn_2_Enb(Matrix3x3 *Cbn, EulerAng *Enb);
extern void Qnb_2_Enb(Quaternion *Qnb, EulerAng *Enb);

extern void Enb_2_Cbn(EulerAng *Enb, Matrix3x3 *Cbn);
extern void Enb_2_Cbd(EulerAng *Enb, Matrix3x3 *Cbd);
extern void Cbn_2_Qnb(Matrix3x3 *Cbn, Quaternion *Qnb);
extern void Enb_Qnb(EulerAng *Enb, Quaternion *Qnb);

extern void MatrixN_Add_MatrixN(float *src1, int rows1,  int cols1,
								float *src2, int rows2, int cols2,
								float *des, int rows, int cols);

extern void MatrixN_Sub_MatrixN(float *src1, int rows1,  int cols1,
								float *src2, int rows2, int cols2,
								float *des, int rows, int cols);

extern void MatrixN_Mult_MatrixN(float *src1, int rows1,  int cols1,
							 	 float *src2, int rows2, int cols2,
							     float *des, int rows, int cols);

extern void MatrixN_Mult_Coeff(float *src1, int rows1,  int cols1,
							   float co,
							   float *des, int rows, int cols);

extern bool MatrixN_Inverse(float *src1, int rows1,  int cols1,
							float *des, int rows, int cols);

extern void MatrixN_Trans(float *src1, int rows1,  int cols1,
						  float *des, int rows, int cols);



#endif

