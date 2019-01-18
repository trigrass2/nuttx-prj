/****************************************************************************
 * examples/arm_basic_math_example/arm_basic_math_example_main.c
 *
 *   Copyright (C) 2008, 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

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

#include <DSP_Lib/arm_math.h>
/****************************************************************************
 * Public Functions
 ****************************************************************************/
/*
*********************************************************************************************************
* 函 数 名: DSP_MatInit
* 功能说明: 矩阵数据初始化
* 形 参：无
* 返 回 值: 无
*********************************************************************************************************
*/
static void DSP_MatInit(void)
{
	uint8_t i;
	/****浮点数数组******************************************************************/
	float32_t pDataA[9] = {1.1f, 1.1f, 2.1f, 2.1f, 3.1f, 3.1f, 4.1f, 4.1f, 5.1f};
	arm_matrix_instance_f32 pSrcA; //3行3列数据

	/****定点数Q31数组******************************************************************/
	q31_t pDataA1[9] = {1, 1, 2, 2, 3, 3, 4, 4, 5};
	arm_matrix_instance_q31 pSrcA1; //3行3列数据

	/****定点数Q15数组******************************************************************/
	q15_t pDataA2[9] = {1, 1, 2, 2, 3, 3, 4, 4, 5};
	arm_matrix_instance_q15 pSrcA2; //3行3列数据

	/****浮点数***********************************************************************/
	printf("****float******************************************\r\n");
	arm_mat_init_f32(&pSrcA, 3,3, pDataA);
	for(i = 0; i < 9; i++){
		printf("pDataA[%d] = %3.6f\r\n", i, pDataA[i]);
	}

	/****定点数Q31***********************************************************************/
	printf("****float******************************************\r\n");
	arm_mat_init_q31(&pSrcA1, 3,3, pDataA1);
	for(i = 0; i < 9; i++){
		printf("pDataA1[%d] = %d\r\n", i, pDataA1[i]);
	}

	/****定点数Q15***********************************************************************/
	printf("****float******************************************\r\n");
	arm_mat_init_q15(&pSrcA2, 3,3, pDataA2);
	for(i = 0; i < 9; i++){
		printf("pDataA2[%d] = %d\r\n", i, pDataA2[i]);
	}
}


/*
*********************************************************************************************************
* 函 数 名: DSP_MatAdd
* 功能说明: 矩阵求和
* 形 参：无
* 返 回 值: 无
*********************************************************************************************************
*/
static void DSP_MatAdd(void)
{
	uint8_t i;

	/****浮点数数组******************************************************************/
	float32_t pDataA[9] = {1.1f, 1.1f, 2.1f, 2.1f, 3.1f, 3.1f, 4.1f, 4.1f, 5.1f};
	float32_t pDataB[9] = {1.1f, 1.1f, 2.1f, 2.1f, 3.1f, 3.1f, 4.1f, 4.1f, 5.1f};
	float32_t pDataDst[9];
	arm_matrix_instance_f32 pSrcA; //3行3列数据
	arm_matrix_instance_f32 pSrcB; //3行3列数据
	arm_matrix_instance_f32 pDst;

	/****定点数Q31数组******************************************************************/
	q31_t pDataA1[9] = {1, 1, 2, 2, 3, 3, 4, 4, 5};
	q31_t pDataB1[9] = {1, 1, 2, 2, 3, 3, 4, 4, 5};
	q31_t pDataDst1[9];
	arm_matrix_instance_q31 pSrcA1; //3行3列数据
	arm_matrix_instance_q31 pSrcB1; //3行3列数据
	arm_matrix_instance_q31 pDst1;

	/****定点数Q15数组******************************************************************/
	q15_t pDataA2[9] = {1, 1, 2, 2, 3, 3, 4, 4, 5};
	q15_t pDataB2[9] = {1, 1, 2, 2, 3, 3, 4, 4, 5};
	q15_t pDataDst2[9];
	arm_matrix_instance_q15 pSrcA2; //3行3列数据
	arm_matrix_instance_q15 pSrcB2; //3行3列数据
	arm_matrix_instance_q15 pDst2;

	/****浮点数***********************************************************************/
	pSrcA.numCols = 3;
	pSrcA.numRows = 3;
	pSrcA.pData = pDataA;
	pSrcB.numCols = 3;
	pSrcB.numRows = 3;
	pSrcB.pData = pDataB;
	pDst.numCols = 3;
	pDst.numRows = 3;
	pDst.pData = pDataDst;

	for(i = 0; i < 9; i++)
	{
		printf("pDataA[%d] = %3.6f, pDataB[%d] = %3.6f\r\n", i, pDataA[i], i, pDataB[i]);
	}

	printf("****float******************************************\r\n");
	arm_mat_add_f32(&pSrcA, &pSrcB, &pDst);

	for(i = 0; i < 9; i++)
	{
		printf("pDataA[%d] = %3.6f, pDataB[%d] = %3.6f\r\n", i, pDataA[i], i, pDataB[i]);
	}
	for(i = 0; i < 9; i++)
	{
		printf("pDataDst[%d] = %3.6f\r\n", i, pDataDst[i]);
	}
	/****定点数Q31***********************************************************************/
	pSrcA1.numCols = 3;
	pSrcA1.numRows = 3;
	pSrcA1.pData = pDataA1;
	pSrcB1.numCols = 3;
	pSrcB1.numRows = 3;
	pSrcB1.pData = pDataB1;
	pDst1.numCols = 3;
	pDst1.numRows = 3;
	pDst1.pData = pDataDst1;
	printf("****Q31******************************************\r\n");
	arm_mat_add_q31(&pSrcA1, &pSrcB1, &pDst1);
	for(i = 0; i < 9; i++)
	{
		printf("pDataDst1[%d] = %d\r\n", i, pDataDst1[i]);
	}
	/****定点数Q15***********************************************************************/
	pSrcA2.numCols = 3;
	pSrcA2.numRows = 3;
	pSrcA2.pData = pDataA2;
	pSrcB2.numCols = 3;
	pSrcB2.numRows = 3;
	pSrcB2.pData = pDataB2;
	pDst2.numCols = 3;
	pDst2.numRows = 3;
	pDst2.pData = pDataDst2;
	printf("****Q15******************************************\r\n");
	arm_mat_add_q15(&pSrcA2, &pSrcB2, &pDst2);
	for(i = 0; i < 9; i++)
	{
		printf("pDataDst2[%d] = %d\r\n", i, pDataDst2[i]);
	}
}

/*
*********************************************************************************************************
* 函 数 名: DSP_MatInverse
* 功能说明: 求逆矩阵
* 形 参：无
* 返 回 值: 无
*********************************************************************************************************
*/
static void DSP_MatInverse(void)
{
	uint8_t i;

	/****浮点数数组******************************************************************/
	float32_t pDataA[9] = {1.1f, 1.1f, 2.1f, 2.1f, 3.1f, 3.1f, 4.1f, 4.1f, 5.1f};
	float32_t pDataB[9];
	arm_matrix_instance_f32 pSrcA; //3行3列数据
	arm_matrix_instance_f32 pSrcB; //3行3列数据;

	for(i = 0; i < 9; i++)
	{
		printf("pDataA[%d] = %3.6f\r\n", i, pDataA[i]);
	}
	printf("\n\n");

	/****浮点数***********************************************************************/
	pSrcA.numCols = 3;
	pSrcA.numRows = 3;
	pSrcA.pData = pDataA;
	pSrcB.numCols = 3;
	pSrcB.numRows = 3;
	pSrcB.pData = pDataB;
	arm_mat_inverse_f32(&pSrcA, &pSrcB);

	for(i = 0; i < 9; i++)
	{
		printf("pDataA[%d] = %3.6f\r\n", i, pDataA[i]);
	}

	printf("\n\n");

	for(i = 0; i < 9; i++)
	{
		printf("pDataB[%d] = %3.6f\r\n", i, pDataB[i]);
	}

	printf("\n\n");
}


/*
*********************************************************************************************************
* 函 数 名: DSP_MatSub
* 功能说明: 矩阵减法
* 形 参：无
* 返 回 值: 无
*********************************************************************************************************
*/
static void DSP_MatSub(void)
{
	uint8_t i;
	/****浮点数数组******************************************************************/
	float32_t pDataA[9] = {1.1f, 1.1f, 2.1f, 2.1f, 3.1f, 3.1f, 4.1f, 4.1f, 5.1f};
	float32_t pDataB[9] = {1.0f, 1.0f, 2.0f, 2.0f, 3.0f, 3.0f, 4.0f, 4.0f, 5.0f};
	float32_t pDataDst[9];
	arm_matrix_instance_f32 pSrcA; //3行3列数据
	arm_matrix_instance_f32 pSrcB; //3行3列数据
	arm_matrix_instance_f32 pDst;
	/****定点数Q31数组******************************************************************/
	q31_t pDataA1[9] = {1, 1, 2, 2, 3, 3, 4, 4, 5};
	q31_t pDataB1[9] = {2, 2, 2, 2, 2, 2, 2, 2, 2};
	q31_t pDataDst1[9];
	arm_matrix_instance_q31 pSrcA1; //3行3列数据
	arm_matrix_instance_q31 pSrcB1; //3行3列数据
	arm_matrix_instance_q31 pDst1;
	/****定点数Q15数组******************************************************************/
	q15_t pDataA2[9] = {1, 1, 2, 2, 3, 3, 4, 4, 5};
	q15_t pDataB2[9] = {2, 2, 2, 2, 23, 2, 2, 2, 2};
	q15_t pDataDst2[9];
	arm_matrix_instance_q15 pSrcA2; //3行3列数据
	arm_matrix_instance_q15 pSrcB2; //3行3列数据
	arm_matrix_instance_q15 pDst2;
	/****浮点数***********************************************************************/
	pSrcA.numCols = 3;
	pSrcA.numRows = 3;
	pSrcA.pData = pDataA;
	pSrcB.numCols = 3;
	pSrcB.numRows = 3;
	pSrcB.pData = pDataB;
	pDst.numCols = 3;
	pDst.numRows = 3;
	pDst.pData = pDataDst;
	printf("****float******************************************\r\n");

	for(i = 0; i < 9; i++)
	{
		printf("pDataA[%d] = %3.6f, pDataB[%d] = %3.6f\r\n", i, pDataA[i], i, pDataB[i]);
	}
	arm_mat_mult_f32(&pSrcA, &pSrcB, &pDst);

	for(i = 0; i < 9; i++)
		{
			printf("pDataA[%d] = %3.6f, pDataB[%d] = %3.6f\r\n", i, pDataA[i], i, pDataB[i]);
		}
	for(i = 0; i < 9; i++)
	{
		printf("pDataDst[%d] = %3.6f\r\n", i, pDataDst[i]);
	}
	/****定点数Q31***********************************************************************/
	pSrcA1.numCols = 3;
	pSrcA1.numRows = 3;
	pSrcA1.pData = pDataA1;
	pSrcB1.numCols = 3;
	pSrcB1.numRows = 3;
	pSrcB1.pData = pDataB1;
	pDst1.numCols = 3;
	pDst1.numRows = 3;
	pDst1.pData = pDataDst1;
	printf("****Q31******************************************\r\n");
	arm_mat_sub_q31(&pSrcA1, &pSrcB1, &pDst1);
	for(i = 0; i < 9; i++)
	{
		printf("pDataDst1[%d] = %d\r\n", i, pDataDst1[i]);
	}
	/****定点数Q15***********************************************************************/
	pSrcA2.numCols = 3;
	pSrcA2.numRows = 3;
	pSrcA2.pData = pDataA2;
	pSrcB2.numCols = 3;
	pSrcB2.numRows = 3;
	pSrcB2.pData = pDataB2;
	pDst2.numCols = 3;
	pDst2.numRows = 3;
	pDst2.pData = pDataDst2;
	printf("****Q15******************************************\r\n");
	arm_mat_sub_q15(&pSrcA2, &pSrcB2, &pDst2);
	for(i = 0; i < 9; i++)
	{
		printf("pDataDst2[%d] = %d\r\n", i, pDataDst2[i]);
	}
}

static void usage(void *arg)
{
	printf("This set of functions operates on complex data vectors.\n");
	printf("  [-i]: Matrix initialize\n");
	printf("  [-a]: Matrix add\n");
	printf("  [-s]: Matrix sub\n");
	printf("  [-I]: Matrix Inverse\n");
	printf("  [-h]: help information\n");

}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int matrix_math_example_main(int argc, char *argv[])
#endif
{
	 int ret;
	 /* invalid ParamSet */
	 if (argc != 2){
	   printf("Invalid options format: %s\n", argv[1]);
	   usage(0);
  	   exit(EXIT_FAILURE);
	 }

	 if (*argv[1] != '-'){
	   printf("Invalid options format: %s\n", argv[1]);
	   usage(0);
  	   exit(EXIT_FAILURE);
	 }

	 switch(*(++argv[1]))
	 {
	 	 case'i':	/* Matrix initialize */
	 	 {
	 		DSP_MatInit();
			ret = OK;
	 	 }break;

	 	 case'a':	/* Matrix add */
	 	 {
	 		DSP_MatAdd();
			 ret = OK;
	 	 }break;

	 	 case's':	/* Matrix sub */
	 	 {
	 		DSP_MatSub();
			 ret = OK;
	 	 }break;

	 	 case'I':	/* Matrix Inverse */
	 	 {
	 		DSP_MatInverse();
			 ret = OK;
	 	 }break;

	 	 case'h':	/* Help information */
	 	 {
			 usage(0);
			 ret = OK;
	 	 }break;

	 	 default:
	 	 {
			 usage(0);
			 ret = OK;
	 	 }break;



	 }

  return ret;
}


















