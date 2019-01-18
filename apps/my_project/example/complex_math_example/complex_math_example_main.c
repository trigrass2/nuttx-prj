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
* 函 数 名: DSP_CONJ
* 功能说明: 浮点数复数共轭
* 形 参：无
* 返 回 值: 无
* 数组pSrc和pDst中存储的数据格式是（实部，虚部，实部，虚部……………），一定要按照这个顺序存储数据;
* 比如数据1-j，j，2+3j这个三个数在数组中的存储格式就是：pSrc[6] = {1, -1, 0, 1, 2, 3}。
* （注意第三个数据是0）。输出结果的实部和虚部是分开的。
*********************************************************************************************************
*/
static void DSP_CONJ(void)
{
	uint8_t i;
	float32_t pSrc[10] = {1.1f, 1.1f, 2.1f, 2.1f, 3.1f, 3.1f, 4.1f, 4.1f, 5.1f, 5.1f};
	float32_t pDst[10] = {0.0f};
	q31_t pSrc1[10] = {1, 1, 2, 2, 3, 3, 4, 4, 5, 5};
	q31_t pDst1[10] = {0};
	q15_t pSrc2[10] = {1, 1, 2, 2, 3, 3, 4, 4, 5, 5};
	q15_t pDst2[10] = {0};
	/***浮点数共轭*******************************************************************************/
	arm_cmplx_conj_f32(pSrc, pDst, 5);
	printf("***float conjugate********************************************\r\n");
	for(i = 0; i < 5; i++){
		printf("pSrc[%d] = %3.6f %3.6fj pDst[%d] = %3.6f %3.6fj\r\n", i, pSrc[2*i], pSrc[2*i+1], i, pDst[2*i],
		pDst[2*i+1]);
	}
	/***定点数共轭Q31*******************************************************************************/
	printf("***Q31 conjugate*****************************************\r\n");
	arm_cmplx_conj_q31(pSrc1, pDst1, 5);
	for(i = 0; i < 5; i++){
		printf("pSrc1[%d] = %d %dj pDst1[%d] = %d %dj\r\n", i, pSrc1[2*i], pSrc1[2*i+1], i, pDst1[2*i],
		pDst1[2*i+1]);
	}
	/***定点数共轭Q15*******************************************************************************/
	printf("***Q15 conjugate*****************************************\r\n");
	arm_cmplx_conj_q15(pSrc2, pDst2, 5);
	for(i = 0; i < 5; i++){
		printf("pSrc2[%d] = %d %dj pDst2[%d] = %d %dj\r\n", i, pSrc2[2*i], pSrc2[2*i+1], i, pDst2[2*i],
		pDst2[2*i+1]);
	}
}

/*
*********************************************************************************************************
* 函 数 名: DSP_CmplxDotProduct
* 功能说明: 浮点数cos和sin计算
* 形 参：无
* 返 回 值: 无
* 数组pSrc和pDst中存储的数据格式是（实部，虚部，实部，虚部……………），一定要按照这个顺序存储数据;
* 比如数据1-j，j，2+3j这个三个数在数组中的存储格式就是：pSrc[6] = {1, -1, 0, 1, 2, 3}。
*（注意第三个数据是0）。输出结果的实部和虚部是分开的。
*********************************************************************************************************
*/
static void DSP_CmplxDotProduct(void)
{
	float32_t pSrcA[10] = {1.1f, 1.1f, 2.1f, 2.1f, 3.1f, 3.1f, 4.1f, 4.1f, 5.1f, 5.1f};
	float32_t pSrcB[10] = {1.1f, 1.1f, 2.1f, 2.1f, 3.1f, 3.1f, 4.1f, 4.1f, 5.1f, 5.1f};
	float32_t realResult = 0.0f;
	float32_t imagResult = 0.0f;
	q31_t pSrcA1[10] = {1*268435456, 1*268435456, 2*268435456, 2*268435456, 3*268435456, 3*268435456,\
						4*268435456, 4*268435456, 5*268435456, 5*268435456};
	q31_t pSrcB1[10] = {1*268435456, 1*268435456, 2*268435456, 2*268435456, 3*268435456, 3*268435456,\
						4*268435456, 4*268435456, 5*268435456, 5*268435456};
	q63_t realResult1 = 0;
	q63_t imagResult1 = 0;
	q15_t pSrcA2[10] = {5000, 10000, 15000, 20000, 25000, 5000, 10000, 15000, 20000, 25000};
	q15_t pSrcB2[10] = {5000, 10000, 15000, 20000, 25000, 5000, 10000, 15000, 20000, 25000};
	q31_t realResult2 = 0;
	q31_t imagResult2 = 0;
	/***浮点数点乘*******************************************************************************/
	arm_cmplx_dot_prod_f32(pSrcA, pSrcB, 5, &realResult, &imagResult);
	printf("arm_cmplx_dot_prod_f32:realResult = %3.6f imagResult = %3.6f\r\n", realResult, imagResult);
	/***定点数点乘Q31*******************************************************************************/
	arm_cmplx_dot_prod_q31(pSrcA1, pSrcB1, 5, &realResult1, &imagResult1);
	printf("arm_cmplx_dot_prod_q31:realResult1 = %lld imagResult1 = %lld\r\n", realResult1, imagResult1);
	/***定点数点乘Q15*******************************************************************************/
	arm_cmplx_dot_prod_q15(pSrcA2, pSrcB2, 5, &realResult2, &imagResult2);
	printf("arm_cmplx_dot_prod_q15:realResult2 = %d imagResult2 = %d\r\n", realResult2, imagResult2);
}

/*
*********************************************************************************************************
* 函 数 名: DSP_CmplxMag
* 功能说明: 浮点数cos和sin计算
* 形 参：无
* 返 回 值: 无
* 数组pSrc和pDst中存储的数据格式是（实部，虚部，实部，虚部……………），一定要按照这个顺序存储数据;
* 比如数据1-j，j，2+3j这个三个数在数组中的存储格式就是：pSrc[6] = {1, -1, 0, 1, 2, 3}。
*（注意第三个数据是0）。输出结果的实部和虚部是分开的。
*********************************************************************************************************
*/
static void DSP_CmplxMag(void)
{
	uint8_t i;
	float32_t pSrc[10] = {1.1f, 1.1f, 2.1f, 2.1f, 3.1f, 3.1f, 4.1f, 4.1f, 5.1f, 5.1f};
	float32_t pDst[10] = {0};
	q31_t pSrc1[10] = {1*268435456, 1*268435456, 2*268435456, 2*268435456, 3*268435456, 3*268435456,
	4*268435456, 4*268435456, 5*268435456, 5*268435456};
	q31_t pDst1[10] = {0};
	q15_t pSrc2[10] = {5000, 10000, 15000, 20000, 25000, 5000, 10000, 15000, 20000, 25000};
	q15_t pDst2[10] = {0};
	/***浮点数求模*******************************************************************************/
	arm_cmplx_mag_f32(pSrc, pDst, 5);
	for(i = 0; i < 5; i++){
		printf("pDst[%d] = %3.6f\r\n", i, pDst[i]);
	}
	/***定点数求模Q31*******************************************************************************/
	arm_cmplx_mag_q31(pSrc1, pDst1, 5);
	for(i = 0; i < 5; i++){
		printf("pDst1[%d] = %d\r\n", i, pDst1[i]);
	}
	/***定点数求模Q15*******************************************************************************/
	arm_cmplx_mag_q15(pSrc2, pDst2, 5);
	for(i = 0; i < 5; i++){
		printf("pDst2[%d] = %d\r\n", i, pDst2[i]);
	}
}

static void usage(void *arg)
{
	printf("This set of functions operates on complex data vectors.\n");
	printf("  [-c]: Complex Conjugate\n");
	printf("  [-d]: Complex DotProduct\n");
	printf("  [-m]: Complex Magnitude\n");
	printf("  [-h]: help information\n");

}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int complex_math_example_main(int argc, char *argv[])
#endif
{
	 int option;
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
	 	 case'c':	/* Complex Conjugate */
	 	 {
			 DSP_CONJ();
			 ret = OK;
	 	 }break;

	 	 case'd':	/* Complex DotProduct */
	 	 {
			 DSP_CmplxDotProduct();
			 ret = OK;
	 	 }break;

	 	 case'm':	/* Complex Magnitude */
	 	 {
			 DSP_CmplxMag();
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


















