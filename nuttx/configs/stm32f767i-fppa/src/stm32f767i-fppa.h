/****************************************************************************************************
 * configs/stm32f769i-disco/src/stm32f769i-disco.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
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
 ****************************************************************************************************/

#ifndef __CONFIGS_STM32F769I_DISCO_SRC_STM32F769I_DISCO__H
#define __CONFIGS_STM32F769I_DISCO_SRC_STM32F769I_DISCO__H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/
/* procfs File System */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define STM32_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define STM32_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

/*
 * The STM32F767I-FPPA has one user controllable LED: LD3.
 * LD3 is controlled by PI3,LD3 is on when PA12 is high.
 */

#define GPIO_LD3           (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET |  GPIO_PORTI | GPIO_PIN3)
#define GPIO_BUZZER0       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET |  GPIO_PORTB | GPIO_PIN7)
/*
 * Pushbutton B1, labelled "User", is connected to GPIO PA0.  A high value will be sensed when the
 * button is depressed. Note that the EXTI interrupt is configured.
 */
#ifdef CONFIG_ARCH_BUTTONS
#define GPIO_BTN_USER      (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | GPIO_PORTH | GPIO_PIN8)
#endif
/* Sporadic scheduler instrumentation. This configuration has been used for evaluating the NuttX
 * sporadic scheduler.  In this evaluation, two GPIO outputs are used.  One indicating the priority
 * (high or low) of the sporadic thread and one indicating where the thread is running or not.
 *
 * There is nothing special about the pin selections:
 *
 *   Arduino D2 PJ1 - Indicates priority1  //优先级指示
 *   Arduino D4 PJ0 - Indicates that the thread is running //线程运行状态指示
 */

//USER_LED_1 PI3    //USER_LED_1 is on when PI3 is low
//USER_LED_2 PI8    //USER_LED_1 is on when PI8 is low

#ifdef CONFIG_SPORADIC_INSTRUMENTATION

//#define GPIO_SCHED_HIGHPRI (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTJ | GPIO_PIN1)

//#define GPIO_SCHED_RUNNING (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTJ | GPIO_PIN0)

#define GPIO_SCHED_RUNNING (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTI | GPIO_PIN8)

#endif


#define SDIO_SLOTNO        0
#define SDIO_MINOR         0
/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/****************************************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the stm32f769i-disco board.
 *
 ****************************************************************************************************/

void weak_function stm32_spidev_initialize(void);

 /****************************************************************************
  * Name: stm32_sdio_initialize
  *
  * Description:
  *   Initialize SDIO-based MMC/SD card support
  *
  ****************************************************************************/

 #if !defined(CONFIG_DISABLE_MOUNTPOINT) && defined(CONFIG_STM32F7_SDMMC1)
 int stm32_sdio_initialize(void);
 #endif
 /************************************************************************************
  * Name: stm32_dma_alloc_init
  *
  * Description:
  *   Called to create a FAT DMA allocator
  *
  * Returned Value:
  *   0 on success or -ENOMEM
  *
  ************************************************************************************/

 #if defined (CONFIG_FAT_DMAMEMORY)
 int stm32_dma_alloc_init(void);
 #endif
 /****************************************************************************************************
  * Name: board_app_initialize
  *
  * Description:
  *   Called to configure SPI chip select GPIO pins for the stm32f769i-disco board.
  *
  ****************************************************************************************************/

// int board_app_initialize(uintptr_t arg);

/****************************************************************************************************
 * Name: arch_sporadic_initialize
 *
 * Description:
 *   This configuration has been used for evaluating the NuttX sporadic scheduler.
 *
 ****************************************************************************************************/

#ifdef CONFIG_SPORADIC_INSTRUMENTATION
void arch_sporadic_initialize(void);
#endif


/****************************************************************************************************
 * Name: stm32_can_setup
 *
 * Description:
 *   This configuration has been used for evaluating the NuttX sporadic scheduler.
 *
 ****************************************************************************************************/
#ifdef CONFIG_CAN
int stm32_can_setup(void);
#endif

/****************************************************************************************************
 * Name: stm32_timer_setup
 *
 * Description:
 *   This configuration has been used for evaluating the NuttX sporadic scheduler.
 *
 ****************************************************************************************************/
#ifdef CONFIG_TIMER
int stm32_timer_setup(void);
#endif

/****************************************************************************************************
 * Name: stm32_spi_setup
 *
 * Description:
 *   This configuration has been used for evaluating the NuttX sporadic scheduler.
 *
 ****************************************************************************************************/
#ifdef CONFIG_SPI
int stm32_spi_setup(void);
#endif


#ifdef CONFIG_PWM
int stm32_pwm_setup(void);
#endif

int stm32_elmo_db4x_setup(void);

/****************************************************************************
 * Name: stm32_hall_setup
 *
 * Description:
 *   stm32_hall_setup() must be called to initialize hall resources.
 *
 ****************************************************************************/
#ifdef CONFIG_HALLS
int stm32_hall_setup(void);
#endif

/************************************************************************************
 * Name: stm32_adc_manager_setup
 *
 * Description:
 *  Initialize ADC and register the ADC device
 *
 ************************************************************************************/
#ifdef CONFIG_ADC
int stm32_adc_manager_setup(void);
#endif

/************************************************************************************
 * Name: stm32_hmc6343_setup
 *
 * Description:
 *  Initialize I2C and register the HMC5343 device
 *
 ************************************************************************************/
#if (defined CONFIG_I2C) && (defined CONFIG_SENSORS_HMC6343)
int stm32_hmc6343_setup(void);
#endif


/************************************************************************************
 * Name: stm32_i2c_xx24xx_setup
 *
 * Description:
 *  Initialize I2C and register the xx24xx EEPROM device
 *
 ************************************************************************************/
#if (defined CONFIG_I2C) && (defined CONFIG_I2C_EE_24XX)
int stm32_i2c_xx24xx_setup(void);
#endif


/************************************************************************************
 * Name: stm32_n25qxxx_setup
 *
 * Description:
 *  Initialize QUADSPI and register the N25QXXX device
 *
 ************************************************************************************/
#if (defined CONFIG_STM32F7_QUADSPI) && (defined CONFIG_MTD_N25QXXX)
int stm32_n25qxxx_setup(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_STM32F769I_DISCO_SRC_STM32F769I_DISCO_H */

