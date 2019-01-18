/************************************************************************************
 * configs/stm32f769i-disco/include/board.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __CONFIG_STM32F769I_DISCO_INCLUDE_BOARD_H
#define __CONFIG_STM32F769I_DISCO_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

#include "stm32_rcc.h"
#if defined(CONFIG_STM32F7_SDMMC1) || defined(CONFIG_STM32F7_SDMMC2)
#  include "stm32_sdmmc.h"
#endif

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/
/* The STM32F7 Discovery board provides the following clock sources:
 *
 *   X2:  25 MHz oscillator for STM32F769NIH6 microcontroller and Ethernet PHY.
 *   X1:  32.768 KHz crystal for STM32F769NIH6 embedded RTC
 *
 * So we have these clock source available within the STM32
 *
 *   HSI: 16 MHz RC factory-trimmed
 *   LSI: 32 KHz RC
 *   HSE: On-board crystal frequency is 25MHz
 *   LSE: 32.768 kHz
 */

#define STM32_BOARD_XTAL        25000000ul

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768

/* Main PLL Configuration.
 *
 * PLL source is HSE = 25,000,000
 *
 * PLL_VCO = (STM32_HSE_FREQUENCY / PLLM) * PLLN
 * Subject to:
 *
 *     2 <= PLLM <= 63
 *   192 <= PLLN <= 432
 *   192 MHz <= PLL_VCO <= 432MHz
 *
 * SYSCLK  = PLL_VCO / PLLP
 * Subject to
 *
 *   PLLP = {2, 4, 6, 8}
 *   SYSCLK <= 216 MHz
 *
 * USB OTG FS, SDMMC and RNG Clock = PLL_VCO / PLLQ
 * Subject to
 *   The USB OTG FS requires a 48 MHz clock to work correctly. The SDMMC
 *   and the random number generator need a frequency lower than or equal
 *   to 48 MHz to work correctly.
 *
 * 2 <= PLLQ <= 15
 */

#if defined(CONFIG_STM32F7_OTGFS)
/* USB OTG FS clock (= SDMMCCLK = RNGCLK) must be 48 MHz
 *
 * PLL_VCO = (25,000,000 / 25) * 384 = 384 MHz
 * SYSCLK  = 384 MHz / 2 = 192 MHz
 * USB OTG FS, SDMMC and RNG Clock = 384 MHz / 8 = 48MHz
 * DSI CLK = PLL_VCO / PLLR = 384 / 7 = 54,86 MHz
 */

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(25)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(384)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_2
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(8)
#define STM32_PLLCFG_PLLR       RCC_PLLCFG_PLLR(7)

#define STM32_VCO_FREQUENCY     ((STM32_HSE_FREQUENCY / 25) * 384)
#define STM32_SYSCLK_FREQUENCY  (STM32_VCO_FREQUENCY / 2)
#define STM32_OTGFS_FREQUENCY   (STM32_VCO_FREQUENCY / 8)

#elif defined(CONFIG_STM32F7_SDMMC1) || defined(CONFIG_STM32F7_SDMMC2) || defined(CONFIG_STM32F7_RNG)
/* SDMMCCLK (= USB OTG FS clock = RNGCLK) should be <= 48MHz
 *
 * PLL_VCO = (25,000,000 / 25) * 432 = 432 MHz
 * SYSCLK  = 432 MHz / 2 = 216 MHz
 * USB OTG FS, SDMMC and RNG Clock = 432 MHz / 10 = 43.2 MHz
 * DSI CLK = PLL_VCO / PLLR = 432 / 8 = 54 MHz
 */

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(25)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(432)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_2
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(10)
#define STM32_PLLCFG_PLLR       RCC_PLLCFG_PLLR(8)

#define STM32_VCO_FREQUENCY     ((STM32_HSE_FREQUENCY / 25) * 432)
#define STM32_SYSCLK_FREQUENCY  (STM32_VCO_FREQUENCY / 2)
#define STM32_OTGFS_FREQUENCY   (STM32_VCO_FREQUENCY / 10)

#else
/* No restrictions by OTGFS
 *
 * PLL_VCO = (25,000,000 / 25) * 432 = 432 MHz
 * SYSCLK  = 432 MHz / 2 = 216 MHz
 * USB OTG FS, SDMMC and RNG Clock = 432 MHz / 10 = 43.2 MHz
 * DSI CLK = PLL_VCO / PLLR = 432 / 8 = 54 MHz
 */

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(25)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(432)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_2
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(10)
#define STM32_PLLCFG_PLLR       RCC_PLLCFG_PLLR(8)

#define STM32_VCO_FREQUENCY     ((STM32_HSE_FREQUENCY / 25) * 432)
#define STM32_SYSCLK_FREQUENCY  (STM32_VCO_FREQUENCY / 2)
#define STM32_OTGFS_FREQUENCY   (STM32_VCO_FREQUENCY / 10)
#endif

/* Configure factors for  PLLSAI clock */

#define STM32_RCC_PLLSAICFGR_PLLSAIN    RCC_PLLSAICFGR_PLLSAIN(192)
#define STM32_RCC_PLLSAICFGR_PLLSAIP    RCC_PLLSAICFGR_PLLSAIP(2)
#define STM32_RCC_PLLSAICFGR_PLLSAIQ    RCC_PLLSAICFGR_PLLSAIQ(2)
#define STM32_RCC_PLLSAICFGR_PLLSAIR    RCC_PLLSAICFGR_PLLSAIR(2)

/* Configure Dedicated Clock Configuration Register */

#define STM32_RCC_DCKCFGR1_PLLI2SDIVQ  RCC_DCKCFGR1_PLLI2SDIVQ(1)
#define STM32_RCC_DCKCFGR1_PLLSAIDIVQ  RCC_DCKCFGR1_PLLSAIDIVQ(1)
#define STM32_RCC_DCKCFGR1_PLLSAIDIVR  RCC_DCKCFGR1_PLLSAIDIVR(0)
#define STM32_RCC_DCKCFGR1_SAI1SRC     RCC_DCKCFGR1_SAI1SEL(0)
#define STM32_RCC_DCKCFGR1_SAI2SRC     RCC_DCKCFGR1_SAI2SEL(0)
#define STM32_RCC_DCKCFGR1_TIMPRESRC   0
#define STM32_RCC_DCKCFGR1_DFSDM1SRC   0
#define STM32_RCC_DCKCFGR1_ADFSDM1SRC  0

/* Configure factors for  PLLI2S clock */

#define STM32_RCC_PLLI2SCFGR_PLLI2SN   RCC_PLLI2SCFGR_PLLI2SN(192)
#define STM32_RCC_PLLI2SCFGR_PLLI2SP   RCC_PLLI2SCFGR_PLLI2SP(2)
#define STM32_RCC_PLLI2SCFGR_PLLI2SQ   RCC_PLLI2SCFGR_PLLI2SQ(2)
#define STM32_RCC_PLLI2SCFGR_PLLI2SR   RCC_PLLI2SCFGR_PLLI2SR(2)

/* Configure Dedicated Clock Configuration Register 2 */

#define STM32_RCC_DCKCFGR2_USART1SRC  RCC_DCKCFGR2_USART1SEL_APB
#define STM32_RCC_DCKCFGR2_USART2SRC  RCC_DCKCFGR2_USART2SEL_APB
#define STM32_RCC_DCKCFGR2_UART4SRC   RCC_DCKCFGR2_UART4SEL_APB
#define STM32_RCC_DCKCFGR2_UART5SRC   RCC_DCKCFGR2_UART5SEL_APB
#define STM32_RCC_DCKCFGR2_USART6SRC  RCC_DCKCFGR2_USART6SEL_APB
#define STM32_RCC_DCKCFGR2_UART7SRC   RCC_DCKCFGR2_UART7SEL_APB
#define STM32_RCC_DCKCFGR2_UART8SRC   RCC_DCKCFGR2_UART8SEL_APB
#define STM32_RCC_DCKCFGR2_I2C1SRC    RCC_DCKCFGR2_I2C1SEL_HSI
#define STM32_RCC_DCKCFGR2_I2C2SRC    RCC_DCKCFGR2_I2C2SEL_HSI
#define STM32_RCC_DCKCFGR2_I2C3SRC    RCC_DCKCFGR2_I2C3SEL_HSI
#define STM32_RCC_DCKCFGR2_I2C4SRC    RCC_DCKCFGR2_I2C4SEL_HSI
#define STM32_RCC_DCKCFGR2_LPTIM1SRC  RCC_DCKCFGR2_LPTIM1SEL_APB
#define STM32_RCC_DCKCFGR2_CECSRC     RCC_DCKCFGR2_CECSEL_HSI
#define STM32_RCC_DCKCFGR2_CK48MSRC   RCC_DCKCFGR2_CK48MSEL_PLLSAI
#define STM32_RCC_DCKCFGR2_SDMMCSRC   RCC_DCKCFGR2_SDMMCSEL_48MHZ
#define STM32_RCC_DCKCFGR2_SDMMC2SRC  RCC_DCKCFGR2_SDMMC2SEL_48MHZ
#define STM32_RCC_DCKCFGR2_DSISRC     RCC_DCKCFGR2_DSISEL_PHY

/* Several prescalers allow the configuration of the two AHB buses, the
 * high-speed APB (APB2) and the low-speed APB (APB1) domains. The maximum
 * frequency of the two AHB buses is 216 MHz while the maximum frequency of
 * the high-speed APB domains is 108 MHz. The maximum allowed frequency of
 * the low-speed APB domain is 54 MHz.
 */

/* AHB clock (HCLK) is SYSCLK (216 MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK  /* HCLK  = SYSCLK / 1 */
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY
#define STM32_BOARD_HCLK        STM32_HCLK_FREQUENCY  /* same as above, to satisfy compiler */
#define BOARD_AHB_FREQUENCY     STM32_HCLK_FREQUENCY
/* APB1 clock (PCLK1) is HCLK/4 (54 MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd4     /* PCLK1 = HCLK / 4 */
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/4)

/* Timers driven from APB1 will be twice PCLK1 */

#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM12_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM13_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM14_CLKIN  (2*STM32_PCLK1_FREQUENCY)

/* APB2 clock (PCLK2) is HCLK/2 (108MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLKd2     /* PCLK2 = HCLK / 2 */
#define STM32_PCLK2_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* Timers driven from APB2 will be twice PCLK2 */

#define STM32_APB2_TIM1_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM9_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM10_CLKIN  (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM11_CLKIN  (2*STM32_PCLK2_FREQUENCY)

/* FLASH wait states
 *
 *  --------- ---------- -----------
 *  VDD       MAX SYSCLK WAIT STATES
 *  --------- ---------- -----------
 *  1.7-2.1 V   180 MHz    8
 *  2.1-2.4 V   216 MHz    9
 *  2.4-2.7 V   216 MHz    8
 *  2.7-3.6 V   216 MHz    7
 *  --------- ---------- -----------
 */
#define BOARD_FLASH_WAITSTATES 7

/* LED definitions ******************************************************************/
/* The STM32F769I-DISCO board has numerous LEDs but only one, LD1 located near the
 * reset button, that can be controlled by software (LD2 is a power indicator, LD3-6
 * indicate USB status, LD7 is controlled by the ST-Link).
 *
 * LD1 is controlled by PI1 which is also the SPI2_SCK at the Arduino interface.
 * One end of LD1 is grounded so a high output on PI1 will illuminate the LED.
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in any way.
 * The following definitions are used to access individual LEDs.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED1        0
#define BOARD_NLEDS       1

#define BOARD_LD1         BOARD_LED1

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)

/* If CONFIG_ARCH_LEDS is defined, the usage by the board port is defined in
 * include/board.h and src/stm32_leds.c. The LEDs are used to encode OS-related
 * events as follows:
 *
 *   SYMBOL              Meaning                 LD1
 *   ------------------- ----------------------- ------
 *   LED_STARTED         NuttX has been started  OFF
 *   LED_HEAPALLOCATE    Heap has been allocated OFF
 *   LED_IRQSENABLED     Interrupts enabled      OFF
 *   LED_STACKCREATED    Idle stack created      ON
 *   LED_INIRQ           In an interrupt         N/C
 *   LED_SIGNAL          In a signal handler     N/C
 *   LED_ASSERTION       An assertion failed     N/C
 *   LED_PANIC           The system has crashed  FLASH
 *
 * Thus is LD1 is statically on, NuttX has successfully  booted and is,
 * apparently, running normally.  If LD1 is flashing at approximately
 * 2Hz, then a fatal error has been detected and the system has halted.
 */

#define LED_STARTED                  0 /* LD1=OFF */
#define LED_HEAPALLOCATE             0 /* LD1=OFF */
#define LED_IRQSENABLED              0 /* LD1=OFF */
#define LED_STACKCREATED             1 /* LD1=ON */
#define LED_INIRQ                    2 /* LD1=no change */
#define LED_SIGNAL                   2 /* LD1=no change */
#define LED_ASSERTION                2 /* LD1=no change */
#define LED_PANIC                    3 /* LD1=flashing */

/* Button definitions ***************************************************************/
/* The STM32F7 Discovery supports one button:  Pushbutton B1, labelled "User", is
 * connected to GPIO PA0.  A high value will be sensed when the button is depressed.
 */

#define BUTTON_USER        0
#define NUM_BUTTONS        1
#define BUTTON_USER_BIT    (1 << BUTTON_USER)

/* Alternate function pin selections ************************************************/

/* EXIT:
 *   HW_RXIN1   GPIO_PORTH|GPIO_PIN15
 *   HW_RXIN2   GPIO_PORTG|GPIO_PIN12
 *   HW_RXIN3   GPIO_PORTI|GPIO_PIN0
 *   HW_RXIN4   GPIO_PORTI|GPIO_PIN1
 *   HW_RXIN5   GPIO_PORTI|GPIO_PIN4
 *   HW_RXIN6   GPIO_PORTG|GPIO_PIN3
 *   -- ----- --------- -----
 */
#define NUM_IRQHALL   6
#define MIN_IRQHALL   0
#define MAX_IRQHALL   NUM_IRQHALL
#define NUM_HALLS     NUM_IRQHALL

#define GPIO_HW_RXIN1   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTH|GPIO_PIN15)
#define GPIO_HW_RXIN2   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTG|GPIO_PIN12)
#define GPIO_HW_RXIN3   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTI|GPIO_PIN0)
#define GPIO_HW_RXIN4   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTI|GPIO_PIN1)
#define GPIO_HW_RXIN5   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTI|GPIO_PIN4)
#define GPIO_HW_RXIN6   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTG|GPIO_PIN3)

/* USART1:
 *   DBG_UART_RX   USART1_RX PA10
 *   DBG_UART_TX   USART1_TX PA9
 *   -- ----- --------- -----
 */
#define GPIO_USART1_RX GPIO_USART1_RX_1
#define GPIO_USART1_TX GPIO_USART1_TX_1

/* USART2:
 *   ENC_UART_RX   USART2_RX PA2
 *   ENC_UART_TX   USART2_TX PA3
 *   -- ----- --------- -----
 */
#define GPIO_USART2_RX GPIO_USART2_RX_1
#define GPIO_USART2_TX GPIO_USART2_TX_1

/* USART3:
 *   DXIMU_UART_RX   USART3_RX PB11
 *   DXIMU_UART_TX   USART3_TX PB10
 *   DXIMU_UART_EN   USART3_EN PG6
 *   -- ----- --------- -----
 */
#define GPIO_USART3_RX 			GPIO_USART3_RX_1
#define GPIO_USART3_TX 			GPIO_USART3_TX_1
#define GPIO_USART3_RS485_DIR   (GPIO_OUTPUT|GPIO_PULLDOWN|GPIO_PUSHPULL|GPIO_OUTPUT_CLEAR|GPIO_SPEED_50MHz|\
								 GPIO_PORTG|GPIO_PIN6)

/* UART4:
 *   XB_UART_RX   UART_RX 	PH14
 *   XB_UART_TX   UART_TX 	PH13
 *   -- ----- --------- -----
 */
#define GPIO_UART4_RX GPIO_UART4_RX_5
#define GPIO_UART4_TX GPIO_UART4_TX_5

/* UART5:
 *   DRV_UART_RX   UART_RX 	PB8
 *   DRV_UART_TX   UART_TX 	PB9
 *   DRV_UART_EN   UART_EN  PD3
 *   -- ----- --------- -----
 */
#define GPIO_UART5_RX 			GPIO_UART5_RX_4
#define GPIO_UART5_TX 			GPIO_UART5_TX_4
#define GPIO_UART5_RS485_DIR   (GPIO_OUTPUT|GPIO_PULLDOWN|GPIO_PUSHPULL|GPIO_OUTPUT_CLEAR|GPIO_SPEED_50MHz|\
								 GPIO_PORTD|GPIO_PIN3)

/* USART6:
 *   GPS_USART_RX  USART_RX PC7
 *   GPS_USART_TX  USART_TX PC6
 *   -- ----- --------- -----
 */
//#define DMAMAP_USART6_RX_1         STM32_DMA_MAP(DMA2,DMA_STREAM1,DMA_CHAN5)
//#define DMAMAP_USART6_RX_2         STM32_DMA_MAP(DMA2,DMA_STREAM2,DMA_CHAN5)
//#define DMAMAP_USART6_TX_1         STM32_DMA_MAP(DMA2,DMA_STREAM6,DMA_CHAN5)
//#define DMAMAP_USART6_TX_2         STM32_DMA_MAP(DMA2,DMA_STREAM7,DMA_CHAN5)

#define GPIO_USART6_RX GPIO_USART6_RX_1
#define GPIO_USART6_TX GPIO_USART6_TX_1
#define DMAMAP_USART6_RX DMAMAP_USART6_RX_1

/* UART7:
 *   IMU_UART_RX  USART_RX PB3
 *   IMU_UART_TX  USART_TX PB4
 *   -- ----- --------- -----
 */
#define GPIO_UART7_RX GPIO_UART7_RX_4
#define GPIO_UART7_TX GPIO_UART7_TX_4

/* CAN3:
 *   DRV_CAN_TXD  CAN3_TX PA15
 *   DRV_CAN_RXD  CAN3_RX PA8
 */
#define GPIO_CAN3_RX GPIO_CAN3_RX_1
#define GPIO_CAN3_TX GPIO_CAN3_TX_1

/* ADC:
 *   CURRENT_AIN1  ADC12_IN8 	PB0
 *   CURRENT_AIN2  ADC12_IN0 	PA0
 *   CURRENT_AIN3  ADC12_IN6 	PA6
 *   CURRENT_AIN4  ADC12_IN5 	PA5
 *   CURRENT_AIN5  ADC12_IN4 	PA4
 *   CURRENT_AIN6  ADC3_IN4     PF6
 *   CURRENT_BUC   ADC12_IN9    PB1
 */
#define ADC1_DMA_CHAN DMAMAP_ADC1_2
#define ADC2_DMA_CHAN DMAMAP_ADC2_1
#define ADC3_DMA_CHAN DMAMAP_ADC3_2

#define GPIO_CURRENT_AIN1	GPIO_ADC1_IN0
#define GPIO_CURRENT_AIN2	GPIO_ADC1_IN4
#define GPIO_CURRENT_AIN3	GPIO_ADC1_IN6
#define GPIO_CURRENT_AIN4	GPIO_ADC1_IN5
#define GPIO_CURRENT_AIN5	GPIO_ADC1_IN4
#define GPIO_CURRENT_AIN6	GPIO_ADC3_IN4
#define GPIO_CURRENT_BUC	GPIO_ADC1_IN9


/****************************************************************/

/* The STM32 F7 connects to a SMSC LAN8710A PHY using these pins:
 *
 *   STM32 F7 BOARD        LAN8710A
 *   GPIO     SIGNAL       PIN NAME
 *   -------- ------------ -------------
 *   PG11     RMII_TX_EN   TXEN
 *   PB12     RMII_TXD0    TXD0
 *   PB13     RMII_TXD1    TXD1
 *   PC4      RMII_RXD0    RXD0/MODE0
 *   PC5      RMII_RXD1    RXD1/MODE1
 *   N/A      RMII_RXER    RXER/PHYAD0
 *   PA7      RMII_CRS_DV  CRS_DV/MODE2
 *   PC1      RMII_MDC     MDC
 *   PA2      RMII_MDIO    MDIO
 *   N/A      NRST         nRST
 *   PA1      RMII_REF_CLK nINT/REFCLK0
 *   N/A      OSC_25M      XTAL1/CLKIN
 *
 * The PHY address is 0, since RMII_RXER/PHYAD0 features a pull down.
 * After reset, RMII_RXER/PHYAD0 switches to the RXER function,
 * receive errors can be detected using GPIO pin PD5
 */
/*

*/
#if (CONFIG_ALENTEK_NET_IOCTL == 1)

	//LAN-8720 ALIENTEK
	#define GPIO_ETH_RMII_TX_EN   GPIO_ETH_RMII_TX_EN_1
	#define GPIO_ETH_RMII_TXD0    GPIO_ETH_RMII_TXD0_2
	#define GPIO_ETH_RMII_TXD1    GPIO_ETH_RMII_TXD1_2

#else

	//LAN-8710 FPPA
	#define GPIO_ETH_RMII_TX_EN   GPIO_ETH_RMII_TX_EN_2
	#define GPIO_ETH_RMII_TXD0    GPIO_ETH_RMII_TXD0_1
	#define GPIO_ETH_RMII_TXD1    GPIO_ETH_RMII_TXD1_1
	
#endif
//#define GPIO_ETH_RMII_CRS_DV  GPIO_ETH_RMII_CRS_DV
//#define GPIO_ETH_RMII_REF_CLK GPIO_ETH_RMII_REF_CLK
//#define GPIO_ETH_RMII_RXD0    GPIO_ETH_RMII_RXD0
//#define GPIO_ETH_RMII_RXD1    GPIO_ETH_RMII_RXD1
//#define GPIO_ETH_MDC          GPIO_ETH_MDC
//#define GPIO_ETH_MDIO         GPIO_ETH_MDIO


/* I2C Mapping
 * I2C #4 is connected to the LCD daughter board
 * and the WM8994 audio codec.
 *
 * I2C4_SCL - PD12
 * I2C4_SDA - PB7
 */
//#define GPIO_I2C4_SCL        GPIO_I2C4_SCL_1
//#define GPIO_I2C4_SDA        GPIO_I2C4_SDA_5

#define GPIO_I2C2_SCL        GPIO_I2C2_SCL_3
#define GPIO_I2C2_SDA        GPIO_I2C2_SDA_3

/* SDMMC */

/* Stream selections are arbitrary for now but might become important in the future
 * if we set aside more DMA channels/streams.
 *
 * SDIO DMA
 *   DMAMAP_SDMMC1_1 = Channel 4, Stream 3
 *   DMAMAP_SDMMC1_2 = Channel 4, Stream 6
 *
 *   DMAMAP_SDMMC2_1 = Channel 11, Stream 0
 *   DMAMAP_SDMMC2_2 = Channel 11, Stream 5
 *
#define DMAMAP_SDMMC1_1            STM32_DMA_MAP(DMA2,DMA_STREAM3,DMA_CHAN4)
#define DMAMAP_SDMMC1_2            STM32_DMA_MAP(DMA2,DMA_STREAM6,DMA_CHAN4)
 */
#define DMAMAP_SDMMC1  STM32_DMA_MAP(DMA2,DMA_STREAM6,DMA_CHAN4)

/* SDIO dividers.  Note that slower clocking is required when DMA is disabled
 * in order to avoid RX overrun/TX underrun errors due to delayed responses
 * to service FIFOs in interrupt driven mode.  These values have not been
 * tuned!!!
 *
 * SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(118+2)=400 KHz
 */
#define STM32_SDMMC_INIT_CLKDIV      (118 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)

/* DMA ON:  SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(1+2)=16 MHz
 * DMA OFF: SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(2+2)=12 MHz
 */
#ifdef CONFIG_SDIO_DMA
#  define STM32_SDMMC_MMCXFR_CLKDIV  (1 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#else
#  define STM32_SDMMC_MMCXFR_CLKDIV  (2 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#endif

/* DMA ON:  SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(1+2)=16 MHz
 * DMA OFF: SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(2+2)=12 MHz
 */
#ifdef CONFIG_SDIO_DMA
#  define STM32_SDMMC_SDXFR_CLKDIV   (1 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#else
#  define STM32_SDMMC_SDXFR_CLKDIV   (2 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#endif

/* SDMMC1 Pin mapping
 *
 * D0 - PC8
 * D1 - PC9
 * D2 - PC10
 * D3 - PC11
 * CLK- PC12
 * CMD- PD2
 */

/*
#define GPIO_SDMMC1_D0   GPIO_SDMMC1_D0
#define GPIO_SDMMC1_D1   GPIO_SDMMC1_D1
#define GPIO_SDMMC1_D2   GPIO_SDMMC1_D2
#define GPIO_SDMMC1_D3   GPIO_SDMMC1_D3
#define GPIO_SDMMC1_CK   GPIO_SDMMC1_CK
#define GPIO_SDMMC1_CMD  GPIO_SDMMC1_CMD
*/
//MicroSD_DET PH8 ---ISR8
#define GPIO_MicroSD_DET (GPIO_INPUT|GPIO_FLOAT|GPIO_OPENDRAIN|GPIO_EXTI|GPIO_PORTH|GPIO_PIN8)
/*############################################################################################*/
/* SPI2 Pin mapping
 * ADIS_IMU_CLK  PA12
 * ADIS_IMU_CS   PA11
 * ADIS_IMU_MOSI PB15
 * ADIS_IMU_MISO PB14
 */
#define GPIO_SPI2_MISO (GPIO_ALT|GPIO_AF5|GPIO_PULLUP|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_PORTB|GPIO_PIN14)
#define GPIO_SPI2_MOSI GPIO_SPI2_MOSI_1
#define GPIO_SPI2_SCK  GPIO_SPI2_SCK_6
#define GPIO_SPI2_NSS  (GPIO_OUTPUT|GPIO_PULLUP|GPIO_PUSHPULL|GPIO_OUTPUT_SET|GPIO_SPEED_50MHz|GPIO_PORTA|GPIO_PIN11)

/* SPI4 Pin mapping
* DA_DRV_SYNCB PI11
* DA_DRV_DINB/DA_DRV_DINA PE6 (MOSI)
* DA_DRV_LDACA PE5 (MISO)
* DA_DRV_SYNCA PE4 (NSS)
* DA_DRV_LDACB PE3
* DA_DRV_SCLKB/DA_DRV_SCLKA (SCK) PE2
*/
//未使用
//#define GPIO_SPI4_MISO

#define DA_DRV_LDACA (GPIO_OUTPUT|GPIO_PULLUP|GPIO_PUSHPULL|GPIO_OUTPUT_SET|GPIO_SPEED_50MHz|\
								 GPIO_PORTE|GPIO_PIN5)

#define DA_DRV_LDACB (GPIO_OUTPUT|GPIO_PULLUP|GPIO_PUSHPULL|GPIO_OUTPUT_SET|GPIO_SPEED_50MHz|\
								 GPIO_PORTE|GPIO_PIN3)

#define DA_DRV_SYNCB (GPIO_OUTPUT|GPIO_PULLUP|GPIO_PUSHPULL|GPIO_OUTPUT_SET|GPIO_SPEED_50MHz|\
								 GPIO_PORTE|GPIO_PIN4)

#define DA_DRV_SYNCA (GPIO_OUTPUT|GPIO_PULLUP|GPIO_PUSHPULL|GPIO_OUTPUT_SET|GPIO_SPEED_50MHz|\
								 GPIO_PORTI|GPIO_PIN11)

#define GPIO_SPI4_MOSI GPIO_SPI4_MOSI_1
#define GPIO_SPI4_SCK  GPIO_SPI4_SCK_1

#define DMAMAP_SPI4_RX           STM32_DMA_MAP(DMA2,DMA_STREAM0,DMA_CHAN4)
#define DMAMAP_SPI4_TX           STM32_DMA_MAP(DMA2,DMA_STREAM1,DMA_CHAN4)
/* SPI5 Pin mapping
 *
 * MAX_SSN  	PD7
 * MAX_MOSI 	PF9
 * MAX_MISO 	PF8
 * MAX_CLK  	PF7
 */
#define GPIO_SPI5_MISO GPIO_SPI5_MISO_1
#define GPIO_SPI5_MOSI GPIO_SPI5_MOSI_1
//默认输出值=高,推挽上拉输出
#define GPIO_SPI5_NSS  (GPIO_OUTPUT|GPIO_PULLUP|GPIO_PUSHPULL|GPIO_OUTPUT_SET|GPIO_SPEED_50MHz|\
						GPIO_PORTD|GPIO_PIN7)
#define GPIO_SPI5_SCK  GPIO_SPI5_SCK_1

#define DMAMAP_SPI5_RX           STM32_DMA_MAP(DMA2,DMA_STREAM3,DMA_CHAN2)
#define DMAMAP_SPI5_TX           STM32_DMA_MAP(DMA2,DMA_STREAM4,DMA_CHAN2)

/* Quad SPI pin mapping */

#define GPIO_QSPI_CS         (GPIO_QUADSPI_BK1_NCS_1 | GPIO_FLOAT | GPIO_PUSHPULL | GPIO_SPEED_100MHz)
#define GPIO_QSPI_IO0        (GPIO_QUADSPI_BK1_IO0_1 | GPIO_FLOAT | GPIO_PUSHPULL | GPIO_SPEED_100MHz)
#define GPIO_QSPI_IO1        (GPIO_QUADSPI_BK1_IO1_1 | GPIO_FLOAT | GPIO_PUSHPULL | GPIO_SPEED_100MHz)
#define GPIO_QSPI_IO2        (GPIO_QUADSPI_BK1_IO2_2 | GPIO_FLOAT | GPIO_PUSHPULL | GPIO_SPEED_100MHz)
#define GPIO_QSPI_IO3        (GPIO_QUADSPI_BK1_IO3_3 | GPIO_FLOAT | GPIO_PUSHPULL | GPIO_SPEED_100MHz)
#define GPIO_QSPI_SCK        (GPIO_QUADSPI_CLK_1 | GPIO_FLOAT | GPIO_PUSHPULL | GPIO_SPEED_100MHz)


/* PWM Pin mapping
 *
 * PWM_TIM5_CH1CFG  	PH10	(PWM4)
 * PWM_TIM5_CH2CFG 		PH11	(PWM5)
 * PWM_TIM5_CH3CFG 		PH12	(PWM6)
 * PWM_TIM8_CH1CFG  	PI5		(PWM1)
 * PWM_TIM8_CH2CFG		PI6		(PWM2)
 * PWM_TIM8_CH3CFG		PI7		(PWM3)
 */
#define	GPIO_TIM5_CH1OUT			GPIO_TIM5_CH1OUT_2
#define	GPIO_TIM5_CH2OUT			GPIO_TIM5_CH2OUT_2
#define	GPIO_TIM5_CH3OUT			GPIO_TIM5_CH3OUT_2
#define	GPIO_TIM8_CH1OUT			GPIO_TIM8_CH1OUT_2
#define	GPIO_TIM8_CH2OUT			GPIO_TIM8_CH2OUT_2
#define	GPIO_TIM8_CH3OUT			GPIO_TIM8_CH3OUT_2
#define GPIO_PWM8_IO_CH1  (GPIO_OUTPUT|GPIO_PULLUP|GPIO_PUSHPULL|GPIO_OUTPUT_SET|GPIO_SPEED_50MHz|GPIO_PORTH|GPIO_PIN10)
#define GPIO_PWM8_IO_CH2  (GPIO_OUTPUT|GPIO_PULLUP|GPIO_PUSHPULL|GPIO_OUTPUT_SET|GPIO_SPEED_50MHz|GPIO_PORTH|GPIO_PIN11)
#define GPIO_PWM8_IO_CH3  (GPIO_OUTPUT|GPIO_PULLUP|GPIO_PUSHPULL|GPIO_OUTPUT_SET|GPIO_SPEED_50MHz|GPIO_PORTH|GPIO_PIN12)


/*
 * ISR confige
 *
 *
 */
//#define GPIO_BTN_USER      (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | GPIO_PORTA | GPIO_PIN0)

/*
 * IO confige
 *
 *
 */


/* LCD DISPLAY
 * (work in progress as of 2017 07 19)
 */
#define	BOARD_LTDC_WIDTH        800
#define	BOARD_LTDC_HEIGHT       472

#define	BOARD_LTDC_HSYNC        10
#define	BOARD_LTDC_HFP          10
#define	BOARD_LTDC_HBP          20
#define	BOARD_LTDC_VSYNC        2
#define	BOARD_LTDC_VFP          4
#define	BOARD_LTDC_VBP          2

#define	BOARD_LTDC_GCR_PCPOL    0
#define	BOARD_LTDC_GCR_DEPOL    0
#define	BOARD_LTDC_GCR_VSPOL    0
#define	BOARD_LTDC_GCR_HSPOL    0

#endif  /* __CONFIG_STM32F769I_DISCO_INCLUDE_BOARD_H */
