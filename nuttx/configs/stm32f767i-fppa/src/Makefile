############################################################################
# configs/stm32f769i-disco/src/Makefile
#
#   Copyright (C) 2015 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

-include $(TOPDIR)/Make.defs

ASRCS =
CSRCS = stm32_boot.c

ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += stm32_autoleds.c
else
CSRCS += stm32_userleds.c
endif

ifeq ($(CONFIG_ARCH_BUTTONS),y)
CSRCS +=  stm32_buttons.c
endif

ifeq ($(CONFIG_LIB_BOARDCTL),y)
CSRCS += stm32_appinitialize.c
endif

ifeq ($(CONFIG_ARCH_FPU),y)
CSRCS += stm32_ostest.c
endif

ifeq ($(CONFIG_SPORADIC_INSTRUMENTATION),y)
CSRCS += stm32_sporadic.c
endif

#
## These drivers depend on CAN support
#
ifeq ($(CONFIG_CAN),y)
CSRCS += stm32_canbus.c
endif

#
## These drivers depend on TIMER support
#
ifeq ($(CONFIG_TIMER),y)
CSRCS += stm32_soft_timer.c
endif

#
## These drivers depend on SPI support
#
ifeq ($(CONFIG_SPI),y)
CSRCS += stm32_spibus.c
endif

#
## These drivers depend on PWM support
#
ifeq ($(CONFIG_PWM),y)
CSRCS += stm32_pwm_timer.c

ifeq ($(CONFIG_MOTOR_LOWER_PWM_IO),y)
CSRCS += stm32_elmo_db4x.c
endif

endif

ifeq ($(CONFIG_HALLS),y)
CSRCS += stm32_hall.c
endif

#
## These drivers depend on MMCSD support
#
ifeq ($(CONFIG_MMCSD),y)
CSRCS += stm32_sdmmc.c
endif

#
## These drivers depend on ADC support
#
ifeq ($(CONFIG_ADC),y)
CSRCS += stm32_current_adc.c
endif

#
## These drivers depend on I2C support
#
ifeq ($(CONFIG_I2C),y)

ifeq ($(CONFIG_SENSORS_HMC6343),y)
CSRCS += stm32_hmc6343.c
endif

ifeq ($(CONFIG_I2C_EE_24XX),y)
CSRCS += stm32_i2c_xx24xx.c
endif

endif

CSRCS +=stm32_buzzer.c

#
## These drivers depend on QUARDSPI support
#
ifeq ($(CONFIG_STM32F7_QUADSPI),y)

ifeq ($(CONFIG_MTD_N25QXXX),y)
CSRCS += stm32_n25qxxx.c
endif

endif

include $(TOPDIR)/configs/Board.mk
