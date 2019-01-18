############################################################################
# apps/my_project/imu/Makefile
#
#   Copyright (C) 2008, 2010-2013 Gregory Nutt. All rights reserved.
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

# imu project ! built-in application info

CONFIG_MYPROJECT_IMU_PRIORITY ?= SCHED_PRIORITY_DEFAULT
CONFIG_MYPROJECT_IMU_STACKSIZE ?= 4096

APPNAME = imu
PRIORITY = $(CONFIG_MYPROJECT_IMU_PRIORITY)
STACKSIZE = $(CONFIG_MYPROJECT_IMU_STACKSIZE)

# imu project

ASRCS =

CSRCS += imu_matrix.c
CSRCS += imu_init.c
CSRCS += imu_subscribe.c
CSRCS += imu_publish.c
CSRCS += imu_gyro_filter.c
CSRCS += imu_acc_filter.c
CSRCS += imu_mag_filter.c
CSRCS += imu_rtk_filter.c
CSRCS += imu_aim.c
CSRCS += imu_fusion_cf.c
CSRCS += imu_fusion_ekf_base.c
CSRCS += imu_fusion.c
CSRCS += imu_print.c
CSRCS += imu_rotate.c
CSRCS += imu_att_rotate.c
CSRCS += imu_mag_declination.c
CSRCS += imu_mag_worldmodel.c



MAINSRC = imu_main.c

CONFIG_MYPROJECT_IMU_PROGNAME ?= imu$(EXEEXT)
PROGNAME = $(CONFIG_MYPROJECT_IMU_PROGNAME)

include $(APPDIR)/Application.mk
