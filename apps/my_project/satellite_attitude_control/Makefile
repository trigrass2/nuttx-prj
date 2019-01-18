############################################################################
# apps/my_project/satellite_attitude_control/Makefile
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

# satellite_attitude_control application info

CONFIG_SATELLITE_ATTITUDE_CONTROL_PRIORITY ?= SCHED_PRIORITY_DEFAULT
CONFIG_SATELLITE_ATTITUDE_CONTROL_STACKSIZE ?= 2048

APPNAME = satellite_attitude_control
PRIORITY = $(CONFIG_SATELLITE_ATTITUDE_CONTROL_PRIORITY)
STACKSIZE = $(CONFIG_SATELLITE_ATTITUDE_CONTROL_STACKSIZE)

# Hello, World! Example

ASRCS =
CSRCS = pid.c fuzzy_pid.c params_storage.c\
		sac_msg_subscribe.c sac_msg_advertise.c sac_msg_poll.c sac_msg_pre_check.c\
		sac_mode_initialize.c sac_mode_run.c sac_mode_stop.c\
		sac_beam_convert.c sac_ctrl_stabilize.c sac_ctrl_output_mixer.c\
		adrc.c
MAINSRC = satellite_attitude_control_main.c

CONFIG_SATELLITE_ATTITUDE_CONTROL_PROGNAME ?= satellite_attitude_control$(EXEEXT)
PROGNAME = $(CONFIG_SATELLITE_ATTITUDE_CONTROL_PROGNAME)

include $(APPDIR)/Application.mk
