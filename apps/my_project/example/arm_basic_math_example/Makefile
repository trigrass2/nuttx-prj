############################################################################
# apps/my_project/examples/uorb_example/Makefile
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

EXAMPLES_ARM_BASIC_MATH_PROGNAME  ?= "arm_basic_math_example"
EXAMPLES_ARM_BASIC_MATH_PRIORITY  ?= SCHED_PRIORITY_DEFAULT
EXAMPLES_ARM_BASIC_MATH_STACKSIZE ?= 2048

# uorb Example

ASRCS =
CSRCS =
MAINSRC = arm_basic_math_example_main.c

CONFIG_XYZ_PROGNAME ?= $(EXAMPLES_ARM_BASIC_MATH_PROGNAME)$(EXEEXT)
PROGNAME = $(CONFIG_XYZ_PROGNAME)

# uorb_example built-in application info

APPNAME = $(EXAMPLES_ARM_BASIC_MATH_PROGNAME)
PRIORITY = $(EXAMPLES_ARM_BASIC_MATH_PRIORITY)
STACKSIZE = $(EXAMPLES_ARM_BASIC_MATH_STACKSIZE)

include $(APPDIR)/Application.mk
