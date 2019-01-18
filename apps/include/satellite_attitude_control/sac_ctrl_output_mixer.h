/****************************************************************************
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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

#ifndef __APPS_SATELLITE_ATTITUDE_OUTPUT_MIXER_H
#define __APPS_SATELLITE_ATTITUDE_OUTPUT_MIXER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/



/****************************************************************************
 * Public Types
 ****************************************************************************/


/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
/****************************************************************************
 * Name: mixer_output_linear_pid
 *
 * Description:
 *   satellite attitude control output model.
 *
 * Input Parameters:
 *   psat_att_ctrl  - pointer of struct sat_att_ctrl_t.
 *   l1				- output to L1
 *   l2				- output to L2
 *   l3				- output to L3
 *
 * Returned Value:
 *   the status of program.
 *
 ****************************************************************************/
int mixer_output_linear_pid(void *psat_att_ctrl,float *l1,float *l2,float *l3);


/****************************************************************************
 * Name: mixer_stabilize_fuzzy_pid
 *
 * Description:
 *   satellite attitude control with fuzzy PID.
 *
 * Input Parameters:
 *   psat_att_ctrl  - pointer of struct sat_att_ctrl_t.
 *
 * Returned Value:
 *   the status of program.
 *
 ****************************************************************************/
int mixer_output_fuzzy_pid(void *psat_att_ctrl,float *l1,float *l2,float *l3);

#endif /* __APPS_SATELLITE_ATTITUDE_OUTPUT_MIXER_H */
