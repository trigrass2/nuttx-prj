/*
This file is part of CanFestival, a library implementing CanOpen Stack.

Copyright (C): Edouard TISSERANT and Francis DUPIN
AT91 Port: Peter CHRISTEN

See COPYING file for copyrights details.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/


#ifndef __CAN_CANFESTIVAL__
#define __CAN_CANFESTIVAL__

#include "canopen_stack/applicfg.h"
#include "canopen_stack/data.h"
#include "canopen_stack/ObjDict.h"

// ---------  to be called by user app ---------
//int initTimer(void);


/****************************************************************************
 * Name: timerDeviceConfigration
 *
 * Description:
 *   configure soft timer.
 *
 ****************************************************************************/
int timerDeviceConfigration(void);



/****************************************************************************
 * Name: timerDeviceOpen
 *
 * Description:
 *   open timer port and return fd.
 *
 ****************************************************************************/
int timerDeviceOpen(void);



/****************************************************************************
 * Name: canDeviceMsgSend
 *
 * Description:
 *   sent a can message.
 *
 ****************************************************************************/
unsigned char canDeviceMsgSend(CAN_PORT CANx, Message *m);


/****************************************************************************
 * Name: canDeviceMsgRead
 *
 * Description:
 *   read a can message.
 *
 ****************************************************************************/
int canDeviceMsgRead(unsigned char type);


/****************************************************************************
 * Name: canDeviceOpen
 *
 * Description:
 *   open can port
 *
 ****************************************************************************/
int canDeviceOpen(int type);


/****************************************************************************
 * Name: canDeviceClose
 *
 * Description:
 *   close can port.
 *
 ****************************************************************************/
int canDeviceClose(void);

#endif
