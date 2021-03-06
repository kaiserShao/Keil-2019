/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: portevent.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- Variables ----------------------------------------*/
static eMBEventType eQueuedEvent;
static _Bool     xEventInQueue;

/* ----------------------- Start implementation -----------------------------*/
_Bool
xMBPortEventInit( void )
{
    xEventInQueue = FALSE;
	
    return TRUE;
}

_Bool
xMBPortEventPost( eMBEventType eEvent )
{
    xEventInQueue = TRUE;
    eQueuedEvent = eEvent;

	//	悬起PendSV，使用PendSV解析协议栈
//	SCB->ICSR = SCB_ICSR_PENDSVSET;
	
    return TRUE;
}

_Bool
xMBPortEventGet( eMBEventType * eEvent )
{
    _Bool            xEventHappened = FALSE;

    if( xEventInQueue )
    {
        *eEvent = eQueuedEvent;
        xEventInQueue = FALSE;
        xEventHappened = TRUE;
    }
    return xEventHappened;
}


//	PendSV中断
//__irq
//void
//PendSV_Handler ( void )
//{
//	eMBPoll();
//}
void	ModbusPoll( const	void * parg )
{
	for( ;; )
	{	
		eMBPoll();
		osDelay( 10u );
	}
}

