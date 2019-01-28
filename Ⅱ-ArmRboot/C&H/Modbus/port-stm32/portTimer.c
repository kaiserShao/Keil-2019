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
 * File: $Id: porttimer.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- static functions ---------------------------------*/
static void prvvTIMERExpiredISR( void );

/* ----------------------- Start implementation -----------------------------*/
_Bool
xMBPortTimersInit( USHORT usTim1Timerout50us )
{
	//	ģ��ʹ��
	SET_BIT( RCC->APB2ENR, RCC_APB2ENR_TIM1EN );
//	//	ʱ����ʼ��
	MB_TIMER->CR1  = 0u;										/* Counter Up, Division: 1 */
	MB_TIMER->PSC  = ( SystemCoreClock / 20000u ) - 1u;			/* Set the Prescaler value */
	MB_TIMER->ARR  = usTim1Timerout50us;						/* Set the Autoreload value */
	MB_TIMER->EGR  = TIM_EGR_UG;           						//	���ɸ����¼����������� PSC
	MB_TIMER->DIER  = TIM_DIER_UIE;													//	ʹ���ж�
	SET_BIT( TIM1->BDTR, TIM_BDTR_MOE );
	//	�����ж�
//	NVIC_SetPriority( TIM4_IRQn, 255u );
	NVIC_EnableIRQ( TIM1_UP_IRQn );
	
	return TRUE;
}

void
vMBPortTimersEnable(  )
{
    /* Enable the timer with the timeout passed to xMBPortTimersInit( ) */
	CLEAR_BIT( MB_TIMER->SR, TIM_SR_UIF );
	SET_BIT( MB_TIMER->DIER, TIM_DIER_UIE );
	MB_TIMER->CNT = 0u;
	SET_BIT( MB_TIMER->CR1, TIM_CR1_CEN );
}

void
vMBPortTimersDisable(  )
{
    /* Disable any pending timers. */
	CLEAR_BIT( MB_TIMER->CR1, TIM_CR1_CEN );
	MB_TIMER->CNT = 0u;
	CLEAR_BIT( MB_TIMER->DIER, TIM_DIER_UIE );
	CLEAR_BIT( MB_TIMER->SR, TIM_SR_UIF );
	
}

/* Create an ISR which is called whenever the timer has expired. This function
 * must then call pxMBPortCBTimerExpired( ) to notify the protocol stack that
 * the timer has expired.
 */
static void prvvTIMERExpiredISR( void )
{
    ( void )pxMBPortCBTimerExpired(  );
}

//	��ʱ���ж�
__irq
void
TIM1_UP_IRQHandler( void )
{
	if ( READ_BIT( MB_TIMER->SR, TIM_SR_UIF ))
	{
		/* Clear TIM Capture Compare1 interrupt pending bit*/
		CLEAR_BIT( MB_TIMER->SR, TIM_SR_UIF );
		prvvTIMERExpiredISR( );
	}
}
