/**************** (C) COPYRIGHT 2018 青岛中特环保仪器有限公司 *******************
* 文 件 名:  
* 创 建 者:  Kaiser
* 描  述  :  
* 最后修改:  
*********************************** 修订记录 ***********************************
* 版  本: 
* 修订人: 
*******************************************************************************/
#ifndef _BIOS_H_
#define _BIOS_H_

#include "stdbool.h"
#include "stm32f10x.h"
#include "cmsis_os.h"
/******************************** 功能说明 *************************************
*	extern
*******************************************************************************/
extern	void		UART1_Send( uint8_t OutByte );
extern	uint8_t	UART1_Received( void );
extern	void		UART2_Send( uint8_t OutByte );
extern	uint8_t	UART2_Received( void );
extern	void		UART3_Send( uint8_t OutByte );
extern	uint8_t	UART3_Received( void );
/********************************** 功能说明 *************************************
*	
*********************************************************************************/
extern	void	PWM_CTRL_X( uint16_t ARRValue, uint16_t CCR3Value );
extern	void	PWM_CTRL_Y( uint16_t ARRValue, uint16_t CCR3Value );
extern	void	PWM_CTRL_Z( uint16_t ARRValue, uint16_t CCR3Value );

extern	void	PWM_CMD_X( _Bool	CMD );
extern	void	PWM_CMD_Y( _Bool	CMD );
extern	void	PWM_CMD_Z( _Bool	CMD );

extern	void	RBOOT_ARM_EN_X( _Bool State );
extern	void	RBOOT_ARM_EN_Y( _Bool State );
extern	void	RBOOT_ARM_EN_Z( _Bool State );

extern	void	RBOOT_ARM_DIR_X( _Bool	State );
extern	void	RBOOT_ARM_DIR_Y( _Bool	State );
extern	void	RBOOT_ARM_DIR_Z( _Bool	State );
	
extern	volatile	uint32_t	SetPluseCount[3u];			//	设置运行脉冲数
extern	volatile	uint32_t	PluseCount[3u];					//	实际运行脉冲数
extern	volatile	_Bool			CountOver[3u];					//	计数器溢出标志

/********************************** 功能说明 *************************************
*	
*********************************************************************************/
extern	void	EXTIx_IRQ_Enable( uint8_t EXTIx );
extern	void	EXTIx_IRQ_Disable( uint8_t EXTIx );

extern	uint32_t	TIMER_F;


#endif 
/********  (C) COPYRIGHT 2018 青岛中特环保仪器有限公司  **** End Of File ********/
