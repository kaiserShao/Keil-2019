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
extern	void	delay_us ( uint32_t us );
/******************************** 功能说明 *************************************
*	extern  串口
*******************************************************************************/
extern	void		UART1_Send( uint8_t OutByte );
extern	uint8_t	UART1_Received( void );
extern	void		UART2_Send( uint8_t OutByte );
extern	uint8_t	UART2_Received( void );
extern  void		RS485_REDE2( _Bool state );
extern	void		UART3_Send( uint8_t OutByte );
extern	uint8_t	UART3_Received( void );
/********************************** 功能说明 *************************************
*	TIM
*********************************************************************************/
extern		void	Air_Charge_CTRL( uint16_t ARRValue, uint16_t CCR3Value );
extern		void	Air_Charge_CMD( _Bool	CMD );
extern		void	Air_Charge_DIR( _Bool	State );
extern		void	Air_Charge_EN( _Bool State );

extern		void	Air_Bleed_CTRL( uint16_t ARRValue, uint16_t CCR3Value );
extern		void	Air_Bleed_CMD( _Bool	CMD );
extern		void	Air_Bleed_DIR( _Bool	State );
extern		void	Air_Bleed_EN( _Bool State );

extern		void	Rotate_Servo_CTRL( uint16_t ARRValue, uint16_t CCR1Value );
extern		void	Rotate_Servo_CMD( _Bool	CMD );

extern		void	Fingers_CTRL( uint16_t ARRValue, uint16_t CCR1Value );
extern		void	Fingers_DIR( _Bool	State );
extern		void	Fingers_CMD( _Bool	CMD );
extern	uint32_t	PWMInCount( uint32_t	CountTarget );

extern		void	SolenoidValve1_Cmd( _Bool	Newstate );
extern		void	SolenoidValve2_Cmd( _Bool	Newstate );
extern		void	SolenoidValve3_Cmd( _Bool	Newstate );

extern	volatile	uint32_t	SetPluseCount;			//	设置运行脉冲数
extern	volatile	uint32_t	PluseCount;					//	实际运行脉冲数
extern	volatile	_Bool			CountOver;					//	计数器溢出标志

/******************************** 功能说明 *************************************
*	
*******************************************************************************/
extern    _Bool	LevelSwitch1Read( void );
extern    _Bool	LevelSwitch2Read( void );

/********************************** 功能说明 *************************************
*	外部中断
*********************************************************************************/



#endif 
/********  (C) COPYRIGHT 2018 青岛中特环保仪器有限公司  **** End Of File ********/
