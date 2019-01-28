/**************** (C) COPYRIGHT 2018 青岛中特环保仪器有限公司 *******************
* 文 件 名:  
* 创 建 者:  Kaiser
* 描  述  :  用于底层与驱动层程序沟通  
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
*	
*******************************************************************************/
extern	void	BIOS_Init( void );
extern	void	delay_us ( uint32_t us );
extern	volatile	uint16_t 	ADC_Databuf[16];	//	ADC-DMA转换数据缓冲区

/******************************** 功能说明 *************************************
*	
*******************************************************************************/
enum	I2C_DirectSet
{
	I2C_Write, 
	I2C_Read,
};
enum	I2C_AcknowlegeSet
{
	I2C_ACK,
	I2C_NoACK,
};
extern	_Bool	bus_i2c_start( uint8_t Address8Bit, enum I2C_DirectSet DirectSet );
extern	void	bus_i2c_stop( void );
extern	_Bool	bus_i2c_shout( uint8_t OutByte );
extern	uint8_t	bus_i2c_shin( enum I2C_AcknowlegeSet AcknowlegeSet );
extern	void	BIOS_I2C_Init( void );
extern	void	I2C_SDA_Out( void );
extern	void	I2C_SDA_In( void );

/********************************** 功能说明 *************************************
*	
*********************************************************************************/
extern	void	PWM_CTRL( uint16_t ARRValue, uint16_t CCR3Value );
extern	void	PWM_CMD( _Bool	CMD );
extern	void	RBOOT_ARM_EN( _Bool State );
extern	void	RBOOT_ARM_DIR( _Bool	State );
extern	void	EXTIx_IRQ_Enable( uint8_t EXTIx );
extern	void	EXTIx_IRQ_Disable( uint8_t EXTIx );

extern	volatile	uint32_t	SetPluseCount;			//	设置运行脉冲数
extern	volatile	uint32_t	PluseCount;					//	实际运行脉冲数
extern	volatile	_Bool			CountOver;					//	计数器溢出标志

/******************************** 功能说明 *************************************
*	
*******************************************************************************/
extern	void	Air_Bleed_CTRL( uint16_t ARRValue, uint16_t CCR3Value );
extern	void	Air_Bleed_CMD( _Bool	CMD );
extern	void	Air_Bleed_DIR( _Bool	State );
extern	void	Air_Bleed_EN( _Bool State );
	

/******************************** 功能说明 *************************************
*	
*******************************************************************************/
extern	void	Stirrer_CTRL( uint16_t ARRValue, uint16_t CCR1Value );
extern	void	Stirrer_CMD( _Bool	CMD );

/******************************** 功能说明 *************************************
*	
*******************************************************************************/
extern	void		UART1_Send( uint8_t OutByte );
extern	uint8_t	UART1_Received( void );
extern	void		UART2_Send( uint8_t OutByte );
extern	uint8_t	UART2_Received( void );
extern  void		RS485_REDE2( _Bool state );
extern	void		UART3_Send( uint8_t OutByte );
extern	uint8_t	UART3_Received( void );

/******************************** 功能说明 *************************************
*	
*******************************************************************************/
extern	void	SolenoidValve1_Cmd( _Bool	Newstate );
extern	void	SolenoidValve2_Cmd( _Bool	Newstate );
extern	void	SolenoidValve3_Cmd( _Bool	Newstate );
extern	void	SolenoidValve4_Cmd( _Bool	Newstate );


/******************************** 功能说明 *************************************
*	
*******************************************************************************/
extern	_Bool	LevelSwitch1Read( void );
extern	_Bool	LevelSwitch2Read( void );
extern	_Bool	LevelSwitch3Read( void );
extern	_Bool	LevelSwitch4Read( void );
extern	_Bool	LevelSwitch5Read( void );

/******************************** 功能说明 *************************************
*	
*******************************************************************************/
extern	void	Mortor_IRQ_Enable( uint8_t EXTIx );
extern	void	Mortor_IRQ_Disable( uint8_t EXTIx );
extern	uint8_t	Mortor_PortRead( void );


#endif 
/********  (C) COPYRIGHT 2018 青岛中特环保仪器有限公司  **** End Of File ********/
