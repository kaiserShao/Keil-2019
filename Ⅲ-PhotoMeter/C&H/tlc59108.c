/**************** (C) COPYRIGHT 2018 青岛中特环保仪器有限公司 *******************
* 文 件 名:  
* 创 建 者:  
* 描  述  :  
* 最后修改:  
*********************************** 修订记录 ***********************************
* 版  本: 
* 修订人: 
*******************************************************************************/
#include "BIOS.H"
#define	_SLAVE_59108		0x91u

#define SLAVE_ADDR		0x92         
/******************************** 功能说明 *************************************
*	LED_PWM
*******************************************************************************/
uint8_t LED_PWM( uint8_t LDT1, uint8_t LDT2, uint8_t LDT3, uint8_t LDT4 )
{
	if ( ! bus_i2c_start( _SLAVE_59108, I2C_Write ))
	{
		bus_i2c_stop();
		return	false;
	}
	if ( ! bus_i2c_shout( 0x80u ))	//	CTRL  所有寄存器增量配置
	{
		bus_i2c_stop();
		return	false;
	}
	if ( ! bus_i2c_shout( 0x01u ))	//	mode1
	{
		bus_i2c_stop();
		return	false;
	}
	if ( ! bus_i2c_shout( 0x00u ))	//	mode2
	{
		bus_i2c_stop();
		return	false;
	}
	if ( ! bus_i2c_shout( LDT1 ))		//	pwm0
	{
		bus_i2c_stop();
		return	false;
	}
	if ( ! bus_i2c_shout( LDT2 ))		//	pwm1
	{
		bus_i2c_stop();
		return	false;
	}
	if ( ! bus_i2c_shout( LDT3 ))		//	pwm2
	{
		bus_i2c_stop();
		return	false;
	}
	if ( ! bus_i2c_shout( LDT4 ))		//	pwm3
	{
		bus_i2c_stop();
		return	false;
	}
	if ( ! bus_i2c_shout( 00u ))		//	pwm4
	{
		bus_i2c_stop();
		return	false;
	}
	if ( ! bus_i2c_shout( 00u ))		//	pwm5
	{
		bus_i2c_stop();
		return	false;
	}
	if ( ! bus_i2c_shout( 00u ))		//	pwm6
	{
		bus_i2c_stop();
		return	false;
	}
	if ( ! bus_i2c_shout( 00u ))		//	pwm7
	{
		bus_i2c_stop();
		return	false;
	}
	if ( ! bus_i2c_shout( 0xFFu ))	//	grppwm
	{
		bus_i2c_stop();
		return	false;
	}
	if ( ! bus_i2c_shout( 0x00u ))	//	grpfreq
	{
		bus_i2c_stop();
		return	false;
	}
	if ( ! bus_i2c_shout( 0xAAu ))	//	ledout0
	{
		bus_i2c_stop();
		return	false;
	}

	bus_i2c_stop();

  return SET; 
}
