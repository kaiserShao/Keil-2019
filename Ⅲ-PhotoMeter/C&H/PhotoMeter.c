/**************** (C) COPYRIGHT 2018 青岛中特环保仪器有限公司 *******************
* 文 件 名:  
* 创 建 者:  Kaiser
* 描  述  :  
* 最后修改:  
*********************************** 修订记录 ************************************
* 版  本: 
* 修订人: 
********************************************************************************/
#include "APPDEF.H"

/******************************** 功能说明 *************************************
*	
*******************************************************************************/
void	All_Bleed( void )
{
	static	uint16_t	i = 0u;
	
	usRegHoldingBuf[24u] = LevelSwitch5Update();

	if( usRegHoldingBuf[21u] )
	{		
			if( usRegHoldingBuf[22u] )
			{
				if( i ++ > usRegHoldingBuf[22u] / 30u )
				{
					i = 0u;
					usRegHoldingBuf[22u] = 0u;
					usRegHoldingBuf[21u] = 0u;
					Air_Bleed_Driver( usRegHoldingBuf[21u] );
					usRegHoldingBuf[23u] = 0u;
				}
				else
				{
					Air_Bleed_Driver( usRegHoldingBuf[21u] );
					usRegHoldingBuf[23u] = 1u;
				}	
			}				
			if( usRegHoldingBuf[24u] )
			{
				i = 0u;
				usRegHoldingBuf[23u] = 1u;
			}
	}
	else
	{
		i = 0u;
		Air_Bleed_Driver( 0u );	
		usRegHoldingBuf[23u] = 0u;
	}

}

/******************************** 功能说明 *************************************
*	
*******************************************************************************/
uint16_t	Version = 100u;
uint16_t	CompileYear = 2018u;
uint16_t	CompileMonthDate = 207u;
uint16_t	PWM = 9900u;
	void	DataAccess( void )
{
	static	uint16_t	PWMControlREG = 0u;	
	uint8_t	PWM_Out[4u];
//	uint8_t i,t;

	usRegHoldingBuf[1u] = Version; 
	usRegHoldingBuf[2u] = CompileYear;
	usRegHoldingBuf[3u] = CompileMonthDate;

	//	机械臂运行
	CoordinateSet = usRegHoldingBuf[5u];
	usRegHoldingBuf[6u] = CoordinatePosition;
	//	TODO： 机械臂速度设置

	ReagentSwitch1( usRegHoldingBuf[10u] );
	ReagentSwitch2( usRegHoldingBuf[12u] );
	ReagentSwitch3( usRegHoldingBuf[14u] );
	ReagentSwitch4( usRegHoldingBuf[16u] );

	usRegHoldingBuf[11u] = SwitchState[0u];
	usRegHoldingBuf[13u] = SwitchState[1u];
	usRegHoldingBuf[15u] = SwitchState[2u];
	usRegHoldingBuf[17u] = SwitchState[3u];
	
	usRegHoldingBuf[29u] = LevelSwitch1Update();
	usRegHoldingBuf[34u] = LevelSwitch2Update();
	usRegHoldingBuf[39u] = LevelSwitch3Update();
	usRegHoldingBuf[44u] = LevelSwitch4Update();
	
	All_Bleed();	//	废液处理

	for( uint8_t i = 0u; i < SyringePump_CHMax; i ++ )	//	注射泵
	{
		VPSitionSet[(enum	enumPumpChannelSelect)i] = usRegHoldingBuf[25u + i * 5u];
		usRegHoldingBuf[26u + i * 5u] = VPPosition[(enum	enumPumpChannelSelect)i];
		VPSpeedSet[(enum	enumPumpChannelSelect)i] = usRegHoldingBuf[27u + i * 5u];
		usRegHoldingBuf[28u + i * 5u] = VPState[(enum	enumPumpChannelSelect)i];
	}
	
	if( usRegHoldingBuf[50u] != PWMControlREG )	//	通道选择
	{
		PWMControlREG = usRegHoldingBuf[50u];
		
		if( PWMControlREG & 0x01u )
			PWM_Out[0u] = CalibrateData[CH1].LEDSet;
		else
			PWM_Out[0u] = 0u;
		if( PWMControlREG & 0x02u )
			PWM_Out[1u] = CalibrateData[CH2].LEDSet;
		else
			PWM_Out[1u] = 0u;
		if( PWMControlREG & 0x04u )
			PWM_Out[2u] = CalibrateData[CH3].LEDSet;
		else
			PWM_Out[2u] = 0u;
		if( PWMControlREG & 0x08u )
			PWM_Out[3u] = CalibrateData[CH4].LEDSet;
		else
			PWM_Out[3u] = 0u;
		
		LED_PWM( PWM_Out[0u], PWM_Out[1u], PWM_Out[2u], PWM_Out[3u] );	//	LED输出开关
			
	}
	
	ADDataConver();
	usRegHoldingBuf[51u] = ConcentrationOut( CH1 );
	usRegHoldingBuf[52u] = ConcentrationOut( CH2 );
	usRegHoldingBuf[53u] = ConcentrationOut( CH3 );
	usRegHoldingBuf[54u] = ConcentrationOut( CH4 );

	if( usRegHoldingBuf[18u])		
	{
		Stirrer_CMD(true);
		if( PWM < 6500 )
			PWM = 9050u;
		Stirrer_CTRL( 10000u, PWM -= 5 );
		usRegHoldingBuf[19u] = 1u;
	}
	else
	{
		Stirrer_CMD(false);
		PWM = 9050u;
		usRegHoldingBuf[19u] = 0u;
	}
	
	if(usRegHoldingBuf[75u])
		SET_BIT( GPIOC->BSRR, GPIO_BSRR_BS10 );
	else
		SET_BIT( GPIOC->BSRR, GPIO_BSRR_BR10 );
	
	if(usRegHoldingBuf[76u])
		SET_BIT( GPIOC->BSRR, GPIO_BSRR_BS11 );
	else
		SET_BIT( GPIOC->BSRR, GPIO_BSRR_BR11 );
	
	if(usRegHoldingBuf[77u])
		SET_BIT( GPIOC->BSRR, GPIO_BSRR_BS12 );
	else
		SET_BIT( GPIOC->BSRR, GPIO_BSRR_BR12 );
	
	if(usRegHoldingBuf[78u])
		SET_BIT( GPIOD->BSRR, GPIO_BSRR_BS2 );
	else
		SET_BIT( GPIOD->BSRR, GPIO_BSRR_BR2 );

}
extern		void	Stirrer_CTRL( uint16_t ARRValue, uint16_t CCR1Value );

//void	MotorC( uint8_t x )
//{
//	switch(x)
//	{
//		case 0u:
//			SET_BIT( GPIOC->BSRR, GPIO_BSRR_BR10 );
//			SET_BIT( GPIOC->BSRR, GPIO_BSRR_BR11 );
//			SET_BIT( GPIOC->BSRR, GPIO_BSRR_BR12 );
//			SET_BIT( GPIOD->BSRR, GPIO_BSRR_BR2 );
//	osDelay(5);			break;
//		case 1u:
//			SET_BIT( GPIOC->BSRR, GPIO_BSRR_BS10 );
//			SET_BIT( GPIOC->BSRR, GPIO_BSRR_BR11 );
//			SET_BIT( GPIOC->BSRR, GPIO_BSRR_BR12 );
//			SET_BIT( GPIOD->BSRR, GPIO_BSRR_BR2 );
//	osDelay(10);
//			break;
//		case 2u:
//			SET_BIT( GPIOC->BSRR, GPIO_BSRR_BR10 );
//			SET_BIT( GPIOC->BSRR, GPIO_BSRR_BS11 );
//			SET_BIT( GPIOC->BSRR, GPIO_BSRR_BR12 );
//			SET_BIT( GPIOD->BSRR, GPIO_BSRR_BR2 );
//	osDelay(10);
//			break;
//		case 3u:
//			SET_BIT( GPIOC->BSRR, GPIO_BSRR_BR10 );
//			SET_BIT( GPIOC->BSRR, GPIO_BSRR_BR11 );
//			SET_BIT( GPIOC->BSRR, GPIO_BSRR_BS12 );
//			SET_BIT( GPIOD->BSRR, GPIO_BSRR_BR2 );
//	osDelay(10);
//			break;
//		case 4u:
//			SET_BIT( GPIOC->BSRR, GPIO_BSRR_BR10 );
//			SET_BIT( GPIOC->BSRR, GPIO_BSRR_BR11 );
//			SET_BIT( GPIOC->BSRR, GPIO_BSRR_BR12 );
//			SET_BIT( GPIOD->BSRR, GPIO_BSRR_BS2 );
//	osDelay(10);
//			break;				
//	}	
//}
/******************************** 功能说明 *************************************
*	
*******************************************************************************/
#define ModbusAddress 3u
int	main( void )
{
	BIOS_Init();
	usRegHoldingBuf[0u] = 0u;
	usRegHoldingBuf[4u] = 0u;
	MODBUS_Init( ModbusAddress );
	while(usRegHoldingBuf[4u] != 1u)
		osDelay( 50u );
	Arm_Init();
	CoordinateSet = usRegHoldingBuf[5u] = 1520u;
	while( usRegHoldingBuf[6u] != usRegHoldingBuf[5u] )
	{
		usRegHoldingBuf[6u] = CoordinatePosition;	
		osDelay( 50u );
	}
	usRegHoldingBuf[0u] = 1u;
	while(usRegHoldingBuf[4u] != 2u)
		osDelay( 50u );
	CoordinateSet = usRegHoldingBuf[5u] = 5600u;
	while( usRegHoldingBuf[6u] != usRegHoldingBuf[5u] )
	{
		usRegHoldingBuf[6u] = CoordinatePosition;	
		osDelay( 50u );
	}
	SyringePump_CTRL_Init();
	osDelay( 50u );
	usRegHoldingBuf[0u] = 2u;
	while(1u)
	{
//		usRegHoldingBuf[0u] ++;
		DataAccess();
		osDelay(30u);
	}

}






/********  (C) COPYRIGHT 2018 青岛中特环保仪器有限公司  **** End Of File ********/
