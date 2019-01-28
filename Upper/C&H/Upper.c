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

extern	void	PWM_CTRL_CoolerPump( uint16_t ARRValue, uint16_t CCR1Value );
extern	void	PWM_CMD_CoolerPump( _Bool	CMD );
extern	void	CoolerPump_EN( _Bool State );

/******************************** 功能说明 *************************************
*	
*******************************************************************************/
int16_t	PT100Temp[3u];

//void	DataAccess( void )
//{
//	static	uint16_t	Heat_CTRL;
//	static	uint16_t	Cold_CTRL;
//	
//	CoordinateSet[0u] = usRegHoldingBuf[5u];
//	usRegHoldingBuf[6u]  = CoordinatePosition[0];

//	CoordinateSet[1u] = usRegHoldingBuf[10u];
//	usRegHoldingBuf[11u] = CoordinatePosition[1];

//	ADDataConver();
//	PT100Temp[0u] = TempConver(ADC_OUT[0u]);
//	PT100Temp[1u] = TempConver(ADC_OUT[1u]);
//	PT100Temp[2u] = TempConver(ADC_OUT[2u]);
//	HeatTemp_Set =  usRegHoldingBuf[17u];
//	
//	if( Heat_CTRL != usRegHoldingBuf[15u] )
//	{
//		Heat_Run_Cmd( usRegHoldingBuf[15u] );
//		Heat_CTRL = usRegHoldingBuf[15u];
//	}
//	if( Heat_CTRL )
//	{
//		if( HeatTemp_Set - PT100Temp[0u] > 5 )
//			usRegHoldingBuf[16u] = 1u;
//		else	if( HeatTemp_Set - PT100Temp[0u] < -5 )
//			usRegHoldingBuf[16u] = 2u;
//	}
//	else
//		usRegHoldingBuf[16u] = 0u;
//	
//	if( Cold_CTRL != usRegHoldingBuf[20u] )
//	{
//		CoolerPump_EN(usRegHoldingBuf[20u]);
//		PWM_CTRL_CoolerPump( 1000u, 700u );
//		PWM_CMD_CoolerPump( usRegHoldingBuf[20u] );
//		Cold_CTRL = usRegHoldingBuf[20u];
//	}
//	if( Cold_CTRL )
//	{
//		if( usRegHoldingBuf[23u] - PT100Temp[0u] < -5 )
//			usRegHoldingBuf[21u] = 1u;
//		else	if(  usRegHoldingBuf[23u] - PT100Temp[0u] > 1 )
//			usRegHoldingBuf[21u] = 2u;
//	}
//	else
//		usRegHoldingBuf[21u] = 0u;
//	
//}

	/******************************** 功能说明 *************************************
	*	访问 USART1，实现RS232收发
	*******************************************************************************/
extern	void	UART1_Send( uint8_t OutByte );

	/******************************** 功能说明 *************************************
	*	访问 USART1，实现RS232收发
	*******************************************************************************/
extern	uint8_t	UART1_Received( void );


/******************************** 功能说明 *************************************
*	
*******************************************************************************/
uint16_t	Version = 100u;
uint16_t	CompileYear = 2018u;
uint16_t	CompileMonthDate = 207u;
#define ModbusAddress 4u
int	main( void )
{
//	usRegHoldingBuf[1u] = Version; 
//	usRegHoldingBuf[2u] = CompileYear;
//	usRegHoldingBuf[3u] = CompileMonthDate;

	BIOS_Init();
	
//	MODBUS_Init( ModbusAddress );

//	Arm_Init();
//	PWM_CTRL_CoolerPump( 10000u, 10000u );
//	PWM_CMD_CoolerPump(1);
//	CoolerPump_EN(1);
	while(1u)
	{
//		DataAccess();
		UART1_Send(Version++);
		osDelay(2000u);
	}

}














/********  (C) COPYRIGHT 2018 青岛中特环保仪器有限公司  **** End Of File ********/
