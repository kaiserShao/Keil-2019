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
void	Air_Charge( void )
{
	static	uint16_t	i = 0u;
	if( usRegHoldingBuf[21u] )
	{		
		if( usRegHoldingBuf[9u] )
		{
			i = 0u;
			Air_Charge_Driver( usRegHoldingBuf[21u] );		
			usRegHoldingBuf[23u] = 1u;
		}
		else
		{
			if( usRegHoldingBuf[22u] )
			{
				if( i ++ > usRegHoldingBuf[22u] / 30u )
				{
					i = 0u;
					usRegHoldingBuf[22u] = 0u;
					usRegHoldingBuf[21u] = 0u;
					Air_Charge_Driver( usRegHoldingBuf[21u] );
					usRegHoldingBuf[23u] = 0u;
					osDelay(1000);
					usRegHoldingBuf[10u] = 0u;
				}
				else
				{
					Air_Charge_Driver( usRegHoldingBuf[21u] );
					usRegHoldingBuf[23u] = 1u;
				}					
			}
		}
	}
	else
	{
		i = 0u;
		Air_Charge_Driver( 0u );	
		usRegHoldingBuf[23u] = 0u;
	}
	
	Air_Charge_Valve( usRegHoldingBuf[10u] );
	usRegHoldingBuf[11u] = usRegHoldingBuf[10u];

}

/******************************** 功能说明 *************************************
*	
*******************************************************************************/
void	Air_Bleed( void )
{
	static	uint16_t	i = 0u;
	if( usRegHoldingBuf[26u] )		//	设定速度不为零
	{		
		if( usRegHoldingBuf[9u] )	//	有液体存在
		{
			i = 0u;
			Air_Bleed_Driver( usRegHoldingBuf[26u] );		
			usRegHoldingBuf[28u] = 1u;
		}
		else
		{
			if( usRegHoldingBuf[27u] )		//	工作时间有限制
			{
				if( i ++ > usRegHoldingBuf[27u] / 30u )
				{
					i = 0u;
					usRegHoldingBuf[27u] = 0u;
					usRegHoldingBuf[26u] = 0u;
					Air_Bleed_Driver( usRegHoldingBuf[26u] );
					usRegHoldingBuf[28u] = 0u;
					osDelay(500);
					usRegHoldingBuf[12u] = 0u;
				}
				else
				{
					Air_Bleed_Driver( usRegHoldingBuf[26u] );
					usRegHoldingBuf[28u] = 1u;
				}					
			}
		}
	}
	else
	{
		i = 0u;
		Air_Bleed_Driver( 0u );
		usRegHoldingBuf[28u] = 0u;
	}
	
	Air_Bleed_Valve( usRegHoldingBuf[12u] );
	usRegHoldingBuf[13u] = usRegHoldingBuf[12u];
}

/******************************** 功能说明 *************************************
*	
*******************************************************************************/
void	WaterIn( void )
{
	Injection_Water_Valve( usRegHoldingBuf[14u] );
	usRegHoldingBuf[15u] = usRegHoldingBuf[14u];
}

/******************************** 功能说明 *************************************
*	液体存在缺失更新
*******************************************************************************/
void	ExistUpdate( void )
{
	static	uint8_t	i = 0u;

	usRegHoldingBuf[9u] = CommonUpdate();	//	液位检测更新
	
	if( (usRegHoldingBuf[6u] < usRegHoldingBuf[5u]) )	//	注射泵当前位置小于目标位置
	{
		if( usRegHoldingBuf[9u] == 0u )
		{
			if( i ++ > 30u )		//	
			{
				i = 0u;
				usRegHoldingBuf[19u] = 0;
			}
		}
		else
		{
			i = 0u;
			usRegHoldingBuf[19u] = 1;
		}
	}
}


/********************************** 功能说明 *************************************
*	数据交换处理
*********************************************************************************/
uint16_t	Version = 100u;
uint16_t	CompileYear = 2018u;
uint16_t	CompileMonthDate = 1020u;
void	DataAccess( void )
{
	usRegHoldingBuf[1u] = Version; 
	usRegHoldingBuf[2u] = CompileYear;
	usRegHoldingBuf[3u] = CompileMonthDate;

	//	机械手指任务数据交换
	CoordinateSet = usRegHoldingBuf[30u];
	usRegHoldingBuf[31u] = CoordinatePosition;
	//	注射泵任务数据交换
	usRegHoldingBuf[6u] = VPPosition;
	usRegHoldingBuf[8u] = VPState;
	
	ExistUpdate();	//	液体存在更新
	
	Air_Charge();		//	定量泵

	Air_Bleed();		//	排空泵
	
	WaterIn();			//	蒸馏水阀
	
	Angel = usRegHoldingBuf[35u];
	usRegHoldingBuf[36u] = AngelRT;
//	Rotate_Servo_Angel( usRegHoldingBuf[35u] );	//	舵机旋转角度
	
}
/******************************** 功能说明 *************************************
*	
*******************************************************************************/
#define ModbusAddress 1u
int	main( void )
{
	BIOS_Init();
	usRegHoldingBuf[0u] = 0u;
	usRegHoldingBuf[4u] = 0u;
	MODBUS_Init( ModbusAddress );
	Arm_Init();
	while(usRegHoldingBuf[4u] != 1u)
		osDelay( 50u );
	Angel = usRegHoldingBuf[35u] = 0u;
	Rotate_Servo_Init();
	while( AngelRT != Angel)
		osDelay( 50u );
	Angel = usRegHoldingBuf[35u] = 100;
	while( AngelRT != Angel)
		osDelay( 50u );
	usRegHoldingBuf[0u] = 1u;
	while(usRegHoldingBuf[4u] != 2u)
		osDelay( 50u );
	usRegHoldingBuf[14u] = 1u;
	WaterIn();			//	蒸馏水阀
	SyringePump_CTRL_Init();
	usRegHoldingBuf[0u] = 2u;
	while(1)
	{
		osDelay( 30u );
		DataAccess();
	}

}














/********  (C) COPYRIGHT 2018 青岛中特环保仪器有限公司  **** End Of File ********/
