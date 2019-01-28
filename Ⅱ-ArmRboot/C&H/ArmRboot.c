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

	
	CoordinateSet[0u] = usRegHoldingBuf[5u];
	CoordinateSet[1u] = usRegHoldingBuf[10u];
	CoordinateSet[2u] = usRegHoldingBuf[15u];

	usRegHoldingBuf[6u]  = CoordinatePosition[0];
	usRegHoldingBuf[11u] = CoordinatePosition[1];
	usRegHoldingBuf[16u] = CoordinatePosition[2];

	if( usRegHoldingBuf[9u] )
	{
		X_FindZero();
		CoordinateSet[2u] = usRegHoldingBuf[15u] = 0u;
		usRegHoldingBuf[9u] = 0u;
	}

	if( usRegHoldingBuf[14u] )
	{
		Y_FindZero();
		CoordinateSet[2u] = usRegHoldingBuf[15u] = 0u;
		usRegHoldingBuf[14u] = 0u;
	}
	
	if( usRegHoldingBuf[19u] )
	{
		Z_FindZero();
		CoordinateSet[2u] = usRegHoldingBuf[15u] = 0u;
		usRegHoldingBuf[19u] = 0u;
	}
}
#define ModbusAddress 2u
void	Init( void )
{
	usRegHoldingBuf[0u] = 0u;
	usRegHoldingBuf[4u] = 0u;
	MODBUS_Init( ModbusAddress );
	while(usRegHoldingBuf[4u] != 1u)
		osDelay( 50u );
	Arm_Init();
	usRegHoldingBuf[0u] = 1u;
	while(usRegHoldingBuf[4u] != 2u)
		osDelay( 50u );
	CoordinateSet[0u] = usRegHoldingBuf[5u] = 1900u;
	CoordinateSet[1u] = usRegHoldingBuf[10u] =1200u;
	while(( usRegHoldingBuf[6u] != usRegHoldingBuf[5u] ) || ( usRegHoldingBuf[11u] != usRegHoldingBuf[10u] ) )
	{
		usRegHoldingBuf[6u]  = CoordinatePosition[0u];
		usRegHoldingBuf[11u] = CoordinatePosition[1u];			
		osDelay(50u);
	}
	CoordinateSet[2u] = usRegHoldingBuf[15u] = 12700;
	while( usRegHoldingBuf[16u] != usRegHoldingBuf[15u] )
	{
		usRegHoldingBuf[16u] = CoordinatePosition[2u];	
		osDelay( 50u );
	}
	usRegHoldingBuf[0u] = 2u;
	while(usRegHoldingBuf[4u] != 3u)
		osDelay( 50u );
	CoordinateSet[2u] = usRegHoldingBuf[15u] = 400;
	while( usRegHoldingBuf[16u] != usRegHoldingBuf[15u] )
	{
		usRegHoldingBuf[16u] = CoordinatePosition[2u];	
		osDelay( 50u );
	}
	CoordinateSet[0u] = usRegHoldingBuf[5u] = 400;
	CoordinateSet[1u] = usRegHoldingBuf[10u] = 400;
	while(( usRegHoldingBuf[6u] != usRegHoldingBuf[5u] ) || ( usRegHoldingBuf[11u] != usRegHoldingBuf[10u] ) )
	{
		usRegHoldingBuf[6u]  = CoordinatePosition[0u];
		usRegHoldingBuf[11u] = CoordinatePosition[1u];			
		osDelay(50u);
	}
	usRegHoldingBuf[0u] = 3u;

}


/******************************** 功能说明 *************************************
*	
*******************************************************************************/

int	main( void )
{
	BIOS_Init();
	Init();
	while(1u)
	{
		osDelay( 30u );
		DataAccess();
	}

}














/********  (C) COPYRIGHT 2018 青岛中特环保仪器有限公司  **** End Of File ********/
