/**************** (C) COPYRIGHT 2018 青岛中特环保仪器有限公司 *******************
* 文 件 名:  
* 创 建 者:  Kaiser
* 描  述  :  
* 最后修改:  
*********************************** 修订记录 ************************************
* 版  本: 
* 修订人: 
********************************************************************************/
#include "BIOS.H"
/********************************** 功能说明 *************************************
*	空气泵控制
*********************************************************************************/

void	Air_Charge_Driver( uint16_t Value )
{
	static	uint16_t	LastValue;
	if( Value != LastValue )
	{
		if( Value ) 
		{
			uint16_t V = Value + 50u;;
			Air_Charge_EN( true );
			Air_Charge_DIR( true );
			Air_Charge_CMD( true );
			while( V != Value)
			{
				Air_Charge_CTRL(V, V / 2u);
				osDelay(50u);
				V -= 10u;
			}
			Air_Charge_CTRL(Value, Value / 2u);
		}
		else
		{
			Air_Charge_EN( false );		//	使能计数器
			Air_Charge_CMD( false );
		}
		LastValue = Value;
	}
}	
	
	
/********  (C) COPYRIGHT 2018 青岛中特环保仪器有限公司  **** End Of File ********/
