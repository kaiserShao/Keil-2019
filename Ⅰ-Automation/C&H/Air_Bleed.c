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
*	废液泵控制
*********************************************************************************/

void	Air_Bleed_Driver( uint16_t Value )
{
	static	uint16_t	LastValue;
	if( Value != LastValue )
	{
		if( Value ) 
		{
			uint16_t V = Value + 50u;
			Air_Bleed_EN( true );
			Air_Bleed_DIR( true );
			Air_Bleed_CMD( true );
			while( V != Value )
			{
				Air_Bleed_CTRL(V, V / 2u);
				osDelay(50u);
				V -= 10u;
			}
			Air_Bleed_CTRL( Value, Value / 2u );
		}
		else
		{
			Air_Bleed_EN( false );		//	使能计数器
			Air_Bleed_CMD( false );
		}
		LastValue = Value;
	}
}	


/********  (C) COPYRIGHT 2018 青岛中特环保仪器有限公司  **** End Of File ********/
