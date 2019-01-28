/**************** (C) COPYRIGHT 2018 �ൺ���ػ����������޹�˾ *******************
* �� �� ��:  
* �� �� ��:  Kaiser
* ��  ��  :  
* ����޸�:  
*********************************** �޶���¼ ************************************
* ��  ��: 
* �޶���: 
********************************************************************************/
#include "BIOS.H"
/********************************** ����˵�� *************************************
*	��Һ�ÿ���
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
			Air_Bleed_EN( false );		//	ʹ�ܼ�����
			Air_Bleed_CMD( false );
		}
		LastValue = Value;
	}
}	


/********  (C) COPYRIGHT 2018 �ൺ���ػ����������޹�˾  **** End Of File ********/
