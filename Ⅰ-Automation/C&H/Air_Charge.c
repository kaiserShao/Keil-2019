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
*	�����ÿ���
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
			Air_Charge_EN( false );		//	ʹ�ܼ�����
			Air_Charge_CMD( false );
		}
		LastValue = Value;
	}
}	
	
	
/********  (C) COPYRIGHT 2018 �ൺ���ػ����������޹�˾  **** End Of File ********/
