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

_Bool	SwitchState[4u];

/********************************** ����˵�� *************************************
*	��Һ������
*********************************************************************************/
void	ReagentSwitch1( _Bool	State )
{
	static	_Bool	OldState;
	if( OldState != State )
	{
		SolenoidValve1_Cmd( State );
		OldState = State;
		SwitchState[0u] = State;
	}
}	

/******************************** ����˵�� *************************************
*	
*******************************************************************************/
void	ReagentSwitch2( _Bool	State )
{
	static	_Bool	OldState;
	if( OldState != State )
	{
		SolenoidValve2_Cmd( State );
		OldState = State;
		SwitchState[1u] = State;
	}
}	

/******************************** ����˵�� *************************************
*	
*******************************************************************************/
void	ReagentSwitch3( _Bool	State )
{
	static	_Bool	OldState;
	if( OldState != State )
	{
		SolenoidValve3_Cmd( State );
		OldState = State;
		SwitchState[2u] = State;
	}
}	

/******************************** ����˵�� *************************************
*	
*******************************************************************************/
void	ReagentSwitch4( _Bool	State )
{
	static	_Bool	OldState;
	if( OldState != State )
	{
		SolenoidValve4_Cmd( State );
		OldState = State;
		SwitchState[3u] = State;
	}
}	


/********  (C) COPYRIGHT 2018 �ൺ���ػ����������޹�˾  **** End Of File ********/
