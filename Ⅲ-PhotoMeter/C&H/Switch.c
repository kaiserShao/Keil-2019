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

_Bool	SwitchState[4u];

/********************************** 功能说明 *************************************
*	废液阀控制
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

/******************************** 功能说明 *************************************
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

/******************************** 功能说明 *************************************
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

/******************************** 功能说明 *************************************
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


/********  (C) COPYRIGHT 2018 青岛中特环保仪器有限公司  **** End Of File ********/
