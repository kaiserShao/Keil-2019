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
/******************************** ����˵�� *************************************
*	
*******************************************************************************/
_Bool	LevelSwitch1Update( void )
{
	static	uint8_t	i = 0u;
	static	uint8_t	j = 0u;

	if( LevelSwitch1Read() )
	{
		j = 0u;
		if( i++ >= 10u )
		{
			i = 10u;
			return	true;
		}
	}
	else
	{
		i = 0u;
		if( j++ >= 10u )
		{
			j = 10u;
			return false;
		}
	}

	return false;
}

/******************************** ����˵�� *************************************
*	
*******************************************************************************/
_Bool	LevelSwitch2Update( void )
{
	static	uint8_t	i = 0u;
	static	uint8_t	j = 0u;

	if( LevelSwitch2Read() )
	{
		j = 0u;
		if( i++ >= 10u )
		{
			i = 10u;
			return	true;
		}
	}
	else
	{
		i = 0u;
		if( j++ >= 10u )
		{
			j = 10u;
			return false;
		}
	}
	
	return false;
}

/******************************** ����˵�� *************************************
*	
*******************************************************************************/
_Bool	LevelSwitch3Update( void )
{
	static	uint8_t	i = 0u;
	static	uint8_t	j = 0u;

	if( LevelSwitch3Read() )
	{
		j = 0u;
		if( i++ >= 10u )
		{
			i = 10u;
			return	true;
		}
	}
	else
	{
		i = 0u;
		if( j++ >= 10u )
		{
			j = 10u;
			return false;
		}
	}

	return false;
}

/******************************** ����˵�� *************************************
*	
*******************************************************************************/
_Bool	LevelSwitch4Update( void )
{
	static	uint8_t	i = 0u;
	static	uint8_t	j = 0u;

	if( LevelSwitch4Read() )
	{
		j = 0u;
		if( i++ >= 10u )
		{
			i = 10u;
			return	true;
		}
	}
	else
	{
		i = 0u;
		if( j++ >= 10u )
		{
			j = 10u;
			return false;
		}
	}

	return false;
}
/******************************** ����˵�� *************************************
*	
*******************************************************************************/
_Bool	LevelSwitch5Update( void )
{
	static	uint8_t	i = 0u;
	static	uint8_t	j = 0u;

	if( LevelSwitch5Read() )
	{
		j = 0u;
		if( i++ >= 10u )
		{
			i = 10u;
			return	true;
		}
	}
	else
	{
		i = 0u;
		if( j++ >= 10u )
		{
			j = 10u;
			return false;
		}
	}
	
	return false;
}

/********  (C) COPYRIGHT 2018 �ൺ���ػ����������޹�˾  **** End Of File ********/
