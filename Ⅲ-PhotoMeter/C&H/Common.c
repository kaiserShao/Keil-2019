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
/******************************** 功能说明 *************************************
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

/******************************** 功能说明 *************************************
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

/******************************** 功能说明 *************************************
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

/******************************** 功能说明 *************************************
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
/******************************** 功能说明 *************************************
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

/********  (C) COPYRIGHT 2018 青岛中特环保仪器有限公司  **** End Of File ********/
