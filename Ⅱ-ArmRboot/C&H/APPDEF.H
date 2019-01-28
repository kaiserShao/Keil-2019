/**************** (C) COPYRIGHT 2018 青岛中特环保仪器有限公司 *******************
* 文 件 名:  
* 创 建 者:  	Kaiser
* 描  述  :  
* 最后修改:  
*********************************** 修订记录 ***********************************
* 版  本: 
* 修订人: 
*******************************************************************************/
#ifndef _APPDEF_H_
#define _APPDEF_H_

#include "stdbool.h"
#include "stm32f10x.h"
#include "cmsis_os.h"
/******************************** 功能说明 *************************************
*	
*******************************************************************************/
extern	int	BIOS_Init( void );
extern	void	delay_us ( uint32_t us );

/******************************** 功能说明 *************************************
*	
*******************************************************************************/
extern	void	ModbusPoll( const	void * parg );
extern	void	MODBUS_Init( uint8_t MBAddress );
extern	uint16_t	usRegInputBuf[];
extern	uint16_t	usRegHoldingBuf[];
extern	uint8_t		ucRegDiscBuf[];
extern	uint8_t		ucRegCoilsBuf[];



/********************************** 功能说明 *************************************
*	
*********************************************************************************/
extern	uint32_t	CoordinateSet[3u];
extern	uint32_t	CoordinatePosition[3u];
extern	void	Arm_Init( void );
extern	void	X_FindZero(void);
extern	void	Y_FindZero(void);
extern	void	Z_FindZero(void);








#endif
/********  (C) COPYRIGHT 2018 青岛中特环保仪器有限公司  **** End Of File ********/
