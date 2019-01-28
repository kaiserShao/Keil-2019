/**************** (C) COPYRIGHT 2018 青岛中特环保仪器有限公司 *******************
* 文 件 名:  
* 创 建 者:  	Kaiser
* 描  述  :  用于驱动层与应用层程序沟通 
* 最后修改:  
*********************************** 修订记录 ***********************************
* 版  本: 
* 修订人: 
*******************************************************************************/
#ifndef _APPDEF_H_
#define _APPDEF_H_
#include "BIOS.H"
#include "stdbool.h"
#include "stm32f10x.h"
#include "cmsis_os.h"
/******************************** 功能说明 *************************************
*	
*******************************************************************************/
extern	void	BIOS_Init( void );
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


extern	_Bool	CalibrationSave( void );
extern	void	CalibrationLoad( void );

extern	void	ADDataConver(void);
extern	uint16_t	ADC_OUT[3u];            

/********************************** 功能说明 *************************************
*	
*********************************************************************************/
extern	uint32_t	CoordinateSet[2u];
extern	uint32_t	CoordinatePosition[2u];
extern	uint8_t		Arm_Init( void );


/******************************** 功能说明 *************************************
*	
*******************************************************************************/
extern	void  Heat_Run_Cmd( _Bool	NewState );
extern	uint16_t	HeatTemp_Set;
/********************************** 功能说明 *************************************
*	
*********************************************************************************/
extern	double	TempConver( uint32_t	DataOut );

/********************************** 功能说明 *************************************
*	
*********************************************************************************/
extern	void	ProcessCTRL_Init( void );
extern	void	__Task_RTStatus( void );



//extern	struct	uCalibrateData CalibrateData[CH_Max];

//extern	float	ConcentrationCovert( uint16_t	AD, enum	enumChannelSelect	ChannelSelect );
//extern	_Bool LeastSquareLinearFit( struct	uCalibrateData * CalibrateData );












#endif
/********  (C) COPYRIGHT 2018 青岛中特环保仪器有限公司  **** End Of File ********/
