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

/******************************** 功能说明 *************************************
*	
*******************************************************************************/
enum	enumPumpChannelSelect
{
	SyringePump_CH1,
	SyringePump_CH2,
	SyringePump_CH3,
	SyringePump_CH4,
	SyringePump_CHMax,

};
extern	_Bool	RxUpToDate;
extern	uint16_t	VPState[4u];
extern	uint16_t	VPPosition[4u];
extern	int16_t	VPSitionSet[SyringePump_CHMax];
extern	int16_t	VPSpeedSet[SyringePump_CHMax];
extern	void	SyringePump_CTRL_Init( void );


/******************************** 功能说明 *************************************
*	
*******************************************************************************/
extern	_Bool	SwitchState[4u];

extern	void	ReagentSwitch1( _Bool	State );
extern	void	ReagentSwitch2( _Bool	State );
extern	void	ReagentSwitch3( _Bool	State );
extern	void	ReagentSwitch4( _Bool	State );

/******************************** 功能说明 *************************************
*	
*******************************************************************************/
extern	_Bool	LevelSwitch1Update( void );
extern	_Bool	LevelSwitch2Update( void );
extern	_Bool	LevelSwitch3Update( void );
extern	_Bool	LevelSwitch4Update( void );
extern	_Bool	LevelSwitch5Update( void );

/******************************** 功能说明 *************************************
*	
*******************************************************************************/
extern	uint8_t LED_PWM( uint8_t LDT1, uint8_t LDT2, uint8_t LDT3, uint8_t LDT4 );


/******************************** 功能说明 *************************************
*	
*******************************************************************************/
extern	void	Air_Bleed_Driver( uint16_t Value );

/******************************** 功能说明 *************************************
*	
*******************************************************************************/
enum	enumLEDChannelSelect
{
	CH1 = 0u,
	CH2,
	CH3,
	CH4,
	CH_Max,
};
struct	uCalibrateData
{
	int16_t		XY[2u][6u];		//	浓度值和对应AD值
	uint16_t	WaterAD;			//	蒸馏水AD值
	float			Scaling;			//	比例系数
	float			Intercept;		//	截距
	uint8_t		LEDSet;				//	灯亮度
};
extern	struct	uCalibrateData CalibrateData[CH_Max];

extern	_Bool	CalibrationSave( void );
extern	void	CalibrationLoad( void );

extern	void	ADDataConver(void);
extern	float	ConcentrationOut( enum	enumLEDChannelSelect	Channel );
extern	uint16_t	ADC_OUT[CH_Max];

/********************************** 功能说明 *************************************
*	
*********************************************************************************/
extern	uint32_t	CoordinateSet;
extern	uint32_t	CoordinatePosition;
extern	uint8_t		Arm_Init( void );


/********************************** 功能说明 *************************************
*	
*********************************************************************************/

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
