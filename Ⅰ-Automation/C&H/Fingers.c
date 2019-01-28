/**************** (C) COPYRIGHT 2018 青岛中特环保仪器有限公司 ********************
* 文 件 名:  
* 创 建 者:  Kaiser
* 描  述  :  
* 最后修改:  
*********************************** 修订记录 *************************************
* 版  本: 
* 修订人: 
*********************************************************************************/
#include "BIOS.H"
#include "stdlib.h"

/********************************** 功能说明 *************************************
*	机械手指运行速度设置
*********************************************************************************/
void	Fingers_PWM( uint16_t Value )
{
	Fingers_CTRL( Value, Value / 2u );
}
/********************************** 功能说明 *************************************
*	机械手指PWM开关
*********************************************************************************/
void	Fingers_PWMCmd( _Bool State )
{
	Fingers_CMD( State );
}
/********************************** 功能说明 *************************************
*	机械手指运行方向设置
*********************************************************************************/
void	Fingers_Dir( _Bool State )
{
	Fingers_DIR( State );
}
/********************************** 功能说明 *************************************
*	机械手指控制器使能
*********************************************************************************/
void	Fingers_En( _Bool State )
{
	;
}


#define	Full_X  26000u
#define	Full_Y  16000u
#define	Full_Z  8000u
/********************************** 功能说明 *************************************
*	机械手指运行任务
*********************************************************************************/
uint32_t	CoordinateSet;
uint32_t	CoordinatePosition;

static	uint32_t	Position;
static	_Bool			TaskArmisRunning = 0;

static	uint32_t	CoordinateSetOld;
static	uint32_t	period = 2000u;
static	int		Differ;
static	float 		Velocity = 6.0f;
static	uint32_t	Shifting = 4000u;
static	_Bool	TargetIsOK = true;
#include "math.h"
void	ArmSoftCtrl( void )
{
#define	ShiftPluse	10.0f
#define	LowSpeed		10.0f
#define	VelocityL		5.0f
#define	VelocityH		500.0f
#define	Speeda			20.0f

	for(;;)
	{
		osDelay( 50u );
		if( (CoordinateSet != CoordinateSetOld) && !TaskArmisRunning )
		{

			period   = 2000u;
			Velocity = 6.0f;
			Shifting = 400u;
			
			CoordinateSetOld = CoordinateSet;
			
			TaskArmisRunning = true;																	//	置位防重入标志
			
			PluseCount = 0u;																					//	清零计数脉冲

			Differ = CoordinateSet - CoordinatePosition;	//	计算设置位置与当前位置的偏差和方向
			SetPluseCount = abs( Differ );											//	设置泵运行脉冲
			Position = CoordinatePosition;
			PWMInCount( 0u );																					//	清零TIMCNT
			CountOver = false;				//	清除计数器溢出
			Fingers_Dir		(  ( Differ >= 0 ) ? true : false );				//	选择方向
			Fingers_PWM		( 10000u );																	//	重置pwm为最低输出
//			Fingers_En		( true );																	//	步进电机控制器控制使能	
			Fingers_PWMCmd( true );																		//	开启脉冲输出
			
			//	todo:	加判是否遇到光耦和零位
			
			while( !CountOver )						//	计数器没有溢出
			{
				if( abs( Differ ) <= ShiftPluse * 2 )			//	分类 
				{
					Fingers_PWM( 1000u );		//	脉冲数过少时直接使用低速模式 100p/s
					osDelay( 5u );
				}
				else																		//	脉冲数多时进行加减速控制
				{

					Shifting = abs(Differ);
					if( Shifting > ShiftPluse )
						Shifting = ShiftPluse;
				
					if( PluseCount < Shifting )		//	前Shifting脉冲加速
					{
						Velocity = sqrt( 2* PluseCount * Speeda);
						if(Velocity > VelocityH)
							Velocity = VelocityH;
						period = VelocityH * 10 / Velocity;
						if(Velocity > 90)
							Velocity = 90;
						Fingers_PWM(  period );
					}
					else																	//	中间匀速
					if( PluseCount <= (abs(Differ) - Shifting) )	
					{
						;
					}
					else 																	//	后Shifting脉冲减速
					//	if( PluseCount >= (abs((float)Differ) - 400u) )
					{
					
						Velocity = sqrt( 2* (abs(Differ) - (float)PluseCount) * Speeda);
						if(Velocity < VelocityL )
							Velocity = VelocityL;
						period = VelocityH * 10 / Velocity;
						Fingers_PWM(  period );
					}
					osDelay( 5u );
					if( Differ >= 0 )	//	根据方向偏差来计算当前位置
					{
						CoordinatePosition = Position + PluseCount;  
					}
					else
					{
						CoordinatePosition = Position - PluseCount; 
					}
				}
				
				if( Differ > 0 )	//	根据最终计数来计算最终位置
				{
					CoordinatePosition = Position + PluseCount; 
				}
				else
				{
					CoordinatePosition = Position - PluseCount; 
				}
			}
			
			TaskArmisRunning = false;	//	清除防重入标志
			CountOver = false;				//	清除计数器溢出
			osDelay( 10u );
			TargetIsOK =PWMInCount( abs( Differ ) );
		}	
	}
}

void	_task_ArmCTRL( const void * p_arg )
{
//	uint32_t	arg = (uint32_t)p_arg;
	ArmSoftCtrl( );
}

osThreadDef( _task_ArmCTRL, osPriorityAboveNormal, 1u, 0u );

void	ArmCTRL( void )
{
	osThreadCreate( osThread( _task_ArmCTRL ), 0 );
}




void	Arm_Init( void )
{
	uint16_t CountO;	
	
	Fingers_DIR( false );

	CoordinateSet = 0u;
	CoordinatePosition = 1u;
	SetPluseCount = 0;

	while( CountO != TIM1->CNT )
	{
		CountO = TIM1->CNT;
		osDelay( 50u );
	}
	ArmCTRL();
	CoordinatePosition = 0u;
	PluseCount = 0u;
	SetPluseCount = 0u;
	CoordinateSet = 0u;
	PWMInCount( 0u );
}










/********  (C) COPYRIGHT 2018 青岛中特环保仪器有限公司  **** End Of File ********/
