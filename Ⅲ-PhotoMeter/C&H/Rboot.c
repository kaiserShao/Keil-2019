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
//#include "APPDEF.H"
#include "string.h"
#include "math.h"
#include "stdlib.h"

//Rboot_CTRL PWM_CTRL
//Rboot_CMD PWM_CMD
//Rboot_DIR RBOOT_ARM_DIR
//PWMInCount  RBOOT_ARM_EN
/********************************** 功能说明 *************************************
*	机械臂运行速度设置
*********************************************************************************/
void	Rboot_PWM( uint16_t Value )
{
	PWM_CTRL( Value, Value / 2u );
}
/********************************** 功能说明 *************************************
*	机械臂PWM开关
*********************************************************************************/
void	Rboot_PWMCmd( _Bool State )
{
	PWM_CMD( State );
}
/********************************** 功能说明 *************************************
*	机械臂运行方向设置
*********************************************************************************/
void	Rboot_Dir( _Bool State )
{
	RBOOT_ARM_DIR( State );
}
/********************************** 功能说明 *************************************
*	机械臂控制器使能
*********************************************************************************/
void	Rboot_En( _Bool State )
{
	RBOOT_ARM_EN( State );
}


#define	Full  10000u

/********************************** 功能说明 *************************************
*	机械臂运行任务
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

void	ArmSoftCtrl( void )
{
#define	ShiftPluse	1000.0f
#define	LowSpeed		50.0f
#define	VelocityL		20.0f
#define	VelocityH		500.0f
#define	Speeda			20.0f

	for(;;)
	{
		osDelay(50u);
		if( (CoordinateSet != CoordinateSetOld) && !TaskArmisRunning )
		{

			period   = 2000u;
			Velocity = 20.0f;
			Shifting = 400u;
			
			CoordinateSetOld = CoordinateSet;
			
			TaskArmisRunning = true;																	//	置位防重入标志
			
			PluseCount = 0u;																					//	清零计数脉冲

			Differ = CoordinateSet - CoordinatePosition;	//	计算设置位置与当前位置的偏差和方向
			SetPluseCount = abs( Differ );											//	设置泵运行脉冲
			Position = CoordinatePosition;
			Rboot_Dir		(  ( Differ >= 0 ) ? false : true );				//	选择方向
			Rboot_PWM		( 15000u );																	//	重置pwm为最低输出
			Rboot_En		( true );																	//	步进电机控制器控制使能	
			Rboot_PWMCmd( true );																		//	开启脉冲输出
			
			//	todo:	加判是否遇到光耦和零位
			if( abs( Differ ) <= ShiftPluse * 2u )			//	分类 
				Rboot_PWM( 50u );		//	脉冲数过少时直接使用低速模式
			
			while( !CountOver )						//	计数器没有溢出
			{
				if( abs( Differ ) <=  ShiftPluse * 2u )			//	分类 
				{
					osDelay( 15u );
					if( Differ >= 0 )	//	根据方向偏差来计算当前位置
					{
						CoordinatePosition = Position + PluseCount;  
					}
					else
					{
						CoordinatePosition = Position - PluseCount; 
					}
				}
				else																		//	脉冲数多时进行加减速控制
				{
					Shifting = abs( Differ );
					if( Shifting > ShiftPluse )
						Shifting = ShiftPluse;
					
					if( PluseCount < Shifting )		//	前Shifting脉冲加速
					{
						Velocity = sqrt( 2* PluseCount * Speeda);
						if(Velocity > VelocityH)
							Velocity = VelocityH;
						period = VelocityH * 10 / Velocity;
						Rboot_PWM(  period );
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
						Rboot_PWM(  period );
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
//			Rboot_En		( false );																	//	步进电机控制器控制使能	
		}	
	}
}


void	EXTI3_IRQHandler( void )
{
	EXTIx_IRQ_Disable( 3u );
	Rboot_PWMCmd( false );
	CoordinatePosition = 0u;
}


void	_task_ArmCTRL( const void * p_arg )
{
	ArmSoftCtrl();
}

osThreadDef( _task_ArmCTRL, osPriorityAboveNormal, 1u, 0u );

void	ArmCTRL( void )
{
	osThreadCreate( osThread( _task_ArmCTRL ), 0 );
}




uint8_t	Arm_Init( void )
{	

	Rboot_Dir( true );

	Rboot_PWMCmd( true );

	EXTIx_IRQ_Disable( 3u );

	Rboot_PWM( 50u );

	Rboot_En( true );

	CoordinateSet = 0u;
	CoordinatePosition = 1u;
	SetPluseCount = Full;


	if( !READ_BIT( GPIOB->IDR, GPIO_Pin_3 ) )
	{
			Rboot_PWMCmd( false );
			CoordinatePosition = 0u;
	}
	else
	{
		Rboot_PWMCmd( true );		
		EXTIx_IRQ_Enable( 3u );
	}

	while( CoordinatePosition )
	{
		PluseCount = 0u;
		osDelay( 10u );
	}
		
	PluseCount = 0u;
	SetPluseCount = 0u;
	CoordinateSet = 0u;
	ArmCTRL();
	return 0u;
}



/********  (C) COPYRIGHT 2018 青岛中特环保仪器有限公司  **** End Of File ********/
