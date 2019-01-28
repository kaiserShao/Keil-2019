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

/********************************** 功能说明 *************************************
*	
*********************************************************************************/
enum	enumPumpSelect
{
	PumpX,
	PumpY,
	PumpZ
}PumpSelect;

/********************************** 功能说明 *************************************
*	机械臂运行速度设置
*********************************************************************************/
void	Rboot_Arm_PWM( enum enumPumpSelect Select, uint16_t Value )
{
	switch( Select )
	{
		case PumpX:
			PWM_CTRL_X( Value, Value / 2u );
			break;
		case PumpY:
			PWM_CTRL_Y( Value, Value / 2u );
			break;
		case PumpZ:
			break;
		default:
			break;
	}
}
/********************************** 功能说明 *************************************
*	机械臂PWM开关
*********************************************************************************/
void	Rboot_Arm_PWMCmd( enum enumPumpSelect Select, _Bool State )
{
	switch( Select )
	{
		case PumpX:
			PWM_CMD_X( State );
			break;
		case PumpY:
			PWM_CMD_Y( State );
			break;
		case PumpZ:
			break;
		default:
			break;
	}
}
/********************************** 功能说明 *************************************
*	机械臂运行方向设置
*********************************************************************************/
void	Rboot_Arm_Dir( enum enumPumpSelect Select, _Bool State )
{
	switch( Select )
	{
		case PumpX:
			RBOOT_ARM_DIR_X( !State );
			break;
		case PumpY:
			RBOOT_ARM_DIR_Y( State );
			break;
		case PumpZ:
			break;
		default:
			break;
	}
}
/********************************** 功能说明 *************************************
*	机械臂控制器使能
*********************************************************************************/
void	Rboot_Arm_En( enum enumPumpSelect Select, _Bool State )
{
	switch( Select )
	{
		case PumpX:
			RBOOT_ARM_EN_X( State );
			break;
		case PumpY:
			RBOOT_ARM_EN_Y( State );
			break;
		case PumpZ:
			break;
		default:
			break;
	}
}


#define	Full_X  26000u
#define	Full_Y  16000u
/********************************** 功能说明 *************************************
*	机械臂运行任务
*********************************************************************************/
_Bool			TaskArmisRunning[2u] = {0};
uint32_t	CoordinateSet[2u];
uint32_t	CoordinatePosition[2u];
uint32_t	Position[2u];

static	uint32_t	CoordinateSetOld[3u];
uint32_t	period[2u] 		= { 2000u, 2000u };
int32_t		Differ[2u];
float 		Velocity[2u]	= { 6.0f, 6.0f };
uint32_t	Shifting[2u] = { 4000u, 4000u };
void	ArmSoftCtrl( enum	enumPumpSelect PumpSelect )
{
#define	ShiftPluse	1000.0f
#define	LowSpeed		50.0f
#define	VelocityL		20.0f
#define	VelocityH		500.0f
#define	Speeda			6.0f
	for(;;)
	{
		osDelay( 50u );
		if( (CoordinateSet[PumpSelect] != CoordinateSetOld[PumpSelect]) && !TaskArmisRunning[PumpSelect] )
		{
			
			for( uint8_t	ix = 0u; ix < 3u; ix ++ )
			{
				period  [ix] = 2000u;
				Velocity[ix] = 6.0f;
				Shifting[ix] = 400u;
//				RTime[ix] = 0.0f;
			}
			
			
			CoordinateSetOld[PumpSelect] = CoordinateSet[PumpSelect];
			
			TaskArmisRunning[PumpSelect] = true;																	//	置位防重入标志
			
			PluseCount[PumpSelect] = 0u;																					//	清零计数脉冲
			CountOver[PumpSelect] = false;				//	清除计数器溢出

			Differ[PumpSelect] = CoordinateSet[PumpSelect] - CoordinatePosition[PumpSelect];	//	计算设置位置与当前位置的偏差和方向
			SetPluseCount[PumpSelect] = abs( Differ[PumpSelect]);											//	设置泵运行脉冲
			Position[PumpSelect] = CoordinatePosition[PumpSelect];

			Rboot_Arm_Dir		( PumpSelect, ( Differ[PumpSelect] >= 0 ) ? false : true );				//	选择方向
			Rboot_Arm_PWM		( PumpSelect, 1000u );																//	重置pwm为最低输出
			Rboot_Arm_En		( PumpSelect, true );																	//	步进电机控制器控制使能	
			Rboot_Arm_PWMCmd( PumpSelect, true );																	//	开启脉冲输出
			
			//	todo:	加判是否遇到光耦和零位
			if( abs( Differ[PumpSelect] ) <= ShiftPluse * 2u )			//	分类 
				Rboot_Arm_PWM( PumpSelect, 100u );		//	脉冲数过少时直接使用低速模式
			
			while( !CountOver[PumpSelect] )						//	计数器没有溢出
			{
				if( abs( Differ[PumpSelect] ) <=  ShiftPluse * 2u )			//	分类 
				{
					osDelay( 15u );
					if( Differ[PumpSelect] >= 0 )	//	根据方向偏差来计算当前位置
					{
						CoordinatePosition[PumpSelect] = Position[PumpSelect] + PluseCount[PumpSelect];  
					}
					else
					{
						CoordinatePosition[PumpSelect] = Position[PumpSelect] - PluseCount[PumpSelect]; 
					}
				}
				else																		//	脉冲数多时进行加减速控制
				{

					Shifting[PumpSelect] = abs(Differ[PumpSelect]);
					if( Shifting[PumpSelect] > ShiftPluse )
						Shifting[PumpSelect] = ShiftPluse;
					
					if( PluseCount[PumpSelect] < Shifting[PumpSelect] )		//	前Shifting[PumpSelect]脉冲加速
					{
						Velocity[PumpSelect] = sqrt( 2* PluseCount[PumpSelect] * Speeda);
						if(Velocity[PumpSelect] > VelocityH)
							Velocity[PumpSelect] = VelocityH;
						period[PumpSelect] = VelocityH * 10 / Velocity[PumpSelect];
						Rboot_Arm_PWM( PumpSelect, period[PumpSelect] );
					}
					else																	//	中间匀速
					if( PluseCount[PumpSelect] <= (abs(Differ[PumpSelect]) - Shifting[PumpSelect]) )	
					{
						;
					}
					else 																	//	后Shifting[PumpSelect]脉冲减速
					//	if( PluseCount[PumpSelect] >= (abs(Differ) - Shifting[PumpSelect]) )
					{
						
						Velocity[PumpSelect] = sqrt( 2* (abs(Differ[PumpSelect]) - (float)PluseCount[PumpSelect]) * Speeda);
						if(Velocity[PumpSelect] < VelocityL )
							Velocity[PumpSelect] = VelocityL;
						period[PumpSelect] = VelocityH * 10 / Velocity[PumpSelect];
						Rboot_Arm_PWM( PumpSelect, period[PumpSelect] );
					}
					osDelay( 5u );
					if( Differ[PumpSelect] >= 0 )	//	根据方向偏差来计算当前位置
					{
						CoordinatePosition[PumpSelect] = Position[PumpSelect] + PluseCount[PumpSelect];  
					}
					else
					{
						CoordinatePosition[PumpSelect] = Position[PumpSelect] - PluseCount[PumpSelect]; 
					}
				}
				
				if( Differ[PumpSelect] >= 0 )	//	根据最终计数来计算最终位置
				{
					CoordinatePosition[PumpSelect] = Position[PumpSelect] + PluseCount[PumpSelect]; 
				}
				else
				{
					CoordinatePosition[PumpSelect] = Position[PumpSelect] - PluseCount[PumpSelect]; 
				}
			}
			Rboot_Arm_En		( PumpSelect, false );																	//	步进电机控制器控制使能	
			TaskArmisRunning[PumpSelect] = false;	//	清除防重入标志
			CountOver[PumpSelect] = false;				//	清除计数器溢出
			osDelay( 15u );
		}	
	}
}

void	_task_ArmCTRL( const void * p_arg )
{
	uint32_t	arg = (uint32_t)p_arg;
	ArmSoftCtrl((enum enumPumpSelect)arg );
//	osThreadTerminate( osThreadGetId());
}

osThreadDef( _task_ArmCTRL, osPriorityAboveNormal, 2u, 0u );

void	ArmCTRL( void )
{
	osThreadCreate( osThread( _task_ArmCTRL ), (void *)PumpX );
	osThreadCreate( osThread( _task_ArmCTRL ), (void *)PumpY );	
}



void	EXTI3_IRQHandler( void )
{
	EXTIx_IRQ_Disable( 3u );
	PWM_CMD_X( false );
	CoordinatePosition[0u] = 0u;
}
void	EXTI4_IRQHandler( void )
{
	EXTIx_IRQ_Disable( 4u );
	PWM_CMD_Y( false );
	CoordinatePosition[1u] = 0U;

}



uint8_t	Arm_Init( void )
{
		RBOOT_ARM_DIR_X( false );
		RBOOT_ARM_DIR_Y( true );
		EXTIx_IRQ_Disable( 3u );
		EXTIx_IRQ_Disable( 4u );


		PWM_CTRL_X( 140u, 70u );
		PWM_CTRL_Y( 140u, 70u );
		
		RBOOT_ARM_EN_X( true );
		RBOOT_ARM_EN_Y( true );
		
		CoordinateSet[0u] = 0u;
		CoordinateSet[1u] = 0u;
		CoordinatePosition[0u] = 1u;
		CoordinatePosition[1u] = 1u;
		SetPluseCount[0u] = Full_X;
		SetPluseCount[1u] = Full_Y;
	
		if( !READ_BIT( GPIOB->IDR, GPIO_Pin_4 ) )
		{
				PWM_CMD_Y( false );
				CoordinatePosition[1u] = 0u;
		}
		else
		{
			PWM_CMD_Y( true );
			EXTIx_IRQ_Enable( 4u );	
			osDelay(3000u);
		}
		
		if( !READ_BIT( GPIOB->IDR, GPIO_Pin_3 ) )
		{
				PWM_CMD_X( false );
				CoordinatePosition[0u] = 0u;
		}
		else
		{
			PWM_CMD_X( true );		
			EXTIx_IRQ_Enable( 3u );
		}
		while( CoordinatePosition[0u] || CoordinatePosition[1u] )
		{
			PluseCount[0u] = 0u;
			PluseCount[1u] = 0u;
			osDelay( 10u );
		}
		
		PluseCount[0u] = 0u;
		PluseCount[1u] = 0u;
		SetPluseCount[0u] = 0u;
		SetPluseCount[1u] = 0u;
		CoordinateSet[0u] = 0u;
		CoordinateSet[1u] = 0u;
		RBOOT_ARM_EN_X( false );
		RBOOT_ARM_EN_Y( false );
		
		ArmCTRL();
		return 0u;
}










/********  (C) COPYRIGHT 2018 青岛中特环保仪器有限公司  **** End Of File ********/
