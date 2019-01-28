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
*	
*********************************************************************************/
enum	enumPumpSelect
{
	PumpX,
	PumpY,
	PumpZ,
	
	pumpMax,
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
			PWM_CTRL_Z( Value, Value / 2u );
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
			PWM_CMD_Z( State );
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
			RBOOT_ARM_DIR_X( State );
			break;
		case PumpY:
			RBOOT_ARM_DIR_Y( State );
			break;
		case PumpZ:
			RBOOT_ARM_DIR_Z( State );
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
			RBOOT_ARM_EN_Z( State );
			break;
		default:
			break;
	}
}


#define	Full_X  26000u
#define	Full_Y  16000u
#define	Full_Z  8000u
/********************************** 功能说明 *************************************
*	机械臂运行任务
*********************************************************************************/
uint32_t	CoordinateSet[pumpMax];
uint32_t	CoordinatePosition[pumpMax];
uint32_t	Position[pumpMax];
_Bool			TaskArmisRunning[pumpMax] = {0};

uint32_t	CoordinateSetOld[pumpMax];
uint32_t	period[pumpMax] 		= { 2000u, 2000u, 2000u };
int32_t		Differ[pumpMax];
float 		Velocity[pumpMax]	= { 20.0f, 20.0f, 20.0f };
uint32_t	Shifting[pumpMax] = { 2000u, 2000u, 2000u };
//static	float			RTime[pumpMax] = { 0.0f,0.0f,0.0f};
#include "math.h"
void	ArmSoftCtrl( enum	enumPumpSelect PumpSelect )
{
	/*Vmin = 50p/s Vmax = 20000p/s  At = 0.2s  a = 100000p/s2  △s = 2000p △t = 0.001s P = 0.000001s */
#define	ShiftPluse	2000.0f
#define	LowSpeed		40.0f
#define	VelocityL		100.0f
#define	VelocityH		20000.0f
#define	AccelerationUP			100000.0f
#define	AccelerationDown		-100000.0f
	uint16_t ArrBuf[512u];
	for(;;)
	{
		static	double V,T,Vo,To;
		static	uint16_t Arr,Arro;
		static	uint16_t Count = 0u;
		static	uint16_t CountSum	= 0u;
		static	_Bool Complete = false;
		/* 放在任务中*/
		Complete = false;
		T = 1 / VelocityL;
		V = VelocityL;
		Arr = T * TIMER_F;
		ArrBuf[Count++] = Arr;
		/*放在中断中*/
		for(;;)
		{	
			if(CountSum < ShiftPluse)		//	前端加速
			{
				T = 1 / V;								//	走过最新一个脉冲经历的时间
				V += AccelerationUP * T;	//	下一个脉冲的速度

				Arr = T * TIMER_F;				//	下一个脉冲的速度转换成PWM值的化简后公式 应为1/V = Arr/F 
				ArrBuf[Count] = Arr;			//	放入DMA缓冲区

			}
			else																	//	中间匀速
			if( CountSum <= (abs(Differ[PumpSelect]) - Shifting[PumpSelect]) )
			{
				ArrBuf[Count] = Arr;
			}	
			else			
			{
				Arr = To * TIMER_F;
				V = AccelerationDown * To;
				T = 1 / V;
				To += T; 
				ArrBuf[Count] = Arr;
				if( Count == 1000u )
				{
					Count = 0u;
					break;
				}	
			}

			
			if( Count++ == 512u )				//	缓冲满1KRAM
			{
				Count = 0u;
				break;
			}
			if(CountSum++ == abs( Differ[PumpSelect] ))
			{
				for( Count = Count; Count < 512; Count ++)
				{
					ArrBuf[Count] = 0u;
				}
				Complete = true;
				break;
			}

		}
	
			
		
		
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
				Rboot_Arm_PWM( PumpSelect, 140u );		//	脉冲数过少时直接使用低速模式
			
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

osThreadDef( _task_ArmCTRL, osPriorityAboveNormal, 3u, 0u );

void	ArmCTRL( void )
{
	osThreadCreate( osThread( _task_ArmCTRL ), (void *)PumpX );
	osThreadCreate( osThread( _task_ArmCTRL ), (void *)PumpY );	
	osThreadCreate( osThread( _task_ArmCTRL ), (void *)PumpZ );	
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
void	EXTI9_5_IRQHandler( void )
{
	EXTIx_IRQ_Disable( 5u );
	PWM_CMD_Z( false );
	CoordinatePosition[2u] = 0u;
}


void	X_FindZero(void)
{
	RBOOT_ARM_DIR_X( true );

	EXTIx_IRQ_Disable( 3u );

	PWM_CTRL_X( 100u, 50u );

	RBOOT_ARM_EN_X( true );

	CoordinatePosition[PumpX] = 1u;
	SetPluseCount[PumpX] = Full_X;

	if( !READ_BIT( GPIOB->IDR, GPIO_Pin_3 ) )
	{
		RBOOT_ARM_DIR_X( false );
		PWM_CMD_X( true );
		osDelay(500u);
		RBOOT_ARM_DIR_X( true );
		EXTIx_IRQ_Enable( 3u );
	}
	else
	{
		EXTIx_IRQ_Enable( 3u );
		PWM_CMD_X( true );
	}

	while( CoordinatePosition[PumpX] )
	{
		PluseCount[PumpX] = 0u;
		osDelay( 20u );
	}
	CoordinateSet[PumpX] = CoordinateSetOld[PumpX] = 0u;
	PluseCount[PumpX] = 0u;
	SetPluseCount[PumpX] = 0u;
}

void	Y_FindZero(void)
{
	RBOOT_ARM_DIR_Y( true );

	EXTIx_IRQ_Disable( 4u );

	PWM_CTRL_Y( 100u, 50u );

	RBOOT_ARM_EN_Y( true );

	CoordinatePosition[PumpY] = 1u;
	SetPluseCount[PumpY] = Full_Y;

	if( !READ_BIT( GPIOB->IDR, GPIO_Pin_4 ) )
	{
		RBOOT_ARM_DIR_Y( false );
		PWM_CMD_Y( true );
		osDelay(500u);
		RBOOT_ARM_DIR_Y( true );
		EXTIx_IRQ_Enable( 4u );
	}
	else
	{
		EXTIx_IRQ_Enable( 4u );
		PWM_CMD_Y( true );
	}

	while( CoordinatePosition[PumpY] )
	{
		PluseCount[PumpY] = 0u;
		osDelay( 20u );
	}
	CoordinateSet[PumpY] = CoordinateSetOld[PumpY] = 0u;
	PluseCount[PumpY] = 0u;
	SetPluseCount[PumpY] = 0u;
}

void	Z_FindZero(void)
{
	RBOOT_ARM_DIR_Z( true );

	EXTIx_IRQ_Disable( 5u );

	PWM_CTRL_Z( 100u, 50u );

	RBOOT_ARM_EN_Z( true );

	CoordinatePosition[PumpZ] = 1u;
	SetPluseCount[PumpZ] = Full_Z;

	if( !READ_BIT( GPIOB->IDR, GPIO_Pin_5 ) )
	{
		RBOOT_ARM_DIR_Z( false );
		PWM_CMD_Z( true );
		osDelay(500u);
		RBOOT_ARM_DIR_Z( true );
		EXTIx_IRQ_Enable( 5u );
	}
	else
	{
		EXTIx_IRQ_Enable( 5u );
		PWM_CMD_Z( true );
	}

	while( CoordinatePosition[PumpZ] )
	{
		PluseCount[PumpZ] = 0u;
		osDelay( 20u );
	}
	CoordinateSet[PumpZ] = CoordinateSetOld[PumpZ] = 0u;
	PluseCount[PumpZ] = 0u;
	SetPluseCount[PumpZ] = 0u;
}

void	Arm_Init( void )
{
		RBOOT_ARM_DIR_X( true );
		RBOOT_ARM_DIR_Y( true );
		RBOOT_ARM_DIR_Z( true );

		EXTIx_IRQ_Disable( 3u );
		EXTIx_IRQ_Disable( 4u );
		EXTIx_IRQ_Disable( 5u );

		PWM_CTRL_X( 100u, 50u );
		PWM_CTRL_Y( 100u, 50u );
		PWM_CTRL_Z( 100u, 50u );
		
		RBOOT_ARM_EN_X( true );
		RBOOT_ARM_EN_Y( true );
		RBOOT_ARM_EN_Z( true );
		
		CoordinateSet[0u] = 0u;
		CoordinateSet[1u] = 0u;
		CoordinateSet[2u] = 0u;
		CoordinatePosition[0u] = 1u;
		CoordinatePosition[1u] = 1u;
		CoordinatePosition[2u] = 1u;
		SetPluseCount[0u] = Full_X;
		SetPluseCount[1u] = Full_Y;
		SetPluseCount[2u] = Full_Z;
	
		if( !READ_BIT( GPIOB->IDR, GPIO_Pin_5 ) )
		{
				PWM_CMD_Z( false );
				CoordinatePosition[2u] = 0u;
		}
		else
		{
			PWM_CMD_Z( true );
			EXTIx_IRQ_Enable( 5u );
			while( CoordinatePosition[2u] )
			{
				PluseCount[2u] = 0u;
				osDelay( 20u );
			}
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
		if( !READ_BIT( GPIOB->IDR, GPIO_Pin_4 ) )
		{
				PWM_CMD_Y( false );
				CoordinatePosition[1u] = 0u;
		}
		else
		{
			PWM_CMD_Y( true );
			EXTIx_IRQ_Enable( 4u );	
		}
		while( CoordinatePosition[0u] || CoordinatePosition[1u] )
		{
			PluseCount[0u] = 0u;
			PluseCount[1u] = 0u;
			osDelay( 20u );
		}
		
		PluseCount[0u] = 0u;
		PluseCount[1u] = 0u;
		PluseCount[2u] = 0u;
		SetPluseCount[0u] = 0u;
		SetPluseCount[1u] = 0u;
		SetPluseCount[2u] = 0u;
		CoordinateSet[0u] = 0u;
		CoordinateSet[1u] = 0u;
		CoordinateSet[2u] = 0u;
		
		ArmCTRL();

}










/********  (C) COPYRIGHT 2018 青岛中特环保仪器有限公司  **** End Of File ********/
