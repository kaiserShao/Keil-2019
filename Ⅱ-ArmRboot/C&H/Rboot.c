/**************** (C) COPYRIGHT 2018 �ൺ���ػ����������޹�˾ ********************
* �� �� ��:  
* �� �� ��:  Kaiser
* ��  ��  :  
* ����޸�:  
*********************************** �޶���¼ *************************************
* ��  ��: 
* �޶���: 
*********************************************************************************/
#include "BIOS.H"
#include "stdlib.h"

/********************************** ����˵�� *************************************
*	
*********************************************************************************/
enum	enumPumpSelect
{
	PumpX,
	PumpY,
	PumpZ,
	
	pumpMax,
}PumpSelect;

/********************************** ����˵�� *************************************
*	��е�������ٶ�����
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
/********************************** ����˵�� *************************************
*	��е��PWM����
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
/********************************** ����˵�� *************************************
*	��е�����з�������
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
/********************************** ����˵�� *************************************
*	��е�ۿ�����ʹ��
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
/********************************** ����˵�� *************************************
*	��е����������
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
	/*Vmin = 50p/s Vmax = 20000p/s  At = 0.2s  a = 100000p/s2  ��s = 2000p ��t = 0.001s P = 0.000001s */
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
		/* ����������*/
		Complete = false;
		T = 1 / VelocityL;
		V = VelocityL;
		Arr = T * TIMER_F;
		ArrBuf[Count++] = Arr;
		/*�����ж���*/
		for(;;)
		{	
			if(CountSum < ShiftPluse)		//	ǰ�˼���
			{
				T = 1 / V;								//	�߹�����һ�����徭����ʱ��
				V += AccelerationUP * T;	//	��һ��������ٶ�

				Arr = T * TIMER_F;				//	��һ��������ٶ�ת����PWMֵ�Ļ����ʽ ӦΪ1/V = Arr/F 
				ArrBuf[Count] = Arr;			//	����DMA������

			}
			else																	//	�м�����
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

			
			if( Count++ == 512u )				//	������1KRAM
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
			
			TaskArmisRunning[PumpSelect] = true;																	//	��λ�������־
			
			PluseCount[PumpSelect] = 0u;																					//	�����������
			CountOver[PumpSelect] = false;				//	������������

			Differ[PumpSelect] = CoordinateSet[PumpSelect] - CoordinatePosition[PumpSelect];	//	��������λ���뵱ǰλ�õ�ƫ��ͷ���
			SetPluseCount[PumpSelect] = abs( Differ[PumpSelect]);											//	���ñ���������
			Position[PumpSelect] = CoordinatePosition[PumpSelect];

			Rboot_Arm_Dir		( PumpSelect, ( Differ[PumpSelect] >= 0 ) ? false : true );				//	ѡ����
			Rboot_Arm_PWM		( PumpSelect, 1000u );																//	����pwmΪ������
			Rboot_Arm_En		( PumpSelect, true );																	//	�����������������ʹ��	
			Rboot_Arm_PWMCmd( PumpSelect, true );																	//	�����������
			
			//	todo:	�����Ƿ������������λ
			if( abs( Differ[PumpSelect] ) <= ShiftPluse * 2u )			//	���� 
				Rboot_Arm_PWM( PumpSelect, 140u );		//	����������ʱֱ��ʹ�õ���ģʽ
			
			while( !CountOver[PumpSelect] )						//	������û�����
			{
				if( abs( Differ[PumpSelect] ) <=  ShiftPluse * 2u )			//	���� 
				{
					osDelay( 15u );
					if( Differ[PumpSelect] >= 0 )	//	���ݷ���ƫ�������㵱ǰλ��
					{
						CoordinatePosition[PumpSelect] = Position[PumpSelect] + PluseCount[PumpSelect];  
					}
					else
					{
						CoordinatePosition[PumpSelect] = Position[PumpSelect] - PluseCount[PumpSelect]; 
					}
				}
				else																		//	��������ʱ���мӼ��ٿ���
				{

					Shifting[PumpSelect] = abs(Differ[PumpSelect]);
					if( Shifting[PumpSelect] > ShiftPluse )
						Shifting[PumpSelect] = ShiftPluse;
					
					if( PluseCount[PumpSelect] < Shifting[PumpSelect] )		//	ǰShifting[PumpSelect]�������
					{
						Velocity[PumpSelect] = sqrt( 2* PluseCount[PumpSelect] * Speeda);
						if(Velocity[PumpSelect] > VelocityH)
							Velocity[PumpSelect] = VelocityH;
						period[PumpSelect] = VelocityH * 10 / Velocity[PumpSelect];
						Rboot_Arm_PWM( PumpSelect, period[PumpSelect] );
					}
					else																	//	�м�����
					if( PluseCount[PumpSelect] <= (abs(Differ[PumpSelect]) - Shifting[PumpSelect]) )	
					{
						;
					}
					else 																	//	��Shifting[PumpSelect]�������
					//	if( PluseCount[PumpSelect] >= (abs(Differ) - Shifting[PumpSelect]) )
					{
						
						Velocity[PumpSelect] = sqrt( 2* (abs(Differ[PumpSelect]) - (float)PluseCount[PumpSelect]) * Speeda);
						if(Velocity[PumpSelect] < VelocityL )
							Velocity[PumpSelect] = VelocityL;
						period[PumpSelect] = VelocityH * 10 / Velocity[PumpSelect];
						Rboot_Arm_PWM( PumpSelect, period[PumpSelect] );
					}
					osDelay( 5u );
					if( Differ[PumpSelect] >= 0 )	//	���ݷ���ƫ�������㵱ǰλ��
					{
						CoordinatePosition[PumpSelect] = Position[PumpSelect] + PluseCount[PumpSelect];  
					}
					else
					{
						CoordinatePosition[PumpSelect] = Position[PumpSelect] - PluseCount[PumpSelect]; 
					}
				}
				
				if( Differ[PumpSelect] >= 0 )	//	�������ռ�������������λ��
				{
					CoordinatePosition[PumpSelect] = Position[PumpSelect] + PluseCount[PumpSelect]; 
				}
				else
				{
					CoordinatePosition[PumpSelect] = Position[PumpSelect] - PluseCount[PumpSelect]; 
				}
			}
			TaskArmisRunning[PumpSelect] = false;	//	����������־
			CountOver[PumpSelect] = false;				//	������������
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










/********  (C) COPYRIGHT 2018 �ൺ���ػ����������޹�˾  **** End Of File ********/
