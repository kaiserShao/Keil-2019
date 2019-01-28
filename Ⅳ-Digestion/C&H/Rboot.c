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
//#include "APPDEF.H"
#include "string.h"
#include "math.h"

/********************************** ����˵�� *************************************
*	
*********************************************************************************/
enum	enumPumpSelect
{
	PumpX,
	PumpY,
	PumpZ
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
			break;
		default:
			break;
	}
}


#define	Full_X  26000u
#define	Full_Y  16000u
/********************************** ����˵�� *************************************
*	��е����������
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
				Rboot_Arm_PWM( PumpSelect, 100u );		//	����������ʱֱ��ʹ�õ���ģʽ
			
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
			Rboot_Arm_En		( PumpSelect, false );																	//	�����������������ʹ��	
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










/********  (C) COPYRIGHT 2018 �ൺ���ػ����������޹�˾  **** End Of File ********/
