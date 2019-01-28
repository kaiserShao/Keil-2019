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
#include "stdlib.h"

//Rboot_CTRL PWM_CTRL
//Rboot_CMD PWM_CMD
//Rboot_DIR RBOOT_ARM_DIR
//PWMInCount  RBOOT_ARM_EN
/********************************** ����˵�� *************************************
*	��е�������ٶ�����
*********************************************************************************/
void	Rboot_PWM( uint16_t Value )
{
	PWM_CTRL( Value, Value / 2u );
}
/********************************** ����˵�� *************************************
*	��е��PWM����
*********************************************************************************/
void	Rboot_PWMCmd( _Bool State )
{
	PWM_CMD( State );
}
/********************************** ����˵�� *************************************
*	��е�����з�������
*********************************************************************************/
void	Rboot_Dir( _Bool State )
{
	RBOOT_ARM_DIR( State );
}
/********************************** ����˵�� *************************************
*	��е�ۿ�����ʹ��
*********************************************************************************/
void	Rboot_En( _Bool State )
{
	RBOOT_ARM_EN( State );
}


#define	Full  10000u

/********************************** ����˵�� *************************************
*	��е����������
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
			
			TaskArmisRunning = true;																	//	��λ�������־
			
			PluseCount = 0u;																					//	�����������

			Differ = CoordinateSet - CoordinatePosition;	//	��������λ���뵱ǰλ�õ�ƫ��ͷ���
			SetPluseCount = abs( Differ );											//	���ñ���������
			Position = CoordinatePosition;
			Rboot_Dir		(  ( Differ >= 0 ) ? false : true );				//	ѡ����
			Rboot_PWM		( 15000u );																	//	����pwmΪ������
			Rboot_En		( true );																	//	�����������������ʹ��	
			Rboot_PWMCmd( true );																		//	�����������
			
			//	todo:	�����Ƿ������������λ
			if( abs( Differ ) <= ShiftPluse * 2u )			//	���� 
				Rboot_PWM( 50u );		//	����������ʱֱ��ʹ�õ���ģʽ
			
			while( !CountOver )						//	������û�����
			{
				if( abs( Differ ) <=  ShiftPluse * 2u )			//	���� 
				{
					osDelay( 15u );
					if( Differ >= 0 )	//	���ݷ���ƫ�������㵱ǰλ��
					{
						CoordinatePosition = Position + PluseCount;  
					}
					else
					{
						CoordinatePosition = Position - PluseCount; 
					}
				}
				else																		//	��������ʱ���мӼ��ٿ���
				{
					Shifting = abs( Differ );
					if( Shifting > ShiftPluse )
						Shifting = ShiftPluse;
					
					if( PluseCount < Shifting )		//	ǰShifting�������
					{
						Velocity = sqrt( 2* PluseCount * Speeda);
						if(Velocity > VelocityH)
							Velocity = VelocityH;
						period = VelocityH * 10 / Velocity;
						Rboot_PWM(  period );
					}
					else																	//	�м�����
					if( PluseCount <= (abs(Differ) - Shifting) )	
					{
						;
					}
					else 																	//	��Shifting�������
					//	if( PluseCount >= (abs((float)Differ) - 400u) )
					{
						
						Velocity = sqrt( 2* (abs(Differ) - (float)PluseCount) * Speeda);
						if(Velocity < VelocityL )
							Velocity = VelocityL;
						period = VelocityH * 10 / Velocity;
						Rboot_PWM(  period );
					}
					osDelay( 5u );
					if( Differ >= 0 )	//	���ݷ���ƫ�������㵱ǰλ��
					{
						CoordinatePosition = Position + PluseCount;  
					}
					else
					{
						CoordinatePosition = Position - PluseCount; 
					}
				}
				
				if( Differ > 0 )	//	�������ռ�������������λ��
				{
					CoordinatePosition = Position + PluseCount; 
				}
				else
				{
					CoordinatePosition = Position - PluseCount; 
				}
			}
			
			TaskArmisRunning = false;	//	����������־
			CountOver = false;				//	������������
			osDelay( 10u );
//			Rboot_En		( false );																	//	�����������������ʹ��	
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



/********  (C) COPYRIGHT 2018 �ൺ���ػ����������޹�˾  **** End Of File ********/
