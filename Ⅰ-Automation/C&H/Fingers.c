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
*	��е��ָ�����ٶ�����
*********************************************************************************/
void	Fingers_PWM( uint16_t Value )
{
	Fingers_CTRL( Value, Value / 2u );
}
/********************************** ����˵�� *************************************
*	��е��ָPWM����
*********************************************************************************/
void	Fingers_PWMCmd( _Bool State )
{
	Fingers_CMD( State );
}
/********************************** ����˵�� *************************************
*	��е��ָ���з�������
*********************************************************************************/
void	Fingers_Dir( _Bool State )
{
	Fingers_DIR( State );
}
/********************************** ����˵�� *************************************
*	��е��ָ������ʹ��
*********************************************************************************/
void	Fingers_En( _Bool State )
{
	;
}


#define	Full_X  26000u
#define	Full_Y  16000u
#define	Full_Z  8000u
/********************************** ����˵�� *************************************
*	��е��ָ��������
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
			
			TaskArmisRunning = true;																	//	��λ�������־
			
			PluseCount = 0u;																					//	�����������

			Differ = CoordinateSet - CoordinatePosition;	//	��������λ���뵱ǰλ�õ�ƫ��ͷ���
			SetPluseCount = abs( Differ );											//	���ñ���������
			Position = CoordinatePosition;
			PWMInCount( 0u );																					//	����TIMCNT
			CountOver = false;				//	������������
			Fingers_Dir		(  ( Differ >= 0 ) ? true : false );				//	ѡ����
			Fingers_PWM		( 10000u );																	//	����pwmΪ������
//			Fingers_En		( true );																	//	�����������������ʹ��	
			Fingers_PWMCmd( true );																		//	�����������
			
			//	todo:	�����Ƿ������������λ
			
			while( !CountOver )						//	������û�����
			{
				if( abs( Differ ) <= ShiftPluse * 2 )			//	���� 
				{
					Fingers_PWM( 1000u );		//	����������ʱֱ��ʹ�õ���ģʽ 100p/s
					osDelay( 5u );
				}
				else																		//	��������ʱ���мӼ��ٿ���
				{

					Shifting = abs(Differ);
					if( Shifting > ShiftPluse )
						Shifting = ShiftPluse;
				
					if( PluseCount < Shifting )		//	ǰShifting�������
					{
						Velocity = sqrt( 2* PluseCount * Speeda);
						if(Velocity > VelocityH)
							Velocity = VelocityH;
						period = VelocityH * 10 / Velocity;
						if(Velocity > 90)
							Velocity = 90;
						Fingers_PWM(  period );
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
						Fingers_PWM(  period );
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










/********  (C) COPYRIGHT 2018 �ൺ���ػ����������޹�˾  **** End Of File ********/
