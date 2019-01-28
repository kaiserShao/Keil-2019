/**************** (C) COPYRIGHT 2018 �ൺ���ػ����������޹�˾ *******************
* �� �� ��:  
* �� �� ��:  Kaiser
* ��  ��  :  
* ����޸�:  
*********************************** �޶���¼ ************************************
* ��  ��: 
* �޶���: 
********************************************************************************/
#include "BIOS.H"
#define	ServoPeriod  20000u
/******************************** ����˵�� *************************************
*	��ת�������
*******************************************************************************/
uint16_t	Angel;
uint16_t	AngelRT;
void	Rotate_Servo_Angel( void )
{
	static	uint16_t	LastAngel;
	static	uint16_t	Time = 100u;
	static	uint16_t	CCRValueSet = 0u;
	static	uint16_t	CCRValueCount = 0u;
	static	uint16_t	CCRValue = 0u;
	static	int16_t	CCRAdd = 0u;
	static	_Bool			TaskArmisRunning = 0;
	static	uint16_t	AngelSet = 0u;
	for( ;; )
	{
		osDelay(5u);
		if( !TaskArmisRunning )
		{
			AngelSet = Angel;
			CCRValueSet = (2000.0f / 180.0f) * (AngelSet / 10.0f) + 500u;
		}
			if( LastAngel != AngelSet )
			{
				AngelRT = ( CCRValue + CCRAdd - 500u ) / (2000.0f / 180.0f) * 10.0f;
				TaskArmisRunning = true;
				if( CCRValue + CCRAdd != CCRValueSet)
				{
						CCRValue = (2000.0f / 180.0f) * (LastAngel / 10.0f) + 500u;
					
					Rotate_Servo_CTRL( ServoPeriod, CCRValue + CCRAdd );
					Rotate_Servo_CMD( true );
					if( CCRValue > CCRValueSet )
						CCRAdd = - CCRValueCount;
					else
						CCRAdd = CCRValueCount;
					Time = 100u;		
					if( CCRValue + CCRAdd != CCRValueSet)
						CCRValueCount ++;
					
					if( CCRValue > CCRValueSet )
						CCRAdd = - CCRValueCount;
					else
						CCRAdd = CCRValueCount;
					if( CCRValue + CCRAdd != CCRValueSet)
						CCRValueCount ++;

					if( CCRValue > CCRValueSet )
						CCRAdd = - CCRValueCount;
					else
						CCRAdd = CCRValueCount;
					if( CCRValue + CCRAdd != CCRValueSet)
						CCRValueCount ++;
				}
				
			}
			if( Time )
			{
				Time --;
				if( !Time )
				{
					Rotate_Servo_CMD( false );
					LastAngel = AngelSet;
					CCRValueCount = 0;
					AngelRT = AngelSet;
					TaskArmisRunning =false;
				}
			}

	}
}

void	_task_Rotate_ServoCTRL( const void * p_arg )
{
//	uint32_t	arg = (uint32_t)p_arg;
	Rotate_Servo_Angel();
//	osThreadTerminate( osThreadGetId());
}

osThreadDef( _task_Rotate_ServoCTRL, osPriorityAboveNormal, 1u, 0u );

void	Rotate_Servo_Init( void )
{

	Rotate_Servo_CTRL( ServoPeriod, (2000.0f / 180.0f) * Angel + 500u );
	Rotate_Servo_CMD( true );
	osDelay( 2500u );
	osThreadCreate( osThread( _task_Rotate_ServoCTRL ), NULL );
}

/********  (C) COPYRIGHT 2018 �ൺ���ػ����������޹�˾  **** End Of File ********/
