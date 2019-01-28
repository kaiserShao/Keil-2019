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
/******************************** ����˵�� *************************************
*	
*******************************************************************************/
typedef struct{
	float Limit;		//����޷�
	float Target;		//Ŀ�������
	float Feedback;		//ʵ�������	
	float Kp;	
	float Ki;	
	float Kd;	
	float E_0;			//��ǰ���
	float E_1;			//��һ�����
	float E_2;			//���ϴ����
}PID;
 
PID TempPID = 
{
			1,								//����޷�
			165,							//Ŀ�������
			25,								//����������
 
			0.1,				//Kp
			0.1,		//Ki
			0.1,				//Kd
	
	165,				//e
	165,
	165,		
};

uint16_t	HeatTemp_Set;

static	enum	enumOP_State
{
	
	eShut, eRunning, eShutReq
} volatile  OP_State;


#define max(a, b)			(a>b? a:b)
#define min(a, b)			(a<b? a:b)
#define range(x, a, b)		(min(max(x, a), b))
#define exchange(a, b, tmp) (tmp=a, a=b, b=tmp)
#define myabs(x)			((x<0)? -x:x)
 

/******************************** ����˵�� *************************************
*	
*******************************************************************************/
void	PID_Calc_Init(PID *pid)
{
	pid->E_0 = pid->Target - pid->Feedback;
	pid->E_2 = pid->E_1;
	pid->E_1 = pid->E_0;
}

/******************************** ����˵�� *************************************
*	
*******************************************************************************/
float pid_calc(PID *pid)
{
	float out;
	float Ep, Ei, Ed;
	
	pid->E_0 = pid->Target - pid->Feedback;
	
	Ep = pid->E_0  - pid->E_1;
	Ei = pid->E_0;
	Ed = (pid->E_0 - pid->E_1) - (pid->E_1 - pid->E_2);
	
	out = pid->Kp * Ep 
			+ pid->Ki * Ei 
			+ pid->Kd * Ed;
	
	out = range(out, - pid->Limit, pid->Limit);
	
	pid->E_2 = pid->E_1;
	pid->E_1 = pid->E_0;
	
	return out;
}

/******************************** ����˵�� *************************************
*	
*******************************************************************************/
static	float	control_val;
static	float	inc_val;
float	PT100Temp[3u];
static	void	PID_Task( void const * p_arg )
{
	uint32_t  SaveTick;
	PID_Calc_Init(&TempPID);
	Heat_CTRL( 1000u, 0u );
	Heat_CMD( true );
	SaveTick = osKernelSysTick();
	while ( eRunning == OP_State )
	{
		TempPID.Feedback = PT100Temp[0u];
		inc_val = pid_calc(&TempPID);  
		//	�������ʱ
		{
			const	uint32_t	oneTick = osKernelSysTickMicroSec( 1000u );
			uint32_t	now = osKernelSysTick();
			uint32_t	lostTick, delayCount;
			//	�����Ѿ���ȥ��tick��ʱ��λ
			lostTick = (uint32_t)( now + oneTick - 1u - SaveTick ) / oneTick;

			//	����������Ҫ����ʱ����(�趨���ʱ�䣺300ms)
			if ( lostTick < 900u )
			{
				delayCount = 900u - lostTick;
			}
			else
			{
				//	�Ѿ���ȥ��ʱ�䳬���趨�ļ�������¼���ʱ������������ʱ��
				SaveTick = now;
				delayCount = 30u;
			}

			osDelay( delayCount );
			SaveTick += delayCount * oneTick;
		}
		control_val += inc_val * 1000.0f;  
		control_val = range( control_val, 0, 10000 );
		if( control_val > 9800u )
			Heat_CTRL( 10000u, 10000 );	
		else
		if( control_val < 200u )
			Heat_CTRL( 10000u, 0 );
		else
			Heat_CTRL( 10000u, control_val );

	}
	Heat_CTRL( 1000u, 0u );
	Heat_CMD( false );
	OP_State = eShutReq;

}

/******************************** ����˵�� *************************************
*	
*******************************************************************************/
osThreadDef( PID_Task, osPriorityHigh, 1, 0 );
void	PID_Task_Init( void )
{
	osThreadCreate( osThread( PID_Task ), NULL );
}
/********************************** ����˵�� ***********************************
*  PID�������ͣ����
*******************************************************************************/
void  Heat_Run_Cmd( _Bool	NewState )
{
	if ( ! NewState )
	{
		//	����־����������־������ֹ��	//	osThread_SetReady;
		OP_State = eShutReq;
	}
	else if ( eRunning == OP_State )
	{
		//	���󿪱ã����ò�û�й�?!
		;
	}
	else
	{
		//	���󿪱ã�����֪���ǹص�
		OP_State = eRunning;
		PID_Task_Init();
	}
}

/********  (C) COPYRIGHT 2018 �ൺ���ػ����������޹�˾  **** End Of File ********/
