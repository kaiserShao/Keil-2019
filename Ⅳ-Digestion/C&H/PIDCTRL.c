/**************** (C) COPYRIGHT 2018 青岛中特环保仪器有限公司 *******************
* 文 件 名:  
* 创 建 者:  Kaiser
* 描  述  :  
* 最后修改:  
*********************************** 修订记录 ************************************
* 版  本: 
* 修订人: 
********************************************************************************/
#include "BIOS.H"
/******************************** 功能说明 *************************************
*	
*******************************************************************************/
typedef struct{
	float Limit;		//输出限幅
	float Target;		//目标输出量
	float Feedback;		//实际输出量	
	float Kp;	
	float Ki;	
	float Kd;	
	float E_0;			//当前误差
	float E_1;			//上一次误差
	float E_2;			//上上次误差
}PID;
 
PID TempPID = 
{
			1,								//输出限幅
			165,							//目标控制量
			25,								//反馈控制量
 
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
 

/******************************** 功能说明 *************************************
*	
*******************************************************************************/
void	PID_Calc_Init(PID *pid)
{
	pid->E_0 = pid->Target - pid->Feedback;
	pid->E_2 = pid->E_1;
	pid->E_1 = pid->E_0;
}

/******************************** 功能说明 *************************************
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

/******************************** 功能说明 *************************************
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
		//	定间隔延时
		{
			const	uint32_t	oneTick = osKernelSysTickMicroSec( 1000u );
			uint32_t	now = osKernelSysTick();
			uint32_t	lostTick, delayCount;
			//	计算已经逝去的tick计时单位
			lostTick = (uint32_t)( now + oneTick - 1u - SaveTick ) / oneTick;

			//	计算余下需要的延时周期(设定间隔时间：300ms)
			if ( lostTick < 900u )
			{
				delayCount = 900u - lostTick;
			}
			else
			{
				//	已经逝去的时间超过设定的间隔，重新计算时间起点后少量延时。
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

/******************************** 功能说明 *************************************
*	
*******************************************************************************/
osThreadDef( PID_Task, osPriorityHigh, 1, 0 );
void	PID_Task_Init( void )
{
	osThreadCreate( osThread( PID_Task ), NULL );
}
/********************************** 功能说明 ***********************************
*  PID任务的启停控制
*******************************************************************************/
void  Heat_Run_Cmd( _Bool	NewState )
{
	if ( ! NewState )
	{
		//	发标志，任务会检测标志自行中止。	//	osThread_SetReady;
		OP_State = eShutReq;
	}
	else if ( eRunning == OP_State )
	{
		//	请求开泵，但泵并没有关?!
		;
	}
	else
	{
		//	请求开泵，且已知泵是关的
		OP_State = eRunning;
		PID_Task_Init();
	}
}

/********  (C) COPYRIGHT 2018 青岛中特环保仪器有限公司  **** End Of File ********/
