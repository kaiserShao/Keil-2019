/**************** (C) COPYRIGHT 2018 青岛中特环保仪器有限公司 *******************
* 文 件 名:  
* 创 建 者:  Kaiser
* 描  述  :  
* 最后修改:  
*********************************** 修订记录 ***********************************
* 版  本: 
* 修订人: 
*******************************************************************************/
#include "BIOS.H"
#include "APPDEF.H"

/******************************** 功能说明 *************************************
*	
*******************************************************************************/
enum	ValveState
{
	StateOK,
	FrameErr,
	ParameterErr,
	OptocouplerErr,
	MotorIsBusy,
	TaskBeSuspended,
	UnknownErr,
};
struct	uDataPackGeneral
{
	uint8_t		FrameHead;
	uint8_t		Address;
	uint8_t		CTRLCommand;
	uint8_t		Data[2u];
	uint8_t		FrameEnd;
	uint8_t		CRCSum[2u];
	
};
struct	uDataPackFactory
{
	uint8_t		FrameHead;
	uint8_t		Address;
	uint8_t		CTRLCommand;
	uint8_t		Password[4u];
	uint8_t		Data[4u];
	uint8_t		FrameEnd;
	uint8_t		CRCSum[2u];
	
};
struct	uDataUnpack
{
	uint8_t		FrameHead;
	uint8_t		Address;
	uint8_t		AckState;
	uint8_t		Data[2u];
	uint8_t		FrameEnd;
	uint8_t		CRCSum[2u];
};

/******************************** 功能说明 *************************************
*	指令定义
*******************************************************************************/
#define	AddrSet								0x00
#define	RS232BaudRateSet			0x01
#define	RS485BaudRateSet			0x02
#define	CANBaudRateSet				0x03
#define	SpeedMaxSet						0x07
#define	ChannelMaxSet					0x0A
#define	ResertSpeedSet				0x0B
#define	ResertDirSet					0x0C
#define	PowerOnRst						0x0E
#define	DestinationCan				0x10
	
#define	AddrQuery							0x20u
#define	RS232BaudRateQuery		0x21u
#define	RS485BaudRateQuery		0x22u
#define	CANBaudRateQuery			0x23u
#define	SpeedMaxQuery					0x27u
#define	ChannelMaxQuery				0x2Au
#define	ResertSpeedQuery			0x2Bu
#define	ResertDirQuery				0x2Cu
#define	PowerOnRstQuery				0x2Du
#define	DestinationCanQuery		0x30u
#define	ChannelCurrentQuery		0x3Eu
#define	VerisonQuery					0x3Fu
#define	StopReasonQuery				0x65u
#define	CurrentPositionQuery	0x66u
#define	ClearCurrentPosition	0x67u
#define	CurrentDIRQuery				0x68u


#define	AntiClockwiseRotation	0x4Du
#define	ClockwiseRotation			0x42u
#define	ChannelSelectSet			0x44u
#define	ChannelResert					0x45u
#define	Braking								0x49u
#define	MotorState						0x4Au
#define	MotorSpeed						0x4Bu
#define	ResetInsertData				0xFFu


#define	PumpAddr							0x00u

#define	FactoryPassword1			0xAAu
#define	FactoryPassword2			0xBBu
#define	FactoryPassword3			0xEEu
#define	FactoryPassword4			0xFFu
#define	FrameHeadValue				0xCCu
#define	FrameEndValue					0xDDu

#define	Baud9600							0x00u
#define	Baud19200							0x01u
#define	Baud38400							0x02u
#define	Baud57600							0x03u
#define	Baud115200						0x04u

#define	SpeedMax							0xC8u

#define	PowerUpRst						0x01u

/******************************** 功能说明 *************************************
*	串口输出
*******************************************************************************/
void	DataPrint( uint8_t * buffer, uint8_t lenth )
{
	uint8_t	i;
	USART_ITConfig( USART2, /*USART_IT_TXE |*/ USART_IT_RXNE, DISABLE );
	RS485_REDE2( true );
	delay_us(50u);
	for( i = 0u; i < lenth; i ++ )
	{
		UART2_Send( * buffer ++ );
		while( !USART_GetFlagStatus(USART2,USART_FLAG_TC) );

	}
	delay_us(50u);
	RS485_REDE2( false );

	USART_ITConfig( USART2, /*USART_IT_TXE |*/ USART_IT_RXNE, ENABLE );

}

void	DataRec( uint8_t * buffer )
{
		* buffer = UART2_Received();	
}
/********************************** 功能说明 *************************************
*	串口接收数据处理
*********************************************************************************/
static	uint8_t		DataInbuf[20u];
static	volatile	uint16_t	UsartDelayTime = 0u;
static	_Bool		RxFlag = false;
void	USART2_DATA_DEAL( void )
{
	static	uint8_t i = 0u;
	uint8_t	j;

	if( !RxFlag )
	{
		RxFlag = true;
		TIM6->CNT = 0u;
		SET_BIT( TIM6->DIER, TIM_DIER_UIE );																						//	打开更新中断
		SET_BIT( TIM6->CR1,  TIM_CR1_CEN );
		for( j = 0u; j < 20u; j ++ )
		{
			DataInbuf[j] = 0u;
		}
		i = 0u;
	}
	
	if( USART_GetITStatus( USART2, USART_IT_RXNE ))
	{
		UsartDelayTime = 0u;
		RxUpToDate = false;
		DataRec( DataInbuf + i ++ );
	}
	
}

/******************************** 功能说明 *************************************
*	指令控制程序
*******************************************************************************/
struct	uDataPackGeneral	General; 
struct	uDataPackFactory	Factory; 
#include "string.h"
void	SyringePumpControl( uint8_t	CMD, uint32_t	Data,	uint8_t	Channel )
{
	switch( CMD )
	{
		case	AddrSet					 :
		case  RS232BaudRateSet :
		case  RS485BaudRateSet :
		case	CANBaudRateSet	 :
		case	SpeedMaxSet			 :
		case	ChannelMaxSet		 :
		case	ResertSpeedSet	 :
		case	ResertDirSet		 :
		case	PowerOnRst			 :
		case	DestinationCan	 :
		case  ResetInsertData				:
			Factory.FrameHead			= FrameHeadValue;
			switch( Channel )
			{
				case PumpAddr:
					Factory.Address				= PumpAddr;
					break;
				default:
					break;
			}
			Factory.Password[3u]		= FactoryPassword1;
			Factory.Password[2u]		= FactoryPassword2;
			Factory.Password[1u]		= FactoryPassword3;
			Factory.Password[0u]		= FactoryPassword4;
			Factory.CTRLCommand			= CMD;
			Factory.Data[3u]				= Data >> 24u;
			Factory.Data[2u]      	= Data >> 16u;
			Factory.Data[1u]      	= Data >> 8u;
			Factory.Data[0u]      	= Data >> 0u;
			Factory.FrameEnd				= FrameEndValue;
			{
			uint16_t	CRCSUM 				=
			Factory.FrameHead				+
			Factory.Address					+
			Factory.Password[0u]		+
			Factory.Password[1u]		+
			Factory.Password[2u]		+
			Factory.Password[3u]		+
			Factory.CTRLCommand			+
			Factory.Data[0u]				+
			Factory.Data[1u]				+
			Factory.Data[2u]				+
			Factory.Data[3u]				+
			Factory.FrameEnd;
			Factory.CRCSum[1u]			= CRCSUM >> 8u;
			Factory.CRCSum[0u]			= CRCSUM >> 0u;
			}
			DataPrint( (uint8_t *)&Factory, 14u );
			break;
		case	AddrQuery							:
		case	RS232BaudRateQuery		:
		case	RS485BaudRateQuery		:
		case	CANBaudRateQuery			:
		case	SpeedMaxQuery					:
		case  ChannelMaxQuery				:
		case  ResertSpeedQuery			:
		case  ResertDirQuery				:
		case  PowerOnRstQuery				:
		case  DestinationCanQuery		:
		case  ChannelCurrentQuery		:
		case  VerisonQuery					:
		case  StopReasonQuery				:
		case  CurrentPositionQuery	:
		case	ClearCurrentPosition	:
		case	CurrentDIRQuery				:
		case  ClockwiseRotation			:
		case  AntiClockwiseRotation	:
		case  ChannelSelectSet			:
		case  ChannelResert					:
		case  Braking								:
		case  MotorState						:
		case	MotorSpeed						:
			General.FrameHead		= FrameHeadValue;
			switch( Channel )
			{
				case PumpAddr:
					General.Address				= PumpAddr;
					break;
				default:
					break;
			}
			General.CTRLCommand	= CMD;
			General.Data[1u]			= Data >> 8u;
			General.Data[0u]			= Data >> 0u;
			General.FrameEnd			= FrameEndValue;
			{
			uint16_t	CRCSUM					= 
			General.FrameHead    +
			General.Address      +
			General.CTRLCommand  +
			General.Data[0u] 		 +
			General.Data[1u] 		 +
			General.FrameEnd;
			General.CRCSum[1u]		= CRCSUM >> 8u;
			General.CRCSum[0u]		= CRCSUM >> 0u;
			}
			DataPrint( (uint8_t *)&General, 8u );	
			break;
		default:
			break;
	}
}

/******************************** 功能说明 *************************************
*	解包程序
*******************************************************************************/
struct	uDataUnpack	DataUnpackValve;
_Bool	DataDealUnpack( struct	uDataUnpack * DataUnpack )
{
	if( ( DataInbuf[0u] == FrameHeadValue ) && ( DataInbuf[5u] == FrameEndValue ) )
	{
		uint8_t	i;
		uint16_t	DataSum;
		uint16_t	CRCsum = DataInbuf[7u] << 8u | DataInbuf[6u];
		DataSum = 0u;
		for( i = 0u; i < 6u; i ++ )
		{
			DataSum += DataInbuf[i];
		}
		if( DataSum == CRCsum )
		{

			DataUnpack->FrameHead	= DataInbuf[0u];
			DataUnpack->Address		= DataInbuf[1u];
			DataUnpack->AckState	= DataInbuf[2u];
			DataUnpack->Data[0u]	= DataInbuf[3u];
			DataUnpack->Data[1u]	= DataInbuf[4u];
			DataUnpack->FrameEnd	= DataInbuf[5u];
			DataUnpack->CRCSum[0u]= DataInbuf[6u];
			DataUnpack->CRCSum[1u]= DataInbuf[7u];
			return	true;
		}
	}
	return	false;
}

/******************************** 功能说明 *************************************
*	串口中断
*******************************************************************************/
void	USART2_IRQHandler( void )
{
	USART2_DATA_DEAL();
}

/******************************** 功能说明 *************************************
*	超时中断
*******************************************************************************/
_Bool	RxUpToDate = false;
void	TIM6_IRQHandler( void )
{
	CLEAR_BIT( TIM6->SR, TIM_SR_UIF ); 
	if( RxFlag )
	{
		if( UsartDelayTime ++ > 15u )
		{
			CLEAR_BIT( TIM6->DIER, TIM_DIER_UIE );																		//	更新中断
			CLEAR_BIT( TIM6->CR1, TIM_CR1_CEN );
			TIM6->CNT = 0u;
			RxUpToDate = DataDealUnpack( &DataUnpackValve );
			UsartDelayTime = 0u;
			RxFlag = false;
		}
	}
}

/********************************** 功能说明 *************************************
*	查询状态函数
*********************************************************************************/
_Bool	QueryResponse( uint8_t CMD, uint8_t Channel )
{
	uint8_t	i = 0u;

	SyringePumpControl( CMD, 0u, Channel );

	do
	{
		osDelay(50u);
	}while( (!RxUpToDate) && ( ++i < 10u ) );
	RxUpToDate = false;

	if( i < 10u )
		return	true;
	return	false;
}

/********************************** 功能说明 *************************************
*	注射泵控制
*********************************************************************************/
void	PumpControl( int16_t	StepSet, uint16_t	SpeedSet )
{
	static	int16_t	StepOldSet;
	static	int16_t	SpeedOldSet;
	if( SpeedOldSet != SpeedSet )
	{
		SyringePumpControl( MotorSpeed, SpeedSet, PumpAddr );
		SpeedOldSet = SpeedSet;
		osDelay( 60u );	
	}

	if(( !VPState ) && ( VPPosition != StepSet ) )
	{	
//		if( VPPosition != StepOldSet )
//		{
//			SyringePumpControl( (StepSet - VPPosition >= 0 )? AntiClockwiseRotation : ClockwiseRotation, __fabs( StepSet - VPPosition ), PumpAddr );
//			
//		}
//		else
//		{
			SyringePumpControl( (StepSet - StepOldSet >= 0 )? AntiClockwiseRotation : ClockwiseRotation, __fabs( StepSet - StepOldSet ), PumpAddr );
			StepOldSet = StepSet;	
//		}
		osDelay( 60u );	
	}

}


/********************************** 功能说明 *************************************
*	注射泵任务
*********************************************************************************/
uint16_t	VPState;
uint16_t	VPPosition;
uint16_t	StopState;

void	_task_SyringePumpCTRL( const	void	*P_arg )
{
	static	_Bool	StopstateOld = 0u;
	for( ;; )
	{
		if( QueryResponse( MotorState, PumpAddr ) )								//	电机状态查询 注射泵
			VPState = DataUnpackValve.AckState;

		if( QueryResponse( CurrentPositionQuery, PumpAddr ) )			//	电机位置查询 注射泵
			VPPosition = DataUnpackValve.Data[1u] << 8u | DataUnpackValve.Data[0u];
		
		if( QueryResponse( StopReasonQuery, PumpAddr ) )								//	停止事件查询 注射泵
			StopState = DataUnpackValve.Data[1u] << 8u | DataUnpackValve.Data[0u];
		if( StopState != StopstateOld )	//	如果遇光耦停
		{
			StopstateOld = StopState;
			if( StopState == 0x06u )
			{
				SyringePumpControl( ClearCurrentPosition, 0u, PumpAddr );		//	电机位置清零 注射泵
				osDelay(80u);			
				usRegHoldingBuf[6u] = 0u;
			}
			if(StopState == 0x02u)
			{
			}
		}
		PumpControl( usRegHoldingBuf[5u], usRegHoldingBuf[7u] );//	电机控制 注射泵
	}
}
/********************************** 功能说明 *************************************
*	注射泵任务初始化
*********************************************************************************/
osThreadDef( _task_SyringePumpCTRL, osPriorityAboveNormal, 1u, 0u );
void	SyringePump_CTRL_Init( void )
{
	uint8_t	i = 0u;
//	SyringePumpControl( ResetInsertData, 0u, 0u );						//	 
//	osDelay( 8000u );
//	SyringePumpControl( AddrSet, PumpAddr, 0u );						//	 
//	osDelay( 12000u );		
	usRegHoldingBuf[7u] = 30u;
	usRegHoldingBuf[5u] = 0u;
	SyringePumpControl( ChannelResert, 0u, PumpAddr );						//	 
	osDelay( 50u );
	i = 0u;
	do{	
		if( QueryResponse( MotorState, PumpAddr ) )								//	电机状态查询 注射泵
			VPState = DataUnpackValve.AckState;
	}while( ( VPState != 0x00u ) && ( i ++ < 250u ) );
	if( i < 250u )
	{
		SyringePumpControl( ClearCurrentPosition, 0u, PumpAddr );		//	电机位置清零 注射泵
	}while(VPPosition) 
		osDelay( 50u );
	
	osThreadCreate( osThread( _task_SyringePumpCTRL ), NULL );
	
//	SolenoidValve_Control( true );
//	SolenoidValve_Control2( true );
//	Blender_Control( 30 );
//	EMotor_Control( 30 );
//	osDelay( 3000u );
//	Blender_Control( 0 );
//	EMotor_Control( 0 );
//	SolenoidValve_Control( false );
//	SolenoidValve_Control2( false );
}











/********  (C) COPYRIGHT 2018 青岛中特环保仪器有限公司  **** End Of File ********/
