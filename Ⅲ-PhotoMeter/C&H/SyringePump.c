/**************** (C) COPYRIGHT 2018 �ൺ���ػ����������޹�˾ *******************
* �� �� ��:  
* �� �� ��:  Kaiser
* ��  ��  :  
* ����޸�:  
*********************************** �޶���¼ ***********************************
* ��  ��: 
* �޶���: 
*******************************************************************************/
#include "BIOS.H"
#include "APPDEF.H"

/******************************** ����˵�� *************************************
*	ָ���
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


#define	Addr_CH1							0x01u
#define	Addr_CH2							0x02u
#define	Addr_CH3							0x03u
#define	Addr_CH4							0x04u


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

/******************************** ����˵�� *************************************
*	�������
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
/********************************** ����˵�� *************************************
*	���ڽ������ݴ���
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
		SET_BIT( TIM6->DIER, TIM_DIER_UIE );																						//	�򿪸����ж�
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

/******************************** ����˵�� *************************************
*	ָ����Ƴ���
*******************************************************************************/
struct	uDataPackGeneral	General; 
struct	uDataPackFactory	Factory; 
#include "string.h"
void	SyringePumpControl( uint8_t	CMD, uint32_t	Data,	enum	enumPumpChannelSelect Channel )
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
				case SyringePump_CH1:
					General.Address				= Addr_CH1;
					break;
				case SyringePump_CH2:
					General.Address				= Addr_CH2;
					break;
				case SyringePump_CH3:
					General.Address				= Addr_CH3;
					break;
				case SyringePump_CH4:
					General.Address				= Addr_CH4;
					break;
				default:
					General.Address				= 0u;
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
				case SyringePump_CH1:
					General.Address				= Addr_CH1;
					break;
				case SyringePump_CH2:
					General.Address				= Addr_CH2;
					break;
				case SyringePump_CH3:
					General.Address				= Addr_CH3;
					break;
				case SyringePump_CH4:
					General.Address				= Addr_CH4;
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

/******************************** ����˵�� *************************************
*	�������
*******************************************************************************/
static	struct	uDataUnpack	DataUnpackValve;
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
/******************************** ����˵�� *************************************
*	�����ж�
*******************************************************************************/
void	USART2_IRQHandler( void )
{
	USART2_DATA_DEAL();
}

/******************************** ����˵�� *************************************
*	��ʱ�ж�
*******************************************************************************/
_Bool	RxUpToDate = false;
void	TIM6_IRQHandler( void )
{
	CLEAR_BIT( TIM6->SR, TIM_SR_UIF ); 
	if( RxFlag )
	{
		if( UsartDelayTime ++ > 15u )
		{
			CLEAR_BIT( TIM6->DIER, TIM_DIER_UIE );																		//	�����ж�
			CLEAR_BIT( TIM6->CR1, TIM_CR1_CEN );
			TIM6->CNT = 0u;
			RxUpToDate = DataDealUnpack( &DataUnpackValve );
			UsartDelayTime = 0u;
			RxFlag = false;
		}
	}
}

/********************************** ����˵�� *************************************
*	��ѯ״̬����
*********************************************************************************/
_Bool	QueryResponse( uint8_t CMD, enum	enumPumpChannelSelect Channel )
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

/********************************** ����˵�� *************************************
*	ע��ÿ���
*********************************************************************************/
int16_t	VPSitionSet[SyringePump_CHMax];
int16_t	VPSpeedSet[SyringePump_CHMax];
static	int16_t	SpeedOldSet[SyringePump_CHMax];
//static	int16_t	StepOldSet[SyringePump_CHMax];

void	PumpControl( enum	enumPumpChannelSelect Channel )
{
	
	int16_t	StepSet = VPSitionSet[Channel];
	int16_t	SpeedSet = VPSpeedSet[Channel];
	
	if( SpeedOldSet[Channel] != SpeedSet )
	{
		SyringePumpControl( MotorSpeed, SpeedSet, Channel );
		SpeedOldSet[Channel] = SpeedSet;
		osDelay( 80u );	
	}

	if(( !VPState[Channel] ) && ( VPPosition[Channel] != StepSet ) )
	{	
		SyringePumpControl( (StepSet - VPPosition[Channel] >= 0 )? AntiClockwiseRotation : ClockwiseRotation, __fabs( StepSet - VPPosition[Channel] ), Channel );
//		StepOldSet[Channel] = StepSet;
		osDelay( 80u );	
	}

}


/********************************** ����˵�� *************************************
*	ע�������
*********************************************************************************/
uint16_t	VPState[SyringePump_CHMax];
uint16_t	VPPosition[SyringePump_CHMax];
uint16_t	StopState[SyringePump_CHMax];
static	_Bool	StopstateOld[SyringePump_CHMax] = {0u,0u,0u,0u};

void	_task_SyringePumpCTRL( const	void	*P_arg )
{
	for( ;; )
	{
		for( uint8_t	i = 0u; i < SyringePump_CHMax; i ++ )
		{
		if( QueryResponse( MotorState, (enum  enumPumpChannelSelect)i ) )								//	���״̬��ѯ ע���
			VPState[(enum  enumPumpChannelSelect)i] = DataUnpackValve.AckState;

		if( QueryResponse( CurrentPositionQuery, (enum  enumPumpChannelSelect)i ) )			//	���λ�ò�ѯ ע���
			VPPosition[(enum  enumPumpChannelSelect)i] = DataUnpackValve.Data[1u] << 8u | DataUnpackValve.Data[0u];
		
		if( QueryResponse( StopReasonQuery, (enum  enumPumpChannelSelect)i ) )								//	ֹͣ�¼���ѯ ע���
			StopState[(enum  enumPumpChannelSelect)i] = DataUnpackValve.Data[1u] << 8u | DataUnpackValve.Data[0u];

		if( StopState[(enum  enumPumpChannelSelect)i] != StopstateOld[(enum  enumPumpChannelSelect)i] )	//	���������ͣ
		{
			StopstateOld[(enum  enumPumpChannelSelect)i] = StopState[(enum  enumPumpChannelSelect)i];
			if( StopState[(enum  enumPumpChannelSelect)i] == 0x06u )
			{
				SyringePumpControl( ClearCurrentPosition, 0u, (enum  enumPumpChannelSelect)i );		//	���λ������ ע���
				osDelay(80u);			
				usRegHoldingBuf[26u + (enum  enumPumpChannelSelect)i * 5u ] = 0u;
			}
		}
			PumpControl( (enum  enumPumpChannelSelect)i );//	������� ע���		
		}
	}
}
/********************************** ����˵�� *************************************
*	ת����ע��������ʼ��
*********************************************************************************/
osThreadDef( _task_SyringePumpCTRL, osPriorityAboveNormal, 1u, 0u );
void	SyringePump_CTRL_Init( void )
{

	usRegHoldingBuf[27u] = 40u;
	usRegHoldingBuf[32u] = 20u;
	usRegHoldingBuf[37u] = 30u;
	usRegHoldingBuf[42u] = 30u;

	for( uint8_t	i = 0u; i < SyringePump_CHMax; i ++ )
	{
		if( QueryResponse( MotorState, (enum  enumPumpChannelSelect)i ) )								//	���״̬��ѯ ע���
			VPState[(enum  enumPumpChannelSelect)i] = DataUnpackValve.AckState;

		if( QueryResponse( CurrentPositionQuery, (enum  enumPumpChannelSelect)i ) )			//	���λ�ò�ѯ ע���
			VPPosition[(enum  enumPumpChannelSelect)i] = DataUnpackValve.Data[1u] << 8u | DataUnpackValve.Data[0u];
		
		if( QueryResponse( StopReasonQuery, (enum  enumPumpChannelSelect)i ) )					//	ֹͣ�¼���ѯ ע���
			StopState[(enum  enumPumpChannelSelect)i] = DataUnpackValve.Data[1u] << 8u | DataUnpackValve.Data[0u];

	}
	VPSitionSet[SyringePump_CH1] = usRegHoldingBuf[26u] = usRegHoldingBuf[25u] = VPPosition[SyringePump_CH1];
	VPSitionSet[SyringePump_CH2] = usRegHoldingBuf[31u] =	usRegHoldingBuf[30u] = VPPosition[SyringePump_CH2];
	VPSitionSet[SyringePump_CH3] = usRegHoldingBuf[36u] =	usRegHoldingBuf[35u] = VPPosition[SyringePump_CH3];
	VPSitionSet[SyringePump_CH4] = usRegHoldingBuf[41u] =	usRegHoldingBuf[40u] = VPPosition[SyringePump_CH4];	
	osThreadCreate( osThread( _task_SyringePumpCTRL ), NULL );

	osDelay( 3000u );

}











/********  (C) COPYRIGHT 2018 �ൺ���ػ����������޹�˾  **** End Of File ********/
