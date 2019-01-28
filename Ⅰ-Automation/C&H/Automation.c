/**************** (C) COPYRIGHT 2018 �ൺ���ػ����������޹�˾ *******************
* �� �� ��:  
* �� �� ��:  Kaiser
* ��  ��  :  
* ����޸�:  
*********************************** �޶���¼ ************************************
* ��  ��: 
* �޶���: 
********************************************************************************/
#include "APPDEF.H"

/******************************** ����˵�� *************************************
*	
*******************************************************************************/
void	Air_Charge( void )
{
	static	uint16_t	i = 0u;
	if( usRegHoldingBuf[21u] )
	{		
		if( usRegHoldingBuf[9u] )
		{
			i = 0u;
			Air_Charge_Driver( usRegHoldingBuf[21u] );		
			usRegHoldingBuf[23u] = 1u;
		}
		else
		{
			if( usRegHoldingBuf[22u] )
			{
				if( i ++ > usRegHoldingBuf[22u] / 30u )
				{
					i = 0u;
					usRegHoldingBuf[22u] = 0u;
					usRegHoldingBuf[21u] = 0u;
					Air_Charge_Driver( usRegHoldingBuf[21u] );
					usRegHoldingBuf[23u] = 0u;
					osDelay(1000);
					usRegHoldingBuf[10u] = 0u;
				}
				else
				{
					Air_Charge_Driver( usRegHoldingBuf[21u] );
					usRegHoldingBuf[23u] = 1u;
				}					
			}
		}
	}
	else
	{
		i = 0u;
		Air_Charge_Driver( 0u );	
		usRegHoldingBuf[23u] = 0u;
	}
	
	Air_Charge_Valve( usRegHoldingBuf[10u] );
	usRegHoldingBuf[11u] = usRegHoldingBuf[10u];

}

/******************************** ����˵�� *************************************
*	
*******************************************************************************/
void	Air_Bleed( void )
{
	static	uint16_t	i = 0u;
	if( usRegHoldingBuf[26u] )		//	�趨�ٶȲ�Ϊ��
	{		
		if( usRegHoldingBuf[9u] )	//	��Һ�����
		{
			i = 0u;
			Air_Bleed_Driver( usRegHoldingBuf[26u] );		
			usRegHoldingBuf[28u] = 1u;
		}
		else
		{
			if( usRegHoldingBuf[27u] )		//	����ʱ��������
			{
				if( i ++ > usRegHoldingBuf[27u] / 30u )
				{
					i = 0u;
					usRegHoldingBuf[27u] = 0u;
					usRegHoldingBuf[26u] = 0u;
					Air_Bleed_Driver( usRegHoldingBuf[26u] );
					usRegHoldingBuf[28u] = 0u;
					osDelay(500);
					usRegHoldingBuf[12u] = 0u;
				}
				else
				{
					Air_Bleed_Driver( usRegHoldingBuf[26u] );
					usRegHoldingBuf[28u] = 1u;
				}					
			}
		}
	}
	else
	{
		i = 0u;
		Air_Bleed_Driver( 0u );
		usRegHoldingBuf[28u] = 0u;
	}
	
	Air_Bleed_Valve( usRegHoldingBuf[12u] );
	usRegHoldingBuf[13u] = usRegHoldingBuf[12u];
}

/******************************** ����˵�� *************************************
*	
*******************************************************************************/
void	WaterIn( void )
{
	Injection_Water_Valve( usRegHoldingBuf[14u] );
	usRegHoldingBuf[15u] = usRegHoldingBuf[14u];
}

/******************************** ����˵�� *************************************
*	Һ�����ȱʧ����
*******************************************************************************/
void	ExistUpdate( void )
{
	static	uint8_t	i = 0u;

	usRegHoldingBuf[9u] = CommonUpdate();	//	Һλ������
	
	if( (usRegHoldingBuf[6u] < usRegHoldingBuf[5u]) )	//	ע��õ�ǰλ��С��Ŀ��λ��
	{
		if( usRegHoldingBuf[9u] == 0u )
		{
			if( i ++ > 30u )		//	
			{
				i = 0u;
				usRegHoldingBuf[19u] = 0;
			}
		}
		else
		{
			i = 0u;
			usRegHoldingBuf[19u] = 1;
		}
	}
}


/********************************** ����˵�� *************************************
*	���ݽ�������
*********************************************************************************/
uint16_t	Version = 100u;
uint16_t	CompileYear = 2018u;
uint16_t	CompileMonthDate = 1020u;
void	DataAccess( void )
{
	usRegHoldingBuf[1u] = Version; 
	usRegHoldingBuf[2u] = CompileYear;
	usRegHoldingBuf[3u] = CompileMonthDate;

	//	��е��ָ�������ݽ���
	CoordinateSet = usRegHoldingBuf[30u];
	usRegHoldingBuf[31u] = CoordinatePosition;
	//	ע����������ݽ���
	usRegHoldingBuf[6u] = VPPosition;
	usRegHoldingBuf[8u] = VPState;
	
	ExistUpdate();	//	Һ����ڸ���
	
	Air_Charge();		//	������

	Air_Bleed();		//	�ſձ�
	
	WaterIn();			//	����ˮ��
	
	Angel = usRegHoldingBuf[35u];
	usRegHoldingBuf[36u] = AngelRT;
//	Rotate_Servo_Angel( usRegHoldingBuf[35u] );	//	�����ת�Ƕ�
	
}
/******************************** ����˵�� *************************************
*	
*******************************************************************************/
#define ModbusAddress 1u
int	main( void )
{
	BIOS_Init();
	usRegHoldingBuf[0u] = 0u;
	usRegHoldingBuf[4u] = 0u;
	MODBUS_Init( ModbusAddress );
	Arm_Init();
	while(usRegHoldingBuf[4u] != 1u)
		osDelay( 50u );
	Angel = usRegHoldingBuf[35u] = 0u;
	Rotate_Servo_Init();
	while( AngelRT != Angel)
		osDelay( 50u );
	Angel = usRegHoldingBuf[35u] = 100;
	while( AngelRT != Angel)
		osDelay( 50u );
	usRegHoldingBuf[0u] = 1u;
	while(usRegHoldingBuf[4u] != 2u)
		osDelay( 50u );
	usRegHoldingBuf[14u] = 1u;
	WaterIn();			//	����ˮ��
	SyringePump_CTRL_Init();
	usRegHoldingBuf[0u] = 2u;
	while(1)
	{
		osDelay( 30u );
		DataAccess();
	}

}














/********  (C) COPYRIGHT 2018 �ൺ���ػ����������޹�˾  **** End Of File ********/
