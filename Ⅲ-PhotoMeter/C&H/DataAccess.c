/**************** (C) COPYRIGHT 2018 �ൺ���ػ����������޹�˾ *******************
* �� �� ��:  
* �� �� ��:  	Kaiser
* ��  ��  :  
* ����޸�:  
*********************************** �޶���¼ ***********************************
* ��  ��: 
* �޶���: 
*******************************************************************************/
#include "BIOS.H"
#include "string.h"
#include "math.h"
#include "APPDEF.H"
#define	CalibrationAddressBase	0x0801F000u						//	�궨������ʼ��ַ
#define	AddressCRCAdd	0x0801F100u											//	�궨����У���ֵ�ַ
#define	AddressCRC		0x35CAu													//	�궨����У����ֵ

struct	uCalibrateData CalibrateData[CH_Max];

/******************************** ����˵�� *************************************
*	
*******************************************************************************/

//-------------------------------------------------------------
//���� : ��С���˷�ֱ����� y = a��x + b�� ����ϵ��a �� b
//���� : x -- ���նȵ�����
//       y --  ���ʵ�����
//       num �����������Ԫ�ظ�����x[]��y[]��Ԫ�ظ����������
//       a,b ���Ƿ���ֵ
//���� : ��ϼ���ɹ�����true, ��ϼ���ʧ�ܷ���false
//���� ��
//-------------------------------------------------------------
static	_Bool LeastSquareLinearFit( struct	uCalibrateData * Data )
{
	float sum_x2 = 0.0f;
	float sum_y  = 0.0f;
	float sum_x  = 0.0f;
	float sum_xy = 0.0f;
	float	XY[2u][6u] = {0};
	const	uint8_t	Num = 6u;
	//	������С���˷����ֱ�����������
	for( uint8_t i = 0u; i < Num; i ++ )
	{
		XY[0u][i] = log10( (double)Data->WaterAD / (double)Data->XY[0u][i] );
	}
	for( uint8_t i = 0u; i < Num; i ++ )
	{
		XY[1u][i] = Data->XY[1u][i];
	}
	for( uint8_t i = 0u; i < Num; i ++ )
	{
		sum_x2 += XY[0u][i] * XY[0u][i];
		sum_y  += XY[1u][i];
		sum_x  += XY[0u][i];
		sum_xy += XY[0u][i] * XY[1u][i];
	}
	//	����A B��ֵ
	Data->Scaling   = ( Num * sum_xy - sum_x * sum_y ) / ( Num * sum_x2 - sum_x * sum_x );

	Data->Intercept = ( sum_x2 * sum_y - sum_x * sum_xy ) / ( Num * sum_x2 - sum_x * sum_x );

	return true;
}


/******************************** ����˵�� *************************************
*	�궨���ݱ���
*******************************************************************************/
_Bool	CalibrationSave( void )
{
//	uint8_t	i;

	return	true;
}
/******************************** ����˵�� *************************************
*	�궨���ݶ�ȡ
*******************************************************************************/
void	CalibrationLoad( void )
{

}




/*******************************************************************************
*	
*******************************************************************************/
/******************************** ����˵�� *************************************
*	��ƽ��ֵ
*******************************************************************************/
float	DataAverage( uint16_t * data, uint8_t lenth )	
{
	uint8_t	i;
	float	Sum = 0.0f;
	for( i = 0u; i < lenth; i ++ )
	{
		Sum += data[i];
	}
	return (float)( Sum / lenth );
}
/******************************** ����˵�� *************************************
*	AD����ת��
*******************************************************************************/
static	uint16_t	ADC_Data[4u][4u];
static	uint16_t	ADC_DataOut[4u][4u];
static	uint16_t	ADC_DataCpoy[4u][4u];
uint16_t	ADC_OUT[4u];
void	ADDataConver(void)
{
	uint8_t	t,i;
	for( t = 0u; t < 16u; t ++ )
	{
		ADC_DataCpoy[t/4u][t%4u] = ADC_Databuf[t];	//	���ݷ���
	}
	
	for( i = 0u; i < 4u; i ++ )										//	�����˲�
	{	
		ADC_Data[i][3u] = ADC_Data[i][2u];
		ADC_Data[i][2u] = ADC_Data[i][1u];
		ADC_Data[i][1u] = ADC_Data[i][0u];
		ADC_Data[i][0u] = DataAverage( ADC_DataCpoy[i], 4u );
	}
	
	for( i = 0u; i < 4u; i ++ )										//	�����˲�
	{	
		ADC_DataOut[i][3u] = ADC_DataOut[i][2u];
		ADC_DataOut[i][2u] = ADC_DataOut[i][1u];
		ADC_DataOut[i][1u] = ADC_DataOut[i][0u];
		ADC_DataOut[i][0u] = (uint16_t)DataAverage( ADC_Data[i], 4u );
		
		ADC_OUT[0u] = DataAverage( ADC_DataOut[0u], 4u );	//��ƽ��
		ADC_OUT[1u] = DataAverage( ADC_DataOut[1u], 4u );
		ADC_OUT[2u] = DataAverage( ADC_DataOut[2u], 4u );
		ADC_OUT[3u] = DataAverage( ADC_DataOut[3u], 4u );

	}
}

/******************************** ����˵�� *************************************
*	Ũ��ֵ����
*******************************************************************************/
float	ConcentrationOut( enum	enumLEDChannelSelect	Channel )
{
	//	X�� ����� = log10��ȥ����ˮ��ADֵ / ����ADֵ��
	//	Y�� Ũ�� = A*����� + B
	float	ValueX = log10( (double)CalibrateData[Channel].WaterAD / ADC_OUT[Channel] );
	float	ValueY = CalibrateData[Channel].Scaling * ValueX + CalibrateData[Channel].Intercept;
	
	return	ValueY;
}
/********  (C) COPYRIGHT 2018 �ൺ���ػ����������޹�˾  **** End Of File ********/
