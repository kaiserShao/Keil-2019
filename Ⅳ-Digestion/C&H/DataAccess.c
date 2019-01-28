/**************** (C) COPYRIGHT 2018 青岛中特环保仪器有限公司 *******************
* 文 件 名:  
* 创 建 者:  Kaiser
* 描  述  :  
* 最后修改:  
*********************************** 修订记录 ************************************
* 版  本: 
* 修订人: 
********************************************************************************/
/*******************************************************************************
*	
*******************************************************************************/
#include "BIOS.H"
/******************************** 功能说明 *************************************
*	求平均值
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
/******************************** 功能说明 *************************************
*	AD数据转换
*******************************************************************************/
static	uint16_t	ADC_Data[3u][4u];
static	uint16_t	ADC_DataOut[3u][4u];
static	uint16_t	ADC_DataCpoy[3u][4u];
uint16_t	ADC_OUT[3u];
void	ADDataConver(void)
{
	uint8_t	t,i;
	for( t = 0u; t < 12u; t ++ )
	{
		ADC_DataCpoy[t/4u][t%4u] = ADC_Databuf[t];	//	数据分类
	}
	
	for( i = 0u; i < 3u; i ++ )										//	滑动滤波
	{	
		ADC_Data[i][3u] = ADC_Data[i][2u];
		ADC_Data[i][2u] = ADC_Data[i][1u];
		ADC_Data[i][1u] = ADC_Data[i][0u];
		ADC_Data[i][0u] = DataAverage( ADC_DataCpoy[i], 4u );
	}
	
	for( i = 0u; i < 3u; i ++ )										//	滑动滤波
	{	
		ADC_DataOut[i][3u] = ADC_DataOut[i][2u];
		ADC_DataOut[i][2u] = ADC_DataOut[i][1u];
		ADC_DataOut[i][1u] = ADC_DataOut[i][0u];
		ADC_DataOut[i][0u] = (uint16_t)DataAverage( ADC_Data[i], 4u );
		
		ADC_OUT[0u] = DataAverage( ADC_DataOut[0u], 4u );	//求平均
		ADC_OUT[1u] = DataAverage( ADC_DataOut[1u], 4u );
		ADC_OUT[2u] = DataAverage( ADC_DataOut[2u], 4u );

	}
}

/******************************** 功能说明 *************************************
*	
*******************************************************************************/
double	TempConver( uint32_t	DataOut )
{
	return
		(
		(( (float) DataOut / 4095.0f * 3300.0f ) / 10.0f)/*PT100两端电压*/
		/ 
		(2.55/2.0)/*1.25ma电流*/
		- 
		100.0f 
		)
		/ 0.3885f;
}

/********  (C) COPYRIGHT 2018 青岛中特环保仪器有限公司  **** End Of File ********/
