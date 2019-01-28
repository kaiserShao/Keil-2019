/**************** (C) COPYRIGHT 2018 青岛中特环保仪器有限公司 *******************
* 文 件 名:  
* 创 建 者:  	Kaiser
* 描  述  :  
* 最后修改:  
*********************************** 修订记录 ***********************************
* 版  本: 
* 修订人: 
*******************************************************************************/
#include "BIOS.H"
#include "string.h"
#include "math.h"
#include "APPDEF.H"
#define	CalibrationAddressBase	0x0801F000u						//	标定数据起始地址
#define	AddressCRCAdd	0x0801F100u											//	标定数据校验字地址
#define	AddressCRC		0x35CAu													//	标定数据校验字值

struct	uCalibrateData CalibrateData[CH_Max];

/******************************** 功能说明 *************************************
*	
*******************************************************************************/

//-------------------------------------------------------------
//功能 : 最小二乘法直线拟合 y = a·x + b， 计算系数a 和 b
//参数 : x -- 辐照度的数组
//       y --  功率的数组
//       num 是数组包含的元素个数，x[]和y[]的元素个数必须相等
//       a,b 都是返回值
//返回 : 拟合计算成功返回true, 拟合计算失败返回false
//作者 ：
//-------------------------------------------------------------
static	_Bool LeastSquareLinearFit( struct	uCalibrateData * Data )
{
	float sum_x2 = 0.0f;
	float sum_y  = 0.0f;
	float sum_x  = 0.0f;
	float sum_xy = 0.0f;
	float	XY[2u][6u] = {0};
	const	uint8_t	Num = 6u;
	//	计算最小二乘法拟合直线所需的数据
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
	//	计算A B的值
	Data->Scaling   = ( Num * sum_xy - sum_x * sum_y ) / ( Num * sum_x2 - sum_x * sum_x );

	Data->Intercept = ( sum_x2 * sum_y - sum_x * sum_xy ) / ( Num * sum_x2 - sum_x * sum_x );

	return true;
}


/******************************** 功能说明 *************************************
*	标定数据保存
*******************************************************************************/
_Bool	CalibrationSave( void )
{
//	uint8_t	i;

	return	true;
}
/******************************** 功能说明 *************************************
*	标定数据读取
*******************************************************************************/
void	CalibrationLoad( void )
{

}




/*******************************************************************************
*	
*******************************************************************************/
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
static	uint16_t	ADC_Data[4u][4u];
static	uint16_t	ADC_DataOut[4u][4u];
static	uint16_t	ADC_DataCpoy[4u][4u];
uint16_t	ADC_OUT[4u];
void	ADDataConver(void)
{
	uint8_t	t,i;
	for( t = 0u; t < 16u; t ++ )
	{
		ADC_DataCpoy[t/4u][t%4u] = ADC_Databuf[t];	//	数据分类
	}
	
	for( i = 0u; i < 4u; i ++ )										//	滑动滤波
	{	
		ADC_Data[i][3u] = ADC_Data[i][2u];
		ADC_Data[i][2u] = ADC_Data[i][1u];
		ADC_Data[i][1u] = ADC_Data[i][0u];
		ADC_Data[i][0u] = DataAverage( ADC_DataCpoy[i], 4u );
	}
	
	for( i = 0u; i < 4u; i ++ )										//	滑动滤波
	{	
		ADC_DataOut[i][3u] = ADC_DataOut[i][2u];
		ADC_DataOut[i][2u] = ADC_DataOut[i][1u];
		ADC_DataOut[i][1u] = ADC_DataOut[i][0u];
		ADC_DataOut[i][0u] = (uint16_t)DataAverage( ADC_Data[i], 4u );
		
		ADC_OUT[0u] = DataAverage( ADC_DataOut[0u], 4u );	//求平均
		ADC_OUT[1u] = DataAverage( ADC_DataOut[1u], 4u );
		ADC_OUT[2u] = DataAverage( ADC_DataOut[2u], 4u );
		ADC_OUT[3u] = DataAverage( ADC_DataOut[3u], 4u );

	}
}

/******************************** 功能说明 *************************************
*	浓度值计算
*******************************************************************************/
float	ConcentrationOut( enum	enumLEDChannelSelect	Channel )
{
	//	X轴 吸光度 = log10（去离子水的AD值 / 试样AD值）
	//	Y轴 浓度 = A*吸光度 + B
	float	ValueX = log10( (double)CalibrateData[Channel].WaterAD / ADC_OUT[Channel] );
	float	ValueY = CalibrateData[Channel].Scaling * ValueX + CalibrateData[Channel].Intercept;
	
	return	ValueY;
}
/********  (C) COPYRIGHT 2018 青岛中特环保仪器有限公司  **** End Of File ********/
