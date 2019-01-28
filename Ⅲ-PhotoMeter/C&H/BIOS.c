/**************** (C) COPYRIGHT 2018 青岛中特环保仪器有限公司 *******************
* 文 件 名:  
* 创 建 者:  Kaiser
* 描  述  :  
* 最后修改:  
*********************************** 修订记录 ************************************
* 版  本: 
* 修订人: 
********************************************************************************/
#include "stm32f10x.h"
#include "stdbool.h"
#include "BIOS.H"

/******************************** 功能说明 *************************************
*	位带操作,实现51类似的GPIO控制功能
*******************************************************************************/
///////////////////////////////////////////////////////////////
//具体实现思想,参考<<CM3权威指南>>第五章(87页~92页).
//IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO口地址映射
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    
 
#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 
 
//IO口操作,只对单一的IO口!
//确保n的值小于16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 
 
#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 
 
#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 
 
#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 
 
#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入
 
#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入
 
#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入

/*

	SET_BIT(REG, BIT)     ((REG) |= (BIT))

	CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

	READ_BIT(REG, BIT)    ((REG) & (BIT))

	CLEAR_REG(REG)        ((REG) = (0x0))

	WRITE_REG(REG, VAL)   ((REG) = (VAL))

	READ_REG(REG)         ((REG))

	MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

*/
//	0：(0000B)模拟输入模式
//	4：(0100B)浮空输入模式(复位后的状态)
//	8：(1000B)上拉/下拉输入模式
//	C：(1100B)保留
//	
//	1：(0001B)通用推挽输出模式10MHz
//	2：(0010B)通用推挽输出模式2MHz
//	3：(0011B)通用推挽输出模式50MHz
//	
//	5：(0101B)通用开漏输出模式10MHz
//	6：(0110B)通用开漏输出模式2MHz
//	7：(0111B)通用开漏输出模式50MHz
//	
//	9：(1001B)复用功能推挽输出模式10MHz
//	A：(1010B)复用功能推挽输出模式2MHz
//	B：(1011B)复用功能推挽输出模式50MHz
//	
//	D：(1101B)复用功能开漏输出模式10MHz
//	E：(1110B)复用功能开漏输出模式2MHz
//	F：(1111B)复用功能开漏输出模式50MHz

/********************************** 功能说明 ***********************************
*	短延时程序，用于硬件同步，精确延时需要防止中断的干扰
*******************************************************************************/
#pragma	push
#pragma O3	Ospace
void	delay_us ( uint32_t us )
{
	while ( us-- )
	{
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP();
	}
}
#pragma	pop


/******************************** 功能说明 *************************************
*	ADC转换
*******************************************************************************/
volatile	uint16_t 	ADC_Databuf[16];	//	ADC-DMA转换数据缓冲区
void	BIOS_ADC_Init( void )
{
	SET_BIT( RCC->APB2ENR, RCC_APB2ENR_ADC1EN ); 						// 打开时钟
	ADC_DeInit( ADC1 );
	ADC1->CR2   = ADC_CR2_CONT | ADC_CR2_DMA;	//	连续转换模式 | DMA接收
	ADC1->SQR3 |=( (5u)<< 0u  ) 
						 | ( (6u)<< 5u  ) 
						 | ( (7u)<< 10u ) 
						 | ( (8u)<< 15u )
						 ; //	ADC转换通道选择
	ADC1->SQR1 |=( (4u)- 1 ) << 20u;			//	ADC转换序列总长度
	SET_BIT( ADC1->CR2, ADC_CR2_ADON ); 	//	ADC开关控制
	/*
	通道配置过程
	下面是配置DMA通道x的过程(x代表通道号)：
	1.在DMA_CPARx寄存器中设置外设寄存器的地址。发生外设数据传输请求时，这个地址将是数据传输的源或目标。
	2.在DMA_CMARx寄存器中设置数据存储器的地址。发生外设数据传输请求时，传输的数据将从这个地址读出或写入这个地址。
	3.在DMA_CNDTRx寄存器中设置要传输的数据量。在每个数据传输后，这个数值递减。
	4.在DMA_CCRx寄存器的PL[1:0]位中设置通道的优先级。
	5.在DMA_CCRx寄存器中设置
													数据传输的方向、
													循环模式、
													外设和
													存储器的增量模式、
													外设和
													存储器的数据宽度、
													传输一半产生中断或传输完成产生中断。
	6.设置DMA_CCRx寄存器的ENABLE位，启动该通道。
	一旦启动了DMA通道，它既可响应联到该通道上的外设的DMA请求。
	当传输一半的数据后，半传输标志(HTIF)被置1，当设置了允许半传输中断位(HTIE)时，将产生一个中断请求。在数据传输结束后，传输完成标志(TCIF)被置1，当设置了允许传输完成中断位(TCIE)时，将产生一个中断请求。
	*/
	
	//	ADC的DMA
	DMA_DeInit( DMA1_Channel1 );
	DMA1_Channel1->CPAR = ADC1_BASE + 0x4C/*ADC_DR偏移地址*/;		//	DMA外设地址
	DMA1_Channel1->CMAR = (uint32_t)ADC_Databuf;								//	DMA数据存储器地址
	DMA1_Channel1->CNDTR = 16u;																	
	DMA1_Channel1->CCR = 	DMA_CCR1_PL_1 |			//	设置通道的优先级 较高
												DMA_CCR1_MSIZE_0 |	//	存储器的数据宽度
												DMA_CCR1_PSIZE_0 |	//	外设的数据宽度
												DMA_CCR1_CIRC |			//	循环模式  循环
												DMA_CCR1_MINC ;			//	存储器地址增量模式 增量

	SET_BIT( DMA1_Channel1->CCR, DMA_CCR1_EN );	//	DMA开启
}



/******************************** 功能说明 *************************************
*	按键中断端口配置
*******************************************************************************/

/*
硬件中断选择
通过下面的过程来配置19个线路做为中断源：
●
配置19个中断线的屏蔽位(EXTI_IMR)
●
配置所选中断线的触发选择位(EXTI_RTSR和EXTI_FTSR)；
●
配置那些控制映像到外部中断控制器(EXTI)的NVIC中断通道的使能和屏蔽位，使得19个中断线中的请求可以被正确地响应。
硬件事件选择 PS：这要仔细看中断向量表

通过下面的过程，可以配置19个线路为事件源
●
配置19个事件线的屏蔽位(EXTI_EMR)
●
配置事件线的触发选择位(EXTI_RTSR和EXTI_FTSR)
软件中断/事件的选择
19个线路可以被配置成软件中断/事件线。下面是产生软件中断的过程：
●
配置19个中断/事件线屏蔽位(EXTI_IMR, EXTI_EMR)
●
设置软件中断寄存器的请求位(EXTI_SWIER）

注意: 外部唤醒线是边沿触发的，这些线上不能出现毛刺信号。
在写EXTI_FTSR寄存器时，在外部中断线上的下降沿信号不能被识别，挂起位不会被置位。
在同一中断线上，可以同时设置上升沿和下降沿触发。即任一边沿都可触发中断。

如果要产生中断，必须事先配置好并使能中断线。
根据需要的边沿检测设置2个触发寄存器，同时在中断屏蔽寄存器的相应位写’1’ 允许中断请求。
当外部中断线上发生了需要的边沿时，将产生一个中断请求，对应的挂起位也随之被置’1’。
在挂起寄存器的对应位写’1’，可以清除该中断请求。
*/

/******************************** 功能说明 *************************************
*	中断向量使能配置
*******************************************************************************/
/******************************** 功能说明 *************************************
*	中断端口初始化使能
*******************************************************************************/
void	BIOS_EXTI_Init( void )
{
	//	按键使用浮空输入模式
	SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPBEN ); 						// 打开E端口的时钟
	GPIOB->BSRR = 0x0008u; 																	// E端口的1--6输出为1；
	MODIFY_REG( GPIOB->CRL, 0x0000F000u, 0x00004000u );			//	配置E端口的1--6为浮空输入

//	SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPDEN ); 						// 打开D端口的时钟
//	GPIOD->BSRR = 0x2F00u; 																	// D端口的8--13输出为1；
//	MODIFY_REG( GPIOD->CRH, 0x00FFFFFFu, 0x00444444u );			//	配置D端口的0--6为浮空输入

	SET_BIT( RCC->APB2ENR, RCC_APB2ENR_AFIOEN );	//	打开复用时钟
																								//	开启中断复用 GPIOC的 #1~#6 号线
	AFIO->EXTICR[0] = 														
//					  AFIO_EXTICR1_EXTI1_PC
//					| AFIO_EXTICR1_EXTI2_PC
					 AFIO_EXTICR1_EXTI3_PB
					;
//	AFIO->EXTICR[1] = 
//					  AFIO_EXTICR2_EXTI4_PC
//					| AFIO_EXTICR2_EXTI5_PE
//					| AFIO_EXTICR2_EXTI6_PE
					;
//	AFIO->EXTICR[2] = 
//					  AFIO_EXTICR3_EXTI8_PD
//					| AFIO_EXTICR3_EXTI9_PD
//					| AFIO_EXTICR3_EXTI10_PD
//					| AFIO_EXTICR3_EXTI11_PD
//					;
//	AFIO->EXTICR[3] = 
//					  AFIO_EXTICR4_EXTI12_PD
//					| AFIO_EXTICR4_EXTI13_PD
//					;
	CLEAR_BIT( EXTI->IMR,  0xFFFFu );	// 禁止中断
	CLEAR_BIT( EXTI->EMR,  0x0008u );	// no event 无中断事件
//	SET_BIT  ( EXTI->RTSR, 0x008Fu );	// rising edge trigger  上升沿触发
	SET_BIT  ( EXTI->FTSR, 0x0008u );	// falling edge trigger 下降沿触发
	WRITE_REG( EXTI->PR,   0xFFFFu );	// 通过写1能复位触发的中断标志

	SET_BIT  ( EXTI->IMR,  0x0008u );	// 允许中断 #1~#6 #8~#13。
	// IO中断	
//	NVIC_EnableIRQ( EXTI0_IRQn );
//	NVIC_EnableIRQ( EXTI1_IRQn );
//	NVIC_EnableIRQ( EXTI2_IRQn );
	NVIC_EnableIRQ( EXTI3_IRQn );
//	NVIC_EnableIRQ( EXTI4_IRQn );
//	NVIC_EnableIRQ( EXTI9_5_IRQn );
//	NVIC_EnableIRQ( EXTI15_10_IRQn );
}
/********************************** 功能说明 ***********************************
*	中断开关控制
*******************************************************************************/
void	EXTIx_IRQ_Enable( uint8_t EXTIx )
{
	SET_BIT( EXTI->PR,   0x01u << EXTIx );	// 写1复位中断标志位
	SET_BIT( EXTI->IMR,  0x01u << EXTIx );	// 允许中断。
}

void	EXTIx_IRQ_Disable( uint8_t EXTIx )
{
	CLEAR_BIT( EXTI->IMR,  0x01u << EXTIx );	// 禁止中断 
	SET_BIT( EXTI->PR,   0x01u << EXTIx );	// 写1复位中断标志位
}


/******************************** 功能说明 *************************************
*	
*******************************************************************************/

/*
#define HIGH 1
#define LOW 0
 
#define TRUE 1
#define FALSE 0
 
// 这里使用的是STM32F103RC的片子, 使用PB6, PB7分别作为SCL和SDA, 
其他片子根据需求选用合适的GPIO口 
#define SDA_IN()  {GPIOB->CRL &= 0X0FFFFFFF; GPIOB->CRL |= 4<<28;}
#define SDA_OUT() {GPIOB->CRL &= 0X0FFFFFFF; GPIOB->CRL |= 3<<28;}
#define IIC_SCL    PBout(6) //SDA	 
#define IIC_SDA    PBout(7) //SDA	 
#define READ_SDA   PBin(7)  //输入SDA
 
 
// 初始化I2C 
void IIC_Init(void)
{
	GPIO_InitTypeDef GPIO_Init_Structure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_Init_Structure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init_Structure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init_Structure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_Init_Structure);
	GPIO_SetBits(GPIOB,GPIO_Pin_6|GPIO_Pin_7);
}
 
// 产生I2C的起始信号 
void IIC_Start(void)
{
    // 设置SDA线为输出 
	SDA_OUT();     
    
    // 在开始传输数据之前, 先让SDA和SCL都拉高 
	IIC_SDA = HIGH;	  	  
	IIC_SCL = HIGH;
    
	delay_us(5);
    
    // 根据I2C总线定义: SCL为高时, 数据由高跳变至低表示开始信号 
 	IIC_SDA = LOW;
	delay_us(5);
    
    // 
//    SCL在低的时候, 是不传送任何数据, 也不作为开始和结束的条件, 
//    所以这样我们可以开始数据的发送而不会导致产生开始或者结束信号 
//    这个就是所谓的钳住I2C总线
    
	IIC_SCL = LOW;
}
 
// 产生I2C停止信号 
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
    
    // 
//    先让SCL拉低这样才能将SDA切换至低的状态而不导致重复产生开始信号 
//    还记得前面对I2C Start的注释吗?就是SCL为高的时候, SDA由高变低
    
	IIC_SCL = LOW;
    
    // 
//        前面已经将SCL拉低了, 所以这里我们就可以肆无忌惮的将SDA拉低, 
//        为产生结束信号做好准备
    
	IIC_SDA = LOW;
    
 	delay_us(5);
    
    // Okay, 我们开始产生结束信号: 首先需要将SCL拉高(为什么要拉高? 因为I2C总线规定了
//    只有在SCL为高的时候才是传输数据或者产生开始/结束信号的有效时间) 
	IIC_SCL = HIGH; 
    
    // 好了, 前面已经提前将SDA拉低了, 所以这里我们只要简单的将SDA置为高就完成了 
	IIC_SDA = HIGH;
	delay_us(5);							   	
}
 
//
//应答信号的意义:
//Master每发送完8bit数据后需要等待Slave的ACK
//也就是在第9个Clock, 如果从(Slave)IC发出ACK, 那么SDA会被拉低
//如果没有ACK, SDA会被置高,这样会导致Master发出Restart或者Stop流程

// 
    等待应答信号到来

u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime = 0; 
	IIC_SDA = HIGH;
    delay_us(5);
    
    // 设置SDA为输入模式, 以便读取SDA上的ACK 
	SDA_IN();      
    
    // 置SCL为高, 进行读取操作 
	IIC_SCL = HIGH;
    delay_us(5); 	
    
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime > 250)
		{//指定时间内没有收到从IC的ACK, 进行超时处理
			return FALSE;
		}
	}
    
    // 
//    好了, 这里表示没有超时并且读取到了SDA为低, 表示收到ACK确认了 
//    那么就可以收工回家, 设置一下SCL为低电平(为啥? 因为有效操作都是在
//    SCL为高的时候才进行的)
    
	IIC_SCL = LOW;
    
	return TRUE;  
}
 
//
// 发出ACK确认的操作
// 发出这个操作的意义在于让从IC知道我们已经收到数据了
// 这样, 从IC就不会进行Restart或者Stop流程

void IIC_Ack(void)
{
    // 第一步先让SCL拉低先, 避免SDA变化影响I2C 
	IIC_SCL = LOW;
    
    // 然后我们设置一下SDA为输出模式, 准备输出数据 
	SDA_OUT();
    
    // SDA拉低, 这样就将确认信号放到总线上, 就差时钟信号了 
	IIC_SDA = LOW;
    
	delay_us(5);
    
    // SCL拉高, 产生必备的有效操作时钟信号 
	IIC_SCL = HIGH;
    
	delay_us(5);
    
    // 
//    前面延时了一会了, 时钟差不多了, 那么就结束ACK信号
//    总不能一直占着总线不放吧 
	IIC_SCL = LOW;
}
 
//
//对于NACK, I2C总线是这样定义的:
//当在第9个时钟脉冲的时候SDA线保持高电平, 就被定义为NACK.
//Master要么产生Stop条件来放弃此次传输,要么重复Start条件来
//发起一个新的开始

void IIC_NAck(void)
{
	IIC_SCL = LOW;
	SDA_OUT();
    
    // 根据定义, 拉高SDA, 作为NACK的定义 
	IIC_SDA = HIGH;
	delay_us(5);
    
    // 置为高电平, 发送NACK信号出去 
	IIC_SCL = HIGH;
	delay_us(5);
    
    // SCL拉低, 发送完毕 
	IIC_SCL = LOW;
}
 
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;  
    // 既然开始了数据的发送, 那么先将SDA口切换为输出模式先   
	SDA_OUT(); 	 
 
    // 时钟信号拉低, 将数据准备好再拉高进行传输 
    IIC_SCL = LOW;
    for(t = 0; t < 8; t++)
    {    
        // I2C数据是按照大端传输的, 也就是高位先传输 
		if((txd&0x80) >> 7)
			IIC_SDA = HIGH;
		else
			IIC_SDA = LOW;
        
		txd <<= 1; 
        // 做一下延时是有必要的        
		delay_us(5);   
        
        // SCL拉高传送数据 
		IIC_SCL = HIGH;
        
		delay_us(5);
        
        // 拉低SCL, 传输完毕 
		IIC_SCL = LOW;	
        
		delay_us(5);
    }	 
} 
 
u8 IIC_Read_Byte(void)
{
	unsigned char i, receive = 0;
    
    // 切换SDA口为输入模式, 准备读取数据 
	SDA_IN();
    
    for(i = 0; i < 8; i++ )
	{
        // SCL拉低 
        IIC_SCL = LOW; 
        delay_us(5);
        
        // 再拉高SCL, 产生一个有效的时钟信号 
		IIC_SCL = HIGH;
        
        // 读取总线上的数据 
        receive = (receive << 1) | READ_SDA;
        
		delay_us(5); 
    }
    
	return receive;
}
*/

/******************************** 功能说明 *************************************
*	I2C
*******************************************************************************/
#define	PinBB( _Port, _Num )	(*(__IO int32_t *)(PERIPH_BB_BASE + ((uint32_t)&(_Port) - PERIPH_BASE) * 32u + (_Num) * 4u ))

#define IIC_PORT				GPIOB

#define	Pin_I2C_SCL_In		PinBB( IIC_PORT->IDR, 6u )
#define	Pin_I2C_SCL_Out		PinBB( IIC_PORT->ODR, 6u )
#define	Pin_I2C_SDA_In		PinBB( IIC_PORT->IDR, 7u )
#define	Pin_I2C_SDA_Out		PinBB( IIC_PORT->ODR, 7u )

/******************************** 功能说明 *************************************
*	
*******************************************************************************/
void	BIOS_I2C_Init( void )
{
	/* 端口初始化 */
	SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPBEN ); 								// 
	MODIFY_REG( IIC_PORT->CRL, 0xFF000000u, 0x77000000u );			//	

	GPIO_SetBits(IIC_PORT,GPIO_Pin_6|GPIO_Pin_7);
	
}

/******************************** 功能说明 *************************************
*	I2C 数据线换向配置
*******************************************************************************/
void I2C_SDA_Out(void)
{
	SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPBEN ); 								// 
	MODIFY_REG( IIC_PORT->CRL, 0xFF000000u, 0x77000000u );			//	
}

/******************************** 功能说明 *************************************
*	I2C 数据线换向配置
*******************************************************************************/
void I2C_SDA_In(void)
{
	SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPBEN ); 								// 
	MODIFY_REG( IIC_PORT->CRL, 0xFF000000u, 0x44000000u );			//	
}

/******************************** 功能说明 *************************************
*	I2C 写函数
*******************************************************************************/
_Bool	bus_i2c_shout ( uint8_t cOutByte )
{
	_Bool		AcknowlegeState;
	uint8_t	i;
	I2C_SDA_Out();
	for( i = 8u; i != 0u; --i )
	{
		delay_us( 30 );
		if ( cOutByte & 0x80u )
		{
			Pin_I2C_SDA_Out = 1;
		}
		else
		{
			Pin_I2C_SDA_Out = 0;
		}

		cOutByte <<= 1;
		delay_us( 20 );
		Pin_I2C_SCL_Out = 1;

		delay_us( 50 );
		Pin_I2C_SCL_Out = 0;
	}

	Pin_I2C_SDA_Out = 1;
	I2C_SDA_In();
	delay_us( 50u );
	Pin_I2C_SCL_Out = 1;
	delay_us( 50u );
	AcknowlegeState	= Pin_I2C_SDA_In;
	Pin_I2C_SCL_Out = 0;
	if ( I2C_ACK != AcknowlegeState )
	{
		return	false;
	}
	else
	{
		return	true;
	}
}

/******************************** 功能说明 *************************************
*	I2C 读函数
*******************************************************************************/
uint8_t	bus_i2c_shin( enum I2C_AcknowlegeSet	AcknowlegeSet )
{
	uint8_t		cInByte = 0u;
	uint8_t		i;
	I2C_SDA_Out();
	Pin_I2C_SDA_Out = 1;		// make SDA an input
	I2C_SDA_In();
	for( i = 8u; i != 0u; --i )
	{
		delay_us( 50u );
		Pin_I2C_SCL_Out = 1;

		delay_us( 50u );
		cInByte <<= 1;

		if ( Pin_I2C_SDA_In )
		{
			cInByte |= 0x01u;
		}
		else
		{
			cInByte &= 0xFEu;
		}

		Pin_I2C_SCL_Out = 0;
	}
	I2C_SDA_Out();
	if ( I2C_ACK == AcknowlegeSet )
	{
		Pin_I2C_SDA_Out = 0;
	}
	else
	{
		Pin_I2C_SDA_Out = 1;
	}
	
	delay_us( 50u );
	Pin_I2C_SCL_Out = 1;
	delay_us( 50u );
	Pin_I2C_SCL_Out = 0;

	return	cInByte;
}

/******************************** 功能说明 *************************************
*	I2C 开始函数
*******************************************************************************/
_Bool	bus_i2c_start ( uint8_t Address8Bit, enum I2C_DirectSet DirectSet )
{


	I2C_SDA_Out();
	//	Verify bus available.
	Pin_I2C_SDA_Out = 1;
	Pin_I2C_SCL_Out = 1;
	I2C_SDA_In();
	delay_us( 1000 );
	if( ! Pin_I2C_SDA_In )
	{
		return  false;
	}

	if( ! Pin_I2C_SCL_In )
	{
		return  false;
	}
	I2C_SDA_Out();
	delay_us( 5u );
	Pin_I2C_SDA_Out = 0;
	delay_us( 10u );
	Pin_I2C_SCL_Out = 0;
	delay_us( 10u );
	
	if ( I2C_Write == DirectSet )
	{
		return	bus_i2c_shout( Address8Bit & 0xFEu );
	}
	else
	{
		return	bus_i2c_shout( Address8Bit | 0x01u );
	}
}

/******************************** 功能说明 *************************************
*	I2C 停止函数
*******************************************************************************/
void	bus_i2c_stop ( void )
{
	I2C_SDA_Out();
	Pin_I2C_SDA_Out = 0;
	delay_us( 1000 );
	Pin_I2C_SCL_Out = 1;
	delay_us( 1000 );
	Pin_I2C_SDA_Out = 1;
	delay_us( 1000 );
}
/******************************** 功能说明 *************************************
*	串口帧接收定时
*******************************************************************************/
	void	BIOS_TIM6_TIMER_Init( void )
	{
		SET_BIT( RCC->APB1ENR, RCC_APB1ENR_TIM6EN );													//	打开时钟
		SET_BIT( TIM6->EGR, TIM_EGR_UG );																			//	初始化所有寄存器
		TIM6->CR1   = TIM_CR1_ARPE;																						//	自动装载
		TIM6->PSC		= (uint32_t)( SystemCoreClock / 2u / 10000u - 1u );				//	设置预分频值
		TIM6->ARR   = 10u;																										//	设置预装载值
		CLEAR_BIT( TIM6->DIER, TIM_DIER_UIE );																//	更新中断
		CLEAR_BIT( TIM6->CR1, TIM_CR1_CEN );		
		TIM6->CNT = 0u;
		NVIC_EnableIRQ( TIM6_IRQn );
	}
/******************************** 功能说明 *************************************
*	TIM
*******************************************************************************/
	/********************************** 功能说明 *************************************
	*	机械臂X
	*********************************************************************************/
		/******************************** 功能说明 *************************************
		*	机械臂端口设置 TIM3->CH3
		*******************************************************************************/
			void	BIOS_RBOOT_ARM( void )
			{
				SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPBEN );
				MODIFY_REG( GPIOB->CRL, 0x0000000Fu, 0x0000000Fu );										//	IO口配置		PWM_OUT
				SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPBEN );
				MODIFY_REG( GPIOB->CRH, 0x00000F00u, 0x00000700u );										//	IO口配置		DIR
				SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPBEN );
				MODIFY_REG( GPIOB->CRH, 0x00F00000u, 0x00700000u );										//	IO口配置		EN

				SET_BIT( GPIOB->BSRR, GPIO_BSRR_BS10 );
				SET_BIT( GPIOB->BSRR, GPIO_BSRR_BR13 );

				SET_BIT( RCC->APB1ENR, RCC_APB1ENR_TIM3EN );													//	打开时钟
				SET_BIT( TIM3->EGR, TIM_EGR_UG );																			//	初始化所有寄存器
				TIM3->CR1   = TIM_CR1_ARPE | TIM_CR1_URS;															//	自动装载 仅溢出中断
				TIM3->PSC		= SystemCoreClock / 2u / 100000u - 1u;										//	设置预分频值
				TIM3->ARR   = 10000u;																									//	设置预装载值
				TIM3->CCMR2 = TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE;	//	设置计数极性和工作模式
				TIM3->CCER  = TIM_CCER_CC3E | TIM_CCER_CC3P;													//	使能通道3
				TIM3->CCR3  = 0u;																											//	设置翻转值
				TIM3->DIER  =  TIM_DIER_CC3IE;																				//	使能3号中断
				CLEAR_BIT( TIM3->SR, TIM_SR_UIF | TIM_SR_CC3IF );											//	清除中断标记
				NVIC_EnableIRQ( TIM3_IRQn );
	//			SET_BIT( TIM3->CR1, TIM_CR1_CEN );																		//	使能计数器
			}
		/******************************** 功能说明 *************************************
		*	机械臂PWM控制
		*******************************************************************************/
			void	PWM_CTRL( uint16_t ARRValue, uint16_t CCR3Value )
			{
				TIM3->ARR   =  ARRValue;					//	设置预装载值
				TIM3->CCR3  =  CCR3Value;					//	设置翻转值
			}
		/******************************** 功能说明 *************************************
		*	机械臂PWM开关
		*******************************************************************************/
			void	PWM_CMD( _Bool	CMD )
			{
				if( CMD )
				{
					SET_BIT( TIM3->CCER,TIM_CCER_CC3E );																	//	使能通道2
					SET_BIT( TIM3->CR1, TIM_CR1_CEN   );																	//	使能计数器
				}
				else
				{
					CLEAR_BIT( TIM3->CCER,TIM_CCER_CC3E );																//	失能通道2
					CLEAR_BIT( TIM3->CR1, TIM_CR1_CEN   );																//	失能计数器
					TIM3->CNT = 0u;
				}
			}
		/******************************** 功能说明 *************************************
		*	中断处理函数
		*******************************************************************************/
			volatile	uint32_t	SetPluseCount;			//	设置运行脉冲数
			volatile	uint32_t	PluseCount;					//	实际运行脉冲数
			volatile	_Bool	CountOver	= false;		//	计数器溢出标志
			
			void	TIM3_IRQHandler( void )
			{
				CLEAR_BIT( TIM3->SR, TIM_SR_UIF | TIM_SR_CC3IF );												//	清除中断标记
				if( PluseCount < SetPluseCount )
					PluseCount ++;
				else
				{
					PWM_CMD( false );
					CountOver = true;
					CLEAR_BIT( TIM3->SR, TIM_SR_UIF | TIM_SR_CC3IF );												//	清除中断标记
				}
			}
		/******************************** 功能说明 *************************************
		*	电机方向控制
		*******************************************************************************/
			void	RBOOT_ARM_DIR( _Bool	State )
			{
				if( State )
					SET_BIT( GPIOB->BSRR, GPIO_BSRR_BS10 );
				else
					SET_BIT( GPIOB->BSRR, GPIO_BSRR_BR10 );
			}
		/******************************** 功能说明 *************************************
		*	电机驱动器使能 
		*******************************************************************************/
			void	RBOOT_ARM_EN( _Bool State )
			{
				if( State )
					SET_BIT( GPIOB->BSRR, GPIO_BSRR_BS13 );
				else
					SET_BIT( GPIOB->BSRR, GPIO_BSRR_BR13 );
			}


	/******************************** 功能说明 *************************************
	*	排空电机端口设置 TIM4->CH3
	*******************************************************************************/
		void	BIOS_Air_Bleed_Init( void )
		{
			SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPBEN );
			MODIFY_REG( GPIOB->CRH, 0x0000000Fu, 0x0000000Fu );										//	IO口配置		PWM_OUT
			SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPBEN );                          
			MODIFY_REG( GPIOB->CRH, 0x0000F000u, 0x00007000u );										//	IO口配置		DIR
			SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPBEN );                          
			MODIFY_REG( GPIOB->CRH, 0x0F000000u, 0x07000000u );										//	IO口配置		EN

			SET_BIT( GPIOB->BSRR, GPIO_BSRR_BS11 );
			SET_BIT( GPIOB->BSRR, GPIO_BSRR_BR14 );
			
			SET_BIT( RCC->APB1ENR, RCC_APB1ENR_TIM4EN );													//	打开时钟
			SET_BIT( TIM4->EGR, TIM_EGR_UG );																			//	初始化所有寄存器
			TIM4->CR1   = TIM_CR1_ARPE | TIM_CR1_URS;															//	自动装载 仅溢出中断
			TIM4->PSC		= SystemCoreClock / 2u / 100000u - 1u;										//	设置预分频值
			TIM4->ARR   = 10000u;																									//	设置预装载值
			TIM4->CCMR2 = TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE;	//	设置计数极性和工作模式
			TIM4->CCER  = TIM_CCER_CC3E |TIM_CCER_CC3P;														//	使能通道2
			TIM4->CCR3  = 0u;																											//	设置翻转值
//			TIM4->DIER  = TIM_DIER_CC3IE;																					//	使能3号中断
//			CLEAR_BIT( TIM4->SR, TIM_SR_UIF | TIM_SR_CC3IF );											//	清除中断标记
//			NVIC_EnableIRQ( TIM4_IRQn );
//			SET_BIT( TIM4->CR1, TIM_CR1_CEN );																		//	使能计数器

		}
	/******************************** 功能说明 *************************************
	*	排空电机PWM控制
	*******************************************************************************/
		void	Air_Bleed_CTRL( uint16_t ARRValue, uint16_t CCR3Value )
		{
			TIM4->ARR   =  ARRValue;					//	设置预装载值
			TIM4->CCR3  =  CCR3Value;					//	设置翻转值
		}
	/******************************** 功能说明 *************************************
	*	排空电机PWM开关
	*******************************************************************************/
		void	Air_Bleed_CMD( _Bool	CMD )
		{
			if( CMD )
			{
				SET_BIT( TIM4->CCER,TIM_CCER_CC3E );																	//	使能通道2
				SET_BIT( TIM4->CR1, TIM_CR1_CEN   );																	//	使能计数器
			} 
			else
			{
				CLEAR_BIT( TIM4->CCER,TIM_CCER_CC3E );																//	失能通道2
				CLEAR_BIT( TIM4->CR1, TIM_CR1_CEN   );																//	失能计数器
				TIM4->CNT = 0u;
			}
		}
	/******************************** 功能说明 *************************************
	*	中断处理函数
	*******************************************************************************/
/*
		void	TIM4_IRQHandler( void )
		{
			CLEAR_BIT( TIM4->SR, TIM_SR_UIF | TIM_SR_CC3IF );												//	清除中断标记
			if( PluseCount[1u] < SetPluseCount[1u] )
				PluseCount[1u] ++;
			else
			{
				PWM_CMD_Y( false );
				CountOver[1u] = true;
				CLEAR_BIT( TIM4->SR, TIM_SR_UIF | TIM_SR_CC3IF );												//	清除中断标记
			}
		}
*/
		/******************************** 功能说明 *************************************
	*	电机方向控制
	*******************************************************************************/
		void	Air_Bleed_DIR( _Bool	State )
		{
			if( State )
				SET_BIT( GPIOB->BSRR, GPIO_BSRR_BS11 );
			else
				SET_BIT( GPIOB->BSRR, GPIO_BSRR_BR11 );
		}
	/******************************** 功能说明 *************************************
	*	电机驱动器使能 
	*******************************************************************************/
		void	Air_Bleed_EN( _Bool State )
		{
			if( State )
				SET_BIT( GPIOB->BSRR, GPIO_BSRR_BS14 );
			else
				SET_BIT( GPIOB->BSRR, GPIO_BSRR_BR14 );
		}
		
	/******************************** 功能说明 *************************************
	*	搅拌电机端口设置 TIM5->CH1
	*******************************************************************************/
		void	BIOS_Stirrer_Init( void )
		{
			SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPAEN );
			MODIFY_REG( GPIOA->CRL, 0x0000000Fu, 0x0000000Bu );										//	IO口配置		PWM_OUT
//			SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPBEN );                          
//			MODIFY_REG( GPIOB->CRH, 0x000F0000u, 0x00070000u );										//	IO口配置		DIR
//			SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPBEN );                          
//			MODIFY_REG( GPIOB->CRH, 0x0F000000u, 0x07000000u );										//	IO口配置		EN

//			SET_BIT( GPIOB->BSRR, GPIO_BSRR_BS12 );
//			SET_BIT( GPIOB->BSRR, GPIO_BSRR_BR15);
			
			SET_BIT( RCC->APB1ENR, RCC_APB1ENR_TIM5EN );													//	打开时钟
			SET_BIT( TIM5->EGR, TIM_EGR_UG );																			//	初始化所有寄存器
			TIM5->CR1   = TIM_CR1_ARPE | TIM_CR1_URS;															//	自动装载 仅溢出中断
			TIM5->PSC		= SystemCoreClock / 100000u - 1u;										//	设置预分频值
			TIM5->ARR   = 10000u-1;																									//	设置预装载值
			TIM5->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE;	//	设置计数极性和工作模式
			TIM5->CCER  = TIM_CCER_CC1E |TIM_CCER_CC1P;														//	使能通道1
			TIM5->CCR1  = 10000u;																											//	设置翻转值
				SET_BIT( TIM5->CCER,TIM_CCER_CC1E );																	//	使能通道1
				SET_BIT( TIM5->CR1, TIM_CR1_CEN   );																	//	使能计数器
//			TIM5->DIER  = TIM_DIER_CC1IE;																					//	使能1号中断
//			CLEAR_BIT( TIM5->SR, TIM_SR_UIF | TIM_SR_CC1IF );											//	清除中断标记
//			NVIC_EnableIRQ( TIM5_IRQn );
//			SET_BIT( TIM5->CR1, TIM_CR1_CEN );																		//	使能计数器
		}
	/******************************** 功能说明 *************************************
	*	搅拌电机PWM控制
	*******************************************************************************/
		void	Stirrer_CTRL( uint16_t ARRValue, uint16_t CCR1Value )
		{
//			TIM5->ARR   =  ARRValue;					//	设置预装载值
			TIM5->CCR1  =  CCR1Value;					//	设置翻转值
		}
	/******************************** 功能说明 *************************************
	*	搅拌电机PWM开关
	*******************************************************************************/
		void	Stirrer_CMD( _Bool	CMD )
		{
			if( CMD )
			{
				SET_BIT( TIM5->CCER,TIM_CCER_CC1E );																	//	使能通道1
				SET_BIT( TIM5->CR1, TIM_CR1_CEN   );																	//	使能计数器
			}
			else
			{
				CLEAR_BIT( TIM5->CCER,TIM_CCER_CC1E );																//	失能通道1
				CLEAR_BIT( TIM5->CR1, TIM_CR1_CEN   );																//	失能计数器
				TIM5->CNT = 0u;
			}
		}
	/******************************** 功能说明 *************************************
	*	中断处理函数
	*******************************************************************************/
/*
		void	TIM5_IRQHandler( void )
		{
			CLEAR_BIT( TIM5->SR, TIM_SR_UIF | TIM_SR_CC1IF );												//	清除中断标记
			if( PluseCount[2u] < SetPluseCount[2u] )
				PluseCount[2u] ++;
			else
			{
				PWM_CMD_Z( false );
				CountOver[2u] = true;
				CLEAR_BIT( TIM5->SR, TIM_SR_UIF | TIM_SR_CC1IF );												//	清除中断标记
			}
		}
*/
	/******************************** 功能说明 *************************************
	*	电机方向控制
	*******************************************************************************/
/*
void	Stirrer_DIR( _Bool	State )
		{
			if( State )
				SET_BIT( GPIOB->BSRR, GPIO_BSRR_BS12 );
			else
				SET_BIT( GPIOB->BSRR, GPIO_BSRR_BR12 );
		}
*/
/******************************** 功能说明 *************************************
	*	电机驱动器使能 
	*******************************************************************************/
/*
	void	Stirrer_EN( _Bool State )
		{
			if( State )
				SET_BIT( GPIOB->BSRR, GPIO_BSRR_BS15 );
			else
				SET_BIT( GPIOB->BSRR, GPIO_BSRR_BR15 );
		}
*/		

	/********************************** 功能说明 *************************************
  *	机械手指端口初始化 TIM8->CH4
  *********************************************************************************/
/*
  	void	BIOS_Fingers_Init( void )
		{
			SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPCEN );
			MODIFY_REG( GPIOC->CRH, 0x000000F0u, 0x000000F0u );										//	IO口配置		PWM_OUT
//			SET_BIT( GPIOC->BSRR, GPIO_BSRR_BS9 );
			SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPAEN );
			MODIFY_REG( GPIOA->CRH, 0x0000000Fu, 0x0000000Fu );										//	IO口配置		Fingers_DIR(Cmd)

			SET_BIT( RCC->APB2ENR, RCC_APB2ENR_TIM8EN );													//	打开时钟
			SET_BIT( TIM8->EGR, TIM_EGR_UG );																			//	初始化所有寄存器
			TIM8->CR1   = TIM_CR1_ARPE | TIM_CR1_URS;															//	自动装载 仅溢出中断
			TIM8->PSC		= SystemCoreClock / 100000u - 1u;													//	设置预分频值
			TIM8->ARR   = 50u;																										//	设置预装载值
			TIM8->CCMR2 = TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE;	//	设置计数极性和工作模式
			TIM8->CCER  = TIM_CCER_CC4E |TIM_CCER_CC4P;														//	使能通道4
			TIM8->CCR1  = 0u;																											//	设置翻转值
			TIM8->DIER  = TIM_DIER_CC1IE;																					//	使能1号中断
			CLEAR_BIT( TIM8->SR, TIM_SR_UIF | TIM_SR_CC4IF );											//	清除中断标记
			SET_BIT( TIM8->BDTR, TIM_BDTR_MOE );
			SET_BIT( TIM8->CR1, TIM_CR1_CEN );																		//	使能计数器
		}
*/	
  /******************************** 功能说明 *************************************
  *	机械手指PWM控制
  *******************************************************************************/
/*
		void	Fingers_CTRL( uint16_t ARRValue, uint16_t CCR1Value )
		{
			TIM8->ARR   =  ARRValue;					//	设置预装载值
			TIM8->CCR4  =  CCR1Value;					//	设置翻转值
		}
*/	
  /******************************** 功能说明 *************************************
  *	机械手指PWM开关
  *******************************************************************************/
 /*
		void	Fingers_CMD( _Bool	CMD )
		{
			if( CMD )
			{
				SET_BIT( TIM8->CCER,TIM_CCER_CC4E );																	//	使能通道1
				SET_BIT( TIM8->CR1, TIM_CR1_CEN   );																	//	使能计数器
			}
			else
			{
				CLEAR_BIT( TIM8->CCER,TIM_CCER_CC4E );																//	失能通道1
				CLEAR_BIT( TIM8->CR1, TIM_CR1_CEN   );																//	失能计数器
				TIM5->CNT = 0u;
			}
		}
*/
	/******************************** 功能说明 *************************************
	*	中断处理函数
	*******************************************************************************/
/*
		volatile	uint32_t	SetPluseCount;			//	设置运行脉冲数
		volatile	uint32_t	PluseCount;					//	实际运行脉冲数
		volatile	_Bool	CountOver	= false;			//	计数器溢出标志
		void	TIM8_IRQHandler( void )
		{
			CLEAR_BIT( TIM8->SR, TIM_SR_UIF | TIM_SR_CC4IF );												//	清除中断标记
			if( PluseCount < SetPluseCount )
				PluseCount ++;
			else
			{
				Fingers_CMD( false );
				CountOver = true;
				CLEAR_BIT( TIM8->SR, TIM_SR_UIF | TIM_SR_CC4IF );												//	清除中断标记
			}
		}
*/		
	/******************************** 功能说明 *************************************
	*	电机方向控制 机械手指运动方向 0 开 1 关
	*******************************************************************************/
/*
		void	Fingers_DIR( _Bool	State )
		{
			if( State )
				SET_BIT( GPIOA->BSRR, GPIO_BSRR_BS12 );
			else
				SET_BIT( GPIOA->BSRR, GPIO_BSRR_BR12 );
		}
*/	
  /******************************** 功能说明 *************************************
  *	机械手指脉冲计数
  *******************************************************************************/
/*
  	void	BIOS_Fingers_PWMIn_Init( void )
		{
			SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPAEN );
			MODIFY_REG( GPIOA->CRH, 0x000F0000u, 0x00040000u );										//	IO口配置		PWM_OUT
			SET_BIT( RCC->APB2ENR, RCC_APB2ENR_TIM1EN );													//	打开时钟
			SET_BIT( TIM1->EGR, TIM_EGR_UG );																			//	初始化所有寄存器
			TIM1->SMCR =  TIM_SMCR_ECE;	//	设置工作模式 外部时钟2模式
			SET_BIT( TIM1->BDTR, TIM_BDTR_MOE );
			SET_BIT( TIM1->CR1, TIM_CR1_CEN );																		//	使能计数器
		}
*/
	/********************************** 功能说明 *************************************
  *	搅拌机PWM控制
  *********************************************************************************/
/*
		void	Blender_Control( uint16_t	Value )
		{
//			if( State )
//				SET_BIT( GPIOC->BSRR, GPIO_BSRR_BR6 );
//			else
//				SET_BIT( GPIOC->BSRR, GPIO_BSRR_BS6 );
			static	uint16_t	LastValue;
			if( Value != LastValue)
			{
				if( Value ) 
				{
					TIM8->ARR  = Value;																										//	设置预装载值
					TIM8->CCR1 = Value / 2;			
					SET_BIT( TIM8->CR1, TIM_CR1_CEN );																		//	使能计数器
				}
				else
				{
					CLEAR_BIT( TIM8->CR1, TIM_CR1_CEN );																		//	使能计数器
				
				}
				LastValue = Value;
			}
		}
*/
	/********************************** 功能说明 *************************************
  *	搅拌器PWM 端口初始化
  *********************************************************************************/
/*
		void	BIOS_Motor_Init( void )
		{
			SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPAEN );
			MODIFY_REG( GPIOA->CRH, 0x0000000Fu, 0x0000000Fu );										//	IO口配置		PWM_OUT
//			SET_BIT( GPIOC->BSRR, GPIO_BSRR_BS6 );
			SET_BIT( RCC->APB2ENR, RCC_APB2ENR_TIM1EN );													//	打开时钟
			SET_BIT( TIM1->EGR, TIM_EGR_UG );																			//	初始化所有寄存器
			TIM1->CR1   = TIM_CR1_ARPE | TIM_CR1_URS;															//	自动装载 仅溢出中断
			TIM1->PSC		= SystemCoreClock / 100000u - 1u;													//	设置预分频值
			TIM1->ARR   = 30u - 1u;																							//	设置预装载值
			TIM1->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE;	//	设置计数极性和工作模式
			TIM1->CCER  = TIM_CCER_CC1E |TIM_CCER_CC1P;														//	使能通道1
			TIM1->CCR1  = 0u;																											//	设置翻转值
//			TIM8->DIER  = TIM_DIER_CC1IE;																					//	使能1号中断
//			CLEAR_BIT( TIM8->SR, TIM_SR_UIF | TIM_SR_CC1IF );											//	清除中断标记
			SET_BIT( TIM1->BDTR, TIM_BDTR_MOE );
			SET_BIT( TIM1->CR1, TIM_CR1_CEN );																		//	使能计数器

		}
*/
	/********************************** 功能说明 *************************************
  *	搅拌机PWM控制
  *********************************************************************************/
/*
void	EMotor_Control( uint16_t	Value )
		{
//			if( State )
//				SET_BIT( GPIOC->BSRR, GPIO_BSRR_BR6 );
//			else
//				SET_BIT( GPIOC->BSRR, GPIO_BSRR_BS6 );
			static	uint16_t	LastValue;
			if( Value != LastValue)
			{
				if( Value ) 
				{
					TIM1->ARR  = Value;																										//	设置预装载值
					TIM1->CCR1 = Value / 2;			
					SET_BIT( TIM1->CR1, TIM_CR1_CEN );																		//	使能计数器
				}
				else
				{
					CLEAR_BIT( TIM1->CR1, TIM_CR1_CEN );																		//	使能计数器
				
				}
				LastValue = Value;
			}
		
//			if( State  )
//				TIM1->CCR1 = 15;
//			else
//				TIM1->CCR1 = 0;
		}
*/	
/******************************** 功能说明 *************************************
*	串口配置
*******************************************************************************/
	/******************************** 功能说明 *************************************
	*	串口1端口配置
	*******************************************************************************/
		void	BIOS_USART1_Init( void )
		{
			USART_TypeDef * USARTx = USART1;

			SET_BIT( RCC->APB2ENR, RCC_APB2ENR_USART1EN );

			// USART1 configured as follow:
			// 	- BaudRate = 9600 baud
			// 	- Word Length = 8 Bits
			// 	- One Stop Bit
			// 	- No parity
			// 	- Hardware flow control disabled (RTS and CTS signals)
			// 	- Receive disable and transmit enabled
			USARTx->BRR = SystemCoreClock / 115200u;	/* 9600 bps							*/
			USARTx->CR1 = 0x0000u;					/* 1 start bit, 8 data bits         */
			USARTx->CR2 = 0x0000u;					/* 1 stop bit                       */
			USARTx->CR3 = 0x0000u; 					/* no flow control                  */
			SET_BIT( USARTx->CR1, USART_CR1_TE );	/* enable TX 						*/
			SET_BIT( USARTx->CR1, USART_CR1_RE );	/* enable RX 						*/
			SET_BIT( USARTx->CR1, USART_CR1_UE );	/* Enable USARTx					*/
			USART_ITConfig( USART1, /*USART_IT_TXE |*/ USART_IT_RXNE, ENABLE );

			/* Configure USART1 Rx (PA10) as input floating */
			/* Configure USART1 Tx (PA9) as alternate function push-pull */
			SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPAEN );
			MODIFY_REG( GPIOA->CRH, 0x00000FF0u, 0x000004B0u );
			NVIC_EnableIRQ( USART1_IRQn );
		}
	/******************************** 功能说明 *************************************
	*	访问 USART1，实现RS232收发
	*******************************************************************************/
		void	UART1_Send( uint8_t OutByte )
		{
			USART_TypeDef * USARTx = USART1;

			while ( ! ( READ_BIT( USARTx->SR, USART_SR_TXE )))
			{
				;
			}

			USARTx->DR = OutByte;
		}
	/******************************** 功能说明 *************************************
	*	访问 USART1，实现RS232收发
	*******************************************************************************/
		uint8_t	UART1_Received( void )
		{
			USART_TypeDef * USARTx = USART1;

			while ( ! ( READ_BIT( USARTx->SR, USART_SR_RXNE )))
			{
				;
			}

			return	USARTx->DR;
		}
	
	/******************************** 功能说明 *************************************
	*	串口2端口配置
	*******************************************************************************/
		void	BIOS_USART2_Init( void )
		{
			USART_TypeDef * USARTx = USART2;

			SET_BIT( RCC->APB1ENR, RCC_APB1ENR_USART2EN );

			// USART1 configured as follow:
			// 	- BaudRate = 9600 baud
			// 	- Word Length = 8 Bits
			// 	- One Stop Bit
			// 	- No parity
			// 	- Hardware flow control disabled (RTS and CTS signals)
			// 	- Receive disable and transmit enabled
			USARTx->BRR = SystemCoreClock / 2 / 9600u;	/* 9600 bps							*/
			USARTx->CR1 = 0x0000u;					/* 1 start bit, 8 data bits         */
			USARTx->CR2 = 0x0000u;					/* 1 stop bit                       */
			USARTx->CR3 = 0x0000u; 					/* no flow control                  */
			SET_BIT( USARTx->CR1, USART_CR1_TE );	/* enable TX 						*/
			SET_BIT( USARTx->CR1, USART_CR1_RE );	/* enable RX 						*/
			SET_BIT( USARTx->CR1, USART_CR1_UE );	/* Enable USARTx					*/
			USART_ITConfig( USARTx, /*USART_IT_TXE |*/ USART_IT_RXNE, ENABLE );

			/* Configure USART1 Rx  as input floating */
			/* Configure USART1 Tx  as alternate function push-pull */
			SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPAEN );
			MODIFY_REG( GPIOA->CRL, 0x0000FF00u, 0x00004B00u );
			MODIFY_REG( GPIOA->CRL, 0x000F0000u, 0x00030000u );
			NVIC_EnableIRQ( USART2_IRQn );
			USART_GetFlagStatus(USART2,USART_FLAG_TC);
		}
	/******************************** 功能说明 *************************************
	*	访问 USART2，实现RS232收发
	*******************************************************************************/
		void	UART2_Send( uint8_t OutByte )
		{
			USART_TypeDef * USARTx = USART2;

			while ( ! ( READ_BIT( USARTx->SR, USART_SR_TXE )))
			{
				;
			}

			USARTx->DR = OutByte;
		}
	/******************************** 功能说明 *************************************
	*	访问 USART2，实现RS232收发
	*******************************************************************************/
		uint8_t	UART2_Received( void )
		{
			USART_TypeDef * USARTx = USART2;

//			while ( ! ( READ_BIT( USARTx->SR, USART_SR_RXNE )))
//			{
//				;
//			}

			return	USARTx->DR;
		}
	
  /******************************** 功能说明 *************************************
  *	485换向
  *******************************************************************************/
  void		RS485_REDE2( _Bool state )
	{
		if( state )
			SET_BIT( GPIOA->BSRR, GPIO_BSRR_BS4 );
		else
			SET_BIT( GPIOA->BSRR, GPIO_BSRR_BR4 );
	}
	
	/******************************** 功能说明 *************************************
	*	串口3端口配置
	*******************************************************************************/
		void	BIOS_USART3_Init( void )
		{
			USART_TypeDef * USARTx = USART3;

			SET_BIT( RCC->APB1ENR, RCC_APB1ENR_USART3EN );

			// USART1 configured as follow:
			// 	- BaudRate = 9600 baud
			// 	- Word Length = 8 Bits
			// 	- One Stop Bit
			// 	- No parity
			// 	- Hardware flow control disabled (RTS and CTS signals)
			// 	- Receive disable and transmit enabled
			USARTx->BRR = SystemCoreClock / 9600u / 2u;	/* 115200 bps							*/
			USARTx->CR1 = 0x0000u;					/* 1 start bit, 8 data bits         */
			USARTx->CR2 = 0x0000u;					/* 1 stop bit                       */
			USARTx->CR3 = 0x0000u; 					/* no flow control                  */
			SET_BIT( USARTx->CR1, USART_CR1_TE );	/* enable TX 						*/
			SET_BIT( USARTx->CR1, USART_CR1_RE );	/* enable TX 						*/
			SET_BIT( USARTx->CR1, USART_CR1_UE );	/* Enable USARTx					*/
			USART_ITConfig( USART3, /*USART_IT_TXE |*/ USART_IT_RXNE, ENABLE );

			/* Configure USART1 Rx (PA10) as input floating */
			/* Configure USART1 Tx (PA9) as alternate function push-pull */
			SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPBEN );
			MODIFY_REG( GPIOB->CRH, 0x0000FF00u, 0x00004B00u );
		}	
	/******************************** 功能说明 *************************************
	*	访问 USART3，实现RS232收发
	*******************************************************************************/
		void	UART3_Send( uint8_t OutByte )
		{
			USART_TypeDef * USARTx = USART3;

			while ( ! ( READ_BIT( USARTx->SR, USART_SR_TXE )))
			{
				;
			}

			USARTx->DR = OutByte;
		}
	/******************************** 功能说明 *************************************
	*	访问 USART3，实现RS232收发
	*******************************************************************************/
		uint8_t	UART3_Received( void )
		{
			USART_TypeDef * USARTx = USART3;

			while ( ! ( READ_BIT( USARTx->SR, USART_SR_RXNE )))
			{
				;
			}

			return	USARTx->DR;
		}
	
	/******************************** 功能说明 *************************************
	*	串口4端口配置
	*******************************************************************************/
		void	BIOS_USART4_Init( void )
		{
			USART_TypeDef * USARTx = UART4;

			SET_BIT( RCC->APB2ENR, RCC_APB1ENR_UART4EN );

			// USART1 configured as follow:
			// 	- BaudRate = 9600 baud
			// 	- Word Length = 8 Bits
			// 	- One Stop Bit
			// 	- No parity
			// 	- Hardware flow control disabled (RTS and CTS signals)
			// 	- Receive disable and transmit enabled
			USARTx->BRR = SystemCoreClock / 9600u * 2u;	/* 9600 bps							*/
			USARTx->CR1 = 0x0000u;					/* 1 start bit, 8 data bits         */
			USARTx->CR2 = 0x0000u;					/* 1 stop bit                       */
			USARTx->CR3 = 0x0000u; 					/* no flow control                  */
			SET_BIT( USARTx->CR1, USART_CR1_TE );	/* enable TX 						*/
			SET_BIT( USARTx->CR1, USART_CR1_RE );	/* enable TX 						*/
			SET_BIT( USARTx->CR1, USART_CR1_UE );	/* Enable USARTx					*/
			USART_ITConfig( USART1, /*USART_IT_TXE |*/ USART_IT_RXNE, ENABLE );

			/* Configure USART1 Rx (PA10) as input floating */
			/* Configure USART1 Tx (PA9) as alternate function push-pull */
			SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPCEN );
			MODIFY_REG( GPIOC->CRH, 0x0000FF00u, 0x00004B00u );
		}
		
	/******************************** 功能说明 *************************************
	*	串口5端口配置
	*******************************************************************************/
		void	BIOS_USART5_Init( void )
		{
			USART_TypeDef * USARTx = UART5;

			SET_BIT( RCC->APB2ENR, RCC_APB1ENR_UART5EN );

			// USART1 configured as follow:
			// 	- BaudRate = 9600 baud
			// 	- Word Length = 8 Bits
			// 	- One Stop Bit
			// 	- No parity
			// 	- Hardware flow control disabled (RTS and CTS signals)
			// 	- Receive disable and transmit enabled
			USARTx->BRR = SystemCoreClock / 9600u * 2u;	/* 9600 bps							*/
			USARTx->CR1 = 0x0000u;					/* 1 start bit, 8 data bits         */
			USARTx->CR2 = 0x0000u;					/* 1 stop bit                       */
			USARTx->CR3 = 0x0000u; 					/* no flow control                  */
			SET_BIT( USARTx->CR1, USART_CR1_TE );	/* enable TX 						*/
			SET_BIT( USARTx->CR1, USART_CR1_RE );	/* enable TX 						*/
			SET_BIT( USARTx->CR1, USART_CR1_UE );	/* Enable USARTx					*/
			USART_ITConfig( USART1, /*USART_IT_TXE |*/ USART_IT_RXNE, ENABLE );

			/* Configure USART1 Rx (PA10) as input floating */
			/* Configure USART1 Tx (PA9) as alternate function push-pull */
			SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPCEN );
			MODIFY_REG( GPIOC->CRH, 0x000F0000u, 0x000B0000u );
			SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPDEN );
			MODIFY_REG( GPIOD->CRL, 0x00000F00u, 0x00000B00u );
		}
	




/******************************** 功能说明 *************************************
*	IO控制
*******************************************************************************/
	/******************************** 功能说明 *************************************
	*	电磁阀控制端口初始化
	*******************************************************************************/
		void	BIOS_SOLENOIDVALVE_Init( void )
		{
			SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPCEN );
			SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPDEN );
			MODIFY_REG( GPIOC->CRL, 0xFF000000u, 0x33000000u );
			MODIFY_REG( GPIOC->CRH, 0x000000FFu, 0x00000033u );
			MODIFY_REG( GPIOC->CRH, 0x000FFF00u, 0x00033300u );
			MODIFY_REG( GPIOD->CRL, 0x00000F00u, 0x00000300u );
			
			SET_BIT( GPIOC->BSRR, GPIO_BSRR_BS6 );
			SET_BIT( GPIOC->BSRR, GPIO_BSRR_BS7 );
			SET_BIT( GPIOC->BSRR, GPIO_BSRR_BS8);
			SET_BIT( GPIOC->BSRR, GPIO_BSRR_BS9);

		}
		
  /******************************** 功能说明 *************************************
	*	三通阀1
  *******************************************************************************/  
		void	SolenoidValve1_Cmd( _Bool	Newstate )
		{
			static	_Bool	state = false;
			if( state != Newstate )
			{
				state = Newstate;
				if( state )
				{
					SET_BIT( GPIOC->BSRR, GPIO_BSRR_BR6 );
				}
				else
				{
					SET_BIT( GPIOC->BSRR, GPIO_BSRR_BS6 );
				}
			}		
		}	
	
  /******************************** 功能说明 *************************************
  *	三通阀2
  *******************************************************************************/
		void	SolenoidValve2_Cmd( _Bool	Newstate )
		{
			static	_Bool	state = false;
			if( state != Newstate )
			{
				state = Newstate;
				if( state )
				{
					SET_BIT( GPIOC->BSRR, GPIO_BSRR_BR7 );
				}
				else
				{
					SET_BIT( GPIOC->BSRR, GPIO_BSRR_BS7 );
				}
			}		
		}	
	
  /******************************** 功能说明 *************************************
  *	三通阀3
  *******************************************************************************/
		void	SolenoidValve3_Cmd( _Bool	Newstate )
		{
			static	_Bool	state = false;
			if( state != Newstate )
			{
				state = Newstate;
				if( state )
				{
					SET_BIT( GPIOC->BSRR, GPIO_BSRR_BR8 );
				}
				else
				{
					SET_BIT( GPIOC->BSRR, GPIO_BSRR_BS8 );
				}
			}		
		}	
	
  /******************************** 功能说明 *************************************
  *	三通阀4
  *******************************************************************************/
		void	SolenoidValve4_Cmd( _Bool	Newstate )
		{
			static	_Bool	state = false;
			if( state != Newstate )
			{
				state = Newstate;
				if( state )
				{
					SET_BIT( GPIOC->BSRR, GPIO_BSRR_BR9 );
				}
				else
				{
					SET_BIT( GPIOC->BSRR, GPIO_BSRR_BS9 );
				}
			}		
		}	


/******************************** 功能说明 *************************************
*	IO输入检测
*******************************************************************************/
	/******************************** 功能说明 *************************************
	*	液位检测配置
	*******************************************************************************/
		void	LEVELSWITCH_Init( void )
		{
			SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPCEN );
			MODIFY_REG( GPIOC->CRL, 0x000FFFFFu, 0x00044444u );										//	IO口配置					
		}
	
	/******************************** 功能说明 *************************************
	*	液位检测1
	*******************************************************************************/
		_Bool	LevelSwitch1Read( void )
		{		
			return	READ_BIT( GPIOC->IDR, GPIO_IDR_IDR0 );
		}
	/******************************** 功能说明 *************************************
	*	液位检测2
	*******************************************************************************/
		_Bool	LevelSwitch2Read( void )
		{		
			return	READ_BIT( GPIOC->IDR, GPIO_IDR_IDR1 );
		}
	/******************************** 功能说明 *************************************
	*	液位检测3
	*******************************************************************************/
		_Bool	LevelSwitch3Read( void )
		{		
			return	READ_BIT( GPIOC->IDR, GPIO_IDR_IDR2 );
		}
	/******************************** 功能说明 *************************************
	*	液位检测4
	*******************************************************************************/
		_Bool	LevelSwitch4Read( void )
		{		
			return	READ_BIT( GPIOC->IDR, GPIO_IDR_IDR3 );
		}
	/******************************** 功能说明 *************************************
	*	液位检测5
	*******************************************************************************/
		_Bool	LevelSwitch5Read( void )
		{		
			return	READ_BIT( GPIOC->IDR, GPIO_IDR_IDR4 );
		}
		
/******************************** 功能说明 *************************************
*	调试接口配置
*******************************************************************************/
	void	BIOS_SWD_Init( void )
	{
		SET_BIT( RCC->APB2ENR, RCC_APB2ENR_AFIOEN );
		MODIFY_REG( AFIO->MAPR, AFIO_MAPR_SWJ_CFG, AFIO_MAPR_SWJ_CFG_JTAGDISABLE );
	}

/******************************** 功能说明 *************************************
*	系统所有异常配置上电优先集中处理
*******************************************************************************/
	void	BIOS_NVIC_Init( void )
	{
		NVIC_SetPriorityGrouping( 7 );	//	本系统设置为禁止中断抢占，即没有中断嵌套 见下图
		
		/*
			表7.4 抢占优先级和子优先级的表达，位数与分组位置的关系
		
			分组位置    表达抢占优先级的位段    表达子优先级的位段
				0            [7:1]                 [0:0]
				1            [7:2]                 [1:0]
				2            [7:3]                 [2:0]
				3            [7:4]                 [3:0]
				4            [7:5]                 [4:0]
				5            [7:6]                 [5:0]
				6            [7:7]                 [6:0]
				7            无                    [7:0]（所有位）
		*/
		
		//	__set_CONTROL( 0x03u );		//	切换到PSP, 并转入非特权模式
	}


/******************************** 功能说明 *************************************
*	
*******************************************************************************/
	void	BIOS_Init( void )
	{
		BIOS_SWD_Init();
		BIOS_NVIC_Init();
		BIOS_EXTI_Init();
		BIOS_Stirrer_Init();
		BIOS_RBOOT_ARM();
		BIOS_Air_Bleed_Init();
		BIOS_USART2_Init();
		BIOS_TIM6_TIMER_Init();
		LEVELSWITCH_Init();
		BIOS_SOLENOIDVALVE_Init();

	}

/********  (C) COPYRIGHT 2018 青岛中特环保仪器有限公司  **** End Of File ********/
