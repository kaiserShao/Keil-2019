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
//	//	按键使用浮空输入模式
	SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPBEN ); 						// 打开E端口的时钟
	GPIOB->BSRR |= 0x0038u; 																// 端口输出为1；
	MODIFY_REG( GPIOB->CRL, 0x00FFF000u, 0x00444000u );			//	配置端口为浮空输入

//	SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPDEN ); 						// 打开D端口的时钟
//	GPIOD->BSRR = 0x2F00u; 																	// D端口的8--13输出为1；
//	MODIFY_REG( GPIOD->CRH, 0x00FFFFFFu, 0x00444444u );			//	配置D端口的0--6为浮空输入

	SET_BIT( RCC->APB2ENR, RCC_APB2ENR_AFIOEN );	//	打开复用时钟
																								//	开启中断复用
	AFIO->EXTICR[0] = 														

						AFIO_EXTICR1_EXTI3_PB
					;
	AFIO->EXTICR[1] = 
					  AFIO_EXTICR2_EXTI4_PB
					| AFIO_EXTICR2_EXTI5_PB
					;
	CLEAR_BIT( EXTI->IMR,  0xFFFFu );	// 禁止中断
	CLEAR_BIT( EXTI->EMR,  0xFFFFu );	// no event 无中断事件
//	SET_BIT  ( EXTI->RTSR, 0x0038u );	// rising edge trigger  上升沿触发
	SET_BIT  ( EXTI->FTSR, 0x0038u );	// falling edge trigger 下降沿触发
	WRITE_REG( EXTI->PR,   0xFFFFu );	// 通过写1能复位触发的中断标志

	SET_BIT  ( EXTI->IMR,  0x0038u );	// 允许中断
//	// IO中断	
//	NVIC_EnableIRQ( EXTI1_IRQn );
//	NVIC_EnableIRQ( EXTI2_IRQn );
	NVIC_EnableIRQ( EXTI3_IRQn );
	NVIC_EnableIRQ( EXTI4_IRQn );
	NVIC_EnableIRQ( EXTI9_5_IRQn );
//	NVIC_EnableIRQ( EXTI15_10_IRQn );
}
/********************************** 功能说明 ***********************************
*	中断开关控制
*******************************************************************************/
void	EXTIx_IRQ_Enable( uint8_t EXTIx )
{
	SET_BIT( EXTI->PR,   0x01u << EXTIx );	// 写1复位中断标志位
	SET_BIT( EXTI->IMR,  0x01u << EXTIx );	// 允许中断 #1~#6。
}

void	EXTIx_IRQ_Disable( uint8_t EXTIx )
{
	CLEAR_BIT( EXTI->IMR,  0x01u << EXTIx );	// 禁止中断 
	SET_BIT( EXTI->PR,   0x01u << EXTIx );	// 写1复位中断标志位
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
			void	BIOS_RBOOT_ARM_X( void )
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
					TIM3->DIER  = TIM_DIER_UIE;																						//	使能中断
					CLEAR_BIT( TIM3->SR, TIM_SR_UIF );																		//	清除中断标记
					NVIC_EnableIRQ( TIM3_IRQn );
	//				SET_BIT( TIM3->CR1, TIM_CR1_CEN );																		//	使能计数器
			}
		/******************************** 功能说明 *************************************
		*	机械臂PWM控制
		*******************************************************************************/
			void	PWM_CTRL_X( uint16_t ARRValue, uint16_t CCR3Value )
			{
				TIM3->ARR   =  ARRValue;					//	设置预装载值
				TIM3->CCR3  =  CCR3Value;					//	设置翻转值
			}
		/******************************** 功能说明 *************************************
		*	机械臂PWM开关
		*******************************************************************************/
			void	PWM_CMD_X( _Bool	CMD )
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
			volatile	uint32_t	SetPluseCount[3u];			//	设置运行脉冲数
			volatile	uint32_t	PluseCount[3u];					//	实际运行脉冲数
			volatile	_Bool	CountOver[3u]	= {false };		//	计数器溢出标志
			
			void	TIM3_IRQHandler( void )
			{
				CLEAR_BIT( TIM3->SR, TIM_SR_UIF );												//	清除中断标记
				if( PluseCount[0u] < SetPluseCount[0u] )
					PluseCount[0u] ++;
				else
				{
					PWM_CMD_X( false );
					CountOver[0u] = true;
					CLEAR_BIT( TIM3->SR, TIM_SR_UIF );												//	清除中断标记
				}
			}
		/******************************** 功能说明 *************************************
		*	电机方向控制
		*******************************************************************************/
			void	RBOOT_ARM_DIR_X( _Bool	State )
			{
				if( State )
					SET_BIT( GPIOB->BSRR, GPIO_BSRR_BS10 );
				else
					SET_BIT( GPIOB->BSRR, GPIO_BSRR_BR10 );
			}
		/******************************** 功能说明 *************************************
		*	电机驱动器使能 
		*******************************************************************************/
			void	RBOOT_ARM_EN_X( _Bool State )
			{
				if( State )
					SET_BIT( GPIOB->BSRR, GPIO_BSRR_BS13 );
				else
					SET_BIT( GPIOB->BSRR, GPIO_BSRR_BR13 );
			}

	/********************************** 功能说明 *************************************
	*	机械臂Y
	*********************************************************************************/
		/******************************** 功能说明 *************************************
		*	机械臂端口设置 TIM4->CH3
		*******************************************************************************/
			void	BIOS_RBOOT_ARM_Y( void )
			{
				SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPBEN );
				MODIFY_REG( GPIOB->CRH, 0x0000000Fu, 0x0000000Fu );										//	IO口配置		PWM_OUT
				SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPBEN );                          
				MODIFY_REG( GPIOB->CRH, 0x0000F000u, 0x00007000u );										//	IO口配置		DIR
				SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPBEN );                          
				MODIFY_REG( GPIOB->CRH, 0x0F000000u, 0x07000000u );										//	IO口配置		EN

				SET_BIT( GPIOB->BSRR, GPIO_BSRR_BR11 );
				SET_BIT( GPIOB->BSRR, GPIO_BSRR_BR14 );
				
				SET_BIT( RCC->APB1ENR, RCC_APB1ENR_TIM4EN );													//	打开时钟
				SET_BIT( TIM4->EGR, TIM_EGR_UG );																			//	初始化所有寄存器
				TIM4->CR1   = TIM_CR1_ARPE | TIM_CR1_URS;															//	自动装载 仅溢出中断
				TIM4->PSC		= SystemCoreClock / 2u / 100000u - 1u;										//	设置预分频值
				TIM4->ARR   = 10000u;																									//	设置预装载值
				TIM4->CCMR2 = TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE;	//	设置计数极性和工作模式
				TIM4->CCER  = TIM_CCER_CC3E |TIM_CCER_CC3P;														//	使能通道2
				TIM4->CCR3  = 0u;																											//	设置翻转值
				TIM4->DIER  = TIM_DIER_UIE;																						//	使能中断
				CLEAR_BIT( TIM4->SR, TIM_SR_UIF );											//	清除中断标记
				NVIC_EnableIRQ( TIM4_IRQn );
//				SET_BIT( TIM4->CR1, TIM_CR1_CEN );																		//	使能计数器
			}
		/******************************** 功能说明 *************************************
		*	机械臂PWM控制
		*******************************************************************************/
			void	PWM_CTRL_Y( uint16_t ARRValue, uint16_t CCR3Value )
			{
				TIM4->ARR   =  ARRValue;					//	设置预装载值
				TIM4->CCR3  =  CCR3Value;					//	设置翻转值
			}
		/******************************** 功能说明 *************************************
		*	机械臂PWM开关
		*******************************************************************************/
			void	PWM_CMD_Y( _Bool	CMD )
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
			void	TIM4_IRQHandler( void )
			{
				CLEAR_BIT( TIM4->SR, TIM_SR_UIF );												//	清除中断标记
				if( PluseCount[1u] < SetPluseCount[1u] )
					PluseCount[1u] ++;
				else
				{
					PWM_CMD_Y( false );
					CountOver[1u] = true;
					CLEAR_BIT( TIM4->SR, TIM_SR_UIF );												//	清除中断标记
				}
			}
		/******************************** 功能说明 *************************************
		*	电机方向控制
		*******************************************************************************/
			void	RBOOT_ARM_DIR_Y( _Bool	State )
			{
				if( State )
					SET_BIT( GPIOB->BSRR, GPIO_BSRR_BR11 );
				else
					SET_BIT( GPIOB->BSRR, GPIO_BSRR_BS11 );
			}
		/******************************** 功能说明 *************************************
		*	电机驱动器使能 
		*******************************************************************************/
			void	RBOOT_ARM_EN_Y( _Bool State )
			{
				if( State )
					SET_BIT( GPIOB->BSRR, GPIO_BSRR_BS14 );
				else
					SET_BIT( GPIOB->BSRR, GPIO_BSRR_BR14 );
			}
		
	/********************************** 功能说明 *************************************
	*	机械臂Z
	*********************************************************************************/
		/******************************** 功能说明 *************************************
		*	机械臂端口设置 TIM2->CH1
		*******************************************************************************/
			uint32_t	TIMER_F = 100000u;
			void	BIOS_RBOOT_ARM_Z( void )
			{
				SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPAEN );
				MODIFY_REG( GPIOA->CRL, 0x0000000Fu, 0x0000000Fu );										//	IO口配置		PWM_OUT
				SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPBEN );                          
				MODIFY_REG( GPIOB->CRH, 0x000F0000u, 0x00070000u );										//	IO口配置		DIR
				SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPBEN );                          
				MODIFY_REG( GPIOB->CRH, 0x0F000000u, 0x07000000u );										//	IO口配置		EN

				SET_BIT( GPIOB->BSRR, GPIO_BSRR_BS12 );
				SET_BIT( GPIOB->BSRR, GPIO_BSRR_BR15);
				
				SET_BIT( RCC->APB1ENR, RCC_APB1ENR_TIM2EN );													//	打开时钟
				SET_BIT( TIM2->EGR, TIM_EGR_UG );																			//	初始化所有寄存器
				TIM2->CR1   = TIM_CR1_ARPE | TIM_CR1_URS;															//	自动装载 仅溢出中断
				TIM2->PSC		= SystemCoreClock / 2u / 100000u - 1u;										//	设置预分频值
				TIM2->ARR   = 10000u;																									//	设置预装载值
				TIM2->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE;	//	设置计数极性和工作模式
				TIM2->CCER  = TIM_CCER_CC1E |TIM_CCER_CC1P;														//	使能通道1
				TIM2->CCR1  = 0u;																											//	设置翻转值
				TIM2->DIER  = TIM_DIER_UIE;																						//	使能中断
				CLEAR_BIT( TIM2->SR, TIM_SR_UIF );											//	清除中断标记
				NVIC_EnableIRQ( TIM2_IRQn );			
//				SET_BIT( TIM2->CR1, TIM_CR1_CEN );																		//	使能计数器
			}
			/******************************** 功能说明 *************************************
		*	机械臂PWM控制
		*******************************************************************************/
			void	PWM_CTRL_Z( uint16_t ARRValue, uint16_t CCR1Value )
			{
				TIM2->ARR   =  ARRValue;					//	设置预装载值
				TIM2->CCR1  =  CCR1Value;					//	设置翻转值
			}
		/******************************** 功能说明 *************************************
		*	机械臂PWM开关
		*******************************************************************************/
			void	PWM_CMD_Z( _Bool	CMD )
			{
				if( CMD )
				{
					SET_BIT( TIM2->CCER,TIM_CCER_CC1E );																	//	使能通道1
					SET_BIT( TIM2->CR1, TIM_CR1_CEN   );																	//	使能计数器
				}
				else
				{
					CLEAR_BIT( TIM2->CCER,TIM_CCER_CC1E );																//	失能通道1
					CLEAR_BIT( TIM2->CR1, TIM_CR1_CEN   );																//	失能计数器
					TIM2->CNT = 0u;
				}
			}
		/******************************** 功能说明 *************************************
		*	中断处理函数
		*******************************************************************************/
			void	TIM2_IRQHandler( void )
			{
				CLEAR_BIT( TIM2->SR, TIM_SR_UIF );												//	清除中断标记
				if( PluseCount[2u] < SetPluseCount[2u] )
					PluseCount[2u] ++;
				else
				{
					PWM_CMD_Z( false );
					CountOver[2u] = true;
					CLEAR_BIT( TIM2->SR, TIM_SR_UIF );												//	清除中断标记
				}
			}

		/******************************** 功能说明 *************************************
		*	电机方向控制
		*******************************************************************************/
			void	RBOOT_ARM_DIR_Z( _Bool	State )
			{
				if( State )
					SET_BIT( GPIOB->BSRR, GPIO_BSRR_BS12 );
				else
					SET_BIT( GPIOB->BSRR, GPIO_BSRR_BR12 );
			}
		/******************************** 功能说明 *************************************
		*	电机驱动器使能 
		*******************************************************************************/
			void	RBOOT_ARM_EN_Z( _Bool State )
			{
				if( State )
					SET_BIT( GPIOB->BSRR, GPIO_BSRR_BS15 );
				else
					SET_BIT( GPIOB->BSRR, GPIO_BSRR_BR15 );
			}

//	/********************************** 功能说明 *************************************
//  *	机械手指端口初始化 TIM8->CH4
//  *********************************************************************************/
//  	void	BIOS_Fingers_Init( void )
//		{
//			SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPCEN );
//			MODIFY_REG( GPIOC->CRH, 0x000000F0u, 0x000000F0u );										//	IO口配置		PWM_OUT
////			SET_BIT( GPIOC->BSRR, GPIO_BSRR_BS9 );
//			SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPAEN );
//			MODIFY_REG( GPIOA->CRH, 0x0000000Fu, 0x0000000Fu );										//	IO口配置		Fingers_DIR(Cmd)

//			SET_BIT( RCC->APB2ENR, RCC_APB2ENR_TIM8EN );													//	打开时钟
//			SET_BIT( TIM8->EGR, TIM_EGR_UG );																			//	初始化所有寄存器
//			TIM8->CR1   = TIM_CR1_ARPE | TIM_CR1_URS;															//	自动装载 仅溢出中断
//			TIM8->PSC		= SystemCoreClock / 100000u - 1u;													//	设置预分频值
//			TIM8->ARR   = 50u;																										//	设置预装载值
//			TIM8->CCMR2 = TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE;	//	设置计数极性和工作模式
//			TIM8->CCER  = TIM_CCER_CC4E |TIM_CCER_CC4P;														//	使能通道4
//			TIM8->CCR1  = 0u;																											//	设置翻转值
//			TIM8->DIER  = TIM_DIER_CC1IE;																					//	使能1号中断
//			CLEAR_BIT( TIM8->SR, TIM_SR_UIF | TIM_SR_CC4IF );											//	清除中断标记
//			SET_BIT( TIM8->BDTR, TIM_BDTR_MOE );
//			SET_BIT( TIM8->CR1, TIM_CR1_CEN );																		//	使能计数器
//		}
//	
//  /******************************** 功能说明 *************************************
//  *	机械手指PWM控制
//  *******************************************************************************/
//		void	Fingers_CTRL( uint16_t ARRValue, uint16_t CCR1Value )
//		{
//			TIM8->ARR   =  ARRValue;					//	设置预装载值
//			TIM8->CCR4  =  CCR1Value;					//	设置翻转值
//		}
//	
//  /******************************** 功能说明 *************************************
//  *	机械手指PWM开关
//  *******************************************************************************/
// 		void	Fingers_CMD( _Bool	CMD )
//		{
//			if( CMD )
//			{
//				SET_BIT( TIM8->CCER,TIM_CCER_CC4E );																	//	使能通道1
//				SET_BIT( TIM8->CR1, TIM_CR1_CEN   );																	//	使能计数器
//			}
//			else
//			{
//				CLEAR_BIT( TIM8->CCER,TIM_CCER_CC4E );																//	失能通道1
//				CLEAR_BIT( TIM8->CR1, TIM_CR1_CEN   );																//	失能计数器
//				TIM5->CNT = 0u;
//			}
//		}
// 
//	/******************************** 功能说明 *************************************
//	*	中断处理函数
//	*******************************************************************************/
//		volatile	uint32_t	SetPluseCount;			//	设置运行脉冲数
//		volatile	uint32_t	PluseCount;					//	实际运行脉冲数
//		volatile	_Bool	CountOver	= false;			//	计数器溢出标志
//		void	TIM8_IRQHandler( void )
//		{
//			CLEAR_BIT( TIM8->SR, TIM_SR_UIF | TIM_SR_CC4IF );												//	清除中断标记
//			if( PluseCount < SetPluseCount )
//				PluseCount ++;
//			else
//			{
//				Fingers_CMD( false );
//				CountOver = true;
//				CLEAR_BIT( TIM8->SR, TIM_SR_UIF | TIM_SR_CC4IF );												//	清除中断标记
//			}
//		}
//	/******************************** 功能说明 *************************************
//	*	电机方向控制 机械手指运动方向 0 开 1 关
//	*******************************************************************************/
//		void	Fingers_DIR( _Bool	State )
//		{
//			if( State )
//				SET_BIT( GPIOA->BSRR, GPIO_BSRR_BS12 );
//			else
//				SET_BIT( GPIOA->BSRR, GPIO_BSRR_BR12 );
//		}
//	
//  /******************************** 功能说明 *************************************
//  *	机械手指脉冲计数
//  *******************************************************************************/
//  	void	BIOS_Fingers_PWMIn_Init( void )
//		{
//			SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPAEN );
//			MODIFY_REG( GPIOA->CRH, 0x000F0000u, 0x00040000u );										//	IO口配置		PWM_OUT
//			SET_BIT( RCC->APB2ENR, RCC_APB2ENR_TIM1EN );													//	打开时钟
//			SET_BIT( TIM1->EGR, TIM_EGR_UG );																			//	初始化所有寄存器
//			TIM1->SMCR =  TIM_SMCR_ECE;	//	设置工作模式 外部时钟2模式
//			SET_BIT( TIM1->BDTR, TIM_BDTR_MOE );
//			SET_BIT( TIM1->CR1, TIM_CR1_CEN );																		//	使能计数器
//		}

	/********************************** 功能说明 *************************************
  *	搅拌机PWM控制
  *********************************************************************************/
//		void	Blender_Control( uint16_t	Value )
//		{
////			if( State )
////				SET_BIT( GPIOC->BSRR, GPIO_BSRR_BR6 );
////			else
////				SET_BIT( GPIOC->BSRR, GPIO_BSRR_BS6 );
//			static	uint16_t	LastValue;
//			if( Value != LastValue)
//			{
//				if( Value ) 
//				{
//					TIM8->ARR  = Value;																										//	设置预装载值
//					TIM8->CCR1 = Value / 2;			
//					SET_BIT( TIM8->CR1, TIM_CR1_CEN );																		//	使能计数器
//				}
//				else
//				{
//					CLEAR_BIT( TIM8->CR1, TIM_CR1_CEN );																		//	使能计数器
//				
//				}
//				LastValue = Value;
//			}
//		}
//	/********************************** 功能说明 *************************************
//  *	搅拌器PWM 端口初始化
//  *********************************************************************************/
//  	void	BIOS_Motor_Init( void )
//		{
//			SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPAEN );
//			MODIFY_REG( GPIOA->CRH, 0x0000000Fu, 0x0000000Fu );										//	IO口配置		PWM_OUT
////			SET_BIT( GPIOC->BSRR, GPIO_BSRR_BS6 );
//			SET_BIT( RCC->APB2ENR, RCC_APB2ENR_TIM1EN );													//	打开时钟
//			SET_BIT( TIM1->EGR, TIM_EGR_UG );																			//	初始化所有寄存器
//			TIM1->CR1   = TIM_CR1_ARPE | TIM_CR1_URS;															//	自动装载 仅溢出中断
//			TIM1->PSC		= SystemCoreClock / 100000u - 1u;													//	设置预分频值
//			TIM1->ARR   = 30u - 1u;																							//	设置预装载值
//			TIM1->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE;	//	设置计数极性和工作模式
//			TIM1->CCER  = TIM_CCER_CC1E |TIM_CCER_CC1P;														//	使能通道1
//			TIM1->CCR1  = 0u;																											//	设置翻转值
////			TIM8->DIER  = TIM_DIER_CC1IE;																					//	使能1号中断
////			CLEAR_BIT( TIM8->SR, TIM_SR_UIF | TIM_SR_CC1IF );											//	清除中断标记
//			SET_BIT( TIM1->BDTR, TIM_BDTR_MOE );
//			SET_BIT( TIM1->CR1, TIM_CR1_CEN );																		//	使能计数器

//		}
//	/********************************** 功能说明 *************************************
//  *	搅拌机PWM控制
//  *********************************************************************************/
//		void	EMotor_Control( uint16_t	Value )
//		{
////			if( State )
////				SET_BIT( GPIOC->BSRR, GPIO_BSRR_BR6 );
////			else
////				SET_BIT( GPIOC->BSRR, GPIO_BSRR_BS6 );
//			static	uint16_t	LastValue;
//			if( Value != LastValue)
//			{
//				if( Value ) 
//				{
//					TIM1->ARR  = Value;																										//	设置预装载值
//					TIM1->CCR1 = Value / 2;			
//					SET_BIT( TIM1->CR1, TIM_CR1_CEN );																		//	使能计数器
//				}
//				else
//				{
//					CLEAR_BIT( TIM1->CR1, TIM_CR1_CEN );																		//	使能计数器
//				
//				}
//				LastValue = Value;
//			}
//		
////			if( State  )
////				TIM1->CCR1 = 15;
////			else
////				TIM1->CCR1 = 0;
//		}
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
			
			NVIC_EnableIRQ( USART2_IRQn );
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

			while ( ! ( READ_BIT( USARTx->SR, USART_SR_RXNE )))
			{
				;
			}

			return	USARTx->DR;
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
	
//	/******************************** 功能说明 *************************************
//	*	串口4端口配置
//	*******************************************************************************/
//		void	BIOS_USART4_Init( void )
//		{
//			USART_TypeDef * USARTx = UART4;

//			SET_BIT( RCC->APB2ENR, RCC_APB1ENR_UART4EN );

//			// USART1 configured as follow:
//			// 	- BaudRate = 9600 baud
//			// 	- Word Length = 8 Bits
//			// 	- One Stop Bit
//			// 	- No parity
//			// 	- Hardware flow control disabled (RTS and CTS signals)
//			// 	- Receive disable and transmit enabled
//			USARTx->BRR = SystemCoreClock / 9600u * 2u;	/* 9600 bps							*/
//			USARTx->CR1 = 0x0000u;					/* 1 start bit, 8 data bits         */
//			USARTx->CR2 = 0x0000u;					/* 1 stop bit                       */
//			USARTx->CR3 = 0x0000u; 					/* no flow control                  */
//			SET_BIT( USARTx->CR1, USART_CR1_TE );	/* enable TX 						*/
//			SET_BIT( USARTx->CR1, USART_CR1_RE );	/* enable TX 						*/
//			SET_BIT( USARTx->CR1, USART_CR1_UE );	/* Enable USARTx					*/
//			USART_ITConfig( USART1, /*USART_IT_TXE |*/ USART_IT_RXNE, ENABLE );

//			/* Configure USART1 Rx (PA10) as input floating */
//			/* Configure USART1 Tx (PA9) as alternate function push-pull */
//			SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPCEN );
//			MODIFY_REG( GPIOC->CRH, 0x0000FF00u, 0x00004B00u );
//		}
//		
//	/******************************** 功能说明 *************************************
//	*	串口5端口配置
//	*******************************************************************************/
//		void	BIOS_USART5_Init( void )
//		{
//			USART_TypeDef * USARTx = UART5;

//			SET_BIT( RCC->APB2ENR, RCC_APB1ENR_UART5EN );

//			// USART1 configured as follow:
//			// 	- BaudRate = 9600 baud
//			// 	- Word Length = 8 Bits
//			// 	- One Stop Bit
//			// 	- No parity
//			// 	- Hardware flow control disabled (RTS and CTS signals)
//			// 	- Receive disable and transmit enabled
//			USARTx->BRR = SystemCoreClock / 9600u * 2u;	/* 9600 bps							*/
//			USARTx->CR1 = 0x0000u;					/* 1 start bit, 8 data bits         */
//			USARTx->CR2 = 0x0000u;					/* 1 stop bit                       */
//			USARTx->CR3 = 0x0000u; 					/* no flow control                  */
//			SET_BIT( USARTx->CR1, USART_CR1_TE );	/* enable TX 						*/
//			SET_BIT( USARTx->CR1, USART_CR1_RE );	/* enable TX 						*/
//			SET_BIT( USARTx->CR1, USART_CR1_UE );	/* Enable USARTx					*/
//			USART_ITConfig( USART1, /*USART_IT_TXE |*/ USART_IT_RXNE, ENABLE );

//			/* Configure USART1 Rx (PA10) as input floating */
//			/* Configure USART1 Tx (PA9) as alternate function push-pull */
//			SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPCEN );
//			MODIFY_REG( GPIOC->CRH, 0x000F0000u, 0x000B0000u );
//			SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPDEN );
//			MODIFY_REG( GPIOD->CRL, 0x00000F00u, 0x00000B00u );
//		}
	




/******************************** 功能说明 *************************************
*	IO控制
*******************************************************************************/
//	/******************************** 功能说明 *************************************
//	*	电磁阀控制端口初始化
//	*******************************************************************************/
//		void	BIOS_SOLENOIDVALVE_Init( void )
//		{
//			SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPBEN );
//			MODIFY_REG( GPIOB->CRL, 0x0FFF0000u, 0x03330000u );
//			
//			SET_BIT( GPIOB->BSRR, GPIO_BSRR_BS4 );
//			SET_BIT( GPIOB->BSRR, GPIO_BSRR_BS5 );
//			SET_BIT( GPIOB->BSRR, GPIO_BSRR_BS6 );

//		}
//		
//  /******************************** 功能说明 *************************************
//  *	定量控制气泵两通阀
//  *******************************************************************************/  
//		void	SolenoidValve1_Cmd( _Bool	Newstate )
//		{
//			static	_Bool	state = false;
//			if( state != Newstate )
//			{
//				state = Newstate;
//				if( state )
//				{
//					SET_BIT( GPIOB->BSRR, GPIO_BSRR_BR4 );
//				}
//				else
//				{
//					SET_BIT( GPIOB->BSRR, GPIO_BSRR_BS4 );
//				}
//			}		
//		}	
//	
//  /******************************** 功能说明 *************************************
//  *	排空气泵两通阀
//  *******************************************************************************/
//		void	SolenoidValve2_Cmd( _Bool	Newstate )
//		{
//			static	_Bool	state = false;
//			if( state != Newstate )
//			{
//				state = Newstate;
//				if( state )
//				{
//					SET_BIT( GPIOB->BSRR, GPIO_BSRR_BR5 );
//				}
//				else
//				{
//					SET_BIT( GPIOB->BSRR, GPIO_BSRR_BS5 );
//				}
//			}		
//		}	
//	
//  /******************************** 功能说明 *************************************
//  *	蒸馏水注射管切换阀
//  *******************************************************************************/
//		void	SolenoidValve3_Cmd( _Bool	Newstate )
//		{
//			static	_Bool	state = false;
//			if( state != Newstate )
//			{
//				state = Newstate;
//				if( state )
//				{
//					SET_BIT( GPIOB->BSRR, GPIO_BSRR_BR6 );
//				}
//				else
//				{
//					SET_BIT( GPIOB->BSRR, GPIO_BSRR_BS6 );
//				}
//			}		
//		}
//		
//		
/******************************** 功能说明 *************************************
*	IO输入检测
*******************************************************************************/
//    /******************************** 功能说明 *************************************
//    *	液位检测配置
//    *******************************************************************************/
//			void	LEVELSWITCH_Init( void )
//			{
//				SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPAEN );
//				MODIFY_REG( GPIOA->CRL, 0x0FF00000u, 0x04400000u );										//	IO口配置		PWM_OUT
//				
//			}
//		
//    /******************************** 功能说明 *************************************
//    *	液位检测1
//    *******************************************************************************/
//    _Bool	LevelSwitch1Read( void )
//		{		
//			return	READ_BIT( GPIOA->IDR, GPIO_IDR_IDR6 );
//		}
//   /******************************** 功能说明 *************************************
//    *	液位检测2
//    *******************************************************************************/
//    _Bool	LevelSwitch2Read( void )
//		{		
//			return	READ_BIT( GPIOA->IDR, GPIO_IDR_IDR2 );
//		}
//	
		
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

int	BIOS_Init( void )
{
	int	ErrCode = 0; 
	BIOS_SWD_Init();
	BIOS_NVIC_Init();
	BIOS_EXTI_Init();
	BIOS_RBOOT_ARM_X();
	BIOS_RBOOT_ARM_Y();
	BIOS_RBOOT_ARM_Z();
	
	return ErrCode;
}

/********  (C) COPYRIGHT 2018 青岛中特环保仪器有限公司  **** End Of File ********/
