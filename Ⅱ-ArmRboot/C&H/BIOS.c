/**************** (C) COPYRIGHT 2018 �ൺ���ػ����������޹�˾ *******************
* �� �� ��:  
* �� �� ��:  Kaiser
* ��  ��  :  
* ����޸�:  
*********************************** �޶���¼ ************************************
* ��  ��: 
* �޶���: 
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
//	0��(0000B)ģ������ģʽ
//	4��(0100B)��������ģʽ(��λ���״̬)
//	8��(1000B)����/��������ģʽ
//	C��(1100B)����
//	
//	1��(0001B)ͨ���������ģʽ10MHz
//	2��(0010B)ͨ���������ģʽ2MHz
//	3��(0011B)ͨ���������ģʽ50MHz
//	
//	5��(0101B)ͨ�ÿ�©���ģʽ10MHz
//	6��(0110B)ͨ�ÿ�©���ģʽ2MHz
//	7��(0111B)ͨ�ÿ�©���ģʽ50MHz
//	
//	9��(1001B)���ù����������ģʽ10MHz
//	A��(1010B)���ù����������ģʽ2MHz
//	B��(1011B)���ù����������ģʽ50MHz
//	
//	D��(1101B)���ù��ܿ�©���ģʽ10MHz
//	E��(1110B)���ù��ܿ�©���ģʽ2MHz
//	F��(1111B)���ù��ܿ�©���ģʽ50MHz

/********************************** ����˵�� ***********************************
*	����ʱ��������Ӳ��ͬ������ȷ��ʱ��Ҫ��ֹ�жϵĸ���
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
/******************************** ����˵�� *************************************
*	�����ж϶˿�����
*******************************************************************************/

/*
Ӳ���ж�ѡ��
ͨ������Ĺ���������19����·��Ϊ�ж�Դ��
��
����19���ж��ߵ�����λ(EXTI_IMR)
��
������ѡ�ж��ߵĴ���ѡ��λ(EXTI_RTSR��EXTI_FTSR)��
��
������Щ����ӳ���ⲿ�жϿ�����(EXTI)��NVIC�ж�ͨ����ʹ�ܺ�����λ��ʹ��19���ж����е�������Ա���ȷ����Ӧ��
Ӳ���¼�ѡ�� PS����Ҫ��ϸ���ж�������

ͨ������Ĺ��̣���������19����·Ϊ�¼�Դ
��
����19���¼��ߵ�����λ(EXTI_EMR)
��
�����¼��ߵĴ���ѡ��λ(EXTI_RTSR��EXTI_FTSR)
����ж�/�¼���ѡ��
19����·���Ա����ó�����ж�/�¼��ߡ������ǲ�������жϵĹ��̣�
��
����19���ж�/�¼�������λ(EXTI_IMR, EXTI_EMR)
��
��������жϼĴ���������λ(EXTI_SWIER��

ע��: �ⲿ�������Ǳ��ش����ģ���Щ���ϲ��ܳ���ë���źš�
��дEXTI_FTSR�Ĵ���ʱ�����ⲿ�ж����ϵ��½����źŲ��ܱ�ʶ�𣬹���λ���ᱻ��λ��
��ͬһ�ж����ϣ�����ͬʱ���������غ��½��ش���������һ���ض��ɴ����жϡ�

���Ҫ�����жϣ������������úò�ʹ���ж��ߡ�
������Ҫ�ı��ؼ������2�������Ĵ�����ͬʱ���ж����μĴ�������Ӧλд��1�� �����ж�����
���ⲿ�ж����Ϸ�������Ҫ�ı���ʱ��������һ���ж����󣬶�Ӧ�Ĺ���λҲ��֮���á�1����
�ڹ���Ĵ����Ķ�Ӧλд��1��������������ж�����
*/

/******************************** ����˵�� *************************************
*	�ж�����ʹ������
*******************************************************************************/
/******************************** ����˵�� *************************************
*	�ж϶˿ڳ�ʼ��ʹ��
*******************************************************************************/
void	BIOS_EXTI_Init( void )
{
//	//	����ʹ�ø�������ģʽ
	SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPBEN ); 						// ��E�˿ڵ�ʱ��
	GPIOB->BSRR |= 0x0038u; 																// �˿����Ϊ1��
	MODIFY_REG( GPIOB->CRL, 0x00FFF000u, 0x00444000u );			//	���ö˿�Ϊ��������

//	SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPDEN ); 						// ��D�˿ڵ�ʱ��
//	GPIOD->BSRR = 0x2F00u; 																	// D�˿ڵ�8--13���Ϊ1��
//	MODIFY_REG( GPIOD->CRH, 0x00FFFFFFu, 0x00444444u );			//	����D�˿ڵ�0--6Ϊ��������

	SET_BIT( RCC->APB2ENR, RCC_APB2ENR_AFIOEN );	//	�򿪸���ʱ��
																								//	�����жϸ���
	AFIO->EXTICR[0] = 														

						AFIO_EXTICR1_EXTI3_PB
					;
	AFIO->EXTICR[1] = 
					  AFIO_EXTICR2_EXTI4_PB
					| AFIO_EXTICR2_EXTI5_PB
					;
	CLEAR_BIT( EXTI->IMR,  0xFFFFu );	// ��ֹ�ж�
	CLEAR_BIT( EXTI->EMR,  0xFFFFu );	// no event ���ж��¼�
//	SET_BIT  ( EXTI->RTSR, 0x0038u );	// rising edge trigger  �����ش���
	SET_BIT  ( EXTI->FTSR, 0x0038u );	// falling edge trigger �½��ش���
	WRITE_REG( EXTI->PR,   0xFFFFu );	// ͨ��д1�ܸ�λ�������жϱ�־

	SET_BIT  ( EXTI->IMR,  0x0038u );	// �����ж�
//	// IO�ж�	
//	NVIC_EnableIRQ( EXTI1_IRQn );
//	NVIC_EnableIRQ( EXTI2_IRQn );
	NVIC_EnableIRQ( EXTI3_IRQn );
	NVIC_EnableIRQ( EXTI4_IRQn );
	NVIC_EnableIRQ( EXTI9_5_IRQn );
//	NVIC_EnableIRQ( EXTI15_10_IRQn );
}
/********************************** ����˵�� ***********************************
*	�жϿ��ؿ���
*******************************************************************************/
void	EXTIx_IRQ_Enable( uint8_t EXTIx )
{
	SET_BIT( EXTI->PR,   0x01u << EXTIx );	// д1��λ�жϱ�־λ
	SET_BIT( EXTI->IMR,  0x01u << EXTIx );	// �����ж� #1~#6��
}

void	EXTIx_IRQ_Disable( uint8_t EXTIx )
{
	CLEAR_BIT( EXTI->IMR,  0x01u << EXTIx );	// ��ֹ�ж� 
	SET_BIT( EXTI->PR,   0x01u << EXTIx );	// д1��λ�жϱ�־λ
}


/******************************** ����˵�� *************************************
*	TIM
*******************************************************************************/
	/********************************** ����˵�� *************************************
	*	��е��X
	*********************************************************************************/
		/******************************** ����˵�� *************************************
		*	��е�۶˿����� TIM3->CH3
		*******************************************************************************/
			void	BIOS_RBOOT_ARM_X( void )
			{
					SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPBEN );
					MODIFY_REG( GPIOB->CRL, 0x0000000Fu, 0x0000000Fu );										//	IO������		PWM_OUT
					SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPBEN );
					MODIFY_REG( GPIOB->CRH, 0x00000F00u, 0x00000700u );										//	IO������		DIR
					SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPBEN );
					MODIFY_REG( GPIOB->CRH, 0x00F00000u, 0x00700000u );										//	IO������		EN

					SET_BIT( GPIOB->BSRR, GPIO_BSRR_BS10 );
					SET_BIT( GPIOB->BSRR, GPIO_BSRR_BR13 );

					SET_BIT( RCC->APB1ENR, RCC_APB1ENR_TIM3EN );													//	��ʱ��
					SET_BIT( TIM3->EGR, TIM_EGR_UG );																			//	��ʼ�����мĴ���
					TIM3->CR1   = TIM_CR1_ARPE | TIM_CR1_URS;															//	�Զ�װ�� ������ж�
					TIM3->PSC		= SystemCoreClock / 2u / 100000u - 1u;										//	����Ԥ��Ƶֵ
					TIM3->ARR   = 10000u;																									//	����Ԥװ��ֵ
					TIM3->CCMR2 = TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE;	//	���ü������Ժ͹���ģʽ
					TIM3->CCER  = TIM_CCER_CC3E | TIM_CCER_CC3P;													//	ʹ��ͨ��3
					TIM3->CCR3  = 0u;																											//	���÷�תֵ
					TIM3->DIER  = TIM_DIER_UIE;																						//	ʹ���ж�
					CLEAR_BIT( TIM3->SR, TIM_SR_UIF );																		//	����жϱ��
					NVIC_EnableIRQ( TIM3_IRQn );
	//				SET_BIT( TIM3->CR1, TIM_CR1_CEN );																		//	ʹ�ܼ�����
			}
		/******************************** ����˵�� *************************************
		*	��е��PWM����
		*******************************************************************************/
			void	PWM_CTRL_X( uint16_t ARRValue, uint16_t CCR3Value )
			{
				TIM3->ARR   =  ARRValue;					//	����Ԥװ��ֵ
				TIM3->CCR3  =  CCR3Value;					//	���÷�תֵ
			}
		/******************************** ����˵�� *************************************
		*	��е��PWM����
		*******************************************************************************/
			void	PWM_CMD_X( _Bool	CMD )
			{
				if( CMD )
				{
					SET_BIT( TIM3->CCER,TIM_CCER_CC3E );																	//	ʹ��ͨ��2
					SET_BIT( TIM3->CR1, TIM_CR1_CEN   );																	//	ʹ�ܼ�����
				}
				else
				{
					CLEAR_BIT( TIM3->CCER,TIM_CCER_CC3E );																//	ʧ��ͨ��2
					CLEAR_BIT( TIM3->CR1, TIM_CR1_CEN   );																//	ʧ�ܼ�����
					TIM3->CNT = 0u;
				}
			}
		/******************************** ����˵�� *************************************
		*	�жϴ�����
		*******************************************************************************/
			volatile	uint32_t	SetPluseCount[3u];			//	��������������
			volatile	uint32_t	PluseCount[3u];					//	ʵ������������
			volatile	_Bool	CountOver[3u]	= {false };		//	�����������־
			
			void	TIM3_IRQHandler( void )
			{
				CLEAR_BIT( TIM3->SR, TIM_SR_UIF );												//	����жϱ��
				if( PluseCount[0u] < SetPluseCount[0u] )
					PluseCount[0u] ++;
				else
				{
					PWM_CMD_X( false );
					CountOver[0u] = true;
					CLEAR_BIT( TIM3->SR, TIM_SR_UIF );												//	����жϱ��
				}
			}
		/******************************** ����˵�� *************************************
		*	����������
		*******************************************************************************/
			void	RBOOT_ARM_DIR_X( _Bool	State )
			{
				if( State )
					SET_BIT( GPIOB->BSRR, GPIO_BSRR_BS10 );
				else
					SET_BIT( GPIOB->BSRR, GPIO_BSRR_BR10 );
			}
		/******************************** ����˵�� *************************************
		*	���������ʹ�� 
		*******************************************************************************/
			void	RBOOT_ARM_EN_X( _Bool State )
			{
				if( State )
					SET_BIT( GPIOB->BSRR, GPIO_BSRR_BS13 );
				else
					SET_BIT( GPIOB->BSRR, GPIO_BSRR_BR13 );
			}

	/********************************** ����˵�� *************************************
	*	��е��Y
	*********************************************************************************/
		/******************************** ����˵�� *************************************
		*	��е�۶˿����� TIM4->CH3
		*******************************************************************************/
			void	BIOS_RBOOT_ARM_Y( void )
			{
				SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPBEN );
				MODIFY_REG( GPIOB->CRH, 0x0000000Fu, 0x0000000Fu );										//	IO������		PWM_OUT
				SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPBEN );                          
				MODIFY_REG( GPIOB->CRH, 0x0000F000u, 0x00007000u );										//	IO������		DIR
				SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPBEN );                          
				MODIFY_REG( GPIOB->CRH, 0x0F000000u, 0x07000000u );										//	IO������		EN

				SET_BIT( GPIOB->BSRR, GPIO_BSRR_BR11 );
				SET_BIT( GPIOB->BSRR, GPIO_BSRR_BR14 );
				
				SET_BIT( RCC->APB1ENR, RCC_APB1ENR_TIM4EN );													//	��ʱ��
				SET_BIT( TIM4->EGR, TIM_EGR_UG );																			//	��ʼ�����мĴ���
				TIM4->CR1   = TIM_CR1_ARPE | TIM_CR1_URS;															//	�Զ�װ�� ������ж�
				TIM4->PSC		= SystemCoreClock / 2u / 100000u - 1u;										//	����Ԥ��Ƶֵ
				TIM4->ARR   = 10000u;																									//	����Ԥװ��ֵ
				TIM4->CCMR2 = TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE;	//	���ü������Ժ͹���ģʽ
				TIM4->CCER  = TIM_CCER_CC3E |TIM_CCER_CC3P;														//	ʹ��ͨ��2
				TIM4->CCR3  = 0u;																											//	���÷�תֵ
				TIM4->DIER  = TIM_DIER_UIE;																						//	ʹ���ж�
				CLEAR_BIT( TIM4->SR, TIM_SR_UIF );											//	����жϱ��
				NVIC_EnableIRQ( TIM4_IRQn );
//				SET_BIT( TIM4->CR1, TIM_CR1_CEN );																		//	ʹ�ܼ�����
			}
		/******************************** ����˵�� *************************************
		*	��е��PWM����
		*******************************************************************************/
			void	PWM_CTRL_Y( uint16_t ARRValue, uint16_t CCR3Value )
			{
				TIM4->ARR   =  ARRValue;					//	����Ԥװ��ֵ
				TIM4->CCR3  =  CCR3Value;					//	���÷�תֵ
			}
		/******************************** ����˵�� *************************************
		*	��е��PWM����
		*******************************************************************************/
			void	PWM_CMD_Y( _Bool	CMD )
			{
				if( CMD )
				{
					SET_BIT( TIM4->CCER,TIM_CCER_CC3E );																	//	ʹ��ͨ��2
					SET_BIT( TIM4->CR1, TIM_CR1_CEN   );																	//	ʹ�ܼ�����
				}
				else
				{
					CLEAR_BIT( TIM4->CCER,TIM_CCER_CC3E );																//	ʧ��ͨ��2
					CLEAR_BIT( TIM4->CR1, TIM_CR1_CEN   );																//	ʧ�ܼ�����
					TIM4->CNT = 0u;
				}
			}
		/******************************** ����˵�� *************************************
		*	�жϴ�����
		*******************************************************************************/
			void	TIM4_IRQHandler( void )
			{
				CLEAR_BIT( TIM4->SR, TIM_SR_UIF );												//	����жϱ��
				if( PluseCount[1u] < SetPluseCount[1u] )
					PluseCount[1u] ++;
				else
				{
					PWM_CMD_Y( false );
					CountOver[1u] = true;
					CLEAR_BIT( TIM4->SR, TIM_SR_UIF );												//	����жϱ��
				}
			}
		/******************************** ����˵�� *************************************
		*	����������
		*******************************************************************************/
			void	RBOOT_ARM_DIR_Y( _Bool	State )
			{
				if( State )
					SET_BIT( GPIOB->BSRR, GPIO_BSRR_BR11 );
				else
					SET_BIT( GPIOB->BSRR, GPIO_BSRR_BS11 );
			}
		/******************************** ����˵�� *************************************
		*	���������ʹ�� 
		*******************************************************************************/
			void	RBOOT_ARM_EN_Y( _Bool State )
			{
				if( State )
					SET_BIT( GPIOB->BSRR, GPIO_BSRR_BS14 );
				else
					SET_BIT( GPIOB->BSRR, GPIO_BSRR_BR14 );
			}
		
	/********************************** ����˵�� *************************************
	*	��е��Z
	*********************************************************************************/
		/******************************** ����˵�� *************************************
		*	��е�۶˿����� TIM2->CH1
		*******************************************************************************/
			uint32_t	TIMER_F = 100000u;
			void	BIOS_RBOOT_ARM_Z( void )
			{
				SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPAEN );
				MODIFY_REG( GPIOA->CRL, 0x0000000Fu, 0x0000000Fu );										//	IO������		PWM_OUT
				SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPBEN );                          
				MODIFY_REG( GPIOB->CRH, 0x000F0000u, 0x00070000u );										//	IO������		DIR
				SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPBEN );                          
				MODIFY_REG( GPIOB->CRH, 0x0F000000u, 0x07000000u );										//	IO������		EN

				SET_BIT( GPIOB->BSRR, GPIO_BSRR_BS12 );
				SET_BIT( GPIOB->BSRR, GPIO_BSRR_BR15);
				
				SET_BIT( RCC->APB1ENR, RCC_APB1ENR_TIM2EN );													//	��ʱ��
				SET_BIT( TIM2->EGR, TIM_EGR_UG );																			//	��ʼ�����мĴ���
				TIM2->CR1   = TIM_CR1_ARPE | TIM_CR1_URS;															//	�Զ�װ�� ������ж�
				TIM2->PSC		= SystemCoreClock / 2u / 100000u - 1u;										//	����Ԥ��Ƶֵ
				TIM2->ARR   = 10000u;																									//	����Ԥװ��ֵ
				TIM2->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE;	//	���ü������Ժ͹���ģʽ
				TIM2->CCER  = TIM_CCER_CC1E |TIM_CCER_CC1P;														//	ʹ��ͨ��1
				TIM2->CCR1  = 0u;																											//	���÷�תֵ
				TIM2->DIER  = TIM_DIER_UIE;																						//	ʹ���ж�
				CLEAR_BIT( TIM2->SR, TIM_SR_UIF );											//	����жϱ��
				NVIC_EnableIRQ( TIM2_IRQn );			
//				SET_BIT( TIM2->CR1, TIM_CR1_CEN );																		//	ʹ�ܼ�����
			}
			/******************************** ����˵�� *************************************
		*	��е��PWM����
		*******************************************************************************/
			void	PWM_CTRL_Z( uint16_t ARRValue, uint16_t CCR1Value )
			{
				TIM2->ARR   =  ARRValue;					//	����Ԥװ��ֵ
				TIM2->CCR1  =  CCR1Value;					//	���÷�תֵ
			}
		/******************************** ����˵�� *************************************
		*	��е��PWM����
		*******************************************************************************/
			void	PWM_CMD_Z( _Bool	CMD )
			{
				if( CMD )
				{
					SET_BIT( TIM2->CCER,TIM_CCER_CC1E );																	//	ʹ��ͨ��1
					SET_BIT( TIM2->CR1, TIM_CR1_CEN   );																	//	ʹ�ܼ�����
				}
				else
				{
					CLEAR_BIT( TIM2->CCER,TIM_CCER_CC1E );																//	ʧ��ͨ��1
					CLEAR_BIT( TIM2->CR1, TIM_CR1_CEN   );																//	ʧ�ܼ�����
					TIM2->CNT = 0u;
				}
			}
		/******************************** ����˵�� *************************************
		*	�жϴ�����
		*******************************************************************************/
			void	TIM2_IRQHandler( void )
			{
				CLEAR_BIT( TIM2->SR, TIM_SR_UIF );												//	����жϱ��
				if( PluseCount[2u] < SetPluseCount[2u] )
					PluseCount[2u] ++;
				else
				{
					PWM_CMD_Z( false );
					CountOver[2u] = true;
					CLEAR_BIT( TIM2->SR, TIM_SR_UIF );												//	����жϱ��
				}
			}

		/******************************** ����˵�� *************************************
		*	����������
		*******************************************************************************/
			void	RBOOT_ARM_DIR_Z( _Bool	State )
			{
				if( State )
					SET_BIT( GPIOB->BSRR, GPIO_BSRR_BS12 );
				else
					SET_BIT( GPIOB->BSRR, GPIO_BSRR_BR12 );
			}
		/******************************** ����˵�� *************************************
		*	���������ʹ�� 
		*******************************************************************************/
			void	RBOOT_ARM_EN_Z( _Bool State )
			{
				if( State )
					SET_BIT( GPIOB->BSRR, GPIO_BSRR_BS15 );
				else
					SET_BIT( GPIOB->BSRR, GPIO_BSRR_BR15 );
			}

//	/********************************** ����˵�� *************************************
//  *	��е��ָ�˿ڳ�ʼ�� TIM8->CH4
//  *********************************************************************************/
//  	void	BIOS_Fingers_Init( void )
//		{
//			SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPCEN );
//			MODIFY_REG( GPIOC->CRH, 0x000000F0u, 0x000000F0u );										//	IO������		PWM_OUT
////			SET_BIT( GPIOC->BSRR, GPIO_BSRR_BS9 );
//			SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPAEN );
//			MODIFY_REG( GPIOA->CRH, 0x0000000Fu, 0x0000000Fu );										//	IO������		Fingers_DIR(Cmd)

//			SET_BIT( RCC->APB2ENR, RCC_APB2ENR_TIM8EN );													//	��ʱ��
//			SET_BIT( TIM8->EGR, TIM_EGR_UG );																			//	��ʼ�����мĴ���
//			TIM8->CR1   = TIM_CR1_ARPE | TIM_CR1_URS;															//	�Զ�װ�� ������ж�
//			TIM8->PSC		= SystemCoreClock / 100000u - 1u;													//	����Ԥ��Ƶֵ
//			TIM8->ARR   = 50u;																										//	����Ԥװ��ֵ
//			TIM8->CCMR2 = TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE;	//	���ü������Ժ͹���ģʽ
//			TIM8->CCER  = TIM_CCER_CC4E |TIM_CCER_CC4P;														//	ʹ��ͨ��4
//			TIM8->CCR1  = 0u;																											//	���÷�תֵ
//			TIM8->DIER  = TIM_DIER_CC1IE;																					//	ʹ��1���ж�
//			CLEAR_BIT( TIM8->SR, TIM_SR_UIF | TIM_SR_CC4IF );											//	����жϱ��
//			SET_BIT( TIM8->BDTR, TIM_BDTR_MOE );
//			SET_BIT( TIM8->CR1, TIM_CR1_CEN );																		//	ʹ�ܼ�����
//		}
//	
//  /******************************** ����˵�� *************************************
//  *	��е��ָPWM����
//  *******************************************************************************/
//		void	Fingers_CTRL( uint16_t ARRValue, uint16_t CCR1Value )
//		{
//			TIM8->ARR   =  ARRValue;					//	����Ԥװ��ֵ
//			TIM8->CCR4  =  CCR1Value;					//	���÷�תֵ
//		}
//	
//  /******************************** ����˵�� *************************************
//  *	��е��ָPWM����
//  *******************************************************************************/
// 		void	Fingers_CMD( _Bool	CMD )
//		{
//			if( CMD )
//			{
//				SET_BIT( TIM8->CCER,TIM_CCER_CC4E );																	//	ʹ��ͨ��1
//				SET_BIT( TIM8->CR1, TIM_CR1_CEN   );																	//	ʹ�ܼ�����
//			}
//			else
//			{
//				CLEAR_BIT( TIM8->CCER,TIM_CCER_CC4E );																//	ʧ��ͨ��1
//				CLEAR_BIT( TIM8->CR1, TIM_CR1_CEN   );																//	ʧ�ܼ�����
//				TIM5->CNT = 0u;
//			}
//		}
// 
//	/******************************** ����˵�� *************************************
//	*	�жϴ�����
//	*******************************************************************************/
//		volatile	uint32_t	SetPluseCount;			//	��������������
//		volatile	uint32_t	PluseCount;					//	ʵ������������
//		volatile	_Bool	CountOver	= false;			//	�����������־
//		void	TIM8_IRQHandler( void )
//		{
//			CLEAR_BIT( TIM8->SR, TIM_SR_UIF | TIM_SR_CC4IF );												//	����жϱ��
//			if( PluseCount < SetPluseCount )
//				PluseCount ++;
//			else
//			{
//				Fingers_CMD( false );
//				CountOver = true;
//				CLEAR_BIT( TIM8->SR, TIM_SR_UIF | TIM_SR_CC4IF );												//	����жϱ��
//			}
//		}
//	/******************************** ����˵�� *************************************
//	*	���������� ��е��ָ�˶����� 0 �� 1 ��
//	*******************************************************************************/
//		void	Fingers_DIR( _Bool	State )
//		{
//			if( State )
//				SET_BIT( GPIOA->BSRR, GPIO_BSRR_BS12 );
//			else
//				SET_BIT( GPIOA->BSRR, GPIO_BSRR_BR12 );
//		}
//	
//  /******************************** ����˵�� *************************************
//  *	��е��ָ�������
//  *******************************************************************************/
//  	void	BIOS_Fingers_PWMIn_Init( void )
//		{
//			SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPAEN );
//			MODIFY_REG( GPIOA->CRH, 0x000F0000u, 0x00040000u );										//	IO������		PWM_OUT
//			SET_BIT( RCC->APB2ENR, RCC_APB2ENR_TIM1EN );													//	��ʱ��
//			SET_BIT( TIM1->EGR, TIM_EGR_UG );																			//	��ʼ�����мĴ���
//			TIM1->SMCR =  TIM_SMCR_ECE;	//	���ù���ģʽ �ⲿʱ��2ģʽ
//			SET_BIT( TIM1->BDTR, TIM_BDTR_MOE );
//			SET_BIT( TIM1->CR1, TIM_CR1_CEN );																		//	ʹ�ܼ�����
//		}

	/********************************** ����˵�� *************************************
  *	�����PWM����
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
//					TIM8->ARR  = Value;																										//	����Ԥװ��ֵ
//					TIM8->CCR1 = Value / 2;			
//					SET_BIT( TIM8->CR1, TIM_CR1_CEN );																		//	ʹ�ܼ�����
//				}
//				else
//				{
//					CLEAR_BIT( TIM8->CR1, TIM_CR1_CEN );																		//	ʹ�ܼ�����
//				
//				}
//				LastValue = Value;
//			}
//		}
//	/********************************** ����˵�� *************************************
//  *	������PWM �˿ڳ�ʼ��
//  *********************************************************************************/
//  	void	BIOS_Motor_Init( void )
//		{
//			SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPAEN );
//			MODIFY_REG( GPIOA->CRH, 0x0000000Fu, 0x0000000Fu );										//	IO������		PWM_OUT
////			SET_BIT( GPIOC->BSRR, GPIO_BSRR_BS6 );
//			SET_BIT( RCC->APB2ENR, RCC_APB2ENR_TIM1EN );													//	��ʱ��
//			SET_BIT( TIM1->EGR, TIM_EGR_UG );																			//	��ʼ�����мĴ���
//			TIM1->CR1   = TIM_CR1_ARPE | TIM_CR1_URS;															//	�Զ�װ�� ������ж�
//			TIM1->PSC		= SystemCoreClock / 100000u - 1u;													//	����Ԥ��Ƶֵ
//			TIM1->ARR   = 30u - 1u;																							//	����Ԥװ��ֵ
//			TIM1->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE;	//	���ü������Ժ͹���ģʽ
//			TIM1->CCER  = TIM_CCER_CC1E |TIM_CCER_CC1P;														//	ʹ��ͨ��1
//			TIM1->CCR1  = 0u;																											//	���÷�תֵ
////			TIM8->DIER  = TIM_DIER_CC1IE;																					//	ʹ��1���ж�
////			CLEAR_BIT( TIM8->SR, TIM_SR_UIF | TIM_SR_CC1IF );											//	����жϱ��
//			SET_BIT( TIM1->BDTR, TIM_BDTR_MOE );
//			SET_BIT( TIM1->CR1, TIM_CR1_CEN );																		//	ʹ�ܼ�����

//		}
//	/********************************** ����˵�� *************************************
//  *	�����PWM����
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
//					TIM1->ARR  = Value;																										//	����Ԥװ��ֵ
//					TIM1->CCR1 = Value / 2;			
//					SET_BIT( TIM1->CR1, TIM_CR1_CEN );																		//	ʹ�ܼ�����
//				}
//				else
//				{
//					CLEAR_BIT( TIM1->CR1, TIM_CR1_CEN );																		//	ʹ�ܼ�����
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
/******************************** ����˵�� *************************************
*	��������
*******************************************************************************/
	/******************************** ����˵�� *************************************
	*	����1�˿�����
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
	/******************************** ����˵�� *************************************
	*	���� USART1��ʵ��RS232�շ�
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
	/******************************** ����˵�� *************************************
	*	���� USART1��ʵ��RS232�շ�
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
	
	/******************************** ����˵�� *************************************
	*	����2�˿�����
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
	/******************************** ����˵�� *************************************
	*	���� USART2��ʵ��RS232�շ�
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
	/******************************** ����˵�� *************************************
	*	���� USART2��ʵ��RS232�շ�
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
	
	/******************************** ����˵�� *************************************
	*	����3�˿�����
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
	/******************************** ����˵�� *************************************
	*	���� USART3��ʵ��RS232�շ�
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
	/******************************** ����˵�� *************************************
	*	���� USART3��ʵ��RS232�շ�
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
	
//	/******************************** ����˵�� *************************************
//	*	����4�˿�����
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
//	/******************************** ����˵�� *************************************
//	*	����5�˿�����
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
	




/******************************** ����˵�� *************************************
*	IO����
*******************************************************************************/
//	/******************************** ����˵�� *************************************
//	*	��ŷ����ƶ˿ڳ�ʼ��
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
//  /******************************** ����˵�� *************************************
//  *	��������������ͨ��
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
//  /******************************** ����˵�� *************************************
//  *	�ſ�������ͨ��
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
//  /******************************** ����˵�� *************************************
//  *	����ˮע����л���
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
/******************************** ����˵�� *************************************
*	IO������
*******************************************************************************/
//    /******************************** ����˵�� *************************************
//    *	Һλ�������
//    *******************************************************************************/
//			void	LEVELSWITCH_Init( void )
//			{
//				SET_BIT( RCC->APB2ENR, RCC_APB2ENR_IOPAEN );
//				MODIFY_REG( GPIOA->CRL, 0x0FF00000u, 0x04400000u );										//	IO������		PWM_OUT
//				
//			}
//		
//    /******************************** ����˵�� *************************************
//    *	Һλ���1
//    *******************************************************************************/
//    _Bool	LevelSwitch1Read( void )
//		{		
//			return	READ_BIT( GPIOA->IDR, GPIO_IDR_IDR6 );
//		}
//   /******************************** ����˵�� *************************************
//    *	Һλ���2
//    *******************************************************************************/
//    _Bool	LevelSwitch2Read( void )
//		{		
//			return	READ_BIT( GPIOA->IDR, GPIO_IDR_IDR2 );
//		}
//	
		
/******************************** ����˵�� *************************************
*	���Խӿ�����
*******************************************************************************/
	void	BIOS_SWD_Init( void )
	{
		SET_BIT( RCC->APB2ENR, RCC_APB2ENR_AFIOEN );
		MODIFY_REG( AFIO->MAPR, AFIO_MAPR_SWJ_CFG, AFIO_MAPR_SWJ_CFG_JTAGDISABLE );
	}

/******************************** ����˵�� *************************************
*	ϵͳ�����쳣�����ϵ����ȼ��д���
*******************************************************************************/
	void	BIOS_NVIC_Init( void )
	{
		NVIC_SetPriorityGrouping( 7 );	//	��ϵͳ����Ϊ��ֹ�ж���ռ����û���ж�Ƕ�� ����ͼ
		
		/*
			��7.4 ��ռ���ȼ��������ȼ��ı�λ�������λ�õĹ�ϵ
		
			����λ��    �����ռ���ȼ���λ��    ��������ȼ���λ��
				0            [7:1]                 [0:0]
				1            [7:2]                 [1:0]
				2            [7:3]                 [2:0]
				3            [7:4]                 [3:0]
				4            [7:5]                 [4:0]
				5            [7:6]                 [5:0]
				6            [7:7]                 [6:0]
				7            ��                    [7:0]������λ��
		*/
		
		//	__set_CONTROL( 0x03u );		//	�л���PSP, ��ת�����Ȩģʽ
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

/********  (C) COPYRIGHT 2018 �ൺ���ػ����������޹�˾  **** End Of File ********/
