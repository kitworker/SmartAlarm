#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"
#include "misc.h"
 
volatile char buffer[50] = {'\0'};
 
void usart_init(void)
{
    /* Enable USART1 and GPIOA clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
 
    /* NVIC Configuration */
    NVIC_InitTypeDef NVIC_InitStructure;
    /* Enable the USARTx Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
 
    /* Configure the GPIOs */
    //GPIO_Configuration();
    GPIO_InitTypeDef GPIO_InitStructure;
 
    /* Configure USART1 Tx (PA.09) as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
 
    /* Configure USART1 Rx (PA.10) as input floating */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
 
    /* Configure the USART1 */
    //USART_Configuration();
    USART_InitTypeDef USART_InitStructure;
 
    /* USART1 configuration ------------------------------------------------------*/
    /* USART1 configured as follow:
              - BaudRate = 115200 baud
              - Word Length = 8 Bits
              - One Stop Bit
              - No parity
              - Hardware flow control disabled (RTS and CTS signals)
              - Receive and transmit enabled
              - USART Clock disabled
              - USART CPOL: Clock is active low
              - USART CPHA: Data is captured on the middle
              - USART LastBit: The clock pulse of the last data bit is not output to
                               the SCLK pin
     */
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
 
    USART_Init(USART1, &USART_InitStructure);
 
    /* Enable USART1 */
    USART_Cmd(USART1, ENABLE);
 
    /* Enable the USART1 Receive interrupt: this interrupt is generated when the
        USART1 receive data register is not empty */
    //USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
 
}
 
void USARTSend(const unsigned char *pucBuffer, unsigned long ulCount)
{
    //
    // Loop while there are more characters to send.
    //
    while(ulCount--)
    {
        USART_SendData(USART1, *pucBuffer++);// Last Version USART_SendData(USART1,(uint16_t) *pucBuffer++);
        /* Loop until the end of transmission */
        while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
        {
        }
    }
}
 
void SetSysClockTo72(void)
{
    ErrorStatus HSEStartUpStatus;
    /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration -----------------------------*/
    /* RCC system reset(for debug purpose) */
    RCC_DeInit();
 
    /* Enable HSE */
    RCC_HSEConfig( RCC_HSE_ON);
 
    /* Wait till HSE is ready */
    HSEStartUpStatus = RCC_WaitForHSEStartUp();
 
    if (HSEStartUpStatus == SUCCESS)
    {
        /* Enable Prefetch Buffer */
        //FLASH_PrefetchBufferCmd( FLASH_PrefetchBuffer_Enable);
 
        /* Flash 2 wait state */
        //FLASH_SetLatency( FLASH_Latency_2);
 
        /* HCLK = SYSCLK */
        RCC_HCLKConfig( RCC_SYSCLK_Div1);
 
        /* PCLK2 = HCLK */
        RCC_PCLK2Config( RCC_HCLK_Div1);
 
        /* PCLK1 = HCLK/2 */
        RCC_PCLK1Config( RCC_HCLK_Div2);
 
        /* PLLCLK = 8MHz * 9 = 72 MHz */
        RCC_PLLConfig(0x00010000, RCC_PLLMul_9);
 
        /* Enable PLL */
        RCC_PLLCmd( ENABLE);
 
        /* Wait till PLL is ready */
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
        {
        }
 
        /* Select PLL as system clock source */
        RCC_SYSCLKConfig( RCC_SYSCLKSource_PLLCLK);
 
        /* Wait till PLL is used as system clock source */
        while (RCC_GetSYSCLKSource() != 0x08)
        {
        }
    }
    else
    { /* If HSE fails to start-up, the application will have wrong clock configuration.
     User can add here some code to deal with this error */
 
        /* Go to infinite loop */
        while (1)
        {
        }
    }
}

void initADC(){
	    //ADC
     //ADC
    ADC_InitTypeDef ADC_InitStructure;
    GPIO_InitTypeDef  GPIO_InitStructure;
    // input of ADC (it doesn't seem to be needed, as default GPIO state is floating input)
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4 ;        // that's ADC1 (PA1 on STM32)
    GPIO_Init(GPIOA, &GPIO_InitStructure);
 
    //clock for ADC (max 14MHz --> 72/6=12MHz)
    RCC_ADCCLKConfig (RCC_PCLK2_Div6);
    // enable ADC system clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
 
    // define ADC config
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;  // we work in continuous sampling mode
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
 
    ADC_RegularChannelConfig(ADC1,ADC_Channel_4, 1,ADC_SampleTime_28Cycles5); // define regular conversion config
    ADC_Init ( ADC1, &ADC_InitStructure);   //set config of ADC1
 
    // enable ADC
    ADC_Cmd (ADC1,ENABLE);  //enable ADC1
 
    //  ADC calibration (optional, but recommended at power on)
    ADC_ResetCalibration(ADC1); // Reset previous calibration
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1); // Start new calibration (ADC must be off at that time)
    while(ADC_GetCalibrationStatus(ADC1));
 
    // start conversion
    ADC_Cmd (ADC1,ENABLE);  //enable ADC1
    ADC_SoftwareStartConvCmd(ADC1, ENABLE); // start conversion (will be endless as we are in 
 
}

void adc_init()
{
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

 // ????????? ADC
 ADC_InitTypeDef ADC_InitStructure;
 ADC_StructInit(&ADC_InitStructure);
 ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; // ????? ?????? - ?????????, ???????????
 ADC_InitStructure.ADC_ScanConvMode = DISABLE; // ?? ??????????? ??????, ?????? ???????? ???? ?????
 ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; // ??????????? ?????????
 ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // ??? ???????? ????????
 ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; //???????????? ????? ????????? - ??????? ??????
 ADC_InitStructure.ADC_NbrOfChannel = 1; //?????????? ??????? - ???? ?????
 ADC_Init(ADC1, &ADC_InitStructure);
 ADC_Cmd(ADC1, ENABLE);
 
 // ????????? ??????
 ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_55Cycles5);
 
 // ?????????? ???
 ADC_ResetCalibration(ADC1);
 while (ADC_GetResetCalibrationStatus(ADC1));
 ADC_StartCalibration(ADC1);
 while (ADC_GetCalibrationStatus(ADC1)); 
 
     // start conversion
    ADC_Cmd (ADC1,ENABLE);  //enable ADC1
    ADC_SoftwareStartConvCmd(ADC1, ENABLE); // start conversion (will be endless as we are in continuous mode)

}



	
void initGPIO(){
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
	
	GPIO_InitTypeDef PORT;
	
	PORT.GPIO_Pin = (GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);
  PORT.GPIO_Mode = GPIO_Mode_AF_PP;
  PORT.GPIO_Speed = GPIO_Speed_2MHz;
	
	
  GPIO_Init(GPIOA, &PORT);
}

void timer_init(int prescaller){
	
	initGPIO();
	TIM_TimeBaseInitTypeDef TIM_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	
	TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_InitStructure.TIM_Prescaler = 24 - 1;
	TIM_InitStructure.TIM_Period = 10000 - 1; // Update event every 10000 us / 10 ms
	TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_InitStructure.TIM_RepetitionCounter = 0;
	
	TIM_TimeBaseInit(TIM2, &TIM_InitStructure);
	
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	
	TIM_Cmd(TIM2, ENABLE);  // ON TIMER
	
                     
 
}  // RUN

int adc_value;
void delay(void) {
	
  volatile uint32_t i;
    for (i=1; i != 0xFF0; i++);
	// for (i=1; i != 0xFFF0; i++);
	
  }

void initPwd(){
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
 
  //GPIO_InitTypeDef PORT;
  // ???????? ???? ?? ???????????? ?? ?????
  //PORT.GPIO_Pin = (GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);
  //????? ???????????? ?????????????? ????? ? ?? ??????? GPIO
  //PORT.GPIO_Mode = GPIO_Mode_AF_PP;
  //PORT.GPIO_Speed = GPIO_Speed_2MHz;
  //GPIO_Init(GPIOA, &PORT);
	
  //????????? ??????? ???????????? ???? PA1,PA2,PA3 ??? ????
  TIM2->CCER |= (TIM_CCER_CC2E|TIM_CCER_CC3E|TIM_CCER_CC4E);
  // ??? ???? ???? ??????? ?????? ????????? ???.
  TIM2->CCMR1|=(TIM_CCMR1_OC2M_0| TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2);
  TIM2->CCMR2|=(TIM_CCMR2_OC3M_0 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 |
								TIM_CCMR2_OC4M_0 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2);
  //????????? ??????!
  TIM2->CR1 |= TIM_CR1_CEN;
  //????? ????? ????? ?????? ? TIM2->CCRx - ? ??????? ??????????? ????????
}


int del = 0;

uint32_t PWM = 0;
int main(void)
{
  //??????? ???? ?
 // RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
  //???????? ?????? 2
	initGPIO();
  initPwd();
	initADC();
 
  uint32_t pwm_arr[]={0,0,6553,13107,19660,26214,32768,
                                  39321,45875,52428,58982,65535};
 
  uint32_t i;
  uint32_t mid ;
	uint32_t const MIN = 1000;
	uint32_t const MAX = 65534;
  while(1)
  {

//   			if(adc_value < 1300 ) {
//					
//						PWM++;
//					 TIM2->CCR3=PWM;
//				} else if ( adc_value > 1305 ) {
//					if(PWM > 0) 
//						PWM--;
// 					 TIM2->CCR3=PWM;
//				} else {
//					del++;
//				}
      
			//  delay();
		
		uint32_t adc;
		for(mid = 0; mid < 10000; ++mid){
				adc += ADC_GetConversionValue(ADC1);
		}
		
		
		adc_value = adc /10000;
		adc = 0;
		
		
		int val = 20;
		int step = 500;
		int istep = 200;
  
			if ( adc_value < val  ) {
				if(i > MIN )
					i-=istep;
				else
					i = 0;
			} else if ( adc_value > (val+1)) {
				if( i <= MAX)
					i+=istep;
				else
					i = 10;
			} else {
				int x = 65535;
			}
			
			
			if( i > (65535 - step)  ){
				if (i < 65535)
					i++;
			}
			
			if( i <= 0  )
				i = 0;
     
			TIM2->CCR3=i;//pwm_arr[i];
    //  delay();
    
//    for (i=1;i<=10;i++) {
//      TIM2->CCR2=pwm_arr[i];
//      delay();
//    }
//    for (i=11;i>=1;i--) {
//      TIM2->CCR2=pwm_arr[i];
//      delay();
//    }
//    for (i=1;i<=10;i++) {
//      TIM2->CCR4=pwm_arr[i];
//      delay();
//    }
//    for (i=11;i>=1;i--) {
//      TIM2->CCR4=pwm_arr[i];
//      delay();
//    }
		
		
  }
}