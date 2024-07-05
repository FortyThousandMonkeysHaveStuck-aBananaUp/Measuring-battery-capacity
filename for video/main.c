#define STM32F103xB
#include "stm32f1xx.h"

//Fatal Error[Pe1696]: cannot open source file "stm32f103xb.h" C:\Users\Pz\Desktop\Education STM_C-C++\0_Testing\CMSIS_learning_2\for video\Different files for this project\stm32f1xx.h 131 
//Fatal Error[Pe1696]: cannot open source file "core_cm3.h" C:\Users\Pz\Desktop\Education STM_C-C++\0_Testing\CMSIS_learning_2\for video\Different files for this project\stm32f103xb.h 132 
//Fatal Error[Pe1696]: cannot open source file "cmsis_version.h" C:\Users\Pz\Desktop\Education STM_C-C++\0_Testing\CMSIS_learning_2\for video\Different files for this project\core_cm3.h 63 
//Fatal Error[Pe1696]: cannot open source file "cmsis_compiler.h" C:\Users\Pz\Desktop\Education STM_C-C++\0_Testing\CMSIS_learning_2\for video\Different files for this project\core_cm3.h 115 
//Fatal Error[Pe1696]: cannot open source file "cmsis_iccarm.h" C:\Users\Pz\Desktop\Education STM_C-C++\0_Testing\CMSIS_learning_2\for video\Different files for this project\cmsis_compiler.h 55 
//Fatal Error[Pe1696]: cannot open source file "system_stm32f1xx.h" C:\Users\Pz\Desktop\Education STM_C-C++\0_Testing\CMSIS_learning_2\for video\Different files for this project\stm32f103xb.h 133 

//Function declarations
void System_Clock_Config(void);
void GPIO_PA9(void);
void TIM1_Config(void);
void ADC2_Config(void);
void ADC2_Start(char channel);
void USER_delay(unsigned long wait);

//Varaibles
  //voltage
  float voltage=0;
  unsigned int sum_of_voltages=0;
  
  //current
  unsigned int sum_of_currents=0;
  
  //another things
  unsigned int capacity=0;
  unsigned int seconds_counter=0;
  unsigned int ADC2_VAL[2]={0, 0};
  float voltage_constant=6.8204/10000;// V/step
    
//Flags
char the_capacity_has_already_been_recalculated_flag=0;
char ADC2_Flag=0;

//Interrupts
//TIM1
void TIM1_UP_IRQHandler(void)
{
  TIM1->SR=0;
  
  ADC2_Flag=0;
  
  ADC2_Start(0);
}

//ADC2
void ADC1_2_IRQHandler(void)
{
  if(ADC2->SR&ADC_SR_EOC)
  {
    if(ADC2_Flag==1)
    {
      ADC2_VAL[1]=ADC2->DR;
      the_capacity_has_already_been_recalculated_flag=0;
      seconds_counter++;
    }else if(ADC2_Flag==0)
      {
        ADC2_VAL[0]=ADC2->DR;
        ADC2_Flag=1;
        ADC2_Start(1);
      }
  }
}

void main(void)
{
//config
  //System clock
  System_Clock_Config();
  
  //GPIO PA9
  GPIO_PA9();
  
  //TIM1
  TIM1_Config();
  
  //ADC2
  ADC2_Config();
  
  //Switch on
  GPIOA->BSRR|=GPIO_BSRR_BR9;
  
while(1)
{
  GPIOA->BSRR|=GPIO_BSRR_BR9;
  if((seconds_counter!=0)&&(the_capacity_has_already_been_recalculated_flag==0))
  {
    the_capacity_has_already_been_recalculated_flag=1;
    //voltages
    sum_of_voltages+=ADC2_VAL[1];
    
    //currents
    sum_of_currents+=(ADC2_VAL[0]*voltage_constant/1)*1000; //I=V/R R=1 Ohm (*1000->mA) [mAs]
      
    if(seconds_counter%8==0)
    {
      //voltage
      voltage=2*(sum_of_voltages/8)*voltage_constant; //V
      sum_of_voltages=0;
      
      if(voltage<=2.9)
      {
        //Switch off
        GPIOA->BSRR|=GPIO_BSRR_BS9;
        
        //Disable TIM1 interrutps
        TIM1->DIER&=~TIM_DIER_UIE;//0: Update interrupt disabled.
        
        //Calculate the battery capacity
        capacity=sum_of_currents/3600; //[mAh]
      }
    }
  }
}

}

//Functions
void System_Clock_Config(void)
{
/* Steps to follow*/
//1 Flash Latency
//2 Enable HSE and wait for
//3.1 PLL miltiplication factor
//3.2 PLL entry clock source
//4 Enable PLL and wait for
//5 Set the prescalers HCLK, PCLK1, PCLK2
//6 Set SW and wait for something  
  
//1 Flash Latency
FLASH->ACR|=FLASH_ACR_LATENCY_1; 

//2 Enable HSE and wait for
RCC->CR|=RCC_CR_HSEON;
while(!(RCC->CR&RCC_CR_HSERDY));

//3.1 PLL miltiplication factor
RCC->CFGR|=RCC_CFGR_PLLMULL9;

//3.2 PLL entry clock source
RCC->CFGR|=RCC_CFGR_PLLSRC;

//4 Enable PLL and wait for
RCC->CR|=RCC_CR_PLLON;
while(!(RCC->CR&RCC_CR_PLLRDY));

//5 Set the prescalers HCLK, PCLK1, PCLK2
//AHB
RCC->CFGR&=~RCC_CFGR_HPRE;

//APB1
RCC->CFGR|=RCC_CFGR_PPRE1_DIV2;

//APB2
RCC->CFGR&=~RCC_CFGR_PPRE2;

//6 Set SW and wait for something  
RCC->CFGR|=RCC_CFGR_SW_PLL;
while(!(RCC->CFGR&RCC_CFGR_SWS_PLL));
}

void GPIO_PA9(void)
{
/* Steps to follow*/
//1 Enable GPIO clock
//2 Reset PA
//3 Mode
//4 Config  
  
//1 Enable GPIO clock
RCC->APB2ENR|=RCC_APB2ENR_IOPAEN;

//2 Reset PA
GPIOA->BRR=0xffff;

//3 Mode
GPIOA->CRH|=GPIO_CRH_MODE9;//11: Output mode, max speed 50 MHz.

//4 Config 
GPIOA->CRH&=~GPIO_CRH_CNF9;//00: General purpose output push-pull
}

void TIM1_Config(void)
{
/* Steps to follow*/
//1 Enable TIM1 clock
//2 Set the prescaler and ARR
//3 Enable ARP
//4 Enable the counter of the timer
//5 Enable update interrupts
//6 Set the interrupt priority
//7 Enable the interrupt
  
//1 Enable TIM1 clock
RCC->APB2ENR|=RCC_APB2ENR_TIM1EN;

//2 Set the prescaler and ARR
//72 000 000 = 7200 * 10000
TIM1->PSC=7200-1;
TIM1->ARR=10000-1;

//3 Enable ARP
TIM1->CR1|=TIM_CR1_ARPE;

//4 Enable the counter of the timer
TIM1->CR1|=TIM_CR1_CEN;

//5 Enable update interrupts
TIM1->DIER|=TIM_DIER_UIE;//1: Update interrupt enabled.

//6 Set the interrupt priority
NVIC_SetPriority(TIM1_UP_IRQn, 1);

//7 Enable the interrupt
NVIC_EnableIRQ(TIM1_UP_IRQn);
}

void ADC2_Config(void)//PA0 and PA1
{
/* Steps to follow*/
//1 Enable ADC2 clock and GPIO clock
//2 Set the prescaler APB2/6
//3 Set the sampling time
//4 Set the sequense lenght
//5 Data alignment
//6 Set respective GPIO PINs in the analog mode
//7 Enable end of conversion interrutps
//8 Set the interrutp priority
//9 Enable the interrupt
//10 Calibrate the ADC2  
//11 Enable the ADC2  

//1 Enable ADC2 clock and GPIO clock
RCC->APB2ENR|=RCC_APB2ENR_ADC2EN;
RCC->APB2ENR|=RCC_APB2ENR_IOPAEN;

//2 Set the prescaler APB2/6
RCC->CFGR|=RCC_CFGR_ADCPRE_DIV6;

//3 Set the sampling time
ADC2->SMPR2|=ADC_SMPR2_SMP0;
ADC2->SMPR2|=ADC_SMPR2_SMP1;

//4 Set the sequense lenght
ADC2->SQR1&=~ADC_SQR1_L;//0000: 1 conversion

//5 Data alignment
ADC2->CR2&=~ADC_CR2_ALIGN;//0: Right Alignment

//6 Set respective GPIO PINs in the analog mode
//PA0
GPIOA->CRL&=~GPIO_CRL_MODE0;//00: Input mode (reset state)
GPIOA->CRL&=~GPIO_CRL_CNF0;//00: Analog input mode

//PA1
GPIOA->CRL&=~GPIO_CRL_MODE1;//00: Input mode (reset state)
GPIOA->CRL&=~GPIO_CRL_CNF1;//00: Analog input mode

//7 Enable end of conversion interrutps
ADC2->CR1|=ADC_CR1_EOCIE;

//8 Set the interrutp priority
NVIC_SetPriority(ADC1_2_IRQn, 1);

//9 Enable the interrupt
NVIC_EnableIRQ(ADC1_2_IRQn);

//10 Calibrate the ADC2
ADC2->CR2&=~ADC_CR2_ADON;
ADC2->CR2|=ADC_CR2_CAL;
while(ADC2->CR2&ADC_CR2_CAL);

//11 Enable the ADC2
ADC2->CR2|=ADC_CR2_ADON;
USER_delay(500); //wait approx 10 us
}

void USER_delay(unsigned long wait)
{while(--wait);}

void ADC2_Start(char channel)
{
/* Steps to follow*/
//1 Reset the ADC2 register
//2 Select the channel
//3 Start the ADC2 conversion
  
//1 Reset the ADC2 register
ADC2->SR=0;

//2 Select the channel
ADC2->SQR3=channel;

//3 Start the ADC2 conversion
ADC2->CR2|=ADC_CR2_ADON;
}