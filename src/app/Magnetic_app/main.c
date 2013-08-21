/*************************************************************************
  main.c
  HKUST Smartcar 2013 manetic Group

  Authoured by:
  Ben Lai (copy from sensor group)

  Hardware By:
  Zyan Shao
  Yu Fank
  Michael Pang
*************************************************************************/

#include "common.h"
#include "include.h"
#include "stdlib.h"

/*************************************************************************
Function header
*************************************************************************/

void motor_init(void);
void pitexti_init(void);
void ftm2_init ();
/*************************************************************************
Main
*************************************************************************/





void main()
{ 
  LED_init();
  adc_init(ADC0,SE15);     //For  sensor 
  adc_init(ADC0,SE14); 
  

  uart_init(UART3, 115200); // For  bluetooth
  
  motor_init();            //For Sevor and motor
   
  ftm2_init ();            //encoder
  
  pitexti_init();           //For pit and exti
  gpio_init(PORTE, 6, GPI, HIGH);  //Switch
  gpio_init(PORTE, 8, GPI, HIGH); 
  gpio_init(PORTE, 9, GPI, HIGH);  
 
  

  while(1){}
  
}












void motor_init()
{
    FTM_PWM_init(FTM0,CH1,50,750); //sevor_init
    time_delay_ms(600);
    
    FTM_PWM_Duty(FTM0,CH1,920);
    time_delay_ms(600);
    
    FTM_PWM_Duty(FTM0,CH1,560);
    time_delay_ms(600);
    
    FTM_PWM_Duty(FTM0,CH1,750);
   
    gpio_init(PORTD,9,GPO,1);         //motor_init
    
    FTM_PWM_init(FTM1,CH1,10000,0);

}
    



void pitexti_init()
{
  DisableInterrupts;
  
  exti_init(PORTC,11,either_down);   //For reed switch reading

  pit_init_ms(PIT3,1);            //For init pit
  
  EnableInterrupts;

}




void ftm2_init ()
{   
        SIM_SCGC3 |= SIM_SCGC3_FTM2_MASK;
        SIM_SCGC5 |=   SIM_SCGC5_PORTB_MASK;
        PORTB_PCR18 =(0|PORT_PCR_MUX(6));        // set PTB18,PTB19 be quadrature mode
        PORTB_PCR19 =(0|PORT_PCR_MUX(6));
        FTM2_SC = 0x00;                           // off counter, interrupt
        FTM2_CNT = 0x0000;                         // initialise counter
        FTM_CNTIN_REG(FTM2_BASE_PTR) = 0x0000;               
        FTM_MODE_REG(FTM2_BASE_PTR) |= (FTM_MODE_WPDIS_MASK|FTM_MODE_FTMEN_MASK);   //WPDIS =1, FTMEN=1
        FTM_MOD_REG(FTM2_BASE_PTR) = 0xFFFF;       // maximum count = 65535
        FTM2_QDCTRL = 0x0F;                        // B = vcc, A = sig input
}
