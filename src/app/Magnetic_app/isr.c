#include "common.h"
#include "include.h"

/******************************************
 Variables for control PIT
******************************************/

u8 system_mode=0;

u16 encoder20ms; 
u16 final_speed=2400;

int ADvalue_left;   //ADC variable
int ADvalue_right;


/******************************************
 change value Variables
******************************************/
u16 ideal_speed;
u8 trankp;
u8 leftkp;
u8 rightkp;
u8 leftkd;
u8 rightkd;



//variables//
int tempk=750;
int errork;
int absoluterrork;
int error;


u16 i;    
int k;
int q;
u16 s=0;

u8 h=0;
u16 e=0;
u16 t=0;
u8 rsw=0;   //reed switch


void reed_switch(void){ 

    u8  n=0;
    
    n=11;
    if(PORTC_ISFR & (1<<n))
    {
        PORTC_ISFR  |= (1<<n);
        rsw++;
        if(rsw==1){
          LED_turn(LED0);}  
        if(rsw==2){
          LED_turn(LED1);}  
        if(rsw==3){
          LED_turn(LED2);}  
        if(rsw==4){
          LED_turn(LED3);}  
        
    }
}


void pit3_system_loop(void)
{

  switch (system_mode)
  {  case 0:
    if(gpio_get(PORTE, 6) == 0)
     {
      system_mode=99;
     }
    if(gpio_get(PORTE, 8) == 0)
     {system_mode=98;}
    if(gpio_get(PORTE, 9) == 0)
     {system_mode=97;}
    if(e<5000){e++;system_mode=0;   //wait 5 sec
    }
   
  
    
    
     break;
    
    
      case 99:
      
             ideal_speed=300;
                   trankp=14;
                   leftkp=12;
                   rightkp=13;
                   leftkd=8;   
                   rightkd=6; 
                   system_mode=100;
                   break;
  case 98:            
             ideal_speed=300;
                   trankp=13;
                   leftkp=13;
                   rightkp=11;
                   leftkd=9;   
                   rightkd=7; 
                   system_mode=100;
        
                         break;
  case 97:                
        ideal_speed=180;
              trankp=18;
                   leftkp=18;
                   rightkp=14;
                   leftkd=11;   
                   rightkd=9;
              system_mode=100;
                   break;
  case 100:
       if(t<3000){t++;          //wait 3 sec
       system_mode=100;
      }
      else{
        {system_mode=1;
         FTM_PWM_Duty(FTM1,CH1,3800);
   }}
      break;
      case 1:
      case 2:       
      case 3:        
      case 4:
      case 5: 
      case 6:
      case 7:
      case 8:        
      case 9:    
      case 10:
      case 11:
      case 12:
      case 13:
      case 14:
      case 15:
      case 16:       
      case 17:
      case 18:
      case 19:  
      case 20:
 
      
      //get sensor advalue
      for(i = 0; i < 6; i++){
      ADvalue_left  += ad_max(ADC0,SE14,ADC_16bit,20);
      ADvalue_right += ad_max(ADC0,SE15,ADC_16bit,20);}
      ADvalue_left=ADvalue_left/6;
      ADvalue_right=ADvalue_right/6;
            
      //decide turn left or turn right
      ADvalue_left=ADvalue_left-41500;
      ADvalue_right=ADvalue_right-42000;
            
      if(ADvalue_left<0){ADvalue_left=1;}  //avoid 0/0
      if(ADvalue_right<0){ADvalue_right=1;}
      
      
      k=ADvalue_right-ADvalue_left;  // easy to go straight
       if(k<1400 && k>-1400  && ADvalue_left>6000 && ADvalue_right>6000)
      {k=750;
      
      FTM_PWM_Duty(FTM0,CH1,k);
      tempk=k;
      system_mode=system_mode+1;
           
      ADvalue_left=0;       // set ad value to zero
      ADvalue_right=0;
      break;
      }
      


      
      
      q=ADvalue_right+ADvalue_left;

      k=k*1000;
      k=k/q;
      if(k>0){     
        k=k*k;}
      if(k<0){     
        k=-k*k;}   
      k=k/1000;
      k=k*trankp/100;
      
      
      if(k>0){k=k*rightkp/10;}
      if(k<0){k=k*leftkp/10;}
      k=750+k;
      if(k>910){k=909;}
      if(k<570){k=571;}
      errork=k-tempk;
   
      
     
      if(k>0){errork=errork*rightkd/10;}
      if(k<0){errork=errork*leftkd/10;}
      k=k+errork;
      if(k>910){k=909;}
      if(k<570){k=571;}
      
      
      absoluterrork=k-tempk;
           
      if(absoluterrork>50 || absoluterrork<-57){k=tempk;}
      
      FTM_PWM_Duty(FTM0,CH1,k);
      
     

      ADvalue_left=0;
      ADvalue_right=0;
      tempk=k;
        
      system_mode=system_mode+1;//go to next state on next cycle
      
     
      if(s<5000){s++;
      rsw=0;}
      else{
        if(rsw>1){system_mode=22;}}
      break;
    
      
      
      
      
      
      
      
      
      
      
      
     case 21: // speed control


      encoder20ms = FTM2_CNT;
     
      error = ideal_speed-encoder20ms ;
                 
      if(error==ideal_speed){error= error*7;}
      if(error>ideal_speed*7/10){error = error*4;}
      if(error>ideal_speed*9/20){error = error*2;}
      if(error>ideal_speed*1/5){error = error*1;}
      if(error>0){error = error*1/5;}
      if(error<-ideal_speed){error = error*7;}
      if(error<-ideal_speed*7/10){error = error*4;}
      if(error<-ideal_speed*9/20){error = error*2;}
      if(error<-ideal_speed*1/5){error = error*1;}
      if(error<0){error = error*1/5;}
           
      if(encoder20ms!=ideal_speed){final_speed = final_speed+error;}
      
      if(final_speed>8000){final_speed=7600;}
      
      if( encoder20ms>ideal_speed*2)
      {
        gpio_set(PORTD,9,0);
        final_speed= 2000;
      }
      
      else{gpio_set(PORTD,9,1);}
            
      FTM_PWM_Duty(FTM1,CH1,final_speed);
     
      FTM2_CNT=0; 

      
      system_mode=1;             //back to the top of pit
      if(s<5000){s++;
      rsw=0;}
      else{
        if(rsw>1){system_mode=22;}}
          
     
      break;
      
      case 22:
       
      FTM_PWM_init(FTM1,CH1,10000,0);
    
      break;
      }
    PIT_Flag_Clear(PIT3);

}
 
