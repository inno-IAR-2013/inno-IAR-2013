#include "common.h"
#include "include.h"

extern unsigned char data[ROW][COLUMN];
extern int speed_current;
extern u32 system_clock;
/*--*/
extern unsigned char p_counter;
extern int delaytime;
extern double middle_slope;
extern double middle_intercept;
extern char foundStopLinePattern;

int detect_stop(void);

/*-----------area cal----------------*/
int MAX_LENGTH = 60; //35
#define MIDDLE COLUMN/2
#define MAX_DIFF 10
int area_L = 0;
int area_R = 0;

/*-----------servo-------------------*/
//#define MAX_TURN 120  //right
//#define MIN_TURN 31   //left
#define MID_TURN 76 
#define L_TURN_RANGE 45
#define R_TURN_RANGE 45 
float servo_kp = 0;
float servo_kd = 0;
float servo_error = 0;
float servo_old_error = 0;
float servo_output = 0;
float servo_kp_turning = 1.2;


/*-----------speed-----------------*/
#define MAX_SPEED_OUTPUT 800
int speed_olderror = 0;
int speed_output = 300;
int speed_integral = 0;
  
int MAX_SPEED = 150 ;
int STD_SPEED = 120 ;
int MIN_SPEED = 90 ;
const float servo_coe = 0.7; 
const float length_coe = 0.5; 

int PID_choice(void){
    int i;
	for(i = 0; data[i][MIDDLE] == 0 && i < ROW; i++);
	if(i > 50){
		//straight
		servo_kp = 0.7; //1
		servo_kd = 0; //2.5 2
	}
	else{
		//turning
		servo_kp = servo_kp_turning; //3.5 2
		servo_kd = 0; //4.5 2.5
	}
	return i;
}

void servo_pid(float input){
	servo_error = input;
	servo_output = servo_kp * servo_error + servo_kd * (servo_error - servo_old_error);
	servo_old_error = servo_error;
	return;
}

void speed_pid(int speed_target)
{
	int speed_error = 0;
    speed_error = speed_target - speed_current;
    speed_output = speed_output + speed_error * 200/75 //200/75 35
                  + (speed_error-speed_olderror)*80/10 //6 1
                  + speed_integral * 0; //larger
    speed_olderror = speed_error; 
    speed_integral += speed_error;
      //printf("%d\n", speed_error);      
    if(speed_error < -10)
    	speed_output = speed_error*70/10;
    if(speed_output<0)
    {
    	speed_output = -1 * speed_output;
        gpio_set(PORTA, 10, 0);
    }
    else
    {
    	gpio_set(PORTA, 10, 1);
    }
    if(speed_output > MAX_SPEED_OUTPUT)
    {
        speed_output = MAX_SPEED_OUTPUT;
    }
}

void print(int input){
	if(p_counter%5 == 0){
		p_counter = 1;

	    DisableInterrupts;
		for(int i = 0; i < ROW; i++)
		{
			int counter=0;
			if(data[i][0] == 0) 
				printf("0 ");
			for (int j = 0; j < COLUMN; j++)
			{
				if(data[i][j-counter] == data[i][j])
					counter++;
				else
				{
					printf("%d ", counter);
					counter = 1;
				}
			}
			printf("%d ", counter);
			printf("\n");
		}
		EnableInterrupts;
//                printf("\"s = %d\"", (int)(30*middle_slope));
                printf("\"t = %d\"", delaytime);
//                printf("\"x = %d\"", input);
//		printf("\"s = %d; L = %d\"", (int)servo_error, input);
			printf("#");	
		}
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
	FTM_MODE_REG(FTM2_BASE_PTR) |= (FTM_MODE_WPDIS_MASK|FTM_MODE_FTMEN_MASK);   // WPDIS =1, FTMEN=1
	FTM_MOD_REG(FTM2_BASE_PTR) = 0xFFFF;       // maximum count = 65535
	FTM2_QDCTRL = 0x0F;                        // B = vcc, A = sig input
}

void initialization(void){
    /*--------motor PWM----------*/
    gpio_init(PORTA,10,GPO, 1);
    FTM_PWM_init (FTM1, CH1, 10000, 0);

    /*--------servo PWM----------*/
    FTM_PWM_init (FTM0, CH0, 50, MID_TURN);
	//camera input
    exti_init(PORTB,8, falling_down); //HS field, falling_down
    exti_init(PORTB,9, rising_down);  //VS line, rising_down
    gpio_init(PORTB,10,GPI, 1);       // GPIN for camera signal
    set_irq_priority(88, 1);

    /*-------speed input--------*/        
    pit_init_ms(PIT0, 10);
    set_irq_priority(68,0);
    ftm2_init();

    LED_init();
    uart_init(UART3, 115200);     // UART initialise

    gpio_init(PORTE,6,GPI,HIGH);//sw2  
    gpio_init(PORTE,7,GPI,HIGH);//sw3  
    gpio_init(PORTE,8,GPI,HIGH);//sw4  
    gpio_init(PORTE,9,GPI,HIGH);//sw5  
}

int speed_cal(int length, float servo_error,int old_length, float old_servo_error, int old_speed){ 
    float servo_abs = servo_error > 0 ? servo_error : -servo_error; 
    float old_servo_abs = old_servo_error > 0 ? old_servo_error : -old_servo_error; 
    float delta_speed = servo_coe * (old_servo_abs - servo_abs) + length_coe * (length - old_length); 
    int speed_tar = 0; 
    if(servo_abs <= 7 && old_servo_abs <= 7) 
        delta_speed += 10; 
    if(length > COLUMN-5 && old_length > COLUMN-5) 
        delta_speed += 5; 
    if(servo_abs >= 40 && old_servo_abs > 40) 
        delta_speed -= 10; 
    if(length < COLUMN/2 && old_length < COLUMN/2) //1st: COLUMN/2 + 10
        delta_speed -= 5; 
    speed_tar = (int)old_speed + delta_speed; 
    if(speed_tar > MAX_SPEED) 
        speed_tar = MAX_SPEED; 
    if(speed_tar < MIN_SPEED) 
        speed_tar = MIN_SPEED; 
    return speed_tar; 
} 

void main(void)
{
    int is_start = 0;      
    int mode = 11;  
	int speed_input;
	int length;
        int stop = 0;
        int round = 0;
        
        int old_speed_input = STD_SPEED; 
        int old_length = COLUMN; 

	DisableInterrupts
	initialization();
	EnableInterrupts
    
        u32 start_time; 
        u32 stop_time = 0; 
  
    while(1){     
        if(is_start == 0){  
            if(PTE9_IN == 0){  
                is_start = 1;  
                PTE25_OUT = 0;  
                start_time = system_clock; 
                while(system_clock - start_time < 150) 
                { 
                } 
                start_time = system_clock; 
            }  
            //mode selection  
            if(PTE7_IN == 0){  
                mode--;  
                time_delay_ms(500);  
                if(mode == 0)  
                    mode = 11;  
            }  
            if(PTE6_IN == 0){  
                mode++;  
                time_delay_ms(500);  
            }  
            switch(mode % 4){  
                case 0: PTE26_OUT = 0;  
                        PTE27_OUT = 1;  
 MAX_SPEED = 120 ;
 STD_SPEED = 100 ;
 MIN_SPEED = 80 ;
 servo_kp_turning = 1.1;
                     break;  
                case 1: PTE26_OUT = 1;  
                        PTE27_OUT = 0; 
 MAX_SPEED = 140 ; //130
 STD_SPEED = 120 ; //110
 MIN_SPEED = 90 ; //90
 servo_kp_turning = 1.1;
                    break;  
                case 2: PTE26_OUT = 0;  
                        PTE27_OUT = 0; 
 MAX_SPEED = 160 ;
 STD_SPEED = 140 ;
 MIN_SPEED = 100 ;
 servo_kp_turning = 1.1;
                     break; 
                case 3: PTE26_OUT = 1; 
                        PTE27_OUT = 1; 
 MAX_SPEED = 80 ;
 STD_SPEED = 80 ;
 MIN_SPEED = 80 ;
 servo_kp_turning = 1.1;
                         break;                          
                default:  
                    ;  
            }  
        }  
        else{  
                length = PID_choice();
                if(stop == 0 && system_clock - start_time > 350) 
                {
                  stop = foundStopLinePattern; 
                }
                if(stop == 1) 
                {
                  stop = 2;
                  stop_time = system_clock; 
                }
                if(stop == 2 && (system_clock - stop_time > 40)) 
                {
                  round ++;
                  stop = 0;
                }

                //led(LED2, LED_ON);
                //detect_broken();
                //print(length);
                float slope = doPredict((unsigned char *)&data[0][0], COLUMN, ROW);
                float input = 18*middle_slope + (middle_intercept - COLUMN/2)*47/100;
		//float input = area_cal(area_L, area_R);
                //led(LED2, LED_OFF);
		if(input != 0){	
			servo_pid(input);
            if(servo_output > R_TURN_RANGE) 
                servo_output = R_TURN_RANGE; 
            if(servo_output < -L_TURN_RANGE) 
                servo_output = -L_TURN_RANGE; 
			FTM_PWM_Duty(FTM0, CH0, MID_TURN + servo_output);
		}
                if(round == 2)
                { 
                   speed_input = 0; 
                   led(LED1, LED_OFF); 
                } 
  
                else
                { 
                    speed_input = speed_cal(length, servo_error, old_length, servo_old_error, old_speed_input); 
                    servo_old_error = servo_error; 
                    old_length = length; 
                    old_speed_input = speed_input;  
                }

        speed_pid(speed_input);
        FTM_PWM_Duty(FTM1, CH1, speed_output);
               
            if (!gpio_get(PORTE,6))
              delaytime ++;//sw2
            if (!gpio_get(PORTE,7))
              delaytime --;//sw3
            

	}
    }
}

