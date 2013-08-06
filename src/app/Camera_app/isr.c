#include "common.h"
#include "include.h"

#define DISTANCE 2//one line every two line
unsigned char data[ROW][COLUMN];

int cur_line = 0;
int cur_line_start = -50; // 70
unsigned char p_counter = 0;
int delaytime = 300; //330: 165 | 340: 180 |250

int speed_current = 0;
int speed_count = 0;

u32 system_clock = 0;

void PORTB_IRQHandler(void){  
//HS PTB8, field interrupt
  if(PORTB_ISFR & (1<<8)){
		PORTB_ISFR  |= (1<<8);//exit
		//DisableInterrupts;
    LED_turn(LED0);
    p_counter++;
		cur_line = cur_line_start;
		//EnableInterrupts;
	}

//VS PTB9, line interrupt

  if(PORTB_ISFR & (1<<9)){
    PORTB_ISFR  |= (1<<9);//exit
    cur_line++;
    if((cur_line >= 0) && (cur_line < ROW*DISTANCE) && (cur_line % DISTANCE == 0)){
      for (int delay=0; delay < delaytime; delay++) //550
        ;  //get dummy 250  
      for (unsigned char cur_column = 0 ; cur_column < COLUMN; cur_column++)
        data[(ROW-1) - cur_line/DISTANCE][cur_column] = PTB10_IN;
    }
  }
}

void PIT0_IRQHandler()
{  
  speed_current = FTM2_CNT;
  speed_count = 1;
  //printf("%d\n", speed_current);
  system_clock ++;
  PIT_Flag_Clear(PIT0);
  FTM2_CNT = 0;
}