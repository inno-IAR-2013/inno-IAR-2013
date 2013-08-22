#include "common.h"
#include "include.h"
#define DISTANCE 2//one line every two line

unsigned char data[ROW][COLUMN];

int cur_line = 0;
unsigned char p_counter = 0;

//interrupt to get data
void PORTB_IRQHandler(void){  
//HS PTB8, field interrupt
  if(PORTB_ISFR & (1<<8)){
		PORTB_ISFR  |= (1<<8);//exit
		//DisableInterrupts;
		p_counter++;
		cur_line = -24;
		//EnableInterrupts;
	}

//VS PTB9, line interrupt

  if(PORTB_ISFR & (1<<9)){
    PORTB_ISFR  |= (1<<9);//exit
    cur_line++;
    if((cur_line >= 0) && (cur_line < ROW*DISTANCE) && (cur_line % DISTANCE == 0)){
      for (unsigned int delay=0; delay<300; delay++)
        ;  //get dummy  
      for (u8 cur_column = 0 ; cur_column < COLUMN; cur_column++)
        data[(ROW-1) - cur_line/DISTANCE][cur_column] = PTB10_IN;
    }
  }
}

void camera_init()
{
  exti_init(PORTB,8, falling_down); //HS field, falling_down
  exti_init(PORTB,9, rising_down);  //VS line, rising_down
  gpio_init(PORTB,10,GPI, 1);       // GPIN for camera signal
}

unsigned char* camera_get()
{
  return data;
}

unsigned char camera_count()
{
  return p_counter;
}

void camera_data_print(u8 type)
{
      for (int i = 0; i < ROW; i++)
      {
	if(type == 0)
	{
        for (u16 j = 0; j < COLUMN; j++)
          printf("%d", data[i][j]);
        printf("\n");
	}
	else if (type == 1)
	{
        int counter=0;
        if(data[i][0] == 0) printf("0 ");
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
      }

//print midline
/*      
      printf("(");
      for ( int i = 0; i < ROW && middle[i] != -1; i++)
      {
        if(i>0) printf(",");
        printf("%d", middle[i]);
      }
      printf(")");
*/
      
      printf("#");

}