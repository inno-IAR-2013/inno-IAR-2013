/*******************************************

linearccd.c

Authored by Louis Mo
for HKUST SmartCar team 2013

Edited by John Ching

*******************************************/

#include  "include.h"
#include  "linearccd.h"
#include  "math.h"

extern int run_speed_mode;
extern int max_available_mode;
extern u32 system_loop_tick;
int vaild_pixel = 244;
 
/*** 外內灣 Variable， 數值愈細，愈貼近內灣行 ***/ 
int32_t left_start_length  = 25;
int32_t right_start_length = 25;

/*** 中心位 Variable，愈大愈接近Left edge，愈細愈接近Right edge ***/
int32_t ccd_mid_pos = 121;         
float atan_multiply_value;

/*********** CCD related counter ************/
u16 g_u16_ccd_sample_clock=0;
/*********** CCD related status flag ************/
int g_int_SI_state_flag=0;                    
int g_int_sampling_state_flag=0;              
int g_int_ccd_operation_state=0;
/*********** CCD related sample result & array ************/
char g_char_ar_ccd_current_pixel[256];       /*** ccd sample result ***/
/************* Variables for direction PID : algorthm 2 *************/
int current_mid_error_pos=121;
int last_sample_error_pos=121;
int previous_mid_error_pos=121;

uint32_t current_dir_error=0;
uint32_t current_dir_arc_value_error=0;

int current_1st_left_edge=243;
int current_1st_right_edge=0;

int32_t current_edge_middle_distance=0;
int32_t previous_edge_middle_distance=0;

int detect_left_flag=0;
int detect_right_flag=0;

//extern int encoder_turn_error;

/************ Special track case variable ************/
int all_white_smaple_flag=0;
int all_black_smaple_flag=0;

/*********** CCD basic library ************/
void ccd_sampling(char array[], int state){  
  g_int_ccd_operation_state = state;  
  while(g_int_ccd_operation_state == 1){        
       ccd_clock_turn();      
       ccd_detect_track(array);       
       ccd_SI_failing_edge_condition();       
       ccd_finish_one_sampling(array);
       g_u16_ccd_sample_clock++;
  }   
}

void ccd_clock_turn(){
    gpio_turn(PORTB, 9); // Gen 3 main board Clock
}

void ccd_trigger_SI(){
    if(g_int_SI_state_flag == 0 ){
               g_int_SI_state_flag = 1;              // SI Flag on
               g_int_sampling_state_flag = 1;        // sampling Flag on
               g_u16_ccd_sample_clock = 0;
               gpio_set(PORTB, 8, 1);                // Gen 3 main board SI rising edge
    }
}

void ccd_detect_track(char array[]){
   if(gpio_get(PORTB, 10) == 1) {  
   // if CCD receives black (2nd gen)
        array[g_u16_ccd_sample_clock] = 'W';
   } else {                        
   // if CCD receives white (2nd gen)
        array[g_u16_ccd_sample_clock] = 'o';
   }
}

void ccd_SI_failing_edge_condition(){
  if(g_u16_ccd_sample_clock == 1 && g_int_SI_state_flag == 1){ // condition for Longer SI failing edge to end
        gpio_set(PORTB, 8, 0); // Gen 2 SI faling edge
  }
}

void ccd_finish_one_sampling(char array[]){  
     if(g_u16_ccd_sample_clock == 255){
          g_int_SI_state_flag = 0;          // SI Flag off
          g_int_sampling_state_flag = 0;    // Sampling flag off
          ccd_shift_sample_to_manageable_position(array);
          ccd_scan_all_white_or_all_black_sample(array);
          //ccd_output_sample_to_UART(array);
          g_int_ccd_operation_state = 0;
     }       
}

void ccd_output_sample_to_UART(char array[]){
     uart_sendStr(UART3,"\n");
     uart_sendStr(UART3,"CCD Sample: ");
     ccd_print(array);
}

void ccd_shift_sample_to_manageable_position(char array[]){
      u16 i; 
      for( i = 0 ; i < vaild_pixel ; i++){
        array[i] = array[i+6];
      }
      
      for( i = vaild_pixel ; i < 256 ; i++){ /*** Filter out 12 pixels ***/
        array[i] = 'X';
      }
}

void ccd_scan_all_white_or_all_black_sample(char array[]){
  u16 i;
  u16 white_counter=0;
  u16 black_counter=0;
  all_white_smaple_flag= 0;
  all_black_smaple_flag= 0;
  
  for( i = 0 ; i < vaild_pixel ; i++){
        if(array[i] == 'o'){
          white_counter++; 
        }else if(array[i] == 'W'){
          black_counter++; 
        }
  }
  
  if(white_counter == vaild_pixel){
    all_white_smaple_flag = 1;
  } else if (black_counter == vaild_pixel){
    all_black_smaple_flag = 1;
  }
}

void ccd_print(char array[]){
      u16 i;  
      for( i = 0 ; i < 256 ; i++){
        printf("%c",array[i]); // print sample to UART
      }
       printf("\n");
}

void ccd_compressed_print(char array[]){
      u16 i;  
      for( i = 0 ; i < 250 ; i+=4){
        printf("%c",array[i]); // print sample to UART
      }
      printf("\n");
}

/*********** CCD Turn PID decision ************/
void ccd_recongize_left_right_edge_and_return_dir_error(char array[]){
    
  volatile int i;
  detect_left_flag = 0;
  detect_right_flag = 0;
  current_1st_left_edge=243;
  current_1st_right_edge=0;
  
  for( i = last_sample_error_pos ; i > 0 ; i--){ // scan from last_sample_error_pos to left edge
    if(array[i] == 'W'){
      current_1st_left_edge = i;
      detect_left_flag = 1;
      i = 1;
    }
  }
  
  for( i = last_sample_error_pos ; i < vaild_pixel ; i++){  // scan from last_sample_error_pos to right edge
    if(array[i] == 'W'){
      current_1st_right_edge = i;
      detect_right_flag = 1;
      i = 243;
    }
  }
  
  /* ||||--------------------------------|||| */
  if(detect_left_flag == 1 && detect_right_flag == 1){
    current_mid_error_pos = (current_1st_left_edge + current_1st_right_edge) / 2;
    //left_start_length = current_mid_error_pos - current_1st_left_edge;
    //right_start_length = current_mid_error_pos + current_1st_right_edge;
  }
  
  /* ||||--------------------------------||||  
     |||||||||||||||------------------------
     |||||||||||||||||||||||--------------- */
  else if(detect_left_flag == 1 && detect_right_flag == 0){
    current_mid_error_pos = current_1st_left_edge + right_start_length;
    
    if( current_1st_left_edge == (vaild_pixel - 1)){
      current_mid_error_pos = ccd_mid_pos;
    }
  }
  
   /* ||||-------------------------------||||
      --------------------------||||||||||||| 
      -----------------|||||||||||||||||||||| */
  else if(detect_left_flag == 0 && detect_right_flag == 1){
    current_mid_error_pos = current_1st_right_edge - left_start_length;
    
    if(current_1st_right_edge == 0){
      current_mid_error_pos = ccd_mid_pos;
    }
  }
    
   /* ---------------------------------------- (no middle noise) Cross road*/ 
  if(all_white_smaple_flag == 1){
    //current_mid_error_pos = ccd_mid_pos+(encoder_turn_error*35/100); // John added
    current_mid_error_pos = ccd_mid_pos;
  }
  
   /* |||||||||||||||||||||||||||||||||||||||| (all black) */
  if(all_black_smaple_flag == 1){
    current_mid_error_pos = ccd_mid_pos;
  }
  
  current_dir_error = (current_mid_error_pos - ccd_mid_pos);
  current_dir_arc_value_error = atan(current_dir_error*(atan_multiply_value))*1000;
  
  previous_mid_error_pos = current_mid_error_pos;
  last_sample_error_pos = current_mid_error_pos;  
  calculate_two_edge_middle_distance(array);
}

void calculate_two_edge_middle_distance(char array[]){
  current_edge_middle_distance = current_1st_right_edge - current_1st_left_edge;
}

void output_algorithm_message_to_UART(){ 
  
  printf("\ncurrent_mid_error_pos is: %d", current_mid_error_pos);
  printf("\n****** ******\n");
  
  ccd_output_sample_to_UART(g_char_ar_ccd_current_pixel);
  printf("\n\n****** ******\n");
  
  if(detect_left_flag == 1 && detect_right_flag == 1){
    printf("Both side detected : STRAIGHT line");
  } else if(detect_left_flag == 1 && detect_right_flag == 0){ 
    printf("Left side detected : ONE-EDGE");
  } else if(detect_left_flag == 0 && detect_right_flag == 1){
    printf("Right side detected : ONE-EDGE");
  } else if(detect_left_flag == 0 && detect_right_flag == 0){
    printf("NO EDGE detected");
  } 
    
  printf("\n****** LEFT & RIGHT edge position ******");
  printf("\ncurrent_1st_right_edge: %d", current_1st_right_edge);
  printf("\ncurrent_1st_left_edge: %d", current_1st_left_edge);
  
  printf("\n****** DIR ERROR ******");
  printf("\ncurrent_dir_error is: %ld", current_dir_error);
}