/*************************************************************
 *                   Camera library                          *
 * build-in get camera function                              *
 * PLEASE DON'T use PORTB interrupt if included this library *
 *************************************************************/

#ifndef __CAMERA_H
#define __CAMERA_H 1

#include  "include.h"  

#ifndef ROW  //initiallize ROW
#define ROW 100
#endif

#ifndef COLUMN   //initiallize COLUMN
#define COLUMN 250
#endif

#undef VECTOR_104  //for portB interrupt
#define VECTOR_104 PORTB_IRQHandler

extern void PORTB_IRQHandler();  //PORTE interrupt, for HS and VS
void camera_init();
unsigned char* camera_get();  //return double array of the latest image
unsigned char camera_count();  //return number of images got
void camera_data_print(u8 type);  //NEED to initiallize UART first
//type: 0->print as 0 and 1
//type: 1-> print the number of continuous 0s or 1s

#endif  //__CAMERA_H