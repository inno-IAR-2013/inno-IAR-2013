#ifndef __ISR_H
#define __ISR_H 1

#include  "include.h"  

#undef VECTOR_104							//PORTB interrupt, for HS and VS
#define VECTOR_104 PORTB_IRQHandler

#undef  VECTOR_084
#define VECTOR_084 PIT0_IRQHandler     		//­PIT0, speed measure 

extern void PORTB_IRQHandler();           	//PORTB interrupt, for HS and VS
extern void PIT0_IRQHandler();            	//PIT0, speed measure 

#endif
//__ISR_H


