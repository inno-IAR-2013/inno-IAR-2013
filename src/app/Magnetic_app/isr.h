/***************************************
  isr.h
   HKUST Smartcar 2013 
**********************************************************************************/	



#ifndef __ISR_H
#define __ISR_H 1

#include  "include.h"


#undef  VECTOR_087
#define VECTOR_087    pit3_system_loop     //System multitask loop, run on 1ms

#undef  VECTOR_105
#define VECTOR_105   reed_switch   //for getting encoder count



extern void pit3_system_loop();
extern void reed_switch();

#endif  //__ISR_H

/* End of "isr.h" */
