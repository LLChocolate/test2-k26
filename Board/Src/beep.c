#include "beep.h"


void myBEEP_Init(void)
{
  gpio_init(PTE21,GPO,LOW);
  BEEP_Open_once();
} 

void BEEP_Open_once(void)
{
  BEEP=1;
  delayms(100);
  BEEP=0;
} 
