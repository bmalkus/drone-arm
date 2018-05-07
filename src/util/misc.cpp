#include <util/misc.h>

bool __todo;
uint32_t __primask;

void Delay(int ms)
{
  READ_BIT(SysTick->CTRL, SysTick_CTRL_COUNTFLAG_Msk);
  while(ms)
    if(READ_BIT(SysTick->CTRL, SysTick_CTRL_COUNTFLAG_Msk))
      ms--;
}
