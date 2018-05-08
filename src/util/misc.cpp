#include <util/misc.h>

void Delay(int ms) {
  READ_BIT(SysTick->CTRL, SysTick_CTRL_COUNTFLAG_Msk);
  while (ms)
    if (READ_BIT(SysTick->CTRL, SysTick_CTRL_COUNTFLAG_Msk))
      ms--;
}
