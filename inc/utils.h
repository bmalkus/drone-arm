#ifndef UTILS_H
#define UTILS_H

#include <stm32f4xx.h>

inline void Delay(int ms)
{
  READ_BIT(SysTick->CTRL, SysTick_CTRL_COUNTFLAG_Msk);
  while(ms)
    if(READ_BIT(SysTick->CTRL, SysTick_CTRL_COUNTFLAG_Msk))
      ms--;
}

// int __io_putchar(int c);

#endif /* UTILS_H */
