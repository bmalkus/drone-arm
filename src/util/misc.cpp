#include <util/misc.h>
// #include <USART.h>

// extern USART uart_usb;

// int __io_putchar(int c)
// {
//   uart_usb.send((uint8_t)c, true);
// }

void Delay(int ms)
{
  READ_BIT(SysTick->CTRL, SysTick_CTRL_COUNTFLAG_Msk);
  while(ms)
    if(READ_BIT(SysTick->CTRL, SysTick_CTRL_COUNTFLAG_Msk))
      ms--;
}
