#ifndef CONTEXT_H
#define CONTEXT_H

#include <USART.h>
#include <Motor.h>

struct Context
{
  USART *uart_usb;
  Motor *motors[4];
};

#endif /* CONTEXT_H */
