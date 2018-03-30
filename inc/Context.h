#ifndef CONTEXT_H
#define CONTEXT_H

#include <USART.h>
#include <Motor.h>
#include <IMU.h>
#include <SimplePID.h>

struct Context
{
  USART *uart_usb;
  Motor *motors[4];
  Readings *readings;
  AngularRates *angular_rates;
  Controls *controls;
  SimplePID *pid;
};

#endif /* CONTEXT_H */
