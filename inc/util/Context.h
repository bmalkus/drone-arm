#ifndef CONTEXT_H
#define CONTEXT_H

#include <protocol/USART.h>
#include <IOwrapper/Motor.h>
#include <IOwrapper/IMU.h>
#include <PID/RatePID.h>

struct Context
{
  USART *uart_usb;
  USART *uart_bt;
  Motor *motors[4];
  Readings *readings;
  AngularRates *angular_rates;
  Controls *controls;
  RatePID *pid;
};

#endif /* CONTEXT_H */
