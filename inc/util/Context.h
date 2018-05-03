#ifndef CONTEXT_H
#define CONTEXT_H

#include <protocol/USART.h>
#include <IOwrapper/Motors.h>
#include <IOwrapper/IMU.h>
#include <PID/RatePID.h>
#include <util/USARTHelper.h>

struct Context
{
  static USART *uart_usb;
  static USART *uart_bt;
  static USARTHelper *usart_helper;
  static Motors *motors[4];
  static Readings *readings;
  static AngularRates *angular_rates;
  static Controls *controls;
  static RatePID *pid;
};

#endif /* CONTEXT_H */
