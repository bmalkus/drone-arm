#ifndef CONTEXT_H
#define CONTEXT_H

#include <protocol/USART.h>
#include <IOwrapper/Motor.h>
#include <IOwrapper/IMU.h>
#include <PID/RatePID.h>
#include <util/USARTHelper.h>
#include "PID/AnglePID.h"

struct Context
{
  static USART *uart_usb;
  static USART *uart_bt;
  static USARTHelper *usart_helper;
  static Motor *motors[4];
  static Readings *readings;
  static EulerianAngles *eulerian_angles;
  static Controls *controls;
  static Sticks *inputs;
  static AnglePID *pid;
};

#endif /* CONTEXT_H */
