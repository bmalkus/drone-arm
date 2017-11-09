#ifndef HANDLERHELPER_H
#define HANDLERHELPER_H

#include <map>
#include <vector>

class HandlerClass;

class HandlerHelper
{
public:
  enum InterruptType { USART2_INT, I2C1_EV_INT };

private:
  static std::map<InterruptType, std::vector<HandlerClass*>> handlers;

public:
  HandlerHelper() = delete;

  static void call_handlers(InterruptType interrupt);
  static void add_handler(InterruptType interrupt, HandlerClass *handler_class);
};

class HandlerClass
{
public:
  virtual void handle_event(HandlerHelper::InterruptType) = 0;
};

#endif /* HANDLERHELPER_H */
