#ifndef HANDLERHELPER_H
#define HANDLERHELPER_H

#include <map>
#include <vector>

class HandlerClass
{
public:
  virtual void handle_event() = 0;
};

class HandlerHelper
{
public:
  enum InterruptType { USART2_INT };

private:
  static std::map<InterruptType, std::vector<HandlerClass*>> handlers;

public:
  HandlerHelper() = delete;

  static void call_handlers(InterruptType interrupt);
  static void add_handler(InterruptType interrupt, HandlerClass *handler_class);
};

#endif /* HANDLERHELPER_H */
