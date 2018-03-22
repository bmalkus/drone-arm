#ifndef HANDLERHELPER_H
#define HANDLERHELPER_H

class HandlerHelper {

public:
  enum InterruptType { USART2_INT = 0, I2C1_EV_INT };

private:
  using handler_type = void (*)(InterruptType, void*);

  struct Handler {
    handler_type handler;
    void *user_data;
  };

  static Handler handlers[16];

public:
  HandlerHelper() = delete;

  static void call_handler(InterruptType interrupt);
  static void set_handler(InterruptType interrupt, handler_type handler, void *user_data);
};

#endif /* HANDLERHELPER_H */
