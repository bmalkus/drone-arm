#ifndef HANDLERHELPER_H
#define HANDLERHELPER_H

extern "C" void USART1_IRQHandler();
extern "C" void USART2_IRQHandler();
extern "C" void USART3_IRQHandler();
extern "C" void UART4_IRQHandler();
extern "C" void UART5_IRQHandler();
extern "C" void USART6_IRQHandler();
extern "C" void I2C1_EV_IRQHandler();

/**
 * @brief Helper class used to set interrupt handlers and to call them with
 * user-provided arguments (if IRQHandler function was implemented next to the
 * class for given functionality, I see no way to call some class method on
 * particular object without using globals)
 */
class HandlerHelper {

public:
  enum InterruptType {
    USART1_INT = 0,
    USART2_INT,
    USART3_INT,
    UART4_INT,
    UART5_INT,
    USART6_INT,
    I2C1_EV_INT,
    NO_OF_INTERRUPTS
  };

private:
  using handler_type = void (*)(InterruptType, void*);

  struct Handler {
    handler_type handler;
    void *user_data;
  };

  static Handler handlers[InterruptType::NO_OF_INTERRUPTS];

  static void call_handler(InterruptType interrupt);

public:
  HandlerHelper() = delete;

  static void set_handler(InterruptType interrupt, handler_type handler, void *user_data);

  friend void USART1_IRQHandler();
  friend void USART2_IRQHandler();
  friend void USART3_IRQHandler();
  friend void UART4_IRQHandler();
  friend void UART5_IRQHandler();
  friend void USART6_IRQHandler();
  friend void I2C1_EV_IRQHandler();
};

#endif /* HANDLERHELPER_H */
