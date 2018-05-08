#ifndef HANDLERHELPER_H
#define HANDLERHELPER_H

#include <stm32f4xx.h>

extern "C" void SysTick_Handler();

extern "C" void USART1_IRQHandler();
extern "C" void USART2_IRQHandler();
extern "C" void USART3_IRQHandler();
extern "C" void UART4_IRQHandler();
extern "C" void UART5_IRQHandler();
extern "C" void USART6_IRQHandler();
extern "C" void I2C1_EV_IRQHandler();
extern "C" void TIM1_BRK_TIM9_IRQHandler();
extern "C" void TIM1_TRG_COM_TIM11_IRQHandler();
extern "C" void TIM2_IRQHandler();
extern "C" void TIM3_IRQHandler();
extern "C" void TIM4_IRQHandler();
extern "C" void TIM5_IRQHandler();
extern "C" void TIM6_DAC_IRQHandler();
extern "C" void TIM8_CC_IRQHandler();

/**
 * @brief Helper class used to set interrupt handlers and to call them with
 * user-provided arguments (if IRQHandler function was implemented next to the
 * class for given functionality, I see no way to call some class method on
 * particular object without using globals)
 */
class HandlerHelper {

public:
  enum InterruptType {
    INVALID = -1, // dummy - to return from interrupt_for()
    USART1_INT = 0,
    USART2_INT,
    USART3_INT,
    UART4_INT,
    UART5_INT,
    USART6_INT,
    I2C1_EV_INT,
    TIM1_BRK_TIM9_INT,
    TIM1_TRG_COM_TIM11_INT,
    TIM2_INT,
    TIM3_INT,
    TIM4_INT,
    TIM5_INT,
    TIM6_DAC_INT,
    TIM8_CC_INT,
    NO_OF_INTERRUPTS
  };

private:
  using handler_type = void (*)(InterruptType, void *);

  struct Handler {
    handler_type handler;
    void *user_data;
  };

  static Handler handlers[InterruptType::NO_OF_INTERRUPTS];

  static void call_handler(InterruptType interrupt);

public:
  HandlerHelper() = delete;

  static void set_handler(InterruptType interrupt, handler_type handler, void *user_data);

  static constexpr HandlerHelper::InterruptType interrupt_for(void *struct_ptr);

  friend void USART1_IRQHandler();
  friend void USART2_IRQHandler();
  friend void USART3_IRQHandler();
  friend void UART4_IRQHandler();
  friend void UART5_IRQHandler();
  friend void USART6_IRQHandler();
  friend void I2C1_EV_IRQHandler();
  friend void TIM1_BRK_TIM9_IRQHandler();
  friend void TIM1_TRG_COM_TIM11_IRQHandler();
  friend void TIM2_IRQHandler();
  friend void TIM3_IRQHandler();
  friend void TIM4_IRQHandler();
  friend void TIM5_IRQHandler();
  friend void TIM6_DAC_IRQHandler();
  friend void TIM8_CC_IRQHandler();
};

constexpr HandlerHelper::InterruptType HandlerHelper::interrupt_for(void *struct_ptr) {
  if (struct_ptr == TIM2)
    return InterruptType::TIM2_INT;
  else if (struct_ptr == TIM3)
    return InterruptType::TIM3_INT;
  else if (struct_ptr == TIM4)
    return InterruptType::TIM4_INT;
  else if (struct_ptr == TIM5)
    return InterruptType::TIM5_INT;
  else if (struct_ptr == TIM6)
    return InterruptType::TIM6_DAC_INT;
  else if (struct_ptr == TIM8)
    return InterruptType::TIM8_CC_INT;
  else if (struct_ptr == TIM9)
    return InterruptType::TIM1_BRK_TIM9_INT;
  else if (struct_ptr == TIM11)
    return InterruptType::TIM1_TRG_COM_TIM11_INT;
  else if (struct_ptr == USART1)
    return HandlerHelper::USART1_INT;
  else if (struct_ptr == USART2)
    return HandlerHelper::USART2_INT;
  else if (struct_ptr == USART3)
    return HandlerHelper::USART3_INT;
  else if (struct_ptr == UART4)
    return HandlerHelper::UART4_INT;
  else if (struct_ptr == UART5)
    return HandlerHelper::UART5_INT;
  else if (struct_ptr == USART6)
    return HandlerHelper::USART6_INT;
  return InterruptType::INVALID;
}

#endif /* HANDLERHELPER_H */
