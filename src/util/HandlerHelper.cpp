#include <util/HandlerHelper.h>

#include <stm32f4xx.h>
#include <protocol/USART.h>

void HandlerHelper::call_handler(InterruptType interrupt)
{
  handlers[interrupt].handler(interrupt, handlers[interrupt].user_data);
}

void HandlerHelper::set_handler(InterruptType interrupt, handler_type handler, void *user_data)
{
  handlers[interrupt] = {handler, user_data};
}

extern "C" void USART1_IRQHandler()
{
  HandlerHelper::call_handler(HandlerHelper::USART1_INT);
}

extern "C" void USART2_IRQHandler()
{
  HandlerHelper::call_handler(HandlerHelper::USART2_INT);
}

extern "C" void USART3_IRQHandler()
{
  HandlerHelper::call_handler(HandlerHelper::USART3_INT);
}

extern "C" void UART4_IRQHandler()
{
  HandlerHelper::call_handler(HandlerHelper::UART4_INT);
}

extern "C" void UART5_IRQHandler()
{
  HandlerHelper::call_handler(HandlerHelper::UART5_INT);
}

extern "C" void USART6_IRQHandler()
{
  HandlerHelper::call_handler(HandlerHelper::USART6_INT);
}

extern "C" void I2C1_EV_IRQHandler()
{
  HandlerHelper::call_handler(HandlerHelper::I2C1_EV_INT);
}

extern "C" void TIM1_BRK_TIM9_IRQHandler()
{
  HandlerHelper::call_handler(HandlerHelper::TIM1_BRK_TIM9_INT);
}

extern "C" void TIM1_TRG_COM_TIM11_IRQHandler()
{
  HandlerHelper::call_handler(HandlerHelper::TIM1_TRG_COM_TIM11_INT);
}

extern "C" void TIM2_IRQHandler()
{
  HandlerHelper::call_handler(HandlerHelper::TIM2_INT);
}

extern "C" void TIM3_IRQHandler()
{
  HandlerHelper::call_handler(HandlerHelper::TIM3_INT);
}

extern "C" void TIM4_IRQHandler()
{
  HandlerHelper::call_handler(HandlerHelper::TIM4_INT);
}

extern "C" void TIM5_IRQHandler()
{
  HandlerHelper::call_handler(HandlerHelper::TIM5_INT);
}

extern "C" void TIM6_DAC_IRQHandler()
{
  HandlerHelper::call_handler(HandlerHelper::TIM6_DAC_INT);
}

extern "C" void TIM8_CC_IRQHandler()
{
  HandlerHelper::call_handler(HandlerHelper::TIM8_CC_INT);
}

HandlerHelper::Handler HandlerHelper::handlers[HandlerHelper::InterruptType::NO_OF_INTERRUPTS];
