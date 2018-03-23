#include <HandlerHelper.h>

#include <stm32f4xx.h>
#include <USART.h>

extern "C" void USART2_IRQHandler()
{
  HandlerHelper::call_handler(HandlerHelper::USART2_INT);
}

extern "C" void I2C1_EV_IRQHandler()
{
  HandlerHelper::call_handler(HandlerHelper::I2C1_EV_INT);
}

void HandlerHelper::call_handler(InterruptType interrupt)
{
  handlers[interrupt].handler(interrupt, handlers[interrupt].user_data);
}

void HandlerHelper::set_handler(InterruptType interrupt, handler_type handler, void *user_data)
{
  handlers[interrupt] = {handler, user_data};
}

HandlerHelper::Handler HandlerHelper::handlers[HandlerHelper::InterruptType::NO_OF_INTERRUPTS];
