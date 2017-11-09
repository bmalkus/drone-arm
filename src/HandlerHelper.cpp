#include <HandlerHelper.h>

#include <stm32f4xx.h>
#include <USART.h>

extern "C" void USART2_IRQHandler()
{
  HandlerHelper::call_handlers(HandlerHelper::USART2_INT);
}

extern "C" void I2C1_EV_IRQHandler()
{
  HandlerHelper::call_handlers(HandlerHelper::I2C1_EV_INT);
}

void HandlerHelper::call_handlers(InterruptType interrupt)
{
  auto vec = handlers[interrupt];
  for (auto handler : vec)
  {
    handler->handle_event(interrupt);
  }
}

void HandlerHelper::add_handler(InterruptType interrupt, HandlerClass *handler)
{
  HandlerHelper::handlers[interrupt].push_back(handler);
}

std::map<HandlerHelper::InterruptType, std::vector<HandlerClass*>> HandlerHelper::handlers;
