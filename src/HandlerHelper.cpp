#include <HandlerHelper.h>

#include <stm32f4xx.h>

extern "C" void USART2_IRQHandler()
{
  HandlerHelper::call_handlers(HandlerHelper::USART2_INT);
}

void HandlerHelper::call_handlers(InterruptType interrupt)
{
  auto vec = handlers[interrupt];
  for (auto handler : vec)
  {
    handler->handle_event();
  }
}

void HandlerHelper::add_handler(InterruptType interrupt, HandlerClass *handler)
{
  HandlerHelper::handlers[interrupt].push_back(handler);
}

std::map<HandlerHelper::InterruptType, std::vector<HandlerClass*>> HandlerHelper::handlers;
