#ifndef USART_H
#define USART_H

#include <cstdint>

#include <stm32f4xx.h>

#include <HandlerHelper.h>

class USART
{
public:
  USART(USART_TypeDef *USART, uint32_t baud_rate, bool async=true);

  void init();

  void send(const char *to_send);
  void send(uint8_t to_send);

  bool is_sending();

private:
  USART_TypeDef *_USART;
  bool _async;
  uint32_t _baud_rate;
  const uint8_t *_to_send;
  uint32_t _size;
  bool _lock;

  uint8_t _byte_to_send;

  void lock();
  void unlock();
  bool is_locked();

  void handle_event(HandlerHelper::InterruptType itype);

  friend void __handle_uart_event(HandlerHelper::InterruptType, void *);
};

void __handle_uart_event(HandlerHelper::InterruptType, void *);

#endif /* USART_H */
