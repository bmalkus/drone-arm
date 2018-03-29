#ifndef USARTHELPER_H
#define USARTHELPER_H

#include <Context.h>

class USARTHelper
{
public:
  USARTHelper(Context *context);

private:
  using menu = void (USARTHelper::*)();

  Context *_context;
  char _buffer[64];
  uint8_t _bytes_read;

  menu _current;

  void rx_callback(uint8_t byte);

  void special();
  void motors();

  static const char *_help;
  static const char *_motors;
  static char output_buffer[64];

  friend void __usart_rx_callback(void *usart_helper, uint8_t bytes);
};

void __usart_rx_callback(void *usart_helper, uint8_t bytes);

#endif /* USARTHELPER_H */
