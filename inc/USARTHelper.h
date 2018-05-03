#ifndef USARTHELPER_H
#define USARTHELPER_H

#include <Context.h>

class USARTHelper
{
public:
  USARTHelper(USART *usart_to_use, Context *context);

private:
  USART *_usart;
  Context *_context;

  char _buffer[64];
  static const char *delimit_tokens;
  static char output_buffer[256];
  uint8_t _bytes_read;

  bool _bt_passthrough;

  void main_menu();
  void bluetooth();
  void uart();

  void rx_callback(uint8_t byte, USART *USART_to_forward=nullptr);

  friend void __usart_rx_callback(void *usart_helper, uint8_t byte);
  friend void __usart_rx_bt_callback(void *usart_helper, uint8_t byte);
};

void __usart_rx_callback(void *usart_helper, uint8_t byte);
void __usart_rx_bt_callback(void *usart_helper, uint8_t byte);

#endif /* USARTHELPER_H */
