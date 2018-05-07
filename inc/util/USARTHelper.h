#ifndef USARTHELPER_H
#define USARTHELPER_H

#include <protocol/USART.h>

class USARTHelper
{
public:
  explicit USARTHelper(USART *usart_to_use);

  void send(char c);
  void send(const char *str);

  void next_loop_iter();

private:
  USART *_usart;

  char _buffer[64];
  static const char *delimit_tokens;
  static char output_buffer[256];
  uint8_t _bytes_read;

  bool _bt_passthrough;

  bool _log_inputs = false, _log_controls = false, _log_angles = false;

  void main_menu();

  void bluetooth();
  void uart();
  void motor();
  void log();
  void pid();
  void get_pid();

  void rx_callback(uint8_t byte, USART *USART_to_forward=nullptr);

  friend void __usart_rx_callback(void *usart_helper, uint8_t byte);
  friend void __usart_rx_bt_callback(void *usart_helper, uint8_t byte);
};

void __usart_rx_callback(void *usart_helper, uint8_t byte);
void __usart_rx_bt_callback(void *usart_helper, uint8_t byte);

extern "C" int __io_putchar(int c);

#endif /* USARTHELPER_H */
