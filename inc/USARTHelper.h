#ifndef USARTHELPER_H
#define USARTHELPER_H

#include <Context.h>

class USARTHelper
{
public:
  USARTHelper(Context *context);

  void next_iter();

private:
  using menu = void (USARTHelper::*)();

  Context *_context;
  char _buffer[64];
  uint8_t _bytes_read;

  menu _current;

  void rx_callback(uint8_t byte);
  void tx_callback();

  bool _send_readings = false;
  bool _send_ang_rates = false;
  bool _send_controls = false;
  uint8_t _sending = 0;

  void special();
  void motors();
  void set_PID_coeff();

  static const char *_help;
  static const char *_motors;
  static char output_buffer[64];

  friend void __usart_rx_callback(void *usart_helper, uint8_t bytes);
  friend void __usart_tx_callback(void *usart_helper);
};

void __usart_rx_callback(void *usart_helper, uint8_t bytes);
void __usart_tx_callback(void *usart_helper);

#endif /* USARTHELPER_H */
