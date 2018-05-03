#ifndef USART_H
#define USART_H

#include <cstdint>

#include <stm32f4xx.h>

#include <util/HandlerHelper.h>
#include <util/misc.h>

using rx_cb_type = void (*)(void*, uint8_t);

/**
 * @brief Class responsible for managing USART interface
 *
 * Note: Appropriate IO pins must be configured separately in order to USART work properly,
 * as such configuration lays beyond this class responsibility
 */
class USART
{
public:
  /**
   * @brief Constructor
   *
   * @param USART pointer to CMSIS USART struct that will be used
   * @param baud_rate baud rate to be set for USART interface
   * @param async bool indicating if communication should be done asynchronously
   */
  USART(USART_TypeDef *USART, uint32_t baud_rate);

  /**
   * @brief Initializes USART peripheral
   */
  void init();

  /**
   * @brief Sends char array via interface
   *
   * @param to_send char array (string) to send
   */
  void send(const char *to_send, cb_type cb=nullptr, void *user_data=nullptr);

  /**
   * @brief Sends single byte via interface
   *
   * @param to_send byte to send
   */
  void send(uint8_t to_send);

  /**
   * @brief Indicates if interface is busy sending data
   *
   * @return true if sending, false otherwise
   */
  bool is_sending();

  /**
   * @brief Sets callback to be called when data arrives
   */
  void set_rx_callback(rx_cb_type cb, void *user_data=nullptr);

  /**
   * @brief Clears callback for arriving data
   *
   * @return Pointer to last set callback
   */
  rx_cb_type clear_rx_callback();

private:
  USART_TypeDef *_USART;
  uint32_t _baud_rate;

  volatile char _out_buffer[256];
  volatile uint32_t _queue_at = 0;
  volatile uint32_t _send_from = 0;
  volatile bool _sending = false;

  cb_type _done_cb = nullptr;
  void *_done_cb_user_data = nullptr;

  rx_cb_type _rx_cb = nullptr;
  void *_rx_cb_user_data = nullptr;

  void handle_event(HandlerHelper::InterruptType itype);

  friend void __handle_uart_event(HandlerHelper::InterruptType, void *);
};

void __handle_uart_event(HandlerHelper::InterruptType, void *);

#endif /* USART_H */
