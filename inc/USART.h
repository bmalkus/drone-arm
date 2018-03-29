#ifndef USART_H
#define USART_H

#include <cstdint>

#include <stm32f4xx.h>

#include <HandlerHelper.h>
#include <utils.h>

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
  void send(const char *to_send);

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
  const uint8_t * volatile _to_send;
  volatile uint32_t _size;
  volatile bool _lock;

  uint8_t _byte_to_send;

  rx_cb_type _callback;
  void *_user_data;

  void lock();
  void unlock();
  bool is_locked();

  void handle_event(HandlerHelper::InterruptType itype);

  friend void __handle_uart_event(HandlerHelper::InterruptType, void *);
};

void __handle_uart_event(HandlerHelper::InterruptType, void *);

#endif /* USART_H */
