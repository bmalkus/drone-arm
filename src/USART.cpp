#include <USART.h>

#include <cstring>

USART::USART(USART_TypeDef *USART, uint32_t baud_rate):
  _USART(USART),
  _baud_rate(baud_rate)
{
  // empty
}

void USART::init()
{
  // Disable USART
  CLEAR_BIT(_USART->CR1, USART_CR1_UE);

  // Configure baud rate
  // first, compute desired usartdiv - transformed formula from documentation, and multiply by 100
  uint32_t usartdiv = (45000000u * 25u) / (4u * _baud_rate); // (f_apb1 / (16 * baud)) * 100
  // extract mantisa (wholes)
  uint32_t mant = usartdiv / 100u;
  // extract fractional part, then round
  // fractional part is kept on 4 bits, so to extract it it is divided by 16
  // therefore, to write it, we multiply it by 16
  uint32_t frac = (((usartdiv - (mant * 100u)) * 16u) + 50u) / 100U;
  // add potential overflow to mantisa
  uint32_t overflow = frac / 16u;
  mant += overflow;
  frac -= overflow;
  MODIFY_REG(
    _USART->BRR,
    USART_BRR_DIV_Fraction | USART_BRR_DIV_Mantissa,
    mant << USART_BRR_DIV_Mantissa_Pos |
    frac << USART_BRR_DIV_Fraction_Pos
  );

  // Enable TX and RX
  SET_BIT(_USART->CR1, USART_CR1_TE);
  SET_BIT(_USART->CR1, USART_CR1_RE);

  // set interrupt handler
  // ugly, but no idea how to do it nicer
  if (_USART == USART1)
    HandlerHelper::set_handler(HandlerHelper::USART1_INT, __handle_uart_event, this);
  else if (_USART == USART2)
    HandlerHelper::set_handler(HandlerHelper::USART2_INT, __handle_uart_event, this);
  else if (_USART == USART3)
    HandlerHelper::set_handler(HandlerHelper::USART3_INT, __handle_uart_event, this);
  else if (_USART == UART4)
    HandlerHelper::set_handler(HandlerHelper::UART4_INT, __handle_uart_event, this);
  else if (_USART == UART5)
    HandlerHelper::set_handler(HandlerHelper::UART5_INT, __handle_uart_event, this);
  else if (_USART == USART6)
    HandlerHelper::set_handler(HandlerHelper::USART6_INT, __handle_uart_event, this);

  // Enable USART
  SET_BIT(_USART->CR1, USART_CR1_UE);

  // Enable RX interrupts
  SET_BIT(_USART->CR1, USART_CR1_RXNEIE);
}

void USART::send(uint8_t to_send)
{
  lock();
  _to_send = nullptr;
  _size = 1;
  _byte_to_send = to_send;
  unlock();
  SET_BIT(_USART->CR1, USART_CR1_TXEIE);
}

void USART::send(const char *to_send, cb_type cb, void *user_data)
{
  lock();
  _to_send = reinterpret_cast<const uint8_t*>(to_send);
  _size = strlen(to_send);
  _done_cb = cb;
  _done_cb_user_data = user_data;
  unlock();
  SET_BIT(_USART->CR1, USART_CR1_TXEIE);
}

bool USART::is_sending()
{
  return _size > 0;
}

void USART::set_rx_callback(rx_cb_type cb, void *user_data)
{
  _rx_cb = cb;
  _rx_cb_user_data = user_data;
}

rx_cb_type USART::clear_rx_callback()
{
  rx_cb_type ret = _rx_cb;
  _rx_cb = nullptr;
  _rx_cb_user_data = nullptr;
  return ret;
}

void USART::lock()
{
  _lock = true;
}

void USART::unlock()
{
  _lock = false;
}

bool USART::is_locked()
{
  return _lock;
}

void USART::handle_event(HandlerHelper::InterruptType /*itype*/)
{
  if (READ_BIT(_USART->SR, USART_SR_RXNE))
  {
    // received bit
    if (_rx_cb != nullptr)
    {
      _rx_cb(_rx_cb_user_data, _USART->DR);
    }
    else
      CLEAR_BIT(_USART->SR, USART_SR_RXNE);
  }
  else
  {
    // sent bit
    if (!is_locked())
    {
      if (_size > 0)
      {
        _USART->DR = _to_send == nullptr ? _byte_to_send : *_to_send++;
        --_size;
        return;
      }
      else
      {
        CLEAR_BIT(_USART->CR1, USART_CR1_TXEIE);
        if (_done_cb != nullptr)
          _done_cb(_done_cb_user_data);
        _done_cb = nullptr;
        _done_cb_user_data = nullptr;
      }
    }
    else
      CLEAR_BIT(_USART->CR1, USART_CR1_TXEIE);
  }
}

// *****************************************************************************
//  non-members
// *****************************************************************************

void __handle_uart_event(HandlerHelper::InterruptType itype, void *_usart)
{
  USART *usart = (USART*)_usart;
  usart->handle_event(itype);
}
