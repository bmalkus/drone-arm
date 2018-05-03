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
  HandlerHelper::set_handler(HandlerHelper::interrupt_for(_USART), __handle_uart_event, this);

  // Enable USART
  SET_BIT(_USART->CR1, USART_CR1_UE);

  // Enable RX interrupts
  SET_BIT(_USART->CR1, USART_CR1_RXNEIE);
}

void USART::send(uint8_t to_send)
{
  _out_buffer[_queue_at] = to_send;
  _queue_at = (_queue_at + 1) % 256;
  if (!_sending)
    SET_BIT(_USART->CR1, USART_CR1_TXEIE);
}

void USART::send(const char *to_send, cb_type cb, void *user_data)
{
  _done_cb = cb;
  _done_cb_user_data = user_data;
  while (*to_send)
  {
    _out_buffer[_queue_at] = *to_send++;
    _queue_at = (_queue_at + 1) % 256;
    if (!_sending)
      SET_BIT(_USART->CR1, USART_CR1_TXEIE);
  }
}

bool USART::is_sending()
{
  return _sending;
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

void USART::handle_event(HandlerHelper::InterruptType /*itype*/)
{
  if (READ_BIT(_USART->SR, USART_SR_RXNE))
  {
    // received bit
    if (_rx_cb != nullptr)
      _rx_cb(_rx_cb_user_data, static_cast<uint8_t>(_USART->DR));
    else
      CLEAR_BIT(_USART->SR, USART_SR_RXNE);
  }
  else
  {
    _sending = true;
    // sent bit
    if ((_queue_at % 256) != (_send_from % 256))
    {
      _USART->DR = _out_buffer[_send_from];
      _send_from = (_send_from + 1) % 256;
      return;
    }
    else
    {
      CLEAR_BIT(_USART->CR1, USART_CR1_TXEIE);
      if (_done_cb != nullptr)
        _done_cb(_done_cb_user_data);
      _done_cb = nullptr;
      _done_cb_user_data = nullptr;
      _sending = false;
    }
  }
}

// *****************************************************************************
//  non-members
// *****************************************************************************

void __handle_uart_event(HandlerHelper::InterruptType itype, void *_usart)
{
  auto *usart = (USART*)_usart;
  usart->handle_event(itype);
}
