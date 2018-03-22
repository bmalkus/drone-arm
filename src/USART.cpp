#include <USART.h>

#include <cstring>

USART::USART(USART_TypeDef *USART, uint32_t baud_rate):
  _USART(USART), _baud_rate(baud_rate)
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
  HandlerHelper::set_handler(HandlerHelper::USART2_INT, __handle_uart_event, this);

  // Enable USART
  SET_BIT(_USART->CR1, USART_CR1_UE);
}

void USART::handle_event(HandlerHelper::InterruptType /*itype*/)
{
  SET_BIT(GPIOA->ODR, GPIO_ODR_OD5);
  if (!is_locked())
  {
    if (_size > 0)
    {
      _USART->DR = _to_send == nullptr ? _byte_to_send : *_to_send++;
      --_size;
      return;
    }
  }
  // if we're here, no more left to send or we're locked and new data to send
  // is written - in former case interrupts are not wanted anymore, in latter
  // they will be enabled after data is written
  CLEAR_BIT(USART2->CR1, USART_CR1_TXEIE);
}

void USART::send(uint8_t to_send)
{
  lock();
  _to_send = nullptr;
  _size = 1;
  _byte_to_send = to_send;
  unlock();
  SET_BIT(USART2->CR1, USART_CR1_TXEIE);
}

void USART::send(const char *to_send)
{
  lock();
  _to_send = reinterpret_cast<const uint8_t*>(to_send);
  _size = strlen(to_send);
  unlock();
  SET_BIT(USART2->CR1, USART_CR1_TXEIE);
}

bool USART::is_sending()
{
  return _size > 0;
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


void __handle_uart_event(HandlerHelper::InterruptType itype, void *_usart)
{
  USART *usart = (USART*)_usart;
  usart->handle_event(itype);
}
