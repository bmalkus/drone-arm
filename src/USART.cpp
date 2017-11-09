#include <USART.h>

#include <cstring>

USART::USART(USART_TypeDef *USART, uint32_t baud_rate, bool async):
  _USART(USART), _baud_rate(baud_rate), _async(async)
{
  // empty
}

void USART::init()
{
  // Disable USART
  CLEAR_BIT(_USART->CR1, USART_CR1_UE);

  // Configure baud rate
  uint32_t usartdiv = (45000000u * 25u) / (4u * _baud_rate); // (f_apb1 / (16 * baud)) * 100
  uint32_t mant = usartdiv / 100u;
  uint32_t frac = (((usartdiv - (mant * 100u)) * 16u) + 50u) / 100U;
  mant += frac / 16u;
  MODIFY_REG(
    _USART->BRR,
    USART_BRR_DIV_Fraction | USART_BRR_DIV_Mantissa,
    mant << USART_BRR_DIV_Mantissa_Pos |
    frac << USART_BRR_DIV_Fraction_Pos
  );

  // Enable TX and RX
  SET_BIT(_USART->CR1, USART_CR1_TE);
  SET_BIT(_USART->CR1, USART_CR1_RE);

  // Enable interrupt
  if (_async)
    HandlerHelper::add_handler(HandlerHelper::USART2_INT, this);

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
  if (_async)
  {
    lock();
    _to_send = reinterpret_cast<const uint8_t*>(to_send);
    _size = strlen(to_send);
    unlock();
    SET_BIT(USART2->CR1, USART_CR1_TXEIE);
  }
  else
  {
    while(*to_send)
    {
      while(!READ_BIT(_USART->SR, USART_SR_TXE))
        ;
      _USART->DR = *to_send++;
    }
  }
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
