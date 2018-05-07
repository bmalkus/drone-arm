#include <util/Timer.h>

#include <protocol/USART.h>
#include <util/misc.h>

#include <cstdint>
#include <util/TIMUtils.h>
#include <stm32f446xx.h>

TIM_TypeDef *Timer::_TIM;
volatile uint32_t Timer::_millis;

Timer::Timer(uint32_t timeout_millis, uint32_t timeout_micros):
  _timeout_millis(timeout_millis),
  _timeout_micros(timeout_micros)
{
  restart();
}

uint32_t Timer::millis()
{
  __disable_irq();
  uint32_t ret = _millis + (_TIM->CNT / 1'000);
  __enable_irq();
  return ret;
}

uint32_t Timer::micros()
{
  return _TIM->CNT % 1'000;
}

void Timer::init()
{
  _TIM = TIM2;

  _TIM->PSC = TIMUtils::presc_for(_TIM, 1'000'000);
  _TIM->ARR = 16'000'000;

  HandlerHelper::set_handler(HandlerHelper::interrupt_for(_TIM), __timer_tim_event_handler, nullptr);

  // enable interrupt
  SET_BIT(_TIM->DIER, TIM_DIER_UIE);

  // trigger update event, but do not trigger interrupt when setting UG
  SET_BIT(_TIM->CR1, TIM_CR1_URS);
  SET_BIT(_TIM->EGR, TIM_EGR_UG);

  // enable counter
  SET_BIT(_TIM->CR1, TIM_CR1_CEN);
}

void Timer::restart() volatile
{
  _end_millis = _millis + (_TIM->CNT / 1'000) + _timeout_millis;
  _end_micros = micros() + _timeout_micros;
  if (_end_micros >= 1'000)
  {
    _end_millis += 1;
    _end_micros -= 1'000;
  }
}

Timer::operator bool() const volatile
{
  return millis() < _end_millis || (millis() == _end_millis && micros() < _end_micros);
}

void Timer::tim_event_handler()
{
  _millis += 16'000;
  CLEAR_BIT(_TIM->SR, TIM_SR_UIF);
}

void __timer_tim_event_handler(HandlerHelper::InterruptType /*itype*/, void */*unused*/)
{
  Timer::tim_event_handler();
}
