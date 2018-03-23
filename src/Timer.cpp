#include <Timer.h>

#include <USART.h>
#include <utils.h>

#include <cstdint>

TIM_TypeDef *Timer::_TIM;

uint32_t Timer::now()
{
  return _TIM->CNT >> 1;
}

void Timer::init()
{
  _TIM = TIM2;

  int8_t APB_presc_shift = APBPrescTable[READ_VAL(RCC->CFGR, RCC_CFGR_PPRE1)];
  uint32_t clk = SystemCoreClock >> max(APB_presc_shift - 1, 0);
  _TIM->PSC = (clk / (2 * 1000)) - 1;
  _TIM->ARR = UINT32_MAX;

  _TIM->CCR1 = UINT32_MAX;

  SET_BIT(_TIM->EGR, TIM_EGR_UG);

  SET_BIT(_TIM->CR1, TIM_CR1_CEN);
}

Timer::Timer(uint32_t timeout):
  _timeout(timeout)
{
  _end = now() + _timeout;
}

void Timer::restart()
{
  _end = now() + _timeout;
}

Timer::operator bool() const
{
  return now() < _end;
}
