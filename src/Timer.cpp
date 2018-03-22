#include <Timer.h>

#include <USART.h>
#include <utils.h>

extern "C" void TIM8_TRG_COM_TIM14_IRQHandler()
{
  CLEAR_BIT(Timer::_TIM->SR, TIM_SR_CC1IF);
  Timer::_millis += 1000;
}

uint32_t Timer::_millis;
TIM_TypeDef *Timer::_TIM;

uint32_t Timer::now()
{
  return _millis + curr_millis();
}

void Timer::init()
{
  _millis = 0;
  _TIM = TIM14;

  NVIC_EnableIRQ(TIM8_TRG_COM_TIM14_IRQn);

  int8_t APB_presc_shift = APBPrescTable[READ_VAL(RCC->CFGR, RCC_CFGR_PPRE1)];
  uint32_t clk = SystemCoreClock >> max(APB_presc_shift - 1, 0);
  _TIM->PSC = (clk / (2 * 1000)) - 1;
  _TIM->ARR = 2000 - 1;

  SET_BIT(_TIM->DIER, TIM_DIER_CC1IE);

  _TIM->CCR1 = 2000 - 1;

  SET_BIT(_TIM->EGR, TIM_EGR_UG);

  SET_BIT(_TIM->CR1, TIM_CR1_CEN);
}

uint32_t Timer::curr_millis()
{
  return _TIM->CNT >> 1;
}

Timer::Timer(uint32_t timeout)
{
  _start = _millis + curr_millis();
  _end = _start + timeout;
}
