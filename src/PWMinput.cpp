#include <PWMinput.h>
#include <HandlerHelper.h>

PWMinput::PWMinput(TIM_TypeDef *TIM, uint32_t counter_freq):
  _TIM(TIM),
  _counter_freq(counter_freq),
  _output(0)
{
  // empty
}

void PWMinput::init()
{
  // Connect I1 to T1 - it will trigger on rising edge
  MODIFY_REG(_TIM->CCMR1, TIM_CCMR1_CC1S_Msk, 0b01 << TIM_CCMR1_CC1S_Pos);
  CLEAR_BIT(_TIM->CCER, TIM_CCER_CC1NP);
  CLEAR_BIT(_TIM->CCER, TIM_CCER_CC1P);

  // Connect I2 to T1 - it will trigger on falling edge
  MODIFY_REG(_TIM->CCMR1, TIM_CCMR1_CC2S_Msk, 0b10 << TIM_CCMR1_CC2S_Pos);
  CLEAR_BIT(_TIM->CCER, TIM_CCER_CC2NP);
  SET_BIT(_TIM->CCER, TIM_CCER_CC2P);

  // select filtered timer input 1 as trigger
  MODIFY_REG(_TIM->SMCR, TIM_SMCR_TS_Msk, 0b101 << TIM_SMCR_TS_Pos);
  // and select reset slave mode - counter is reinitialized on rising edge of
  // trigger selected above
  MODIFY_REG(_TIM->SMCR, TIM_SMCR_SMS_Msk, 0b100 << TIM_SMCR_SMS_Pos);

  // enable interrupt
  SET_BIT(_TIM->DIER, TIM_DIER_CC2IE);

  // if APBx prescaler > 1, timer freq is 2 * PCLKx, so prescaler must be divided by 2
  uint32_t clk = SystemCoreClock >> max(APB_presc_shift_for(_TIM) - 1, 0);
  _TIM->PSC = (clk / _counter_freq) - 1;

  // enable compare for each channel
  SET_BIT(_TIM->CCER, TIM_CCER_CC1E);
  SET_BIT(_TIM->CCER, TIM_CCER_CC2E);

  HandlerHelper::set_handler(HandlerHelper::interrupt_for(_TIM), __pwm_input_tim_event, this);
}

void PWMinput::start()
{
  SET_BIT(_TIM->CR1, TIM_CR1_CEN);
}

void PWMinput::stop()
{
  CLEAR_BIT(_TIM->CR1, TIM_CR1_CEN);
}

uint32_t PWMinput::get()
{
  return _output;
}

void PWMinput::handle_event(HandlerHelper::InterruptType /*itype*/)
{
  if (READ_BIT(_TIM->SR, TIM_SR_CC2IF))
  {
    _output = _TIM->CCR2;
    CLEAR_BIT(_TIM->SR, TIM_SR_CC2IF);
  }
}

uint16_t PWMinput::APB_presc_shift_for(TIM_TypeDef *TIM)
{
  // TODO: not all timers listed
  if (TIM == TIM1 || TIM == TIM8 || TIM == TIM9 || TIM == TIM10 || TIM == TIM11)
    return APBPrescTable[READ_VAL(RCC->CFGR, RCC_CFGR_PPRE2)];
  else
    return APBPrescTable[READ_VAL(RCC->CFGR, RCC_CFGR_PPRE1)];
}

void __pwm_input_tim_event(HandlerHelper::InterruptType itype, void *_pwm)
{
  PWMinput *pwm = (PWMinput*)_pwm;
  pwm->handle_event(itype);
}
