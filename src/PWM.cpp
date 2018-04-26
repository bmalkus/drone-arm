#include <PWM.h>
#include <utils.h>

PWM::PWM(TIM_TypeDef *TIM, uint16_t pwm_freq, uint16_t resolution, uint8_t channels):
  _TIM(TIM),
  _pwm_freq(pwm_freq),
  _resolution(resolution),
  _channels(channels),
  _compare_regs{&_TIM->CCR1, &_TIM->CCR2, &_TIM->CCR3, &_TIM->CCR4}
{
  // empty
}

void PWM::init()
{
  // enable auto-reload preload register - needed by PWM mode
  SET_BIT(_TIM->CR1, TIM_CR1_ARPE);

  // enable register preloading for all channels
  SET_BIT(_TIM->CCMR1, TIM_CCMR1_OC1PE);
  SET_BIT(_TIM->CCMR1, TIM_CCMR1_OC2PE);
  SET_BIT(_TIM->CCMR2, TIM_CCMR2_OC3PE);
  SET_BIT(_TIM->CCMR2, TIM_CCMR2_OC4PE);

  // if APBx prescaler > 1, timer freq is 2 * PCLKx, so prescaler must be divided by 2
  int8_t APB_presc_shift;

  // TODO: adjust to usage of timers clocked by APB1
  APB_presc_shift = APBPrescTable[READ_VAL(RCC->CFGR, RCC_CFGR_PPRE2)];
  uint32_t clk = SystemCoreClock >> max(APB_presc_shift - 1, 0);
  _TIM->PSC = (clk / (_pwm_freq * _resolution)) - 1;
  _TIM->ARR = _resolution - 1;

  // set output compare mode to PWM mode 1 for each channel and enable compare
  // output for each channel
  switch (_channels)
  {
    case 4:
      SET_BIT(_TIM->CCER, TIM_CCER_CC4E);
      MODIFY_REG(_TIM->CCMR2, TIM_CCMR2_OC4M_Msk, 0b110 << TIM_CCMR2_OC4M_Pos);
      // yup, fall-through
    case 3:
      SET_BIT(_TIM->CCER, TIM_CCER_CC3E);
      MODIFY_REG(_TIM->CCMR2, TIM_CCMR2_OC3M_Msk, 0b110 << TIM_CCMR2_OC3M_Pos);
      // still falling
    case 2:
      SET_BIT(_TIM->CCER, TIM_CCER_CC2E);
      MODIFY_REG(_TIM->CCMR1, TIM_CCMR1_OC2M_Msk, 0b110 << TIM_CCMR1_OC2M_Pos);
      // to this day
    case 1:
      SET_BIT(_TIM->CCER, TIM_CCER_CC1E);
      MODIFY_REG(_TIM->CCMR1, TIM_CCMR1_OC1M_Msk, 0b110 << TIM_CCMR1_OC1M_Pos);
  }

  // trigger event to update real registers with preloaded values
  trigger_update_event();

  // enable main output
  SET_BIT(_TIM->BDTR, TIM_BDTR_MOE);
}

void PWM::start()
{
  SET_BIT(_TIM->CR1, TIM_CR1_CEN);
}

void PWM::stop()
{
  CLEAR_BIT(_TIM->CR1, TIM_CR1_CEN);

  // reset counter values
  switch (_channels)
  {
    case 4:
      _TIM->CCR4 = 0;
      // another fall-through?!
    case 3:
      _TIM->CCR3 = 0;
      // yep, another
    case 2:
      _TIM->CCR2 = 0;
      // falling further
    case 1:
      _TIM->CCR1 = 0;
  }
  trigger_update_event();
}

void PWM::set(uint8_t channel, uint16_t val)
{
  *_compare_regs[channel - 1] = val;
}

uint16_t PWM::get_resolution()
{
  return _resolution;
}

void PWM::trigger_update_event()
{
  SET_BIT(_TIM->EGR, TIM_EGR_UG);
}
