#include <cstdint>

#include <PWMInput.h>
#include <HandlerHelper.h>
#include <stm32f446xx.h>

PWMInput::PWMInput(TIM_TypeDef *TIM, uint32_t counter_freq, uint8_t channels):
  _TIM(TIM),
  _counter_freq(counter_freq),
  _channels(channels),
  _start{0, 0, 0, 0},
  _outputs{0, 0, 0, 0},
  _alive{Timer(ALIVE_TIMEOUT), Timer(ALIVE_TIMEOUT), Timer(ALIVE_TIMEOUT), Timer(ALIVE_TIMEOUT)}
{
  // empty
}

void PWMInput::init()
{
  // Connect Ix to TIx - for now leave the default to trigger on rising edge, enable interrupts and compare
  switch (_channels)
  {
  case 4:
    MODIFY_REG(_TIM->CCMR2, TIM_CCMR2_CC4S_Msk, 0b01u << TIM_CCMR2_CC4S_Pos);
    SET_BIT(_TIM->DIER, TIM_DIER_CC4IE);
    SET_BIT(_TIM->CCER, TIM_CCER_CC4E);
    // fall-through
  case 3:
    MODIFY_REG(_TIM->CCMR2, TIM_CCMR2_CC3S_Msk, 0b01u << TIM_CCMR2_CC3S_Pos);
    SET_BIT(_TIM->DIER, TIM_DIER_CC3IE);
    SET_BIT(_TIM->CCER, TIM_CCER_CC3E);
    // fall-through
  case 2:
    MODIFY_REG(_TIM->CCMR1, TIM_CCMR1_CC2S_Msk, 0b01u << TIM_CCMR1_CC2S_Pos);
    SET_BIT(_TIM->DIER, TIM_DIER_CC2IE);
    SET_BIT(_TIM->CCER, TIM_CCER_CC2E);
    // fall-through
  case 1:
    MODIFY_REG(_TIM->CCMR1, TIM_CCMR1_CC1S_Msk, 0b01u << TIM_CCMR1_CC1S_Pos);
    SET_BIT(_TIM->DIER, TIM_DIER_CC1IE);
    SET_BIT(_TIM->CCER, TIM_CCER_CC1E);
  default:
    break;
  }

  SET_BIT(_TIM->DIER, TIM_DIER_UIE);

  // if APBx prescaler > 1, timer freq is 2 * PCLKx, so prescaler must be divided by 2
  uint32_t clk = SystemCoreClock >> max(APB_presc_shift_for(_TIM) - 1, 0);
  _TIM->PSC = (clk / _counter_freq) - 1;
  _TIM->ARR = TIM_CNTR_MAX;

  HandlerHelper::set_handler(HandlerHelper::interrupt_for(_TIM), __pwm_input_tim_event, this);
}

void PWMInput::start()
{
  SET_BIT(_TIM->CR1, TIM_CR1_CEN);
}

void PWMInput::stop()
{
  CLEAR_BIT(_TIM->CR1, TIM_CR1_CEN);
}

int32_t PWMInput::get(uint8_t channel)
{
  return _outputs[channel];
}

void PWMInput::handle_event(HandlerHelper::InterruptType /*itype*/)
{
  if (READ_BIT(_TIM->SR, TIM_SR_CC1IF))
  {
    if (READ_BIT(_TIM->CCER, TIM_CCER_CC1P))
    {
      // PWM signal is now low, set sensitivity to rising edge
      CLEAR_BIT(_TIM->CCER, TIM_CCER_CC1P);
      _outputs[0] = _TIM->CCR1 - _start[0];
    }
    else
    {
      // PWM signal is now low, set sensitivity to falling edge
      SET_BIT(_TIM->CCER, TIM_CCER_CC1P);
      _start[0] = _TIM->CCR1;
    }
    CLEAR_BIT(_TIM->SR, TIM_SR_CC1IF);
    _alive[0].restart();
  }
  else if (READ_BIT(_TIM->SR, TIM_SR_CC2IF))
  {
    if (READ_BIT(_TIM->CCER, TIM_CCER_CC2P))
    {
      // PWM signal is now low, set sensitivity to rising edge
      CLEAR_BIT(_TIM->CCER, TIM_CCER_CC2P);
      _outputs[1] = _TIM->CCR2 - _start[1];
    }
    else
    {
      // PWM signal is now low, set sensitivity to falling edge
      SET_BIT(_TIM->CCER, TIM_CCER_CC2P);
      _start[1] = _TIM->CCR2;
    }
    CLEAR_BIT(_TIM->SR, TIM_SR_CC2IF);
    _alive[1].restart();
  }
  else if (READ_BIT(_TIM->SR, TIM_SR_CC3IF))
  {
    if (READ_BIT(_TIM->CCER, TIM_CCER_CC3P))
    {
      // PWM signal is now low, set sensitivity to rising edge
      CLEAR_BIT(_TIM->CCER, TIM_CCER_CC3P);
      _outputs[2] = _TIM->CCR3 - _start[2];
    }
    else
    {
      // PWM signal is now low, set sensitivity to falling edge
      SET_BIT(_TIM->CCER, TIM_CCER_CC3P);
      _start[2] = _TIM->CCR3;
    }
    CLEAR_BIT(_TIM->SR, TIM_SR_CC3IF);
    _alive[2].restart();
  }
  else if (READ_BIT(_TIM->SR, TIM_SR_CC4IF))
  {
    if (READ_BIT(_TIM->CCER, TIM_CCER_CC4P))
    {
      // PWM signal is now low, set sensitivity to rising edge
      CLEAR_BIT(_TIM->CCER, TIM_CCER_CC4P);
      _outputs[3] = _TIM->CCR4 - _start[3];
    }
    else
    {
      // PWM signal is now low, set sensitivity to falling edge
      SET_BIT(_TIM->CCER, TIM_CCER_CC4P);
      _start[3] = _TIM->CCR4;
    }
    CLEAR_BIT(_TIM->SR, TIM_SR_CC4IF);
    _alive[3].restart();
  }
  else if (READ_BIT(_TIM->SR, TIM_SR_UIF))
  {
    for(int i = 0; i < _channels; ++i)
    {
      if (_start[i] > 0)
        _start[i] -= TIM_CNTR_MAX;
      if (!_alive[i])
        _outputs[i] = 0;
    }
    CLEAR_BIT(_TIM->SR, TIM_SR_UIF);
  }
}

uint16_t PWMInput::APB_presc_shift_for(TIM_TypeDef *TIM)
{
  // TODO: not all timers listed
  if (TIM == TIM1 || TIM == TIM8 || TIM == TIM9 || TIM == TIM10 || TIM == TIM11)
    return APBPrescTable[READ_VAL(RCC->CFGR, RCC_CFGR_PPRE2)];
  return APBPrescTable[READ_VAL(RCC->CFGR, RCC_CFGR_PPRE1)];
}

void __pwm_input_tim_event(HandlerHelper::InterruptType itype, void *_pwm)
{
  auto *pwm = (PWMInput*)_pwm;
  pwm->handle_event(itype);
}
