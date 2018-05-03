#ifndef HELLOWORLD_TIMUTILS_H
#define HELLOWORLD_TIMUTILS_H

#include <stm32f4xx.h>

#include <cstdint>
#include <util/misc.h>


class TIMUtils
{
public:
  static uint16_t presc_for(TIM_TypeDef *TIM, uint32_t counter_freq)
  {
    uint32_t clk = SystemCoreClock >> max(APB_presc_shift_for(TIM) - 1, 0);
    return (clk / counter_freq) - 1;
  }

  static uint16_t APB_presc_shift_for(TIM_TypeDef *TIM)
  {
    // TODO: not all timers listed
    if (TIM == TIM1 || TIM == TIM8 || TIM == TIM9 || TIM == TIM10 || TIM == TIM11)
      return APBPrescTable[READ_VAL(RCC->CFGR, RCC_CFGR_PPRE2)];
    return APBPrescTable[READ_VAL(RCC->CFGR, RCC_CFGR_PPRE1)];
  }
};


#endif //HELLOWORLD_TIMUTILS_H
