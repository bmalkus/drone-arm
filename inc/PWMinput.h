#ifndef PWM_INPUT_H
#define PWM_INPUT_H

#include <stm32f4xx.h>

#include <HandlerHelper.h>
#include <utils.h>

class PWMinput
{
public:
  PWMinput(TIM_TypeDef *TIM, uint32_t counter_freq);

  void init();

  void start();
  void stop();

  uint32_t get();

  uint16_t get_resolution();

private:
  TIM_TypeDef *_TIM;
  uint32_t _counter_freq;
  bool _input = true;
  __IO uint32_t _output;

  void handle_event(HandlerHelper::InterruptType);

  static uint16_t APB_presc_shift_for(TIM_TypeDef *TIM);

  friend void __pwm_input_tim_event(HandlerHelper::InterruptType, void *);
};

void __pwm_input_tim_event(HandlerHelper::InterruptType, void *);

#endif /* PWM_INPUT_H */
