#ifndef PWM_INPUT_H
#define PWM_INPUT_H

#include <stm32f4xx.h>

#include <HandlerHelper.h>
#include <utils.h>
#include <Timer.h>

class PWMInput
{
public:
  PWMInput(TIM_TypeDef *TIM, uint32_t counter_freq, uint8_t channels);

  void init();

  void start();
  void stop();

  int32_t get(uint8_t channel);

private:
  TIM_TypeDef *_TIM;
  uint32_t _counter_freq;
  uint8_t _channels;
  __IO int32_t _start[4];
  __IO int32_t _outputs[4];
  __IO Timer _alive[4];

  void handle_event(HandlerHelper::InterruptType);

  static uint16_t APB_presc_shift_for(TIM_TypeDef *TIM);

  static constexpr uint32_t TIM_CNTR_MAX = 10'000;
  static constexpr uint32_t ALIVE_TIMEOUT = 100;

  friend void __pwm_input_tim_event(HandlerHelper::InterruptType, void *);
};

void __pwm_input_tim_event(HandlerHelper::InterruptType, void *);

#endif /* PWM_INPUT_H */
