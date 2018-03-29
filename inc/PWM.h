#ifndef PWM_H
#define PWM_H

#include <stm32f4xx.h>

/**
 * @brief Class responsible for managing timer put into PWM mode
 *
 * Note: Appropriate IO pins must be configured separately in order to PWM work properly,
 * as such configuration lays beyond this class responsibility
 */
class PWM
{
public:
  /**
   * @brief Constructor - does not initialize any registers
   *
   * @param TIM pointer to CMSIS TIM struct that will be used
   * @param pwm_freq desired frequency of PWM
   * @param resolution number up to which timer counter will count and up to which values should be provided
   * @param channels number of channels to enable
   */
  PWM(TIM_TypeDef *TIM, uint16_t pwm_freq, uint16_t resolution, uint8_t channels);

  /**
   * @brief Initializes timer peripheral, sets frequency of timer, enables channels and other needed stuff
   */
  void init();

  /**
   * @brief Enables counter (effectively PWM)
   */
  void start();
  /**
   * @brief Disables counter (effectively PWM)
   */
  void stop();

  /**
   * @brief Sets value for given channel at which output signal should go from high to low
   * (should be in range 0..resolution - see PWM())
   */
  void set(uint8_t channel, uint16_t val);

  uint16_t get_resolution();

private:
  TIM_TypeDef *_TIM;
  uint16_t _pwm_freq;
  uint16_t _resolution;
  uint8_t _channels;
  __IO uint32_t *_compare_regs[4];

  void trigger_update_event();
};

#endif /* PWM_H */
