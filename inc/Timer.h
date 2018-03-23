#ifndef TIMER_H
#define TIMER_H

#include <cinttypes>

#include <stm32f4xx.h>

/**
 * @brief Helper class for timing related functionalities
 *
 * Uses TIM2 which is counting with 2kHz speed (atm config won't allow for it
 * to be slower). Elapsed time is computed from CNT register of timer. Objects,
 * constructed with timeout parameter will be evaluated to true until timeout
 * is reached.
 * TIM2 has 32-bit counter, so it will overflow after ~24 days, but I doubt I
 * will fly a drone constantly for 24 days ;)
 */
class Timer
{
public:
  /**
   * @brief Constructs timer object, object will be evaluated to true until
   * timeout is reached
   *
   * @param timeout (in milliseconds), timeout after which object will be
   * evaluated to false
   */
  Timer(uint32_t timeout);

  /**
   * @brief Returns milliseconds elapsed since calling init()
   */
  static uint32_t now();

  /**
   * @brief Initializes timer whose counter will be used to retrieve
   * milliseconds elapsed
   */
  static void init();

  /**
   * @brief Restarts timer using timeout from constructor - timer will again be
   * evaluated to true until timeout is reached
   */
  void restart();

  /**
   * @brief bool evaluation of timer, returns true until timeout is reached
   * starting on construction or last restart()
   */
  operator bool() const;

  static TIM_TypeDef *_TIM; //!< CMSIS timer structure for timer whose counter will be used

private:
  uint32_t _end;
  uint32_t _timeout;
};

#endif /* TIMER_H */
