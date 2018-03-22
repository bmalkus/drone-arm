#ifndef TIMER_H
#define TIMER_H

#include <cinttypes>

#include <stm32f4xx.h>

/**
 * @brief Helper class for timing related functionalities
 *
 * Uses TIM14 to generate interrupts every second and updates internal _millis
 * counter. Objects, constructed with timeout parameter will be evaluated to
 * true until timeout is reached.
 * Counter in this class will overflow after ~49 days, but I doubt someone will fly a
 * drone constantly for 49 days ;)
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

  static void init();

  operator bool() const;

  // used to keep milliseconds elapsed since init()
  static uint32_t _millis;

  static TIM_TypeDef *_TIM;

private:
  uint32_t _start;
  uint32_t _end;

  static uint32_t curr_millis();
};

#endif /* TIMER_H */
