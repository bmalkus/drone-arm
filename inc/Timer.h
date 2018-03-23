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

  /**
   * @brief Initializes timer that will be used to update _millis
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

  volatile static uint32_t _millis; //!< used to keep milliseconds elapsed since init()

  static TIM_TypeDef *_TIM; //!< CMSIS timer structure used to setup interrupts

private:
  uint32_t _end;
  uint32_t _timeout;

  // returns milliseconds since last interrupt
  static uint32_t curr_millis();
};

#endif /* TIMER_H */
