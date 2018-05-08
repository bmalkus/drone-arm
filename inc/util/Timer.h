#ifndef TIMER_H
#define TIMER_H

#include <cinttypes>

#include <stm32f4xx.h>
#include "HandlerHelper.h"

/**
 * @brief Helper class for timing related functionalities
 */
class Timer {
public:
  /**
   * @brief Constructs default timer object, object will always be evaluated to
   * true
   */
  Timer() = default;

  /**
   * @brief Constructs timer object, object will be evaluated to true until
   * timeout is reached
   *
   * @param timeout (in milliseconds), timeout after which object will be
   * evaluated to false
   */
  explicit Timer(uint32_t timeout_millis, uint32_t timeout_micros = 0);

  /**
   * @brief Returns milliseconds elapsed since calling init()
   */
  static uint32_t millis();

  /**
   * @brief Returns microseconds elapsed since last millisecond
   */
  static uint32_t micros();

  /**
   * @brief Initializes timer whose counter will be used to retrieve
   * milliseconds elapsed
   */
  static void init();

  /**
   * @brief Restarts timer using timeout from constructor - timer will again be
   * evaluated to true until timeout is reached
   */
  void restart() volatile;

  /**
   * @brief bool evaluation of timer, returns true until timeout is reached
   * starting on construction or last restart()
   */
  operator bool() const volatile;

  static TIM_TypeDef *_TIM; //!< CMSIS timer structure for timer whose counter will be used

private:
  volatile static uint32_t _millis;

  volatile uint32_t _end_millis;
  volatile uint32_t _end_micros;
  uint32_t _timeout_millis;
  uint32_t _timeout_micros;

  static void tim_event_handler();

  friend void __timer_tim_event_handler(HandlerHelper::InterruptType, void *);
};

void __timer_tim_event_handler(HandlerHelper::InterruptType, void *);

#endif /* TIMER_H */
