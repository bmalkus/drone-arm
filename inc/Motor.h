#ifndef MOTOR_H
#define MOTOR_H

#include <stm32f4xx.h>

#include <PWM.h>
#include <Timer.h>
#include <utils.h>

union Multipliers
{
  struct
  {
    int8_t pitch;
    int8_t roll;
    int8_t yaw;
  };
  int8_t data[3];
};

union Controls
{
  struct
  {
    int16_t pitch;
    int16_t roll;
    int16_t yaw;
    int16_t throttle;
  };
  int16_t data[4];
};

constexpr int16_t MOTOR_RESOLUTION = 1000;

/**
 * @brief Class responsible for controlling single motor
 */
class Motor
{
public:
  Motor(PWM *pwm, uint8_t channel, Multipliers multipliers);

  void init();

  bool ready();
  bool armed();

  void arm();
  void disarm();

  void set(Controls controls);

private:
  PWM *_pwm;
  float _pwm_multiplier;

  uint16_t _pwm_1_ms;
  uint16_t _pwm_low_offset;

  uint8_t _pwm_channel;
  Multipliers _multipliers;
  bool _armed;
  bool _init_called;
  Timer _init_timer;
};

#endif /* MOTOR_H */
