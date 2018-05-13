#ifndef HELLOWORLD_STICKINPUTS_H
#define HELLOWORLD_STICKINPUTS_H

#include <cstdint>
#include <protocol/PWMInput.h>

#include <util/USARTHelper.h>

union Sticks {
  struct {
    float pitch;
    float roll;
    float yaw;
    float throttle;
  };
  float data[4];

  enum Channel {
    PITCH = 0,
    ROLL,
    YAW,
    THROTTLE
  };
};

class StickInputs {
public:
  explicit StickInputs(PWMInput *pwm_input);

  Sticks get();

  bool should_be_armed();
  bool should_calibrate();

private:
  PWMInput *_pwm_input;
  Timer _timer;
  bool _arming = false, _should_be_armed = false;
  bool _calibrating = false, _should_calibrate = false;
  uint32_t _hold_arm_action = 0, _hold_calib_action = 0;

  void arm_action(bool conditions_met);
  void calibre_action(bool conditions_met);

  friend class USARTHelper;
};


#endif //HELLOWORLD_STICKINPUTS_H
