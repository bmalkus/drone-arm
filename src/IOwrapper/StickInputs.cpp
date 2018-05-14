#include <initializer_list>

#include <IOwrapper/Motor.h>
#include <cstdio>
#include "IOwrapper/StickInputs.h"

static constexpr uint32_t STICK_ACTIONS_DELAY_MS = 500;

StickInputs::StickInputs(PWMInput *pwm_input) :
    _pwm_input(pwm_input),
    _timer(STICK_ACTIONS_DELAY_MS) {
  // empty
}

Sticks StickInputs::get() {
  Sticks inputs{0.f, 0.f, 0.f, 0.f};

  // variable used to check against lost PWM input
  // if more than two channels are reported as zero
  // (which mean there are no PWM inputs for them)
  // zero all the inputs
  uint8_t lost = 0;

  auto th = _pwm_input->get(Sticks::THROTTLE);
  if (1000 <= th && th <= 2000)
    inputs.throttle = (th - 1000) / 1000.f;
  else if (th == 0) {
    inputs.throttle = 0.f;
    ++lost;
  }

  for (auto ch : {Sticks::PITCH, Sticks::ROLL, Sticks::YAW}) {
    auto val = _pwm_input->get(ch);
    if (1000 <= val && val <= 2000)
      inputs.data[ch] = (val - 1500) / 1000.f;
    else if (val == 0) {
      inputs.data[ch] = 0.f;
      ++lost;
    }
  }

  if (lost >= 3 && !_ignore_disabled_tx) {
    _should_be_armed = false;
    return {0.f, 0.f, 0.f, 0.f};
  }

  arm_action(inputs.throttle < 0.3f && inputs.yaw > 0.3f && inputs.pitch < -0.3f && inputs.roll < -0.3f);
  calibre_action(inputs.throttle < 0.3f && inputs.yaw < -0.3f && inputs.pitch < -0.3f && inputs.roll < -0.3f);

  return inputs;
}

bool StickInputs::should_be_armed() {
  return _should_be_armed;
}

bool StickInputs::should_calibrate() {
  bool ret = false;
  if (_should_calibrate) {
    ret = true;
    _should_calibrate = false;
  }
  return ret;
}

void StickInputs::arm_action(bool conditions_met) {
  if (conditions_met) {
    if (_hold_arm_action == 0) {
      if (!_arming) {
        _arming = true;
        _timer.restart();
      } else if (!_timer) {
        _should_be_armed = !_should_be_armed;
        _ignore_disabled_tx = false;
        _hold_arm_action = 100;
      }
    } else {
      _hold_arm_action = 100;
    }
  } else {
    _arming = false;
    if (_hold_arm_action)
      _hold_arm_action -= 1;
  }
}

void StickInputs::calibre_action(bool conditions_met) {
  if (conditions_met) {
    if (_hold_calib_action == 0) {
      if (!_calibrating) {
        _calibrating = true;
        _timer.restart();
      } else if (!_timer) {
        _should_calibrate = true;
        _hold_calib_action = 100;
      }
    } else {
      _hold_calib_action = 100;
    }
  } else {
    _calibrating = false;
    if (_hold_calib_action)
      _hold_calib_action -= 1;
  }
}
