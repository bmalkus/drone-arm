#include <IOwrapper/Motor.h>

constexpr float THROTTLE_MULTIPLIER = 0.8f;

constexpr float THROTTLE_OFFSET = (1.f - THROTTLE_MULTIPLIER) / 2.f;

Motor::Motor(PWM *pwm, uint8_t channel, Multipliers multipliers) :
    _pwm(pwm),
    _pwm_channel(channel),
    _multipliers(multipliers),
    _armed(false),
    _init_called(false),
    _init_timer(Timer(0)) {
  _pwm_1_ms = _pwm->get_resolution() / 5;
  _pwm_multiplier = ((9.f / 10.f) * _pwm_1_ms);
  _pwm_low_offset = _pwm_1_ms + (_pwm_1_ms * 0.1f);
}

void Motor::init() {
  _pwm->set(_pwm_channel, _pwm_1_ms);
  _init_timer = Timer(1000);
  _init_called = true;
}

bool Motor::ready() {
  return _init_called && !_init_timer;
}

bool Motor::armed() {
  return _armed;
}

void Motor::arm() {
  _armed = true;
  _pwm->set(_pwm_channel, _pwm_low_offset);
}

void Motor::disarm() {
  _armed = false;
  _pwm->set(_pwm_channel, _pwm_1_ms);
}

void Motor::set(Controls controls) {
  if (!_armed || _disabled) {
    _pwm->set(_pwm_channel, _pwm_1_ms);
    return;
  }
  float output = THROTTLE_OFFSET + controls.throttle * THROTTLE_MULTIPLIER;
  for (uint8_t i = 0; i < 3; ++i)
    output += controls.data[i] * _multipliers.data[i];
  output = clamp(output, 0.f, 1.f);
  _current = output;
  _pwm->set(_pwm_channel, _pwm_low_offset + static_cast<uint16_t>(_pwm_multiplier * output));
}

float Motor::current() {
  return _current;
}

void Motor::enable() {
  _disabled = false;
}

void Motor::disable() {
  _disabled = true;
}

bool Motor::is_disabled() {
  return _disabled;
}
