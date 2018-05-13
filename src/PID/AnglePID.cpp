#include "PID/AnglePID.h"

#include <cmath>

static constexpr float MAX_INPUT_ANGLE = M_PI_4;
static constexpr float MAX_ANGLE = M_PI;
static constexpr float SCALE_MULTIPLIER = (1.f / MAX_ANGLE) * (1.f / 3.f);

AnglePID::AnglePID() :
    _coeffs{{1.f, 1.f, 1.f}} {
  for (float &c : _coeffs.data)
    c *= SCALE_MULTIPLIER;
}

void AnglePID::set_coeff(AnglePID::PART part, float coeff) {
  _coeffs.data[part] = coeff * SCALE_MULTIPLIER;
}

float AnglePID::get_coeff(AnglePID::PART part) {
  return _coeffs.data[part] / SCALE_MULTIPLIER;
}

Controls AnglePID::process(EulerianAngles &angles, Sticks &inputs) {
  _curr_errors = 1 - _curr_errors;

  Controls ret{{0.f, 0.f, 0.f, inputs.throttle}};

  for (uint8_t i = 0; i < 3; ++i) {
    _errors[_curr_errors].data[i] = ((inputs.data[i] * MAX_INPUT_ANGLE) - angles.data[i]);

    // P part
    ret.data[i] += _errors[_curr_errors].data[i] * _coeffs.P;

    // // D part
    // ret.data[i] += (_errors[_curr_errors].data[i] - _errors[1-_curr_errors].data[i]) * _coeffs.D;

    // // I part
    // _cumulated_error.data[i] = clamp(_cumulated_error.data[i] + _errors[_curr_errors].data[i], -MAX_ANG_RATE, MAX_ANG_RATE);
    // if (std::abs(inputs.data[i] - _prev_inputs.data[i]))
    //   _cumulated_error.data[i] = 0.f;

    // _prev_inputs.data[i] = inputs.data[i];

    // ret.data[i] += _cumulated_error.data[i] * _coeffs.I;
  }

  return ret;
}
