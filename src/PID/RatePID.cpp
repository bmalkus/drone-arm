#include <PID/RatePID.h>

#include <cmath>

static constexpr float MAX_INPUT_ANG_RATE = M_PI_2;
static constexpr float MAX_ANG_RATE = deg_to_rad(250.f);
static constexpr float SCALE_MULTIPLIER = (1.f / MAX_ANG_RATE) * (1.f / 3.f);

RatePID::RatePID() :
    _coeffs{{1.f, 1.f, 1.f}} {
  for (float &c : _coeffs.data)
    c *= SCALE_MULTIPLIER;
}

void RatePID::set_coeff(PART part, float coeff) {
  _coeffs.data[part] = coeff * SCALE_MULTIPLIER;
}

Controls RatePID::process(AngularRates rates, Sticks inputs) {
  _curr_errors = 1 - _curr_errors;

  Controls ret{{0.f, 0.f, 0.f, inputs.throttle}};

  for (uint8_t i = 0; i < 3; ++i) {
    _errors[_curr_errors].data[i] = ((inputs.data[i] * MAX_INPUT_ANG_RATE) - rates.data[i]);

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
