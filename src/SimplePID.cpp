#include <SimplePID.h>

#include <cmath>

constexpr float mult = (1.f / ((250.f * M_PI) / 180.f)) * (1.f / 2.f);

SimplePID::SimplePID():
  _coeffs{{1.f, 1.f, 1.f}}
{
  for (uint8_t i = 0; i < 3; ++i)
    _coeffs.data[i] *= mult;
}

void SimplePID::set_coeff(PART part, float coeff)
{
  _coeffs.data[part] = coeff * mult;
}

Controls SimplePID::process(AngularRates rates, StickInputs inputs)
{
  float errors[3];
  uint8_t oldest = (_newest + 1) % KEEP;
  Controls ret{{0.f, 0.f, 0.f, inputs.throttle}};
  for (uint8_t i = 0; i < 3; ++i)
  {
    errors[i] = (inputs.data[i] - rates.data[i]);

    // P part
    ret.data[i] += errors[i] * _coeffs.data[P];

    // D part
    ret.data[i] -= (errors[i] - _errors[oldest][i]) * _coeffs.data[D];
  }

  for (uint8_t i = 0; i < 3; ++i)
    _errors[oldest][i] = errors[i];
  _newest = oldest;

  return ret;
}
