#include <SimpleGyroFilter.h>

#include <cmath>

SimpleGyroFilter::SimpleGyroFilter(float gyro_sensitivity, float delay):
  _gyro_sensitivity(gyro_sensitivity),
  _delay(delay)
{
  // empty
}

void SimpleGyroFilter::set(const Readings &gyro, const Readings &/* acc */)
{
  float _gyro[3] = {gyro.x * _gyro_sensitivity * _delay, gyro.y * _gyro_sensitivity * _delay, gyro.z * _gyro_sensitivity * _delay};
  _angles.x += _gyro[0] + _gyro[1] * sin(_angles.y) * tan(_angles.y) + _gyro[2] * cos(_angles.x) * tan(_angles.y);
  _angles.y += _gyro[1] * cos(_angles.x) - _gyro[2] * sin(_angles.x);
  _angles.z += _gyro[1] * sin(_angles.x)/cos(_angles.y) + _gyro[2] * cos(_angles.x)/cos(_angles.y);
  for (int i = 0; i < 3; ++i)
  {
    if (_angles.xyz[i] > 2*M_PI)
      _angles.xyz[i] -= 2*M_PI;
    else if (_angles.xyz[i] < 0)
      _angles.xyz[i] += 2*M_PI;
  }
}

Angles SimpleGyroFilter::get_angles()
{
  return _angles;
}

void SimpleGyroFilter::reset_angles()
{
  _angles = {{0.f, 0.f, 0.f}};
}
