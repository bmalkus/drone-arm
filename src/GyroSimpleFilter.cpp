#include <GyroSimpleFilter.h>
#include <utils.h>

#include <cmath>

GyroSimpleFilter::GyroSimpleFilter(float gyro_sensitivity):
  _gyro_sensitivity(gyro_sensitivity),
  _prev({{0.f, 0.f, 0.f}})
{
  // empty
}

AngularRates GyroSimpleFilter::process(const Readings &gyro)
{
  float curr[3] = {gyro.x * _gyro_sensitivity, gyro.y * _gyro_sensitivity, gyro.z * _gyro_sensitivity};
  AngularRates ret{{
    clamp(curr[0], static_cast<float>(_prev.data[0] - M_PI * 0.5f), static_cast<float>(_prev.data[0]  + M_PI * 0.5f)),
    clamp(curr[1], static_cast<float>(_prev.data[1] - M_PI * 0.5f), static_cast<float>(_prev.data[1]  + M_PI * 0.5f)),
    clamp(curr[2], static_cast<float>(_prev.data[2] - M_PI * 0.5f), static_cast<float>(_prev.data[2]  + M_PI * 0.5f))
  }};
  _prev = ret;
  return ret;
}
