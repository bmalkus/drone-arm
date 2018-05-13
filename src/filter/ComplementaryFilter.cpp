#include "ComplementaryFilter.h"

#include <cmath>
#include <cstdint>

ComplementaryFilter::ComplementaryFilter(float gyro_sensitivity, float delay) :
    _gyro_sensitivity(gyro_sensitivity),
    _delay(delay) {
  // empty
}

const EulerianAngles& ComplementaryFilter::set(const Readings& gyro, const Readings& acc) {

  EulerianAngles gyro_angles = {{0.f, 0.f, 0.f}};
  EulerianAngles acc_angles = {{0.f, 0.f, 0.f}};

  float _gyro[3] = {
      gyro.x * _gyro_sensitivity * _delay,
      gyro.y * _gyro_sensitivity * _delay,
      gyro.z * _gyro_sensitivity * _delay
  };

  gyro_angles.x = _gyro[0] + _gyro[1] * sin(_angles.y) * tan(_angles.y) + _gyro[2] * cos(_angles.x) * tan(_angles.y);
  gyro_angles.y = _gyro[1] * cos(_angles.x) - _gyro[2] * sin(_angles.x);
  gyro_angles.z = _gyro[1] * sin(_angles.x) / cos(_angles.y) + _gyro[2] * cos(_angles.x) / cos(_angles.y);

  _angles.x += gyro_angles.x;
  _angles.y += gyro_angles.y;
  _angles.z += gyro_angles.z;

  int32_t sum = std::abs(acc.x) + std::abs(acc.y) + std::abs(acc.z);
  if (sum > 8192 && sum < 32768) {
    acc_angles.x = static_cast<float>(atan2(acc.y, acc.z));
    acc_angles.y = static_cast<float>(atan2(-acc.x, sqrt(acc.y * acc.y + acc.z * acc.z)));

    _angles.x = _angles.x * 0.98f + acc_angles.x * 0.02f;
    _angles.y = _angles.y * 0.98f + acc_angles.y * 0.02f;

    // _angles.x = acc_angles.x;
    // _angles.y = acc_angles.y;
    // _angles.z = 0.f;
  }

  for (float& ang : _angles.data) {
    if (ang > M_PI)
      ang -= 2 * M_PI;
    else if (ang < -M_PI)
      ang += 2 * M_PI;
  }

  return _angles;
}

const EulerianAngles& ComplementaryFilter::get_angles() {
  return _angles;
}

void ComplementaryFilter::reset_angles() {
  _angles = {{0.f, 0.f, 0.f}};
}
