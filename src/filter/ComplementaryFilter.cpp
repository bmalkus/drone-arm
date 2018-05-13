#include "ComplementaryFilter.h"

#include <cmath>
#include <cstdint>

ComplementaryFilter::ComplementaryFilter(float gyro_sensitivity, float delay) :
    _gyro_sensitivity(gyro_sensitivity),
    _delay(delay) {
  // empty
}

const FilteredReadings& ComplementaryFilter::set(const Readings& gyro, const Readings& acc) {
  float _gyro[3] = {
      gyro.x * _gyro_sensitivity * _delay,
      gyro.y * _gyro_sensitivity * _delay,
      gyro.z * _gyro_sensitivity * _delay
  };

  _readings.angles.roll += _gyro[0] + _gyro[1] * sin(_readings.angles.pitch) * tan(_readings.angles.pitch) + _gyro[2] * cos(_readings.angles.roll) * tan(_readings.angles.pitch);
  _readings.angles.pitch += _gyro[1] * cos(_readings.angles.roll) - _gyro[2] * sin(_readings.angles.roll);
  _readings.yaw_rate = clamp(gyro.z * _gyro_sensitivity, static_cast<float>(_prev_yaw - M_PI * 0.5f), static_cast<float>(_prev_yaw + M_PI * 0.5f));

  int32_t sum = std::abs(acc.x) + std::abs(acc.y) + std::abs(acc.z);
  if (sum > 8192 && sum < 32768) {
    FilteredReadings acc_angles = {{0.f, 0.f, 0.f}};

    acc_angles.angles.roll = static_cast<float>(atan2(acc.y, acc.z));
    acc_angles.angles.pitch = static_cast<float>(atan2(-acc.x, sqrt(acc.y * acc.y + acc.z * acc.z)));

    _readings.angles.roll = _readings.angles.roll * 0.98f + acc_angles.angles.roll * 0.02f;
    _readings.angles.pitch = _readings.angles.pitch * 0.98f + acc_angles.angles.pitch * 0.02f;

    // _readings.angles.roll = acc_angles.angles.roll;
    // _readings.angles.pitch = acc_angles.angles.pitch;
  }

  for (float& ang : _readings.data) {
    if (ang > M_PI)
      ang -= 2 * M_PI;
    else if (ang < -M_PI)
      ang += 2 * M_PI;
  }

  return _readings;
}

const FilteredReadings& ComplementaryFilter::get_angles() {
  return _readings;
}

void ComplementaryFilter::reset_angles() {
  _readings = {{0.f, 0.f, 0.f}};
}
