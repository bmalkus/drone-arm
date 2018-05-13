#include <filter/GyroAngleFilter.h>

#include <cmath>

GyroAngleFilter::GyroAngleFilter(float gyro_sensitivity, float delay) :
    _gyro_sensitivity(gyro_sensitivity),
    _delay(delay) {
  // empty
}

void GyroAngleFilter::set(const Readings &gyro) {
  float _gyro[3] = {
      gyro.x * _gyro_sensitivity * _delay,
      gyro.y * _gyro_sensitivity * _delay,
      gyro.z * _gyro_sensitivity * _delay
  };

  _readings.angles.roll += _gyro[0] + _gyro[1] * sin(_readings.angles.pitch) * tan(_readings.angles.pitch) + _gyro[2] * cos(_readings.angles.roll) * tan(_readings.angles.pitch);
  _readings.angles.pitch += _gyro[1] * cos(_readings.angles.roll) - _gyro[2] * sin(_readings.angles.roll);
  _readings.yaw_rate += _gyro[1] * sin(_readings.angles.roll) / cos(_readings.angles.pitch) + _gyro[2] * cos(_readings.angles.roll) / cos(_readings.angles.pitch);

  for (float &ang : _readings.data) {
    if (ang > M_PI)
      ang -= 2 * M_PI;
    else if (ang < -M_PI)
      ang += 2 * M_PI;
  }
}

FilteredReadings GyroAngleFilter::get_angles() {
  return _readings;
}

void GyroAngleFilter::reset_angles() {
  _readings = {{0.f, 0.f, 0.f}};
}
