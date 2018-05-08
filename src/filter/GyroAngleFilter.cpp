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

  _angles.x += _gyro[0] + _gyro[1] * sin(_angles.y) * tan(_angles.y) + _gyro[2] * cos(_angles.x) * tan(_angles.y);
  _angles.y += _gyro[1] * cos(_angles.x) - _gyro[2] * sin(_angles.x);
  _angles.z += _gyro[1] * sin(_angles.x) / cos(_angles.y) + _gyro[2] * cos(_angles.x) / cos(_angles.y);

  for (float &ang : _angles.data) {
    if (ang > M_PI)
      ang -= 2 * M_PI;
    else if (ang < -M_PI)
      ang += 2 * M_PI;
  }
}

EulerianAngles GyroAngleFilter::get_angles() {
  return _angles;
}

void GyroAngleFilter::reset_angles() {
  _angles = {{0.f, 0.f, 0.f}};
}
