#ifndef GYROANGLEFILTER_H
#define GYROANGLEFILTER_H

#include <IOwrapper/IMU.h>

class GyroAngleFilter
{
public:
  GyroAngleFilter(float gyro_sensitivity, float delay);

  void set(const Readings &gyro);

  EulerianAngles get_angles();

  void reset_angles();

private:
  const float _gyro_sensitivity;
  const float _delay;

  EulerianAngles _angles{{0.f, 0.f, 0.f}};
};

#endif /* GYROANGLEFILTER_H */
