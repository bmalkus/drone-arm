#ifndef GYROSIMPLEFILTER_H
#define GYROSIMPLEFILTER_H

#include <IMU.h>

class GyroSimpleFilter
{
public:
  GyroSimpleFilter(float gyro_sensitivity);

  AngularRates process(const Readings &gyro);

private:
  const float _gyro_sensitivity;

  AngularRates _prev;
};

#endif /* GYROSIMPLEFILTER_H */
