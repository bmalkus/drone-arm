#ifndef SIMPLEFILTER_H
#define SIMPLEFILTER_H

#include <IMU.h>

class SimpleGyroFilter
{
public:
  SimpleGyroFilter(float gyro_sensitivity, float delay);

  void set(const Readings &gyro, const Readings &acc);

  Angles get_angles();

  void reset_angles();

private:
  const float _gyro_sensitivity;
  const float _delay;

  Angles _angles{{0.f, 0.f, 0.f}};
};

#endif /* SIMPLEFILTER_H */
