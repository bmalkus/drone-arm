#ifndef HELLOWORLD_COMPLEMENTARYFILTER_H
#define HELLOWORLD_COMPLEMENTARYFILTER_H

#include <IOwrapper/IMU.h>

class ComplementaryFilter {
public:
  ComplementaryFilter(float gyro_sensitivity, float delay);

  const EulerianAngles& set(const Readings& gyro, const Readings& acc);

  const EulerianAngles& get_angles();

  void reset_angles();

private:
  const float _gyro_sensitivity;
  const float _delay;

  EulerianAngles _angles{{0.f, 0.f, 0.f}};
};


#endif //HELLOWORLD_COMPLEMENTARYFILTER_H
