#ifndef HELLOWORLD_COMPLEMENTARYFILTER_H
#define HELLOWORLD_COMPLEMENTARYFILTER_H

#include <IOwrapper/IMU.h>

class ComplementaryFilter {
public:
  ComplementaryFilter(float gyro_sensitivity, float delay);

  const FilteredReadings& set(const Readings& gyro, const Readings& acc);

  const FilteredReadings& get_angles();

  void reset_angles();

private:
  const float _gyro_sensitivity;
  const float _delay;

  FilteredReadings _readings{{0.f, 0.f, 0.f}};

  float _prev_yaw = 0.f;
};


#endif //HELLOWORLD_COMPLEMENTARYFILTER_H
