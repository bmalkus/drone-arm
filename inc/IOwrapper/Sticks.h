#ifndef HELLOWORLD_STICKINPUTS_H
#define HELLOWORLD_STICKINPUTS_H

#include <cstdint>

class Sticks
{
public:
  Sticks();

  void set(uint8_t channel, float val);

  void zero();

  enum Channel
  {
    ROLL = 0,
    PITCH,
    THROTTLE,
    YAW
  };

  // public data
  union
  {
    struct
    {
      float roll;
      float pitch;
      float throttle;
      float yaw;
    };
    float data[4];
  };
};


#endif //HELLOWORLD_STICKINPUTS_H
