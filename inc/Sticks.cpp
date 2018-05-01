#include "Sticks.h"

Sticks::Sticks():
  data{0.f, 0.f, 0.f, 0.f}
{

}

void Sticks::set(uint8_t channel, float val)
{
  data[channel] = val;
}

void Sticks::zero()
{
  roll = pitch = yaw = throttle = 0.f;
}
