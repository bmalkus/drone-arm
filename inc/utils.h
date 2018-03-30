#ifndef UTILS_H
#define UTILS_H

#include <stm32f4xx.h>

#define READ_VAL(REG, PART) (((REG) & (PART##_Msk)) >> (PART##_Pos))

using cb_type = void (*)(void*);

void Delay(int ms);

template <typename T>
T min(T v1, T v2)
{
  return v1 < v2 ? v1 : v2;
}

template <typename T>
T max(T v1, T v2)
{
  return v1 < v2 ? v2 : v1;
}

template <typename T>
T clamp(T val, T lo, T hi)
{
  return max(min(val, hi), lo);
}

union StickInputs
{
  struct
  {
    float roll;
    float pitch;
    float yaw;
    float throttle;
  };
  float data[4];
};

// int __io_putchar(int c);

#endif /* UTILS_H */
