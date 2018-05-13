#ifndef UTILS_H
#define UTILS_H

#include <cstddef>

#include <stm32f4xx.h>

#include <cmath>

#define READ_VAL(REG, PART) (((REG) & (PART##_Msk)) >> (PART##_Pos))

using cb_type = void (*)(void *);

void Delay(int ms);

template<typename T>
T min(T v1, T v2) {
  return v1 < v2 ? v1 : v2;
}

template<typename T>
T max(T v1, T v2) {
  return v1 < v2 ? v2 : v1;
}

template<typename T>
T clamp(T val, T lo, T hi) {
  return max(min(val, hi), lo);
}

template<class T, size_t N>
constexpr size_t arr_size(T (&)[N]) { return N; }

constexpr float rad_to_deg(float radians);
constexpr float deg_to_rad(float degrees);

constexpr float rad_to_deg(float radians) {
  return static_cast<float>(radians * 180.f * M_1_PI);
}

constexpr float deg_to_rad(float degrees) {
  return static_cast<float>(degrees * M_PI / 180.f);
}

#endif /* UTILS_H */
