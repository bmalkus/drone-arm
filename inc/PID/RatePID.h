#ifndef SIMPLEPID_H
#define SIMPLEPID_H

#include <filter/GyroRateFilter.h>
#include <IOwrapper/Motors.h>
#include <util/misc.h>
#include <IOwrapper/IMU.h>
#include "IOwrapper/Sticks.h"
#include "PID/common.h"

class RatePID
{
public:
  RatePID();

  enum PART
  {
    P, I, D
  };

  void set_coeff(PART part, float coeff);

  Controls process(AngularRates rates, Sticks inputs);

private:
  Coefficients _coeffs;

  AngularRates _errors[2];
  uint8_t _curr_errors = 0;

  AngularRates _cumulated_error = {{0.f, 0.f, 0.f}};
  Sticks _prev_inputs;
};

#endif /* SIMPLEPID_H */
