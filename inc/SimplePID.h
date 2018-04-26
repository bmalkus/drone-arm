#ifndef SIMPLEPID_H
#define SIMPLEPID_H

#include <GyroSimpleFilter.h>
#include <Motor.h>
#include <utils.h>

union Coefficients
{
  struct
  {
    float P, I, D;
  };
  float data[3];
};

class SimplePID
{
public:
  SimplePID();

  enum PART
  {
    P, I, D
  };

  void set_coeff(PART part, float coeff);

  Controls process(AngularRates rates, StickInputs inputs);

private:
  Coefficients _coeffs;

  AngularRates _errors[2];
  uint8_t _curr_errors = 0;

  AngularRates _cumulated_error = {{0.f, 0.f, 0.f}};
  StickInputs _prev_inputs;
};

#endif /* SIMPLEPID_H */
