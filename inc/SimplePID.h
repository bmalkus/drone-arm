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

  static constexpr uint8_t KEEP = 3;

  float _errors[KEEP][3];
  uint8_t _newest = 0;
};

#endif /* SIMPLEPID_H */
