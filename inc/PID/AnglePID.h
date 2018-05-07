#ifndef HELLOWORLD_ANGLEPID_H
#define HELLOWORLD_ANGLEPID_H


#include <IOwrapper/Motor.h>
#include <IOwrapper/IMU.h>
#include <IOwrapper/StickInputs.h>
#include <PID/common.h>

class AnglePID
{
public:
  AnglePID();

  enum PART
  {
    P, I, D
  };

  void set_coeff(PART part, float coeff);
  float get_coeff(PART part);

  Controls process(EulerianAngles &angles, Sticks &inputs);

private:
  Coefficients _coeffs;

  EulerianAngles _errors[2];
  uint8_t _curr_errors = 0;

  AngularRates _cumulated_error = {{0.f, 0.f, 0.f}};
  Sticks _prev_inputs;
};


#endif //HELLOWORLD_ANGLEPID_H
