#ifndef HELLOWORLD_COMMON_H
#define HELLOWORLD_COMMON_H

union Coefficients {
  struct {
    float P, I, D;
  };
  float data[3];
};

#endif //HELLOWORLD_COMMON_H
