#ifndef Quadcopter_h
#define Quadcopter_h

#include "Arduino.h"
#include "Servo.h"
#include "../MatrixMath/MatrixMath.h"

class Quadcopter {
  Servo motor[4];
  int TAU_MAX;
  int TAU_MIN;
  int ESC_MAX;
  int ESC_MIN;
  float pwm[4];
  float T_alloc[4][4];
  float k_range;
public:
  Quadcopter(int TAU_MIN, int TAU_MAX, int ESC_MIN, int ESC_MAX);
  void init(int *motorPin);
  void input(float *tau, int print);
};


#endif
