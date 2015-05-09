#ifndef __PID__
#define __PID__

#include <Arduino.h>

class PID {
  float x_d;
  float x_i;
  float k_p;
  float k_i;
  float k_d;
  float I_MAX;
  float I_MIN;
  int U_MIN;
  int U_MAX;
public:
  PID(float k_p, float k_i, float k_d, float I_MIN, float I_MAX,int,int);
  float update(float error, float plant, int print);
  void reset_int();
  void set_kp(float gain);
  void set_ki(float gain);
  void set_kd(float gain);
};


#endif
