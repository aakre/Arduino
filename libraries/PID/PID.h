#ifndef __PID__
#define __PID__

#include <Arduino.h>

class PID {
  float x_i;
  float x_d;
  float k_p;
  float k_i;
  float k_d;
  float I_MAX;
  float I_MIN;
  int U_MIN;
  int U_MAX;
public:
  PID();
  PID(float k_p, float k_i, float k_d, float I_MIN, float I_MAX,int,int);
  float update(float error, float d_plant, int print);
  void reset_int();
  void set_k_p(float gain);
  void set_k_i(float gain);
  void set_k_d(float gain);
  void init(float k_p, float k_i, float k_d, float I_MIN, float I_MAX, float U_MIN, float U_MAX );
};


#endif
