#include <Arduino.h>

#ifndef RefMod_h
#define RefMod_h

/* A class for a reference model */
class RefMod {
  float x[3];
  float Ad[3];
  float Bd;
  float Ts;
public:
  RefMod();
  RefMod(float zeta, float omega, float Ts);
  void init(float zeta, float omega, float Ts);
  float update(float u);
};
#endif
