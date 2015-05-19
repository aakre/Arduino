#include "RefMod.h"

RefMod::RefMod() {}

RefMod::RefMod(float zeta, float omega, float Ts) {
  x[0] = 0;
  x[1] = 0;
  x[2] = 0;

  Ad[0] = -omega*omega*omega*Ts;
  Ad[1] = -(2*zeta+1)*omega*omega*Ts;
  Ad[2] = -(2*zeta+1)*omega*Ts;
  Bd = -Ad[0];
  this->Ts = Ts;
}

void RefMod::init(float zeta, float omega, float Ts) {
  x[0] = 0;
  x[1] = 0;
  x[2] = 0;

  Ad[0] = -omega*omega*omega*Ts;
  Ad[1] = -(2*zeta+1)*omega*omega*Ts;
  Ad[2] = -(2*zeta+1)*omega*Ts;
  Bd = -Ad[0];
  this->Ts = Ts;
}

float RefMod::update(float u) {
  x[2] += Ad[0]*x[0] + Ad[1]*x[1] + Ad[2]*x[2] + Bd*u;
  x[1] += Ts*x[2];
  x[0] += Ts*x[1];
  return x[0];
}
