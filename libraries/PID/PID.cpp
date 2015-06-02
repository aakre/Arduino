#include "PID.h"

PID::PID() {
  this->k_p = 0;
  this->k_i = 0;
  this->k_d = 0;
  this->I_MIN = 0;
  this->I_MAX = 0;
  this->U_MIN = 0;
  this->U_MAX = 0;
  this->x_d=0;
  this->x_i=0;
}

PID::PID(float k_p, float k_i, float k_d, float I_MIN, float I_MAX, int U_MIN, int U_MAX) {
  this->k_p = k_p;
  this->k_i = k_i;
  this->k_d = k_d;
  this->I_MIN = I_MIN;
  this->I_MAX = I_MAX;
  this->U_MIN = U_MIN;
  this->U_MAX = U_MAX;
  this->x_d=0;
  this->x_i=0;
}

float PID::update(float error, float d_plant, int print) {
  float P=0;
  float I=0;
  float D=0;
  float u;

  P = k_p*error;

  x_i += error;
  x_i = max(I_MIN, min(I_MAX, x_i)); // Saturation
  I = k_i*x_i;
  //this is a dummy comment
  D = -k_d*d_plant;

  u = P+I+D;
  if (print) {
    Serial.print("\r\nControl output \t");
    Serial.print(u);
  }
  /* Saturate */
  return max(U_MIN, min(U_MAX,u));
}

void PID::reset_int() {
  x_i = 0;
}

void PID::set_k_p(float k_p) {
  this->k_p = k_p;
}

void PID::set_k_i(float k_i) {
  this->k_i = k_i;
}

void PID::set_k_d(float k_d) {
  this->k_d = k_d;
}

void PID::init(float k_p, float k_i, float k_d, float I_MIN, float I_MAX, float U_MIN, float U_MAX ) {
  this->k_p = k_p;
  this->k_i = k_i;
  this->k_d = k_d;
  this->I_MIN = I_MIN;
  this->I_MAX = I_MAX;
  this->U_MIN = U_MIN;
  this->U_MAX = U_MAX;
}



