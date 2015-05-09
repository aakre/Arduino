#include "PID.h"

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

float PID::update(float error, float plant, int print) {
  float P=0;
  float I=0;
  float D=0;
  float u;
  P = k_p*error;

  x_i += error;
  x_i = max(I_MIN, min(I_MAX, x_i)); // Saturation
  I = k_i*x_i;

  D = k_d*(x_d - plant);
  x_d = plant;

  u = P+I+D;
  if (print) {
    Serial.println();
    Serial.println(P);
    Serial.println(I);
    Serial.println(D);
    Serial.println("--");
    Serial.println(u);
    Serial.println(max(U_MIN, min(U_MAX,u)));
  }

  /* Saturate */
  return max(U_MIN, min(U_MAX,u));
}

void PID::reset_int() {
  x_i = 0;
}

void PID::set_kp(float k_p) {
  this->k_p = k_p;
}

void PID::set_ki(float k_i) {
  this->k_i = k_i;
}

void PID::set_kd(float k_d) {
  this->k_d = k_d;
}
