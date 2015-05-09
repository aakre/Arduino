#include "Quadcopter.h"

Quadcopter::Quadcopter(int TAU_MIN, int TAU_MAX, int ESC_MIN, int ESC_MAX) {
  this->TAU_MIN = TAU_MIN;
  this->TAU_MAX = TAU_MAX;
  this->ESC_MIN = ESC_MIN;
  this->ESC_MAX = ESC_MAX;
  float L = 0.2125; //Half the distance between the center of two adjacent motors
  float C = 1.0F; // Scaling factor for yaw moment
  float T[4][4] = {
    {L, -L, -L, L},
    {L, L, -L, -L},
    {L, -L, L, -L},
    {C, C, C, C}
  };

  matrix.Invert((float*)T, 4);
  matrix.Copy((float*)T, 4, 4, (float*)T_alloc);


  /* Hardcoded matrix
  T_alloc[0][0] = 1.1765;
  T_alloc[0][1] = 1.1765;
  T_alloc[0][2] = 1.1765;
  T_alloc[0][3] = 0.2500;

  T_alloc[1][0] = -1.1765;
  T_alloc[1][1] = 1.1765;
  T_alloc[1][2] = -1.1765;
  T_alloc[1][3] = 0.2500;

  T_alloc[2][0] = -1.1765;
  T_alloc[2][1] = -1.1765;
  T_alloc[2][2] = 1.1765;
  T_alloc[2][3] = 0.2500;

  T_alloc[3][0] = 1.1765;
  T_alloc[3][1] = -1.1765;
  T_alloc[3][2] = -1.1765;
  T_alloc[3][3] = 0.2500;
  */
  k_range = (float)(ESC_MAX-ESC_MIN)/(TAU_MAX-TAU_MIN);
  for (int i=0; i<4; i++) {
    pwm[i] = ESC_MIN;
  }
}

void Quadcopter::init(int *motorPin) {
  for (int i=0; i<4; i++) {
    motor[i].attach(motorPin[i]);
    motor[i].writeMicroseconds(ESC_MIN);
  }
}

void Quadcopter::input(float *tau, int print) {
  matrix.Multiply((float*)T_alloc, (float*)tau, 4,4,1, (float*)pwm);
  for (int i=0; i<4; i++) {
    // Use a linear relationship between tau and PWM
    pwm[i] = (pwm[i]-TAU_MIN)*k_range + ESC_MIN;
    pwm[i] = max(ESC_MIN, min(ESC_MAX, (int)pwm[i]));
    motor[i].writeMicroseconds((int)pwm[i]);
  }
  if (print) {
    Serial.println();
    Serial.println("PWM input");
    for (int i=0; i<4; i++) {
      Serial.println(pwm[i]);
    }
  }
}
