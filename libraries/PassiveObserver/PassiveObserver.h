/* Øyvind Løberg Aakre 17th April 2015
 *
 * Reference
 * Fossen, T.I. "Handbook of Marine Craft Hydrodynamics and Motion Control", Ch. 11
 */

#ifndef PassiveObserver_h
#define PassiveObserver_h

#include <Arduino.h>
#include "../MatrixMath/MatrixMath.h"
#include "../Quaternion/Quaternion.h"

class PassiveObserver {
  Quaternion q;     // Measured quaternion
  Quaternion qerr;  // Quaternion error
  Quaternion qbias; // Quaternion bias due to angle offset
  Quaternion qtemp; // Quaternion for temporary calculations
  float bias[3];    // Bias estimate
  float omega[3];   // Angular velocity with bias compensation
  float Ki;         // Integral gain for bias estimate
  float Kp;         // Proportional gain for observer correction
  float Ts;         // Sampling time in seconds
  void processInput(float *acc, float *gyro, float *mag);
  float calc_roll(float *acc);
  float calc_pitch(float *acc);
  float calc_yaw(float roll, float pitch, float *mag);
public:
  PassiveObserver(float Ts);
  Quaternion qhat;  // Quaternion estimate
  void update(float *acc, float *gyro, float *mag);
  void getRPY(float *RPY);
  void getImag(float *qimag);
  void getGyro(float *gyro);
  void printRPY();
  void printOmega();
  void Test();
  void setBias();
};

#endif
