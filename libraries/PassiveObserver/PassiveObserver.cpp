#include "PassiveObserver.h"

const float acc_bias[3] = {22, 1054, -710};
const float mag_hard[3] = {24.4,-116.5, 34};//{27, -118, 36};
const float mag_soft[3][3] = {
  {0.8192, -0.0186, 0.0291},
  {-0.0186, 0.8140, 0.0319},
  {0.0291, 0.0319, 0.9908}
};

float sign(float x) {
  return (x>0) - (x<0);
}

PassiveObserver::PassiveObserver(float Ts) {
  this->Ts = Ts;
  Ki = 1;
  Kp = 50;

  for (int i=0; i<3; i++) {
    bias[i] = 0;
  }
}

void PassiveObserver::update(float *acc, float *gyro, float *mag) {
  processInput(acc, gyro, mag);
  float roll  = calc_roll(acc);
  float pitch = calc_pitch(acc);
  float yaw   = calc_yaw(roll, pitch, mag);

  qtemp.fromEuler(roll, pitch, yaw);
  q.prod(qbias, qtemp); // Subtract angle offset due to misalignment
  qerr.prodInv1(qhat, q);

  float sgn = sign(qerr.eta);
  float filterCorr[3] = {sgn*qerr.eps1, sgn*qerr.eps2, sgn*qerr.eps3};
  float tmp = -0.5*Ts*Ki;
  bias[0] += tmp*filterCorr[0];
  bias[1] += tmp*filterCorr[1];
  bias[2] += tmp*filterCorr[2];

  omega[0] = gyro[0]-bias[0];
  omega[1] = gyro[1]-bias[1];
  omega[2] = gyro[2]-bias[2];

  float qhat_dot_[3];
  qhat_dot_[0] = omega[0] + Kp*filterCorr[0];
  qhat_dot_[1] = omega[1] + Kp*filterCorr[1];
  qhat_dot_[2] = omega[2] + Kp*filterCorr[2];

  float Rqhat_T[3][3];
  float Tqhat[4][3];
  float Rq[3][3];
  float Rtemp[3][3];
  float tmp2[3];
  float qhat_dot[4];
  qhat.Rquat((float*)Rtemp);
  qhat.Tquat((float*)Tqhat);
  q.Rquat((float*)Rq);
  matrix.Transpose((float*)Rtemp, 3, 3, (float*)Rqhat_T);
  matrix.Multiply((float*)Rqhat_T, (float*)Rq, 3, 3, 3, (float*)Rtemp);
  matrix.Multiply((float*)Rtemp, (float*)qhat_dot_, 3, 3, 1, (float*)tmp2);
  matrix.Multiply((float*)Tqhat, (float*)tmp2, 4, 3, 1, (float*)qhat_dot);

  qhat.eta  += Ts*qhat_dot[0];
  qhat.eps1 += Ts*qhat_dot[1];
  qhat.eps2 += Ts*qhat_dot[2];
  qhat.eps3 += Ts*qhat_dot[3];
  qhat.norm();
}

void PassiveObserver::processInput(float *acc, float *gyro, float *mag) {
  /* Rotate input
     The acc/gyro sensor uses a right-handed frame with z pointing up.
     Furthermore, it is aligned in the IMU with the y-axis pointing forward,
     i.e., along North-axis (NED)

     The magnetometer also uses a right-handed frame with z pointing up, but
     the alignment is different where -x is pointing forward along North (NED).
  */
  float temp = acc[0];
  acc[0] = acc[1]  - acc_bias[0];
  acc[1] = -acc[2] - acc_bias[1];
  acc[2] = -temp   - acc_bias[2];

  temp = gyro[0];
  gyro[0] = gyro[1];
  gyro[1] = -gyro[2];
  gyro[2] = -temp;

  mag[0] = -mag[0];
  mag[2] = -mag[2];

  /* Scale input */
  for (int i=0; i<3; i++) {
    // 250/2^15 * pi/180
    gyro[i] = gyro[i]*(float)4.3633/32768;
  }

  // Compensate for hard and soft errors in the mag. readings
  float mag_temp[] = {mag[0], mag[1], mag[2]};
  matrix.Subtract((float*)mag, (float*)mag_hard, 3, 1, (float*)mag_temp);
  matrix.Multiply((float*)mag_soft, (float*)mag_temp, 3,3,1,(float*)mag);
}

void PassiveObserver::getRPY(float *RPY) {
  qhat.toEuler(RPY);
}

void PassiveObserver::printRPY() {
  qhat.printRPY();
}

void PassiveObserver::printOmega() {
  Serial.println();
  Serial.print("\np (rad/s) ");
  Serial.print(omega[0]);
  Serial.print("\nq (rad/S) ");
  Serial.print(omega[1]);
  Serial.print("\nr (rad/s) ");
  Serial.print(omega[2]);
}


float PassiveObserver::calc_roll(float *acc) {
  return atan(acc[1]/acc[2]);
}

float PassiveObserver::calc_pitch(float *acc) {
  return atan(acc[0]/(sqrt(acc[1]*acc[1]+acc[2]*acc[2])));
}

float PassiveObserver::calc_yaw(float roll, float pitch, float *mag) {
  float yaw = 0;
  float mx = mag[0], my = mag[1], mz = mag[2];
  float croll = cos(roll);
  float sroll = sin(roll);
  float cpitch = cos(pitch);
  float spitch = sin(pitch);
  float hx = mx*cpitch+my*spitch*sroll+mz*croll*spitch;
  float hy = my*croll-mz*sroll;
  float ratio = hy/hx;
  if (hx<0) {
    yaw = PI-atan(ratio);
  } else if (hx>0 && hy<0) {
    yaw = -atan(ratio);
  } else if (hx>0 && hy>0) {
    yaw = 2*PI-atan(ratio);
  } else if (hx==0 && hy<0) {
    yaw = PI/2;
  } else if (hx==0 && hy>0) {
    yaw = 1.5*PI;
  }
  return yaw;
}

void PassiveObserver::getGyro(float *gyro) {
  gyro[0] = omega[0];
  gyro[1] = omega[1];
  gyro[2] = omega[2];
}

void PassiveObserver::Test() {
  float roll = PI/4;  //45;
  float pitch = PI/3; //60
  float yaw = 1.5*PI; //270
  float gyro[3] = {1,5,10};

  // Run the observer N times
  for (int i=0; i<10; i++) {
  q.fromEuler(roll, pitch, yaw);
  qerr.prodInv1(qhat, q);
  float sgn = sign(qerr.eta);
  float filterCorr[3] = {sgn*qerr.eps1, sgn*qerr.eps2, sgn*qerr.eps3};
  float tmp = -0.5*Ts*Ki;
  bias[0] += tmp*filterCorr[0];
  bias[1] += tmp*filterCorr[1];
  bias[2] += tmp*filterCorr[2];

  float qhat_dot_[3];
  qhat_dot_[0] = gyro[0]-bias[0]+Kp*filterCorr[0];
  qhat_dot_[1] = gyro[1]-bias[1]+Kp*filterCorr[1];
  qhat_dot_[2] = gyro[2]-bias[2]+Kp*filterCorr[2];

  float Rqhat_T[3][3];
  float Tqhat[4][3];
  float Rq[3][3];
  float Rtemp[3][3];
  float tmp2[3];
  float qhat_dot[4];
  qhat.Rquat((float*)Rtemp);
  qhat.Tquat((float*)Tqhat);
  q.Rquat((float*)Rq);
  matrix.Transpose((float*)Rtemp, 3, 3, (float*)Rqhat_T);
  matrix.Multiply((float*)Rqhat_T, (float*)Rq, 3, 3, 3, (float*)Rtemp);
  matrix.Multiply((float*)Rtemp, (float*)qhat_dot_, 3, 3, 1, (float*)tmp2);
  matrix.Multiply((float*)Tqhat, (float*)tmp2, 4, 3, 1, (float*)qhat_dot);

  qhat.eta  += Ts*qhat_dot[0];
  qhat.eps1 += Ts*qhat_dot[1];
  qhat.eps2 += Ts*qhat_dot[2];
  qhat.eps3 += Ts*qhat_dot[3];
  qhat.norm();
}
  Serial.println();
  Serial.println("qhat = [eta; eps1; eps2; eps3]");
  Serial.println(qhat.eta);
  Serial.println(qhat.eps1);
  Serial.println(qhat.eps2);
  Serial.println(qhat.eps3);

  float euler[3];
  qhat.toEuler((float*)euler);
  qhat.printRPY();
}

void PassiveObserver::getImag(float *qimag) {
  qimag[0] = qhat.eps1;
  qimag[1] = qhat.eps2;
  qimag[2] = qhat.eps3;
}

void PassiveObserver::setBias() {
  /* Compensate for the misaligned of the IMU
   * Run the filter N times, then call this function
   * The idea is to compensate for the measured roll and pitch angle
   * such that they read ~zero when kept at rest in this position.
   * In other words, roll and pitch are reset, but yaw is unaltered.
   * This is done by multiplying the measured quaternion with the
   * bias quaternion qbias := q_yaw x q_rpy*
   * where q_yaw preserves the yaw angle and q_rpy is the conjugate of the
   * current quaternion estimate of the RPY angles.
   */
  Quaternion q_rpy;
  Quaternion q_yaw;
  q_rpy.eta  =  qhat.eta;
  q_rpy.eps1 = -qhat.eps1;
  q_rpy.eps2 = -qhat.eps2;
  q_rpy.eps3 = -qhat.eps3;

  float rpy[3];
  qhat.toEuler((float*)rpy);
  q_yaw.fromEuler(0.0, 0.0, rpy[2]);
  qbias.prod(q_yaw, q_rpy);
}

