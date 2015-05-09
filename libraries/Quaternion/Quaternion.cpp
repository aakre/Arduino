#include "Quaternion.h"
#include "../MatrixMath/MatrixMath.h"
Quaternion::Quaternion() {
  eta  = 1.0F;
  eps1 = 0.0F;
  eps2 = 0.0F;
  eps3 = 0.0F;
}

Quaternion::Quaternion(float n, float e1, float e2, float e3) {
  eta  = n;
  eps1 = e1;
  eps2 = e2;
  eps3 = e3;
  norm();
}

void Quaternion::norm() {
  float mag = sqrt(eta*eta+eps1*eps1+eps2*eps2+eps3*eps3);
  eta  /= mag;
  eps1 /= mag;
  eps2 /= mag;
  eps3 /= mag;
}


void Quaternion::prod(Quaternion qhat, Quaternion q) {
  /* This is the quaternion product
   * See the main reference for more information.
   *
   * qtilde = [q1.eta*q2.eta - q1.eps'*q2.eps;
               q2.eta*q1.eps + q1.eta*q2.eps + Smtrx(q1.eps)*q2.eps];
   */
  this->eta  = q.eta*qhat.eta  - q.eps2*qhat.eps2 - q.eps3*qhat.eps3 - q.eps1*qhat.eps1;
  this->eps1 = q.eps1*qhat.eta - q.eps2*qhat.eps3 + q.eps3*qhat.eps2 + q.eta*qhat.eps1;
  this->eps2 = q.eps2*qhat.eta + q.eps1*qhat.eps3 - q.eps3*qhat.eps1 + q.eta*qhat.eps2;
  this->eps3 = q.eps3*qhat.eta - q.eps1*qhat.eps2 + q.eps2*qhat.eps1 + q.eta*qhat.eps3;
}

void Quaternion::prodInv1(Quaternion qhat, Quaternion q) {
  /* This is the quaternion product where the complex
   * conjugate of the first input is used, i.e.
   * qhat* x q
   * See the main reference for more information.
   *
   * qtilde = [q1.eta*q2.eta + q1.eps'*q2.eps;
              -q2.eta*q1.eps + q1.eta*q2.eps - Smtrx(q1.eps)*q2.eps];
  */
  this->eta  = q.eta*qhat.eta  + q.eps1*qhat.eps1 + q.eps2*qhat.eps2 + q.eps3*qhat.eps3;
  this->eps1 = q.eps1*qhat.eta + q.eps2*qhat.eps3 - q.eps3*qhat.eps2 - qhat.eps1*q.eta;
  this->eps2 = q.eps2*qhat.eta - q.eps1*qhat.eps3 + q.eps3*qhat.eps1 - q.eta*qhat.eps2;
  this->eps3 = q.eps3*qhat.eta + q.eps1*qhat.eps2 - q.eps2*qhat.eps1 - q.eta*qhat.eps3;
}

void Quaternion::Rquat(float *R) {
  R[0] = 1-2*(eps2*eps2 + eps3*eps3);
  R[1] =   2*(eps1*eps2 - eps3*eta);
  R[2] =   2*(eps1*eps3 + eps2*eta);

  R[3] =   2*(eps1*eps2 + eps3*eta);
  R[4] = 1-2*(eps1*eps1 + eps3*eps3);
  R[5] =   2*(eps2*eps3 - eps1*eta);

  R[6] =   2*(eps1*eps3 - eps2*eta);
  R[7] =   2*(eps2*eps3 + eps1*eta);
  R[8] = 1-2*(eps1*eps1 + eps2*eps2);
}

void Quaternion::Tquat(float *T) {
  T[0] = -eps1;
  T[1] = -eps2;
  T[2] = -eps3;

  T[3] = eta;
  T[4] = -eps3;
  T[5] = eps2;

  T[6] = eps3;
  T[7] = eta;
  T[8] = -eps1;

  T[9] = -eps2;
  T[10] = eps1;
  T[11] = eta;

  for (int i=0; i<12; i++) {
    T[i] *= 0.5;
  }
}

void Quaternion::toEuler(float *euler) {
  // Copyright (C) 2008 Thor I. Fossen and Tristan Perez
  float R[3][3];
  Rquat(*R);
  euler[0] = atan2(R[2][1], R[2][2]);
  euler[1] = -atan(R[2][0] / (sqrt(1-R[2][0]*R[2][0])));
  euler[2] = atan2(R[1][0], R[0][0]);
}

void Quaternion::fromEuler(float roll, float pitch, float yaw) {
  float cphi = cos(roll);
  float sphi = sin(roll);
  float cth  = cos(pitch);
  float sth  = sin(pitch);
  float cpsi = cos(yaw);
  float spsi = sin(yaw);

  float R[3][3] = {
       {cpsi*cth,  -spsi*cphi+cpsi*sth*sphi,  spsi*sphi+cpsi*cphi*sth},
       {spsi*cth,  cpsi*cphi+sphi*sth*spsi,   -cpsi*sphi+sth*spsi*cphi},
       {-sth,      cth*sphi,                  cth*cphi}
  };
  float R33 = R[0][0] + R[1][1] + R[2][2];

  float Rmax = R33;
  int imax = 3;
  for (int i=0; i<3; i++) {
    if (R[i][i] > Rmax) {
      Rmax = R[i][i];
      imax = i;
    }
  }

  float p_i;
  if (imax==3) {
    p_i = sqrt(1+R33);
  } else {
    p_i = sqrt(1+2*R[imax][imax]-R33);
  }

  float p0;
  float p1;
  float p2;
  float p3;
  switch (imax) {
    case 0:
      p0 = p_i;
      p1 = (R[1][0]+R[0][1])/p_i;
      p2 = (R[0][2]+R[2][0])/p_i;
      p3 = (R[2][1]-R[1][2])/p_i;
      break;
    case 1:
      p0 = (R[1][0]+R[0][1])/p_i;
      p1 = p_i;
      p2 = (R[2][1]+R[1][2])/p_i;
      p2 = (R[0][2]-R[2][0])/p_i;
      break;
    case 2:
      p0 = (R[0][2]+R[2][0])/p_i;
      p1 = (R[2][1]+R[1][2])/p_i;
      p2 = p_i;
      p3 = (R[1][0]-R[0][1])/p_i;
      break;
    case 3:
      p0 = (R[2][1]-R[1][2])/p_i;
      p1 = (R[0][2]-R[2][0])/p_i;
      p2 = (R[1][0]-R[0][1])/p_i;
      p3 = p_i;
      break;
  }

  eta  = 0.5*p3;
  eps1 = 0.5*p0;
  eps2 = 0.5*p1;
  eps3 = 0.5*p2;

  norm();
}

void Quaternion::printRPY() {
  float RPY[3];
  toEuler(RPY);
  Serial.println();
  Serial.print("\nRoll  (deg) ");
  Serial.print(RPY[0]*57.3);
  Serial.print("\nPitch (deg) ");
  Serial.print(RPY[1]*57.3);
  Serial.print("\nYaw   (deg) ");
  Serial.print(RPY[2]*57.3);
}

void Quaternion::print() {
  Serial.println();
  Serial.println("Eta eps1 eps2 eps3");
  Serial.println(eta);
  Serial.println(eps1);
  Serial.println(eps2);
  Serial.println(eps3);
}

