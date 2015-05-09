#include <Arduino.h>
#include <math.h>

#ifndef Quaternion_h
#define Quaternion_h

/* A class for quaternions to be used in attitude estimation
 * quaternion = eta + i*eps1 + j*eps2 + k*eps3
 * where i,j,k are the 3D imaginary units
 *
 * Some code is taken from the MSS GNC Matlab Toolbox and is
 * is protected by Copyright, but can be used or modified in
 * accordance with GNU General Public License.
 * Copyright (C) 2008 Thor I. Fossen and Tristan Perez
 *
 * The published reference being used
 * Fossen, T.I. "Handbook of Marine Craft Hydrodynamics and Motion Control", Ch. 11
 *
 * Øyvind Løberg Aakre 14th April 2015
 */

class Quaternion {
public:
  float eta;
  float eps1;
  float eps2;
  float eps3;
  Quaternion();
  Quaternion(float,float,float,float);
  void norm();
  void prod(Quaternion, Quaternion);
  void prodInv1(Quaternion, Quaternion);
  void Rquat(float *R);
  void Tquat(float *T);
  void toEuler(float *euler);
  void fromEuler(float roll, float pitch, float yaw);
  void printRPY();
  void print();
};
#endif

