//
//  KalmanFilter.h
//  KalmanFilter
//
//  Created by Øyvind Løberg Aakre on 18.02.15.
//  Copyright (c) 2015 Øyvind Løberg Aakre. All rights reserved.
//

#ifndef __KalmanFilter2__KalmanFilter2__
#define __KalmanFilter2__KalmanFilter2__

#include "MatrixMath.h"

class KalmanFilter2 {
    static const int N = 6;     // Number of states
    static const int NW = 3;    // Number of independent noise sources
    static const int NY = 3;    // Number of measurements
    static const int NU = 3;    // Number of inputs
    static const float alpha  = 0.1; // For low pass filter
    static const float var_y = 0.001;
    static const float var_bias = 50;
    float x[N];         // State vector with RPY angles and bias estimates
    float omega[NU];    // Angular velocity vector
    float PHI[N][N];    // Transition matrix
    float DELTA[N][NU]; // Input matrix
    float H[NY][N];     // Mesurement matrix
    float H_t[N][NY];
    float GQG[N][N];    // Constant product GAMMA*Q*GAMMA^T in error covariance update step
    float P[N][N];      // Error covariance matrix
    float R[NY][NY];    // Measurement covariance matrix
    float K[N][NY];     // Gain matrix
    float lp_x[3];      // Low pass filter states
public:
    KalmanFilter2(unsigned int Tsamp);
    void Test();
    void update_K();
    void lp_filter(float *u);
    void update_x(float *y, float *u);
    void update_P();
    void update(float *acc, float *gyro, float *mag);
    void PrintX();
    void PrintP();
    void PrintK();
    void ProcessInput(float *acc, float *gyro, float *mag);
    float calc_roll(float *acc);
    float calc_pitch(float *acc);
    float calc_yaw(float roll, float pitch, float *mag);
    void getStates(float *out);
};
#endif /* defined(__KalmanFilter2__KalmanFilter2__) */
