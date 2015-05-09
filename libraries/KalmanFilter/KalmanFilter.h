//
//  KalmanFilter.h
//  KalmanFilter
//
//  Created by Øyvind Løberg Aakre on 18.02.15.
//  Copyright (c) 2015 Øyvind Løberg Aakre. All rights reserved.
//

#ifndef __KalmanFilter__KalmanFilter__
#define __KalmanFilter__KalmanFilter__

#include "matrix.h"

class KalmanFilter {
    static const int N = 6;     // Number of states
    static const int NW = 4;    // Number of independent noise sources
    static const int NY = 4;    // Number of measurements
    static const float alpha = 0.2;  // For low pass filter
    static const float var_pros = 0.02;// Process variance
    static const float var_bias = 0.01;// Bias variance
    matrix PHI;                 // Transition matrix
    matrix H;                   // Mesurement matrix
    matrix H_t;
    matrix GQG;                 // Constant product GAMMA*Q*GAMMA^T in error covariance update step
    matrix P;                   // Error covariance matrix
    matrix R;                   // Measurement covariance matrix
    matrix K;                   // Gain matrix
    float acc_lp[3];            // Low pass filter for measured roll and pitch angles
    float calc_roll(float *acc);
    float calc_pitch(float *acc);
public:
    KalmanFilter(int Tsamp);
    matrix x;                   // State vector
    void Test();
    void lp_filter(float acc[]);
    void update_K();
    void update_x(matrix y);
    void update_P();
    void update(float acc[], float gyro[]);
};
#endif /* defined(__KalmanFilter__KalmanFilter__) */
