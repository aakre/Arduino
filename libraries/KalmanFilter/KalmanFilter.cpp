//
//  KalmanFilter.cpp
//  KalmanFilter
//
//  Created by Øyvind Løberg Aakre on 18.02.15.
//  Copyright (c) 2015 Øyvind Løberg Aakre. All rights reserved.
//

#include "KalmanFilter.h"
#include <math.h>


KalmanFilter::KalmanFilter(int T){
    //T: sampling time in ms
    acc_lp[0]=0; acc_lp[1]=0; acc_lp[2]=-9.81;
    
    // Initialize transition matrix PHI
    float dt = (float)T/1000;
    float A[N][N] = {
        {0,0,dt,0,0,0},
        {0,0,0,dt,0,0},
        {0,0,0,0,-dt,0},
        {0,0,0,0,0,-dt},
        {0,0,0,0,0,0},
        {0,0,0,0,0,0}
    };
    matrix A_mat((float*)A,N,N);
    matrix I(N,N);
    I.Eye();
    PHI = I+A_mat;
    //PHI.Print("PHI");
    
    
    // Create the constant product GAMMA*Q*GAMMA^T
    float Gamma[N][NW] = {
        {0,0,0,0},
        {0,0,0,0},
        {dt,0,0,0},
        {0,dt,0,0},
        {0,0,dt,0},
        {0,0,0,dt}
    };
    matrix GAMMA((float*)Gamma, N,NW);
    
    // Mulitply by 1000 to avoid small numbers
    // Math trick: 1000*dt = 1000*T/1000 = T
    float *d = new float[NW];
    d[0]= var_pros*T;
    d[1]= var_bias*T;
    d[2]= var_pros*T;
    d[3]= var_bias*T;
    matrix Q(NW,NW);
    Q.Diag(d);
    
    GQG = GAMMA*Q*GAMMA.Transpose();
    //GQG.Print("GQG");
    
    // Create measurement noise covariance matrix R
    delete(d);
    d = new float[NY];
    d[0] = 0.00001;
    d[1] = 0.00001;
    d[2] = 0.0025;
    d[3] = 0.0025;
    R = matrix(NY,NY);
    R.Diag(d);
    delete(d);
    //matrix.Print((float*)R, NY, NY, "R");
    
    //Create error covariance matrix P
    P = matrix(N,N);
    P.Eye();
    //P.Print("P");
    /* Scale P? */
    
    //Create measurement matrix H and its transpose
    H = matrix(NY,N);
    d = new float[NY];
    for (int i=0; i<NY; i++) {d[i] = 1;}
    H.Diag(d);
    //H.Print("H");
    delete(d);
    H_t = H.Transpose();
    
    x = matrix(N,1);
    K = matrix(N,NY);
}

void KalmanFilter::Test() {
    update_K();
    K.Print("K");
     float y[] = {1,2,3,4,5};
     matrix Y((float*)y, 4, 1);
     update_x(Y);
     update_P();
     x.Print("x");
     P.Print("P");
}

void KalmanFilter::update_K() {
  K = (P*H_t) / (H*P*H_t + R);
  //K.Print("K");
}

void KalmanFilter::update_x(matrix y) {
    x += K*(y-H*x);
    x = PHI*x;
    //x.Print("x");
}

void KalmanFilter::update_P() {
    matrix I(N,N);
    I.Eye();
    matrix tmp = (I-K*H);
    P = tmp*P*(tmp.Transpose());
    P += K*R*(K.Transpose());
    
    P = PHI*P*(PHI.Transpose());
    P += GQG;
    //P.Print("P");
}

void KalmanFilter::lp_filter(float *acc){
    float tmp = 1-alpha;
    // Account for the orientation of the IMU
    // which is rotated 90 deg about the NED y-axis
    acc_lp[0] = tmp*acc_lp[0]+alpha*acc[2];
    acc_lp[1] = tmp*acc_lp[1]+alpha*acc[1];
    acc_lp[2] = tmp*acc_lp[2]-alpha*acc[0];
}


void KalmanFilter::update(float acc[], float gyro[]) {
    lp_filter(acc);
    for (int i=0; i<3; i++) {
        // 250/2^15 * pi/180
        gyro[i] = gyro[i]*(float)4.3633/32768;
    }
    float roll = calc_roll(acc_lp);
    float pitch = calc_pitch(acc_lp); 
    
    float y[NY] = {roll, pitch, gyro[0], gyro[1]};
    matrix Y((float*)y, NY,1);
    update_K();
    update_x(Y);
    update_P();
}

float KalmanFilter::calc_roll(float *acc) {
  return (float)atan(acc[1]/acc[2]);
}

float KalmanFilter::calc_pitch(float *acc) {
  return (float)atan(acc[0]/(sqrt(acc[1]*acc[1]+acc[2]*acc[2])));
}







