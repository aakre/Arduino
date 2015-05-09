//
//  KalmanFilter2.cpp
//  KalmanFilter2
//
//  Created by Øyvind Løberg Aakre on 18.02.15.
//  Copyright (c) 2015 Øyvind Løberg Aakre. All rights reserved.
//

#include "KalmanFilter2.h"
#include <math.h>

const float acc_bias[3] = {22, 1054, -710};
const float mag_hard[3] = {24.4,-116.5, 34};//{27, -118, 36};
const float mag_soft[3][3] = {
  {0.8192, -0.0186, 0.0291},
  {-0.0186, 0.8140, 0.0319},
  {0.0291, 0.0319, 0.9908}
};

KalmanFilter2::KalmanFilter2(unsigned int T){
    //T: sampling time in ms
    lp_x[0] = 0;
    lp_x[1] = 0;
    lp_x[2] = 0;
    // Initialize transition matrix PHI
    float dt = (float)T/1000;
    float Tb = 0.01;
    float A[N][N] = {
        {0,0,0,dt,0,0},     // | roll       |
        {0,0,0,0,dt,0},     // | pitch      |
        {0,0,0,0,0,dt},     // | yaw        |
        {0,0,0,-Tb,0,0},    // | bias roll  |
        {0,0,0,0,-Tb,0},    // | bias pitch |
        {0,0,0,0,0,-Tb}     // | bias yaw   |
    };

    float B[N][NU] = {
      {1,0,0},
      {0,1,0},
      {0,0,1},
      {0,0,0},
      {0,0,0},
      {0,0,0}
    };

    float E[N][NW] = {
      {0,0,0},
      {0,0,0},
      {0,0,0},
      {1,0,0},
      {0,1,0},
      {0,0,1}
    };
    
    float I[N][N];
    matrix.Eye((float*)I, N);
    matrix.Add((float*)A, (float*)I, N, N, (float*)PHI);
    
    
    // Create the constant product GAMMA*Q*GAMMA^T
    float Idt[N][N];
    float GAMMA[N][NW];
    matrix.Eye((float*)Idt, N);
    matrix.Scale((float*)Idt,N,N,dt);

    matrix.Scale((float*)A,N,N,dt/2); // Not dt*dt/2 since A is already multiplied with dt...
    matrix.Add((float*)Idt, (float*)A, N, N, (float*)Idt);
    matrix.Multiply((float*)Idt, (float*)B, N, N, NU, (float*)DELTA);
    matrix.Multiply((float*)Idt, (float*)E, N, N, NW, (float*)GAMMA);

    //matrix.Print((float*)DELTA, N,NU, "DELTA");
    //matrix.Print((float*)GAMMA, N,NW, "GAMMA");

   
    float Q[NW][NW];
    // Mulitply by 1000 to avoid small numbers
    // Math trick: 1000*dt = 1000*T/1000 = T
    matrix.Eye((float*)Q, NW);
    matrix.Scale((float*)Q, NW, NW, var_bias*T);
    
    //matrix.Print((float*) Q, NW,NW,"Q");
    
    float tmp[N][NW];
    matrix.Multiply((float*)GAMMA, (float*) Q, N, NW, NW, (float*)tmp);
    float GAMMA_T[NW][N];
    matrix.Transpose((float*)GAMMA, N, NW, (float*)GAMMA_T);
    matrix.Multiply((float*)tmp, (float*)GAMMA_T, N, NW, N, (float*)GQG);
    //matrix.Print((float*) GQG, N,N, "GQG");
    
    // Create measurement noise covariance matrix R
    matrix.Eye((float*)R, NY);
    matrix.Scale((float*)R, NY, NY, var_y); // Note: No extra mult with dt or 1000...
    //matrix.Print((float*)R, NY, NY, "R");
    
    //Create error covariance matrix P
    matrix.Eye((float*)P, N);
    matrix.Scale((float*)P, N, N, 1);
    
    //Create measurement matrix H and its transpose
    matrix.Zeros((float*)H, NY, N);
    for (int i=0; i<NY; i++) {
        H[i][i] = 1;
    }
    matrix.Transpose((float*)H, NY, N, (float*)H_t);
    //matrix.Print((float*) H_t, N,NY,"H_t");
    
    //Initialize state vector
    for (int i=0; i<N; i++) {
        x[i] = 0;
    }
    //matrix.Print((float*) x, N,1,"x");
    
    matrix.Zeros((float*)K, N, NY);

}

void KalmanFilter2::Test() {
    update_K();
    float y[] = {1,2,3};
    float u[] = {0,0,0};
    update_x(y,u);
    update_P();
}

void KalmanFilter2::update_K() {
    // Implements K = P*H' / (H*P*H' + R)
    float tmp1[N][NY];
    float tmp2[NY][NY];
    matrix.Multiply((float*)P, (float*)H_t, N, N, NY, (float*)tmp1);
    matrix.Multiply((float*)H, (float*)tmp1, NY, N, NY, (float*)tmp2);
    matrix.Add((float*)tmp2, (float*)R, NY, NY, (float*)tmp2);
    matrix.Invert((float*) tmp2, NY);
    matrix.Multiply((float*) tmp1, (float*)tmp2, N,NY,NY, (float*) K);
    //matrix.Print((float*)K,N,NY, "K");
}

void KalmanFilter2::update_x(float y[], float u[]) {
    /* Implements 
       - Corretion:  x[k]   = x[k]_ + K*(y-H*x[k]_)
       - Prediction: x[k+1] = PHI*x[k] + DELTA*u[k]
    */
    float tmp1[NY];
    float tmp2[N];
    
    //Correction
    matrix.Multiply((float*)H, (float*)x, NY, N, 1, tmp1);
    matrix.Subtract((float*)y, (float*)tmp1, NY, 1, tmp1);
    matrix.Multiply((float*)K, (float*)tmp1, N, NY, 1, (float*)tmp2);
    matrix.Add((float*)x, (float*)tmp2, N, 1, (float*)tmp2);
    
    //Prediction
    matrix.Multiply((float*)PHI, (float*)tmp2, N, N, 1, (float*)x);
    matrix.Multiply((float*)DELTA, (float*)u, N, NU, 1, (float*)tmp2);
    matrix.Add((float*)x, (float*)tmp2, N ,1, (float*)x);
    
    //matrix.Print((float*)x, N, 1, "x");
}

void KalmanFilter2::update_P() {
    // Implements P = (I-KH)*P*(I-KH)' + K*R*K'
    float I[N][N];
    float tmp1[N][N];
    float tmp2[N][N];
    float tmp3[N][NY];
    float tmp4[NY][N];
    matrix.Eye((float*)I, N);
    matrix.Multiply((float*)K, (float*)H, N, NY, N, (float*)tmp1);
    matrix.Subtract((float*)I, (float*)tmp1, N, N, (float*)I); // I-KH
    matrix.Multiply((float*)I, (float*)P, N, N, N, (float*)tmp1); //(I-KH)*P
    matrix.Transpose((float*)I, N, N, (float*)tmp2); // (I-KH)'
    matrix.Multiply((float*)tmp1, (float*)tmp2, N, N, N, (float*)P); //P = (I-KH)*P*(I-KH)'
    
    matrix.Multiply((float*)K, (float*)R, N, NY, NY, (float*)tmp3);
    matrix.Transpose((float*)K, N, NY, (float*)tmp4);
    matrix.Multiply((float*)tmp3, (float*)tmp4, N, NY, N, (float*)tmp2);
    matrix.Add((float*)P, (float*)tmp2, N, N, (float*)P);
    //matrix.Print((float*)P,N,N, "P");
    
    matrix.Transpose((float*)PHI, N, N, (float*)tmp1);
    matrix.Multiply((float*)PHI, (float*)P, N, N, N, (float*)tmp2);
    matrix.Multiply((float*)tmp2, (float*)tmp1, N, N, N, (float*)P);
    matrix.Add((float*)P, (float*)GQG, N,N, (float*)P);
    //matrix.Print((float*)P,N,N, "P");
}

void KalmanFilter2::lp_filter(float *u){
    float tmp = 1-alpha;
    lp_x[0] = tmp*lp_x[0]+alpha*u[0];
    lp_x[1] = tmp*lp_x[1]+alpha*u[1];
    lp_x[2] = tmp*lp_x[2]+alpha*u[2];

    /* User model
       x: signal with noise
       lp_filter(x): filter the signal
       x: filtered signal
    */ 
    u[0] = lp_x[0];
    u[1] = lp_x[1];
    u[2] = lp_x[2];
}


void KalmanFilter2::update(float *acc, float *gyro, float *mag) {
    ProcessInput(acc,gyro,mag);
    float roll = calc_roll(acc);
    float pitch = calc_pitch(acc);
    float y[NY] = {roll, pitch, calc_yaw(roll, pitch, mag)};
    lp_filter(gyro);
    update_K();
    update_x(y, gyro);
    update_P();

    /* Store angular velocities in the vector omega
       and compensate with bias estimates
    */
    for (int i=0; i<3; i++) {
      omega[i] = gyro[i]+x[i+3];
    }
}

void KalmanFilter2::ProcessInput(float *acc, float *gyro, float *mag) {
  //lp_filter(acc); No need when the MPU employs a low pass filter

  /* Rotate input
     The acc/gyro sensor uses a right-handed frame with z pointing up.
     Furthermore, it is aligned in the IMU with the y-axis pointing forward,
     i.e., along NED-x.

     The magnetometer also uses a right-handed frame with z pointing up, but
     the alignment is different where -x is pointing forward along NED-x.
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


void KalmanFilter2::PrintX() {
  int i=0;
  Serial.println();
  Serial.print("\nRoll (deg): ");
  Serial.print(x[i++]*57);
  Serial.print("\nPitch (deg): ");
  Serial.print(x[i++]*57);
  Serial.print("\nYaw (deg): ");
  Serial.print(x[i++]*57);

  i=0;
  Serial.print("\np (deg/s): ");
  Serial.print(omega[i++]*57);
  Serial.print("\nq (deg/s): ");
  Serial.print(omega[i++]*57);
  Serial.print("\nr (deg/s): ");
  Serial.print(omega[i++]*57);
}

void KalmanFilter2::getStates(float *out) {
  for (int i=0; i<3; i++) {
    out[i] = x[i];
    out[i+3] = omega[i];
  }
}

void KalmanFilter2::PrintP() {
  matrix.Print((float*)P, N, N, "P");
}

void KalmanFilter2::PrintK() {
  matrix.Print((float*)K, N, NY, "K");
}

float KalmanFilter2::calc_roll(float *acc) {
  return atan(acc[1]/acc[2]);
}

float KalmanFilter2::calc_pitch(float *acc) {
  return atan(acc[0]/(sqrt(acc[1]*acc[1]+acc[2]*acc[2])));
}

float KalmanFilter2::calc_yaw(float roll, float pitch, float *mag) {
  float yaw = 0;
  float mx = mag[0], my = mag[1], mz = mag[2];
  float hx = mx*cos(pitch)+my*sin(pitch)*sin(roll)+mz*cos(roll)*sin(pitch);
  float hy = my*cos(roll)-mz*sin(roll);
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








